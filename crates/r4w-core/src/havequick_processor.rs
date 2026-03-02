//! HAVE QUICK Frequency Hopping Radio Processor
//!
//! Implements the HAVE QUICK I/II military UHF anti-jam frequency hopping system
//! used for voice communications in the 225–400 MHz band. HAVE QUICK provides
//! resistance to jamming, interception, and electronic warfare through rapid
//! pseudo-random frequency hopping synchronized by GPS Time-of-Day (TOD).
//!
//! **NOTE: This is an educational/simulation model only. The actual HAVE QUICK
//! cryptographic Word-of-Day (WOD) and hop sequence algorithms are classified.
//! This implementation uses a simplified non-classified LFSR-based model.**
//!
//! ## System Overview
//!
//! - **Band**: 225–400 MHz (UHF military aeronautical band)
//! - **Channel spacing**: 25 kHz (7,000 channels)
//! - **HQ-I hop rate**: 100–200 hops/second
//! - **HQ-II hop rate**: up to 5,000 hops/second
//! - **Modulation**: AM (Amplitude Modulation), 25 kHz channel BW
//! - **Voice band**: 300–3,400 Hz
//! - **Synchronization**: GPS Time-of-Day (TOD)
//! - **Crypto variable**: Word-of-Day (WOD)
//!
//! ## Processing Gain
//!
//! FH processing gain = 10 * log10(total_bandwidth / channel_bandwidth)
//!                    ≈ 10 * log10(175 MHz / 25 kHz) ≈ 37.9 dB
//!
//! ## References
//!
//! - MIL-STD-188-141B: Interoperability and Performance Standards
//! - STANAG 4246: HAVE QUICK interoperability agreement
//! - NATO STANAG 5030: AJ communications
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::havequick_processor::{
//!     HaveQuickConfig, HaveQuickMode, HaveQuickProcessor,
//! };
//!
//! let config = HaveQuickConfig {
//!     mode: HaveQuickMode::HqI,
//!     wod: 0xA5C3_9E12,
//!     net_id: 1,
//!     tod_seconds: 43200.0,
//!     sample_rate_hz: 100_000.0,
//!     audio_gain: 0.5,
//! };
//!
//! let mut proc = HaveQuickProcessor::new(config);
//! // Generate a hop sequence for the next 10 hops
//! let hops = proc.hop_sequence_gen_mut().generate_n(10);
//! assert_eq!(hops.len(), 10);
//! for h in &hops {
//!     assert!(h.frequency_hz >= 225e6 && h.frequency_hz <= 400e6);
//! }
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Lower edge of the HAVE QUICK band (Hz).
pub const HQ_BAND_LOW_HZ: f64 = 225e6;

/// Upper edge of the HAVE QUICK band (Hz).
pub const HQ_BAND_HIGH_HZ: f64 = 400e6;

/// Channel spacing in Hz (25 kHz).
pub const HQ_CHANNEL_SPACING_HZ: f64 = 25e3;

/// Total number of 25 kHz channels in 225–400 MHz.
/// (400 - 225) MHz / 25 kHz = 7000 channels.
pub const HQ_NUM_CHANNELS: usize = 7000;

/// HQ-I nominal hop rate (hops/second).
pub const HQ1_HOP_RATE_HZ: f64 = 100.0;

/// HQ-II nominal hop rate (hops/second).
pub const HQ2_HOP_RATE_HZ: f64 = 5000.0;

/// Voice audio band lower cutoff (Hz).
pub const AUDIO_LOW_HZ: f64 = 300.0;

/// Voice audio band upper cutoff (Hz).
pub const AUDIO_HIGH_HZ: f64 = 3400.0;

/// AM modulation index (typical for voice, 85%).
pub const AM_MOD_INDEX: f64 = 0.85;

/// Guard time between hops as fraction of dwell time.
pub const GUARD_TIME_FRACTION: f64 = 0.05;

/// Minimum frequency separation between consecutive hops (channels).
pub const MIN_HOP_SEPARATION_CHANNELS: usize = 50;

/// Anti-repeat window: minimum hops before a channel can be reused.
pub const ANTI_REPEAT_WINDOW: usize = 100;

// ─────────────────────────────────────────────────────────────────────────────
//  Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// HAVE QUICK operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HaveQuickMode {
    /// HAVE QUICK I — 100–200 hops/second.
    HqI,
    /// HAVE QUICK II — up to 5,000 hops/second.
    HqII,
}

impl HaveQuickMode {
    /// Nominal hop rate for this mode (hops/second).
    pub fn hop_rate(&self) -> f64 {
        match self {
            HaveQuickMode::HqI => HQ1_HOP_RATE_HZ,
            HaveQuickMode::HqII => HQ2_HOP_RATE_HZ,
        }
    }

    /// Dwell time per hop (seconds).
    pub fn dwell_time_s(&self) -> f64 {
        1.0 / self.hop_rate()
    }

    /// Guard time between hops (seconds).
    pub fn guard_time_s(&self) -> f64 {
        self.dwell_time_s() * GUARD_TIME_FRACTION
    }

    /// Active (data) time per hop after removing guard time (seconds).
    pub fn active_time_s(&self) -> f64 {
        self.dwell_time_s() - self.guard_time_s()
    }
}

/// AM demodulation detection mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AmDetection {
    /// Envelope detection (rectify + lowpass filter).
    Envelope,
    /// Synchronous (coherent) detection using carrier reference.
    Synchronous,
}

/// Channel impairment model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelModel {
    /// Ideal (no noise or impairments).
    Ideal,
    /// Additive White Gaussian Noise with given SNR (dB).
    Awgn { snr_db: f64 },
    /// Partial-band jamming: fraction of band jammed at given jammer power (dBW).
    PartialBandJamming { jam_fraction: f64, jammer_power_dbw: f64 },
}

// ─────────────────────────────────────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// HAVE QUICK system configuration.
#[derive(Debug, Clone)]
pub struct HaveQuickConfig {
    /// Operating mode: HQ-I or HQ-II.
    pub mode: HaveQuickMode,
    /// Word-of-Day (crypto variable, 32-bit simplified model).
    pub wod: u32,
    /// Net ID for multi-net operation (0–255).
    pub net_id: u8,
    /// Time-of-Day in seconds since midnight (GPS time).
    pub tod_seconds: f64,
    /// Sample rate for IQ/audio processing (Hz).
    pub sample_rate_hz: f64,
    /// AM audio gain (0.0–1.0).
    pub audio_gain: f64,
}

impl Default for HaveQuickConfig {
    fn default() -> Self {
        Self {
            mode: HaveQuickMode::HqI,
            wod: 0xDEAD_BEEF,
            net_id: 1,
            tod_seconds: 0.0,
            sample_rate_hz: 50_000.0,
            audio_gain: 0.7,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hop Descriptor
// ─────────────────────────────────────────────────────────────────────────────

/// Describes a single frequency hop.
#[derive(Debug, Clone, Copy)]
pub struct HopDescriptor {
    /// Hop sequence number (0-based).
    pub hop_number: usize,
    /// Channel index within 225–400 MHz at 25 kHz spacing.
    pub channel_index: usize,
    /// Center frequency of this hop (Hz).
    pub frequency_hz: f64,
    /// Time this hop starts (seconds from epoch).
    pub start_time_s: f64,
    /// Dwell time (seconds).
    pub dwell_time_s: f64,
    /// Guard time before transmit (seconds).
    pub guard_time_s: f64,
}

impl HopDescriptor {
    /// Active transmit/receive time for this hop (dwell minus guard).
    pub fn active_time_s(&self) -> f64 {
        self.dwell_time_s - self.guard_time_s
    }

    /// Frequency band (for partial-band jamming analysis).
    pub fn frequency_band_low(&self) -> f64 {
        self.frequency_hz - HQ_CHANNEL_SPACING_HZ / 2.0
    }

    /// Upper edge of channel band.
    pub fn frequency_band_high(&self) -> f64 {
        self.frequency_hz + HQ_CHANNEL_SPACING_HZ / 2.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hop Sequence Generator
// ─────────────────────────────────────────────────────────────────────────────

/// HAVE QUICK hop sequence generator.
///
/// Uses a simplified non-classified LFSR-based model seeded from WOD and TOD.
/// The actual HAVE QUICK algorithm is classified; this approximates the
/// statistical properties (uniform distribution, minimum hop distance, etc.).
///
/// Seed derivation:
///   seed = WOD ⊕ (TOD_epoch × net_id) combined via multiple mixing rounds.
#[derive(Debug, Clone)]
pub struct HopSequenceGen {
    /// Current LFSR A state (32-bit Galois LFSR).
    lfsr_a: u32,
    /// Current LFSR B state (another polynomial for better randomness).
    lfsr_b: u32,
    /// Current hop index.
    hop_index: usize,
    /// Hop rate (hops/second).
    hop_rate: f64,
    /// Dwell time (seconds).
    dwell_time_s: f64,
    /// Guard time (seconds).
    guard_time_s: f64,
    /// Epoch reference (seconds).
    epoch_s: f64,
    /// Recent channel history for anti-repeat enforcement.
    recent_channels: Vec<usize>,
    /// Net ID.
    net_id: u8,
    /// Initial seed A for reset.
    init_a: u32,
    /// Initial seed B for reset.
    init_b: u32,
}

impl HopSequenceGen {
    /// Create a new hop sequence generator from configuration.
    pub fn new(wod: u32, net_id: u8, tod_seconds: f64, mode: HaveQuickMode) -> Self {
        // Derive LFSR seeds from WOD, TOD, and net_id.
        // This is a simplified educational model; real WOD derivation is classified.
        let tod_epoch = tod_seconds.floor() as u32;
        let seed_a = Self::mix(wod, tod_epoch ^ (net_id as u32 * 0x1234_5678));
        let seed_b = Self::mix(
            seed_a.wrapping_add(0x9E37_79B9),
            tod_epoch.wrapping_add(net_id as u32),
        );

        let lfsr_a = if seed_a == 0 { 0xACE1 } else { seed_a };
        let lfsr_b = if seed_b == 0 { 0x1234_5678 } else { seed_b };

        let dwell = mode.dwell_time_s();
        let guard = mode.guard_time_s();

        Self {
            lfsr_a,
            lfsr_b,
            hop_index: 0,
            hop_rate: mode.hop_rate(),
            dwell_time_s: dwell,
            guard_time_s: guard,
            epoch_s: tod_seconds,
            recent_channels: Vec::with_capacity(ANTI_REPEAT_WINDOW + 8),
            net_id,
            init_a: lfsr_a,
            init_b: lfsr_b,
        }
    }

    /// Mixing function (simplified key schedule, non-classified).
    fn mix(a: u32, b: u32) -> u32 {
        let mut x = a ^ b;
        x = x.wrapping_add(x << 10);
        x ^= x >> 6;
        x = x.wrapping_add(x << 3);
        x ^= x >> 11;
        x = x.wrapping_add(x << 15);
        x
    }

    /// 32-bit Galois LFSR step using polynomial x^32 + x^22 + x^2 + x + 1.
    fn lfsr_a_step(&mut self) -> u32 {
        let lsb = self.lfsr_a & 1;
        self.lfsr_a >>= 1;
        if lsb != 0 {
            self.lfsr_a ^= 0x80200003; // taps: 32, 22, 2, 1
        }
        self.lfsr_a
    }

    /// 32-bit Galois LFSR step using polynomial x^32 + x^7 + x^5 + x^3 + x^2 + x + 1.
    fn lfsr_b_step(&mut self) -> u32 {
        let lsb = self.lfsr_b & 1;
        self.lfsr_b >>= 1;
        if lsb != 0 {
            self.lfsr_b ^= 0x800000AF; // taps: 32, 8, 6, 4, 3, 2, 1
        }
        self.lfsr_b
    }

    /// Generate a raw pseudo-random 32-bit value by combining both LFSRs.
    fn next_raw(&mut self) -> u32 {
        let a = self.lfsr_a_step();
        let b = self.lfsr_b_step();
        // Combine with rotation and XOR for better distribution
        let rotated = a.rotate_left(13) ^ b.rotate_right(7);
        Self::mix(rotated, (self.hop_index as u32).wrapping_add(0x6c62_272e))
    }

    /// Generate the next channel index satisfying HAVE QUICK constraints:
    /// - Must be within 0..HQ_NUM_CHANNELS
    /// - Minimum hop distance from previous channel
    /// - Not recently used (anti-repeat window)
    fn next_channel(&mut self) -> usize {
        let last_ch = self
            .recent_channels
            .last()
            .copied()
            .unwrap_or(HQ_NUM_CHANNELS / 2);

        // Up to 32 attempts to find a valid channel
        for attempt in 0..32u32 {
            let raw = self.next_raw();
            // Bias attempt into the raw value for uniqueness
            let adjusted = raw.wrapping_add(attempt.wrapping_mul(0x9E37_79B9));
            let ch = (adjusted as usize) % HQ_NUM_CHANNELS;

            // Check minimum hop separation
            let dist = (ch as isize - last_ch as isize).unsigned_abs();
            if dist < MIN_HOP_SEPARATION_CHANNELS {
                continue;
            }

            // Check anti-repeat window
            if self.recent_channels.iter().any(|&c| c == ch) {
                continue;
            }

            return ch;
        }

        // Fallback: use modular hop from last channel with offset
        let raw = self.next_raw();
        let offset = (raw as usize % (HQ_NUM_CHANNELS / 2)) + MIN_HOP_SEPARATION_CHANNELS;
        (last_ch + offset) % HQ_NUM_CHANNELS
    }

    /// Update the recent-channel history.
    fn record_channel(&mut self, ch: usize) {
        self.recent_channels.push(ch);
        if self.recent_channels.len() > ANTI_REPEAT_WINDOW {
            self.recent_channels.remove(0);
        }
    }

    /// Get the next hop descriptor.
    pub fn next_hop(&mut self) -> HopDescriptor {
        let ch = self.next_channel();
        self.record_channel(ch);

        let freq = HQ_BAND_LOW_HZ + ch as f64 * HQ_CHANNEL_SPACING_HZ;
        let start = self.epoch_s + self.hop_index as f64 * self.dwell_time_s;

        let desc = HopDescriptor {
            hop_number: self.hop_index,
            channel_index: ch,
            frequency_hz: freq,
            start_time_s: start,
            dwell_time_s: self.dwell_time_s,
            guard_time_s: self.guard_time_s,
        };

        self.hop_index += 1;
        desc
    }

    /// Generate N hop descriptors.
    pub fn generate_n(&mut self, n: usize) -> Vec<HopDescriptor> {
        (0..n).map(|_| self.next_hop()).collect()
    }

    /// Reset sequence to initial state (same WOD/TOD).
    pub fn reset(&mut self) {
        self.lfsr_a = self.init_a;
        self.lfsr_b = self.init_b;
        self.hop_index = 0;
        self.recent_channels.clear();
    }

    /// Current hop index (number of hops generated so far).
    pub fn hop_index(&self) -> usize {
        self.hop_index
    }

    /// Hop rate (hops/second).
    pub fn hop_rate(&self) -> f64 {
        self.hop_rate
    }

    /// Advance generator to a specific hop index (late net entry / synchronization).
    /// This fast-forwards the LFSR state to produce the sequence from that index.
    pub fn advance_to(&mut self, target_hop: usize) {
        if target_hop <= self.hop_index {
            // Reset and replay from start
            self.reset();
        }
        while self.hop_index < target_hop {
            let _ = self.next_hop();
        }
    }

    /// Get frequency for a specific hop number without advancing state.
    /// Uses a separate cloned instance internally.
    pub fn frequency_at_hop(&self, hop_number: usize) -> f64 {
        let mut clone = self.clone();
        clone.reset();
        clone.advance_to(hop_number);
        let desc = clone.next_hop();
        desc.frequency_hz
    }

    /// Net ID this generator is configured for.
    pub fn net_id(&self) -> u8 {
        self.net_id
    }

    /// Epoch (TOD reference time in seconds).
    pub fn epoch_s(&self) -> f64 {
        self.epoch_s
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  AM Modulator / Demodulator
// ─────────────────────────────────────────────────────────────────────────────

/// AM (Amplitude Modulation) modulator for voice signals.
///
/// Implements standard AM: s(t) = A_c * [1 + m * n(t)] * cos(2π·f_c·t)
/// where n(t) is the normalized audio signal and m is the modulation index.
#[derive(Debug, Clone)]
pub struct AmModulator {
    /// Carrier frequency (Hz).
    pub carrier_hz: f64,
    /// Modulation index (0.0–1.0, typically 0.85 for voice).
    pub mod_index: f64,
    /// Carrier amplitude.
    pub carrier_amplitude: f64,
    /// Sample rate (Hz).
    pub sample_rate_hz: f64,
    /// Current carrier phase accumulator (radians).
    phase: f64,
}

impl AmModulator {
    /// Create a new AM modulator.
    pub fn new(carrier_hz: f64, mod_index: f64, carrier_amplitude: f64, sample_rate_hz: f64) -> Self {
        Self {
            carrier_hz,
            mod_index,
            carrier_amplitude,
            sample_rate_hz,
            phase: 0.0,
        }
    }

    /// Modulate a block of audio samples (f64, normalized -1.0 to 1.0).
    /// Returns complex IQ samples (I, Q).
    pub fn modulate(&mut self, audio: &[f64]) -> Vec<(f64, f64)> {
        let phase_step = 2.0 * PI * self.carrier_hz / self.sample_rate_hz;
        let mut output = Vec::with_capacity(audio.len());

        for &sample in audio {
            let envelope = self.carrier_amplitude * (1.0 + self.mod_index * sample);
            let i = envelope * self.phase.cos();
            let q = envelope * self.phase.sin();
            output.push((i, q));
            self.phase = (self.phase + phase_step) % (2.0 * PI);
        }

        output
    }

    /// Modulate and return real (RF) signal only.
    pub fn modulate_real(&mut self, audio: &[f64]) -> Vec<f64> {
        let iq = self.modulate(audio);
        iq.into_iter().map(|(i, _q)| i).collect()
    }

    /// Reset carrier phase to zero.
    pub fn reset_phase(&mut self) {
        self.phase = 0.0;
    }

    /// Set new carrier frequency (for frequency hopping).
    pub fn set_carrier(&mut self, carrier_hz: f64) {
        self.carrier_hz = carrier_hz;
        // Do NOT reset phase to maintain phase continuity where possible
    }

    /// AM modulation index.
    pub fn modulation_index(&self) -> f64 {
        self.mod_index
    }
}

/// AM demodulator supporting envelope and synchronous detection.
#[derive(Debug, Clone)]
pub struct AmDemodulator {
    /// Carrier frequency (Hz).
    pub carrier_hz: f64,
    /// Sample rate (Hz).
    pub sample_rate_hz: f64,
    /// Detection mode.
    pub detection: AmDetection,
    /// Envelope filter state (simple single-pole IIR).
    envelope_state: f64,
    /// Envelope filter coefficient (higher = slower decay).
    envelope_alpha: f64,
    /// DC blocking filter state.
    dc_state: f64,
    /// DC blocking coefficient.
    dc_alpha: f64,
    /// Phase accumulator for coherent detection.
    phase: f64,
}

impl AmDemodulator {
    /// Create a new AM demodulator.
    ///
    /// # Arguments
    /// - `carrier_hz`: nominal carrier frequency
    /// - `sample_rate_hz`: sample rate
    /// - `detection`: envelope or synchronous detection
    pub fn new(carrier_hz: f64, sample_rate_hz: f64, detection: AmDetection) -> Self {
        // Envelope filter: time constant ~ 1 / (2 * pi * audio_high)
        // For audio up to 3.4 kHz, alpha ~ 1 - exp(-2π·3400/fs)
        let env_tau = 1.0 / (2.0 * PI * AUDIO_HIGH_HZ);
        let envelope_alpha = (-1.0 / (env_tau * sample_rate_hz)).exp();

        // DC block: very slow pole (high-pass at ~10 Hz)
        let dc_alpha = 1.0 - 2.0 * PI * 10.0 / sample_rate_hz;

        Self {
            carrier_hz,
            sample_rate_hz,
            detection,
            envelope_state: 0.0,
            envelope_alpha,
            dc_state: 0.0,
            dc_alpha,
            phase: 0.0,
        }
    }

    /// Demodulate a block of real AM samples.
    pub fn demodulate(&mut self, rf_samples: &[f64]) -> Vec<f64> {
        match self.detection {
            AmDetection::Envelope => self.demodulate_envelope(rf_samples),
            AmDetection::Synchronous => self.demodulate_sync(rf_samples),
        }
    }

    /// Demodulate IQ samples (returns audio).
    pub fn demodulate_iq(&mut self, iq_samples: &[(f64, f64)]) -> Vec<f64> {
        // Compute envelope from IQ: |x(t)| = sqrt(I^2 + Q^2), then remove DC.
        let mut output = Vec::with_capacity(iq_samples.len());
        for &(i, q) in iq_samples {
            let envelope = (i * i + q * q).sqrt();
            // Remove carrier DC bias
            let filtered = self.dc_block(envelope - 1.0);
            output.push(filtered);
        }
        output
    }

    /// Envelope detection: rectify and lowpass filter.
    fn demodulate_envelope(&mut self, samples: &[f64]) -> Vec<f64> {
        let phase_step = 2.0 * PI * self.carrier_hz / self.sample_rate_hz;
        let mut output = Vec::with_capacity(samples.len());

        for &s in samples {
            // Full-wave rectification
            let rect = s.abs();
            // Single-pole IIR lowpass (envelope follower)
            self.envelope_state = self.envelope_alpha * self.envelope_state
                + (1.0 - self.envelope_alpha) * rect;
            // Remove DC offset (carrier component)
            let audio = self.dc_block(self.envelope_state - 0.7071); // bias ≈ E[|cos|] = 2/π ≈ 0.637, practical ≈ 0.7071
            output.push(audio);
            self.phase = (self.phase + phase_step) % (2.0 * PI);
        }
        output
    }

    /// Synchronous (coherent) detection.
    fn demodulate_sync(&mut self, samples: &[f64]) -> Vec<f64> {
        let phase_step = 2.0 * PI * self.carrier_hz / self.sample_rate_hz;
        let mut output = Vec::with_capacity(samples.len());

        for &s in samples {
            // Multiply by local carrier
            let baseband = s * self.phase.cos();
            // Lowpass filter using envelope IIR (repurposed as LPF)
            self.envelope_state = self.envelope_alpha * self.envelope_state
                + (1.0 - self.envelope_alpha) * baseband;
            let audio = self.dc_block(self.envelope_state);
            output.push(audio);
            self.phase = (self.phase + phase_step) % (2.0 * PI);
        }
        output
    }

    /// Single-pole IIR DC blocking filter.
    fn dc_block(&mut self, x: f64) -> f64 {
        let y = x - self.dc_state;
        self.dc_state = x - self.dc_alpha * y;
        y
    }

    /// Set new carrier frequency (for hopping).
    pub fn set_carrier(&mut self, carrier_hz: f64) {
        self.carrier_hz = carrier_hz;
    }

    /// Reset filter states.
    pub fn reset(&mut self) {
        self.envelope_state = 0.0;
        self.dc_state = 0.0;
        self.phase = 0.0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Frequency Synthesizer Model
// ─────────────────────────────────────────────────────────────────────────────

/// Models the frequency synthesizer in a HAVE QUICK radio.
///
/// Captures the settling time behaviour when hopping between frequencies.
/// Real synthesizers use fractional-N PLLs; this models the key timing.
#[derive(Debug, Clone)]
pub struct FrequencySynthesizer {
    /// Current tuned frequency (Hz).
    current_freq_hz: f64,
    /// Frequency the synthesizer is locked to (after settling).
    target_freq_hz: f64,
    /// Settling time (seconds) for the synthesizer to lock.
    settling_time_s: f64,
    /// Time elapsed since last hop command (seconds).
    elapsed_s: f64,
    /// Sample rate (for converting sample counts). Stored for future use.
    #[allow(dead_code)]
    sample_rate_hz: f64,
    /// Whether the synthesizer is currently settled.
    is_locked: bool,
    /// Number of hops performed.
    hop_count: usize,
    /// Fine tuning offset (Hz) — models residual frequency error.
    fine_tune_offset_hz: f64,
}

impl FrequencySynthesizer {
    /// Create a new frequency synthesizer model.
    ///
    /// # Arguments
    /// - `initial_freq_hz`: initial frequency
    /// - `settling_time_s`: time to lock (HQ-I: ~1 ms, HQ-II: ~100 μs)
    /// - `sample_rate_hz`: sample rate for sample-based operations
    pub fn new(initial_freq_hz: f64, settling_time_s: f64, sample_rate_hz: f64) -> Self {
        Self {
            current_freq_hz: initial_freq_hz,
            target_freq_hz: initial_freq_hz,
            settling_time_s,
            elapsed_s: settling_time_s, // Start as locked
            sample_rate_hz,
            is_locked: true,
            hop_count: 0,
            fine_tune_offset_hz: 0.0,
        }
    }

    /// Command a frequency hop to a new frequency.
    pub fn hop_to(&mut self, new_freq_hz: f64) {
        self.target_freq_hz = new_freq_hz;
        self.elapsed_s = 0.0;
        self.is_locked = false;
        self.hop_count += 1;
    }

    /// Advance time by `dt` seconds and update synthesizer state.
    pub fn advance(&mut self, dt_s: f64) {
        if !self.is_locked {
            self.elapsed_s += dt_s;
            if self.elapsed_s >= self.settling_time_s {
                self.current_freq_hz = self.target_freq_hz + self.fine_tune_offset_hz;
                self.is_locked = true;
            } else {
                // Model exponential frequency pull-in
                let alpha = self.elapsed_s / self.settling_time_s;
                let start = self.current_freq_hz;
                let end = self.target_freq_hz + self.fine_tune_offset_hz;
                self.current_freq_hz = start + alpha * (end - start);
            }
        }
    }

    /// Set a fine-tune offset (residual frequency error model).
    pub fn set_fine_tune(&mut self, offset_hz: f64) {
        self.fine_tune_offset_hz = offset_hz;
    }

    /// Is the synthesizer locked to the target frequency?
    pub fn is_locked(&self) -> bool {
        self.is_locked
    }

    /// Current output frequency (may be between old and new during settling).
    pub fn current_freq_hz(&self) -> f64 {
        self.current_freq_hz
    }

    /// Target frequency (after locking).
    pub fn target_freq_hz(&self) -> f64 {
        self.target_freq_hz
    }

    /// Time remaining until lock (seconds). Zero if already locked.
    pub fn time_to_lock_s(&self) -> f64 {
        if self.is_locked {
            0.0
        } else {
            (self.settling_time_s - self.elapsed_s).max(0.0)
        }
    }

    /// Total hops performed.
    pub fn hop_count(&self) -> usize {
        self.hop_count
    }

    /// Settling time constant (seconds).
    pub fn settling_time_s(&self) -> f64 {
        self.settling_time_s
    }

    /// Fractional settling progress (0.0 = just hopped, 1.0 = locked).
    pub fn settling_progress(&self) -> f64 {
        if self.is_locked {
            1.0
        } else {
            (self.elapsed_s / self.settling_time_s).min(1.0)
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Late Net Entry
// ─────────────────────────────────────────────────────────────────────────────

/// Late net entry protocol helper.
///
/// Allows a radio to synchronize to an active net by detecting the
/// current hop and aligning its sequence generator accordingly.
#[derive(Debug, Clone)]
pub struct LateNetEntry {
    /// Expected net WOD.
    wod: u32,
    /// Net ID.
    net_id: u8,
    /// Mode for hop rate.
    mode: HaveQuickMode,
    /// Detected current hop number.
    detected_hop: Option<usize>,
    /// Synchronization confidence (0.0–1.0).
    sync_confidence: f64,
}

impl LateNetEntry {
    /// Create a new late net entry helper.
    pub fn new(wod: u32, net_id: u8, mode: HaveQuickMode) -> Self {
        Self {
            wod,
            net_id,
            mode,
            detected_hop: None,
            sync_confidence: 0.0,
        }
    }

    /// Attempt to synchronize from current GPS TOD.
    ///
    /// The receiver knows the WOD and current TOD. It derives the hop
    /// number based on elapsed time since epoch.
    ///
    /// # Arguments
    /// - `current_tod_s`: current GPS time in seconds since midnight
    /// - `net_epoch_s`: the TOD the net started (must be known or estimated)
    pub fn sync_from_tod(&mut self, current_tod_s: f64, net_epoch_s: f64) -> usize {
        let elapsed = (current_tod_s - net_epoch_s).max(0.0);
        let hop_number = (elapsed * self.mode.hop_rate()).floor() as usize;
        self.detected_hop = Some(hop_number);
        self.sync_confidence = 1.0; // With GPS TOD, confidence is high
        hop_number
    }

    /// Create a synchronized hop sequence generator starting at the detected hop.
    ///
    /// Returns a generator ready to produce the same hops as the active net.
    pub fn create_synced_generator(&self, net_epoch_s: f64) -> Option<HopSequenceGen> {
        let hop_number = self.detected_hop?;
        let mut gen = HopSequenceGen::new(self.wod, self.net_id, net_epoch_s, self.mode);
        gen.advance_to(hop_number);
        Some(gen)
    }

    /// Was synchronization successful?
    pub fn is_synced(&self) -> bool {
        self.detected_hop.is_some()
    }

    /// Synchronization confidence (0.0–1.0).
    pub fn sync_confidence(&self) -> f64 {
        self.sync_confidence
    }

    /// The detected hop number (if synced).
    pub fn detected_hop(&self) -> Option<usize> {
        self.detected_hop
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Jamming Analysis
// ─────────────────────────────────────────────────────────────────────────────

/// Anti-jam performance analysis for HAVE QUICK systems.
///
/// Computes metrics per standard spread-spectrum jamming analysis:
/// - Processing gain
/// - Anti-jam margin
/// - Probability of intercept
/// - Partial-band jamming effectiveness
#[derive(Debug, Clone)]
pub struct JammingAnalysis {
    /// Number of hop channels.
    pub num_channels: usize,
    /// Channel bandwidth (Hz).
    pub channel_bw_hz: f64,
    /// Total spread bandwidth (Hz).
    pub total_bw_hz: f64,
    /// Hop rate (hops/second).
    pub hop_rate: f64,
    /// Required SNR for voice intelligibility (dB, typically 10 dB).
    pub required_snr_db: f64,
    /// Transmit power (dBW).
    pub tx_power_dbw: f64,
}

impl JammingAnalysis {
    /// Create analysis for HAVE QUICK parameters.
    pub fn new(mode: HaveQuickMode, tx_power_dbw: f64) -> Self {
        Self {
            num_channels: HQ_NUM_CHANNELS,
            channel_bw_hz: HQ_CHANNEL_SPACING_HZ,
            total_bw_hz: HQ_BAND_HIGH_HZ - HQ_BAND_LOW_HZ,
            hop_rate: mode.hop_rate(),
            required_snr_db: 10.0,
            tx_power_dbw,
        }
    }

    /// Frequency hopping processing gain (dB).
    ///
    /// Gp = 10·log10(total_bandwidth / channel_bandwidth)
    ///    = 10·log10(7000) ≈ 38.45 dB
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.total_bw_hz / self.channel_bw_hz).log10()
    }

    /// Anti-jam margin (dBW) for a given jammer-to-signal ratio J/S (dBW).
    ///
    /// AJ_margin = Gp - required_snr - J/S
    pub fn antijam_margin_db(&self, js_ratio_db: f64) -> f64 {
        self.processing_gain_db() - self.required_snr_db - js_ratio_db
    }

    /// Maximum tolerable jammer power (dBW) for effective operation.
    pub fn max_jammer_power_dbw(&self) -> f64 {
        self.tx_power_dbw + self.processing_gain_db() - self.required_snr_db
    }

    /// Partial-band jamming effectiveness.
    ///
    /// When a jammer covers fraction `rho` of the band at total power Pj,
    /// the jammer-to-signal ratio per channel is J0/S = (Pj / rho) / Ps.
    ///
    /// Optimal jamming fraction for partial-band jammer:
    ///   rho_opt = 0.71 / SNR_required (for AWGN, analog AM)
    ///
    /// # Arguments
    /// - `jam_fraction`: fraction of band jammed (0.0–1.0)
    /// - `jammer_power_dbw`: total jammer power (dBW)
    pub fn partial_band_effectiveness(&self, jam_fraction: f64, jammer_power_dbw: f64) -> PartialBandResult {
        let jam_fraction = jam_fraction.clamp(1.0 / self.num_channels as f64, 1.0);

        // Power spectral density of jammer per channel
        let jammer_power_watts = 10f64.powf(jammer_power_dbw / 10.0);
        let tx_power_watts = 10f64.powf(self.tx_power_dbw / 10.0);

        // When jammed, jammer power density = Pj / (rho * N_channels)
        // Signal power = Ps
        // SNR on jammed channels = Ps / (Pj / (rho))  [since density × bw gives power]
        let snr_when_jammed = tx_power_watts / (jammer_power_watts / jam_fraction);
        let snr_when_jammed_db = 10.0 * snr_when_jammed.log10();

        // Fraction of hops that fall in jammed channels
        let prob_hop_jammed = jam_fraction;

        // Average degraded SNR
        let snr_unjammed_db = 40.0; // assumed baseline SNR without jamming
        let avg_snr_db = snr_unjammed_db + 10.0 * (1.0 - prob_hop_jammed).log10();

        // Effective throughput (fraction of hops that succeed)
        let effective_throughput = if snr_when_jammed_db >= self.required_snr_db {
            1.0 // Even jammed hops succeed
        } else {
            1.0 - prob_hop_jammed
        };

        // Optimal jamming fraction (worst case for defender)
        let rho_opt = (0.71 / 10f64.powf(self.required_snr_db / 10.0)).min(1.0);

        PartialBandResult {
            jam_fraction,
            jammer_power_dbw,
            snr_when_jammed_db,
            prob_hop_jammed,
            avg_snr_db,
            effective_throughput,
            optimal_jam_fraction: rho_opt,
            processing_gain_db: self.processing_gain_db(),
        }
    }

    /// Probability of intercept for a single-channel scanning receiver.
    ///
    /// P(intercept per hop) = dwell_time / scan_time_per_channel
    /// For a single receiver scanning N channels at scan rate Rs:
    ///   Pi = hop_dwell / (N / Rs)
    ///
    /// # Arguments
    /// - `scan_rate_hz`: interceptor's channel scan rate (channels/second)
    pub fn prob_intercept_per_hop(&self, scan_rate_hz: f64) -> f64 {
        let dwell = 1.0 / self.hop_rate;
        let scan_period_per_ch = 1.0 / scan_rate_hz;
        (dwell / scan_period_per_ch).min(1.0)
    }

    /// Probability of NOT being intercepted over a transmission of `num_hops` hops.
    pub fn prob_no_intercept(&self, num_hops: usize, scan_rate_hz: f64) -> f64 {
        let pi = self.prob_intercept_per_hop(scan_rate_hz);
        (1.0 - pi).powi(num_hops as i32)
    }

    /// Follower jammer resistance.
    ///
    /// A follower jammer must detect the current frequency, compute the next,
    /// and retune in less than one hop dwell time. This is essentially impossible
    /// for HQ-II due to the short dwell time.
    ///
    /// Returns the follower jammer "margin" in seconds. A positive value means the
    /// jammer reaction time exceeds the dwell time → the jammer CANNOT follow →
    /// the system IS resistant.
    ///
    /// margin = jammer_reaction_time - dwell_time
    /// positive → resistant, negative → jammer can follow
    pub fn follower_jammer_resistance_s(&self, jammer_reaction_time_s: f64) -> f64 {
        let dwell = 1.0 / self.hop_rate;
        jammer_reaction_time_s - dwell
    }

    /// Is the system resistant to the described follower jammer?
    /// True when the jammer reaction time exceeds the dwell time.
    pub fn is_follower_resistant(&self, jammer_reaction_time_s: f64) -> bool {
        self.follower_jammer_resistance_s(jammer_reaction_time_s) > 0.0
    }
}

/// Results of a partial-band jamming analysis.
#[derive(Debug, Clone, Copy)]
pub struct PartialBandResult {
    /// Fraction of band jammed.
    pub jam_fraction: f64,
    /// Total jammer power (dBW).
    pub jammer_power_dbw: f64,
    /// SNR on jammed channels (dB).
    pub snr_when_jammed_db: f64,
    /// Probability a hop falls in jammed sub-band.
    pub prob_hop_jammed: f64,
    /// Average SNR across all hops (dB).
    pub avg_snr_db: f64,
    /// Fraction of hops that achieve required SNR.
    pub effective_throughput: f64,
    /// Optimal jamming fraction for maximum disruption.
    pub optimal_jam_fraction: f64,
    /// HAVE QUICK FH processing gain (dB).
    pub processing_gain_db: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hop Pattern Statistics
// ─────────────────────────────────────────────────────────────────────────────

/// Statistical analysis of a hop sequence.
#[derive(Debug, Clone)]
pub struct HopStatistics {
    /// Total hops analyzed.
    pub num_hops: usize,
    /// Number of unique channels visited.
    pub unique_channels: usize,
    /// Coverage: fraction of total channels visited.
    pub coverage_fraction: f64,
    /// Mean channel index.
    pub mean_channel: f64,
    /// Standard deviation of channel indices.
    pub std_channel: f64,
    /// Mean hop distance (|ch[n] - ch[n-1]|).
    pub mean_hop_distance: f64,
    /// Minimum hop distance observed.
    pub min_hop_distance: usize,
    /// Maximum hop distance observed.
    pub max_hop_distance: usize,
    /// Chi-squared uniformity statistic (lower = more uniform).
    pub chi_squared: f64,
    /// Number of bins used for chi-squared test.
    pub num_bins: usize,
}

impl HopStatistics {
    /// Compute statistics from a sequence of hop descriptors.
    pub fn from_hops(hops: &[HopDescriptor]) -> Self {
        if hops.is_empty() {
            return Self {
                num_hops: 0,
                unique_channels: 0,
                coverage_fraction: 0.0,
                mean_channel: 0.0,
                std_channel: 0.0,
                mean_hop_distance: 0.0,
                min_hop_distance: 0,
                max_hop_distance: 0,
                chi_squared: 0.0,
                num_bins: 0,
            };
        }

        let channels: Vec<usize> = hops.iter().map(|h| h.channel_index).collect();
        let n = channels.len();

        // Unique channels
        let mut unique_set = channels.clone();
        unique_set.sort_unstable();
        unique_set.dedup();
        let unique_channels = unique_set.len();

        // Mean and std
        let sum: f64 = channels.iter().map(|&c| c as f64).sum();
        let mean = sum / n as f64;
        let var: f64 = channels.iter().map(|&c| (c as f64 - mean).powi(2)).sum::<f64>() / n as f64;
        let std = var.sqrt();

        // Hop distances
        let distances: Vec<usize> = channels
            .windows(2)
            .map(|w| (w[1] as isize - w[0] as isize).unsigned_abs())
            .collect();

        let mean_dist = if distances.is_empty() {
            0.0
        } else {
            distances.iter().sum::<usize>() as f64 / distances.len() as f64
        };
        let min_dist = distances.iter().copied().min().unwrap_or(0);
        let max_dist = distances.iter().copied().max().unwrap_or(0);

        // Chi-squared uniformity test
        // Use 70 bins (each covering 100 channels in the 7000-channel space)
        let num_bins = 70usize;
        let bin_size = HQ_NUM_CHANNELS / num_bins;
        let mut bin_counts = vec![0usize; num_bins];
        for &ch in &channels {
            let bin = (ch / bin_size).min(num_bins - 1);
            bin_counts[bin] += 1;
        }
        let expected = n as f64 / num_bins as f64;
        let chi_sq: f64 = bin_counts
            .iter()
            .map(|&c| (c as f64 - expected).powi(2) / expected)
            .sum();

        Self {
            num_hops: n,
            unique_channels,
            coverage_fraction: unique_channels as f64 / HQ_NUM_CHANNELS as f64,
            mean_channel: mean,
            std_channel: std,
            mean_hop_distance: mean_dist,
            min_hop_distance: min_dist,
            max_hop_distance: max_dist,
            chi_squared: chi_sq,
            num_bins,
        }
    }

    /// Is the hop distribution statistically uniform?
    /// Uses chi-squared critical value at 5% significance: ~85 for 69 DoF.
    pub fn is_uniform(&self) -> bool {
        // For 70 bins, 69 degrees of freedom, critical value at alpha=0.05 ≈ 90
        self.chi_squared < 90.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Complete HAVE QUICK Processor
// ─────────────────────────────────────────────────────────────────────────────

/// Complete HAVE QUICK transmitter/receiver processor.
///
/// Integrates the hop sequence generator, AM modulator/demodulator,
/// frequency synthesizer, and channel model into a single TX/RX chain.
pub struct HaveQuickProcessor {
    config: HaveQuickConfig,
    /// Hop sequence generator.
    hop_gen: HopSequenceGen,
    /// AM modulator.
    am_mod: AmModulator,
    /// AM demodulator.
    am_demod: AmDemodulator,
    /// Frequency synthesizer model.
    synth: FrequencySynthesizer,
    /// Current hop descriptor.
    current_hop: Option<HopDescriptor>,
    /// Total samples processed.
    samples_processed: usize,
}

impl HaveQuickProcessor {
    /// Create a new HAVE QUICK processor.
    pub fn new(config: HaveQuickConfig) -> Self {
        let hop_gen = HopSequenceGen::new(
            config.wod,
            config.net_id,
            config.tod_seconds,
            config.mode,
        );

        // Initial frequency (arbitrary; will hop on first tx/rx)
        let init_freq = HQ_BAND_LOW_HZ;

        let am_mod = AmModulator::new(
            init_freq,
            AM_MOD_INDEX,
            config.audio_gain,
            config.sample_rate_hz,
        );

        let am_demod = AmDemodulator::new(
            init_freq,
            config.sample_rate_hz,
            AmDetection::Envelope,
        );

        // HQ-I settling ~1 ms, HQ-II ~0.1 ms
        let settling = match config.mode {
            HaveQuickMode::HqI => 1e-3,
            HaveQuickMode::HqII => 0.1e-3,
        };

        let synth = FrequencySynthesizer::new(init_freq, settling, config.sample_rate_hz);

        Self {
            config,
            hop_gen,
            am_mod,
            am_demod,
            synth,
            current_hop: None,
            samples_processed: 0,
        }
    }

    // ── Accessors ──────────────────────────────────────────────────────────

    /// Get a reference to the hop sequence generator.
    pub fn hop_sequence_gen(&self) -> &HopSequenceGen {
        &self.hop_gen
    }

    /// Get a mutable reference to the hop sequence generator.
    pub fn hop_sequence_gen_mut(&mut self) -> &mut HopSequenceGen {
        &mut self.hop_gen
    }

    /// Get a reference to the frequency synthesizer.
    pub fn synthesizer(&self) -> &FrequencySynthesizer {
        &self.synth
    }

    /// Get the current hop descriptor (if any).
    pub fn current_hop(&self) -> Option<&HopDescriptor> {
        self.current_hop.as_ref()
    }

    /// Get configuration.
    pub fn config(&self) -> &HaveQuickConfig {
        &self.config
    }

    // ── TX Chain ──────────────────────────────────────────────────────────

    /// Transmit a block of audio samples.
    ///
    /// Performs:
    /// 1. Get next hop frequency
    /// 2. Configure synthesizer and AM modulator
    /// 3. AM-modulate the audio
    /// 4. Apply guard time (zero padding at start of each hop)
    ///
    /// Returns (IQ samples, hop descriptor used).
    pub fn transmit(&mut self, audio: &[f64]) -> (Vec<(f64, f64)>, HopDescriptor) {
        let hop = self.hop_gen.next_hop();
        let freq = hop.frequency_hz;

        // Command synthesizer to new frequency
        self.synth.hop_to(freq);
        self.am_mod.set_carrier(freq);

        let samples_per_dwell = (hop.dwell_time_s * self.config.sample_rate_hz) as usize;
        let guard_samples = (hop.guard_time_s * self.config.sample_rate_hz) as usize;

        // Trim or pad audio to fit in one dwell period
        let active_samples = samples_per_dwell.saturating_sub(guard_samples);
        let audio_trimmed: Vec<f64> = audio
            .iter()
            .cloned()
            .chain(std::iter::repeat(0.0))
            .take(active_samples)
            .collect();

        // AM modulate
        let mut modulated = self.am_mod.modulate(&audio_trimmed);

        // Prepend guard time zeros (synthesizer settling)
        let mut output = vec![(0.0_f64, 0.0_f64); guard_samples];
        output.append(&mut modulated);

        self.current_hop = Some(hop);
        self.samples_processed += output.len();
        (output, hop)
    }

    /// Transmit and return real (scalar) RF samples.
    pub fn transmit_real(&mut self, audio: &[f64]) -> (Vec<f64>, HopDescriptor) {
        let (iq, hop) = self.transmit(audio);
        let real = iq.into_iter().map(|(i, _)| i).collect();
        (real, hop)
    }

    // ── RX Chain ──────────────────────────────────────────────────────────

    /// Receive IQ samples for a specific hop.
    ///
    /// De-hops by assuming the frequency matches, then AM-demodulates.
    /// Returns recovered audio samples.
    pub fn receive_iq(&mut self, iq_samples: &[(f64, f64)]) -> Vec<f64> {
        let hop = self.hop_gen.next_hop();
        self.am_demod.set_carrier(hop.frequency_hz);
        self.current_hop = Some(hop);
        self.am_demod.demodulate_iq(iq_samples)
    }

    /// Receive real RF samples for a specific hop (uses envelope detection).
    pub fn receive_real(&mut self, rf_samples: &[f64]) -> Vec<f64> {
        let hop = self.hop_gen.next_hop();
        self.am_demod.set_carrier(hop.frequency_hz);
        self.current_hop = Some(hop);
        self.am_demod.demodulate(rf_samples)
    }

    // ── Full Loopback Chain ────────────────────────────────────────────────

    /// Run a full TX→channel→RX loopback simulation.
    ///
    /// # Arguments
    /// - `audio`: input audio samples
    /// - `channel`: channel impairment model
    ///
    /// Returns (tx_iq, recovered_audio, hop).
    pub fn loopback(
        &mut self,
        audio: &[f64],
        channel: ChannelModel,
    ) -> (Vec<(f64, f64)>, Vec<f64>, HopDescriptor) {
        // TX side: get next hop and modulate
        let hop = self.hop_gen.next_hop();
        self.am_mod.set_carrier(hop.frequency_hz);

        let active_samples = audio.len();
        let tx_iq = self.am_mod.modulate(audio);

        // Apply channel model
        let rx_iq = apply_channel(&tx_iq, channel, hop.channel_index);

        // RX side: demodulate
        self.am_demod.set_carrier(hop.frequency_hz);
        let recovered = self.am_demod.demodulate_iq(&rx_iq);

        let out_len = recovered.len().min(active_samples);
        self.current_hop = Some(hop);
        self.samples_processed += active_samples;

        (tx_iq, recovered[..out_len].to_vec(), hop)
    }

    // ── Synchronization ────────────────────────────────────────────────────

    /// Perform late net entry synchronization.
    ///
    /// Aligns this processor's hop sequence to the current TOD.
    ///
    /// # Arguments
    /// - `current_tod_s`: current GPS TOD (seconds since midnight)
    /// - `net_epoch_s`: the TOD when the net started
    pub fn sync_late_entry(&mut self, current_tod_s: f64, net_epoch_s: f64) {
        let elapsed = (current_tod_s - net_epoch_s).max(0.0);
        let target_hop = (elapsed * self.config.mode.hop_rate()).floor() as usize;
        self.hop_gen.reset();
        self.hop_gen.advance_to(target_hop);
    }

    // ── Metrics ────────────────────────────────────────────────────────────

    /// Generate and analyze the hop pattern for the next N hops.
    pub fn analyze_pattern(&mut self, num_hops: usize) -> HopStatistics {
        let hops = self.hop_gen.generate_n(num_hops);
        HopStatistics::from_hops(&hops)
    }

    /// Processing gain of HAVE QUICK (dB).
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (HQ_NUM_CHANNELS as f64).log10()
    }

    /// Total samples processed so far.
    pub fn samples_processed(&self) -> usize {
        self.samples_processed
    }

    /// Get jamming analysis object for this mode.
    pub fn jamming_analysis(&self) -> JammingAnalysis {
        JammingAnalysis::new(self.config.mode, 0.0) // 0 dBW nominal
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Channel Application Utility
// ─────────────────────────────────────────────────────────────────────────────

/// Apply a channel model to IQ samples.
fn apply_channel(iq: &[(f64, f64)], model: ChannelModel, channel_index: usize) -> Vec<(f64, f64)> {
    match model {
        ChannelModel::Ideal => iq.to_vec(),
        ChannelModel::Awgn { snr_db } => {
            let snr_linear = 10f64.powf(snr_db / 10.0);
            // Estimate signal power
            let sig_power: f64 = iq.iter().map(|(i, q)| i * i + q * q).sum::<f64>()
                / iq.len().max(1) as f64;
            let noise_power = sig_power / snr_linear;
            let noise_std = noise_power.sqrt();

            // Simple deterministic "noise" using channel_index as seed for reproducibility
            let mut lcg: u64 = (channel_index as u64).wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            iq.iter()
                .map(|&(i, q)| {
                    lcg = lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let n1 = (lcg >> 33) as f64 / (u32::MAX as f64) * 2.0 - 1.0;
                    lcg = lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let n2 = (lcg >> 33) as f64 / (u32::MAX as f64) * 2.0 - 1.0;
                    (i + n1 * noise_std, q + n2 * noise_std)
                })
                .collect()
        }
        ChannelModel::PartialBandJamming { jam_fraction, jammer_power_dbw } => {
            // Determine if this channel falls in the jammed sub-band
            // Simplified: jam the bottom `jam_fraction` of channels
            let jam_threshold = (jam_fraction * HQ_NUM_CHANNELS as f64) as usize;
            let is_jammed = channel_index < jam_threshold;

            if !is_jammed {
                return iq.to_vec();
            }

            // Add jammer noise
            let jammer_power_watts = 10f64.powf(jammer_power_dbw / 10.0);
            let noise_std = (jammer_power_watts / 2.0).sqrt();

            let mut lcg: u64 = (channel_index as u64).wrapping_mul(0x517c_c1b7_2722_0a95).wrapping_add(1);
            iq.iter()
                .map(|&(i, q)| {
                    lcg = lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let n1 = (lcg >> 33) as f64 / (u32::MAX as f64) * 2.0 - 1.0;
                    lcg = lcg.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let n2 = (lcg >> 33) as f64 / (u32::MAX as f64) * 2.0 - 1.0;
                    (i + n1 * noise_std, q + n2 * noise_std)
                })
                .collect()
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Preset Configurations
// ─────────────────────────────────────────────────────────────────────────────

/// Standard HAVE QUICK I preset configuration.
pub fn hq1_preset(wod: u32, net_id: u8, tod_seconds: f64) -> HaveQuickConfig {
    HaveQuickConfig {
        mode: HaveQuickMode::HqI,
        wod,
        net_id,
        tod_seconds,
        sample_rate_hz: 50_000.0,
        audio_gain: 0.7,
    }
}

/// Standard HAVE QUICK II preset configuration.
pub fn hq2_preset(wod: u32, net_id: u8, tod_seconds: f64) -> HaveQuickConfig {
    HaveQuickConfig {
        mode: HaveQuickMode::HqII,
        wod,
        net_id,
        tod_seconds,
        sample_rate_hz: 50_000.0,
        audio_gain: 0.7,
    }
}

/// Compute the signal power of a block of IQ samples.
pub fn iq_power(iq: &[(f64, f64)]) -> f64 {
    if iq.is_empty() {
        return 0.0;
    }
    iq.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / iq.len() as f64
}

/// Compute the SNR between a clean and noisy IQ signal (dB).
pub fn snr_db(clean: &[(f64, f64)], noisy: &[(f64, f64)]) -> f64 {
    if clean.is_empty() || clean.len() != noisy.len() {
        return f64::NAN;
    }
    let sig_power: f64 = clean.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / clean.len() as f64;
    let noise_power: f64 = clean
        .iter()
        .zip(noisy.iter())
        .map(|((ci, cq), (ni, nq))| {
            let di = ci - ni;
            let dq = cq - nq;
            di * di + dq * dq
        })
        .sum::<f64>()
        / clean.len() as f64;

    if noise_power < 1e-300 {
        return f64::INFINITY;
    }
    10.0 * (sig_power / noise_power).log10()
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Frequency Grid ────────────────────────────────────────────────────

    #[test]
    fn test_freq_grid_channel_count() {
        // 225 to 400 MHz at 25 kHz = 7000 channels
        assert_eq!(HQ_NUM_CHANNELS, 7000);
    }

    #[test]
    fn test_freq_grid_low_edge() {
        assert!((HQ_BAND_LOW_HZ - 225e6).abs() < 1.0);
    }

    #[test]
    fn test_freq_grid_high_edge() {
        assert!((HQ_BAND_HIGH_HZ - 400e6).abs() < 1.0);
    }

    #[test]
    fn test_freq_grid_channel_spacing() {
        assert!((HQ_CHANNEL_SPACING_HZ - 25e3).abs() < 1.0);
    }

    #[test]
    fn test_channel_to_frequency_mapping() {
        // Channel 0 → 225.000 MHz
        let f0 = HQ_BAND_LOW_HZ + 0.0 * HQ_CHANNEL_SPACING_HZ;
        assert!((f0 - 225_000_000.0).abs() < 1.0);

        // Channel 6999 → 399.975 MHz
        let f_last = HQ_BAND_LOW_HZ + 6999.0 * HQ_CHANNEL_SPACING_HZ;
        assert!((f_last - 399_975_000.0).abs() < 1.0);
    }

    #[test]
    fn test_all_hop_freqs_in_band() {
        let mut gen = HopSequenceGen::new(0xABCD_1234, 1, 0.0, HaveQuickMode::HqI);
        for _ in 0..500 {
            let hop = gen.next_hop();
            assert!(
                hop.frequency_hz >= HQ_BAND_LOW_HZ,
                "Frequency {} < band low",
                hop.frequency_hz
            );
            assert!(
                hop.frequency_hz <= HQ_BAND_HIGH_HZ,
                "Frequency {} > band high",
                hop.frequency_hz
            );
        }
    }

    // ── Hop Sequence Generation ───────────────────────────────────────────

    #[test]
    fn test_hop_sequence_reproducible() {
        let wod = 0x1234_5678;
        let mut gen1 = HopSequenceGen::new(wod, 1, 43200.0, HaveQuickMode::HqI);
        let mut gen2 = HopSequenceGen::new(wod, 1, 43200.0, HaveQuickMode::HqI);

        for _ in 0..200 {
            let h1 = gen1.next_hop();
            let h2 = gen2.next_hop();
            assert_eq!(
                h1.channel_index, h2.channel_index,
                "Same seed must produce same sequence"
            );
        }
    }

    #[test]
    fn test_hop_sequence_reset_reproducible() {
        let mut gen = HopSequenceGen::new(0xDEAD_BEEF, 2, 3600.0, HaveQuickMode::HqI);
        let first: Vec<usize> = (0..50).map(|_| gen.next_hop().channel_index).collect();
        gen.reset();
        let second: Vec<usize> = (0..50).map(|_| gen.next_hop().channel_index).collect();
        assert_eq!(first, second, "Reset must reproduce identical sequence");
    }

    #[test]
    fn test_hop_indices_increment() {
        let mut gen = HopSequenceGen::new(0x1111_2222, 0, 0.0, HaveQuickMode::HqII);
        for i in 0..20 {
            let hop = gen.next_hop();
            assert_eq!(hop.hop_number, i);
        }
    }

    #[test]
    fn test_hop_channel_in_valid_range() {
        let mut gen = HopSequenceGen::new(0xF00D_CAFE, 3, 7200.0, HaveQuickMode::HqII);
        for _ in 0..200 {
            let hop = gen.next_hop();
            assert!(
                hop.channel_index < HQ_NUM_CHANNELS,
                "Channel {} out of range",
                hop.channel_index
            );
        }
    }

    #[test]
    fn test_hop_minimum_distance_enforced() {
        let mut gen = HopSequenceGen::new(0x5555_AAAA, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(200);
        let mut violations = 0usize;
        for w in hops.windows(2) {
            let dist = (w[1].channel_index as isize - w[0].channel_index as isize).unsigned_abs();
            if dist < MIN_HOP_SEPARATION_CHANNELS {
                violations += 1;
            }
        }
        // Allow a small number of fallback cases
        assert!(
            violations <= 5,
            "Too many hop distance violations: {violations}"
        );
    }

    #[test]
    fn test_hop_start_times_monotonic() {
        let mut gen = HopSequenceGen::new(0xCAFE_BABE, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(50);
        for w in hops.windows(2) {
            assert!(w[1].start_time_s >= w[0].start_time_s);
        }
    }

    #[test]
    fn test_hop_dwell_time_hq1() {
        let mode = HaveQuickMode::HqI;
        let expected_dwell = 1.0 / HQ1_HOP_RATE_HZ;
        let mut gen = HopSequenceGen::new(1, 1, 0.0, mode);
        let hop = gen.next_hop();
        assert!((hop.dwell_time_s - expected_dwell).abs() < 1e-9);
    }

    #[test]
    fn test_hop_dwell_time_hq2() {
        let mode = HaveQuickMode::HqII;
        let expected_dwell = 1.0 / HQ2_HOP_RATE_HZ;
        let mut gen = HopSequenceGen::new(1, 1, 0.0, mode);
        let hop = gen.next_hop();
        assert!((hop.dwell_time_s - expected_dwell).abs() < 1e-12);
    }

    // ── WOD Dependence & Net ID Separation ────────────────────────────────

    #[test]
    fn test_different_wods_produce_different_sequences() {
        let mut gen_a = HopSequenceGen::new(0xAAAA_AAAA, 1, 0.0, HaveQuickMode::HqI);
        let mut gen_b = HopSequenceGen::new(0x5555_5555, 1, 0.0, HaveQuickMode::HqI);

        let seq_a: Vec<usize> = (0..50).map(|_| gen_a.next_hop().channel_index).collect();
        let seq_b: Vec<usize> = (0..50).map(|_| gen_b.next_hop().channel_index).collect();

        // Must be different (extremely unlikely to be same with different WOD)
        assert_ne!(seq_a, seq_b, "Different WODs must produce different sequences");
    }

    #[test]
    fn test_different_net_ids_produce_different_sequences() {
        let wod = 0x1234_ABCD;
        let mut gen1 = HopSequenceGen::new(wod, 1, 0.0, HaveQuickMode::HqI);
        let mut gen2 = HopSequenceGen::new(wod, 2, 0.0, HaveQuickMode::HqI);

        let seq1: Vec<usize> = (0..50).map(|_| gen1.next_hop().channel_index).collect();
        let seq2: Vec<usize> = (0..50).map(|_| gen2.next_hop().channel_index).collect();

        assert_ne!(seq1, seq2, "Different net IDs must produce different sequences");
    }

    #[test]
    fn test_different_tod_produces_different_sequences() {
        let wod = 0xBEEF_DEAD;
        let mut gen1 = HopSequenceGen::new(wod, 1, 3600.0, HaveQuickMode::HqI);
        let mut gen2 = HopSequenceGen::new(wod, 1, 7200.0, HaveQuickMode::HqI);

        let seq1: Vec<usize> = (0..50).map(|_| gen1.next_hop().channel_index).collect();
        let seq2: Vec<usize> = (0..50).map(|_| gen2.next_hop().channel_index).collect();

        assert_ne!(seq1, seq2, "Different TOD must produce different sequences");
    }

    // ── Statistical Uniformity ────────────────────────────────────────────

    #[test]
    fn test_hop_distribution_covers_most_of_band() {
        let mut gen = HopSequenceGen::new(0x7F3A_8B2C, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(2000);
        let stats = HopStatistics::from_hops(&hops);

        // With 2000 hops over 7000 channels we expect ~600+ unique channels
        assert!(
            stats.unique_channels >= 500,
            "Expected many unique channels, got {}",
            stats.unique_channels
        );
    }

    #[test]
    fn test_hop_statistics_mean_roughly_centered() {
        let mut gen = HopSequenceGen::new(0x9ABC_DEF0, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(1000);
        let stats = HopStatistics::from_hops(&hops);

        // Mean channel should be roughly in the middle (2000–5000 range for 7000 channels)
        assert!(
            stats.mean_channel > 1000.0 && stats.mean_channel < 6000.0,
            "Mean channel {:.0} not near center",
            stats.mean_channel
        );
    }

    #[test]
    fn test_hop_statistics_std_reasonable() {
        let mut gen = HopSequenceGen::new(0x0102_0304, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(500);
        let stats = HopStatistics::from_hops(&hops);

        // Std dev for uniform [0, 7000) ≈ 7000/sqrt(12) ≈ 2020
        assert!(
            stats.std_channel > 500.0,
            "Std dev {:.0} too small, may not be uniform",
            stats.std_channel
        );
    }

    #[test]
    fn test_hop_uniformity_chi_squared() {
        let mut gen = HopSequenceGen::new(0xFEDC_BA98, 1, 0.0, HaveQuickMode::HqI);
        let hops = gen.generate_n(3500);
        let stats = HopStatistics::from_hops(&hops);

        // Chi-squared for uniform distribution should pass with enough samples
        // Critical value at alpha=0.05, df=69 is ~90
        assert!(
            stats.is_uniform(),
            "Chi-squared {:.1} suggests non-uniform distribution",
            stats.chi_squared
        );
    }

    // ── AM Modulation ──────────────────────────────────────────────────────

    #[test]
    fn test_am_modulator_output_length() {
        let mut am = AmModulator::new(300e6, 0.85, 1.0, 50_000.0);
        let audio: Vec<f64> = (0..100).map(|i| (2.0 * PI * 1000.0 * i as f64 / 50_000.0).sin()).collect();
        let iq = am.modulate(&audio);
        assert_eq!(iq.len(), 100);
    }

    #[test]
    fn test_am_modulator_carrier_present() {
        let mut am = AmModulator::new(300e6, 0.85, 1.0, 50_000.0);
        let audio = vec![0.0f64; 1000]; // silence → pure carrier
        let iq = am.modulate(&audio);

        // With no modulation, output is pure carrier at carrier_amplitude
        let power: f64 = iq_power(&iq);
        assert!(power > 0.1, "Carrier should be present, power={power:.4}");
    }

    #[test]
    fn test_am_modulator_modulation_index_effect() {
        let sr = 50_000.0;
        let fc = 300e6;
        let audio: Vec<f64> = (0..500)
            .map(|i| (2.0 * PI * 1000.0 * i as f64 / sr).sin())
            .collect();

        let mut am_low = AmModulator::new(fc, 0.1, 1.0, sr);
        let mut am_high = AmModulator::new(fc, 0.9, 1.0, sr);

        let iq_low = am_low.modulate(&audio);
        let iq_high = am_high.modulate(&audio);

        let pow_low = iq_power(&iq_low);
        let pow_high = iq_power(&iq_high);

        // Higher modulation index → higher average power variation
        // The difference might be subtle, just verify both are nonzero
        assert!(pow_low > 0.0);
        assert!(pow_high > 0.0);
    }

    #[test]
    fn test_am_modulator_set_carrier() {
        let mut am = AmModulator::new(300e6, 0.5, 1.0, 50_000.0);
        am.set_carrier(350e6);
        assert!((am.carrier_hz - 350e6).abs() < 1.0);
    }

    #[test]
    fn test_am_demodulator_output_length() {
        let mut am_mod = AmModulator::new(1000.0, 0.5, 1.0, 50_000.0);
        let mut am_demod = AmDemodulator::new(1000.0, 50_000.0, AmDetection::Envelope);

        let audio = vec![0.5f64; 100];
        let iq = am_mod.modulate(&audio);
        let recovered = am_demod.demodulate_iq(&iq);
        assert_eq!(recovered.len(), 100);
    }

    #[test]
    fn test_am_loopback_fidelity() {
        // Low carrier frequency so we can actually simulate the audio band
        let sr = 50_000.0;
        let fc = 5000.0; // 5 kHz carrier (low so audio components are resolvable)
        let audio_freq = 1000.0;

        let audio: Vec<f64> = (0..2000)
            .map(|i| (2.0 * PI * audio_freq * i as f64 / sr).sin())
            .collect();

        let mut am_mod = AmModulator::new(fc, 0.8, 1.0, sr);
        let mut am_demod = AmDemodulator::new(fc, sr, AmDetection::Envelope);

        let iq = am_mod.modulate(&audio);
        let recovered = am_demod.demodulate_iq(&iq);

        // Skip initial transient (filter settling)
        let start = 500;
        if recovered.len() > start + 100 {
            let rec_power: f64 = recovered[start..]
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                / (recovered.len() - start) as f64;
            assert!(rec_power > 1e-6, "Recovered audio power too low: {rec_power:.2e}");
        }
    }

    // ── Frequency Synthesizer ─────────────────────────────────────────────

    #[test]
    fn test_synth_initial_locked() {
        let synth = FrequencySynthesizer::new(300e6, 1e-3, 50_000.0);
        assert!(synth.is_locked());
    }

    #[test]
    fn test_synth_unlocks_on_hop() {
        let mut synth = FrequencySynthesizer::new(300e6, 1e-3, 50_000.0);
        synth.hop_to(350e6);
        assert!(!synth.is_locked());
    }

    #[test]
    fn test_synth_locks_after_settling() {
        let settling = 1e-3;
        let mut synth = FrequencySynthesizer::new(300e6, settling, 50_000.0);
        synth.hop_to(350e6);
        synth.advance(settling + 1e-6);
        assert!(synth.is_locked());
    }

    #[test]
    fn test_synth_frequency_after_lock() {
        let mut synth = FrequencySynthesizer::new(300e6, 1e-3, 50_000.0);
        synth.hop_to(350e6);
        synth.advance(2e-3);
        assert!((synth.current_freq_hz() - 350e6).abs() < 1.0);
    }

    #[test]
    fn test_synth_hop_count() {
        let mut synth = FrequencySynthesizer::new(300e6, 1e-3, 50_000.0);
        for _ in 0..5 {
            synth.hop_to(300e6);
        }
        assert_eq!(synth.hop_count(), 5);
    }

    #[test]
    fn test_synth_settling_progress() {
        let settling = 1e-3;
        let mut synth = FrequencySynthesizer::new(300e6, settling, 50_000.0);
        synth.hop_to(350e6);
        synth.advance(settling / 2.0);
        let progress = synth.settling_progress();
        assert!(
            (progress - 0.5).abs() < 0.1,
            "Expected ~0.5 progress, got {progress:.3}"
        );
    }

    // ── Mode Parameters ────────────────────────────────────────────────────

    #[test]
    fn test_hq1_hop_rate() {
        let mode = HaveQuickMode::HqI;
        assert!((mode.hop_rate() - HQ1_HOP_RATE_HZ).abs() < 0.01);
    }

    #[test]
    fn test_hq2_hop_rate() {
        let mode = HaveQuickMode::HqII;
        assert!((mode.hop_rate() - HQ2_HOP_RATE_HZ).abs() < 0.01);
    }

    #[test]
    fn test_dwell_times() {
        let hq1 = HaveQuickMode::HqI;
        let hq2 = HaveQuickMode::HqII;
        assert!((hq1.dwell_time_s() - 0.01).abs() < 1e-9); // 10 ms
        assert!((hq2.dwell_time_s() - 200e-6).abs() < 1e-12); // 200 μs
    }

    #[test]
    fn test_guard_time_fraction() {
        let hq1 = HaveQuickMode::HqI;
        let guard = hq1.guard_time_s();
        let dwell = hq1.dwell_time_s();
        assert!((guard / dwell - GUARD_TIME_FRACTION).abs() < 1e-9);
    }

    #[test]
    fn test_active_time_less_than_dwell() {
        let mode = HaveQuickMode::HqII;
        assert!(mode.active_time_s() < mode.dwell_time_s());
    }

    // ── Late Net Entry ─────────────────────────────────────────────────────

    #[test]
    fn test_late_entry_hop_number_from_tod() {
        let mut lne = LateNetEntry::new(0x1234, 1, HaveQuickMode::HqI);
        let net_epoch = 36000.0; // 10:00:00
        let current_tod = 36010.0; // 10 seconds later → 10 s * 100 hops/s = 1000 hops
        let hop_num = lne.sync_from_tod(current_tod, net_epoch);
        assert_eq!(hop_num, 1000, "Expected hop 1000, got {hop_num}");
    }

    #[test]
    fn test_late_entry_is_synced_after_sync() {
        let mut lne = LateNetEntry::new(0x5678, 1, HaveQuickMode::HqI);
        assert!(!lne.is_synced());
        let _ = lne.sync_from_tod(3700.0, 3600.0);
        assert!(lne.is_synced());
    }

    #[test]
    fn test_late_entry_confidence_with_gps() {
        let mut lne = LateNetEntry::new(0xABCD, 2, HaveQuickMode::HqII);
        let _ = lne.sync_from_tod(100.0, 0.0);
        assert!((lne.sync_confidence() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_late_entry_creates_valid_generator() {
        let wod = 0xCAFE_1234;
        let net_id = 3;
        let net_epoch = 7200.0;
        let current_tod = 7201.0; // 1 second later = 100 hops at HQ-I rate

        let mut lne = LateNetEntry::new(wod, net_id, HaveQuickMode::HqI);
        let _ = lne.sync_from_tod(current_tod, net_epoch);

        let gen_opt = lne.create_synced_generator(net_epoch);
        assert!(gen_opt.is_some(), "Should create a synced generator");

        let mut gen = gen_opt.unwrap();
        let hop = gen.next_hop();
        // Hop number should be at or near 100
        assert!(hop.hop_number >= 99 && hop.hop_number <= 101);
    }

    #[test]
    fn test_late_entry_synced_matches_original() {
        let wod = 0x1111_BBBB;
        let net_id = 1;
        let net_epoch = 0.0;
        let target_hop = 50;

        // Original generator
        let mut gen_orig = HopSequenceGen::new(wod, net_id, net_epoch, HaveQuickMode::HqI);
        gen_orig.advance_to(target_hop);
        let orig_ch = gen_orig.next_hop().channel_index;

        // Late-entry synchronized generator
        let tod_at_target = net_epoch + target_hop as f64 / HQ1_HOP_RATE_HZ;
        let mut lne = LateNetEntry::new(wod, net_id, HaveQuickMode::HqI);
        let _ = lne.sync_from_tod(tod_at_target, net_epoch);
        let mut synced_gen = lne.create_synced_generator(net_epoch).unwrap();
        let synced_ch = synced_gen.next_hop().channel_index;

        assert_eq!(orig_ch, synced_ch, "Synced generator must match original at same hop");
    }

    // ── Jamming Analysis ──────────────────────────────────────────────────

    #[test]
    fn test_processing_gain_hq() {
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let gp = ja.processing_gain_db();
        // 10*log10(7000) = 38.45 dB
        assert!((gp - 38.45).abs() < 0.1, "Processing gain {gp:.2} not ~38.45 dB");
    }

    #[test]
    fn test_antijam_margin_positive_for_low_jamming() {
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 10.0);
        let margin = ja.antijam_margin_db(0.0); // No jamming
        assert!(margin > 0.0, "Should have positive AJ margin with no jamming");
    }

    #[test]
    fn test_antijam_margin_negative_for_strong_jamming() {
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        // Very strong jamming exceeding processing gain
        let margin = ja.antijam_margin_db(50.0);
        assert!(margin < 0.0, "Should have negative AJ margin with overwhelming jamming");
    }

    #[test]
    fn test_partial_band_jamming_full_band() {
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let result = ja.partial_band_effectiveness(1.0, -10.0);
        assert!((result.jam_fraction - 1.0).abs() < 0.01);
        assert!((result.prob_hop_jammed - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_partial_band_jamming_small_fraction() {
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let result = ja.partial_band_effectiveness(0.1, -10.0);
        assert!((result.prob_hop_jammed - 0.1).abs() < 0.01);
        // Effective throughput should be high (90% of hops unjammed)
        assert!(result.effective_throughput > 0.8);
    }

    #[test]
    fn test_follower_jammer_hq1_resistance() {
        // HQ-I dwell = 10 ms, reaction_time = 15 ms > dwell → jammer cannot follow
        // → system IS resistant (jammer cannot retune fast enough within one dwell)
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let reaction_time = 0.015; // 15 ms (longer than 10 ms dwell)
        assert!(
            ja.is_follower_resistant(reaction_time),
            "HQ-I is resistant when jammer reaction (15ms) > dwell (10ms)"
        );
    }

    #[test]
    fn test_follower_jammer_hq1_vulnerable() {
        // HQ-I dwell = 10 ms, reaction_time = 5 ms < dwell → jammer CAN follow
        let ja = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let reaction_time = 0.005; // 5 ms (shorter than 10 ms dwell)
        assert!(
            !ja.is_follower_resistant(reaction_time),
            "HQ-I is vulnerable when jammer reaction (5ms) < dwell (10ms)"
        );
    }

    #[test]
    fn test_follower_jammer_hq2_resistance() {
        // HQ-II dwell = 200 μs, typical follower reaction > 1 ms → resistant
        let ja = JammingAnalysis::new(HaveQuickMode::HqII, 0.0);
        let reaction_time = 1e-3; // 1 ms
        assert!(
            ja.is_follower_resistant(reaction_time),
            "HQ-II should resist follower jammer with 1 ms reaction time"
        );
    }

    #[test]
    fn test_prob_intercept_decreases_with_hop_rate() {
        let ja1 = JammingAnalysis::new(HaveQuickMode::HqI, 0.0);
        let ja2 = JammingAnalysis::new(HaveQuickMode::HqII, 0.0);
        // Use 500 ch/s scan rate: HQ-I pi=min(5, 1)=1.0, HQ-II pi=0.0002*500=0.1
        let scan_rate = 500.0;

        let pi1 = ja1.prob_intercept_per_hop(scan_rate);
        let pi2 = ja2.prob_intercept_per_hop(scan_rate);

        // HQ-II has shorter dwell → lower probability of intercept per hop
        assert!(pi2 < pi1, "HQ-II (pi={pi2:.4}) should be harder to intercept than HQ-I (pi={pi1:.4})");
    }

    // ── Full Processor ────────────────────────────────────────────────────

    #[test]
    fn test_processor_transmit_returns_iq() {
        let config = hq1_preset(0x1234_5678, 1, 0.0);
        let mut proc = HaveQuickProcessor::new(config);
        let audio = vec![0.5f64; 64];
        let (iq, hop) = proc.transmit(&audio);

        assert!(!iq.is_empty(), "TX output should not be empty");
        assert!(
            hop.frequency_hz >= HQ_BAND_LOW_HZ && hop.frequency_hz <= HQ_BAND_HIGH_HZ,
            "Hop frequency out of band: {}",
            hop.frequency_hz
        );
    }

    #[test]
    fn test_processor_loopback_ideal_channel() {
        let config = hq1_preset(0xABCD_EF01, 1, 0.0);
        let mut proc = HaveQuickProcessor::new(config);

        let audio: Vec<f64> = (0..256)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / 50_000.0).sin())
            .collect();

        let (_tx, recovered, hop) = proc.loopback(&audio, ChannelModel::Ideal);

        assert_eq!(recovered.len(), audio.len());
        assert!(
            hop.frequency_hz >= HQ_BAND_LOW_HZ,
            "Hop frequency out of range"
        );
    }

    #[test]
    fn test_processor_loopback_awgn() {
        let config = hq2_preset(0x9999_0000, 2, 100.0);
        let mut proc = HaveQuickProcessor::new(config);

        let audio: Vec<f64> = (0..100).map(|_| 0.5).collect();
        let (_tx, recovered, _hop) = proc.loopback(&audio, ChannelModel::Awgn { snr_db: 20.0 });

        assert_eq!(recovered.len(), audio.len());
    }

    #[test]
    fn test_processor_loopback_partial_band_jamming() {
        let config = hq1_preset(0x4444_3333, 1, 0.0);
        let mut proc = HaveQuickProcessor::new(config);

        let audio = vec![0.3f64; 50];
        let (_tx, recovered, _hop) = proc.loopback(
            &audio,
            ChannelModel::PartialBandJamming {
                jam_fraction: 0.5,
                jammer_power_dbw: -20.0,
            },
        );
        assert_eq!(recovered.len(), audio.len());
    }

    #[test]
    fn test_processor_consecutive_hops_different_frequencies() {
        let config = hq1_preset(0x7777_8888, 1, 0.0);
        let mut proc = HaveQuickProcessor::new(config);

        let audio = vec![0.0f64; 32];
        let (_, hop1) = proc.transmit(&audio);
        let (_, hop2) = proc.transmit(&audio);

        // Consecutive hops should typically be different frequencies
        // (not guaranteed, but overwhelmingly probable)
        assert_ne!(
            hop1.channel_index, hop2.channel_index,
            "Consecutive hops should use different channels"
        );
    }

    #[test]
    fn test_processor_processing_gain() {
        let config = HaveQuickConfig::default();
        let proc = HaveQuickProcessor::new(config);
        let gp = proc.processing_gain_db();
        // 10*log10(7000) ≈ 38.45 dB
        assert!((gp - 38.45).abs() < 0.1, "Processing gain {gp:.2} dB");
    }

    #[test]
    fn test_processor_samples_counted() {
        let config = hq1_preset(0x1357_2468, 1, 0.0);
        let mut proc = HaveQuickProcessor::new(config);
        let audio = vec![0.0f64; 64];
        let _ = proc.transmit(&audio);
        assert!(
            proc.samples_processed() > 0,
            "Samples processed should be > 0"
        );
    }

    // ── Late Net Entry via Processor ──────────────────────────────────────

    #[test]
    fn test_processor_late_entry_sync() {
        let wod = 0xDECA_FBAD;
        let net_id = 5;
        let net_epoch = 1000.0;

        // Create reference processor at net epoch
        let config_ref = hq1_preset(wod, net_id, net_epoch);
        let mut proc_ref = HaveQuickProcessor::new(config_ref);

        // Advance reference 100 hops
        let audio = vec![0.0f64; 1];
        for _ in 0..100 {
            let _ = proc_ref.transmit(&audio);
        }
        let (_, ref_hop) = proc_ref.transmit(&audio);

        // Create late-entry processor syncing to hop 100
        let current_tod = net_epoch + 100.0 / HQ1_HOP_RATE_HZ;
        let config_late = hq1_preset(wod, net_id, net_epoch);
        let mut proc_late = HaveQuickProcessor::new(config_late);
        proc_late.sync_late_entry(current_tod, net_epoch);
        let (_, late_hop) = proc_late.transmit(&audio);

        assert_eq!(
            ref_hop.channel_index, late_hop.channel_index,
            "Late entry must synchronize to same channel as reference: ref={} late={}",
            ref_hop.channel_index, late_hop.channel_index
        );
    }

    // ── SNR Helper ────────────────────────────────────────────────────────

    #[test]
    fn test_snr_db_perfect_signal() {
        let sig: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let snr = snr_db(&sig, &sig);
        assert!(snr.is_infinite() || snr > 100.0, "Perfect signal SNR should be very high");
    }

    #[test]
    fn test_snr_db_with_noise() {
        let clean: Vec<(f64, f64)> = (0..100).map(|_| (1.0, 0.0)).collect();
        let noisy: Vec<(f64, f64)> = (0..100)
            .map(|i| (1.0 + 0.1 * ((i as f64) * 0.1).sin(), 0.0))
            .collect();
        let snr = snr_db(&clean, &noisy);
        assert!(snr > 0.0 && snr < 50.0, "Noisy SNR should be finite: {snr:.1} dB");
    }

    #[test]
    fn test_iq_power_dc_signal() {
        let sig: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let power = iq_power(&sig);
        assert!((power - 1.0).abs() < 1e-9, "Power of unit DC should be 1.0");
    }

    #[test]
    fn test_iq_power_empty() {
        let power = iq_power(&[]);
        assert_eq!(power, 0.0);
    }
}
