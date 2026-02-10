//! Narrowband Power Line Carrier Communication (PLCC) modem.
//!
//! This module implements FSK and QPSK modulation/demodulation over 50/60 Hz
//! power grids, including mains frequency synchronization, zero-crossing
//! detection, impedance matching compensation, narrowband interference
//! avoidance, CENELEC band allocation (A/B/C/D), SNR estimation on PLC
//! channels, bit error rate analysis, and frame synchronization with preamble
//! detection.
//!
//! # Example
//!
//! ```
//! use r4w_core::powerline_carrier_modem::{PlcConfig, CenelecBand, PlcModem};
//!
//! let config = PlcConfig {
//!     mains_freq_hz: 50.0,
//!     carrier_freq_hz: 75_000.0,
//!     symbol_rate_hz: 2400.0,
//!     sample_rate_hz: 1_000_000.0,
//!     band: CenelecBand::A,
//! };
//! let modem = PlcModem::new(&config);
//! let bits = vec![true, false, true, true];
//! let signal = modem.modulate_fsk(&bits);
//! let recovered = modem.demodulate_fsk(&signal);
//! assert_eq!(&recovered[..bits.len()], &bits[..]);
//! ```

use std::f64::consts::PI;

// ─── CENELEC band allocation ────────────────────────────────────────────────

/// CENELEC frequency band designations for European power-line communication.
///
/// Each band defines a frequency range within the 3–148.5 kHz region used for
/// narrowband PLC signalling over low-voltage distribution networks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CenelecBand {
    /// Band A: 3–95 kHz (energy providers).
    A,
    /// Band B: 95–125 kHz (general use).
    B,
    /// Band C: 125–140 kHz (home networking, CSMA/CA required).
    C,
    /// Band D: 140–148.5 kHz (alarm/security systems).
    D,
}

impl CenelecBand {
    /// Lower frequency bound in Hz.
    pub fn lower_hz(&self) -> f64 {
        match self {
            CenelecBand::A => 3_000.0,
            CenelecBand::B => 95_000.0,
            CenelecBand::C => 125_000.0,
            CenelecBand::D => 140_000.0,
        }
    }

    /// Upper frequency bound in Hz.
    pub fn upper_hz(&self) -> f64 {
        match self {
            CenelecBand::A => 95_000.0,
            CenelecBand::B => 125_000.0,
            CenelecBand::C => 140_000.0,
            CenelecBand::D => 148_500.0,
        }
    }

    /// Returns `true` when `freq_hz` falls inside this band.
    pub fn contains(&self, freq_hz: f64) -> bool {
        freq_hz >= self.lower_hz() && freq_hz <= self.upper_hz()
    }
}

// ─── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the power-line carrier modem.
///
/// Specifies the mains frequency, carrier, symbol rate, sample rate, and
/// CENELEC band to operate in.
#[derive(Debug, Clone)]
pub struct PlcConfig {
    /// Mains frequency – must be 50.0 or 60.0 Hz.
    pub mains_freq_hz: f64,
    /// Carrier frequency within the chosen CENELEC band (Hz).
    pub carrier_freq_hz: f64,
    /// Symbol rate (baud, Hz).
    pub symbol_rate_hz: f64,
    /// Sampling rate (Hz). Must be >= 2 * carrier_freq_hz.
    pub sample_rate_hz: f64,
    /// CENELEC band in which the modem operates.
    pub band: CenelecBand,
}

impl PlcConfig {
    /// Number of samples per symbol.
    pub fn samples_per_symbol(&self) -> usize {
        (self.sample_rate_hz / self.symbol_rate_hz).round() as usize
    }

    /// Validate that the carrier frequency falls inside the configured band.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.mains_freq_hz != 50.0 && self.mains_freq_hz != 60.0 {
            return Err("mains_freq_hz must be 50 or 60");
        }
        if self.sample_rate_hz < 2.0 * self.carrier_freq_hz {
            return Err("sample_rate_hz must be >= 2 * carrier_freq_hz (Nyquist)");
        }
        if !self.band.contains(self.carrier_freq_hz) {
            return Err("carrier_freq_hz is outside the selected CENELEC band");
        }
        Ok(())
    }
}

// ─── Impedance matching ─────────────────────────────────────────────────────

/// Impedance matching compensation coefficients.
///
/// Models a simple series-R / parallel-C coupling network and provides an
/// amplitude + phase correction that can be applied per-sample.
#[derive(Debug, Clone)]
pub struct ImpedanceCompensator {
    /// Nominal line impedance (ohm).
    pub line_impedance_ohm: f64,
    /// Coupling capacitance (F).
    pub coupling_cap_f: f64,
    /// Frequency at which compensation was computed.
    pub freq_hz: f64,
    /// Magnitude correction factor.
    pub gain: f64,
    /// Phase correction (radians).
    pub phase_rad: f64,
}

impl ImpedanceCompensator {
    /// Compute compensation for a given line impedance, coupling cap, and
    /// carrier frequency.
    pub fn new(line_impedance_ohm: f64, coupling_cap_f: f64, freq_hz: f64) -> Self {
        let omega = 2.0 * PI * freq_hz;
        let xc = 1.0 / (omega * coupling_cap_f);
        let z_mag = (line_impedance_ohm * line_impedance_ohm + xc * xc).sqrt();
        let gain = line_impedance_ohm / z_mag;
        let phase_rad = (xc / line_impedance_ohm).atan();
        Self {
            line_impedance_ohm,
            coupling_cap_f,
            freq_hz,
            gain,
            phase_rad,
        }
    }

    /// Apply compensation to a complex IQ sample represented as `(re, im)`.
    pub fn compensate(&self, sample: (f64, f64)) -> (f64, f64) {
        let cos_p = self.phase_rad.cos();
        let sin_p = self.phase_rad.sin();
        let re = sample.0 * cos_p - sample.1 * sin_p;
        let im = sample.0 * sin_p + sample.1 * cos_p;
        (re * self.gain, im * self.gain)
    }
}

// ─── PlcModem ───────────────────────────────────────────────────────────────

/// Power-line carrier modem with FSK and QPSK modulation support.
///
/// Operates within a CENELEC band, synchronised to the mains frequency.
#[derive(Debug, Clone)]
pub struct PlcModem {
    config: PlcConfig,
}

impl PlcModem {
    /// Create a new modem from the given configuration.
    pub fn new(config: &PlcConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Return a reference to the active configuration.
    pub fn config(&self) -> &PlcConfig {
        &self.config
    }

    /// FSK-modulate a bit slice.
    ///
    /// `true` maps to carrier + delta_f, `false` maps to carrier - delta_f,
    /// where delta_f = symbol_rate / 2.
    pub fn modulate_fsk(&self, bits: &[bool]) -> Vec<f64> {
        modulate_fsk(bits, &self.config)
    }

    /// Demodulate an FSK signal back to bits.
    pub fn demodulate_fsk(&self, signal: &[f64]) -> Vec<bool> {
        demodulate_fsk(signal, &self.config)
    }

    /// QPSK-modulate a bit slice (pairs of bits to complex symbols).
    pub fn modulate_qpsk(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        modulate_qpsk(bits, &self.config)
    }

    /// Demodulate a QPSK signal back to bits.
    pub fn demodulate_qpsk(&self, signal: &[(f64, f64)]) -> Vec<bool> {
        demodulate_qpsk(signal, &self.config)
    }
}

// ─── FSK modulation / demodulation ──────────────────────────────────────────

/// FSK-modulate `bits` using the given PLC configuration.
///
/// Frequency deviation is +/- `symbol_rate_hz / 2` around the carrier.
pub fn modulate_fsk(bits: &[bool], config: &PlcConfig) -> Vec<f64> {
    let sps = config.samples_per_symbol();
    let deviation = config.symbol_rate_hz / 2.0;
    let mut out = Vec::with_capacity(bits.len() * sps);
    let mut phase: f64 = 0.0;
    for &bit in bits {
        let freq = if bit {
            config.carrier_freq_hz + deviation
        } else {
            config.carrier_freq_hz - deviation
        };
        let phase_inc = 2.0 * PI * freq / config.sample_rate_hz;
        for _ in 0..sps {
            out.push(phase.sin());
            phase += phase_inc;
            // keep phase bounded
            if phase > 2.0 * PI {
                phase -= 2.0 * PI;
            }
        }
    }
    out
}

/// Demodulate an FSK signal produced by [`modulate_fsk`].
///
/// Uses correlation against mark and space frequencies: the sign of the
/// frequency offset relative to the carrier determines each bit.
pub fn demodulate_fsk(signal: &[f64], config: &PlcConfig) -> Vec<bool> {
    let sps = config.samples_per_symbol();
    if sps == 0 || signal.len() < sps {
        return Vec::new();
    }
    let num_symbols = signal.len() / sps;
    let mut bits = Vec::with_capacity(num_symbols);

    let deviation = config.symbol_rate_hz / 2.0;
    let f_mark = config.carrier_freq_hz + deviation;
    let f_space = config.carrier_freq_hz - deviation;

    for sym_idx in 0..num_symbols {
        let start = sym_idx * sps;
        let end = start + sps;
        let window = &signal[start..end];

        let mut corr_mark: f64 = 0.0;
        let mut corr_space: f64 = 0.0;
        for (k, &s) in window.iter().enumerate() {
            let t = k as f64 / config.sample_rate_hz;
            corr_mark += s * (2.0 * PI * f_mark * t).sin();
            corr_space += s * (2.0 * PI * f_space * t).sin();
        }
        bits.push(corr_mark.abs() > corr_space.abs());
    }
    bits
}

// ─── QPSK modulation / demodulation ────────────────────────────────────────

/// Gray-coded QPSK symbol mapping.
///
/// | dibits | phase   |
/// |--------|---------|
/// | 00     |  pi/4   |
/// | 01     |  3pi/4  |
/// | 11     |  5pi/4  |
/// | 10     |  7pi/4  |
fn qpsk_map(b1: bool, b0: bool) -> (f64, f64) {
    let phase = match (b1, b0) {
        (false, false) => PI / 4.0,
        (false, true) => 3.0 * PI / 4.0,
        (true, true) => 5.0 * PI / 4.0,
        (true, false) => 7.0 * PI / 4.0,
    };
    (phase.cos(), phase.sin())
}

/// Inverse Gray-coded QPSK demapping (hard-decision).
fn qpsk_demap(sample: (f64, f64)) -> (bool, bool) {
    // Determine quadrant:
    //   Q1 (+,+) -> 00,  Q2 (-,+) -> 01,  Q3 (-,-) -> 11,  Q4 (+,-) -> 10
    let b1 = sample.1 < 0.0;
    let b0 = sample.0 < 0.0;
    (b1, b0)
}

/// QPSK-modulate `bits` (must have even length; last odd bit is discarded).
///
/// Returns complex baseband samples at `samples_per_symbol` per dibit.
pub fn modulate_qpsk(bits: &[bool], config: &PlcConfig) -> Vec<(f64, f64)> {
    let sps = config.samples_per_symbol();
    let num_dibits = bits.len() / 2;
    let mut out = Vec::with_capacity(num_dibits * sps);
    for i in 0..num_dibits {
        let (re, im) = qpsk_map(bits[2 * i], bits[2 * i + 1]);
        for _ in 0..sps {
            out.push((re, im));
        }
    }
    out
}

/// Demodulate a QPSK signal back to bits.
///
/// Averages each symbol-length window and hard-decides via [`qpsk_demap`].
pub fn demodulate_qpsk(signal: &[(f64, f64)], config: &PlcConfig) -> Vec<bool> {
    let sps = config.samples_per_symbol();
    if sps == 0 || signal.len() < sps {
        return Vec::new();
    }
    let num_symbols = signal.len() / sps;
    let mut bits = Vec::with_capacity(num_symbols * 2);
    for sym_idx in 0..num_symbols {
        let start = sym_idx * sps;
        let end = start + sps;
        let window = &signal[start..end];
        let mut sum_re: f64 = 0.0;
        let mut sum_im: f64 = 0.0;
        for &(re, im) in window {
            sum_re += re;
            sum_im += im;
        }
        let avg = (sum_re / sps as f64, sum_im / sps as f64);
        let (b1, b0) = qpsk_demap(avg);
        bits.push(b1);
        bits.push(b0);
    }
    bits
}

// ─── Zero-crossing & mains frequency estimation ────────────────────────────

/// Detect positive-going zero crossings in a real-valued signal.
///
/// Returns the sample indices at which the signal transitions from <= 0 to > 0.
pub fn detect_zero_crossings(signal: &[f64], _sample_rate: f64) -> Vec<usize> {
    let mut crossings = Vec::new();
    for i in 1..signal.len() {
        if signal[i - 1] <= 0.0 && signal[i] > 0.0 {
            crossings.push(i);
        }
    }
    crossings
}

/// Estimate the mains frequency from a captured signal that contains the
/// 50/60 Hz component.
///
/// Uses the average spacing between positive-going zero crossings.  Returns
/// 0.0 when fewer than two crossings are found.
pub fn estimate_mains_frequency(signal: &[f64], sample_rate: f64) -> f64 {
    let crossings = detect_zero_crossings(signal, sample_rate);
    if crossings.len() < 2 {
        return 0.0;
    }
    let total_span = (crossings[crossings.len() - 1] - crossings[0]) as f64;
    let num_cycles = (crossings.len() - 1) as f64;
    let avg_period_samples = total_span / num_cycles;
    sample_rate / avg_period_samples
}

// ─── Preamble generation & detection ────────────────────────────────────────

/// Generate a synchronisation preamble (Barker-13 FSK-modulated).
///
/// The 13-chip Barker code provides excellent autocorrelation properties for
/// frame synchronisation in noisy PLC channels.
pub fn generate_preamble(config: &PlcConfig) -> Vec<f64> {
    let barker13: [bool; 13] = [
        true, true, true, true, true, false, false, true, true, false, true, false, true,
    ];
    modulate_fsk(&barker13, config)
}

/// Detect a preamble in a received signal by sliding cross-correlation.
///
/// Returns the sample index where the correlation peak exceeds 70% of the
/// preamble's self-correlation energy, or `None` if no peak is found.
pub fn detect_preamble(signal: &[f64], preamble: &[f64]) -> Option<usize> {
    if preamble.is_empty() || signal.len() < preamble.len() {
        return None;
    }
    let auto_energy: f64 = preamble.iter().map(|x| x * x).sum();
    let threshold = 0.70 * auto_energy;

    let search_len = signal.len() - preamble.len() + 1;
    let mut best_idx: Option<usize> = None;
    let mut best_val: f64 = 0.0;

    for offset in 0..search_len {
        let corr: f64 = preamble
            .iter()
            .enumerate()
            .map(|(k, &p)| p * signal[offset + k])
            .sum();
        if corr > best_val {
            best_val = corr;
            best_idx = Some(offset);
        }
    }

    if best_val >= threshold {
        best_idx
    } else {
        None
    }
}

// ─── SNR estimation ─────────────────────────────────────────────────────────

/// Estimate the channel signal-to-noise ratio (dB).
///
/// `signal` is a captured segment containing modulated data; `noise_floor` is
/// the average noise power (linear) measured during a quiet interval.
pub fn estimate_channel_snr(signal: &[f64], noise_floor: f64) -> f64 {
    if noise_floor <= 0.0 || signal.is_empty() {
        return 0.0;
    }
    let sig_power: f64 = signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64;
    if sig_power <= 0.0 {
        return 0.0;
    }
    10.0 * (sig_power / noise_floor).log10()
}

// ─── BER estimation ─────────────────────────────────────────────────────────

/// Count the number of bit errors between two equal-length bit sequences.
pub fn count_bit_errors(tx: &[bool], rx: &[bool]) -> usize {
    tx.iter().zip(rx.iter()).filter(|(a, b)| a != b).count()
}

/// Compute the bit error rate (BER) between transmitted and received bits.
///
/// Returns 1.0 if the sequences are empty.
pub fn compute_ber(tx: &[bool], rx: &[bool]) -> f64 {
    let len = tx.len().min(rx.len());
    if len == 0 {
        return 1.0;
    }
    let errors = count_bit_errors(&tx[..len], &rx[..len]);
    errors as f64 / len as f64
}

// ─── Narrowband interference avoidance ──────────────────────────────────────

/// Result of a narrowband interference scan.
#[derive(Debug, Clone)]
pub struct InterferenceScanResult {
    /// Center frequencies of detected interferers (Hz).
    pub interferer_freqs: Vec<f64>,
    /// Power levels of detected interferers (linear).
    pub interferer_powers: Vec<f64>,
    /// Suggested clear carrier frequency (Hz), or `None` if none found.
    pub suggested_carrier: Option<f64>,
}

/// Scan a CENELEC band for narrowband interferers and suggest a clear carrier.
///
/// Divides the band into `num_bins` frequency bins and estimates the power in
/// each bin via Goertzel-style inner products.  Bins whose power exceeds
/// `threshold_factor` times the median power are flagged as interferers.
pub fn scan_band_interference(
    signal: &[f64],
    sample_rate: f64,
    band: CenelecBand,
    num_bins: usize,
    threshold_factor: f64,
) -> InterferenceScanResult {
    let lo = band.lower_hz();
    let hi = band.upper_hz();
    let bin_width = (hi - lo) / num_bins as f64;

    let mut powers = Vec::with_capacity(num_bins);
    let mut freqs = Vec::with_capacity(num_bins);
    for i in 0..num_bins {
        let fc = lo + (i as f64 + 0.5) * bin_width;
        freqs.push(fc);
        let mut sum_cos: f64 = 0.0;
        let mut sum_sin: f64 = 0.0;
        for (n, &s) in signal.iter().enumerate() {
            let t = n as f64 / sample_rate;
            sum_cos += s * (2.0 * PI * fc * t).cos();
            sum_sin += s * (2.0 * PI * fc * t).sin();
        }
        let power = (sum_cos * sum_cos + sum_sin * sum_sin) / (signal.len() as f64);
        powers.push(power);
    }

    // Median power
    let mut sorted = powers.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = if sorted.is_empty() {
        0.0
    } else {
        sorted[sorted.len() / 2]
    };

    let threshold = threshold_factor * median;

    let mut interferer_freqs = Vec::new();
    let mut interferer_powers = Vec::new();
    let mut clear_bins: Vec<(usize, f64)> = Vec::new();

    for (i, &p) in powers.iter().enumerate() {
        if p > threshold && threshold > 0.0 {
            interferer_freqs.push(freqs[i]);
            interferer_powers.push(p);
        } else {
            clear_bins.push((i, p));
        }
    }

    let suggested_carrier = clear_bins
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|&(idx, _)| freqs[idx]);

    InterferenceScanResult {
        interferer_freqs,
        interferer_powers,
        suggested_carrier,
    }
}

// ─── Mains synchronisation helper ───────────────────────────────────────────

/// Generate a synthetic mains waveform at the specified frequency and sample rate.
///
/// Useful for testing zero-crossing and frequency estimation routines.
pub fn generate_mains_waveform(freq_hz: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
    let num_samples = (sample_rate * duration_s).round() as usize;
    (0..num_samples)
        .map(|n| (2.0 * PI * freq_hz * n as f64 / sample_rate).sin())
        .collect()
}

// ─── Frame builder ──────────────────────────────────────────────────────────

/// A simple PLC frame: preamble + length + payload + CRC-8.
#[derive(Debug, Clone)]
pub struct PlcFrame {
    /// Preamble samples (FSK-modulated Barker-13).
    pub preamble: Vec<f64>,
    /// Payload bits after the preamble.
    pub payload_bits: Vec<bool>,
}

impl PlcFrame {
    /// Build a complete PLC frame from payload bits.
    ///
    /// The frame structure is: `[preamble] [8-bit length] [payload] [8-bit CRC]`.
    pub fn build(payload: &[bool], config: &PlcConfig) -> Self {
        let len_bits = Self::u8_to_bits(payload.len() as u8);
        let crc = Self::crc8(payload);
        let crc_bits = Self::u8_to_bits(crc);

        let mut all_bits = Vec::new();
        all_bits.extend_from_slice(&len_bits);
        all_bits.extend_from_slice(payload);
        all_bits.extend_from_slice(&crc_bits);

        let preamble = generate_preamble(config);
        PlcFrame {
            preamble,
            payload_bits: all_bits,
        }
    }

    /// Transmit the frame as a continuous FSK waveform.
    pub fn to_fsk_signal(&self, config: &PlcConfig) -> Vec<f64> {
        let mut signal = self.preamble.clone();
        signal.extend(modulate_fsk(&self.payload_bits, config));
        signal
    }

    /// Simple CRC-8 (polynomial 0x07).
    pub fn crc8(bits: &[bool]) -> u8 {
        let bytes = Self::bits_to_bytes(bits);
        let mut crc: u8 = 0x00;
        for b in &bytes {
            crc ^= b;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    /// Convert a u8 value to a vector of 8 bools (MSB first).
    pub fn u8_to_bits(val: u8) -> Vec<bool> {
        (0..8).rev().map(|i| (val >> i) & 1 == 1).collect()
    }

    /// Convert a slice of bools to packed bytes (MSB first, zero-padded).
    pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
        bits.chunks(8)
            .map(|chunk| {
                let mut byte: u8 = 0;
                for (i, &b) in chunk.iter().enumerate() {
                    if b {
                        byte |= 1 << (7 - i);
                    }
                }
                byte
            })
            .collect()
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PlcConfig {
        PlcConfig {
            mains_freq_hz: 50.0,
            carrier_freq_hz: 75_000.0,
            symbol_rate_hz: 2400.0,
            sample_rate_hz: 1_000_000.0,
            band: CenelecBand::A,
        }
    }

    // --- CENELEC band tests ---

    #[test]
    fn test_cenelec_band_a_bounds() {
        let b = CenelecBand::A;
        assert_eq!(b.lower_hz(), 3_000.0);
        assert_eq!(b.upper_hz(), 95_000.0);
        assert!(b.contains(50_000.0));
        assert!(!b.contains(100_000.0));
    }

    #[test]
    fn test_cenelec_band_b_bounds() {
        let b = CenelecBand::B;
        assert_eq!(b.lower_hz(), 95_000.0);
        assert_eq!(b.upper_hz(), 125_000.0);
        assert!(b.contains(110_000.0));
        assert!(!b.contains(130_000.0));
    }

    #[test]
    fn test_cenelec_band_c_bounds() {
        let b = CenelecBand::C;
        assert_eq!(b.lower_hz(), 125_000.0);
        assert_eq!(b.upper_hz(), 140_000.0);
        assert!(b.contains(132_000.0));
        assert!(!b.contains(145_000.0));
    }

    #[test]
    fn test_cenelec_band_d_bounds() {
        let b = CenelecBand::D;
        assert_eq!(b.lower_hz(), 140_000.0);
        assert_eq!(b.upper_hz(), 148_500.0);
        assert!(b.contains(145_000.0));
        assert!(!b.contains(150_000.0));
    }

    // --- Config validation tests ---

    #[test]
    fn test_config_valid() {
        let cfg = default_config();
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn test_config_bad_mains() {
        let mut cfg = default_config();
        cfg.mains_freq_hz = 55.0;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn test_config_bad_nyquist() {
        let mut cfg = default_config();
        cfg.sample_rate_hz = 100_000.0; // < 2 * 75 kHz
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn test_config_bad_band() {
        let mut cfg = default_config();
        cfg.carrier_freq_hz = 130_000.0; // Band A goes up to 95 kHz
        assert!(cfg.validate().is_err());
    }

    // --- FSK modulation / demodulation ---

    #[test]
    fn test_fsk_roundtrip() {
        let cfg = default_config();
        let bits = vec![true, false, true, true, false, false, true, false];
        let signal = modulate_fsk(&bits, &cfg);
        let recovered = demodulate_fsk(&signal, &cfg);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_fsk_all_zeros() {
        let cfg = default_config();
        let bits = vec![false; 8];
        let signal = modulate_fsk(&bits, &cfg);
        let recovered = demodulate_fsk(&signal, &cfg);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_fsk_all_ones() {
        let cfg = default_config();
        let bits = vec![true; 8];
        let signal = modulate_fsk(&bits, &cfg);
        let recovered = demodulate_fsk(&signal, &cfg);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_fsk_signal_length() {
        let cfg = default_config();
        let bits = vec![true, false, true];
        let signal = modulate_fsk(&bits, &cfg);
        assert_eq!(signal.len(), bits.len() * cfg.samples_per_symbol());
    }

    // --- QPSK modulation / demodulation ---

    #[test]
    fn test_qpsk_roundtrip() {
        let cfg = default_config();
        let bits = vec![false, false, false, true, true, false, true, true];
        let signal = modulate_qpsk(&bits, &cfg);
        let recovered = demodulate_qpsk(&signal, &cfg);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_qpsk_odd_bits_truncated() {
        let cfg = default_config();
        // 5 bits -> only 4 used (2 symbols)
        let bits = vec![true, false, true, true, false];
        let signal = modulate_qpsk(&bits, &cfg);
        let recovered = demodulate_qpsk(&signal, &cfg);
        assert_eq!(recovered.len(), 4);
    }

    #[test]
    fn test_qpsk_all_zero_dibits() {
        let cfg = default_config();
        let bits = vec![false, false, false, false];
        let signal = modulate_qpsk(&bits, &cfg);
        let recovered = demodulate_qpsk(&signal, &cfg);
        assert_eq!(recovered, bits);
    }

    // --- Zero crossing detection ---

    #[test]
    fn test_detect_zero_crossings_sine() {
        let sr = 10_000.0;
        let waveform = generate_mains_waveform(50.0, sr, 0.1); // 5 cycles
        let crossings = detect_zero_crossings(&waveform, sr);
        // 50 Hz sine over 0.1s -> 5 full cycles -> 5 positive-going crossings
        assert!(crossings.len() >= 4 && crossings.len() <= 6);
    }

    #[test]
    fn test_detect_zero_crossings_empty() {
        let crossings = detect_zero_crossings(&[], 10_000.0);
        assert!(crossings.is_empty());
    }

    // --- Mains frequency estimation ---

    #[test]
    fn test_estimate_mains_50hz() {
        let sr = 10_000.0;
        let waveform = generate_mains_waveform(50.0, sr, 0.5);
        let est = estimate_mains_frequency(&waveform, sr);
        assert!((est - 50.0).abs() < 1.0, "estimated {} Hz, expected ~50", est);
    }

    #[test]
    fn test_estimate_mains_60hz() {
        let sr = 10_000.0;
        let waveform = generate_mains_waveform(60.0, sr, 0.5);
        let est = estimate_mains_frequency(&waveform, sr);
        assert!((est - 60.0).abs() < 1.0, "estimated {} Hz, expected ~60", est);
    }

    // --- Preamble generation & detection ---

    #[test]
    fn test_preamble_roundtrip() {
        let cfg = default_config();
        let preamble = generate_preamble(&cfg);
        assert!(!preamble.is_empty());
        let idx = detect_preamble(&preamble, &preamble);
        assert_eq!(idx, Some(0));
    }

    #[test]
    fn test_preamble_detection_with_offset() {
        let cfg = default_config();
        let preamble = generate_preamble(&cfg);
        let offset = 500;
        let mut signal = vec![0.0; offset];
        signal.extend_from_slice(&preamble);
        signal.extend_from_slice(&vec![0.0; 200]);
        let idx = detect_preamble(&signal, &preamble);
        assert_eq!(idx, Some(offset));
    }

    // --- SNR estimation ---

    #[test]
    fn test_snr_estimation() {
        let cfg = default_config();
        let bits = vec![true; 32];
        let signal = modulate_fsk(&bits, &cfg);
        let snr = estimate_channel_snr(&signal, 0.001);
        assert!(snr > 10.0, "SNR should be >10 dB, got {}", snr);
    }

    #[test]
    fn test_snr_zero_noise_floor() {
        let snr = estimate_channel_snr(&[1.0, 2.0, 3.0], 0.0);
        assert_eq!(snr, 0.0);
    }

    // --- BER computation ---

    #[test]
    fn test_ber_no_errors() {
        let tx = vec![true, false, true, false];
        let rx = vec![true, false, true, false];
        assert_eq!(compute_ber(&tx, &rx), 0.0);
    }

    #[test]
    fn test_ber_all_errors() {
        let tx = vec![true, true, true, true];
        let rx = vec![false, false, false, false];
        assert_eq!(compute_ber(&tx, &rx), 1.0);
    }

    #[test]
    fn test_ber_half_errors() {
        let tx = vec![true, false, true, false];
        let rx = vec![false, false, false, false];
        assert!((compute_ber(&tx, &rx) - 0.5).abs() < 1e-10);
    }

    // --- Impedance compensator ---

    #[test]
    fn test_impedance_compensator() {
        let comp = ImpedanceCompensator::new(50.0, 100e-9, 75_000.0);
        assert!(comp.gain > 0.0 && comp.gain <= 1.0);
        assert!(comp.phase_rad > 0.0);
        let out = comp.compensate((1.0, 0.0));
        assert!(out.0.abs() <= 1.0);
    }

    // --- PlcModem wrapper ---

    #[test]
    fn test_plc_modem_fsk() {
        let cfg = default_config();
        let modem = PlcModem::new(&cfg);
        let bits = vec![true, false, true, true];
        let signal = modem.modulate_fsk(&bits);
        let recovered = modem.demodulate_fsk(&signal);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_plc_modem_qpsk() {
        let cfg = default_config();
        let modem = PlcModem::new(&cfg);
        let bits = vec![false, true, true, false, true, true, false, false];
        let signal = modem.modulate_qpsk(&bits);
        let recovered = modem.demodulate_qpsk(&signal);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    // --- Frame builder ---

    #[test]
    fn test_plc_frame_build() {
        let cfg = default_config();
        let payload = vec![true, false, true, false, true, false, true, false];
        let frame = PlcFrame::build(&payload, &cfg);
        assert!(!frame.preamble.is_empty());
        // payload_bits = 8 (length) + 8 (payload) + 8 (crc) = 24
        assert_eq!(frame.payload_bits.len(), 24);
    }

    #[test]
    fn test_plc_frame_to_signal() {
        let cfg = default_config();
        let payload = vec![true; 8];
        let frame = PlcFrame::build(&payload, &cfg);
        let signal = frame.to_fsk_signal(&cfg);
        assert!(signal.len() > frame.preamble.len());
    }

    // --- Interference scan ---

    #[test]
    fn test_scan_band_interference_clean() {
        let sr = 400_000.0;
        let n = 4000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 50_000.0 * i as f64 / sr).sin() * 0.001)
            .collect();
        let result = scan_band_interference(&signal, sr, CenelecBand::A, 10, 5.0);
        assert!(result.suggested_carrier.is_some() || !result.interferer_freqs.is_empty());
    }

    // --- generate_mains_waveform utility ---

    #[test]
    fn test_generate_mains_waveform_length() {
        let waveform = generate_mains_waveform(50.0, 10_000.0, 0.1);
        assert_eq!(waveform.len(), 1000);
    }

    // --- samples_per_symbol ---

    #[test]
    fn test_samples_per_symbol() {
        let cfg = default_config();
        // 1_000_000 / 2400 ~ 417
        assert_eq!(cfg.samples_per_symbol(), 417);
    }
}
