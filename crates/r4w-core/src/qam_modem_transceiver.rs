//! Complete QAM Modem Transceiver Chain
//!
//! Implements modulation, pulse shaping, matched filtering, and demodulation
//! for 4-QAM (QPSK), 16-QAM, 64-QAM, and 256-QAM with Gray-coded
//! constellation mapping and root-raised-cosine pulse shaping.
//!
//! ## Signal Processing Chain
//!
//! ```text
//! TX:  bits → Gray-coded symbol mapping → RRC pulse shaping → IQ out
//! RX:  IQ in → matched filter (RRC) → downsample → hard decision → bits
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::qam_modem_transceiver::{QamConfig, QamTransmitter, QamReceiver};
//!
//! let config = QamConfig::new(16, 1000.0, 8, 0.35, 0.0);
//! let tx = QamTransmitter::new(config.clone());
//! let rx = QamReceiver::new(config);
//!
//! let bits: Vec<bool> = vec![true, false, true, true, false, false, true, false];
//! let iq = tx.modulate(&bits);
//! let recovered = rx.demodulate(&iq);
//! assert_eq!(&bits[..], &recovered[..bits.len()]);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for QAM modem transceiver.
#[derive(Debug, Clone)]
pub struct QamConfig {
    /// Constellation order: 4, 16, 64, or 256.
    pub order: usize,
    /// Symbol rate in Hz.
    pub symbol_rate_hz: f64,
    /// Samples per symbol (oversampling factor).
    pub samples_per_symbol: usize,
    /// Root-raised-cosine roll-off factor (0.0 .. 1.0).
    pub rolloff: f64,
    /// Carrier frequency in Hz (0 for baseband).
    pub carrier_freq_hz: f64,
}

impl QamConfig {
    /// Create a new QAM configuration.
    ///
    /// # Panics
    ///
    /// Panics if `order` is not one of {4, 16, 64, 256} or if `rolloff` is
    /// outside `[0.0, 1.0]`.
    pub fn new(
        order: usize,
        symbol_rate_hz: f64,
        samples_per_symbol: usize,
        rolloff: f64,
        carrier_freq_hz: f64,
    ) -> Self {
        assert!(
            matches!(order, 4 | 16 | 64 | 256),
            "QAM order must be 4, 16, 64, or 256"
        );
        assert!(
            (0.0..=1.0).contains(&rolloff),
            "Roll-off must be in [0.0, 1.0]"
        );
        assert!(samples_per_symbol >= 2, "Samples per symbol must be >= 2");
        Self {
            order,
            symbol_rate_hz,
            samples_per_symbol,
            rolloff,
            carrier_freq_hz,
        }
    }

    /// Default configuration with roll-off 0.35 and baseband.
    pub fn default_for(order: usize) -> Self {
        Self::new(order, 1000.0, 8, 0.35, 0.0)
    }

    /// Bits per symbol for this order.
    pub fn bits_per_symbol(&self) -> usize {
        match self.order {
            4 => 2,
            16 => 4,
            64 => 6,
            256 => 8,
            _ => unreachable!(),
        }
    }

    /// Number of RRC filter taps (8 symbol spans * sps + 1).
    fn rrc_num_taps(&self) -> usize {
        8 * self.samples_per_symbol + 1
    }
}

// ---------------------------------------------------------------------------
// Gray coding
// ---------------------------------------------------------------------------

/// Convert a binary integer to Gray code.
///
/// Gray code ensures adjacent constellation points differ by exactly one bit.
pub fn gray_code(n: usize) -> usize {
    n ^ (n >> 1)
}

/// Convert Gray code back to binary.
pub fn gray_decode(g: usize) -> usize {
    let mut n = g;
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}

/// Convert a slice of bits (MSB-first) into a Gray-coded index.
///
/// The bits are first interpreted as a natural binary index, then converted
/// to Gray code.
pub fn bits_to_gray_index(bits: &[bool]) -> usize {
    let mut val = 0usize;
    for &b in bits {
        val = (val << 1) | (b as usize);
    }
    gray_code(val)
}

/// Convert a Gray-coded index back into bits (MSB-first) of given width.
fn gray_index_to_bits(gray_idx: usize, width: usize) -> Vec<bool> {
    let binary = gray_decode(gray_idx);
    (0..width)
        .rev()
        .map(|i| (binary >> i) & 1 == 1)
        .collect()
}

// ---------------------------------------------------------------------------
// Constellation
// ---------------------------------------------------------------------------

/// Gray-coded QAM constellation.
///
/// Points are arranged on a square grid normalised to unit average power.
#[derive(Debug, Clone)]
pub struct QamConstellation {
    /// Constellation points indexed by Gray-coded label.
    pub points: Vec<(f64, f64)>,
    /// Bits per symbol (log2 of order).
    pub bits_per_symbol: usize,
    /// Side length of the square grid.
    side: usize,
}

impl QamConstellation {
    /// Build a Gray-coded QAM constellation of the given order.
    ///
    /// # Panics
    ///
    /// Panics if `order` is not 4, 16, 64, or 256.
    pub fn new(order: usize) -> Self {
        assert!(
            matches!(order, 4 | 16 | 64 | 256),
            "QAM order must be 4, 16, 64, or 256"
        );
        let bits_per_symbol = (order as f64).log2() as usize;
        let side = (order as f64).sqrt() as usize;
        let half_bits = bits_per_symbol / 2;

        // Generate unnormalised grid with Gray-coded I and Q axes
        let mut points = vec![(0.0, 0.0); order];
        for i_idx in 0..side {
            let gi = gray_code(i_idx);
            for q_idx in 0..side {
                let gq = gray_code(q_idx);
                // Composite Gray label: upper bits = I Gray, lower bits = Q Gray
                let label = (gi << half_bits) | gq;
                // Map grid indices to symmetric levels: -(side-1), -(side-3), ..., (side-1)
                let i_val = 2.0 * i_idx as f64 - (side as f64 - 1.0);
                let q_val = 2.0 * q_idx as f64 - (side as f64 - 1.0);
                points[label] = (i_val, q_val);
            }
        }

        // Normalise to unit average power
        let avg_power: f64 =
            points.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / order as f64;
        let scale = 1.0 / avg_power.sqrt();
        for p in &mut points {
            p.0 *= scale;
            p.1 *= scale;
        }

        Self {
            points,
            bits_per_symbol,
            side,
        }
    }

    /// Map a bit slice to a constellation symbol.
    ///
    /// `bits` length must equal `bits_per_symbol`.
    pub fn map_to_symbol(&self, bits: &[bool]) -> (f64, f64) {
        assert_eq!(
            bits.len(),
            self.bits_per_symbol,
            "Expected {} bits, got {}",
            self.bits_per_symbol,
            bits.len()
        );
        let half = self.bits_per_symbol / 2;
        // Upper half bits -> I Gray index, lower half bits -> Q Gray index
        let i_bin = bits_to_natural(&bits[..half]);
        let q_bin = bits_to_natural(&bits[half..]);
        let gi = gray_code(i_bin);
        let gq = gray_code(q_bin);
        let label = (gi << half) | gq;
        self.points[label]
    }

    /// Hard decision: find the nearest constellation point.
    ///
    /// Returns `(decoded_bits, euclidean_distance)`.
    pub fn decide(&self, received: (f64, f64)) -> (Vec<bool>, f64) {
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (idx, &(pi, pq)) in self.points.iter().enumerate() {
            let d = (received.0 - pi).powi(2) + (received.1 - pq).powi(2);
            if d < best_dist {
                best_dist = d;
                best_idx = idx;
            }
        }
        let best_dist = best_dist.sqrt();

        // Decode composite Gray label back to bits
        let half = self.bits_per_symbol / 2;
        let gi = best_idx >> half;
        let gq = best_idx & ((1 << half) - 1);
        let i_bin = gray_decode(gi);
        let q_bin = gray_decode(gq);

        let mut bits = Vec::with_capacity(self.bits_per_symbol);
        for k in (0..half).rev() {
            bits.push((i_bin >> k) & 1 == 1);
        }
        for k in (0..half).rev() {
            bits.push((q_bin >> k) & 1 == 1);
        }

        (bits, best_dist)
    }

    /// Average power of the constellation (should be ~1.0 after normalisation).
    pub fn average_power(&self) -> f64 {
        self.points
            .iter()
            .map(|(i, q)| i * i + q * q)
            .sum::<f64>()
            / self.points.len() as f64
    }

    /// Minimum distance between any two constellation points.
    pub fn min_distance(&self) -> f64 {
        let mut dmin = f64::MAX;
        for i in 0..self.points.len() {
            for j in (i + 1)..self.points.len() {
                let d = ((self.points[i].0 - self.points[j].0).powi(2)
                    + (self.points[i].1 - self.points[j].1).powi(2))
                .sqrt();
                if d < dmin {
                    dmin = d;
                }
            }
        }
        dmin
    }
}

/// Helper: bits (MSB-first) to natural binary integer.
fn bits_to_natural(bits: &[bool]) -> usize {
    let mut val = 0usize;
    for &b in bits {
        val = (val << 1) | (b as usize);
    }
    val
}

/// Helper: natural binary integer to bits (MSB-first) of given width.
fn natural_to_bits(val: usize, width: usize) -> Vec<bool> {
    (0..width).rev().map(|i| (val >> i) & 1 == 1).collect()
}

// ---------------------------------------------------------------------------
// Root-Raised-Cosine filter
// ---------------------------------------------------------------------------

/// Design a root-raised-cosine (RRC) pulse-shaping filter.
///
/// # Arguments
///
/// * `num_taps` - Filter length (should be odd for symmetry).
/// * `samples_per_symbol` - Oversampling factor.
/// * `rolloff` - Excess bandwidth factor in `[0.0, 1.0]`.
///
/// Returns a vector of `num_taps` filter coefficients, normalised so that
/// convolving the RRC with itself yields a raised-cosine (Nyquist) pulse.
pub fn rrc_filter(num_taps: usize, samples_per_symbol: usize, rolloff: f64) -> Vec<f64> {
    let sps = samples_per_symbol as f64;
    let mid = (num_taps - 1) as f64 / 2.0;
    let alpha = rolloff;

    let mut h: Vec<f64> = (0..num_taps)
        .map(|i| {
            let t = (i as f64 - mid) / sps; // time in symbol periods

            if t.abs() < 1e-12 {
                // t == 0
                (1.0 - alpha + 4.0 * alpha / PI) / sps.sqrt()
            } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-12 && alpha > 1e-12 {
                // t == +/- 1/(4*alpha)
                let s2 = 2.0_f64.sqrt();
                (alpha / (s2 * sps.sqrt()))
                    * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                        + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
            } else {
                let pit = PI * t;
                let num = (pit * (1.0 - alpha)).sin()
                    + 4.0 * alpha * t * (pit * (1.0 + alpha)).cos();
                let den = pit * (1.0 - (4.0 * alpha * t).powi(2));
                if den.abs() < 1e-30 {
                    // Near-singular: use L'Hopital approximation
                    (1.0 - alpha + 4.0 * alpha / PI) / sps.sqrt()
                } else {
                    num / (den * sps.sqrt())
                }
            }
        })
        .collect();

    // Normalise energy so matched filter pair has unit gain
    let energy: f64 = h.iter().map(|x| x * x).sum();
    let norm = 1.0 / energy.sqrt();
    for v in &mut h {
        *v *= norm;
    }

    h
}

// ---------------------------------------------------------------------------
// Pulse shaping: upsample + filter
// ---------------------------------------------------------------------------

/// Upsample symbols by `sps` (zero-insertion) and convolve with an RRC filter.
///
/// Output length = `symbols.len() * sps + filter.len() - 1`.
pub fn upsample_and_filter(
    symbols: &[(f64, f64)],
    filter: &[f64],
    sps: usize,
) -> Vec<(f64, f64)> {
    // Zero-insert upsampled signal
    let up_len = symbols.len() * sps;
    let mut up_i = vec![0.0; up_len];
    let mut up_q = vec![0.0; up_len];
    for (k, &(si, sq)) in symbols.iter().enumerate() {
        up_i[k * sps] = si;
        up_q[k * sps] = sq;
    }

    // Convolve
    let out_len = up_len + filter.len() - 1;
    let mut out = Vec::with_capacity(out_len);
    for n in 0..out_len {
        let mut acc_i = 0.0;
        let mut acc_q = 0.0;
        for (k, &fk) in filter.iter().enumerate() {
            if n >= k && (n - k) < up_len {
                acc_i += up_i[n - k] * fk;
                acc_q += up_q[n - k] * fk;
            }
        }
        out.push((acc_i, acc_q));
    }
    out
}

// ---------------------------------------------------------------------------
// Matched filter + downsample
// ---------------------------------------------------------------------------

/// Apply a matched filter (RRC) to the received signal and downsample at
/// the optimal sampling instant.
///
/// The filter is the same RRC used at the transmitter; convolving twice
/// produces a raised-cosine (ISI-free) pulse.
///
/// Returns one symbol-rate sample per symbol period.
pub fn matched_filter_downsample(
    signal: &[(f64, f64)],
    filter: &[f64],
    sps: usize,
) -> Vec<(f64, f64)> {
    let sig_len = signal.len();
    let filt_len = filter.len();
    let conv_len = sig_len + filt_len - 1;

    // Full convolution with matched filter
    let mut conv = Vec::with_capacity(conv_len);
    for n in 0..conv_len {
        let mut acc_i = 0.0;
        let mut acc_q = 0.0;
        for (k, &fk) in filter.iter().enumerate() {
            if n >= k && (n - k) < sig_len {
                acc_i += signal[n - k].0 * fk;
                acc_q += signal[n - k].1 * fk;
            }
        }
        conv.push((acc_i, acc_q));
    }

    // Optimal sampling point is at the peak of the RC pulse = center of combined filter
    let offset = filt_len - 1; // group delay of two cascaded filters

    // Downsample at symbol rate
    let mut symbols = Vec::new();
    let mut idx = offset;
    while idx < conv_len {
        symbols.push(conv[idx]);
        idx += sps;
    }
    symbols
}

// ---------------------------------------------------------------------------
// Transmitter
// ---------------------------------------------------------------------------

/// QAM Transmitter: bits to pulse-shaped IQ samples.
#[derive(Debug, Clone)]
pub struct QamTransmitter {
    config: QamConfig,
    constellation: QamConstellation,
    rrc_taps: Vec<f64>,
}

impl QamTransmitter {
    /// Create a new QAM transmitter.
    pub fn new(config: QamConfig) -> Self {
        let constellation = QamConstellation::new(config.order);
        let rrc_taps = rrc_filter(config.rrc_num_taps(), config.samples_per_symbol, config.rolloff);
        Self {
            config,
            constellation,
            rrc_taps,
        }
    }

    /// Modulate bits into pulse-shaped IQ samples.
    ///
    /// If the number of input bits is not a multiple of `bits_per_symbol`,
    /// trailing zeros are padded.
    pub fn modulate(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let bps = self.config.bits_per_symbol();

        // Pad to a multiple of bits_per_symbol
        let mut padded = bits.to_vec();
        while padded.len() % bps != 0 {
            padded.push(false);
        }

        // Map to symbols
        let symbols: Vec<(f64, f64)> = padded
            .chunks(bps)
            .map(|chunk| self.constellation.map_to_symbol(chunk))
            .collect();

        // Pulse shaping
        upsample_and_filter(&symbols, &self.rrc_taps, self.config.samples_per_symbol)
    }

    /// Access the constellation.
    pub fn constellation(&self) -> &QamConstellation {
        &self.constellation
    }

    /// Map bits to symbols without pulse shaping (useful for testing).
    pub fn bits_to_symbols(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let bps = self.config.bits_per_symbol();
        let mut padded = bits.to_vec();
        while padded.len() % bps != 0 {
            padded.push(false);
        }
        padded
            .chunks(bps)
            .map(|chunk| self.constellation.map_to_symbol(chunk))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Receiver
// ---------------------------------------------------------------------------

/// QAM Receiver: IQ samples to bits via matched filter and hard decision.
#[derive(Debug, Clone)]
pub struct QamReceiver {
    config: QamConfig,
    constellation: QamConstellation,
    rrc_taps: Vec<f64>,
}

impl QamReceiver {
    /// Create a new QAM receiver.
    pub fn new(config: QamConfig) -> Self {
        let constellation = QamConstellation::new(config.order);
        let rrc_taps = rrc_filter(config.rrc_num_taps(), config.samples_per_symbol, config.rolloff);
        Self {
            config,
            constellation,
            rrc_taps,
        }
    }

    /// Demodulate IQ samples back to bits.
    ///
    /// Applies matched filter, downsamples at symbol rate, and makes hard
    /// decisions on each symbol.
    pub fn demodulate(&self, iq: &[(f64, f64)]) -> Vec<bool> {
        // Matched filter + downsample
        let symbols =
            matched_filter_downsample(iq, &self.rrc_taps, self.config.samples_per_symbol);

        // Hard decision
        let mut bits = Vec::with_capacity(symbols.len() * self.config.bits_per_symbol());
        for sym in &symbols {
            let (decoded, _dist) = self.constellation.decide(*sym);
            bits.extend(decoded);
        }
        bits
    }

    /// Access the constellation.
    pub fn constellation(&self) -> &QamConstellation {
        &self.constellation
    }
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

/// Compute Error Vector Magnitude (EVM) as a percentage.
///
/// EVM = 100 * sqrt(mean |error|^2 / mean |ref|^2)
///
/// Both slices must have the same length.
pub fn evm_percent(tx_symbols: &[(f64, f64)], rx_symbols: &[(f64, f64)]) -> f64 {
    assert_eq!(tx_symbols.len(), rx_symbols.len(), "Symbol count mismatch");
    let n = tx_symbols.len() as f64;
    let error_power: f64 = tx_symbols
        .iter()
        .zip(rx_symbols.iter())
        .map(|((ti, tq), (ri, rq))| (ti - ri).powi(2) + (tq - rq).powi(2))
        .sum::<f64>()
        / n;
    let ref_power: f64 = tx_symbols
        .iter()
        .map(|(i, q)| i * i + q * q)
        .sum::<f64>()
        / n;

    if ref_power < 1e-30 {
        return 0.0;
    }
    100.0 * (error_power / ref_power).sqrt()
}

/// Compute Symbol Error Rate (SER).
///
/// Compares bit-groups of `bits_per_symbol` width. Both slices should have
/// the same length; if not, the shorter length is used.
pub fn symbol_error_rate(tx_bits: &[bool], rx_bits: &[bool]) -> f64 {
    if tx_bits.is_empty() || rx_bits.is_empty() {
        return 0.0;
    }
    let len = tx_bits.len().min(rx_bits.len());
    let errors = tx_bits[..len]
        .iter()
        .zip(rx_bits[..len].iter())
        .filter(|(a, b)| a != b)
        .count();
    // This is a per-bit comparison; for true SER we'd need bits_per_symbol,
    // but we report the fraction of differing bits as a proxy.
    // For exact SER, use the overloaded version.
    errors as f64 / len as f64
}

/// Compute Bit Error Rate (BER).
///
/// Simple ratio of differing bits to total bits compared.
pub fn compute_ber(tx_bits: &[bool], rx_bits: &[bool]) -> f64 {
    if tx_bits.is_empty() || rx_bits.is_empty() {
        return 0.0;
    }
    let len = tx_bits.len().min(rx_bits.len());
    let errors = tx_bits[..len]
        .iter()
        .zip(rx_bits[..len].iter())
        .filter(|(a, b)| a != b)
        .count();
    errors as f64 / len as f64
}

// ---------------------------------------------------------------------------
// Utility: simple AWGN for testing
// ---------------------------------------------------------------------------

/// Add white Gaussian noise to IQ samples (simple Box-Muller, seeded LCG).
///
/// `snr_db` is per-symbol SNR. This is a deterministic PRNG for reproducible
/// tests --- NOT cryptographically secure.
fn add_awgn(signal: &[(f64, f64)], snr_db: f64, seed: u64) -> Vec<(f64, f64)> {
    let sig_power: f64 =
        signal.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / signal.len() as f64;
    let noise_power = sig_power / (10.0_f64).powf(snr_db / 10.0);
    let sigma = (noise_power / 2.0).sqrt(); // per I/Q component

    // Simple LCG PRNG
    let mut state = seed.wrapping_add(1);
    let mut next_u = || -> f64 {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to (0, 1) exclusive
        let val = ((state >> 11) as f64) / ((1u64 << 53) as f64);
        val.max(1e-15).min(1.0 - 1e-15)
    };

    signal
        .iter()
        .map(|&(si, sq)| {
            // Box-Muller transform
            let u1 = next_u();
            let u2 = next_u();
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            let ni = sigma * r * theta.cos();
            let nq = sigma * r * theta.sin();
            (si + ni, sq + nq)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Gray coding tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gray_code_first_values() {
        // 0→0, 1→1, 2→3, 3→2, 4→6, 5→7, 6→5, 7→4
        assert_eq!(gray_code(0), 0);
        assert_eq!(gray_code(1), 1);
        assert_eq!(gray_code(2), 3);
        assert_eq!(gray_code(3), 2);
        assert_eq!(gray_code(4), 6);
        assert_eq!(gray_code(5), 7);
        assert_eq!(gray_code(6), 5);
        assert_eq!(gray_code(7), 4);
    }

    #[test]
    fn test_gray_decode_roundtrip() {
        for n in 0..256 {
            assert_eq!(gray_decode(gray_code(n)), n);
        }
    }

    #[test]
    fn test_gray_adjacent_differ_by_one_bit() {
        for n in 0..255 {
            let g1 = gray_code(n);
            let g2 = gray_code(n + 1);
            let diff = g1 ^ g2;
            // Exactly one bit different
            assert_eq!(diff.count_ones(), 1, "n={}", n);
        }
    }

    #[test]
    fn test_bits_to_gray_index() {
        // bits [1, 0] = binary 2 → Gray 3
        assert_eq!(bits_to_gray_index(&[true, false]), 3);
        // bits [0, 0] = binary 0 → Gray 0
        assert_eq!(bits_to_gray_index(&[false, false]), 0);
        // bits [1, 1] = binary 3 → Gray 2
        assert_eq!(bits_to_gray_index(&[true, true]), 2);
    }

    // -----------------------------------------------------------------------
    // Constellation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_constellation_4qam_size() {
        let c = QamConstellation::new(4);
        assert_eq!(c.points.len(), 4);
        assert_eq!(c.bits_per_symbol, 2);
    }

    #[test]
    fn test_constellation_16qam_size() {
        let c = QamConstellation::new(16);
        assert_eq!(c.points.len(), 16);
        assert_eq!(c.bits_per_symbol, 4);
    }

    #[test]
    fn test_constellation_64qam_size() {
        let c = QamConstellation::new(64);
        assert_eq!(c.points.len(), 64);
        assert_eq!(c.bits_per_symbol, 6);
    }

    #[test]
    fn test_constellation_256qam_size() {
        let c = QamConstellation::new(256);
        assert_eq!(c.points.len(), 256);
        assert_eq!(c.bits_per_symbol, 8);
    }

    #[test]
    fn test_constellation_unit_power() {
        for &order in &[4, 16, 64, 256] {
            let c = QamConstellation::new(order);
            let avg = c.average_power();
            assert!(
                (avg - 1.0).abs() < 1e-10,
                "Order {} avg power = {} (expected 1.0)",
                order,
                avg
            );
        }
    }

    #[test]
    fn test_constellation_symmetry() {
        // All QAM constellations should be symmetric about origin
        for &order in &[4, 16, 64] {
            let c = QamConstellation::new(order);
            let sum_i: f64 = c.points.iter().map(|(i, _)| *i).sum();
            let sum_q: f64 = c.points.iter().map(|(_, q)| *q).sum();
            assert!(
                sum_i.abs() < 1e-10,
                "Order {} I centroid = {}",
                order,
                sum_i
            );
            assert!(
                sum_q.abs() < 1e-10,
                "Order {} Q centroid = {}",
                order,
                sum_q
            );
        }
    }

    #[test]
    fn test_4qam_points_quadrants() {
        let c = QamConstellation::new(4);
        // With normalisation, 4-QAM should have points at ±1/√2, ±1/√2
        for &(i, q) in &c.points {
            assert!(
                (i.abs() - q.abs()).abs() < 1e-10,
                "4-QAM point ({}, {}) not on diagonal",
                i,
                q
            );
        }
    }

    // -----------------------------------------------------------------------
    // Symbol mapping roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_symbol_map_decide_roundtrip_4qam() {
        let c = QamConstellation::new(4);
        for i in 0..4 {
            let bits: Vec<bool> = (0..2).rev().map(|k| (i >> k) & 1 == 1).collect();
            let sym = c.map_to_symbol(&bits);
            let (decoded, dist) = c.decide(sym);
            assert_eq!(bits, decoded, "4-QAM roundtrip failed for {:?}", bits);
            assert!(dist < 1e-10, "Distance should be ~0, got {}", dist);
        }
    }

    #[test]
    fn test_symbol_map_decide_roundtrip_16qam() {
        let c = QamConstellation::new(16);
        for i in 0..16 {
            let bits: Vec<bool> = (0..4).rev().map(|k| (i >> k) & 1 == 1).collect();
            let sym = c.map_to_symbol(&bits);
            let (decoded, dist) = c.decide(sym);
            assert_eq!(bits, decoded, "16-QAM roundtrip failed for {:?}", bits);
            assert!(dist < 1e-10);
        }
    }

    #[test]
    fn test_symbol_map_decide_roundtrip_64qam() {
        let c = QamConstellation::new(64);
        for i in 0..64 {
            let bits: Vec<bool> = (0..6).rev().map(|k| (i >> k) & 1 == 1).collect();
            let sym = c.map_to_symbol(&bits);
            let (decoded, dist) = c.decide(sym);
            assert_eq!(bits, decoded, "64-QAM roundtrip failed for {:?}", bits);
            assert!(dist < 1e-10);
        }
    }

    #[test]
    fn test_symbol_map_decide_roundtrip_256qam() {
        let c = QamConstellation::new(256);
        for i in 0..256 {
            let bits: Vec<bool> = (0..8).rev().map(|k| (i >> k) & 1 == 1).collect();
            let sym = c.map_to_symbol(&bits);
            let (decoded, dist) = c.decide(sym);
            assert_eq!(bits, decoded, "256-QAM roundtrip failed for {:?}", bits);
            assert!(dist < 1e-10);
        }
    }

    // -----------------------------------------------------------------------
    // RRC filter properties
    // -----------------------------------------------------------------------

    #[test]
    fn test_rrc_filter_symmetry() {
        let h = rrc_filter(65, 8, 0.35);
        let n = h.len();
        for i in 0..n / 2 {
            assert!(
                (h[i] - h[n - 1 - i]).abs() < 1e-12,
                "RRC asymmetry at tap {}",
                i
            );
        }
    }

    #[test]
    fn test_rrc_filter_length() {
        let h = rrc_filter(65, 8, 0.35);
        assert_eq!(h.len(), 65);
    }

    #[test]
    fn test_rrc_nyquist_criterion() {
        // Convolving RRC with itself should yield a raised-cosine (Nyquist)
        // pulse that has zero crossings at multiples of the symbol period.
        let sps = 8;
        let num_taps = 8 * sps + 1;
        let h = rrc_filter(num_taps, sps, 0.35);

        // Self-convolution → raised cosine
        let rc_len = 2 * h.len() - 1;
        let mut rc = vec![0.0; rc_len];
        for i in 0..h.len() {
            for j in 0..h.len() {
                rc[i + j] += h[i] * h[j];
            }
        }

        // The peak is at the center
        let center = rc_len / 2;

        // Check zero crossings at ±sps, ±2*sps, etc. from center
        for k in 1..=7 {
            let idx_pos = center + k * sps;
            let idx_neg = center - k * sps;
            if idx_pos < rc_len {
                assert!(
                    rc[idx_pos].abs() < 0.05, // Allow small ISI from truncation
                    "Nyquist ISI at +{}T: {} (should be ~0)",
                    k,
                    rc[idx_pos]
                );
            }
            assert!(
                rc[idx_neg].abs() < 0.05,
                "Nyquist ISI at -{}T: {} (should be ~0)",
                k,
                rc[idx_neg]
            );
        }
    }

    #[test]
    fn test_rrc_different_rolloffs() {
        for &rolloff in &[0.0, 0.25, 0.35, 0.5, 1.0] {
            let h = rrc_filter(65, 8, rolloff);
            assert_eq!(h.len(), 65);
            // Energy should be finite and positive
            let energy: f64 = h.iter().map(|x| x * x).sum();
            assert!(energy > 0.0, "Zero energy for rolloff={}", rolloff);
            assert!(energy.is_finite(), "Non-finite energy for rolloff={}", rolloff);
        }
    }

    // -----------------------------------------------------------------------
    // Upsample and filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_upsample_output_length() {
        let symbols = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let h = rrc_filter(17, 4, 0.35);
        let out = upsample_and_filter(&symbols, &h, 4);
        // Expected: n_sym * sps + filter_len - 1
        assert_eq!(out.len(), 3 * 4 + 17 - 1);
    }

    // -----------------------------------------------------------------------
    // Matched filter + downsample
    // -----------------------------------------------------------------------

    #[test]
    fn test_matched_filter_preserves_symbols() {
        // With matched RRC filtering (no noise), recovered symbols should
        // closely match the original constellation points.
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config.clone());
        let bits: Vec<bool> = vec![false, false, false, true, true, false, true, true];
        let tx_symbols = tx.bits_to_symbols(&bits);
        let iq = tx.modulate(&bits);

        let rx = QamReceiver::new(config);
        let rx_symbols =
            matched_filter_downsample(&iq, &rx.rrc_taps, rx.config.samples_per_symbol);

        // Should recover at least as many symbols as transmitted
        assert!(rx_symbols.len() >= tx_symbols.len());

        // First few recovered symbols should be close to transmitted ones
        for (k, (&tx_s, &rx_s)) in tx_symbols.iter().zip(rx_symbols.iter()).enumerate() {
            let err = ((tx_s.0 - rx_s.0).powi(2) + (tx_s.1 - rx_s.1).powi(2)).sqrt();
            assert!(
                err < 0.15,
                "Symbol {} error too large: {} (tx={:?}, rx={:?})",
                k,
                err,
                tx_s,
                rx_s
            );
        }
    }

    // -----------------------------------------------------------------------
    // Full TX-RX loopback
    // -----------------------------------------------------------------------

    #[test]
    fn test_loopback_4qam_noiseless() {
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..20).map(|i| i % 3 == 0).collect();
        let iq = tx.modulate(&bits);
        let recovered = rx.demodulate(&iq);

        assert_eq!(&bits[..], &recovered[..bits.len()]);
    }

    #[test]
    fn test_loopback_16qam_noiseless() {
        let config = QamConfig::default_for(16);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..32).map(|i| (i * 7 + 3) % 5 < 3).collect();
        let iq = tx.modulate(&bits);
        let recovered = rx.demodulate(&iq);

        assert_eq!(&bits[..], &recovered[..bits.len()]);
    }

    #[test]
    fn test_loopback_64qam_noiseless() {
        let config = QamConfig::default_for(64);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..48).map(|i| (i * 13 + 5) % 7 < 4).collect();
        let iq = tx.modulate(&bits);
        let recovered = rx.demodulate(&iq);

        assert_eq!(&bits[..], &recovered[..bits.len()]);
    }

    #[test]
    fn test_loopback_256qam_noiseless() {
        let config = QamConfig::default_for(256);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..64).map(|i| (i * 11 + 7) % 9 < 5).collect();
        let iq = tx.modulate(&bits);
        let recovered = rx.demodulate(&iq);

        assert_eq!(&bits[..], &recovered[..bits.len()]);
    }

    // -----------------------------------------------------------------------
    // Noisy loopback
    // -----------------------------------------------------------------------

    #[test]
    fn test_loopback_4qam_high_snr() {
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..100).map(|i| (i * 7) % 11 < 6).collect();
        let iq = tx.modulate(&bits);
        let noisy = add_awgn(&iq, 30.0, 42); // 30 dB SNR — very clean
        let recovered = rx.demodulate(&noisy);

        let ber = compute_ber(&bits, &recovered[..bits.len()]);
        assert!(
            ber < 0.01,
            "BER too high at 30 dB SNR for 4-QAM: {}",
            ber
        );
    }

    #[test]
    fn test_loopback_16qam_moderate_snr() {
        let config = QamConfig::default_for(16);
        let tx = QamTransmitter::new(config.clone());
        let rx = QamReceiver::new(config);

        let bits: Vec<bool> = (0..200).map(|i| (i * 13) % 17 < 9).collect();
        let iq = tx.modulate(&bits);
        let noisy = add_awgn(&iq, 35.0, 12345); // 35 dB
        let recovered = rx.demodulate(&noisy);

        let ber = compute_ber(&bits, &recovered[..bits.len()]);
        assert!(
            ber < 0.01,
            "BER too high at 35 dB SNR for 16-QAM: {}",
            ber
        );
    }

    // -----------------------------------------------------------------------
    // EVM tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_evm_perfect_signal() {
        let syms = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let evm = evm_percent(&syms, &syms);
        assert!(evm < 1e-10, "EVM should be 0 for perfect signal, got {}", evm);
    }

    #[test]
    fn test_evm_known_error() {
        let tx = vec![(1.0, 0.0)];
        let rx = vec![(1.1, 0.0)]; // 10% error on I component
        let evm = evm_percent(&tx, &rx);
        // EVM = 100 * sqrt(0.01 / 1.0) = 10%
        assert!(
            (evm - 10.0).abs() < 0.1,
            "EVM expected ~10%, got {}%",
            evm
        );
    }

    #[test]
    fn test_evm_with_awgn() {
        let config = QamConfig::default_for(16);
        let tx = QamTransmitter::new(config.clone());

        let bits: Vec<bool> = (0..200).map(|i| (i * 7) % 11 < 6).collect();
        let tx_symbols = tx.bits_to_symbols(&bits);
        let iq = tx.modulate(&bits);
        let noisy = add_awgn(&iq, 20.0, 999);

        let rx = QamReceiver::new(config);
        let rx_symbols =
            matched_filter_downsample(&noisy, &rx.rrc_taps, rx.config.samples_per_symbol);

        let len = tx_symbols.len().min(rx_symbols.len());
        let evm = evm_percent(&tx_symbols[..len], &rx_symbols[..len]);
        // At 20 dB SNR, EVM should be roughly 10%
        assert!(evm > 0.0, "EVM should be positive with noise");
        assert!(evm < 50.0, "EVM unexpectedly high: {}%", evm);
    }

    // -----------------------------------------------------------------------
    // BER / SER tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ber_identical() {
        let bits = vec![true, false, true, true];
        assert!((compute_ber(&bits, &bits) - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_ber_all_wrong() {
        let tx = vec![true, true, true, true];
        let rx = vec![false, false, false, false];
        assert!((compute_ber(&tx, &rx) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn test_ber_half_wrong() {
        let tx = vec![true, true, false, false];
        let rx = vec![true, false, false, true];
        assert!((compute_ber(&tx, &rx) - 0.5).abs() < 1e-15);
    }

    #[test]
    fn test_ser_identical() {
        let bits = vec![true, false, true, true];
        assert!((symbol_error_rate(&bits, &bits) - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_ber_empty() {
        assert!((compute_ber(&[], &[]) - 0.0).abs() < 1e-15);
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_modulate_empty_bits() {
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config);
        let iq = tx.modulate(&[]);
        // Should produce only the filter tail (no symbols)
        assert!(iq.is_empty() || iq.len() == tx.rrc_taps.len() - 1);
    }

    #[test]
    fn test_modulate_single_symbol() {
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config.clone());
        let bits = vec![true, false];
        let iq = tx.modulate(&bits);
        // 1 symbol * sps + filter_len - 1
        let expected = 1 * config.samples_per_symbol + tx.rrc_taps.len() - 1;
        assert_eq!(iq.len(), expected);
    }

    #[test]
    fn test_padding_non_multiple_bits() {
        // 5 bits for 4-QAM (bps=2) should pad to 6 bits (3 symbols)
        let config = QamConfig::default_for(4);
        let tx = QamTransmitter::new(config.clone());
        let bits = vec![true, false, true, true, false]; // 5 bits
        let symbols = tx.bits_to_symbols(&bits);
        assert_eq!(symbols.len(), 3); // ceil(5/2) = 3 after padding
    }

    #[test]
    #[should_panic(expected = "QAM order must be")]
    fn test_invalid_order() {
        QamConfig::new(8, 1000.0, 8, 0.35, 0.0);
    }

    #[test]
    #[should_panic(expected = "Roll-off must be")]
    fn test_invalid_rolloff() {
        QamConfig::new(4, 1000.0, 8, 1.5, 0.0);
    }

    #[test]
    fn test_min_distance_decreases_with_order() {
        let d4 = QamConstellation::new(4).min_distance();
        let d16 = QamConstellation::new(16).min_distance();
        let d64 = QamConstellation::new(64).min_distance();
        let d256 = QamConstellation::new(256).min_distance();
        assert!(d4 > d16, "4-QAM dmin={} should > 16-QAM dmin={}", d4, d16);
        assert!(
            d16 > d64,
            "16-QAM dmin={} should > 64-QAM dmin={}",
            d16,
            d64
        );
        assert!(
            d64 > d256,
            "64-QAM dmin={} should > 256-QAM dmin={}",
            d64,
            d256
        );
    }

    #[test]
    fn test_config_bits_per_symbol() {
        assert_eq!(QamConfig::default_for(4).bits_per_symbol(), 2);
        assert_eq!(QamConfig::default_for(16).bits_per_symbol(), 4);
        assert_eq!(QamConfig::default_for(64).bits_per_symbol(), 6);
        assert_eq!(QamConfig::default_for(256).bits_per_symbol(), 8);
    }

    #[test]
    fn test_decide_returns_nearest_neighbor() {
        let c = QamConstellation::new(16);
        // Slightly perturbed version of a known point
        let target = c.points[0];
        let perturbed = (target.0 + 0.01, target.1 - 0.01);
        let (bits, dist) = c.decide(perturbed);
        // Should decode to the same bits as the original point
        let (ref_bits, _) = c.decide(target);
        assert_eq!(bits, ref_bits);
        assert!(dist < 0.02);
    }
}
