//! # DRM OFDM Signal Processor
//!
//! Implements Digital Radio Mondiale (DRM) OFDM signal processing for
//! HF/VHF digital broadcasting per ETSI ES 201 980.
//!
//! DRM uses coded OFDM (COFDM) with four robustness modes (A/B/C/D)
//! optimised for different propagation channels:
//!
//! - **Mode A**: Gaussian (local, benign) channels
//! - **Mode B**: Time/frequency-selective (international HF)
//! - **Mode C**: Similar to B with wider guard for worst-case HF
//! - **Mode D**: Severe time/frequency-selective (extreme HF)
//!
//! Spectral occupancy identifiers (0..5) select the occupied bandwidth
//! (4.5 / 5 / 9 / 10 / 18 / 20 kHz).
//!
//! # Example
//!
//! ```
//! use r4w_core::drm_ofdm_processor::{DrmOfdmProcessor, RobustnessMode, QamOrder};
//!
//! // Create a Mode B processor (typical international HF)
//! let mut proc = DrmOfdmProcessor::new(RobustnessMode::B);
//!
//! // Map some data bits to 4-QAM symbols, build an OFDM symbol
//! let bits: Vec<bool> = vec![false, true, true, false, false, false, true, true];
//! let qam_symbols = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam4);
//! assert_eq!(qam_symbols.len(), 4); // 2 bits per 4-QAM symbol
//!
//! // Generate one OFDM symbol (FFT + cyclic prefix)
//! let ofdm_symbol = proc.generate_ofdm_symbol(&qam_symbols, 0);
//! assert_eq!(ofdm_symbol.len(), proc.params().fft_size + proc.params().guard_samples);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex-number helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

/// Complex addition.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex magnitude squared.
#[inline]
fn cabs2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex magnitude.
#[inline]
fn cabs(a: (f64, f64)) -> f64 {
    cabs2(a).sqrt()
}

/// Complex division: a / b.
#[inline]
fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let d = cabs2(b);
    if d == 0.0 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
    }
}

/// Scale a complex value.
#[inline]
fn cscale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// FFT (radix-2 Cooley-Tukey, in-place, decimation-in-time)
// ---------------------------------------------------------------------------

/// In-place radix-2 FFT. `inverse` selects IFFT (with 1/N scaling).
/// `buf.len()` **must** be a power of two.
fn fft_inplace(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be power of two");

    // Bit-reversal permutation
    let mut j: usize = 0;
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

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w: (f64, f64) = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = cmul(w, buf[start + k + half]);
                buf[start + k] = cadd(u, t);
                buf[start + k + half] = csub(u, t);
                w = cmul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in buf.iter_mut() {
            *x = cscale(*x, inv_n);
        }
    }
}

/// Convenience: forward FFT returning a new vector.
fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, false);
    buf
}

/// Convenience: inverse FFT returning a new vector.
fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, true);
    buf
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// DRM robustness mode (ETSI ES 201 980, Table 82).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobustnessMode {
    /// Gaussian / benign channel
    A,
    /// Time- and frequency-selective (international HF)
    B,
    /// Worst-case HF
    C,
    /// Extreme HF
    D,
}

/// QAM constellation order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QamOrder {
    /// 4-QAM (2 bits/symbol)
    Qam4,
    /// 16-QAM (4 bits/symbol)
    Qam16,
    /// 64-QAM (6 bits/symbol)
    Qam64,
}

impl QamOrder {
    /// Bits per symbol for this constellation.
    pub fn bits_per_symbol(self) -> usize {
        match self {
            QamOrder::Qam4 => 2,
            QamOrder::Qam16 => 4,
            QamOrder::Qam64 => 6,
        }
    }
}

/// DRM spectral occupancy identifier (ETSI ES 201 980, Table 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpectralOccupancy {
    /// ~4.5 kHz
    So0,
    /// ~5 kHz
    So1,
    /// ~9 kHz
    So2,
    /// ~10 kHz
    So3,
    /// ~18 kHz
    So4,
    /// ~20 kHz
    So5,
}

impl SpectralOccupancy {
    /// Nominal bandwidth in Hz.
    pub fn bandwidth_hz(self) -> f64 {
        match self {
            SpectralOccupancy::So0 => 4500.0,
            SpectralOccupancy::So1 => 5000.0,
            SpectralOccupancy::So2 => 9000.0,
            SpectralOccupancy::So3 => 10000.0,
            SpectralOccupancy::So4 => 18000.0,
            SpectralOccupancy::So5 => 20000.0,
        }
    }
}

/// Mode-dependent OFDM parameters.
#[derive(Debug, Clone)]
pub struct DrmOfdmParams {
    /// FFT size (power of two; determines subcarrier spacing).
    pub fft_size: usize,
    /// Guard interval length in samples (cyclic prefix).
    pub guard_samples: usize,
    /// Number of useful data carriers (excluding DC and guard band).
    pub carrier_count: usize,
    /// Total OFDM symbol duration in samples (`fft_size + guard_samples`).
    pub symbol_samples: usize,
    /// Nominal subcarrier spacing in Hz (at 48 kHz sample rate).
    pub subcarrier_spacing_hz: f64,
    /// Robustness mode.
    pub mode: RobustnessMode,
    /// Index of the lowest used subcarrier (offset from DC).
    pub k_min: isize,
    /// Index of the highest used subcarrier.
    pub k_max: isize,
}

/// A single pilot cell definition.
#[derive(Debug, Clone, Copy)]
pub struct PilotCell {
    /// Subcarrier index (relative to DC).
    pub carrier: isize,
    /// OFDM symbol index within the transmission frame.
    pub symbol: usize,
    /// Known complex value (reference amplitude and phase).
    pub value: (f64, f64),
}

/// Result of guard-interval-based mode detection.
#[derive(Debug, Clone)]
pub struct ModeDetectionResult {
    /// Detected robustness mode.
    pub mode: RobustnessMode,
    /// Correlation metric (higher = more confident).
    pub correlation: f64,
}

/// Channel estimate for a single subcarrier.
#[derive(Debug, Clone, Copy)]
pub struct SubcarrierEstimate {
    /// Subcarrier index.
    pub carrier: isize,
    /// Estimated channel response H(k).
    pub h: (f64, f64),
}

/// Time-synchronization result from cyclic-prefix correlation.
#[derive(Debug, Clone)]
pub struct TimeSyncResult {
    /// Estimated sample offset of the symbol boundary.
    pub offset: usize,
    /// Peak correlation value.
    pub peak_correlation: f64,
}

/// Frequency offset estimate from pilot cells.
#[derive(Debug, Clone)]
pub struct FreqSyncResult {
    /// Fractional subcarrier frequency offset (normalised to subcarrier spacing).
    pub fractional_offset: f64,
    /// Integer subcarrier offset.
    pub integer_offset: isize,
}

// ---------------------------------------------------------------------------
// DrmOfdmProcessor
// ---------------------------------------------------------------------------

/// Main DRM OFDM signal processor.
///
/// Encapsulates mode-dependent OFDM parameters, pilot patterns, QAM mapping,
/// symbol generation, demodulation, channel estimation, and synchronization.
#[derive(Debug, Clone)]
pub struct DrmOfdmProcessor {
    params: DrmOfdmParams,
    /// Spectral occupancy (default: So2 = 9 kHz).
    spectral_occupancy: SpectralOccupancy,
}

impl DrmOfdmProcessor {
    // -- Construction -------------------------------------------------------

    /// Create a processor for the given robustness mode.
    ///
    /// Default spectral occupancy is 9 kHz (SO2).
    pub fn new(mode: RobustnessMode) -> Self {
        let params = Self::make_params(mode);
        Self {
            params,
            spectral_occupancy: SpectralOccupancy::So2,
        }
    }

    /// Create a processor with explicit spectral occupancy.
    pub fn with_spectral_occupancy(mode: RobustnessMode, so: SpectralOccupancy) -> Self {
        let params = Self::make_params(mode);
        Self {
            params,
            spectral_occupancy: so,
        }
    }

    /// Access the OFDM parameters.
    pub fn params(&self) -> &DrmOfdmParams {
        &self.params
    }

    /// Access the spectral occupancy.
    pub fn spectral_occupancy(&self) -> SpectralOccupancy {
        self.spectral_occupancy
    }

    // -- Mode parameters (ETSI ES 201 980, Table 82-like) -------------------

    /// Build mode-specific OFDM parameters.
    ///
    /// Reference sample rate: 48 kHz. Parameters are loosely based on the
    /// DRM standard with power-of-two FFT sizes for efficient computation.
    fn make_params(mode: RobustnessMode) -> DrmOfdmParams {
        // (fft_size, guard_samples, carrier_count, k_min, k_max)
        let (fft, guard, carriers, k_min, k_max) = match mode {
            // Mode A: short guard, many carriers
            RobustnessMode::A => (256, 32, 208, -106isize, 102isize),
            // Mode B: medium guard
            RobustnessMode::B => (256, 64, 182, -91isize, 91isize),
            // Mode C: longer guard for worst-case HF
            RobustnessMode::C => (128, 32, 114, -57isize, 57isize),
            // Mode D: very long guard, few carriers
            RobustnessMode::D => (128, 44, 98, -49isize, 49isize),
        };
        let sample_rate = 48000.0_f64;
        DrmOfdmParams {
            fft_size: fft,
            guard_samples: guard,
            carrier_count: carriers,
            symbol_samples: fft + guard,
            subcarrier_spacing_hz: sample_rate / fft as f64,
            mode,
            k_min,
            k_max,
        }
    }

    // -- QAM mapping --------------------------------------------------------

    /// Map a slice of bits to QAM constellation points.
    ///
    /// The bit slice length must be a multiple of `order.bits_per_symbol()`.
    /// Gray-coded mapping is used for all orders.
    pub fn qam_map(bits: &[bool], order: QamOrder) -> Vec<(f64, f64)> {
        let bps = order.bits_per_symbol();
        assert!(
            bits.len() % bps == 0,
            "Bit length {} not a multiple of {}",
            bits.len(),
            bps
        );
        let n_symbols = bits.len() / bps;
        let mut out = Vec::with_capacity(n_symbols);
        for i in 0..n_symbols {
            let chunk = &bits[i * bps..(i + 1) * bps];
            out.push(Self::map_one_symbol(chunk, order));
        }
        out
    }

    /// Demap QAM symbols back to hard-decision bits.
    pub fn qam_demap(symbols: &[(f64, f64)], order: QamOrder) -> Vec<bool> {
        let bps = order.bits_per_symbol();
        let mut out = Vec::with_capacity(symbols.len() * bps);
        for &sym in symbols {
            let bits = Self::demap_one_symbol(sym, order);
            out.extend_from_slice(&bits);
        }
        out
    }

    /// Map a single bit-group to a constellation point.
    fn map_one_symbol(bits: &[bool], order: QamOrder) -> (f64, f64) {
        match order {
            QamOrder::Qam4 => {
                // Gray: 00->(+1,+1), 01->(+1,-1), 10->(-1,+1), 11->(-1,-1)
                let scale = 1.0 / 2.0_f64.sqrt();
                let re = if bits[0] { -1.0 } else { 1.0 };
                let im = if bits[1] { -1.0 } else { 1.0 };
                (re * scale, im * scale)
            }
            QamOrder::Qam16 => {
                let scale = 1.0 / 10.0_f64.sqrt(); // avg power = 1
                let re = Self::gray_2bit_to_pam4(bits[0], bits[1]);
                let im = Self::gray_2bit_to_pam4(bits[2], bits[3]);
                (re * scale, im * scale)
            }
            QamOrder::Qam64 => {
                let scale = 1.0 / 42.0_f64.sqrt(); // avg power = 1
                let re = Self::gray_3bit_to_pam8(bits[0], bits[1], bits[2]);
                let im = Self::gray_3bit_to_pam8(bits[3], bits[4], bits[5]);
                (re * scale, im * scale)
            }
        }
    }

    /// Demap a single constellation point to hard-decision bits.
    fn demap_one_symbol(sym: (f64, f64), order: QamOrder) -> Vec<bool> {
        match order {
            QamOrder::Qam4 => {
                vec![sym.0 < 0.0, sym.1 < 0.0]
            }
            QamOrder::Qam16 => {
                let scale = 10.0_f64.sqrt();
                let re = sym.0 * scale;
                let im = sym.1 * scale;
                let mut bits = Self::pam4_to_gray_2bit(re);
                bits.extend_from_slice(&Self::pam4_to_gray_2bit(im));
                bits
            }
            QamOrder::Qam64 => {
                let scale = 42.0_f64.sqrt();
                let re = sym.0 * scale;
                let im = sym.1 * scale;
                let mut bits = Self::pam8_to_gray_3bit(re);
                bits.extend_from_slice(&Self::pam8_to_gray_3bit(im));
                bits
            }
        }
    }

    /// Gray-coded 2-bit -> 4-PAM: 00->+3, 01->+1, 11->-1, 10->-3.
    fn gray_2bit_to_pam4(b0: bool, b1: bool) -> f64 {
        match (b0, b1) {
            (false, false) => 3.0,
            (false, true) => 1.0,
            (true, true) => -1.0,
            (true, false) => -3.0,
        }
    }

    /// Nearest 4-PAM -> Gray-coded 2 bits.
    fn pam4_to_gray_2bit(val: f64) -> Vec<bool> {
        if val >= 2.0 {
            vec![false, false]
        } else if val >= 0.0 {
            vec![false, true]
        } else if val >= -2.0 {
            vec![true, true]
        } else {
            vec![true, false]
        }
    }

    /// Gray-coded 3-bit -> 8-PAM.
    fn gray_3bit_to_pam8(b0: bool, b1: bool, b2: bool) -> f64 {
        // Gray sequence: 000->+7, 001->+5, 011->+3, 010->+1,
        //                110->-1, 111->-3, 101->-5, 100->-7
        match (b0, b1, b2) {
            (false, false, false) => 7.0,
            (false, false, true) => 5.0,
            (false, true, true) => 3.0,
            (false, true, false) => 1.0,
            (true, true, false) => -1.0,
            (true, true, true) => -3.0,
            (true, false, true) => -5.0,
            (true, false, false) => -7.0,
        }
    }

    /// Nearest 8-PAM -> Gray-coded 3 bits.
    fn pam8_to_gray_3bit(val: f64) -> Vec<bool> {
        if val >= 6.0 {
            vec![false, false, false]
        } else if val >= 4.0 {
            vec![false, false, true]
        } else if val >= 2.0 {
            vec![false, true, true]
        } else if val >= 0.0 {
            vec![false, true, false]
        } else if val >= -2.0 {
            vec![true, true, false]
        } else if val >= -4.0 {
            vec![true, true, true]
        } else if val >= -6.0 {
            vec![true, false, true]
        } else {
            vec![true, false, false]
        }
    }

    // -- Pilot pattern ------------------------------------------------------

    /// Generate frequency-reference pilot cells for a given OFDM symbol index.
    ///
    /// In DRM, pilot cells are placed at known subcarrier/symbol positions so
    /// the receiver can estimate the channel. This simplified pattern places
    /// pilots at every `pilot_spacing`-th subcarrier within the used range.
    pub fn pilot_cells(&self, symbol_idx: usize) -> Vec<PilotCell> {
        let pilot_spacing: isize = match self.params.mode {
            RobustnessMode::A => 4,
            RobustnessMode::B => 6,
            RobustnessMode::C => 4,
            RobustnessMode::D => 6,
        };
        // Phase pattern that varies with symbol index (simple BPSK pilot)
        let mut pilots = Vec::new();
        let mut k = self.params.k_min;
        // Offset pattern by symbol index to create a 2-D pilot grid
        let offset = (symbol_idx as isize % pilot_spacing) * 2;
        while k <= self.params.k_max {
            // Skip DC
            if k == 0 {
                k += 1;
                continue;
            }
            let idx_in_grid = (k - self.params.k_min + offset) % pilot_spacing;
            if idx_in_grid == 0 {
                // Pilot value: known BPSK modulated by a PN-like pattern
                let phase = ((k.unsigned_abs() + symbol_idx) % 2) as f64 * PI;
                let value = (phase.cos(), phase.sin());
                pilots.push(PilotCell {
                    carrier: k,
                    symbol: symbol_idx,
                    value,
                });
            }
            k += 1;
        }
        pilots
    }

    /// Return a list of data-carrier indices for the given symbol (those not
    /// occupied by pilots).
    pub fn data_carriers(&self, symbol_idx: usize) -> Vec<isize> {
        let pilot_set: std::collections::HashSet<isize> = self
            .pilot_cells(symbol_idx)
            .iter()
            .map(|p| p.carrier)
            .collect();

        let mut carriers = Vec::new();
        for k in self.params.k_min..=self.params.k_max {
            if k == 0 {
                continue;
            }
            if !pilot_set.contains(&k) {
                carriers.push(k);
            }
        }
        carriers
    }

    // -- OFDM symbol generation ---------------------------------------------

    /// Generate one OFDM time-domain symbol (IFFT + cyclic prefix).
    ///
    /// `data_symbols` are placed on data subcarriers; pilot cells are inserted
    /// automatically. If fewer data symbols are provided than data carriers,
    /// the remaining carriers are zero-filled. Extra symbols are ignored.
    pub fn generate_ofdm_symbol(
        &self,
        data_symbols: &[(f64, f64)],
        symbol_idx: usize,
    ) -> Vec<(f64, f64)> {
        let n = self.params.fft_size;
        let mut freq_domain = vec![(0.0, 0.0); n];

        // Insert pilots
        for pilot in self.pilot_cells(symbol_idx) {
            let bin = self.carrier_to_bin(pilot.carrier);
            if bin < n {
                freq_domain[bin] = pilot.value;
            }
        }

        // Insert data
        let data_carr = self.data_carriers(symbol_idx);
        for (i, &k) in data_carr.iter().enumerate() {
            if i >= data_symbols.len() {
                break;
            }
            let bin = self.carrier_to_bin(k);
            if bin < n {
                freq_domain[bin] = data_symbols[i];
            }
        }

        // IFFT to time domain
        let time_domain = ifft(&freq_domain);

        // Prepend cyclic prefix
        let gi = self.params.guard_samples;
        let mut symbol = Vec::with_capacity(n + gi);
        symbol.extend_from_slice(&time_domain[n - gi..]);
        symbol.extend_from_slice(&time_domain);
        symbol
    }

    /// Generate a sequence of OFDM symbols from a flat stream of data symbols.
    ///
    /// Returns concatenated time-domain samples for `num_symbols` OFDM symbols.
    pub fn generate_frame(
        &self,
        data_symbols: &[(f64, f64)],
        num_symbols: usize,
    ) -> Vec<(f64, f64)> {
        let mut output = Vec::new();
        let mut offset = 0;
        for sym_idx in 0..num_symbols {
            let dc = self.data_carriers(sym_idx);
            let end = (offset + dc.len()).min(data_symbols.len());
            let chunk = if offset < data_symbols.len() {
                &data_symbols[offset..end]
            } else {
                &[]
            };
            let ofdm_sym = self.generate_ofdm_symbol(chunk, sym_idx);
            output.extend_from_slice(&ofdm_sym);
            offset = end;
        }
        output
    }

    // -- OFDM demodulation --------------------------------------------------

    /// Demodulate one OFDM symbol: remove cyclic prefix, FFT, extract data carriers.
    ///
    /// `samples` must have length >= `symbol_samples` (FFT size + guard).
    /// Returns the complex values on data subcarriers.
    pub fn demodulate_ofdm_symbol(
        &self,
        samples: &[(f64, f64)],
        symbol_idx: usize,
    ) -> Vec<(f64, f64)> {
        let n = self.params.fft_size;
        let gi = self.params.guard_samples;
        assert!(
            samples.len() >= n + gi,
            "Need at least {} samples, got {}",
            n + gi,
            samples.len()
        );

        // Remove CP
        let useful = &samples[gi..gi + n];

        // FFT
        let freq = fft(useful);

        // Extract data carriers
        let data_carr = self.data_carriers(symbol_idx);
        let mut out = Vec::with_capacity(data_carr.len());
        for &k in &data_carr {
            let bin = self.carrier_to_bin(k);
            if bin < n {
                out.push(freq[bin]);
            }
        }
        out
    }

    // -- Mode detection -----------------------------------------------------

    /// Detect the robustness mode by correlating the cyclic prefix of each
    /// candidate mode against the signal.
    ///
    /// Tries all four modes and returns the one with the highest correlation.
    pub fn detect_mode(signal: &[(f64, f64)]) -> ModeDetectionResult {
        let modes = [
            RobustnessMode::A,
            RobustnessMode::B,
            RobustnessMode::C,
            RobustnessMode::D,
        ];
        let mut best_mode = RobustnessMode::B;
        let mut best_corr = -1.0_f64;

        for &mode in &modes {
            let params = Self::make_params(mode);
            let corr = Self::guard_interval_correlation(signal, &params);
            if corr > best_corr {
                best_corr = corr;
                best_mode = mode;
            }
        }

        ModeDetectionResult {
            mode: best_mode,
            correlation: best_corr,
        }
    }

    /// Compute cyclic-prefix correlation for given OFDM parameters.
    ///
    /// Correlates the guard interval portion with the corresponding tail of
    /// the useful part for each candidate symbol boundary and returns the
    /// maximum normalised correlation.
    fn guard_interval_correlation(signal: &[(f64, f64)], params: &DrmOfdmParams) -> f64 {
        let sym_len = params.symbol_samples;
        let gi = params.guard_samples;
        let n = params.fft_size;

        if signal.len() < sym_len {
            return 0.0;
        }

        let mut max_corr = 0.0_f64;
        let search_range = signal.len().saturating_sub(sym_len);
        // Limit search to avoid excessive computation
        let limit = search_range.min(sym_len * 4);

        for offset in 0..limit {
            if offset + sym_len > signal.len() {
                break;
            }
            // CP is at offset..offset+gi
            // Corresponding tail is at offset+n..offset+n+gi
            let mut corr_val = (0.0, 0.0);
            let mut power_a = 0.0;
            let mut power_b = 0.0;
            for j in 0..gi {
                let a = signal[offset + j];
                let b = signal[offset + n + j];
                corr_val = cadd(corr_val, cmul(a, conj(b)));
                power_a += cabs2(a);
                power_b += cabs2(b);
            }
            let norm = (power_a * power_b).sqrt();
            let c = if norm > 1e-30 { cabs(corr_val) / norm } else { 0.0 };
            if c > max_corr {
                max_corr = c;
            }
        }
        max_corr
    }

    // -- Channel estimation -------------------------------------------------

    /// Estimate the channel from received pilot cells.
    ///
    /// Given the received frequency-domain data (after FFT) and the known
    /// pilot pattern, returns per-subcarrier channel estimates.
    pub fn estimate_channel(
        &self,
        received_freq: &[(f64, f64)],
        symbol_idx: usize,
    ) -> Vec<SubcarrierEstimate> {
        let pilots = self.pilot_cells(symbol_idx);
        let n = self.params.fft_size;

        let mut estimates = Vec::with_capacity(pilots.len());
        for pilot in &pilots {
            let bin = self.carrier_to_bin(pilot.carrier);
            if bin < n && bin < received_freq.len() {
                // H(k) = Y(k) / X(k)
                let h = cdiv(received_freq[bin], pilot.value);
                estimates.push(SubcarrierEstimate {
                    carrier: pilot.carrier,
                    h,
                });
            }
        }
        estimates
    }

    /// Interpolate channel estimates across all data carriers using linear
    /// interpolation between pilot positions.
    pub fn interpolate_channel(
        &self,
        pilot_estimates: &[SubcarrierEstimate],
        symbol_idx: usize,
    ) -> Vec<SubcarrierEstimate> {
        if pilot_estimates.is_empty() {
            return Vec::new();
        }

        let data_carr = self.data_carriers(symbol_idx);
        let mut result = Vec::with_capacity(data_carr.len());

        // Sort pilot estimates by carrier index
        let mut sorted_pilots: Vec<_> = pilot_estimates.to_vec();
        sorted_pilots.sort_by_key(|e| e.carrier);

        for &k in &data_carr {
            // Find surrounding pilots
            let h = Self::lerp_channel(&sorted_pilots, k);
            result.push(SubcarrierEstimate { carrier: k, h });
        }
        result
    }

    /// Linear interpolation of channel estimate at carrier `k`.
    fn lerp_channel(sorted: &[SubcarrierEstimate], k: isize) -> (f64, f64) {
        // Find the last pilot <= k and first pilot >= k
        let mut left: Option<&SubcarrierEstimate> = None;
        let mut right: Option<&SubcarrierEstimate> = None;

        for est in sorted {
            if est.carrier <= k {
                left = Some(est);
            }
            if est.carrier >= k && right.is_none() {
                right = Some(est);
            }
        }

        match (left, right) {
            (Some(l), Some(r)) => {
                if l.carrier == r.carrier {
                    l.h
                } else {
                    let alpha =
                        (k - l.carrier) as f64 / (r.carrier - l.carrier) as f64;
                    (
                        l.h.0 + alpha * (r.h.0 - l.h.0),
                        l.h.1 + alpha * (r.h.1 - l.h.1),
                    )
                }
            }
            (Some(l), None) => l.h,
            (None, Some(r)) => r.h,
            (None, None) => (1.0, 0.0),
        }
    }

    /// Apply channel equalisation (zero-forcing) to received data carriers.
    pub fn equalize(
        &self,
        received_data: &[(f64, f64)],
        channel_estimates: &[SubcarrierEstimate],
    ) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(received_data.len());
        for (i, &rx) in received_data.iter().enumerate() {
            let h = if i < channel_estimates.len() {
                channel_estimates[i].h
            } else {
                (1.0, 0.0)
            };
            out.push(cdiv(rx, h));
        }
        out
    }

    // -- Synchronization helpers --------------------------------------------

    /// Time synchronization using cyclic-prefix correlation.
    ///
    /// Scans the signal for the symbol boundary that maximises the correlation
    /// between CP and the tail of the useful part.
    pub fn time_sync(&self, signal: &[(f64, f64)]) -> TimeSyncResult {
        let n = self.params.fft_size;
        let gi = self.params.guard_samples;
        let sym_len = self.params.symbol_samples;

        if signal.len() < sym_len {
            return TimeSyncResult {
                offset: 0,
                peak_correlation: 0.0,
            };
        }

        let search = signal.len().saturating_sub(sym_len);
        let mut best_offset = 0;
        let mut best_corr = 0.0_f64;

        for d in 0..search {
            let mut corr_val = (0.0, 0.0);
            let mut pwr = 0.0;
            for j in 0..gi {
                let a = signal[d + j];
                let b = signal[d + n + j];
                corr_val = cadd(corr_val, cmul(a, conj(b)));
                pwr += cabs2(a) + cabs2(b);
            }
            let c = if pwr > 1e-30 {
                2.0 * cabs(corr_val) / pwr
            } else {
                0.0
            };
            if c > best_corr {
                best_corr = c;
                best_offset = d;
            }
        }

        TimeSyncResult {
            offset: best_offset,
            peak_correlation: best_corr,
        }
    }

    /// Estimate fractional frequency offset from phase of the CP correlation.
    pub fn fractional_freq_offset(&self, signal: &[(f64, f64)], symbol_start: usize) -> f64 {
        let n = self.params.fft_size;
        let gi = self.params.guard_samples;

        if symbol_start + n + gi > signal.len() {
            return 0.0;
        }

        let mut corr = (0.0, 0.0);
        for j in 0..gi {
            let a = signal[symbol_start + j];
            let b = signal[symbol_start + n + j];
            corr = cadd(corr, cmul(a, conj(b)));
        }

        // Phase of correlation -> fractional offset in subcarrier spacings
        let phase = corr.1.atan2(corr.0);
        -phase / (2.0 * PI) * (n as f64 / gi as f64)
    }

    /// Estimate integer frequency offset from pilot correlation.
    ///
    /// Tries candidate integer offsets and picks the one that maximises
    /// pilot correlation.
    pub fn integer_freq_offset(
        &self,
        received_freq: &[(f64, f64)],
        symbol_idx: usize,
        max_offset: isize,
    ) -> FreqSyncResult {
        let pilots = self.pilot_cells(symbol_idx);
        let n = self.params.fft_size;

        let mut best_offset: isize = 0;
        let mut best_corr = 0.0_f64;

        for delta in -max_offset..=max_offset {
            let mut corr = 0.0;
            for pilot in &pilots {
                let shifted_carrier = pilot.carrier + delta;
                let bin = self.carrier_to_bin_raw(shifted_carrier, n);
                if let Some(b) = bin {
                    if b < received_freq.len() {
                        corr += cmul(received_freq[b], conj(pilot.value)).0;
                    }
                }
            }
            if corr > best_corr {
                best_corr = corr;
                best_offset = delta;
            }
        }

        FreqSyncResult {
            fractional_offset: 0.0,
            integer_offset: best_offset,
        }
    }

    // -- Helpers ------------------------------------------------------------

    /// Convert a subcarrier index (can be negative) to an FFT bin index.
    fn carrier_to_bin(&self, k: isize) -> usize {
        let n = self.params.fft_size as isize;
        ((k % n + n) % n) as usize
    }

    /// Carrier-to-bin with bounds checking.
    fn carrier_to_bin_raw(&self, k: isize, n: usize) -> Option<usize> {
        let ni = n as isize;
        let bin = ((k % ni + ni) % ni) as usize;
        if bin < n {
            Some(bin)
        } else {
            None
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Helper --------------------------------------------------------------

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn capprox_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    // -- 1. Mode parameters --------------------------------------------------

    #[test]
    fn test_mode_a_params() {
        let p = DrmOfdmProcessor::new(RobustnessMode::A);
        let params = p.params();
        assert_eq!(params.fft_size, 256);
        assert_eq!(params.guard_samples, 32);
        assert_eq!(params.carrier_count, 208);
        assert_eq!(params.symbol_samples, 288);
        assert!(approx_eq(params.subcarrier_spacing_hz, 48000.0 / 256.0, 0.01));
    }

    #[test]
    fn test_mode_b_params() {
        let p = DrmOfdmProcessor::new(RobustnessMode::B);
        let params = p.params();
        assert_eq!(params.fft_size, 256);
        assert_eq!(params.guard_samples, 64);
        assert_eq!(params.symbol_samples, 320);
    }

    #[test]
    fn test_mode_c_params() {
        let p = DrmOfdmProcessor::new(RobustnessMode::C);
        assert_eq!(p.params().fft_size, 128);
        assert_eq!(p.params().guard_samples, 32);
        assert_eq!(p.params().carrier_count, 114);
    }

    #[test]
    fn test_mode_d_params() {
        let p = DrmOfdmProcessor::new(RobustnessMode::D);
        assert_eq!(p.params().fft_size, 128);
        assert_eq!(p.params().guard_samples, 44);
        assert_eq!(p.params().carrier_count, 98);
    }

    // -- 2. QAM mapping / demapping -----------------------------------------

    #[test]
    fn test_qam4_roundtrip() {
        let bits = vec![false, true, true, false, false, false, true, true];
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam4);
        assert_eq!(syms.len(), 4);
        let recovered = DrmOfdmProcessor::qam_demap(&syms, QamOrder::Qam4);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_qam16_roundtrip() {
        // All 16 combinations
        let mut bits = Vec::new();
        for i in 0..16u8 {
            for b in (0..4).rev() {
                bits.push((i >> b) & 1 == 1);
            }
        }
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam16);
        assert_eq!(syms.len(), 16);
        let recovered = DrmOfdmProcessor::qam_demap(&syms, QamOrder::Qam16);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_qam64_roundtrip() {
        // All 64 combinations
        let mut bits = Vec::new();
        for i in 0..64u8 {
            for b in (0..6).rev() {
                bits.push((i >> b) & 1 == 1);
            }
        }
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam64);
        assert_eq!(syms.len(), 64);
        let recovered = DrmOfdmProcessor::qam_demap(&syms, QamOrder::Qam64);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_qam4_unit_power() {
        // Average power should be approximately 1
        let bits = vec![false, false, false, true, true, false, true, true];
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam4);
        let avg_power: f64 = syms.iter().map(|s| cabs2(*s)).sum::<f64>() / syms.len() as f64;
        assert!(approx_eq(avg_power, 1.0, 0.01));
    }

    #[test]
    fn test_qam16_unit_power() {
        // Average power over all constellation points should be ~1
        let mut bits = Vec::new();
        for i in 0..16u8 {
            for b in (0..4).rev() {
                bits.push((i >> b) & 1 == 1);
            }
        }
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam16);
        let avg_power: f64 = syms.iter().map(|s| cabs2(*s)).sum::<f64>() / syms.len() as f64;
        assert!(approx_eq(avg_power, 1.0, 0.01));
    }

    // -- 3. OFDM symbol generation ------------------------------------------

    #[test]
    fn test_ofdm_symbol_length() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let data = vec![(1.0, 0.0); 50];
        let sym = proc.generate_ofdm_symbol(&data, 0);
        assert_eq!(sym.len(), proc.params().fft_size + proc.params().guard_samples);
    }

    #[test]
    fn test_cyclic_prefix_matches_tail() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::A);
        let data = vec![(0.5, 0.3); 100];
        let sym = proc.generate_ofdm_symbol(&data, 0);
        let gi = proc.params().guard_samples;
        let n = proc.params().fft_size;
        // CP (first `gi` samples) should equal last `gi` samples of useful part
        for i in 0..gi {
            assert!(
                capprox_eq(sym[i], sym[gi + n - gi + i], 1e-10),
                "CP mismatch at index {}: {:?} vs {:?}",
                i,
                sym[i],
                sym[gi + n - gi + i]
            );
        }
    }

    // -- 4. OFDM roundtrip --------------------------------------------------

    #[test]
    fn test_ofdm_moddemod_roundtrip() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let dc = proc.data_carriers(0);
        // Create known data on each data carrier
        let data: Vec<(f64, f64)> = dc
            .iter()
            .enumerate()
            .map(|(i, _)| {
                let phase = 2.0 * PI * i as f64 / dc.len() as f64;
                (phase.cos(), phase.sin())
            })
            .collect();

        let sym = proc.generate_ofdm_symbol(&data, 0);
        let recovered = proc.demodulate_ofdm_symbol(&sym, 0);

        assert_eq!(recovered.len(), data.len());
        for (i, (a, b)) in recovered.iter().zip(data.iter()).enumerate() {
            assert!(
                capprox_eq(*a, *b, 1e-8),
                "Mismatch at carrier {}: got {:?}, expected {:?}",
                i,
                a,
                b
            );
        }
    }

    // -- 5. Pilot pattern ---------------------------------------------------

    #[test]
    fn test_pilots_non_empty() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let pilots = proc.pilot_cells(0);
        assert!(!pilots.is_empty(), "Pilot pattern should not be empty");
    }

    #[test]
    fn test_pilots_no_dc() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::A);
        for sym in 0..10 {
            let pilots = proc.pilot_cells(sym);
            for p in &pilots {
                assert_ne!(p.carrier, 0, "No pilot should be placed on DC");
            }
        }
    }

    #[test]
    fn test_data_carriers_exclude_pilots() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let pilots: std::collections::HashSet<isize> =
            proc.pilot_cells(0).iter().map(|p| p.carrier).collect();
        let data = proc.data_carriers(0);
        for k in &data {
            assert!(!pilots.contains(k), "Data carrier {} is also a pilot", k);
        }
    }

    // -- 6. Mode detection --------------------------------------------------

    #[test]
    fn test_mode_detection() {
        // Generate a signal from Mode A and verify detection picks it
        let proc = DrmOfdmProcessor::new(RobustnessMode::A);
        let data = vec![(1.0, 0.0); 100];
        // Generate multiple symbols for better detection
        let mut signal = Vec::new();
        for s in 0..8 {
            signal.extend_from_slice(&proc.generate_ofdm_symbol(&data, s));
        }

        let result = DrmOfdmProcessor::detect_mode(&signal);
        assert_eq!(result.mode, RobustnessMode::A);
        assert!(result.correlation > 0.5, "Correlation too low: {}", result.correlation);
    }

    // -- 7. Channel estimation ----------------------------------------------

    #[test]
    fn test_channel_estimation_identity() {
        // No channel distortion -> estimates should be ~(1, 0)
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let data = vec![(0.5, 0.5); 100];
        let sym = proc.generate_ofdm_symbol(&data, 0);
        // Demod -> get frequency domain
        let gi = proc.params().guard_samples;
        let n = proc.params().fft_size;
        let freq = fft(&sym[gi..gi + n]);

        let estimates = proc.estimate_channel(&freq, 0);
        assert!(!estimates.is_empty());
        for est in &estimates {
            assert!(
                capprox_eq(est.h, (1.0, 0.0), 1e-8),
                "Channel est at k={}: {:?} (expected ~(1,0))",
                est.carrier,
                est.h
            );
        }
    }

    // -- 8. Synchronization -------------------------------------------------

    #[test]
    fn test_time_sync_detects_boundary() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let data = vec![(1.0, 0.0); 100];
        // Prepend some noise
        let mut signal: Vec<(f64, f64)> = (0..50).map(|_| (0.001, 0.0)).collect();
        let sym = proc.generate_ofdm_symbol(&data, 0);
        signal.extend_from_slice(&sym);
        // Add padding so search works
        signal.extend(vec![(0.0, 0.0); 100]);

        let ts = proc.time_sync(&signal);
        // Should find the symbol near offset 50
        assert!(
            (ts.offset as isize - 50).unsigned_abs() <= 5,
            "Expected offset ~50, got {}",
            ts.offset
        );
        assert!(ts.peak_correlation > 0.5);
    }

    #[test]
    fn test_fractional_freq_offset_zero() {
        // No frequency offset -> estimate should be ~0
        let proc = DrmOfdmProcessor::new(RobustnessMode::A);
        let data = vec![(1.0, 0.0); 100];
        let sym = proc.generate_ofdm_symbol(&data, 0);
        let fo = proc.fractional_freq_offset(&sym, 0);
        assert!(
            fo.abs() < 0.1,
            "Expected near-zero fractional offset, got {}",
            fo
        );
    }

    // -- 9. Spectral occupancy ----------------------------------------------

    #[test]
    fn test_spectral_occupancy_bandwidths() {
        assert!(approx_eq(SpectralOccupancy::So0.bandwidth_hz(), 4500.0, 0.1));
        assert!(approx_eq(SpectralOccupancy::So1.bandwidth_hz(), 5000.0, 0.1));
        assert!(approx_eq(SpectralOccupancy::So2.bandwidth_hz(), 9000.0, 0.1));
        assert!(approx_eq(SpectralOccupancy::So3.bandwidth_hz(), 10000.0, 0.1));
        assert!(approx_eq(SpectralOccupancy::So4.bandwidth_hz(), 18000.0, 0.1));
        assert!(approx_eq(SpectralOccupancy::So5.bandwidth_hz(), 20000.0, 0.1));
    }

    #[test]
    fn test_with_spectral_occupancy() {
        let proc = DrmOfdmProcessor::with_spectral_occupancy(
            RobustnessMode::C,
            SpectralOccupancy::So4,
        );
        assert_eq!(proc.spectral_occupancy(), SpectralOccupancy::So4);
        assert_eq!(proc.params().mode, RobustnessMode::C);
    }

    // -- 10. Frame generation -----------------------------------------------

    #[test]
    fn test_generate_frame_length() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let data = vec![(1.0, 0.0); 500];
        let frame = proc.generate_frame(&data, 3);
        assert_eq!(frame.len(), 3 * proc.params().symbol_samples);
    }

    // -- 11. QAM bits per symbol --------------------------------------------

    #[test]
    fn test_bits_per_symbol() {
        assert_eq!(QamOrder::Qam4.bits_per_symbol(), 2);
        assert_eq!(QamOrder::Qam16.bits_per_symbol(), 4);
        assert_eq!(QamOrder::Qam64.bits_per_symbol(), 6);
    }

    // -- 12. FFT basic sanity -----------------------------------------------

    #[test]
    fn test_fft_ifft_roundtrip() {
        let signal: Vec<(f64, f64)> = (0..64)
            .map(|i| {
                let t = i as f64 / 64.0;
                ((2.0 * PI * 3.0 * t).cos(), (2.0 * PI * 3.0 * t).sin())
            })
            .collect();
        let freq = fft(&signal);
        let recovered = ifft(&freq);
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!(capprox_eq(*a, *b, 1e-10));
        }
    }

    // -- 13. Channel interpolation ------------------------------------------

    #[test]
    fn test_channel_interpolation() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        // Artificial pilot estimates
        let pilot_estimates = vec![
            SubcarrierEstimate {
                carrier: -90,
                h: (1.0, 0.0),
            },
            SubcarrierEstimate {
                carrier: 0,
                h: (0.5, 0.5),
            },
            SubcarrierEstimate {
                carrier: 90,
                h: (0.0, 1.0),
            },
        ];
        let interp = proc.interpolate_channel(&pilot_estimates, 0);
        assert!(!interp.is_empty());
        // Values should be interpolated between the pilot estimates
        // Just check no NaN/Inf
        for est in &interp {
            assert!(est.h.0.is_finite());
            assert!(est.h.1.is_finite());
        }
    }

    // -- 14. Equalisation ---------------------------------------------------

    #[test]
    fn test_equalize_identity() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::A);
        let data = vec![(1.0, 0.5), (-0.3, 0.7), (0.0, -1.0)];
        let channel = vec![
            SubcarrierEstimate {
                carrier: 1,
                h: (1.0, 0.0),
            },
            SubcarrierEstimate {
                carrier: 2,
                h: (1.0, 0.0),
            },
            SubcarrierEstimate {
                carrier: 3,
                h: (1.0, 0.0),
            },
        ];
        let eq = proc.equalize(&data, &channel);
        for (a, b) in eq.iter().zip(data.iter()) {
            assert!(capprox_eq(*a, *b, 1e-10));
        }
    }

    // -- 15. Integer frequency offset detection ------------------------------

    #[test]
    fn test_integer_freq_offset_zero() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let data = vec![(1.0, 0.0); 100];
        let sym = proc.generate_ofdm_symbol(&data, 0);
        let gi = proc.params().guard_samples;
        let n = proc.params().fft_size;
        let freq = fft(&sym[gi..gi + n]);

        let result = proc.integer_freq_offset(&freq, 0, 3);
        assert_eq!(
            result.integer_offset, 0,
            "Expected zero integer offset, got {}",
            result.integer_offset
        );
    }

    // -- 16. QAM64 unit power -----------------------------------------------

    #[test]
    fn test_qam64_unit_power() {
        let mut bits = Vec::new();
        for i in 0..64u8 {
            for b in (0..6).rev() {
                bits.push((i >> b) & 1 == 1);
            }
        }
        let syms = DrmOfdmProcessor::qam_map(&bits, QamOrder::Qam64);
        let avg_power: f64 = syms.iter().map(|s| cabs2(*s)).sum::<f64>() / syms.len() as f64;
        assert!(approx_eq(avg_power, 1.0, 0.01));
    }

    // -- 17. Carrier-to-bin mapping -----------------------------------------

    #[test]
    fn test_carrier_to_bin_positive_and_negative() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::B);
        let n = proc.params().fft_size;
        // Positive carrier maps directly
        assert_eq!(proc.carrier_to_bin(1), 1);
        assert_eq!(proc.carrier_to_bin(10), 10);
        // Negative carrier wraps around
        assert_eq!(proc.carrier_to_bin(-1), n - 1);
        assert_eq!(proc.carrier_to_bin(-10), n - 10);
        // DC
        assert_eq!(proc.carrier_to_bin(0), 0);
    }

    // -- 18. Multiple symbols demod consistency -----------------------------

    #[test]
    fn test_multi_symbol_frame_demod() {
        let proc = DrmOfdmProcessor::new(RobustnessMode::C);
        let dc0 = proc.data_carriers(0);
        let dc1 = proc.data_carriers(1);
        // Fill data for two symbols
        let data0: Vec<(f64, f64)> = dc0.iter().map(|_| (0.7, 0.3)).collect();
        let data1: Vec<(f64, f64)> = dc1.iter().map(|_| (-0.5, 0.5)).collect();
        let mut all_data = data0.clone();
        all_data.extend_from_slice(&data1);

        let frame = proc.generate_frame(&all_data, 2);
        let sym_len = proc.params().symbol_samples;

        // Demod first symbol
        let rec0 = proc.demodulate_ofdm_symbol(&frame[0..sym_len], 0);
        assert_eq!(rec0.len(), data0.len());
        for (a, b) in rec0.iter().zip(data0.iter()) {
            assert!(capprox_eq(*a, *b, 1e-8));
        }

        // Demod second symbol
        let rec1 = proc.demodulate_ofdm_symbol(&frame[sym_len..2 * sym_len], 1);
        assert_eq!(rec1.len(), data1.len());
        for (a, b) in rec1.iter().zip(data1.iter()) {
            assert!(capprox_eq(*a, *b, 1e-8));
        }
    }
}
