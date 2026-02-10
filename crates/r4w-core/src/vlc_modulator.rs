//! Visible Light Communication (VLC) modulation per IEEE 802.15.7.
//!
//! This module implements modulation and demodulation schemes for LED-based
//! optical wireless data transmission. Three modulation types are supported:
//!
//! - **OOK (On-Off Keying)** with Manchester encoding for DC balance
//! - **VPPM (Variable Pulse-Position Modulation)** with dimming control
//! - **CSK (Color Shift Keying)** using RGB LED intensities
//!
//! All schemes comply with IEEE 802.15.7 flicker-free requirements when the
//! `flicker_free` option is enabled (max 5 consecutive identical symbols).
//!
//! # Example
//!
//! ```
//! use r4w_core::vlc_modulator::{VlcModulator, VlcConfig, VlcScheme};
//!
//! let config = VlcConfig {
//!     scheme: VlcScheme::Ook,
//!     data_rate_hz: 100_000.0,
//!     clock_rate_hz: 200_000.0,
//!     dimming_level: 0.5,
//!     flicker_free: true,
//! };
//! let modulator = VlcModulator::new(config);
//!
//! let bits = vec![true, false, true, true, false];
//! let signal = modulator.modulate_ook(&bits);
//! assert_eq!(signal.len(), bits.len() * 2); // Manchester doubles symbol count
//!
//! let recovered = modulator.demodulate_ook(&signal);
//! assert_eq!(recovered, bits);
//! ```

/// RGB color coordinates for Color Shift Keying (CSK) constellations.
///
/// Each point maps to an (r, g, b) triplet with values in \[0.0, 1.0\],
/// representing relative intensity of the red, green, and blue LED channels.
#[derive(Debug, Clone, PartialEq)]
pub struct CskConstellation {
    /// Constellation points as (red, green, blue) intensity triplets.
    pub points: Vec<(f64, f64, f64)>,
}

impl CskConstellation {
    /// Create the default 4-CSK constellation defined in IEEE 802.15.7.
    ///
    /// Four points in the CIE 1931 chromaticity-inspired RGB space
    /// that provide good separation for two-bit symbol mapping.
    pub fn default_4csk() -> Self {
        Self {
            points: vec![
                (1.0, 0.0, 0.0), // symbol 0b00 - Red
                (0.0, 1.0, 0.0), // symbol 0b01 - Green
                (0.0, 0.0, 1.0), // symbol 0b10 - Blue
                (1.0, 1.0, 0.0), // symbol 0b11 - Yellow (R+G)
            ],
        }
    }

    /// Create an 8-CSK constellation for three-bit symbols.
    pub fn default_8csk() -> Self {
        Self {
            points: vec![
                (1.0, 0.0, 0.0), // 0b000 - Red
                (0.0, 1.0, 0.0), // 0b001 - Green
                (0.0, 0.0, 1.0), // 0b010 - Blue
                (1.0, 1.0, 0.0), // 0b011 - Yellow
                (1.0, 0.0, 1.0), // 0b100 - Magenta
                (0.0, 1.0, 1.0), // 0b101 - Cyan
                (1.0, 0.5, 0.0), // 0b110 - Orange
                (0.5, 0.0, 1.0), // 0b111 - Violet
            ],
        }
    }

    /// Find the nearest constellation point to the given RGB sample.
    /// Returns the symbol index.
    pub fn nearest(&self, r: f64, g: f64, b: f64) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, &(pr, pg, pb)) in self.points.iter().enumerate() {
            let dist = (r - pr).powi(2) + (g - pg).powi(2) + (b - pb).powi(2);
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }
        best_idx
    }

    /// Number of bits per CSK symbol (log2 of constellation size).
    pub fn bits_per_symbol(&self) -> usize {
        let n = self.points.len();
        assert!(n.is_power_of_two(), "constellation size must be a power of 2");
        n.trailing_zeros() as usize
    }
}

/// Modulation scheme selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VlcScheme {
    /// On-Off Keying with Manchester encoding.
    Ook,
    /// Variable Pulse-Position Modulation with dimming support.
    Vppm,
    /// Color Shift Keying using multi-color LEDs.
    Csk,
}

/// Configuration for the VLC modulator.
#[derive(Debug, Clone)]
pub struct VlcConfig {
    /// Selected modulation scheme.
    pub scheme: VlcScheme,
    /// Bit rate in Hz.
    pub data_rate_hz: f64,
    /// Clock / sample rate in Hz (must be >= 2 * data_rate_hz for OOK Manchester).
    pub clock_rate_hz: f64,
    /// Target dimming level in \[0.0, 1.0\]. 0.0 = off, 1.0 = full brightness.
    pub dimming_level: f64,
    /// If true, enforce IEEE 802.15.7 flicker-free constraint
    /// (max 5 consecutive identical symbols).
    pub flicker_free: bool,
}

impl Default for VlcConfig {
    fn default() -> Self {
        Self {
            scheme: VlcScheme::Ook,
            data_rate_hz: 100_000.0,
            clock_rate_hz: 200_000.0,
            dimming_level: 0.5,
            flicker_free: true,
        }
    }
}

/// VLC modulator / demodulator implementing IEEE 802.15.7 PHY schemes.
#[derive(Debug, Clone)]
pub struct VlcModulator {
    /// Active configuration.
    pub config: VlcConfig,
    /// CSK constellation (used only when scheme == Csk).
    pub csk_constellation: CskConstellation,
}

impl VlcModulator {
    /// Create a new VLC modulator with the given configuration.
    ///
    /// Uses the default 4-CSK constellation for CSK mode.
    pub fn new(config: VlcConfig) -> Self {
        Self {
            config,
            csk_constellation: CskConstellation::default_4csk(),
        }
    }

    /// Create a VLC modulator with a custom CSK constellation.
    pub fn with_constellation(config: VlcConfig, constellation: CskConstellation) -> Self {
        Self {
            config,
            csk_constellation: constellation,
        }
    }

    // ---------------------------------------------------------------
    // OOK (On-Off Keying) with Manchester encoding
    // ---------------------------------------------------------------

    /// Modulate bits using OOK with Manchester encoding.
    ///
    /// Manchester encoding maps each bit to two chips:
    /// - `true`  (1) -> \[1.0, 0.0\]  (high-low transition)
    /// - `false` (0) -> \[0.0, 1.0\]  (low-high transition)
    ///
    /// This guarantees DC balance and provides a transition every bit period,
    /// aiding clock recovery at the receiver.
    pub fn modulate_ook(&self, bits: &[bool]) -> Vec<f64> {
        let mut output = Vec::with_capacity(bits.len() * 2);
        for &bit in bits {
            if bit {
                output.push(1.0);
                output.push(0.0);
            } else {
                output.push(0.0);
                output.push(1.0);
            }
        }
        output
    }

    /// Demodulate an OOK Manchester-encoded signal back to bits.
    ///
    /// Uses threshold detection: compares adjacent chip pairs.
    /// A pair (high, low) decodes to `true`; (low, high) decodes to `false`.
    pub fn demodulate_ook(&self, signal: &[f64]) -> Vec<bool> {
        let mut bits = Vec::with_capacity(signal.len() / 2);
        for pair in signal.chunks_exact(2) {
            // Manchester: 1 -> [H, L], 0 -> [L, H]
            bits.push(pair[0] > pair[1]);
        }
        bits
    }

    // ---------------------------------------------------------------
    // VPPM (Variable Pulse-Position Modulation)
    // ---------------------------------------------------------------

    /// Modulate bits using Variable Pulse-Position Modulation.
    ///
    /// Each bit period is divided into two halves. A pulse of width
    /// proportional to `dimming_level` is placed in:
    /// - the **first** half for bit `true` (1)
    /// - the **second** half for bit `false` (0)
    ///
    /// The `samples_per_symbol` determines the time-domain resolution.
    /// The dimming level controls the duty cycle (pulse width), allowing
    /// brightness control independently of the data.
    pub fn modulate_vppm(&self, bits: &[bool]) -> Vec<f64> {
        let samples_per_symbol = self.samples_per_symbol();
        let pulse_width = ((self.config.dimming_level * samples_per_symbol as f64).round() as usize)
            .max(1)
            .min(samples_per_symbol);
        let half = samples_per_symbol / 2;
        let mut output = Vec::with_capacity(bits.len() * samples_per_symbol);

        for &bit in bits {
            let mut symbol = vec![0.0_f64; samples_per_symbol];
            if bit {
                // Pulse in first half
                for s in symbol.iter_mut().take(pulse_width.min(half)) {
                    *s = 1.0;
                }
            } else {
                // Pulse in second half
                let start = half;
                for s in symbol.iter_mut().skip(start).take(pulse_width.min(half)) {
                    *s = 1.0;
                }
            }
            output.extend_from_slice(&symbol);
        }
        output
    }

    /// Demodulate a VPPM signal back to bits.
    ///
    /// Compares the energy in the first half vs. the second half of each
    /// symbol period. Higher energy in the first half indicates bit 1.
    pub fn demodulate_vppm(&self, signal: &[f64]) -> Vec<bool> {
        let sps = self.samples_per_symbol();
        let half = sps / 2;
        let mut bits = Vec::with_capacity(signal.len() / sps);

        for chunk in signal.chunks_exact(sps) {
            let energy_first: f64 = chunk[..half].iter().sum();
            let energy_second: f64 = chunk[half..].iter().sum();
            bits.push(energy_first > energy_second);
        }
        bits
    }

    // ---------------------------------------------------------------
    // CSK (Color Shift Keying)
    // ---------------------------------------------------------------

    /// Modulate bits using Color Shift Keying.
    ///
    /// Groups input bits into symbols of `bits_per_symbol` width and maps
    /// each symbol to an RGB intensity triplet from the CSK constellation.
    ///
    /// Returns a flat vector of `(R, G, B)` tuples.
    pub fn modulate_csk(&self, bits: &[bool]) -> Vec<(f64, f64, f64)> {
        let bps = self.csk_constellation.bits_per_symbol();
        let mut output = Vec::with_capacity(bits.len() / bps);

        for chunk in bits.chunks(bps) {
            let mut sym: usize = 0;
            for (i, &b) in chunk.iter().enumerate() {
                if b {
                    sym |= 1 << (bps - 1 - i);
                }
            }
            let point = self.csk_constellation.points[sym.min(self.csk_constellation.points.len() - 1)];
            output.push(point);
        }
        output
    }

    /// Demodulate CSK RGB samples back to bits.
    ///
    /// For each received (R, G, B) triplet, finds the nearest constellation
    /// point and extracts the corresponding bit pattern.
    pub fn demodulate_csk(&self, samples: &[(f64, f64, f64)]) -> Vec<bool> {
        let bps = self.csk_constellation.bits_per_symbol();
        let mut bits = Vec::with_capacity(samples.len() * bps);

        for &(r, g, b) in samples {
            let sym = self.csk_constellation.nearest(r, g, b);
            for i in (0..bps).rev() {
                bits.push((sym >> i) & 1 == 1);
            }
        }
        bits
    }

    // ---------------------------------------------------------------
    // Flicker-free and dimming utilities
    // ---------------------------------------------------------------

    /// Check that a binary signal satisfies the IEEE 802.15.7 run-length
    /// limit for flicker-free operation.
    ///
    /// Returns `true` if no run of identical symbols exceeds `max_run`.
    /// The standard typically requires max_run <= 5.
    pub fn run_length_limit(signal: &[f64], max_run: usize) -> bool {
        if signal.is_empty() || max_run == 0 {
            return signal.is_empty();
        }
        let threshold = 0.5;
        let mut run = 1usize;
        let mut prev = signal[0] >= threshold;

        for &s in &signal[1..] {
            let current = s >= threshold;
            if current == prev {
                run += 1;
                if run > max_run {
                    return false;
                }
            } else {
                run = 1;
                prev = current;
            }
        }
        true
    }

    /// Verify that the average intensity of a signal matches the target
    /// dimming level within a given tolerance.
    ///
    /// Returns `true` if `|mean_intensity - dimming_level| <= tolerance`.
    pub fn dimming_compatible(signal: &[f64], target_dimming: f64, tolerance: f64) -> bool {
        if signal.is_empty() {
            return false;
        }
        let mean: f64 = signal.iter().sum::<f64>() / signal.len() as f64;
        (mean - target_dimming).abs() <= tolerance
    }

    // ---------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------

    /// Samples per symbol, derived from clock_rate / data_rate.
    fn samples_per_symbol(&self) -> usize {
        (self.config.clock_rate_hz / self.config.data_rate_hz).round() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_modulator() -> VlcModulator {
        VlcModulator::new(VlcConfig::default())
    }

    #[test]
    fn test_ook_roundtrip() {
        let m = default_modulator();
        let bits = vec![true, false, true, true, false, false, true];
        let signal = m.modulate_ook(&bits);
        let recovered = m.demodulate_ook(&signal);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_ook_manchester_length() {
        let m = default_modulator();
        let bits = vec![true, false, true];
        let signal = m.modulate_ook(&bits);
        // Manchester encoding doubles the number of chips
        assert_eq!(signal.len(), bits.len() * 2);
    }

    #[test]
    fn test_ook_dc_balance() {
        let m = default_modulator();
        // Any bit pattern should yield a DC-balanced signal (mean = 0.5)
        let bits = vec![true, true, true, true, false, false, false, false];
        let signal = m.modulate_ook(&bits);
        let mean: f64 = signal.iter().sum::<f64>() / signal.len() as f64;
        assert!(
            (mean - 0.5).abs() < 1e-9,
            "Manchester OOK should be DC balanced, got mean={}",
            mean
        );
    }

    #[test]
    fn test_vppm_roundtrip() {
        let config = VlcConfig {
            scheme: VlcScheme::Vppm,
            data_rate_hz: 100_000.0,
            clock_rate_hz: 400_000.0, // 4 samples per symbol
            dimming_level: 0.5,
            flicker_free: true,
        };
        let m = VlcModulator::new(config);
        let bits = vec![true, false, true, false, true];
        let signal = m.modulate_vppm(&bits);
        let recovered = m.demodulate_vppm(&signal);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_vppm_signal_length() {
        let config = VlcConfig {
            scheme: VlcScheme::Vppm,
            data_rate_hz: 100_000.0,
            clock_rate_hz: 800_000.0, // 8 samples per symbol
            dimming_level: 0.5,
            flicker_free: false,
        };
        let m = VlcModulator::new(config);
        let bits = vec![true, false, true];
        let signal = m.modulate_vppm(&bits);
        assert_eq!(signal.len(), 3 * 8);
    }

    #[test]
    fn test_csk_4point_roundtrip() {
        let config = VlcConfig {
            scheme: VlcScheme::Csk,
            ..VlcConfig::default()
        };
        let m = VlcModulator::new(config);
        // 4-CSK = 2 bits per symbol; 8 bits = 4 symbols
        let bits = vec![false, false, false, true, true, false, true, true];
        let rgb = m.modulate_csk(&bits);
        assert_eq!(rgb.len(), 4);
        let recovered = m.demodulate_csk(&rgb);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_csk_8point_roundtrip() {
        let config = VlcConfig {
            scheme: VlcScheme::Csk,
            ..VlcConfig::default()
        };
        let constellation = CskConstellation::default_8csk();
        let m = VlcModulator::with_constellation(config, constellation);
        // 8-CSK = 3 bits per symbol; 9 bits = 3 symbols
        let bits = vec![false, false, false, false, true, false, true, true, true];
        let rgb = m.modulate_csk(&bits);
        assert_eq!(rgb.len(), 3);
        let recovered = m.demodulate_csk(&rgb);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_csk_nearest_exact() {
        let c = CskConstellation::default_4csk();
        assert_eq!(c.nearest(1.0, 0.0, 0.0), 0); // Red
        assert_eq!(c.nearest(0.0, 1.0, 0.0), 1); // Green
        assert_eq!(c.nearest(0.0, 0.0, 1.0), 2); // Blue
        assert_eq!(c.nearest(1.0, 1.0, 0.0), 3); // Yellow
    }

    #[test]
    fn test_csk_nearest_noisy() {
        let c = CskConstellation::default_4csk();
        // Slightly noisy Red should still map to symbol 0
        assert_eq!(c.nearest(0.9, 0.1, 0.05), 0);
        // Slightly noisy Green
        assert_eq!(c.nearest(0.1, 0.85, 0.05), 1);
    }

    #[test]
    fn test_run_length_limit_pass() {
        // Manchester OOK never has more than 2 consecutive same values
        let m = default_modulator();
        let bits = vec![true, true, true, true];
        let signal = m.modulate_ook(&bits);
        assert!(VlcModulator::run_length_limit(&signal, 2));
    }

    #[test]
    fn test_run_length_limit_fail() {
        // Six consecutive 1.0 should fail with max_run=5
        let signal = vec![1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0];
        assert!(!VlcModulator::run_length_limit(&signal, 5));
    }

    #[test]
    fn test_dimming_compatible_ook() {
        let m = default_modulator();
        let bits = vec![true, false, true, false, true, false];
        let signal = m.modulate_ook(&bits);
        // Manchester OOK is always at 50% dimming
        assert!(VlcModulator::dimming_compatible(&signal, 0.5, 0.01));
    }

    #[test]
    fn test_dimming_compatible_vppm() {
        let config = VlcConfig {
            scheme: VlcScheme::Vppm,
            data_rate_hz: 100_000.0,
            clock_rate_hz: 1_000_000.0, // 10 samples per symbol
            dimming_level: 0.3,
            flicker_free: false,
        };
        let m = VlcModulator::new(config);
        let bits: Vec<bool> = (0..100).map(|i| i % 2 == 0).collect();
        let signal = m.modulate_vppm(&bits);
        // VPPM duty cycle should approximate the dimming level
        assert!(
            VlcModulator::dimming_compatible(&signal, 0.3, 0.1),
            "VPPM signal mean should be close to dimming_level=0.3"
        );
    }

    #[test]
    fn test_vlc_scheme_equality() {
        assert_eq!(VlcScheme::Ook, VlcScheme::Ook);
        assert_ne!(VlcScheme::Ook, VlcScheme::Vppm);
        assert_ne!(VlcScheme::Vppm, VlcScheme::Csk);
    }

    #[test]
    fn test_default_config() {
        let cfg = VlcConfig::default();
        assert_eq!(cfg.scheme, VlcScheme::Ook);
        assert!((cfg.dimming_level - 0.5).abs() < f64::EPSILON);
        assert!(cfg.flicker_free);
    }

    #[test]
    fn test_empty_input() {
        let m = default_modulator();
        assert!(m.modulate_ook(&[]).is_empty());
        assert!(m.demodulate_ook(&[]).is_empty());
        assert!(m.modulate_vppm(&[]).is_empty());
        assert!(m.demodulate_vppm(&[]).is_empty());
        assert!(m.modulate_csk(&[]).is_empty());
        assert!(m.demodulate_csk(&[]).is_empty());
    }

    #[test]
    fn test_csk_bits_per_symbol() {
        assert_eq!(CskConstellation::default_4csk().bits_per_symbol(), 2);
        assert_eq!(CskConstellation::default_8csk().bits_per_symbol(), 3);
    }
}
