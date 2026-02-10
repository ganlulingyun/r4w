//! Spectral Mask Checking and Enforcement for Regulatory Compliance
//!
//! Provides tools for defining, checking, and enforcing spectral emission masks
//! as required by regulatory bodies such as FCC (Part 15) and ETSI. A spectral
//! mask defines the maximum permissible power spectral density (PSD) as a function
//! of frequency offset from the carrier. This module supports:
//!
//! - **Mask definition** via piecewise-linear breakpoints ([`MaskPoint`])
//! - **Compliance checking** against measured or simulated PSD ([`SpectralMask::check_compliance`])
//! - **Mask enforcement** by clamping frequency-domain bins that exceed the limit ([`MaskEnforcer`])
//! - **Factory functions** for common masks (Wi-Fi 20 MHz, LTE 10 MHz, FCC Part 15)
//!
//! # Example
//!
//! ```rust
//! use r4w_core::spectral_mask::{SpectralMask, MaskPoint, MaskEnforcer};
//!
//! // Define a simple emission mask
//! let mask = SpectralMask::new("simple", vec![
//!     MaskPoint { freq_offset_hz: -20e6, power_db: -40.0 },
//!     MaskPoint { freq_offset_hz: -10e6, power_db: -20.0 },
//!     MaskPoint { freq_offset_hz:   0.0, power_db:   0.0 },
//!     MaskPoint { freq_offset_hz:  10e6, power_db: -20.0 },
//!     MaskPoint { freq_offset_hz:  20e6, power_db: -40.0 },
//! ]);
//!
//! // Check compliance of a measured spectrum
//! let spectrum = vec![
//!     (-15e6, -35.0),
//!     ( -5e6, -10.0),
//!     (  0.0,  -1.0),
//!     (  5e6, -10.0),
//!     ( 15e6, -35.0),
//! ];
//! let result = mask.check_compliance(&spectrum);
//! assert!(result.compliant);
//! ```

#[allow(unused_imports)]
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Core types
// ---------------------------------------------------------------------------

/// A single breakpoint in a spectral mask, defining the power limit at a
/// particular frequency offset from the carrier centre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaskPoint {
    /// Frequency offset from carrier centre (Hz). Negative values represent
    /// offsets below the carrier.
    pub freq_offset_hz: f64,
    /// Maximum permissible power spectral density at this offset (dB relative
    /// to the in-band PSD, i.e. 0 dB = carrier level).
    pub power_db: f64,
}

/// A recorded violation where the signal PSD exceeds the spectral mask.
#[derive(Debug, Clone, PartialEq)]
pub struct MaskViolation {
    /// Frequency offset where the violation occurred (Hz).
    pub freq_offset_hz: f64,
    /// Measured signal power at this offset (dB).
    pub signal_db: f64,
    /// Mask limit at this offset (dB).
    pub mask_db: f64,
    /// Amount by which the signal exceeds the mask (dB, positive = violation).
    pub violation_db: f64,
}

/// Result of a spectral mask compliance check.
#[derive(Debug, Clone)]
pub struct MaskResult {
    /// `true` if the signal meets the mask at every measured point.
    pub compliant: bool,
    /// List of individual violations (empty when compliant).
    pub violations: Vec<MaskViolation>,
    /// Worst-case margin (dB). Positive means the signal is within the mask;
    /// negative means the signal exceeds it. For an empty spectrum this is
    /// `f64::INFINITY`.
    pub margin_db: f64,
}

// ---------------------------------------------------------------------------
// SpectralMask
// ---------------------------------------------------------------------------

/// A piecewise-linear spectral emission mask.
///
/// Points are stored sorted by ascending frequency offset. The mask limit
/// between adjacent breakpoints is obtained by linear interpolation of the
/// dB values. Outside the defined range the limit is taken from the nearest
/// endpoint (flat extrapolation).
#[derive(Debug, Clone)]
pub struct SpectralMask {
    /// Breakpoints sorted by `freq_offset_hz`.
    pub points: Vec<MaskPoint>,
    /// Human-readable name (e.g. "IEEE 802.11a 20 MHz").
    pub name: String,
}

impl SpectralMask {
    /// Create a new spectral mask. The provided points are sorted by frequency
    /// offset internally; duplicates are permitted.
    pub fn new(name: &str, mut points: Vec<MaskPoint>) -> Self {
        points.sort_by(|a, b| {
            a.freq_offset_hz
                .partial_cmp(&b.freq_offset_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        Self {
            points,
            name: name.to_string(),
        }
    }

    /// Linearly interpolate the mask limit at `freq_offset_hz`.
    ///
    /// - If `freq_offset_hz` falls between two breakpoints the dB value is
    ///   linearly interpolated.
    /// - If it falls outside the defined range the nearest endpoint value is
    ///   returned (flat extrapolation).
    /// - Returns `f64::INFINITY` (no limit) when the mask has no points.
    pub fn limit_at(&self, freq_offset_hz: f64) -> f64 {
        if self.points.is_empty() {
            return f64::INFINITY;
        }

        // Below the lowest breakpoint — flat extrapolation.
        if freq_offset_hz <= self.points[0].freq_offset_hz {
            return self.points[0].power_db;
        }

        // Above the highest breakpoint — flat extrapolation.
        let last = self.points.len() - 1;
        if freq_offset_hz >= self.points[last].freq_offset_hz {
            return self.points[last].power_db;
        }

        // Find the bracketing segment and interpolate.
        for i in 0..last {
            let p0 = &self.points[i];
            let p1 = &self.points[i + 1];
            if freq_offset_hz >= p0.freq_offset_hz && freq_offset_hz <= p1.freq_offset_hz {
                let span = p1.freq_offset_hz - p0.freq_offset_hz;
                if span.abs() < 1e-30 {
                    // Degenerate segment (two points at same freq) — return
                    // the more restrictive (lower) limit.
                    return p0.power_db.min(p1.power_db);
                }
                let t = (freq_offset_hz - p0.freq_offset_hz) / span;
                return p0.power_db + t * (p1.power_db - p0.power_db);
            }
        }

        // Should not be reached, but fall back to the last point.
        self.points[last].power_db
    }

    /// Check whether a measured (or simulated) power spectrum complies with
    /// this mask.
    ///
    /// `spectrum_db` is a slice of `(freq_offset_hz, power_db)` pairs
    /// representing the signal's power spectral density. Each pair is compared
    /// against the interpolated mask limit at its frequency offset.
    pub fn check_compliance(&self, spectrum_db: &[(f64, f64)]) -> MaskResult {
        if spectrum_db.is_empty() {
            return MaskResult {
                compliant: true,
                violations: Vec::new(),
                margin_db: f64::INFINITY,
            };
        }

        let mut violations = Vec::new();
        let mut worst_margin = f64::INFINITY;

        for &(freq, power) in spectrum_db {
            let limit = self.limit_at(freq);
            let margin = limit - power; // positive = within mask

            if margin < worst_margin {
                worst_margin = margin;
            }

            if power > limit {
                violations.push(MaskViolation {
                    freq_offset_hz: freq,
                    signal_db: power,
                    mask_db: limit,
                    violation_db: power - limit,
                });
            }
        }

        MaskResult {
            compliant: violations.is_empty(),
            violations,
            margin_db: worst_margin,
        }
    }
}

// ---------------------------------------------------------------------------
// MaskEnforcer
// ---------------------------------------------------------------------------

/// Enforces a spectral mask by clamping frequency-domain bins that exceed
/// the mask limit.
///
/// The workflow is:
/// 1. Accept a frequency-domain representation of the signal as
///    `(freq_offset_hz, power_db)` pairs.
/// 2. For each bin, compute the mask limit via linear interpolation.
/// 3. If the bin's power exceeds the limit, clamp it to the limit.
/// 4. Return the (possibly modified) spectrum.
///
/// This is a simplified spectral-domain enforcer. A full implementation would
/// perform FFT on time-domain samples, clamp, and IFFT back; the interface
/// here works on already-transformed data so the caller controls the FFT
/// details (size, window, overlap).
#[derive(Debug, Clone)]
pub struct MaskEnforcer {
    /// The spectral mask to enforce.
    mask: SpectralMask,
    /// Nominal sample rate (Hz). Stored for documentation / future use.
    #[allow(dead_code)]
    sample_rate: f64,
    /// FFT size used for the frequency-domain representation.
    #[allow(dead_code)]
    fft_size: usize,
}

impl MaskEnforcer {
    /// Create a new enforcer bound to the given mask, sample rate, and FFT
    /// size.
    pub fn new(mask: SpectralMask, sample_rate: f64, fft_size: usize) -> Self {
        Self {
            mask,
            sample_rate,
            fft_size,
        }
    }

    /// Enforce the mask on a frequency-domain spectrum.
    ///
    /// Each element of `input` is `(freq_offset_hz, power_db)`. Returns a new
    /// vector where any bin exceeding the mask has been clamped to the mask
    /// limit.
    pub fn enforce(&self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        input
            .iter()
            .map(|&(freq, power)| {
                let limit = self.mask.limit_at(freq);
                if power > limit {
                    (freq, limit)
                } else {
                    (freq, power)
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Factory functions for common masks
// ---------------------------------------------------------------------------

/// IEEE 802.11a/g/n 20 MHz spectral mask per IEEE Std 802.11-2020, Section
/// 17.3.9.6.1 (Transmit spectrum mask).
///
/// Breakpoints (symmetric about centre):
/// - 0 to +/-9 MHz:   0 dBr
/// - +/-9 to +/-11 MHz: -20 dBr
/// - +/-11 to +/-20 MHz: -28 dBr
/// - beyond +/-20 MHz: -40 dBr
pub fn wifi_mask_20mhz() -> SpectralMask {
    SpectralMask::new(
        "IEEE 802.11 20 MHz",
        vec![
            MaskPoint { freq_offset_hz: -30e6, power_db: -40.0 },
            MaskPoint { freq_offset_hz: -20e6, power_db: -40.0 },
            MaskPoint { freq_offset_hz: -11e6, power_db: -28.0 },
            MaskPoint { freq_offset_hz:  -9e6, power_db: -20.0 },
            MaskPoint { freq_offset_hz:   0.0, power_db:   0.0 },
            MaskPoint { freq_offset_hz:   9e6, power_db: -20.0 },
            MaskPoint { freq_offset_hz:  11e6, power_db: -28.0 },
            MaskPoint { freq_offset_hz:  20e6, power_db: -40.0 },
            MaskPoint { freq_offset_hz:  30e6, power_db: -40.0 },
        ],
    )
}

/// 3GPP LTE 10 MHz channel emission mask per 3GPP TS 36.104, Table 6.6.3.1-4.
///
/// Simplified symmetric breakpoints:
/// - 0 to +/-4.5 MHz (in-band): 0 dBr
/// - +/-4.5 to +/-5.0 MHz: -1 dBr (transition)
/// - +/-5.0 to +/-10 MHz:  -10 dBr
/// - +/-10 to +/-15 MHz:   -25 dBr
/// - beyond +/-15 MHz:     -40 dBr
pub fn lte_mask_10mhz() -> SpectralMask {
    SpectralMask::new(
        "3GPP LTE 10 MHz",
        vec![
            MaskPoint { freq_offset_hz: -20e6,  power_db: -40.0 },
            MaskPoint { freq_offset_hz: -15e6,  power_db: -25.0 },
            MaskPoint { freq_offset_hz: -10e6,  power_db: -10.0 },
            MaskPoint { freq_offset_hz:  -5e6,  power_db:  -1.0 },
            MaskPoint { freq_offset_hz: -4.5e6, power_db:   0.0 },
            MaskPoint { freq_offset_hz:   0.0,  power_db:   0.0 },
            MaskPoint { freq_offset_hz:  4.5e6, power_db:   0.0 },
            MaskPoint { freq_offset_hz:   5e6,  power_db:  -1.0 },
            MaskPoint { freq_offset_hz:  10e6,  power_db: -10.0 },
            MaskPoint { freq_offset_hz:  15e6,  power_db: -25.0 },
            MaskPoint { freq_offset_hz:  20e6,  power_db: -40.0 },
        ],
    )
}

/// FCC Part 15 generic emission mask for unlicensed devices.
///
/// Constructs a mask based on `center_freq_hz` and `bandwidth_hz`.
/// Inside the nominal bandwidth the limit is 0 dBr; outside it rolls off
/// to -20 dBr at 1.5x the bandwidth edge, and -40 dBr at 2.5x.
///
/// This is a simplified model; real Part 15 rules depend on frequency band
/// and device class.
pub fn fcc_part15_mask(center_freq_hz: f64, bandwidth_hz: f64) -> SpectralMask {
    // We express everything as offsets from center. The `center_freq_hz` is
    // embedded in the name for documentation.
    let _ = center_freq_hz; // used only for the name
    let half_bw = bandwidth_hz / 2.0;

    SpectralMask::new(
        &format!(
            "FCC Part 15 ({:.1} MHz, BW {:.1} kHz)",
            center_freq_hz / 1e6,
            bandwidth_hz / 1e3
        ),
        vec![
            MaskPoint { freq_offset_hz: -2.5 * half_bw, power_db: -40.0 },
            MaskPoint { freq_offset_hz: -1.5 * half_bw, power_db: -20.0 },
            MaskPoint { freq_offset_hz: -half_bw,        power_db:   0.0 },
            MaskPoint { freq_offset_hz:  0.0,            power_db:   0.0 },
            MaskPoint { freq_offset_hz:  half_bw,        power_db:   0.0 },
            MaskPoint { freq_offset_hz:  1.5 * half_bw,  power_db: -20.0 },
            MaskPoint { freq_offset_hz:  2.5 * half_bw,  power_db: -40.0 },
        ],
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a simple symmetric triangular mask for testing.
    fn test_mask() -> SpectralMask {
        SpectralMask::new(
            "test",
            vec![
                MaskPoint { freq_offset_hz: -10.0, power_db: -40.0 },
                MaskPoint { freq_offset_hz:  -5.0, power_db: -20.0 },
                MaskPoint { freq_offset_hz:   0.0, power_db:   0.0 },
                MaskPoint { freq_offset_hz:   5.0, power_db: -20.0 },
                MaskPoint { freq_offset_hz:  10.0, power_db: -40.0 },
            ],
        );
        // Intentionally re-create to exercise sorting with out-of-order input.
        SpectralMask::new(
            "test",
            vec![
                MaskPoint { freq_offset_hz:   5.0, power_db: -20.0 },
                MaskPoint { freq_offset_hz:  -5.0, power_db: -20.0 },
                MaskPoint { freq_offset_hz:  10.0, power_db: -40.0 },
                MaskPoint { freq_offset_hz:   0.0, power_db:   0.0 },
                MaskPoint { freq_offset_hz: -10.0, power_db: -40.0 },
            ],
        )
    }

    #[test]
    fn test_mask_interpolation() {
        let mask = test_mask();

        // At breakpoints.
        assert!((mask.limit_at(0.0) - 0.0).abs() < 1e-10);
        assert!((mask.limit_at(5.0) - (-20.0)).abs() < 1e-10);
        assert!((mask.limit_at(-10.0) - (-40.0)).abs() < 1e-10);

        // Midpoint between 0 and 5 Hz offset: should be -10 dB.
        assert!((mask.limit_at(2.5) - (-10.0)).abs() < 1e-10);

        // Midpoint between -5 and 0 Hz offset: should be -10 dB.
        assert!((mask.limit_at(-2.5) - (-10.0)).abs() < 1e-10);

        // Quarter point between 5 and 10 Hz offset: -20 + 0.5*(-20) = -30 dB.
        assert!((mask.limit_at(7.5) - (-30.0)).abs() < 1e-10);

        // Extrapolation beyond defined range.
        assert!((mask.limit_at(-100.0) - (-40.0)).abs() < 1e-10);
        assert!((mask.limit_at(100.0) - (-40.0)).abs() < 1e-10);
    }

    #[test]
    fn test_compliance_pass() {
        let mask = test_mask();

        // All points well within the mask.
        let spectrum = vec![
            (-8.0, -45.0),
            (-3.0, -15.0),
            ( 0.0,  -5.0),
            ( 3.0, -15.0),
            ( 8.0, -45.0),
        ];
        let result = mask.check_compliance(&spectrum);
        assert!(result.compliant);
        assert!(result.violations.is_empty());
        assert!(result.margin_db > 0.0);
    }

    #[test]
    fn test_compliance_fail() {
        let mask = test_mask();

        // Signal at 0 Hz offset is 5 dB, but mask limit is 0 dB => violation.
        let spectrum = vec![
            (0.0, 5.0),
        ];
        let result = mask.check_compliance(&spectrum);
        assert!(!result.compliant);
        assert_eq!(result.violations.len(), 1);
        assert!(result.margin_db < 0.0);
    }

    #[test]
    fn test_violations() {
        let mask = test_mask();

        // Two violations: at +7 Hz and at -7 Hz.
        // Mask at +/-7.5 = -30 dB; we put signal at -25 dB => 5 dB violation.
        let spectrum = vec![
            (-7.5, -25.0),
            ( 0.0,  -5.0),
            ( 7.5, -25.0),
        ];
        let result = mask.check_compliance(&spectrum);
        assert!(!result.compliant);
        assert_eq!(result.violations.len(), 2);

        for v in &result.violations {
            assert!((v.violation_db - 5.0).abs() < 1e-10);
            assert!((v.signal_db - (-25.0)).abs() < 1e-10);
            assert!((v.mask_db - (-30.0)).abs() < 1e-10);
        }
    }

    #[test]
    fn test_margin() {
        let mask = test_mask();

        // Single point at 0 Hz offset with -3 dB => margin = 0 - (-3) = 3 dB.
        let spectrum = vec![(0.0, -3.0)];
        let result = mask.check_compliance(&spectrum);
        assert!(result.compliant);
        assert!((result.margin_db - 3.0).abs() < 1e-10);

        // Two points: one with 3 dB margin, one with 1 dB margin.
        // Worst margin should be 1 dB.
        let spectrum2 = vec![
            (0.0, -3.0),  // margin = 3
            (5.0, -21.0), // mask = -20, margin = -20 - (-21) = 1
        ];
        let result2 = mask.check_compliance(&spectrum2);
        assert!(result2.compliant);
        assert!((result2.margin_db - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_enforce_clamps() {
        let mask = test_mask();
        let enforcer = MaskEnforcer::new(mask, 100.0, 256);

        let input = vec![
            ( 0.0,   5.0),  // exceeds 0 dBr limit => clamp to 0
            ( 5.0, -15.0),  // exceeds -20 dBr limit => clamp to -20
            ( 2.5, -15.0),  // within -10 dBr limit => no change
            (10.0, -50.0),  // within -40 dBr limit => no change
        ];

        let output = enforcer.enforce(&input);

        assert!((output[0].1 - 0.0).abs() < 1e-10,
            "bin at 0 Hz should be clamped to 0 dBr");
        assert!((output[1].1 - (-20.0)).abs() < 1e-10,
            "bin at 5 Hz should be clamped to -20 dBr");
        assert!((output[2].1 - (-15.0)).abs() < 1e-10,
            "bin at 2.5 Hz should be unchanged");
        assert!((output[3].1 - (-50.0)).abs() < 1e-10,
            "bin at 10 Hz should be unchanged");
    }

    #[test]
    fn test_wifi_mask() {
        let mask = wifi_mask_20mhz();

        // In-band (0 MHz offset) should allow 0 dBr.
        assert!((mask.limit_at(0.0) - 0.0).abs() < 1e-10);

        // At +/-9 MHz transition should be -20 dBr.
        assert!((mask.limit_at(9e6) - (-20.0)).abs() < 1e-10);

        // At +/-11 MHz should be -28 dBr.
        assert!((mask.limit_at(11e6) - (-28.0)).abs() < 1e-10);

        // At +/-20 MHz should be -40 dBr.
        assert!((mask.limit_at(20e6) - (-40.0)).abs() < 1e-10);

        // Far out-of-band should be -40 dBr.
        assert!((mask.limit_at(50e6) - (-40.0)).abs() < 1e-10);

        // Symmetry.
        assert!((mask.limit_at(-9e6) - mask.limit_at(9e6)).abs() < 1e-10);
    }

    #[test]
    fn test_lte_mask() {
        let mask = lte_mask_10mhz();

        // In-band centre.
        assert!((mask.limit_at(0.0) - 0.0).abs() < 1e-10);

        // In-band edge at 4.5 MHz.
        assert!((mask.limit_at(4.5e6) - 0.0).abs() < 1e-10);

        // Transition at 5 MHz.
        assert!((mask.limit_at(5e6) - (-1.0)).abs() < 1e-10);

        // Out-of-band at 10 MHz.
        assert!((mask.limit_at(10e6) - (-10.0)).abs() < 1e-10);

        // Far out-of-band at 15 MHz.
        assert!((mask.limit_at(15e6) - (-25.0)).abs() < 1e-10);

        // Beyond defined range.
        assert!((mask.limit_at(20e6) - (-40.0)).abs() < 1e-10);

        // Symmetry.
        assert!((mask.limit_at(-10e6) - mask.limit_at(10e6)).abs() < 1e-10);
    }

    #[test]
    fn test_fcc_mask() {
        let mask = fcc_part15_mask(915e6, 1e6); // 915 MHz, 1 MHz BW
        let half_bw = 0.5e6;

        // In-band at centre.
        assert!((mask.limit_at(0.0) - 0.0).abs() < 1e-10);

        // At band edge (0.5 MHz offset).
        assert!((mask.limit_at(half_bw) - 0.0).abs() < 1e-10);

        // At 1.5x band edge (0.75 MHz offset).
        assert!((mask.limit_at(1.5 * half_bw) - (-20.0)).abs() < 1e-10);

        // At 2.5x band edge (1.25 MHz offset).
        assert!((mask.limit_at(2.5 * half_bw) - (-40.0)).abs() < 1e-10);

        // Name should contain frequency and bandwidth info.
        assert!(mask.name.contains("915.0"));
        assert!(mask.name.contains("1000.0"));
    }

    #[test]
    fn test_empty_spectrum() {
        let mask = test_mask();
        let result = mask.check_compliance(&[]);

        assert!(result.compliant);
        assert!(result.violations.is_empty());
        assert!(result.margin_db.is_infinite());
        assert!(result.margin_db > 0.0);
    }
}
