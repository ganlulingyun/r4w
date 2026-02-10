//! Signal Quality Metrics -- comprehensive signal quality measurement tools
//!
//! Provides a suite of measurements for characterizing digital communication
//! signals: PAPR, CCDF, occupied bandwidth, adjacent channel power ratio,
//! SNR estimation, DC offset, IQ imbalance, and phase noise analysis.
//! All functions operate on `(f64, f64)` I/Q sample tuples using only the
//! standard library.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::signal_quality_metrics::{SignalQuality, papr, ccdf};
//!
//! // Generate a constant-envelope tone
//! let n = 1024;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * i as f64 / 64.0;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let sq = SignalQuality::new();
//! let report = sq.analyze(&samples);
//! assert!(report.papr_db < 1.0, "CW tone should have near-zero PAPR");
//!
//! let p = papr(&samples);
//! assert!(p < 1.0);
//!
//! let dist = ccdf(&samples, 50);
//! assert_eq!(dist.len(), 50);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper: instantaneous power of an I/Q sample
// ---------------------------------------------------------------------------

#[inline]
fn power(s: &(f64, f64)) -> f64 {
    s.0 * s.0 + s.1 * s.1
}

/// Convert linear power to dBm (referenced to 1 mW into 50 ohms, but here we
/// treat power directly as watts-equivalent so dBm = 10*log10(p) + 30).
/// For our purposes we use the simpler convention: dBm = 10*log10(p / 0.001).
#[inline]
fn power_to_dbm(p: f64) -> f64 {
    if p < 1e-30 {
        return -300.0;
    }
    10.0 * p.log10() + 30.0
}

// ---------------------------------------------------------------------------
// QualityReport
// ---------------------------------------------------------------------------

/// Comprehensive signal quality report produced by [`SignalQuality::analyze`].
#[derive(Debug, Clone)]
pub struct QualityReport {
    /// Peak-to-average power ratio in dB.
    pub papr_db: f64,
    /// RMS power in dBm.
    pub rms_power_dbm: f64,
    /// Peak instantaneous power in dBm.
    pub peak_power_dbm: f64,
    /// Crest factor in dB (= PAPR / 2 in linear, but in dB = 20*log10(peak_amp/rms_amp)).
    pub crest_factor_db: f64,
    /// DC offset as (I mean, Q mean).
    pub dc_offset: (f64, f64),
    /// IQ power imbalance in dB: 10*log10(I_power / Q_power).
    pub iq_imbalance_db: f64,
    /// Number of phase discontinuities exceeding an internal threshold (pi/2 radians).
    pub phase_discontinuities: usize,
}

// ---------------------------------------------------------------------------
// SignalQuality
// ---------------------------------------------------------------------------

/// Signal quality analyzer that produces a [`QualityReport`].
#[derive(Debug, Clone)]
pub struct SignalQuality {
    /// Phase-jump threshold in radians for discontinuity detection.
    phase_threshold: f64,
}

impl SignalQuality {
    /// Create a new analyzer with default settings.
    ///
    /// The default phase discontinuity threshold is pi/2 radians.
    pub fn new() -> Self {
        Self {
            phase_threshold: PI / 2.0,
        }
    }

    /// Analyze a block of I/Q samples and return a [`QualityReport`].
    pub fn analyze(&self, samples: &[(f64, f64)]) -> QualityReport {
        if samples.is_empty() {
            return QualityReport {
                papr_db: 0.0,
                rms_power_dbm: -300.0,
                peak_power_dbm: -300.0,
                crest_factor_db: 0.0,
                dc_offset: (0.0, 0.0),
                iq_imbalance_db: 0.0,
                phase_discontinuities: 0,
            };
        }

        let n = samples.len() as f64;

        // Power statistics
        let powers: Vec<f64> = samples.iter().map(power).collect();
        let avg_power = powers.iter().sum::<f64>() / n;
        let peak_power = powers.iter().cloned().fold(0.0f64, f64::max);

        let papr_db = if avg_power < 1e-30 {
            0.0
        } else {
            10.0 * (peak_power / avg_power).log10()
        };

        let rms_amp = avg_power.sqrt();
        let peak_amp = peak_power.sqrt();

        let crest_factor_db = if rms_amp < 1e-30 {
            0.0
        } else {
            20.0 * (peak_amp / rms_amp).log10()
        };

        // DC offset
        let dc_i = samples.iter().map(|s| s.0).sum::<f64>() / n;
        let dc_q = samples.iter().map(|s| s.1).sum::<f64>() / n;

        // IQ imbalance
        let i_power = samples.iter().map(|s| s.0 * s.0).sum::<f64>() / n;
        let q_power = samples.iter().map(|s| s.1 * s.1).sum::<f64>() / n;
        let iq_imbalance_db = if q_power < 1e-30 {
            0.0
        } else {
            10.0 * (i_power / q_power).log10()
        };

        // Phase discontinuities
        let mut disc_count = 0usize;
        let mut prev_phase = samples[0].1.atan2(samples[0].0);
        for s in samples.iter().skip(1) {
            let ph = s.1.atan2(s.0);
            let mut delta = ph - prev_phase;
            // Wrap to [-pi, pi]
            while delta > PI {
                delta -= 2.0 * PI;
            }
            while delta < -PI {
                delta += 2.0 * PI;
            }
            if delta.abs() > self.phase_threshold {
                disc_count += 1;
            }
            prev_phase = ph;
        }

        QualityReport {
            papr_db,
            rms_power_dbm: power_to_dbm(avg_power),
            peak_power_dbm: power_to_dbm(peak_power),
            crest_factor_db,
            dc_offset: (dc_i, dc_q),
            iq_imbalance_db,
            phase_discontinuities: disc_count,
        }
    }
}

impl Default for SignalQuality {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// papr
// ---------------------------------------------------------------------------

/// Peak-to-average power ratio in dB for complex I/Q samples.
///
/// PAPR = 10 * log10(max(|x|^2) / mean(|x|^2))
///
/// A constant-envelope signal (e.g. CW tone) has PAPR near 0 dB.
pub fn papr(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let n = samples.len() as f64;
    let avg = samples.iter().map(power).sum::<f64>() / n;
    let peak = samples.iter().map(power).fold(0.0f64, f64::max);
    if avg < 1e-30 {
        return 0.0;
    }
    10.0 * (peak / avg).log10()
}

// ---------------------------------------------------------------------------
// ccdf
// ---------------------------------------------------------------------------

/// Complementary Cumulative Distribution Function of instantaneous power.
///
/// Returns `num_points` pairs of `(dB_above_average, probability)`.
/// The x-axis ranges from 0 dB to the signal's PAPR.
pub fn ccdf(samples: &[(f64, f64)], num_points: usize) -> Vec<(f64, f64)> {
    if samples.is_empty() || num_points == 0 {
        return Vec::new();
    }

    let n = samples.len() as f64;
    let avg_power = samples.iter().map(power).sum::<f64>() / n;
    if avg_power < 1e-30 {
        return vec![(0.0, 1.0); num_points];
    }

    let powers_db: Vec<f64> = samples
        .iter()
        .map(|s| 10.0 * (power(s) / avg_power).log10())
        .filter(|v| v.is_finite())
        .collect();

    let max_db = powers_db.iter().cloned().fold(0.0f64, f64::max);
    let max_db = if max_db < 1e-6 { 1.0 } else { max_db };

    let mut result = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let threshold = max_db * i as f64 / (num_points - 1).max(1) as f64;
        let count = powers_db.iter().filter(|&&p| p > threshold).count();
        let prob = count as f64 / powers_db.len() as f64;
        result.push((threshold, prob));
    }
    result
}

// ---------------------------------------------------------------------------
// occupied_bandwidth
// ---------------------------------------------------------------------------

/// Occupied bandwidth: the bandwidth containing the specified `fraction` of
/// total power in a power spectral density array.
///
/// `psd` is assumed to be in linear power units, with bins evenly spaced
/// over `sample_rate` Hz. `fraction` is typically 0.99 (99% of power).
///
/// Returns bandwidth in Hz.
pub fn occupied_bandwidth(psd: &[f64], sample_rate: f64, fraction: f64) -> f64 {
    if psd.is_empty() || fraction <= 0.0 {
        return 0.0;
    }
    let fraction = fraction.min(1.0);

    let total_power: f64 = psd.iter().sum();
    if total_power < 1e-30 {
        return 0.0;
    }
    let target = total_power * fraction;

    let n = psd.len();
    let bin_hz = sample_rate / n as f64;

    // Expand outward from the bin with the most power
    let center = psd
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    let mut lo = center;
    let mut hi = center;
    let mut accum = psd[center];

    while accum < target {
        let can_lo = lo > 0;
        let can_hi = hi < n - 1;
        if !can_lo && !can_hi {
            break;
        }
        // Expand toward whichever neighbor has more power
        let lo_val = if can_lo { psd[lo - 1] } else { -1.0 };
        let hi_val = if can_hi { psd[hi + 1] } else { -1.0 };
        if lo_val >= hi_val {
            lo -= 1;
            accum += psd[lo];
        } else {
            hi += 1;
            accum += psd[hi];
        }
    }

    ((hi - lo + 1) as f64) * bin_hz
}

// ---------------------------------------------------------------------------
// adjacent_channel_power
// ---------------------------------------------------------------------------

/// Adjacent Channel Power Ratio (ACPR) in dB.
///
/// Computes the ratio of power in the main channel (centered, width
/// `channel_bw`) to power in an adjacent channel centered at `offset` Hz
/// from the main channel center.
///
/// `psd`: linear-power PSD bins spanning `sample_rate` Hz.
pub fn adjacent_channel_power(psd: &[f64], sample_rate: f64, channel_bw: f64, offset: f64) -> f64 {
    if psd.is_empty() || sample_rate <= 0.0 || channel_bw <= 0.0 {
        return 0.0;
    }

    let n = psd.len();
    let bin_hz = sample_rate / n as f64;
    let center_bin = n / 2; // DC is at center (FFT-shifted)

    let half_main_bins = ((channel_bw / 2.0) / bin_hz).round() as usize;
    let main_lo = center_bin.saturating_sub(half_main_bins);
    let main_hi = (center_bin + half_main_bins).min(n - 1);

    let main_power: f64 = psd[main_lo..=main_hi].iter().sum();

    // Adjacent channel centered at offset from center
    let adj_center = ((center_bin as f64) + offset / bin_hz).round() as isize;
    let adj_lo = (adj_center - half_main_bins as isize).max(0) as usize;
    let adj_hi = ((adj_center + half_main_bins as isize) as usize).min(n - 1);

    let adj_power: f64 = if adj_lo <= adj_hi && adj_hi < n {
        psd[adj_lo..=adj_hi].iter().sum()
    } else {
        1e-30
    };

    if adj_power < 1e-30 {
        return 100.0; // Very high ACPR means very clean
    }
    if main_power < 1e-30 {
        return 0.0;
    }

    10.0 * (main_power / adj_power).log10()
}

// ---------------------------------------------------------------------------
// snr_estimate
// ---------------------------------------------------------------------------

/// Estimate SNR (in dB) given received samples and a known clean reference.
///
/// SNR = 10 * log10( sum(|ref|^2) / sum(|ref - rx|^2) )
///
/// The two slices are compared element-by-element up to the shorter length.
pub fn snr_estimate(samples: &[(f64, f64)], reference: &[(f64, f64)]) -> f64 {
    let n = samples.len().min(reference.len());
    if n == 0 {
        return 0.0;
    }

    let signal_power: f64 = reference[..n].iter().map(power).sum::<f64>();
    let noise_power: f64 = reference[..n]
        .iter()
        .zip(samples[..n].iter())
        .map(|(r, s)| {
            let di = s.0 - r.0;
            let dq = s.1 - r.1;
            di * di + dq * dq
        })
        .sum::<f64>();

    if noise_power < 1e-30 {
        return 100.0; // Essentially noiseless
    }
    if signal_power < 1e-30 {
        return -100.0;
    }

    10.0 * (signal_power / noise_power).log10()
}

// ---------------------------------------------------------------------------
// PhaseNoiseResult / PhaseNoiseAnalyzer
// ---------------------------------------------------------------------------

/// Result of phase noise analysis.
#[derive(Debug, Clone)]
pub struct PhaseNoiseResult {
    /// Offset frequencies from the carrier (Hz).
    pub offset_freqs: Vec<f64>,
    /// Phase noise power spectral density at each offset (dBc/Hz).
    pub pn_dbc_hz: Vec<f64>,
    /// Integrated phase jitter over the measured offsets (degrees).
    pub integrated_jitter_deg: f64,
}

/// Phase noise analyzer using FFT-based spectral estimation.
#[derive(Debug, Clone)]
pub struct PhaseNoiseAnalyzer {
    fft_size: usize,
}

impl PhaseNoiseAnalyzer {
    /// Create a new analyzer with the given FFT size.
    ///
    /// `fft_size` must be a power of two and at least 4.
    pub fn new(fft_size: usize) -> Self {
        let fft_size = fft_size.max(4).next_power_of_two();
        Self { fft_size }
    }

    /// Analyze phase noise of a carrier in the supplied I/Q samples.
    ///
    /// `carrier_freq`: expected carrier frequency in Hz.
    /// `sample_rate`: sample rate in Hz.
    pub fn analyze(
        &self,
        samples: &[(f64, f64)],
        carrier_freq: f64,
        sample_rate: f64,
    ) -> PhaseNoiseResult {
        let n = self.fft_size.min(samples.len());
        if n < 4 || sample_rate <= 0.0 {
            return PhaseNoiseResult {
                offset_freqs: Vec::new(),
                pn_dbc_hz: Vec::new(),
                integrated_jitter_deg: 0.0,
            };
        }

        // Extract instantaneous phase
        let phases: Vec<f64> = samples[..n]
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let expected = 2.0 * PI * carrier_freq * i as f64 / sample_rate;
                let measured = s.1.atan2(s.0);
                let mut residual = measured - expected;
                // Wrap
                while residual > PI {
                    residual -= 2.0 * PI;
                }
                while residual < -PI {
                    residual += 2.0 * PI;
                }
                residual
            })
            .collect();

        // Simple DFT-based PSD of phase residuals (Bartlett window, single segment)
        let bin_hz = sample_rate / n as f64;
        let half = n / 2;

        // DFT of phase residuals
        let mut psd = Vec::with_capacity(half);
        let mut offset_freqs = Vec::with_capacity(half);
        let mut pn_dbc_hz = Vec::with_capacity(half);

        // Carrier power estimate
        let carrier_power: f64 = samples[..n].iter().map(power).sum::<f64>() / n as f64;
        let carrier_power = carrier_power.max(1e-30);

        for k in 1..=half {
            let freq = k as f64 * bin_hz;
            // DFT bin k
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &ph) in phases.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                re += ph * angle.cos();
                im += ph * angle.sin();
            }
            let mag_sq = (re * re + im * im) / (n as f64 * n as f64);
            // Phase noise PSD in dBc/Hz
            let pn = if mag_sq < 1e-30 {
                -200.0
            } else {
                10.0 * (mag_sq / bin_hz).log10() - 10.0 * carrier_power.log10()
            };
            offset_freqs.push(freq);
            pn_dbc_hz.push(pn);
            psd.push(mag_sq);
        }

        // Integrated jitter: sqrt(2 * integral of phase PSD)
        // Integral approximated as sum of PSD * bin_hz, result in radians^2
        let integrated_radsq: f64 = psd.iter().sum::<f64>();
        // Total phase variance = integrated_radsq (already divided by n^2, multiply by n to get variance)
        let phase_var = integrated_radsq * n as f64;
        let jitter_rad = (2.0 * phase_var).sqrt();
        let integrated_jitter_deg = jitter_rad * 180.0 / PI;

        PhaseNoiseResult {
            offset_freqs,
            pn_dbc_hz,
            integrated_jitter_deg,
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Generate a constant-envelope complex tone.
    fn cw_tone(n: usize, cycles: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * cycles * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 1. QualityReport from known signal
    // -----------------------------------------------------------------------
    #[test]
    fn test_quality_report_known_signal() {
        let samples = cw_tone(1024, 4.0);
        let sq = SignalQuality::new();
        let report = sq.analyze(&samples);

        // CW tone: PAPR should be near 0 dB
        assert!(
            report.papr_db.abs() < 0.5,
            "CW PAPR should be ~0 dB, got {}",
            report.papr_db
        );
        // Power should be reasonable
        assert!(report.rms_power_dbm.is_finite());
        assert!(report.peak_power_dbm >= report.rms_power_dbm);
        // Crest factor should be near 0 dB for constant envelope
        assert!(
            report.crest_factor_db.abs() < 0.5,
            "CW crest factor should be ~0 dB, got {}",
            report.crest_factor_db
        );
        // DC offset should be near zero for a full-cycle tone
        assert!(
            report.dc_offset.0.abs() < 0.01,
            "DC-I should be ~0, got {}",
            report.dc_offset.0
        );
        assert!(
            report.dc_offset.1.abs() < 0.01,
            "DC-Q should be ~0, got {}",
            report.dc_offset.1
        );
    }

    // -----------------------------------------------------------------------
    // 2. PAPR of CW tone (~0 dB)
    // -----------------------------------------------------------------------
    #[test]
    fn test_papr_cw_tone() {
        let samples = cw_tone(2048, 10.0);
        let p = papr(&samples);
        assert!(
            p.abs() < 0.5,
            "CW tone PAPR should be ~0 dB, got {}",
            p
        );
    }

    // -----------------------------------------------------------------------
    // 3. PAPR of OFDM-like signal (high PAPR)
    // -----------------------------------------------------------------------
    #[test]
    fn test_papr_ofdm_like() {
        // Sum of many tones at different frequencies -> high PAPR
        let n = 2048;
        let num_carriers = 64;
        let mut samples = vec![(0.0, 0.0); n];
        for k in 0..num_carriers {
            let freq = k as f64;
            for i in 0..n {
                let phase = 2.0 * PI * freq * i as f64 / n as f64;
                samples[i].0 += phase.cos();
                samples[i].1 += phase.sin();
            }
        }
        let p = papr(&samples);
        // Multi-tone OFDM-like signal should have PAPR well above 3 dB
        assert!(
            p > 5.0,
            "OFDM-like PAPR should be high (>5 dB), got {}",
            p
        );
    }

    // -----------------------------------------------------------------------
    // 4. CCDF output format
    // -----------------------------------------------------------------------
    #[test]
    fn test_ccdf_output_format() {
        let samples = cw_tone(1024, 8.0);
        let dist = ccdf(&samples, 50);
        assert_eq!(dist.len(), 50, "Should return requested number of points");

        // First point: threshold = 0 dB above average
        assert!(
            dist[0].0.abs() < 0.01,
            "First threshold should be ~0 dB"
        );
        // Probabilities should be in [0, 1]
        for &(db, prob) in &dist {
            assert!(db >= -0.01, "Threshold should be non-negative, got {}", db);
            assert!(
                (0.0..=1.0).contains(&prob),
                "Probability out of range: {}",
                prob
            );
        }
        // Probability should be non-increasing
        for i in 1..dist.len() {
            assert!(
                dist[i].1 <= dist[i - 1].1 + 1e-9,
                "CCDF should be non-increasing at index {}",
                i
            );
        }
    }

    // -----------------------------------------------------------------------
    // 5. Occupied bandwidth
    // -----------------------------------------------------------------------
    #[test]
    fn test_occupied_bandwidth() {
        // Create a PSD with power concentrated in the center bins
        let n = 256;
        let sample_rate = 1000.0; // 1 kHz
        let mut psd = vec![1e-6; n]; // very low noise floor
        // Put strong signal in center 10 bins
        let center = n / 2;
        for i in (center - 5)..=(center + 5) {
            psd[i] = 1.0;
        }
        let bw = occupied_bandwidth(&psd, sample_rate, 0.99);
        let bin_hz = sample_rate / n as f64;
        // Expect roughly 11 bins * bin_hz (noise floor is negligible)
        let expected_bw = 11.0 * bin_hz;
        assert!(
            (bw - expected_bw).abs() < 5.0 * bin_hz,
            "Occupied BW should be ~{:.1} Hz, got {:.1} Hz",
            expected_bw,
            bw
        );
    }

    // -----------------------------------------------------------------------
    // 6. Adjacent channel power
    // -----------------------------------------------------------------------
    #[test]
    fn test_adjacent_channel_power() {
        let n = 512;
        let sample_rate = 10000.0;
        let bin_hz = sample_rate / n as f64;
        let center = n / 2;

        let mut psd = vec![0.0001; n]; // very low noise floor
        // Main channel: +-20 bins around center
        for i in (center - 20)..=(center + 20) {
            psd[i] = 1.0;
        }
        // Adjacent channel: some leakage 50 bins offset
        let adj_center = center + 50;
        for i in (adj_center.saturating_sub(20))..=(adj_center + 20).min(n - 1) {
            psd[i] = 0.01; // 20 dB below main
        }

        let channel_bw = 41.0 * bin_hz;
        let offset = 50.0 * bin_hz;
        let acpr = adjacent_channel_power(&psd, sample_rate, channel_bw, offset);
        // Main has ~41 bins * 1.0 = 41.0 power
        // Adj has ~41 bins * 0.01 = 0.41 power
        // ACPR ~ 10*log10(41/0.41) ~ 20 dB
        assert!(
            acpr > 15.0 && acpr < 25.0,
            "ACPR should be ~20 dB, got {:.1} dB",
            acpr
        );
    }

    // -----------------------------------------------------------------------
    // 7. SNR estimate accuracy
    // -----------------------------------------------------------------------
    #[test]
    fn test_snr_estimate_accuracy() {
        let n = 4096;
        let reference = cw_tone(n, 16.0);

        // Add noise at known level
        // Use a simple deterministic pseudo-noise for reproducibility
        let noise_amplitude = 0.1; // ~20 dB SNR
        let noisy: Vec<(f64, f64)> = reference
            .iter()
            .enumerate()
            .map(|(i, &(ri, rq))| {
                // Simple hash-based pseudo-random
                let seed = ((i as u64).wrapping_mul(6364136223846793005).wrapping_add(1)) as f64;
                let ni = (seed / u64::MAX as f64 - 0.5) * 2.0 * noise_amplitude;
                let seed2 = ((i as u64 + 9999).wrapping_mul(6364136223846793005).wrapping_add(1)) as f64;
                let nq = (seed2 / u64::MAX as f64 - 0.5) * 2.0 * noise_amplitude;
                (ri + ni, rq + nq)
            })
            .collect();

        let snr = snr_estimate(&noisy, &reference);
        // Should be roughly in the right ballpark (> 10 dB given small noise)
        assert!(
            snr > 10.0,
            "SNR should be > 10 dB for small noise, got {:.1} dB",
            snr
        );
        assert!(
            snr < 60.0,
            "SNR should be bounded, got {:.1} dB",
            snr
        );
    }

    // -----------------------------------------------------------------------
    // 8. DC offset detection
    // -----------------------------------------------------------------------
    #[test]
    fn test_dc_offset_detection() {
        let dc_i = 0.25;
        let dc_q = -0.15;
        // CW tone + DC offset
        let samples: Vec<(f64, f64)> = (0..2048)
            .map(|i| {
                let phase = 2.0 * PI * 8.0 * i as f64 / 2048.0;
                (phase.cos() + dc_i, phase.sin() + dc_q)
            })
            .collect();

        let sq = SignalQuality::new();
        let report = sq.analyze(&samples);

        assert!(
            (report.dc_offset.0 - dc_i).abs() < 0.01,
            "DC-I should be ~{}, got {}",
            dc_i,
            report.dc_offset.0
        );
        assert!(
            (report.dc_offset.1 - dc_q).abs() < 0.01,
            "DC-Q should be ~{}, got {}",
            dc_q,
            report.dc_offset.1
        );
    }

    // -----------------------------------------------------------------------
    // 9. IQ imbalance detection
    // -----------------------------------------------------------------------
    #[test]
    fn test_iq_imbalance_detection() {
        // Create signal with 2x amplitude on I vs Q -> 6 dB imbalance
        let samples: Vec<(f64, f64)> = (0..2048)
            .map(|i| {
                let phase = 2.0 * PI * 8.0 * i as f64 / 2048.0;
                (2.0 * phase.cos(), 1.0 * phase.sin())
            })
            .collect();

        let sq = SignalQuality::new();
        let report = sq.analyze(&samples);

        // I power is 4x Q power -> 10*log10(4) ~ 6.02 dB
        assert!(
            (report.iq_imbalance_db - 6.02).abs() < 0.5,
            "IQ imbalance should be ~6 dB, got {:.2} dB",
            report.iq_imbalance_db
        );
    }

    // -----------------------------------------------------------------------
    // 10. Phase noise analyzer construction
    // -----------------------------------------------------------------------
    #[test]
    fn test_phase_noise_analyzer_construction() {
        let pna = PhaseNoiseAnalyzer::new(1024);
        assert_eq!(pna.fft_size, 1024);

        // Non-power-of-two should be rounded up
        let pna2 = PhaseNoiseAnalyzer::new(1000);
        assert_eq!(pna2.fft_size, 1024);

        // Very small should be clamped to at least 4
        let pna3 = PhaseNoiseAnalyzer::new(1);
        assert_eq!(pna3.fft_size, 4);

        // Run analysis on a clean tone
        let samples = cw_tone(2048, 10.0);
        let carrier_freq = 10.0 * 1000.0 / 2048.0; // match the tone freq at 1000 Hz sample rate
        let result = pna.analyze(&samples, carrier_freq, 1000.0);

        assert!(!result.offset_freqs.is_empty());
        assert_eq!(result.offset_freqs.len(), result.pn_dbc_hz.len());
        assert!(result.integrated_jitter_deg >= 0.0);
        // All offset frequencies should be positive
        for &f in &result.offset_freqs {
            assert!(f > 0.0, "Offset freq should be positive, got {}", f);
        }
    }
}
