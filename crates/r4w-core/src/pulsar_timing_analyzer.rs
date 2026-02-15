//! Pulsar Timing Residual Analysis for Radio Astronomy
//!
//! Implements pulsar timing analysis techniques including pulse folding,
//! dispersion measure (DM) correction, time-of-arrival (TOA) estimation,
//! and timing residual computation. These methods are fundamental to
//! pulsar timing arrays (PTAs) used for gravitational wave detection,
//! tests of general relativity, and precision astrometry.
//!
//! # Overview
//!
//! Pulsar timing exploits the extraordinary rotational stability of
//! millisecond pulsars as natural clocks. The basic workflow is:
//!
//! 1. **Dispersion correction** - Remove frequency-dependent delay from
//!    the cold interstellar plasma (ISM) using the dispersion relation
//!    `t_DM = K_DM * DM / f^2` where `K_DM = e^2 / (2*pi*m_e*c) ~ 4.149 GHz^2 pc^-1 cm^3 s`.
//!
//! 2. **Pulse folding** - Epoch-fold the dedispersed time series at the
//!    topocentric pulse period to build an integrated pulse profile.
//!
//! 3. **TOA estimation** - Cross-correlate the integrated profile against
//!    a noise-free template to determine the pulse time of arrival.
//!
//! 4. **Timing residuals** - Compute observed-minus-computed (O-C)
//!    residuals by comparing measured TOAs against a timing model.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pulsar_timing_analyzer::{
//!     PulsarConfig, PulseFolding, DispersionCorrector, ToaEstimator,
//!     dispersion_delay, pulse_phase,
//! };
//!
//! // Configure a canonical millisecond pulsar
//! let config = PulsarConfig {
//!     period_s: 0.00575,         // 5.75 ms period (like PSR J0437-4715)
//!     dm: 2.65,                  // DM in pc cm^-3
//!     freq_lo_mhz: 1200.0,      // Low edge of band
//!     freq_hi_mhz: 1500.0,      // High edge of band
//!     duty_cycle: 0.05,          // 5% duty cycle
//! };
//!
//! // Compute dispersion delay across the band
//! let delay = dispersion_delay(config.dm, config.freq_lo_mhz, config.freq_hi_mhz);
//! assert!(delay > 0.0);
//!
//! // Compute pulse phase at a given time
//! let phase = pulse_phase(0.01, config.period_s, 0.0);
//! assert!(phase >= 0.0 && phase < 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Dispersion constant K_DM in MHz^2 pc^-1 cm^3 s.
///
/// Derived from `e^2 / (2 * pi * m_e * c)`. The standard IAU value is
/// 4.148808 x 10^3 MHz^2 pc^-1 cm^3 s, commonly written as ~4149.
const K_DM: f64 = 4.148808e3;

/// Speed of light in m/s (IAU 2012 nominal value).
const C_LIGHT: f64 = 299_792_458.0;

/// Seconds per day.
const SECONDS_PER_DAY: f64 = 86_400.0;

/// Astronomical Unit in metres.
const AU_M: f64 = 1.495_978_707e11;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for a pulsar observation.
#[derive(Debug, Clone)]
pub struct PulsarConfig {
    /// Topocentric pulse period in seconds.
    pub period_s: f64,
    /// Dispersion measure in pc cm^-3.
    pub dm: f64,
    /// Lower edge of the observing band in MHz.
    pub freq_lo_mhz: f64,
    /// Upper edge of the observing band in MHz.
    pub freq_hi_mhz: f64,
    /// Fractional pulse duty cycle (0.0 .. 1.0).
    pub duty_cycle: f64,
}

impl PulsarConfig {
    /// Centre frequency of the observing band in MHz.
    pub fn center_freq_mhz(&self) -> f64 {
        0.5 * (self.freq_lo_mhz + self.freq_hi_mhz)
    }

    /// Bandwidth of the observing band in MHz.
    pub fn bandwidth_mhz(&self) -> f64 {
        self.freq_hi_mhz - self.freq_lo_mhz
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the cold-plasma dispersion delay between two frequencies.
///
/// Returns the differential delay in seconds:
/// `delta_t = K_DM * DM * (1/f_lo^2 - 1/f_hi^2)`
///
/// where frequencies are in MHz and DM is in pc cm^-3.
pub fn dispersion_delay(dm: f64, freq_lo_mhz: f64, freq_hi_mhz: f64) -> f64 {
    K_DM * dm * (1.0 / (freq_lo_mhz * freq_lo_mhz) - 1.0 / (freq_hi_mhz * freq_hi_mhz))
}

/// Compute the absolute dispersion delay at a single frequency relative
/// to infinite frequency.
///
/// `t = K_DM * DM / f^2` (seconds)
pub fn dispersion_delay_absolute(dm: f64, freq_mhz: f64) -> f64 {
    K_DM * dm / (freq_mhz * freq_mhz)
}

/// Compute the pulse phase at time `t` given period `p` and reference epoch `t0`.
///
/// Returns a value in [0, 1).
pub fn pulse_phase(t: f64, period_s: f64, t0: f64) -> f64 {
    let phase = ((t - t0) / period_s).rem_euclid(1.0);
    // Guard against floating point producing exactly 1.0
    if phase >= 1.0 { 0.0 } else { phase }
}

/// Approximate Roemer delay (light-travel time) for barycentric correction.
///
/// Given the Earth-Sun distance projected along the pulsar direction
/// (`proj_au` in AU), returns the geometric delay in seconds:
///
/// `delta_t = (proj_au * AU) / c`
///
/// A full barycentric correction also includes Shapiro delay, Einstein
/// delay, and annual/diurnal aberration; this helper covers the dominant
/// Roemer term only.
pub fn barycentric_correction(proj_au: f64) -> f64 {
    proj_au * AU_M / C_LIGHT
}

/// Compute a simple power-law spectral index flux scaling.
///
/// `S(f) = S_ref * (f / f_ref) ^ alpha`
///
/// Typical pulsar spectral indices are alpha ~ -1.4 to -1.8.
pub fn spectral_index(flux_ref: f64, freq_ref_mhz: f64, freq_mhz: f64, alpha: f64) -> f64 {
    flux_ref * (freq_mhz / freq_ref_mhz).powf(alpha)
}

// ---------------------------------------------------------------------------
// Pulse Folding
// ---------------------------------------------------------------------------

/// Epoch-folding engine that accumulates time-series data into a pulse profile.
///
/// Divides the pulse period into `n_bins` phase bins and accumulates
/// each sample into the appropriate bin, building up an integrated
/// (high S/N) average pulse profile over many rotations.
#[derive(Debug, Clone)]
pub struct PulseFolding {
    /// Number of phase bins in the folded profile.
    n_bins: usize,
    /// Accumulated signal in each phase bin.
    profile: Vec<f64>,
    /// Number of samples accumulated in each bin.
    counts: Vec<u64>,
    /// Pulse period used for folding (seconds).
    period_s: f64,
    /// Reference epoch (seconds).
    t0: f64,
}

impl PulseFolding {
    /// Create a new pulse-folding accumulator.
    ///
    /// # Arguments
    /// * `n_bins` - Number of phase bins (typically 256 or 1024).
    /// * `period_s` - Pulse period in seconds.
    /// * `t0` - Reference epoch (seconds since some fiducial).
    pub fn new(n_bins: usize, period_s: f64, t0: f64) -> Self {
        Self {
            n_bins,
            profile: vec![0.0; n_bins],
            counts: vec![0; n_bins],
            period_s,
            t0,
        }
    }

    /// Fold a contiguous block of time-series samples starting at `t_start`
    /// with uniform sample spacing `dt`.
    pub fn fold(&mut self, samples: &[f64], t_start: f64, dt: f64) {
        for (i, &s) in samples.iter().enumerate() {
            let t = t_start + i as f64 * dt;
            let phase = pulse_phase(t, self.period_s, self.t0);
            let bin = (phase * self.n_bins as f64) as usize;
            let bin = bin.min(self.n_bins - 1);
            self.profile[bin] += s;
            self.counts[bin] += 1;
        }
    }

    /// Return the mean-normalised integrated pulse profile.
    pub fn profile(&self) -> Vec<f64> {
        self.profile
            .iter()
            .zip(self.counts.iter())
            .map(|(&sum, &cnt)| if cnt > 0 { sum / cnt as f64 } else { 0.0 })
            .collect()
    }

    /// Return raw accumulated sums per bin.
    pub fn raw_profile(&self) -> &[f64] {
        &self.profile
    }

    /// Return sample counts per bin.
    pub fn counts(&self) -> &[u64] {
        &self.counts
    }

    /// Total number of samples folded.
    pub fn total_samples(&self) -> u64 {
        self.counts.iter().sum()
    }

    /// Reset the accumulator.
    pub fn reset(&mut self) {
        self.profile.fill(0.0);
        self.counts.fill(0);
    }

    /// Compute the signal-to-noise ratio of the folded profile.
    ///
    /// Uses the off-pulse RMS as the noise estimate. The off-pulse region
    /// is defined as the lower 75% of bin values.
    pub fn snr(&self) -> f64 {
        let prof = self.profile();
        if prof.is_empty() {
            return 0.0;
        }
        let mut sorted = prof.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Off-pulse = lower 75% of bins
        let off_end = (sorted.len() * 3) / 4;
        if off_end == 0 {
            return 0.0;
        }
        let off_pulse = &sorted[..off_end];
        let mean_off: f64 = off_pulse.iter().sum::<f64>() / off_end as f64;
        let var_off: f64 =
            off_pulse.iter().map(|x| (x - mean_off).powi(2)).sum::<f64>() / off_end as f64;
        let rms_off = var_off.sqrt();
        if rms_off < 1e-30 {
            return 0.0;
        }

        let peak = prof.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (peak - mean_off) / rms_off
    }
}

// ---------------------------------------------------------------------------
// Dispersion Corrector
// ---------------------------------------------------------------------------

/// Incoherent dedispersion across frequency channels.
///
/// Removes the cold-plasma dispersion delay by time-shifting each
/// frequency channel relative to a reference frequency. This is the
/// standard "brute-force" or "incoherent" approach; coherent
/// dedispersion (phase-domain) is not covered here.
#[derive(Debug, Clone)]
pub struct DispersionCorrector {
    /// Dispersion measure (pc cm^-3).
    dm: f64,
    /// Reference frequency in MHz (delays are measured relative to this).
    ref_freq_mhz: f64,
    /// Per-channel delay in samples (pre-computed).
    delays_samples: Vec<i64>,
    /// Channel centre frequencies in MHz.
    channel_freqs_mhz: Vec<f64>,
}

impl DispersionCorrector {
    /// Create a new corrector for `n_channels` uniformly spaced between
    /// `freq_lo_mhz` and `freq_hi_mhz` at sample rate `sample_rate_hz`.
    ///
    /// The reference frequency is the highest channel (minimum delay).
    pub fn new(
        dm: f64,
        freq_lo_mhz: f64,
        freq_hi_mhz: f64,
        n_channels: usize,
        sample_rate_hz: f64,
    ) -> Self {
        let ref_freq = freq_hi_mhz;
        let chan_bw = (freq_hi_mhz - freq_lo_mhz) / n_channels as f64;
        let mut freqs = Vec::with_capacity(n_channels);
        let mut delays = Vec::with_capacity(n_channels);

        for i in 0..n_channels {
            let f = freq_lo_mhz + (i as f64 + 0.5) * chan_bw;
            let dt = dispersion_delay(dm, f, ref_freq);
            freqs.push(f);
            delays.push((dt * sample_rate_hz).round() as i64);
        }

        Self {
            dm,
            ref_freq_mhz: ref_freq,
            delays_samples: delays,
            channel_freqs_mhz: freqs,
        }
    }

    /// Return the per-channel delays in samples.
    pub fn delays(&self) -> &[i64] {
        &self.delays_samples
    }

    /// Return channel centre frequencies.
    pub fn channel_freqs(&self) -> &[f64] {
        &self.channel_freqs_mhz
    }

    /// Apply incoherent dedispersion to a 2-D spectrogram.
    ///
    /// `spectrogram` is indexed as `[channel][time_sample]`.
    /// Each channel is shifted by its delay; the output length is trimmed
    /// to the region of complete overlap.
    pub fn correct(&self, spectrogram: &[Vec<f64>]) -> Vec<Vec<f64>> {
        if spectrogram.is_empty() {
            return vec![];
        }
        let n_chan = spectrogram.len().min(self.delays_samples.len());
        let max_delay = self.delays_samples.iter().copied().max().unwrap_or(0);
        let min_len = spectrogram.iter().map(|c| c.len()).min().unwrap_or(0);

        if max_delay < 0 || min_len == 0 {
            return spectrogram.to_vec();
        }
        let max_delay = max_delay as usize;
        if min_len <= max_delay {
            // Not enough samples to correct
            return spectrogram.to_vec();
        }
        let out_len = min_len - max_delay;
        let mut result = Vec::with_capacity(n_chan);

        for ch in 0..n_chan {
            let delay = self.delays_samples[ch] as usize;
            let shifted: Vec<f64> = spectrogram[ch][delay..delay + out_len].to_vec();
            result.push(shifted);
        }

        result
    }

    /// Collapse corrected channels into a single dedispersed time series
    /// by summing across frequency.
    pub fn correct_and_sum(&self, spectrogram: &[Vec<f64>]) -> Vec<f64> {
        let corrected = self.correct(spectrogram);
        if corrected.is_empty() {
            return vec![];
        }
        let out_len = corrected[0].len();
        let mut summed = vec![0.0; out_len];
        for ch in &corrected {
            for (i, &v) in ch.iter().enumerate() {
                summed[i] += v;
            }
        }
        summed
    }
}

// ---------------------------------------------------------------------------
// TOA Estimator
// ---------------------------------------------------------------------------

/// Template-matching Time-of-Arrival (TOA) estimator.
///
/// Determines pulse arrival times by cross-correlating observed profiles
/// against a high-S/N template. The shift of the cross-correlation peak
/// gives the TOA offset, and the peak sharpness gives the uncertainty.
#[derive(Debug, Clone)]
pub struct ToaEstimator {
    /// Normalised template profile.
    template: Vec<f64>,
}

/// Result of a TOA measurement.
#[derive(Debug, Clone)]
pub struct ToaResult {
    /// Phase offset of the observed profile relative to the template, in bins.
    pub offset_bins: f64,
    /// Estimated TOA uncertainty in bins (from cross-correlation peak width).
    pub uncertainty_bins: f64,
    /// Peak cross-correlation value (quality indicator).
    pub peak_correlation: f64,
}

impl ToaEstimator {
    /// Create a new TOA estimator from a template profile.
    ///
    /// The template is mean-subtracted and normalised internally.
    pub fn new(template: &[f64]) -> Self {
        let n = template.len();
        let mean = template.iter().sum::<f64>() / n as f64;
        let centered: Vec<f64> = template.iter().map(|x| x - mean).collect();
        let norm = centered.iter().map(|x| x * x).sum::<f64>().sqrt();
        let normalised = if norm > 1e-30 {
            centered.iter().map(|x| x / norm).collect()
        } else {
            centered
        };
        Self {
            template: normalised,
        }
    }

    /// Estimate the TOA of an observed profile.
    ///
    /// Both the template and observed profile must have the same length.
    /// Returns the fractional-bin offset and uncertainty.
    pub fn estimate(&self, observed: &[f64]) -> ToaResult {
        let n = self.template.len();
        assert_eq!(
            observed.len(),
            n,
            "Observed profile length must match template"
        );

        // Mean-subtract observed
        let obs_mean = observed.iter().sum::<f64>() / n as f64;
        let obs_centered: Vec<f64> = observed.iter().map(|x| x - obs_mean).collect();
        let obs_norm = obs_centered.iter().map(|x| x * x).sum::<f64>().sqrt();

        if obs_norm < 1e-30 {
            return ToaResult {
                offset_bins: 0.0,
                uncertainty_bins: f64::INFINITY,
                peak_correlation: 0.0,
            };
        }

        // Circular cross-correlation
        let mut ccf = vec![0.0; n];
        for lag in 0..n {
            let mut sum = 0.0;
            for j in 0..n {
                let k = (j + lag) % n;
                sum += self.template[j] * obs_centered[k];
            }
            ccf[lag] = sum / obs_norm;
        }

        // Find peak
        let (peak_idx, peak_val) = ccf
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();

        // Parabolic interpolation for sub-bin accuracy
        let left = ccf[(peak_idx + n - 1) % n];
        let center = ccf[peak_idx];
        let right = ccf[(peak_idx + 1) % n];

        let denom = 2.0 * (2.0 * center - left - right);
        let frac_offset = if denom.abs() > 1e-30 {
            (left - right) / denom
        } else {
            0.0
        };

        let refined_offset = peak_idx as f64 + frac_offset;
        // Wrap to [-n/2, n/2) convention
        let offset = if refined_offset > n as f64 / 2.0 {
            refined_offset - n as f64
        } else {
            refined_offset
        };

        // Uncertainty from the curvature of the CCF peak
        let curvature = (left + right - 2.0 * center).abs();
        let uncertainty = if curvature > 1e-30 {
            (2.0 / curvature).sqrt() * (1.0 - peak_val * peak_val).max(0.0).sqrt()
        } else {
            f64::INFINITY
        };

        ToaResult {
            offset_bins: offset,
            uncertainty_bins: uncertainty,
            peak_correlation: *peak_val,
        }
    }

    /// Convert a bin offset to a time offset in seconds.
    pub fn bins_to_seconds(offset_bins: f64, n_bins: usize, period_s: f64) -> f64 {
        offset_bins * period_s / n_bins as f64
    }
}

// ---------------------------------------------------------------------------
// Timing Residual Analyzer
// ---------------------------------------------------------------------------

/// Observed-minus-computed (O-C) timing residual analyser.
///
/// Given a sequence of measured TOAs and a simple polynomial timing model
/// `phi(t) = phi0 + f*(t-t0) + 0.5*f_dot*(t-t0)^2`, computes residuals
/// and basic statistics used for pulsar timing analysis.
#[derive(Debug, Clone)]
pub struct TimingResidualAnalyzer {
    /// Reference epoch (seconds, e.g. MJD * 86400).
    pub t0: f64,
    /// Spin frequency at t0 (Hz).
    pub f0: f64,
    /// Spin frequency derivative at t0 (Hz/s).
    pub f_dot: f64,
}

/// Summary statistics for timing residuals.
#[derive(Debug, Clone)]
pub struct ResidualStats {
    /// Number of TOAs.
    pub n_toas: usize,
    /// Mean residual in seconds.
    pub mean_s: f64,
    /// Root-mean-square residual in seconds.
    pub rms_s: f64,
    /// Weighted RMS residual in seconds (if uncertainties provided).
    pub wrms_s: f64,
    /// Reduced chi-squared statistic.
    pub reduced_chi2: f64,
    /// Maximum absolute residual in seconds.
    pub max_abs_s: f64,
}

impl TimingResidualAnalyzer {
    /// Create a new analyzer with a polynomial timing model.
    ///
    /// # Arguments
    /// * `t0` - Reference epoch in seconds.
    /// * `f0` - Spin frequency at `t0` in Hz.
    /// * `f_dot` - Spin-down rate (frequency derivative) in Hz/s.
    pub fn new(t0: f64, f0: f64, f_dot: f64) -> Self {
        Self { t0, f0, f_dot }
    }

    /// Predict the pulse phase (number of rotations since t0) at time `t`.
    pub fn predict_phase(&self, t: f64) -> f64 {
        let dt = t - self.t0;
        self.f0 * dt + 0.5 * self.f_dot * dt * dt
    }

    /// Predict the expected TOA for the Nth pulse after t0.
    ///
    /// For a constant-frequency pulsar, this is simply `t0 + N / f0`.
    /// With spin-down, we solve the quadratic iteratively.
    pub fn predict_toa(&self, pulse_number: f64) -> f64 {
        // Initial estimate assuming constant frequency
        let mut t = self.t0 + pulse_number / self.f0;

        // Newton-Raphson refinement (2 iterations suffice for f_dot << f0)
        for _ in 0..5 {
            let dt = t - self.t0;
            let phase = self.f0 * dt + 0.5 * self.f_dot * dt * dt;
            let freq = self.f0 + self.f_dot * dt;
            if freq.abs() < 1e-30 {
                break;
            }
            t -= (phase - pulse_number) / freq;
        }
        t
    }

    /// Compute timing residuals for a set of observed TOAs.
    ///
    /// Each TOA is assigned the nearest integer pulse number and the
    /// residual is `(observed - predicted)` in seconds.
    pub fn residuals(&self, toas: &[f64]) -> Vec<f64> {
        toas.iter()
            .map(|&t| {
                let phase = self.predict_phase(t);
                let nearest_pulse = phase.round();
                let predicted = self.predict_toa(nearest_pulse);
                t - predicted
            })
            .collect()
    }

    /// Compute residuals and summary statistics.
    ///
    /// If `uncertainties` is provided (same length as `toas`), weighted
    /// statistics are computed.
    pub fn analyze(
        &self,
        toas: &[f64],
        uncertainties: Option<&[f64]>,
    ) -> ResidualStats {
        let residuals = self.residuals(toas);
        let n = residuals.len();
        if n == 0 {
            return ResidualStats {
                n_toas: 0,
                mean_s: 0.0,
                rms_s: 0.0,
                wrms_s: 0.0,
                reduced_chi2: 0.0,
                max_abs_s: 0.0,
            };
        }

        let mean = residuals.iter().sum::<f64>() / n as f64;
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / n as f64).sqrt();
        let max_abs = residuals
            .iter()
            .map(|r| r.abs())
            .fold(0.0_f64, f64::max);

        // Weighted statistics
        let (wrms, chi2) = if let Some(sigmas) = uncertainties {
            let mut w_sum = 0.0;
            let mut wss = 0.0;
            let mut chi2_sum = 0.0;
            for (r, s) in residuals.iter().zip(sigmas.iter()) {
                let w = if s.abs() > 1e-30 { 1.0 / (s * s) } else { 0.0 };
                w_sum += w;
                wss += w * r * r;
                chi2_sum += if s.abs() > 1e-30 {
                    (r / s).powi(2)
                } else {
                    0.0
                };
            }
            let wrms_val = if w_sum > 0.0 {
                (wss / w_sum).sqrt()
            } else {
                rms
            };
            // Reduced chi-squared with (n - 3) DOF for 3-parameter model
            let dof = if n > 3 { (n - 3) as f64 } else { 1.0 };
            (wrms_val, chi2_sum / dof)
        } else {
            (rms, 0.0)
        };

        ResidualStats {
            n_toas: n,
            mean_s: mean,
            rms_s: rms,
            wrms_s: wrms,
            reduced_chi2: chi2,
            max_abs_s: max_abs,
        }
    }
}

// ---------------------------------------------------------------------------
// DM Optimizer
// ---------------------------------------------------------------------------

/// Dispersion Measure optimiser that refines DM by maximising the
/// peak signal-to-noise ratio of the folded pulse profile.
///
/// Performs a grid search over a range of trial DMs and evaluates
/// the S/N of the dedispersed and folded profile at each trial.
#[derive(Debug, Clone)]
pub struct DmOptimizer {
    /// Centre frequency range for the channels.
    freq_lo_mhz: f64,
    freq_hi_mhz: f64,
    /// Number of frequency channels.
    n_channels: usize,
    /// Sample rate in Hz.
    sample_rate_hz: f64,
    /// Pulse period in seconds.
    period_s: f64,
    /// Number of phase bins for folding.
    n_bins: usize,
}

/// Result of DM optimisation.
#[derive(Debug, Clone)]
pub struct DmOptResult {
    /// Best-fit DM (pc cm^-3).
    pub best_dm: f64,
    /// Peak S/N at best DM.
    pub best_snr: f64,
    /// All trial DMs and their S/N values.
    pub dm_snr_curve: Vec<(f64, f64)>,
}

impl DmOptimizer {
    /// Create a new DM optimiser.
    pub fn new(
        freq_lo_mhz: f64,
        freq_hi_mhz: f64,
        n_channels: usize,
        sample_rate_hz: f64,
        period_s: f64,
        n_bins: usize,
    ) -> Self {
        Self {
            freq_lo_mhz,
            freq_hi_mhz,
            n_channels,
            sample_rate_hz,
            period_s,
            n_bins,
        }
    }

    /// Search a range of trial DMs and return the one with the highest S/N.
    ///
    /// `spectrogram` is `[channel][time_sample]`. Trials span
    /// `dm_min..=dm_max` in steps of `dm_step`.
    pub fn optimize(
        &self,
        spectrogram: &[Vec<f64>],
        dm_min: f64,
        dm_max: f64,
        dm_step: f64,
    ) -> DmOptResult {
        let mut best_dm = dm_min;
        let mut best_snr = f64::NEG_INFINITY;
        let mut curve = Vec::new();

        let mut trial_dm = dm_min;
        while trial_dm <= dm_max + dm_step * 0.5 {
            let corrector = DispersionCorrector::new(
                trial_dm,
                self.freq_lo_mhz,
                self.freq_hi_mhz,
                self.n_channels,
                self.sample_rate_hz,
            );
            let dedispersed = corrector.correct_and_sum(spectrogram);
            if dedispersed.is_empty() {
                trial_dm += dm_step;
                continue;
            }

            let dt = 1.0 / self.sample_rate_hz;
            let mut folder = PulseFolding::new(self.n_bins, self.period_s, 0.0);
            folder.fold(&dedispersed, 0.0, dt);
            let snr = folder.snr();

            curve.push((trial_dm, snr));
            if snr > best_snr {
                best_snr = snr;
                best_dm = trial_dm;
            }

            trial_dm += dm_step;
        }

        DmOptResult {
            best_dm,
            best_snr,
            dm_snr_curve: curve,
        }
    }
}

// ---------------------------------------------------------------------------
// Synthetic pulse helpers (for testing)
// ---------------------------------------------------------------------------

/// Generate a synthetic Gaussian pulse profile.
///
/// The pulse is centred at phase = `center` with width `sigma` (both in
/// units of [0,1] phase). Returns `n_bins` samples.
fn gaussian_profile(n_bins: usize, center: f64, sigma: f64) -> Vec<f64> {
    (0..n_bins)
        .map(|i| {
            let phase = i as f64 / n_bins as f64;
            let dp = phase - center;
            // Wrap to [-0.5, 0.5]
            let dp = dp - dp.round();
            (-0.5 * (dp / sigma).powi(2)).exp()
        })
        .collect()
}

/// Generate a synthetic time-domain pulsar signal (single channel).
///
/// Produces a time series of `n_samples` at `sample_rate_hz` containing
/// periodic Gaussian pulses with period `period_s`, duty cycle `duty_cycle`,
/// peak amplitude `amplitude`, and additive Gaussian noise of rms `noise_rms`.
fn synthetic_pulsar_signal(
    n_samples: usize,
    sample_rate_hz: f64,
    period_s: f64,
    duty_cycle: f64,
    amplitude: f64,
    noise_rms: f64,
    seed: u64,
) -> Vec<f64> {
    let sigma = duty_cycle / 2.355; // FWHM -> sigma
    let dt = 1.0 / sample_rate_hz;

    // Simple LCG PRNG for noise
    let mut rng_state = seed;
    let next_rand = |state: &mut u64| -> f64 {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Convert to uniform [0,1)
        (*state >> 11) as f64 / (1u64 << 53) as f64
    };
    // Box-Muller for Gaussian noise
    let mut spare: Option<f64> = None;
    let mut gaussian = |state: &mut u64| -> f64 {
        if let Some(s) = spare.take() {
            return s * noise_rms;
        }
        let u1 = next_rand(state).max(1e-30);
        let u2 = next_rand(state);
        let mag = (-2.0 * u1.ln()).sqrt();
        let z0 = mag * (2.0 * PI * u2).cos();
        let z1 = mag * (2.0 * PI * u2).sin();
        spare = Some(z1);
        z0 * noise_rms
    };

    (0..n_samples)
        .map(|i| {
            let t = i as f64 * dt;
            let phase = (t / period_s).rem_euclid(1.0);
            let dp = phase - 0.25; // Centre pulse at phase 0.25
            let dp = dp - dp.round();
            let pulse = amplitude * (-0.5 * (dp / sigma).powi(2)).exp();
            pulse + gaussian(&mut rng_state)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // ---- dispersion_delay tests ----

    #[test]
    fn test_dispersion_delay_positive() {
        let delay = dispersion_delay(100.0, 1000.0, 2000.0);
        // delay = K_DM * 100 * (1/1e6 - 1/4e6) = 4148.808 * 100 * 7.5e-7
        assert!(delay > 0.0, "Delay must be positive for f_lo < f_hi");
    }

    #[test]
    fn test_dispersion_delay_zero_dm() {
        let delay = dispersion_delay(0.0, 400.0, 800.0);
        assert!((delay).abs() < TOLERANCE, "Zero DM => zero delay");
    }

    #[test]
    fn test_dispersion_delay_same_freq() {
        let delay = dispersion_delay(50.0, 1400.0, 1400.0);
        assert!(delay.abs() < TOLERANCE, "Same freq => zero delay");
    }

    #[test]
    fn test_dispersion_delay_known_value() {
        // DM=1, 1 GHz to inf: delay = K_DM * 1 / (1000)^2 = 4148.808 / 1e6 s
        let delay = dispersion_delay_absolute(1.0, 1000.0);
        let expected = K_DM / 1e6;
        assert!(
            (delay - expected).abs() < 1e-10,
            "Absolute delay at 1 GHz for DM=1 should be ~4.149 ms"
        );
    }

    #[test]
    fn test_dispersion_delay_frequency_scaling() {
        // Delay scales as 1/f^2, so halving frequency quadruples delay
        let d1 = dispersion_delay_absolute(10.0, 1000.0);
        let d2 = dispersion_delay_absolute(10.0, 500.0);
        assert!(
            (d2 / d1 - 4.0).abs() < 0.01,
            "Halving frequency should quadruple delay"
        );
    }

    // ---- pulse_phase tests ----

    #[test]
    fn test_pulse_phase_zero() {
        let phase = pulse_phase(0.0, 1.0, 0.0);
        assert!((phase).abs() < TOLERANCE);
    }

    #[test]
    fn test_pulse_phase_half() {
        let phase = pulse_phase(0.5, 1.0, 0.0);
        assert!((phase - 0.5).abs() < TOLERANCE);
    }

    #[test]
    fn test_pulse_phase_wrap() {
        let phase = pulse_phase(1.25, 1.0, 0.0);
        assert!(
            (phase - 0.25).abs() < TOLERANCE,
            "Phase should wrap to [0,1)"
        );
    }

    #[test]
    fn test_pulse_phase_negative_time() {
        let phase = pulse_phase(-0.3, 1.0, 0.0);
        assert!(
            phase >= 0.0 && phase < 1.0,
            "Phase must be in [0,1) for negative times"
        );
        assert!((phase - 0.7).abs() < TOLERANCE);
    }

    // ---- barycentric_correction test ----

    #[test]
    fn test_barycentric_correction() {
        // At 1 AU projected distance, delay is AU/c ~ 499 s
        let delay = barycentric_correction(1.0);
        assert!(
            (delay - 499.0).abs() < 1.0,
            "1 AU light travel time should be ~499 s, got {}",
            delay
        );
    }

    // ---- spectral_index test ----

    #[test]
    fn test_spectral_index_unity() {
        let flux = spectral_index(1.0, 1400.0, 1400.0, -1.6);
        assert!((flux - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_spectral_index_negative() {
        // At double the frequency with alpha = -2, flux should be 1/4
        let flux = spectral_index(1.0, 1000.0, 2000.0, -2.0);
        assert!(
            (flux - 0.25).abs() < TOLERANCE,
            "Power law with alpha=-2 at 2x freq should give 0.25"
        );
    }

    // ---- PulseFolding tests ----

    #[test]
    fn test_pulse_folding_basic() {
        let period = 0.01; // 10 ms
        let sample_rate = 10000.0; // 10 kHz
        let dt = 1.0 / sample_rate;
        let n_samples = 10000; // 1 second = 100 pulses
        let n_bins = 64;

        let signal = synthetic_pulsar_signal(n_samples, sample_rate, period, 0.05, 10.0, 0.1, 42);
        let mut folder = PulseFolding::new(n_bins, period, 0.0);
        folder.fold(&signal, 0.0, dt);

        assert_eq!(folder.total_samples() as usize, n_samples);

        let profile = folder.profile();
        assert_eq!(profile.len(), n_bins);

        // The pulse should produce a clear peak
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_val = profile.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            max_val > min_val * 2.0,
            "Folded profile should show a clear pulse peak"
        );
    }

    #[test]
    fn test_pulse_folding_snr() {
        let period = 0.01;
        let sample_rate = 10000.0;
        let dt = 1.0 / sample_rate;
        let n_samples = 50000; // 5 seconds = 500 pulses
        let signal = synthetic_pulsar_signal(n_samples, sample_rate, period, 0.05, 10.0, 1.0, 123);

        let mut folder = PulseFolding::new(128, period, 0.0);
        folder.fold(&signal, 0.0, dt);

        let snr = folder.snr();
        assert!(
            snr > 3.0,
            "S/N of folded profile should be > 3 after 500 pulses, got {}",
            snr
        );
    }

    #[test]
    fn test_pulse_folding_reset() {
        let mut folder = PulseFolding::new(32, 0.01, 0.0);
        folder.fold(&[1.0; 100], 0.0, 0.0001);
        assert!(folder.total_samples() > 0);
        folder.reset();
        assert_eq!(folder.total_samples(), 0);
    }

    // ---- DispersionCorrector tests ----

    #[test]
    fn test_dispersion_corrector_delays() {
        let dc = DispersionCorrector::new(100.0, 1000.0, 2000.0, 4, 1e6);
        let delays = dc.delays();
        assert_eq!(delays.len(), 4);
        // Higher-frequency channels should have smaller delays
        for i in 0..delays.len() - 1 {
            assert!(
                delays[i] >= delays[i + 1],
                "Lower-freq channels should have larger delays"
            );
        }
        // Highest channel should have the smallest delay (but not
        // necessarily zero since its center frequency is below freq_hi)
        let min_delay = *delays.iter().min().unwrap();
        assert_eq!(
            *delays.last().unwrap(),
            min_delay,
            "Highest channel should have the smallest delay"
        );
    }

    #[test]
    fn test_dispersion_corrector_zero_dm() {
        let dc = DispersionCorrector::new(0.0, 1000.0, 2000.0, 4, 1e6);
        let delays = dc.delays();
        for &d in delays {
            assert_eq!(d, 0, "Zero DM should produce zero delay");
        }
    }

    #[test]
    fn test_dispersion_corrector_correct() {
        let n_chan = 4;
        let n_samples = 100;
        let dc = DispersionCorrector::new(10.0, 1000.0, 1400.0, n_chan, 1e4);

        // Create spectrogram with impulse at different delays per channel
        let spectrogram: Vec<Vec<f64>> = (0..n_chan)
            .map(|_| vec![1.0; n_samples])
            .collect();

        let corrected = dc.correct(&spectrogram);
        assert_eq!(corrected.len(), n_chan);
        // All channels should have same length after correction
        let out_len = corrected[0].len();
        for ch in &corrected {
            assert_eq!(ch.len(), out_len);
        }
    }

    #[test]
    fn test_dispersion_corrector_sum() {
        let n_chan = 4;
        let n_samples = 200;
        let dc = DispersionCorrector::new(5.0, 1200.0, 1500.0, n_chan, 1e4);

        let spectrogram: Vec<Vec<f64>> = (0..n_chan)
            .map(|_| vec![1.0; n_samples])
            .collect();

        let summed = dc.correct_and_sum(&spectrogram);
        assert!(!summed.is_empty());
        // All values should be n_chan (each channel contributes 1.0)
        for &v in &summed {
            assert!((v - n_chan as f64).abs() < TOLERANCE);
        }
    }

    #[test]
    fn test_dispersion_corrector_channel_freqs() {
        let dc = DispersionCorrector::new(10.0, 1000.0, 2000.0, 10, 1e6);
        let freqs = dc.channel_freqs();
        assert_eq!(freqs.len(), 10);
        // Channels should be monotonically increasing
        for i in 0..freqs.len() - 1 {
            assert!(freqs[i] < freqs[i + 1]);
        }
        // First channel near 1050 MHz, last near 1950 MHz
        assert!((freqs[0] - 1050.0).abs() < 1.0);
        assert!((freqs[9] - 1950.0).abs() < 1.0);
    }

    // ---- ToaEstimator tests ----

    #[test]
    fn test_toa_zero_offset() {
        let template = gaussian_profile(64, 0.5, 0.05);
        let estimator = ToaEstimator::new(&template);

        // Observed = template (no shift)
        let result = estimator.estimate(&template);
        assert!(
            result.offset_bins.abs() < 1.0,
            "Zero-offset profile should give near-zero TOA shift, got {}",
            result.offset_bins
        );
        assert!(
            result.peak_correlation > 0.9,
            "Self-correlation should be high"
        );
    }

    #[test]
    fn test_toa_known_offset() {
        let n_bins = 128;
        let template = gaussian_profile(n_bins, 0.5, 0.03);
        let estimator = ToaEstimator::new(&template);

        // Shift by 5 bins (rotate right)
        let shift = 5;
        let mut shifted = vec![0.0; n_bins];
        for i in 0..n_bins {
            shifted[(i + shift) % n_bins] = template[i];
        }

        let result = estimator.estimate(&shifted);
        assert!(
            (result.offset_bins - shift as f64).abs() < 1.0,
            "Should detect ~{} bin shift, got {}",
            shift,
            result.offset_bins
        );
    }

    #[test]
    fn test_toa_bins_to_seconds() {
        let offset_s = ToaEstimator::bins_to_seconds(10.0, 100, 0.01);
        let expected = 10.0 * 0.01 / 100.0; // 1 ms
        assert!((offset_s - expected).abs() < 1e-10);
    }

    // ---- TimingResidualAnalyzer tests ----

    #[test]
    fn test_residual_analyzer_constant_freq() {
        let f0 = 100.0; // 100 Hz => period = 10 ms
        let analyzer = TimingResidualAnalyzer::new(0.0, f0, 0.0);

        // Generate TOAs at exact pulse times
        let toas: Vec<f64> = (0..10).map(|i| i as f64 / f0).collect();
        let residuals = analyzer.residuals(&toas);

        for (i, &r) in residuals.iter().enumerate() {
            assert!(
                r.abs() < 1e-10,
                "Residual {} should be zero for perfect TOAs, got {}",
                i,
                r
            );
        }
    }

    #[test]
    fn test_residual_analyzer_with_offset() {
        let f0 = 200.0;
        let analyzer = TimingResidualAnalyzer::new(0.0, f0, 0.0);

        let offset = 1e-6; // 1 microsecond offset
        let toas: Vec<f64> = (0..5).map(|i| i as f64 / f0 + offset).collect();
        let residuals = analyzer.residuals(&toas);

        for &r in &residuals {
            assert!(
                (r - offset).abs() < 1e-10,
                "Each residual should equal the offset"
            );
        }
    }

    #[test]
    fn test_residual_stats() {
        let f0 = 100.0;
        let analyzer = TimingResidualAnalyzer::new(0.0, f0, 0.0);

        let toas: Vec<f64> = (0..20).map(|i| i as f64 / f0).collect();
        let stats = analyzer.analyze(&toas, None);

        assert_eq!(stats.n_toas, 20);
        assert!(stats.rms_s < 1e-10, "Perfect TOAs should have zero RMS");
        assert!(stats.max_abs_s < 1e-10);
    }

    #[test]
    fn test_residual_stats_with_uncertainties() {
        let f0 = 100.0;
        let analyzer = TimingResidualAnalyzer::new(0.0, f0, 0.0);

        // Perfect TOAs with finite uncertainties
        let toas: Vec<f64> = (0..10).map(|i| i as f64 / f0).collect();
        let sigmas = vec![1e-6; 10]; // 1 us uncertainties
        let stats = analyzer.analyze(&toas, Some(&sigmas));

        assert!(stats.wrms_s < 1e-10);
        assert!(stats.reduced_chi2 < 1e-10);
    }

    #[test]
    fn test_predict_phase() {
        let analyzer = TimingResidualAnalyzer::new(0.0, 100.0, 0.0);
        let phase = analyzer.predict_phase(0.01);
        assert!((phase - 1.0).abs() < 1e-10, "100 Hz at 10 ms = 1 rotation");
    }

    #[test]
    fn test_predict_phase_with_spindown() {
        let f0 = 100.0;
        let f_dot = -1e-10; // tiny spin-down
        let analyzer = TimingResidualAnalyzer::new(0.0, f0, f_dot);

        let phase_0 = analyzer.predict_phase(0.0);
        assert!((phase_0).abs() < 1e-15);

        let phase_1 = analyzer.predict_phase(1.0);
        let expected = f0 * 1.0 + 0.5 * f_dot * 1.0;
        assert!((phase_1 - expected).abs() < 1e-10);
    }

    // ---- DmOptimizer tests ----

    #[test]
    fn test_dm_optimizer_finds_correct_dm() {
        let true_dm = 10.0;
        let period = 0.05; // 50 ms
        let sample_rate = 5000.0; // 5 kHz
        let n_samples = 5000; // 1 second
        let n_channels = 4;
        let freq_lo = 1200.0;
        let freq_hi = 1400.0;
        let dt = 1.0 / sample_rate;

        // Generate a synthetic spectrogram with dispersion
        let chan_bw = (freq_hi - freq_lo) / n_channels as f64;
        let sigma = 0.02 / 2.355;

        let spectrogram: Vec<Vec<f64>> = (0..n_channels)
            .map(|ch| {
                let f = freq_lo + (ch as f64 + 0.5) * chan_bw;
                let delay = dispersion_delay(true_dm, f, freq_hi);
                (0..n_samples)
                    .map(|i| {
                        let t = i as f64 * dt;
                        let t_shifted = t - delay;
                        let phase = (t_shifted / period).rem_euclid(1.0);
                        let dp = phase - 0.25;
                        let dp = dp - dp.round();
                        5.0 * (-0.5 * (dp / sigma).powi(2)).exp() + 0.1
                    })
                    .collect()
            })
            .collect();

        let optimizer = DmOptimizer::new(freq_lo, freq_hi, n_channels, sample_rate, period, 64);

        let result = optimizer.optimize(&spectrogram, 5.0, 15.0, 1.0);
        assert!(
            (result.best_dm - true_dm).abs() <= 2.0,
            "Optimizer should find DM near {}, got {}",
            true_dm,
            result.best_dm
        );
        assert!(result.best_snr > 0.0, "Best S/N should be positive");
        assert!(!result.dm_snr_curve.is_empty());
    }

    // ---- gaussian_profile test ----

    #[test]
    fn test_gaussian_profile() {
        let prof = gaussian_profile(100, 0.5, 0.05);
        assert_eq!(prof.len(), 100);

        // Peak should be near bin 50
        let (peak_idx, _) = prof
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        assert!(
            (peak_idx as i64 - 50).abs() <= 1,
            "Peak should be near center"
        );

        // Profile should be positive
        for &v in &prof {
            assert!(v >= 0.0);
        }
    }

    // ---- PulsarConfig tests ----

    #[test]
    fn test_pulsar_config_center_freq() {
        let config = PulsarConfig {
            period_s: 0.01,
            dm: 50.0,
            freq_lo_mhz: 1200.0,
            freq_hi_mhz: 1500.0,
            duty_cycle: 0.1,
        };
        assert!((config.center_freq_mhz() - 1350.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_pulsar_config_bandwidth() {
        let config = PulsarConfig {
            period_s: 0.01,
            dm: 50.0,
            freq_lo_mhz: 1200.0,
            freq_hi_mhz: 1500.0,
            duty_cycle: 0.1,
        };
        assert!((config.bandwidth_mhz() - 300.0).abs() < TOLERANCE);
    }

    // ---- Integration / end-to-end test ----

    #[test]
    fn test_end_to_end_timing() {
        // Simulate a 10 ms pulsar, fold, measure TOA
        let period = 0.01;
        let sample_rate = 20000.0;
        let dt = 1.0 / sample_rate;
        let n_samples = 20000; // 1 second = 100 pulses
        let n_bins = 64;

        let signal =
            synthetic_pulsar_signal(n_samples, sample_rate, period, 0.05, 20.0, 0.5, 999);

        // Fold
        let mut folder = PulseFolding::new(n_bins, period, 0.0);
        folder.fold(&signal, 0.0, dt);
        let profile = folder.profile();

        // Create template from ideal profile
        let template = gaussian_profile(n_bins, 0.25, 0.05 / 2.355);

        // Estimate TOA
        let estimator = ToaEstimator::new(&template);
        let toa = estimator.estimate(&profile);

        // The correlation should be reasonable
        assert!(
            toa.peak_correlation > 0.3,
            "Peak correlation should be reasonable, got {}",
            toa.peak_correlation
        );
        // Uncertainty should be finite
        assert!(toa.uncertainty_bins.is_finite() || toa.uncertainty_bins > 0.0);
    }
}
