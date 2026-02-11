//! Propagation Mode Sounder
//!
//! Characterizes radio propagation modes (reflections, diffraction, ducting,
//! scatter) by exciting environments with probe signals and analyzing returns.
//! Estimates mode delay, attenuation, angle of arrival (AoA), and channel
//! dispersion.
//!
//! The sounder generates a linear FM chirp probe, cross-correlates it with
//! received responses on multiple receivers, detects multipath peaks, and
//! estimates per-mode parameters. Output includes RMS delay spread, coherence
//! bandwidth, Rician K-factor, and per-mode classification.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::propagation_mode_sounder::{
//!     PropagationModeSounder, SounderConfig,
//! };
//!
//! let config = SounderConfig {
//!     probe_freq_hz: 2.4e9,
//!     bandwidth_hz: 100e6,
//!     num_receivers: 4,
//!     receiver_spacing_m: 0.0625,
//!     speed_of_light: 3e8,
//! };
//!
//! let sounder = PropagationModeSounder::new(config);
//! let probe = sounder.generate_probe(1e-6);
//! assert!(!probe.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the propagation mode sounder.
#[derive(Debug, Clone)]
pub struct SounderConfig {
    /// Center frequency of the probe signal in Hz.
    pub probe_freq_hz: f64,
    /// Bandwidth of the linear FM chirp probe in Hz.
    pub bandwidth_hz: f64,
    /// Number of receivers in the array (>= 1).
    pub num_receivers: usize,
    /// Spacing between adjacent receivers in metres (uniform linear array).
    pub receiver_spacing_m: f64,
    /// Speed of light in m/s (default 3e8).
    pub speed_of_light: f64,
}

impl Default for SounderConfig {
    fn default() -> Self {
        Self {
            probe_freq_hz: 2.4e9,
            bandwidth_hz: 100e6,
            num_receivers: 4,
            receiver_spacing_m: 0.0625,
            speed_of_light: 3e8,
        }
    }
}

// ---------------------------------------------------------------------------
// Mode classification
// ---------------------------------------------------------------------------

/// Type of propagation mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeType {
    /// Direct line-of-sight path.
    LineOfSight,
    /// Reflection off the ground plane.
    GroundReflection,
    /// Reflection off a wall or vertical surface.
    WallReflection,
    /// Diffraction around an edge or obstacle.
    Diffraction,
    /// Scattering from rough surfaces or small objects.
    Scattering,
    /// Atmospheric or waveguide ducting.
    Ducting,
    /// Cannot be classified.
    Unknown,
}

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// A single detected propagation mode.
#[derive(Debug, Clone)]
pub struct PropagationMode {
    /// Excess delay relative to the earliest arrival, in nanoseconds.
    pub delay_ns: f64,
    /// Attenuation relative to the strongest path, in dB (always <= 0).
    pub attenuation_db: f64,
    /// Angle of arrival in degrees (broadside = 0, end-fire = +/-90).
    pub aoa_deg: f64,
    /// Classified propagation mechanism.
    pub mode_type: ModeType,
    /// Linear power relative to the total received power (0..1).
    pub relative_power: f64,
}

/// A single multipath component extracted from the impulse response.
#[derive(Debug, Clone)]
pub struct MultipathComponent {
    /// Delay in samples from the start of the impulse response.
    pub delay_samples: usize,
    /// Amplitude (magnitude) of the component.
    pub amplitude: f64,
    /// Phase of the component in radians.
    pub phase_rad: f64,
}

/// Results of a channel sounding measurement.
#[derive(Debug, Clone)]
pub struct ChannelSoundingResult {
    /// Detected propagation modes, sorted by delay.
    pub modes: Vec<PropagationMode>,
    /// RMS delay spread in nanoseconds.
    pub rms_delay_spread_ns: f64,
    /// Coherence bandwidth in Hz (1 / (5 * sigma_tau)).
    pub coherence_bandwidth_hz: f64,
    /// Rician K-factor in dB.
    pub k_factor_db: f64,
}

// ---------------------------------------------------------------------------
// Sounder
// ---------------------------------------------------------------------------

/// Propagation mode sounder.
///
/// Generates probe signals, processes multi-receiver responses, and produces
/// a [`ChannelSoundingResult`] characterising the channel.
#[derive(Debug, Clone)]
pub struct PropagationModeSounder {
    config: SounderConfig,
}

impl PropagationModeSounder {
    /// Create a new sounder with the given configuration.
    pub fn new(config: SounderConfig) -> Self {
        Self { config }
    }

    /// Generate a linear FM chirp probe signal.
    ///
    /// The chirp sweeps from `-bandwidth/2` to `+bandwidth/2` around DC
    /// over `duration_s` seconds.  The sample rate is `2 * bandwidth_hz`
    /// (Nyquist).  Returns a vector of `(I, Q)` pairs.
    pub fn generate_probe(&self, duration_s: f64) -> Vec<(f64, f64)> {
        let sample_rate = 2.0 * self.config.bandwidth_hz;
        let n_samples = (duration_s * sample_rate).ceil() as usize;
        if n_samples == 0 {
            return Vec::new();
        }
        let bw = self.config.bandwidth_hz;
        let f_start = -bw / 2.0;
        let chirp_rate = bw / duration_s;
        let dt = 1.0 / sample_rate;

        (0..n_samples)
            .map(|i| {
                let t = i as f64 * dt;
                let phase = 2.0 * PI * (f_start * t + 0.5 * chirp_rate * t * t);
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Sound the channel using a transmitted probe and multi-receiver responses.
    ///
    /// * `probe` - The transmitted probe signal (I, Q pairs).
    /// * `received` - One received signal per receiver element.  Each entry is
    ///   a vector of (I, Q) pairs aligned in time with the probe.
    ///
    /// Returns a [`ChannelSoundingResult`] with detected modes and channel
    /// statistics.
    pub fn sound_channel(
        &self,
        probe: &[(f64, f64)],
        received: &[Vec<(f64, f64)>],
    ) -> ChannelSoundingResult {
        if received.is_empty() || probe.is_empty() {
            return ChannelSoundingResult {
                modes: Vec::new(),
                rms_delay_spread_ns: 0.0,
                coherence_bandwidth_hz: f64::INFINITY,
                k_factor_db: f64::INFINITY,
            };
        }

        let sample_rate = 2.0 * self.config.bandwidth_hz;
        let threshold_db = -20.0; // default peak detection threshold

        // Per-receiver impulse responses and peak detection.
        let impulse_responses: Vec<Vec<(f64, f64)>> = received
            .iter()
            .map(|rx| extract_impulse_response(probe, rx))
            .collect();

        // Detect peaks from the first receiver (reference).
        let ref_ir = &impulse_responses[0];
        let peaks = detect_multipath_peaks(ref_ir, threshold_db);

        if peaks.is_empty() {
            return ChannelSoundingResult {
                modes: Vec::new(),
                rms_delay_spread_ns: 0.0,
                coherence_bandwidth_hz: f64::INFINITY,
                k_factor_db: f64::INFINITY,
            };
        }

        // Find maximum amplitude for relative power and attenuation.
        let max_amp = peaks
            .iter()
            .map(|p| p.amplitude)
            .fold(0.0_f64, f64::max);

        // Expected LOS delay (earliest peak).
        let earliest_delay = peaks
            .iter()
            .map(|p| p.delay_samples)
            .min()
            .unwrap_or(0);
        let los_delay_ns = earliest_delay as f64 / sample_rate * 1e9;

        // Total linear power across all peaks.
        let total_power: f64 = peaks.iter().map(|p| p.amplitude * p.amplitude).sum();

        // Build propagation modes.
        let mut modes: Vec<PropagationMode> = peaks
            .iter()
            .map(|peak| {
                let delay_ns = peak.delay_samples as f64 / sample_rate * 1e9;
                let atten_db = if max_amp > 0.0 {
                    20.0 * (peak.amplitude / max_amp).log10()
                } else {
                    0.0
                };
                let power_lin = peak.amplitude * peak.amplitude;
                let rel_power = if total_power > 0.0 {
                    power_lin / total_power
                } else {
                    0.0
                };

                // AoA: collect the phase of this peak across receivers.
                let aoa = if impulse_responses.len() > 1 {
                    let phases: Vec<f64> = impulse_responses
                        .iter()
                        .map(|ir| {
                            if peak.delay_samples < ir.len() {
                                let (re, im) = ir[peak.delay_samples];
                                im.atan2(re)
                            } else {
                                0.0
                            }
                        })
                        .collect();
                    estimate_aoa(
                        &phases,
                        self.config.probe_freq_hz,
                        self.config.receiver_spacing_m,
                        self.config.speed_of_light,
                    )
                } else {
                    0.0
                };

                let mode_type = classify_mode(delay_ns, aoa, los_delay_ns);

                PropagationMode {
                    delay_ns,
                    attenuation_db: atten_db,
                    aoa_deg: aoa,
                    mode_type,
                    relative_power: rel_power,
                }
            })
            .collect();

        modes.sort_by(|a, b| a.delay_ns.partial_cmp(&b.delay_ns).unwrap());

        let rms_ds = compute_delay_spread(&modes);
        let coh_bw = compute_coherence_bandwidth(rms_ds);
        let k_db = compute_k_factor(&modes);

        ChannelSoundingResult {
            modes,
            rms_delay_spread_ns: rms_ds,
            coherence_bandwidth_hz: coh_bw,
            k_factor_db: k_db,
        }
    }
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Cross-correlate the probe with a received signal (matched filter).
///
/// Returns the complex impulse response as (I, Q) pairs.  The length of
/// the output is `received.len()` (zero-padded probe, sliding correlation).
pub fn extract_impulse_response(
    probe: &[(f64, f64)],
    received: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    if probe.is_empty() || received.is_empty() {
        return Vec::new();
    }

    let n_probe = probe.len();
    let n_rx = received.len();
    // Output length: valid cross-correlation region.
    let out_len = if n_rx >= n_probe {
        n_rx - n_probe + 1
    } else {
        return Vec::new();
    };

    // Compute the energy of the probe for normalisation.
    let probe_energy: f64 = probe.iter().map(|(re, im)| re * re + im * im).sum();
    let norm = if probe_energy > 0.0 {
        1.0 / probe_energy.sqrt()
    } else {
        1.0
    };

    (0..out_len)
        .map(|lag| {
            let mut re_acc = 0.0;
            let mut im_acc = 0.0;
            for k in 0..n_probe {
                let (rx_re, rx_im) = received[lag + k];
                let (pr_re, pr_im) = probe[k];
                // Complex multiply: received * conj(probe)
                re_acc += rx_re * pr_re + rx_im * pr_im;
                im_acc += rx_im * pr_re - rx_re * pr_im;
            }
            (re_acc * norm, im_acc * norm)
        })
        .collect()
}

/// Detect multipath peaks in a complex impulse response.
///
/// `threshold_db` is the detection floor relative to the strongest peak
/// (typically negative, e.g. -20 dB).  Returns peaks sorted by descending
/// amplitude.
pub fn detect_multipath_peaks(
    impulse_response: &[(f64, f64)],
    threshold_db: f64,
) -> Vec<MultipathComponent> {
    if impulse_response.is_empty() {
        return Vec::new();
    }

    // Compute magnitudes.
    let magnitudes: Vec<f64> = impulse_response
        .iter()
        .map(|(re, im)| (re * re + im * im).sqrt())
        .collect();

    let max_mag = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
    if max_mag == 0.0 {
        return Vec::new();
    }

    // Linear threshold.
    let lin_thresh = max_mag * 10.0_f64.powf(threshold_db / 20.0);

    let n = magnitudes.len();
    let mut peaks = Vec::new();

    for i in 0..n {
        let mag = magnitudes[i];
        if mag < lin_thresh {
            continue;
        }

        // Local maximum check (allow edges to be peaks).
        let is_peak = (i == 0 || mag >= magnitudes[i - 1])
            && (i == n - 1 || mag >= magnitudes[i + 1]);

        if is_peak {
            let (re, im) = impulse_response[i];
            peaks.push(MultipathComponent {
                delay_samples: i,
                amplitude: mag,
                phase_rad: im.atan2(re),
            });
        }
    }

    // Sort descending by amplitude.
    peaks.sort_by(|a, b| b.amplitude.partial_cmp(&a.amplitude).unwrap());
    peaks
}

/// Estimate angle of arrival from phase differences across a uniform linear array.
///
/// Uses the average progressive phase difference between adjacent receivers.
///
/// * `phases_across_receivers` - Phase (rad) of the peak at each receiver.
/// * `freq_hz` - Carrier frequency in Hz.
/// * `spacing_m` - Element spacing in metres.
/// * `c` - Speed of light in m/s.
///
/// Returns AoA in degrees. Broadside = 0, end-fire = +/-90.
pub fn estimate_aoa(
    phases_across_receivers: &[f64],
    freq_hz: f64,
    spacing_m: f64,
    c: f64,
) -> f64 {
    if phases_across_receivers.len() < 2 || freq_hz <= 0.0 || spacing_m <= 0.0 || c <= 0.0 {
        return 0.0;
    }

    let lambda = c / freq_hz;
    let n = phases_across_receivers.len();

    // Compute average progressive phase difference (unwrapped).
    let mut total_delta = 0.0;
    for i in 1..n {
        let mut delta = phases_across_receivers[i] - phases_across_receivers[i - 1];
        // Wrap to [-pi, pi].
        while delta > PI {
            delta -= 2.0 * PI;
        }
        while delta < -PI {
            delta += 2.0 * PI;
        }
        total_delta += delta;
    }
    let avg_delta = total_delta / (n - 1) as f64;

    // Phase difference = 2 * pi * d * sin(theta) / lambda
    // => sin(theta) = avg_delta * lambda / (2 * pi * d)
    let sin_theta = avg_delta * lambda / (2.0 * PI * spacing_m);

    // Clamp to valid range.
    let sin_clamped = sin_theta.clamp(-1.0, 1.0);
    sin_clamped.asin() * 180.0 / PI
}

/// Compute RMS delay spread from detected propagation modes.
///
/// RMS delay spread = sqrt( sum(P_i * tau_i^2) / sum(P_i) - (sum(P_i * tau_i) / sum(P_i))^2 )
///
/// All delays are in nanoseconds; result is in nanoseconds.
pub fn compute_delay_spread(modes: &[PropagationMode]) -> f64 {
    if modes.is_empty() {
        return 0.0;
    }

    let total_power: f64 = modes.iter().map(|m| m.relative_power).sum();
    if total_power <= 0.0 {
        return 0.0;
    }

    let mean_delay: f64 = modes
        .iter()
        .map(|m| m.relative_power * m.delay_ns)
        .sum::<f64>()
        / total_power;

    let mean_delay_sq: f64 = modes
        .iter()
        .map(|m| m.relative_power * m.delay_ns * m.delay_ns)
        .sum::<f64>()
        / total_power;

    let variance = mean_delay_sq - mean_delay * mean_delay;
    if variance <= 0.0 {
        0.0
    } else {
        variance.sqrt()
    }
}

/// Compute coherence bandwidth from RMS delay spread.
///
/// B_c = 1 / (5 * sigma_tau)
///
/// `rms_delay_spread_ns` is in nanoseconds; result is in Hz.
pub fn compute_coherence_bandwidth(rms_delay_spread_ns: f64) -> f64 {
    if rms_delay_spread_ns <= 0.0 {
        return f64::INFINITY;
    }
    let sigma_s = rms_delay_spread_ns * 1e-9;
    1.0 / (5.0 * sigma_s)
}

/// Compute Rician K-factor from detected modes.
///
/// K = P_LOS / sum(P_NLOS) expressed in dB.
///
/// The first `LineOfSight` mode is treated as the LOS component.  If no
/// LOS mode exists the strongest mode is used instead.
pub fn compute_k_factor(modes: &[PropagationMode]) -> f64 {
    if modes.is_empty() {
        return f64::INFINITY;
    }

    // Find LOS mode or fall back to strongest.
    let los_idx = modes
        .iter()
        .position(|m| m.mode_type == ModeType::LineOfSight)
        .unwrap_or_else(|| {
            modes
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| {
                    a.relative_power
                        .partial_cmp(&b.relative_power)
                        .unwrap()
                })
                .map(|(i, _)| i)
                .unwrap_or(0)
        });

    let los_power = modes[los_idx].relative_power;
    let nlos_power: f64 = modes
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != los_idx)
        .map(|(_, m)| m.relative_power)
        .sum();

    if nlos_power <= 0.0 {
        // Pure LOS, K -> infinity.
        return f64::INFINITY;
    }

    10.0 * (los_power / nlos_power).log10()
}

/// Heuristic classification of a propagation mode.
///
/// * `delay_ns` - Excess delay of this mode.
/// * `aoa_deg` - Estimated angle of arrival in degrees.
/// * `expected_los_delay_ns` - Expected LOS delay (earliest arrival).
///
/// Rules:
/// - Delay within 5 ns of LOS and |AoA| < 15: `LineOfSight`
/// - Delay < 200 ns and |AoA| between 15 and 45: `GroundReflection`
/// - Delay < 500 ns and |AoA| > 45: `WallReflection`
/// - Delay between 200 ns and 2000 ns and low power implied by moderate AoA: `Diffraction`
/// - Delay > 2000 ns: `Ducting`
/// - Otherwise: `Scattering`
pub fn classify_mode(delay_ns: f64, aoa_deg: f64, expected_los_delay_ns: f64) -> ModeType {
    let excess = (delay_ns - expected_los_delay_ns).abs();
    let abs_aoa = aoa_deg.abs();

    if excess < 5.0 && abs_aoa < 15.0 {
        ModeType::LineOfSight
    } else if excess < 200.0 && abs_aoa >= 15.0 && abs_aoa < 45.0 {
        ModeType::GroundReflection
    } else if excess < 500.0 && abs_aoa >= 45.0 {
        ModeType::WallReflection
    } else if excess >= 200.0 && excess < 2000.0 {
        ModeType::Diffraction
    } else if excess >= 2000.0 {
        ModeType::Ducting
    } else {
        ModeType::Scattering
    }
}

/// Free-space path loss in dB.
///
/// FSPL = 20 * log10(4 * pi * d * f / c)
///
/// * `freq_hz` - Carrier frequency in Hz.
/// * `distance_m` - Distance in metres.
///
/// Returns FSPL in dB (positive value).  Returns 0.0 for non-positive
/// distance or frequency.
pub fn free_space_path_loss_db(freq_hz: f64, distance_m: f64) -> f64 {
    if freq_hz <= 0.0 || distance_m <= 0.0 {
        return 0.0;
    }
    let c = 3e8_f64;
    20.0 * (4.0 * PI * distance_m * freq_hz / c).log10()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // Helper: generate a simple delayed and scaled copy of a probe.
    fn delayed_probe(
        probe: &[(f64, f64)],
        delay_samples: usize,
        scale: f64,
        total_len: usize,
    ) -> Vec<(f64, f64)> {
        let mut out = vec![(0.0, 0.0); total_len];
        for (i, &(re, im)) in probe.iter().enumerate() {
            let idx = i + delay_samples;
            if idx < total_len {
                out[idx].0 += re * scale;
                out[idx].1 += im * scale;
            }
        }
        out
    }

    // Helper: add phase shift to a signal (simulates AoA on array element).
    fn phase_shift_signal(signal: &[(f64, f64)], phase_rad: f64) -> Vec<(f64, f64)> {
        let (cp, sp) = (phase_rad.cos(), phase_rad.sin());
        signal
            .iter()
            .map(|&(re, im)| (re * cp - im * sp, re * sp + im * cp))
            .collect()
    }

    // -----------------------------------------------------------------------
    // 1. Probe signal generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_probe_generation_length() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-3);
        // sample_rate = 2 * 1e6 = 2e6, duration = 1e-3 => 2000 samples
        assert_eq!(probe.len(), 2000);
    }

    #[test]
    fn test_probe_generation_zero_duration() {
        let sounder = PropagationModeSounder::new(SounderConfig::default());
        let probe = sounder.generate_probe(0.0);
        assert!(probe.is_empty());
    }

    #[test]
    fn test_probe_unit_amplitude() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        for &(re, im) in &probe {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < TOL, "Chirp sample should have unit magnitude");
        }
    }

    #[test]
    fn test_probe_chirp_bandwidth_via_instantaneous_freq() {
        // Verify the chirp sweeps approximately the configured bandwidth by
        // checking the instantaneous frequency at start and end.
        let bw = 1e6_f64;
        let duration = 1e-3;
        let config = SounderConfig {
            bandwidth_hz: bw,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(duration);

        // Instantaneous frequency from phase difference between adjacent samples.
        let sample_rate = 2.0 * bw;
        let inst_freq = |i: usize| {
            let (re0, im0) = probe[i];
            let (re1, im1) = probe[i + 1];
            // Phase difference.
            let dp_re = re1 * re0 + im1 * im0;
            let dp_im = im1 * re0 - re1 * im0;
            dp_im.atan2(dp_re) * sample_rate / (2.0 * PI)
        };

        let f_start = inst_freq(0);
        let f_end = inst_freq(probe.len() - 2);

        // Start frequency should be near -bw/2, end near +bw/2.
        assert!(
            (f_start - (-bw / 2.0)).abs() < bw * 0.05,
            "Start freq {f_start} should be near {}",
            -bw / 2.0
        );
        assert!(
            (f_end - (bw / 2.0)).abs() < bw * 0.05,
            "End freq {f_end} should be near {}",
            bw / 2.0
        );
    }

    // -----------------------------------------------------------------------
    // 2. Impulse response extraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_impulse_response_single_path() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        let total_len = probe.len() + 50;

        // Create received signal: probe delayed by 10 samples, unity scale.
        let received = delayed_probe(&probe, 10, 1.0, total_len);
        let ir = extract_impulse_response(&probe, &received);

        // Peak should be at sample 10.
        let peak_idx = ir
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                let ma = (a.0 * a.0 + a.1 * a.1).sqrt();
                let mb = (b.0 * b.0 + b.1 * b.1).sqrt();
                ma.partial_cmp(&mb).unwrap()
            })
            .unwrap()
            .0;

        assert_eq!(peak_idx, 10, "IR peak should be at delay 10");
    }

    #[test]
    fn test_impulse_response_empty_inputs() {
        let ir = extract_impulse_response(&[], &[(1.0, 0.0)]);
        assert!(ir.is_empty());

        let ir2 = extract_impulse_response(&[(1.0, 0.0)], &[]);
        assert!(ir2.is_empty());
    }

    #[test]
    fn test_impulse_response_probe_longer_than_received() {
        let probe = vec![(1.0, 0.0); 100];
        let received = vec![(1.0, 0.0); 50];
        let ir = extract_impulse_response(&probe, &received);
        assert!(ir.is_empty(), "Should return empty when probe > received");
    }

    #[test]
    fn test_impulse_response_two_reflections() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        let total_len = probe.len() + 100;

        let mut received = delayed_probe(&probe, 10, 1.0, total_len);
        let echo = delayed_probe(&probe, 50, 0.5, total_len);
        for (i, &(re, im)) in echo.iter().enumerate() {
            received[i].0 += re;
            received[i].1 += im;
        }

        let ir = extract_impulse_response(&probe, &received);
        let peaks = detect_multipath_peaks(&ir, -10.0);

        assert!(peaks.len() >= 2, "Should detect at least 2 peaks, got {}", peaks.len());

        // Verify both delays are present.
        let delays: Vec<usize> = peaks.iter().map(|p| p.delay_samples).collect();
        assert!(delays.contains(&10), "Should find peak at delay 10");
        assert!(delays.contains(&50), "Should find peak at delay 50");
    }

    #[test]
    fn test_cross_correlation_peak_alignment() {
        // A simple known cross-correlation: delta function response.
        let probe = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let mut received = vec![(0.0, 0.0); 20];
        for (i, &s) in probe.iter().enumerate() {
            received[5 + i] = s;
        }
        let ir = extract_impulse_response(&probe, &received);

        // Peak should be at lag 5.
        let peak_idx = ir
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                let ma = (a.0 * a.0 + a.1 * a.1).sqrt();
                let mb = (b.0 * b.0 + b.1 * b.1).sqrt();
                ma.partial_cmp(&mb).unwrap()
            })
            .unwrap()
            .0;
        assert_eq!(peak_idx, 5);
    }

    // -----------------------------------------------------------------------
    // 3. Peak detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_peaks_single() {
        let ir: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                if i == 30 {
                    (1.0, 0.0)
                } else {
                    (0.0, 0.0)
                }
            })
            .collect();

        let peaks = detect_multipath_peaks(&ir, -30.0);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].delay_samples, 30);
    }

    #[test]
    fn test_detect_peaks_threshold_filtering() {
        // Two peaks: one at amplitude 1.0, another at 0.01 (-40 dB).
        let mut ir = vec![(0.0, 0.0); 100];
        ir[20] = (1.0, 0.0);
        ir[60] = (0.01, 0.0);

        // Threshold -30 dB should only keep the first peak.
        let peaks = detect_multipath_peaks(&ir, -30.0);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].delay_samples, 20);

        // Threshold -50 dB should keep both.
        let peaks2 = detect_multipath_peaks(&ir, -50.0);
        assert_eq!(peaks2.len(), 2);
    }

    #[test]
    fn test_detect_peaks_empty_ir() {
        let peaks = detect_multipath_peaks(&[], -20.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_detect_peaks_all_zeros() {
        let ir = vec![(0.0, 0.0); 50];
        let peaks = detect_multipath_peaks(&ir, -20.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_detect_peaks_descending_order() {
        let mut ir = vec![(0.0, 0.0); 100];
        ir[10] = (0.5, 0.0);
        ir[30] = (1.0, 0.0);
        ir[50] = (0.3, 0.0);

        let peaks = detect_multipath_peaks(&ir, -20.0);
        assert!(peaks.len() >= 2);
        // Should be sorted descending by amplitude.
        for i in 1..peaks.len() {
            assert!(peaks[i].amplitude <= peaks[i - 1].amplitude);
        }
    }

    #[test]
    fn test_detect_peaks_phase_extraction() {
        let mut ir = vec![(0.0, 0.0); 50];
        // Peak with known phase = pi/4.
        let angle = PI / 4.0;
        ir[20] = (angle.cos(), angle.sin());

        let peaks = detect_multipath_peaks(&ir, -30.0);
        assert_eq!(peaks.len(), 1);
        assert!(
            (peaks[0].phase_rad - PI / 4.0).abs() < 1e-10,
            "Phase should be pi/4"
        );
    }

    // -----------------------------------------------------------------------
    // 4. AoA estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_aoa_broadside() {
        // All receivers see the same phase => broadside (0 degrees).
        let phases = vec![0.0, 0.0, 0.0, 0.0];
        let aoa = estimate_aoa(&phases, 2.4e9, 0.0625, 3e8);
        assert!(aoa.abs() < 1.0, "Broadside should give ~0 degrees, got {aoa}");
    }

    #[test]
    fn test_aoa_known_angle_30deg() {
        // sin(30 deg) = 0.5.  Phase difference = 2*pi*d*sin(theta)/lambda.
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0; // half-wavelength spacing
        let theta_rad = 30.0_f64.to_radians();
        let delta_phase = 2.0 * PI * d * theta_rad.sin() / lambda;

        let phases: Vec<f64> = (0..4).map(|i| i as f64 * delta_phase).collect();
        let aoa = estimate_aoa(&phases, freq, d, c);
        assert!(
            (aoa - 30.0).abs() < 0.5,
            "AoA should be ~30 degrees, got {aoa}"
        );
    }

    #[test]
    fn test_aoa_negative_angle() {
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;
        let theta_rad = (-45.0_f64).to_radians();
        let delta_phase = 2.0 * PI * d * theta_rad.sin() / lambda;

        let phases: Vec<f64> = (0..4).map(|i| i as f64 * delta_phase).collect();
        let aoa = estimate_aoa(&phases, freq, d, c);
        assert!(
            (aoa - (-45.0)).abs() < 0.5,
            "AoA should be ~-45 degrees, got {aoa}"
        );
    }

    #[test]
    fn test_aoa_single_receiver() {
        let aoa = estimate_aoa(&[0.5], 2.4e9, 0.0625, 3e8);
        assert_eq!(aoa, 0.0, "Single receiver should give 0 degrees");
    }

    #[test]
    fn test_aoa_endfire() {
        // sin(90 deg) = 1.  d = lambda/2 => delta_phase = pi.
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;
        let delta_phase = PI; // 2*pi*d*1/lambda = 2*pi*(lambda/2)/lambda = pi

        let phases: Vec<f64> = (0..4).map(|i| i as f64 * delta_phase).collect();
        let aoa = estimate_aoa(&phases, freq, d, c);
        assert!(
            (aoa - 90.0).abs() < 1.0,
            "End-fire should give ~90 degrees, got {aoa}"
        );
    }

    #[test]
    fn test_aoa_phase_wrapping() {
        // Phases that wrap around 2*pi should still give correct AoA.
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;
        let theta_rad = 20.0_f64.to_radians();
        let delta_phase = 2.0 * PI * d * theta_rad.sin() / lambda;

        // Add multiples of 2*pi to some phases.
        let phases = vec![
            0.0,
            delta_phase + 2.0 * PI,
            2.0 * delta_phase - 2.0 * PI,
            3.0 * delta_phase + 4.0 * PI,
        ];
        let aoa = estimate_aoa(&phases, freq, d, c);
        assert!(
            (aoa - 20.0).abs() < 1.0,
            "Should handle phase wrapping, got {aoa}"
        );
    }

    #[test]
    fn test_multi_receiver_phase_consistency() {
        // With uniform progressive phase, all adjacent pairs should give
        // the same AoA estimate.
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;
        let theta = 25.0_f64;
        let theta_rad = theta.to_radians();
        let delta_phase = 2.0 * PI * d * theta_rad.sin() / lambda;

        for n_rx in 2..=8 {
            let phases: Vec<f64> = (0..n_rx).map(|i| i as f64 * delta_phase).collect();
            let aoa = estimate_aoa(&phases, freq, d, c);
            assert!(
                (aoa - theta).abs() < 1.0,
                "With {} receivers, AoA should be ~{} degrees, got {}",
                n_rx,
                theta,
                aoa
            );
        }
    }

    // -----------------------------------------------------------------------
    // 5. Delay spread
    // -----------------------------------------------------------------------

    #[test]
    fn test_delay_spread_single_mode() {
        let modes = vec![PropagationMode {
            delay_ns: 100.0,
            attenuation_db: 0.0,
            aoa_deg: 0.0,
            mode_type: ModeType::LineOfSight,
            relative_power: 1.0,
        }];
        let ds = compute_delay_spread(&modes);
        assert!(ds.abs() < TOL, "Single mode should have 0 delay spread");
    }

    #[test]
    fn test_delay_spread_two_equal_modes() {
        let modes = vec![
            PropagationMode {
                delay_ns: 0.0,
                attenuation_db: 0.0,
                aoa_deg: 0.0,
                mode_type: ModeType::LineOfSight,
                relative_power: 0.5,
            },
            PropagationMode {
                delay_ns: 100.0,
                attenuation_db: -3.0,
                aoa_deg: 30.0,
                mode_type: ModeType::GroundReflection,
                relative_power: 0.5,
            },
        ];
        let ds = compute_delay_spread(&modes);
        // Two equal power paths at 0 and 100 ns:
        // mean = 50, mean_sq = 5000, variance = 5000 - 2500 = 2500, rms = 50
        assert!(
            (ds - 50.0).abs() < TOL,
            "RMS delay spread should be 50 ns, got {ds}"
        );
    }

    #[test]
    fn test_delay_spread_empty() {
        let ds = compute_delay_spread(&[]);
        assert_eq!(ds, 0.0);
    }

    #[test]
    fn test_delay_spread_unequal_power() {
        // Strong LOS at 0 ns, weak reflection at 200 ns.
        let modes = vec![
            PropagationMode {
                delay_ns: 0.0,
                attenuation_db: 0.0,
                aoa_deg: 0.0,
                mode_type: ModeType::LineOfSight,
                relative_power: 0.9,
            },
            PropagationMode {
                delay_ns: 200.0,
                attenuation_db: -10.0,
                aoa_deg: 30.0,
                mode_type: ModeType::GroundReflection,
                relative_power: 0.1,
            },
        ];
        let ds = compute_delay_spread(&modes);
        // mean = 0.1*200 = 20, mean_sq = 0.1*40000 = 4000
        // variance = 4000 - 400 = 3600, rms = 60
        assert!(
            (ds - 60.0).abs() < TOL,
            "RMS delay spread should be 60 ns, got {ds}"
        );
    }

    // -----------------------------------------------------------------------
    // 6. Coherence bandwidth
    // -----------------------------------------------------------------------

    #[test]
    fn test_coherence_bandwidth_known_value() {
        // sigma_tau = 100 ns => B_c = 1 / (5 * 100e-9) = 2 MHz
        let bw = compute_coherence_bandwidth(100.0);
        assert!(
            (bw - 2e6).abs() < 1.0,
            "Coherence BW should be 2 MHz, got {bw}"
        );
    }

    #[test]
    fn test_coherence_bandwidth_inverse_relationship() {
        let bw1 = compute_coherence_bandwidth(50.0);
        let bw2 = compute_coherence_bandwidth(100.0);
        assert!(
            bw1 > bw2,
            "Smaller delay spread should give larger coherence BW"
        );
        // bw1 should be 2x bw2.
        assert!(
            (bw1 / bw2 - 2.0).abs() < TOL,
            "Doubling delay spread should halve coherence BW"
        );
    }

    #[test]
    fn test_coherence_bandwidth_zero_spread() {
        let bw = compute_coherence_bandwidth(0.0);
        assert!(bw.is_infinite(), "Zero delay spread -> infinite coherence BW");
    }

    // -----------------------------------------------------------------------
    // 7. K-factor
    // -----------------------------------------------------------------------

    #[test]
    fn test_k_factor_pure_los() {
        let modes = vec![PropagationMode {
            delay_ns: 0.0,
            attenuation_db: 0.0,
            aoa_deg: 0.0,
            mode_type: ModeType::LineOfSight,
            relative_power: 1.0,
        }];
        let k = compute_k_factor(&modes);
        assert!(k.is_infinite(), "Pure LOS should give K -> infinity");
    }

    #[test]
    fn test_k_factor_equal_los_nlos() {
        let modes = vec![
            PropagationMode {
                delay_ns: 0.0,
                attenuation_db: 0.0,
                aoa_deg: 0.0,
                mode_type: ModeType::LineOfSight,
                relative_power: 0.5,
            },
            PropagationMode {
                delay_ns: 100.0,
                attenuation_db: -3.0,
                aoa_deg: 30.0,
                mode_type: ModeType::GroundReflection,
                relative_power: 0.5,
            },
        ];
        let k = compute_k_factor(&modes);
        // K = 0.5/0.5 = 1 => 0 dB.
        assert!(
            k.abs() < 0.1,
            "Equal LOS/NLOS should give K ~ 0 dB, got {k}"
        );
    }

    #[test]
    fn test_k_factor_strong_multipath() {
        let modes = vec![
            PropagationMode {
                delay_ns: 0.0,
                attenuation_db: 0.0,
                aoa_deg: 0.0,
                mode_type: ModeType::LineOfSight,
                relative_power: 0.1,
            },
            PropagationMode {
                delay_ns: 50.0,
                attenuation_db: -1.0,
                aoa_deg: 20.0,
                mode_type: ModeType::GroundReflection,
                relative_power: 0.3,
            },
            PropagationMode {
                delay_ns: 200.0,
                attenuation_db: -2.0,
                aoa_deg: 50.0,
                mode_type: ModeType::WallReflection,
                relative_power: 0.6,
            },
        ];
        let k = compute_k_factor(&modes);
        // K = 0.1 / 0.9 => -9.54 dB.
        let expected = 10.0 * (0.1_f64 / 0.9).log10();
        assert!(
            (k - expected).abs() < 0.01,
            "K should be {expected:.2} dB, got {k:.2}"
        );
    }

    #[test]
    fn test_k_factor_empty() {
        let k = compute_k_factor(&[]);
        assert!(k.is_infinite());
    }

    // -----------------------------------------------------------------------
    // 8. Mode classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_los() {
        let mt = classify_mode(100.0, 5.0, 98.0);
        assert_eq!(mt, ModeType::LineOfSight);
    }

    #[test]
    fn test_classify_ground_reflection() {
        let mt = classify_mode(150.0, 25.0, 100.0);
        assert_eq!(mt, ModeType::GroundReflection);
    }

    #[test]
    fn test_classify_wall_reflection() {
        let mt = classify_mode(200.0, 60.0, 100.0);
        assert_eq!(mt, ModeType::WallReflection);
    }

    #[test]
    fn test_classify_diffraction() {
        let mt = classify_mode(500.0, 10.0, 100.0);
        assert_eq!(mt, ModeType::Diffraction);
    }

    #[test]
    fn test_classify_ducting() {
        let mt = classify_mode(5000.0, 5.0, 100.0);
        assert_eq!(mt, ModeType::Ducting);
    }

    #[test]
    fn test_classify_scattering_fallback() {
        // Excess < 200, AoA < 15, but excess > 5 => Scattering.
        let mt = classify_mode(110.0, 5.0, 100.0);
        assert_eq!(mt, ModeType::Scattering);
    }

    // -----------------------------------------------------------------------
    // 9. Free-space path loss
    // -----------------------------------------------------------------------

    #[test]
    fn test_fspl_known_value() {
        // 2.4 GHz, 100 m.
        // FSPL = 20*log10(4*pi*100*2.4e9/3e8)
        //      = 20*log10(4*pi*800)
        //      = 20*log10(10053.1) ~ 80.05 dB
        let fspl = free_space_path_loss_db(2.4e9, 100.0);
        let expected = 20.0 * (4.0 * PI * 100.0 * 2.4e9 / 3e8).log10();
        assert!(
            (fspl - expected).abs() < 0.01,
            "FSPL should be {expected:.2}, got {fspl:.2}"
        );
    }

    #[test]
    fn test_fspl_doubles_with_distance() {
        // Doubling distance adds ~6 dB.
        let fspl1 = free_space_path_loss_db(1e9, 100.0);
        let fspl2 = free_space_path_loss_db(1e9, 200.0);
        let diff = fspl2 - fspl1;
        assert!(
            (diff - 6.02).abs() < 0.1,
            "Doubling distance should add ~6 dB, got {diff:.2}"
        );
    }

    #[test]
    fn test_fspl_zero_distance() {
        let fspl = free_space_path_loss_db(1e9, 0.0);
        assert_eq!(fspl, 0.0);
    }

    #[test]
    fn test_fspl_zero_frequency() {
        let fspl = free_space_path_loss_db(0.0, 100.0);
        assert_eq!(fspl, 0.0);
    }

    // -----------------------------------------------------------------------
    // 10. Full channel sounding pipeline
    // -----------------------------------------------------------------------

    #[test]
    fn test_sound_channel_single_receiver_single_path() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            num_receivers: 1,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        let total_len = probe.len() + 50;

        let rx = delayed_probe(&probe, 5, 1.0, total_len);
        let result = sounder.sound_channel(&probe, &[rx]);

        assert!(!result.modes.is_empty(), "Should detect at least one mode");
        // Chirp matched filter sidelobes may produce minor secondary peaks;
        // delay spread should still be small relative to the probe duration.
        let probe_duration_ns = 1e-4 * 1e9; // 100_000 ns
        assert!(
            result.rms_delay_spread_ns < probe_duration_ns * 0.01,
            "Single path delay spread ({:.1} ns) should be small vs probe ({:.0} ns)",
            result.rms_delay_spread_ns,
            probe_duration_ns,
        );
    }

    #[test]
    fn test_sound_channel_two_paths() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            num_receivers: 1,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        let total_len = probe.len() + 100;

        let mut rx = delayed_probe(&probe, 5, 1.0, total_len);
        let echo = delayed_probe(&probe, 40, 0.5, total_len);
        for (i, &(re, im)) in echo.iter().enumerate() {
            rx[i].0 += re;
            rx[i].1 += im;
        }

        let result = sounder.sound_channel(&probe, &[rx]);
        assert!(
            result.modes.len() >= 2,
            "Should detect 2 modes, got {}",
            result.modes.len()
        );
        assert!(result.rms_delay_spread_ns > 0.0, "Should have non-zero delay spread");
        assert!(
            result.coherence_bandwidth_hz.is_finite(),
            "Should have finite coherence BW"
        );
    }

    #[test]
    fn test_sound_channel_empty_received() {
        let sounder = PropagationModeSounder::new(SounderConfig::default());
        let probe = sounder.generate_probe(1e-5);
        let result = sounder.sound_channel(&probe, &[]);
        assert!(result.modes.is_empty());
    }

    #[test]
    fn test_sound_channel_multi_receiver_aoa() {
        // Two receivers, signal arriving from broadside.
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;

        let config = SounderConfig {
            probe_freq_hz: freq,
            bandwidth_hz: 10e6,
            num_receivers: 4,
            receiver_spacing_m: d,
            speed_of_light: c,
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-5);
        let total_len = probe.len() + 30;

        // All receivers see the same signal (broadside, AoA = 0).
        let rx0 = delayed_probe(&probe, 5, 1.0, total_len);
        let rxs: Vec<Vec<(f64, f64)>> = (0..4).map(|_| rx0.clone()).collect();

        let result = sounder.sound_channel(&probe, &rxs);
        assert!(!result.modes.is_empty());
        // AoA should be near 0 for broadside.
        assert!(
            result.modes[0].aoa_deg.abs() < 5.0,
            "Broadside AoA should be ~0, got {}",
            result.modes[0].aoa_deg
        );
    }

    #[test]
    fn test_sound_channel_multi_receiver_oblique_aoa() {
        let freq = 3e9;
        let c = 3e8;
        let lambda = c / freq;
        let d = lambda / 2.0;
        let theta = 30.0_f64;
        let theta_rad = theta.to_radians();
        let delta_phase = 2.0 * PI * d * theta_rad.sin() / lambda;

        let config = SounderConfig {
            probe_freq_hz: freq,
            bandwidth_hz: 10e6,
            num_receivers: 4,
            receiver_spacing_m: d,
            speed_of_light: c,
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-5);
        let total_len = probe.len() + 30;

        let rx_base = delayed_probe(&probe, 5, 1.0, total_len);
        let rxs: Vec<Vec<(f64, f64)>> = (0..4)
            .map(|i| phase_shift_signal(&rx_base, i as f64 * delta_phase))
            .collect();

        let result = sounder.sound_channel(&probe, &rxs);
        assert!(!result.modes.is_empty());
        assert!(
            (result.modes[0].aoa_deg - theta).abs() < 5.0,
            "AoA should be ~{} degrees, got {}",
            theta,
            result.modes[0].aoa_deg
        );
    }

    // -----------------------------------------------------------------------
    // 11. Edge cases and miscellaneous
    // -----------------------------------------------------------------------

    #[test]
    fn test_default_config() {
        let config = SounderConfig::default();
        assert_eq!(config.probe_freq_hz, 2.4e9);
        assert_eq!(config.bandwidth_hz, 100e6);
        assert_eq!(config.num_receivers, 4);
        assert_eq!(config.speed_of_light, 3e8);
    }

    #[test]
    fn test_mode_type_equality() {
        assert_eq!(ModeType::LineOfSight, ModeType::LineOfSight);
        assert_ne!(ModeType::LineOfSight, ModeType::Scattering);
        assert_ne!(ModeType::Ducting, ModeType::Unknown);
    }

    #[test]
    fn test_propagation_mode_clone() {
        let mode = PropagationMode {
            delay_ns: 50.0,
            attenuation_db: -3.0,
            aoa_deg: 15.0,
            mode_type: ModeType::GroundReflection,
            relative_power: 0.4,
        };
        let cloned = mode.clone();
        assert_eq!(cloned.delay_ns, mode.delay_ns);
        assert_eq!(cloned.mode_type, mode.mode_type);
    }

    #[test]
    fn test_result_modes_sorted_by_delay() {
        let config = SounderConfig {
            bandwidth_hz: 1e6,
            num_receivers: 1,
            ..Default::default()
        };
        let sounder = PropagationModeSounder::new(config);
        let probe = sounder.generate_probe(1e-4);
        let total_len = probe.len() + 100;

        // Insert three paths at delays 40, 10, 70.
        let mut rx = delayed_probe(&probe, 40, 0.8, total_len);
        let p2 = delayed_probe(&probe, 10, 1.0, total_len);
        let p3 = delayed_probe(&probe, 70, 0.3, total_len);
        for i in 0..total_len {
            rx[i].0 += p2[i].0 + p3[i].0;
            rx[i].1 += p2[i].1 + p3[i].1;
        }

        let result = sounder.sound_channel(&probe, &[rx]);
        for i in 1..result.modes.len() {
            assert!(
                result.modes[i].delay_ns >= result.modes[i - 1].delay_ns,
                "Modes should be sorted by delay"
            );
        }
    }

    #[test]
    fn test_aoa_invalid_params() {
        // Zero frequency.
        assert_eq!(estimate_aoa(&[0.0, 0.1], 0.0, 0.05, 3e8), 0.0);
        // Zero spacing.
        assert_eq!(estimate_aoa(&[0.0, 0.1], 1e9, 0.0, 3e8), 0.0);
        // Zero speed of light.
        assert_eq!(estimate_aoa(&[0.0, 0.1], 1e9, 0.05, 0.0), 0.0);
    }
}
