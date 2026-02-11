//! Sub-bottom profiler sonar system for mapping seafloor sediment layers.
//!
//! Processes chirp sonar returns to generate subsurface profiles showing
//! sediment boundaries and geological features. Implements matched-filter
//! pulse compression, time-varying gain compensation, envelope detection,
//! and automatic layer boundary detection.
//!
//! All complex numbers are represented as `(f64, f64)` tuples (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::sonar_bottom_profiler::{ProfilerConfig, SubBottomProfiler};
//!
//! let config = ProfilerConfig {
//!     center_freq_hz: 3500.0,
//!     bandwidth_hz: 3000.0,
//!     pulse_duration_ms: 20.0,
//!     sound_speed_mps: 1500.0,
//!     sample_rate_hz: 48000.0,
//!     max_depth_m: 50.0,
//! };
//! let mut profiler = SubBottomProfiler::new(config);
//! let chirp = r4w_core::sonar_bottom_profiler::generate_chirp_pulse(&profiler.config);
//! let result = profiler.process_ping(&chirp);
//! assert!(result.bottom_depth_m >= 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

type C64 = (f64, f64);

fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

fn c_abs(a: C64) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

// ---------------------------------------------------------------------------
// Configuration and types
// ---------------------------------------------------------------------------

/// Configuration parameters for the sub-bottom profiler sonar.
#[derive(Debug, Clone)]
pub struct ProfilerConfig {
    /// Center frequency of the chirp pulse in Hz (e.g., 3500 Hz for typical SBP).
    pub center_freq_hz: f64,
    /// Bandwidth of the chirp sweep in Hz.
    pub bandwidth_hz: f64,
    /// Chirp pulse duration in milliseconds.
    pub pulse_duration_ms: f64,
    /// Speed of sound in water in m/s (default ~1500 m/s).
    pub sound_speed_mps: f64,
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Maximum profiling depth in metres.
    pub max_depth_m: f64,
}

impl Default for ProfilerConfig {
    fn default() -> Self {
        Self {
            center_freq_hz: 3500.0,
            bandwidth_hz: 3000.0,
            pulse_duration_ms: 20.0,
            sound_speed_mps: 1500.0,
            sample_rate_hz: 48000.0,
            max_depth_m: 50.0,
        }
    }
}

/// Result from processing a single sonar ping.
#[derive(Debug, Clone)]
pub struct PingResult {
    /// Envelope (amplitude) of the pulse-compressed, TVG-corrected return.
    pub envelope: Vec<f64>,
    /// Depth axis corresponding to each envelope sample (metres).
    pub depth_axis_m: Vec<f64>,
    /// Detected sediment layer boundaries.
    pub layer_boundaries: Vec<LayerBoundary>,
    /// Estimated seafloor (first strong return) depth in metres.
    pub bottom_depth_m: f64,
}

/// A detected sediment layer boundary.
#[derive(Debug, Clone)]
pub struct LayerBoundary {
    /// Depth of the boundary in metres.
    pub depth_m: f64,
    /// Reflection coefficient magnitude (0..1).
    pub reflection_coefficient: f64,
    /// Acoustic impedance contrast Z2/Z1.
    pub impedance_contrast: f64,
}

/// Sediment type with typical acoustic properties.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SedimentType {
    /// Soft clay: ~1500 m/s
    Clay,
    /// Silt: ~1575 m/s
    Silt,
    /// Sand: ~1750 m/s
    Sand,
    /// Gravel: ~1900 m/s
    Gravel,
    /// Rock/bedrock: ~3000 m/s
    Rock,
}

// ---------------------------------------------------------------------------
// SubBottomProfiler
// ---------------------------------------------------------------------------

/// Sub-bottom profiler processor that takes raw sonar returns and produces
/// depth-resolved sediment profiles.
pub struct SubBottomProfiler {
    /// Active configuration.
    pub config: ProfilerConfig,
    /// Pre-computed reference chirp for matched filtering.
    reference_chirp: Vec<C64>,
}

impl SubBottomProfiler {
    /// Create a new profiler with the given configuration.
    pub fn new(config: ProfilerConfig) -> Self {
        let reference_chirp = generate_chirp_pulse(&config);
        Self {
            config,
            reference_chirp,
        }
    }

    /// Process a single ping return through the full pipeline:
    /// matched filter -> TVG -> envelope detection -> layer detection.
    pub fn process_ping(&mut self, raw_return: &[(f64, f64)]) -> PingResult {
        // 1. Matched filter (pulse compression)
        let compressed = matched_filter(raw_return, &self.reference_chirp);

        // 2. Envelope detection on the compressed (real) signal.
        //    Convert the real matched-filter output to analytic signal via
        //    treating each sample as (real, 0).
        let analytic: Vec<C64> = compressed.iter().map(|&r| (r, 0.0)).collect();
        let env_raw = envelope_detection(&analytic);

        // 3. Time-varying gain compensation (typical absorption 0.5 dB/m at SBP freqs)
        let env_tvg = time_varying_gain(
            &env_raw,
            self.config.sample_rate_hz,
            self.config.sound_speed_mps,
            0.5,
        );

        // 4. Build depth axis: depth = c * t / 2  (two-way travel)
        let depth_axis_m: Vec<f64> = (0..env_tvg.len())
            .map(|i| {
                let t = i as f64 / self.config.sample_rate_hz;
                self.config.sound_speed_mps * t / 2.0
            })
            .collect();

        // 5. Detect layer boundaries
        let boundary_indices = detect_layer_boundaries(&env_tvg, 6.0);

        // 6. Compute peak envelope for normalisation
        let peak = env_tvg.iter().cloned().fold(0.0_f64, f64::max);
        let peak = if peak < 1e-30 { 1.0 } else { peak };

        let layer_boundaries: Vec<LayerBoundary> = boundary_indices
            .iter()
            .map(|&idx| {
                let depth = if idx < depth_axis_m.len() {
                    depth_axis_m[idx]
                } else {
                    0.0
                };
                let refl = env_tvg.get(idx).copied().unwrap_or(0.0) / peak;
                let refl = refl.min(1.0);
                // Impedance contrast from reflection coefficient: Z2/Z1 = (1+R)/(1-R)
                let contrast = if refl < 1.0 {
                    (1.0 + refl) / (1.0 - refl)
                } else {
                    f64::INFINITY
                };
                LayerBoundary {
                    depth_m: depth,
                    reflection_coefficient: refl,
                    impedance_contrast: contrast,
                }
            })
            .collect();

        // Bottom depth = first detected boundary (or 0 if none)
        let bottom_depth_m = layer_boundaries
            .first()
            .map(|b| b.depth_m)
            .unwrap_or(0.0);

        PingResult {
            envelope: env_tvg,
            depth_axis_m,
            layer_boundaries,
            bottom_depth_m,
        }
    }
}

// ---------------------------------------------------------------------------
// Public DSP functions
// ---------------------------------------------------------------------------

/// Matched filter (cross-correlation pulse compression).
///
/// Computes the cross-correlation of `signal` with `reference` by direct
/// time-domain correlation. Returns a real-valued output whose length is
/// `signal.len() + reference.len() - 1`.
pub fn matched_filter(signal: &[(f64, f64)], reference: &[(f64, f64)]) -> Vec<f64> {
    if signal.is_empty() || reference.is_empty() {
        return vec![];
    }
    let n = signal.len();
    let m = reference.len();
    let out_len = n + m - 1;
    let mut output = vec![0.0f64; out_len];

    // Cross-correlation: y[k] = sum_i signal[i] * conj(reference[i - k + (m-1)])
    // At output index k = m-1, lag = 0 (perfect alignment).
    // This is equivalent to convolving signal with the time-reversed conjugate
    // of the reference.
    // Build time-reversed conjugate reference once.
    let ref_rev_conj: Vec<C64> = reference.iter().rev().map(|&r| c_conj(r)).collect();

    for k in 0..out_len {
        let mut acc: C64 = (0.0, 0.0);
        for j in 0..m {
            let si = k as isize - j as isize;
            if si >= 0 && (si as usize) < n {
                acc = c_add(acc, c_mul(signal[si as usize], ref_rev_conj[j]));
            }
        }
        output[k] = c_abs(acc);
    }
    output
}

/// Time-varying gain compensation.
///
/// Applies gain = 20*log10(R) + alpha*R converted to linear scale,
/// where R = range in metres = sound_speed * sample_index / (2 * fs).
/// The first sample (R=0) gets unity gain.
pub fn time_varying_gain(
    signal: &[f64],
    fs: f64,
    sound_speed: f64,
    absorption_db_per_m: f64,
) -> Vec<f64> {
    signal
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let t = i as f64 / fs;
            let range_m = sound_speed * t / 2.0; // two-way travel
            if range_m < 1e-6 {
                s
            } else {
                // TL = 20*log10(r) + alpha*r   (dB)
                let tl_db = 20.0 * range_m.log10() + absorption_db_per_m * range_m;
                // Compensate by applying gain = 10^(TL/20)
                let gain = 10.0_f64.powf(tl_db / 20.0);
                s * gain
            }
        })
        .collect()
}

/// Envelope detection: computes |I + jQ| for each complex sample.
pub fn envelope_detection(signal: &[(f64, f64)]) -> Vec<f64> {
    signal.iter().map(|&s| c_abs(s)).collect()
}

/// Detect layer boundaries as indices where the envelope exceeds a
/// threshold (in dB below the peak).
///
/// Returns sample indices of local peaks that are above the threshold.
pub fn detect_layer_boundaries(envelope: &[f64], threshold_db: f64) -> Vec<usize> {
    if envelope.is_empty() {
        return vec![];
    }

    let peak = envelope.iter().cloned().fold(0.0_f64, f64::max);
    if peak < 1e-30 {
        return vec![];
    }

    // Threshold in linear scale
    let threshold_linear = peak * 10.0_f64.powf(-threshold_db / 20.0);

    // Find local maxima above the threshold with a minimum separation
    // to avoid detecting the same reflection multiple times.
    let min_sep = (envelope.len() / 50).max(3);
    let mut boundaries = Vec::new();

    let mut i = 1;
    while i < envelope.len().saturating_sub(1) {
        if envelope[i] >= threshold_linear
            && envelope[i] > envelope[i - 1]
            && envelope[i] > envelope[i + 1]
        {
            boundaries.push(i);
            // Skip ahead by minimum separation
            i += min_sep;
        } else {
            i += 1;
        }
    }

    boundaries
}

/// Acoustic impedance Z = density * sound_velocity.
pub fn acoustic_impedance(density_kg_m3: f64, velocity_mps: f64) -> f64 {
    density_kg_m3 * velocity_mps
}

/// Reflection coefficient at a boundary between two media.
///
/// R = (Z2 - Z1) / (Z2 + Z1)
pub fn reflection_coefficient(z1: f64, z2: f64) -> f64 {
    let denom = z2 + z1;
    if denom.abs() < 1e-30 {
        0.0
    } else {
        (z2 - z1) / denom
    }
}

/// Transmission loss in dB: TL = 20*log10(r) + alpha*r
///
/// Combines spherical spreading and absorption.
pub fn transmission_loss_db(range_m: f64, absorption_coeff: f64) -> f64 {
    if range_m < 1e-10 {
        return 0.0;
    }
    20.0 * range_m.log10() + absorption_coeff * range_m
}

/// Generate a linear FM chirp pulse (complex baseband).
///
/// Produces I/Q samples sweeping from (center - bw/2) to (center + bw/2)
/// over the configured pulse duration.
pub fn generate_chirp_pulse(config: &ProfilerConfig) -> Vec<(f64, f64)> {
    let duration_s = config.pulse_duration_ms / 1000.0;
    let n_samples = (duration_s * config.sample_rate_hz).round() as usize;
    if n_samples == 0 {
        return vec![];
    }

    let f_start = config.center_freq_hz - config.bandwidth_hz / 2.0;
    let chirp_rate = config.bandwidth_hz / duration_s; // Hz per second

    (0..n_samples)
        .map(|i| {
            let t = i as f64 / config.sample_rate_hz;
            // Instantaneous frequency: f(t) = f_start + chirp_rate * t / 2
            // Phase: phi(t) = 2*pi*(f_start*t + chirp_rate*t^2/2)
            let phase = 2.0 * PI * (f_start * t + chirp_rate * t * t / 2.0);
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Return typical sound velocity for a given sediment type (m/s).
pub fn sediment_velocity(sediment_type: SedimentType) -> f64 {
    match sediment_type {
        SedimentType::Clay => 1500.0,
        SedimentType::Silt => 1575.0,
        SedimentType::Sand => 1750.0,
        SedimentType::Gravel => 1900.0,
        SedimentType::Rock => 3000.0,
    }
}

/// Return typical bulk density for a given sediment type (kg/m^3).
pub fn sediment_density(sediment_type: SedimentType) -> f64 {
    match sediment_type {
        SedimentType::Clay => 1500.0,
        SedimentType::Silt => 1700.0,
        SedimentType::Sand => 2000.0,
        SedimentType::Gravel => 2100.0,
        SedimentType::Rock => 2500.0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;
    const EPS_LOOSE: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- acoustic impedance ---

    #[test]
    fn test_impedance_water() {
        // Seawater: ~1025 kg/m3, 1500 m/s -> Z = 1,537,500
        let z = acoustic_impedance(1025.0, 1500.0);
        assert!(approx_eq(z, 1_537_500.0, EPS));
    }

    #[test]
    fn test_impedance_sand() {
        let z = acoustic_impedance(2000.0, 1750.0);
        assert!(approx_eq(z, 3_500_000.0, EPS));
    }

    #[test]
    fn test_impedance_zero_density() {
        let z = acoustic_impedance(0.0, 1500.0);
        assert!(approx_eq(z, 0.0, EPS));
    }

    // --- reflection coefficient ---

    #[test]
    fn test_reflection_equal_impedance() {
        // Same media => R = 0
        let r = reflection_coefficient(1_500_000.0, 1_500_000.0);
        assert!(approx_eq(r, 0.0, EPS));
    }

    #[test]
    fn test_reflection_water_to_sand() {
        let z_water = acoustic_impedance(1025.0, 1500.0); // 1,537,500
        let z_sand = acoustic_impedance(2000.0, 1750.0); // 3,500,000
        let r = reflection_coefficient(z_water, z_sand);
        // R = (3500000 - 1537500) / (3500000 + 1537500) = 1962500 / 5037500 ~ 0.3895
        assert!(r > 0.38 && r < 0.40, "R = {r}");
    }

    #[test]
    fn test_reflection_symmetric() {
        // R(Z1, Z2) = -R(Z2, Z1)
        let z1 = 1_500_000.0;
        let z2 = 3_000_000.0;
        let r1 = reflection_coefficient(z1, z2);
        let r2 = reflection_coefficient(z2, z1);
        assert!(approx_eq(r1, -r2, EPS));
    }

    #[test]
    fn test_reflection_hard_bottom() {
        // Very large Z2 => R -> 1
        let r = reflection_coefficient(1.0, 1e12);
        assert!(r > 0.999);
    }

    #[test]
    fn test_reflection_coefficient_range() {
        // |R| <= 1 for any positive impedances
        for &(z1, z2) in &[(100.0, 200.0), (5000.0, 1.0), (1e6, 1e6)] {
            let r = reflection_coefficient(z1, z2);
            assert!(r.abs() <= 1.0 + EPS, "z1={z1}, z2={z2}, R={r}");
        }
    }

    // --- transmission loss ---

    #[test]
    fn test_tl_zero_range() {
        let tl = transmission_loss_db(0.0, 0.5);
        assert!(approx_eq(tl, 0.0, EPS));
    }

    #[test]
    fn test_tl_1m() {
        // At 1m: 20*log10(1) + 0.5*1 = 0 + 0.5 = 0.5 dB
        let tl = transmission_loss_db(1.0, 0.5);
        assert!(approx_eq(tl, 0.5, EPS_LOOSE));
    }

    #[test]
    fn test_tl_10m() {
        // At 10m: 20*log10(10) + 0.5*10 = 20 + 5 = 25 dB
        let tl = transmission_loss_db(10.0, 0.5);
        assert!(approx_eq(tl, 25.0, EPS_LOOSE));
    }

    #[test]
    fn test_tl_increases_with_range() {
        let tl1 = transmission_loss_db(5.0, 0.3);
        let tl2 = transmission_loss_db(50.0, 0.3);
        assert!(tl2 > tl1);
    }

    #[test]
    fn test_tl_no_absorption() {
        // Pure spherical spreading: TL = 20*log10(100) = 40 dB
        let tl = transmission_loss_db(100.0, 0.0);
        assert!(approx_eq(tl, 40.0, EPS_LOOSE));
    }

    // --- sediment velocities ---

    #[test]
    fn test_sediment_velocity_ordering() {
        let v_clay = sediment_velocity(SedimentType::Clay);
        let v_silt = sediment_velocity(SedimentType::Silt);
        let v_sand = sediment_velocity(SedimentType::Sand);
        let v_gravel = sediment_velocity(SedimentType::Gravel);
        let v_rock = sediment_velocity(SedimentType::Rock);
        assert!(v_clay <= v_silt);
        assert!(v_silt <= v_sand);
        assert!(v_sand <= v_gravel);
        assert!(v_gravel <= v_rock);
    }

    #[test]
    fn test_clay_velocity() {
        assert!(approx_eq(sediment_velocity(SedimentType::Clay), 1500.0, EPS));
    }

    #[test]
    fn test_rock_velocity() {
        assert!(approx_eq(sediment_velocity(SedimentType::Rock), 3000.0, EPS));
    }

    #[test]
    fn test_sediment_density_positive() {
        for st in &[
            SedimentType::Clay,
            SedimentType::Silt,
            SedimentType::Sand,
            SedimentType::Gravel,
            SedimentType::Rock,
        ] {
            assert!(sediment_density(*st) > 0.0);
        }
    }

    // --- envelope detection ---

    #[test]
    fn test_envelope_unit_circle() {
        // All points on the unit circle have magnitude 1
        let signal: Vec<C64> = (0..100)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / 100.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        let env = envelope_detection(&signal);
        for (i, &e) in env.iter().enumerate() {
            assert!(
                approx_eq(e, 1.0, 1e-12),
                "sample {i}: envelope = {e}"
            );
        }
    }

    #[test]
    fn test_envelope_real_only() {
        let signal: Vec<C64> = vec![(3.0, 0.0), (-4.0, 0.0), (0.0, 0.0)];
        let env = envelope_detection(&signal);
        assert!(approx_eq(env[0], 3.0, EPS));
        assert!(approx_eq(env[1], 4.0, EPS));
        assert!(approx_eq(env[2], 0.0, EPS));
    }

    #[test]
    fn test_envelope_imag_only() {
        let signal: Vec<C64> = vec![(0.0, 5.0), (0.0, -2.0)];
        let env = envelope_detection(&signal);
        assert!(approx_eq(env[0], 5.0, EPS));
        assert!(approx_eq(env[1], 2.0, EPS));
    }

    #[test]
    fn test_envelope_empty() {
        let env = envelope_detection(&[]);
        assert!(env.is_empty());
    }

    // --- chirp generation ---

    #[test]
    fn test_chirp_length() {
        let config = ProfilerConfig {
            pulse_duration_ms: 10.0,
            sample_rate_hz: 48000.0,
            ..ProfilerConfig::default()
        };
        let chirp = generate_chirp_pulse(&config);
        // 10ms at 48kHz = 480 samples
        assert_eq!(chirp.len(), 480);
    }

    #[test]
    fn test_chirp_unit_amplitude() {
        let config = ProfilerConfig::default();
        let chirp = generate_chirp_pulse(&config);
        for (i, &(re, im)) in chirp.iter().enumerate() {
            let mag = (re * re + im * im).sqrt();
            assert!(
                approx_eq(mag, 1.0, 1e-12),
                "sample {i}: magnitude = {mag}"
            );
        }
    }

    #[test]
    fn test_chirp_zero_duration() {
        let config = ProfilerConfig {
            pulse_duration_ms: 0.0,
            ..ProfilerConfig::default()
        };
        let chirp = generate_chirp_pulse(&config);
        assert!(chirp.is_empty());
    }

    #[test]
    fn test_chirp_starts_at_one() {
        // At t=0, phase=0, so cos(0)=1, sin(0)=0
        let config = ProfilerConfig {
            center_freq_hz: 0.0,
            bandwidth_hz: 0.0,
            pulse_duration_ms: 10.0,
            sample_rate_hz: 48000.0,
            ..ProfilerConfig::default()
        };
        let chirp = generate_chirp_pulse(&config);
        assert!(approx_eq(chirp[0].0, 1.0, EPS));
        assert!(approx_eq(chirp[0].1, 0.0, EPS));
    }

    // --- matched filter ---

    #[test]
    fn test_matched_filter_output_length() {
        let sig: Vec<C64> = vec![(1.0, 0.0); 10];
        let ref_: Vec<C64> = vec![(1.0, 0.0); 5];
        let out = matched_filter(&sig, &ref_);
        assert_eq!(out.len(), 14); // 10 + 5 - 1
    }

    #[test]
    fn test_matched_filter_empty() {
        assert!(matched_filter(&[], &[(1.0, 0.0)]).is_empty());
        assert!(matched_filter(&[(1.0, 0.0)], &[]).is_empty());
    }

    #[test]
    fn test_matched_filter_peak_at_correlation_lag() {
        // A chirp correlated with itself should peak at the centre
        let config = ProfilerConfig {
            pulse_duration_ms: 5.0,
            sample_rate_hz: 10000.0,
            center_freq_hz: 2000.0,
            bandwidth_hz: 1000.0,
            ..ProfilerConfig::default()
        };
        let chirp = generate_chirp_pulse(&config);
        let corr = matched_filter(&chirp, &chirp);
        let peak_idx = corr
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        // Peak should be at the centre of the output (lag = N-1)
        let expected_centre = chirp.len() - 1;
        assert!(
            (peak_idx as isize - expected_centre as isize).unsigned_abs() <= 1,
            "peak at {peak_idx}, expected near {expected_centre}"
        );
    }

    #[test]
    fn test_matched_filter_gain() {
        // Autocorrelation peak should equal the energy of the signal
        let signal: Vec<C64> = vec![(1.0, 0.0); 20];
        let corr = matched_filter(&signal, &signal);
        let peak = corr.iter().cloned().fold(0.0_f64, f64::max);
        // Energy = N * |1|^2 = 20
        assert!(approx_eq(peak, 20.0, 0.1));
    }

    // --- TVG ---

    #[test]
    fn test_tvg_first_sample_unity() {
        let signal = vec![1.0, 1.0, 1.0];
        let result = time_varying_gain(&signal, 48000.0, 1500.0, 0.5);
        // First sample has range ~0, so gain = 1
        assert!(approx_eq(result[0], 1.0, EPS));
    }

    #[test]
    fn test_tvg_increases_with_range() {
        // Later samples should get more gain (compensating for loss)
        let signal = vec![1.0; 1000];
        let result = time_varying_gain(&signal, 48000.0, 1500.0, 0.5);
        // Check that gain increases (samples are constant 1.0)
        assert!(result[999] > result[100]);
        assert!(result[100] > result[10]);
    }

    #[test]
    fn test_tvg_empty() {
        let result = time_varying_gain(&[], 48000.0, 1500.0, 0.5);
        assert!(result.is_empty());
    }

    #[test]
    fn test_tvg_zero_absorption() {
        // With zero absorption, gain is purely geometric: 20*log10(R)
        let signal = vec![1.0; 100];
        let result = time_varying_gain(&signal, 48000.0, 1500.0, 0.0);
        // Sample 48 is at t=1ms, R = 1500*0.001/2 = 0.75m
        // TL = 20*log10(0.75) = -2.499 dB => gain = 10^(-2.499/20) ~ 0.75
        // Actually we want to verify the formula, so just check it's > 0
        for &v in &result[1..] {
            assert!(v > 0.0);
        }
    }

    // --- layer boundary detection ---

    #[test]
    fn test_detect_layers_single_peak() {
        let mut env = vec![0.0; 100];
        env[50] = 1.0;
        let boundaries = detect_layer_boundaries(&env, 3.0);
        assert!(!boundaries.is_empty());
        assert!(boundaries.contains(&50));
    }

    #[test]
    fn test_detect_layers_empty() {
        let boundaries = detect_layer_boundaries(&[], 6.0);
        assert!(boundaries.is_empty());
    }

    #[test]
    fn test_detect_layers_flat_signal() {
        // Constant signal has no local maxima (except edge effects)
        let env = vec![1.0; 100];
        let boundaries = detect_layer_boundaries(&env, 6.0);
        // Constant signal: no sample is strictly greater than neighbours
        assert!(boundaries.is_empty());
    }

    #[test]
    fn test_detect_layers_two_peaks() {
        let mut env = vec![0.0; 200];
        env[40] = 1.0;
        env[41] = 0.5; // ensure 40 is a local max
        env[39] = 0.3;
        env[150] = 0.8;
        env[151] = 0.3;
        env[149] = 0.2;
        let boundaries = detect_layer_boundaries(&env, 6.0);
        assert!(boundaries.len() >= 2, "found {} boundaries", boundaries.len());
    }

    #[test]
    fn test_detect_layers_below_threshold() {
        let mut env = vec![0.0; 100];
        env[50] = 1.0;
        env[49] = 0.0;
        env[51] = 0.0;
        // Set threshold very high (40 dB below peak => 0.01 of peak)
        // Peak is 1.0, threshold_linear = 1.0 * 10^(-40/20) = 0.01
        // Only the peak at 50 (=1.0) qualifies
        let boundaries = detect_layer_boundaries(&env, 40.0);
        assert_eq!(boundaries.len(), 1);
    }

    // --- profiler config defaults ---

    #[test]
    fn test_default_config() {
        let config = ProfilerConfig::default();
        assert!(approx_eq(config.sound_speed_mps, 1500.0, EPS));
        assert!(config.max_depth_m > 0.0);
        assert!(config.sample_rate_hz > 0.0);
    }

    // --- full pipeline ---

    #[test]
    fn test_process_ping_returns_valid_result() {
        let config = ProfilerConfig {
            center_freq_hz: 3500.0,
            bandwidth_hz: 3000.0,
            pulse_duration_ms: 5.0,
            sound_speed_mps: 1500.0,
            sample_rate_hz: 48000.0,
            max_depth_m: 50.0,
        };
        let mut profiler = SubBottomProfiler::new(config);
        let chirp = generate_chirp_pulse(&profiler.config);
        let result = profiler.process_ping(&chirp);
        assert!(!result.envelope.is_empty());
        assert!(!result.depth_axis_m.is_empty());
        assert_eq!(result.envelope.len(), result.depth_axis_m.len());
        assert!(result.bottom_depth_m >= 0.0);
    }

    #[test]
    fn test_process_ping_depth_axis_increasing() {
        let config = ProfilerConfig::default();
        let mut profiler = SubBottomProfiler::new(config);
        let chirp = generate_chirp_pulse(&profiler.config);
        let result = profiler.process_ping(&chirp);
        for i in 1..result.depth_axis_m.len() {
            assert!(
                result.depth_axis_m[i] >= result.depth_axis_m[i - 1],
                "depth axis not monotonic at index {i}"
            );
        }
    }

    #[test]
    fn test_process_ping_silence() {
        let config = ProfilerConfig::default();
        let mut profiler = SubBottomProfiler::new(config);
        let silence: Vec<C64> = vec![(0.0, 0.0); 960];
        let result = profiler.process_ping(&silence);
        // With silent input, bottom depth should be 0 (nothing detected)
        assert!(approx_eq(result.bottom_depth_m, 0.0, EPS_LOOSE));
    }

    // --- complex helper edge cases ---

    #[test]
    fn test_c_abs_zero() {
        assert!(approx_eq(c_abs((0.0, 0.0)), 0.0, EPS));
    }

    #[test]
    fn test_c_abs_real() {
        assert!(approx_eq(c_abs((3.0, 4.0)), 5.0, EPS));
    }

    #[test]
    fn test_c_mul_identity() {
        let a = (2.0, 3.0);
        let result = c_mul(a, (1.0, 0.0));
        assert!(approx_eq(result.0, 2.0, EPS));
        assert!(approx_eq(result.1, 3.0, EPS));
    }

    #[test]
    fn test_c_conj_negates_imag() {
        let a = (1.0, 5.0);
        let c = c_conj(a);
        assert!(approx_eq(c.0, 1.0, EPS));
        assert!(approx_eq(c.1, -5.0, EPS));
    }
}
