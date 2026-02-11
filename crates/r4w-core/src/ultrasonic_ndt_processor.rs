//! Ultrasonic Non-Destructive Testing (NDT) signal processing for flaw detection.
//!
//! This module implements ultrasonic NDT signal processing for detecting and
//! characterizing flaws in materials. It processes A-scan (amplitude vs. time),
//! B-scan (cross-section), and C-scan (plan view) data from ultrasonic transducers.
//!
//! # Features
//!
//! - **A-scan processing** with gating, peak detection, and flaw classification
//! - **Distance-Amplitude Correction (DAC)** curve interpolation
//! - **Time-Corrected Gain (TCG)** for depth-dependent attenuation compensation
//! - **Snell's law** refraction angle computation for angle beam inspection
//! - **Near field** and **beam divergence** calculations
//! - **Envelope detection** via Hilbert transform approximation
//! - **Synthetic A-scan generation** for testing and calibration
//!
//! # Example
//!
//! ```
//! use r4w_core::ultrasonic_ndt_processor::{NdtConfig, NdtProcessor, Gate};
//!
//! let config = NdtConfig {
//!     transducer_freq_mhz: 5.0,
//!     sample_rate_hz: 100_000_000.0,
//!     sound_velocity_mps: 5900.0,
//!     beam_diameter_mm: 10.0,
//!     dead_zone_mm: 5.0,
//! };
//! let processor = NdtProcessor::new(config);
//!
//! // Generate a synthetic A-scan with a flaw at 30mm and back wall at 50mm
//! let signal = r4w_core::ultrasonic_ndt_processor::generate_ascan_signal(
//!     50.0,
//!     &[(30.0, 0.6)],
//!     &processor.config,
//! );
//!
//! let gates = vec![
//!     Gate { id: 1, start_mm: 10.0, end_mm: 45.0, threshold: 0.2 },
//! ];
//! let result = processor.process_ascan(&signal, &gates);
//! assert!(result.back_wall_depth_mm > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Classification of detected flaw types in ultrasonic NDT.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlawType {
    /// Linear planar defect (high amplitude, sharp echo).
    Crack,
    /// Distributed gas voids (moderate amplitude, broad echo).
    Porosity,
    /// Foreign material embedded in the matrix (moderate amplitude).
    Inclusion,
    /// Incomplete bonding in a weld joint.
    LackOfFusion,
    /// Layer separation in composite or laminated material.
    Delamination,
    /// Flaw type could not be determined.
    Unknown,
}

/// A detected flaw indication from A-scan analysis.
#[derive(Debug, Clone)]
pub struct FlawIndication {
    /// Depth of the flaw from the surface in millimeters.
    pub depth_mm: f64,
    /// Peak amplitude as a percentage of full-screen height (0-100).
    pub amplitude_percent: f64,
    /// Equivalent reflector size in millimeters (DGS/AVG method estimate).
    pub equivalent_size_mm: f64,
    /// Classified flaw type based on echo characteristics.
    pub flaw_type: FlawType,
}

/// Result of processing a single gate on an A-scan.
#[derive(Debug, Clone)]
pub struct GateResult {
    /// Gate identifier.
    pub gate_id: usize,
    /// Peak amplitude within the gate (0.0 to 1.0 scale).
    pub peak_amplitude: f64,
    /// Depth of the peak within the gate in millimeters.
    pub peak_depth_mm: f64,
    /// Time-of-flight to the peak in microseconds.
    pub tof_us: f64,
}

/// Complete result of A-scan processing.
#[derive(Debug, Clone)]
pub struct AScanResult {
    /// Results for each configured gate.
    pub gates: Vec<GateResult>,
    /// Detected back-wall echo depth in millimeters.
    pub back_wall_depth_mm: f64,
    /// Back-wall echo amplitude (0.0 to 1.0 scale).
    pub back_wall_amplitude: f64,
    /// All flaw indications found above gate thresholds.
    pub indications: Vec<FlawIndication>,
}

/// Gate definition for A-scan evaluation.
#[derive(Debug, Clone)]
pub struct Gate {
    /// Gate identifier.
    pub id: usize,
    /// Start depth in millimeters.
    pub start_mm: f64,
    /// End depth in millimeters.
    pub end_mm: f64,
    /// Amplitude threshold (0.0 to 1.0). Peaks above this trigger indications.
    pub threshold: f64,
}

/// Configuration for the ultrasonic NDT processor.
#[derive(Debug, Clone)]
pub struct NdtConfig {
    /// Transducer centre frequency in MHz.
    pub transducer_freq_mhz: f64,
    /// Digitiser sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Longitudinal sound velocity in the test material (m/s).
    /// Default: 5900.0 (steel).
    pub sound_velocity_mps: f64,
    /// Active element diameter in millimeters.
    pub beam_diameter_mm: f64,
    /// Dead zone from the surface where echoes are unreliable (mm).
    pub dead_zone_mm: f64,
}

impl Default for NdtConfig {
    fn default() -> Self {
        Self {
            transducer_freq_mhz: 5.0,
            sample_rate_hz: 100_000_000.0,
            sound_velocity_mps: 5900.0,
            beam_diameter_mm: 10.0,
            dead_zone_mm: 5.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Convert a pulse-echo time-of-flight to depth.
///
/// In pulse-echo mode the sound travels to the reflector and back, so
/// depth = velocity * tof / 2. The input `tof_us` is in microseconds and
/// `velocity_mps` is in metres per second. Returns depth in millimetres.
pub fn time_to_depth(tof_us: f64, velocity_mps: f64) -> f64 {
    // depth_m = v * t / 2
    // depth_mm = depth_m * 1000
    velocity_mps * tof_us * 1e-6 / 2.0 * 1000.0
}

/// Convert a depth to the expected pulse-echo time-of-flight.
///
/// `depth_mm` in millimetres, `velocity_mps` in metres per second.
/// Returns time-of-flight in microseconds.
pub fn depth_to_time(depth_mm: f64, velocity_mps: f64) -> f64 {
    // tof = 2 * depth / velocity
    2.0 * (depth_mm / 1000.0) / velocity_mps * 1e6
}

/// Distance-Amplitude Correction (DAC) curve interpolation.
///
/// Given a set of `reference_amplitudes` as (depth_mm, amplitude) pairs from
/// calibration reflectors, linearly interpolates the expected amplitude at
/// `depth_mm`. This compensates for beam spread and material attenuation so
/// that reflectors of equal size at different depths produce equal screen height.
///
/// Returns the interpolated reference amplitude. Extrapolates linearly beyond
/// the calibration range.
pub fn dac_curve(reference_amplitudes: &[(f64, f64)], depth_mm: f64) -> f64 {
    if reference_amplitudes.is_empty() {
        return 0.0;
    }
    if reference_amplitudes.len() == 1 {
        return reference_amplitudes[0].1;
    }

    // Sort by depth (work on a copy).
    let mut sorted: Vec<(f64, f64)> = reference_amplitudes.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Clamp / extrapolate
    if depth_mm <= sorted[0].0 {
        // Extrapolate from first two points
        let (d0, a0) = sorted[0];
        let (d1, a1) = sorted[1];
        if (d1 - d0).abs() < 1e-12 {
            return a0;
        }
        let slope = (a1 - a0) / (d1 - d0);
        return a0 + slope * (depth_mm - d0);
    }
    if depth_mm >= sorted[sorted.len() - 1].0 {
        let n = sorted.len();
        let (d0, a0) = sorted[n - 2];
        let (d1, a1) = sorted[n - 1];
        if (d1 - d0).abs() < 1e-12 {
            return a1;
        }
        let slope = (a1 - a0) / (d1 - d0);
        return a1 + slope * (depth_mm - d1);
    }

    // Find the bracketing interval
    for i in 0..sorted.len() - 1 {
        let (d0, a0) = sorted[i];
        let (d1, a1) = sorted[i + 1];
        if depth_mm >= d0 && depth_mm <= d1 {
            if (d1 - d0).abs() < 1e-12 {
                return a0;
            }
            let t = (depth_mm - d0) / (d1 - d0);
            return a0 + t * (a1 - a0);
        }
    }

    // Fallback (should not reach here)
    sorted.last().unwrap().1
}

/// Time-Corrected Gain (TCG).
///
/// Computes the gain in dB required to compensate for material attenuation at
/// a given depth. The model is linear: gain = 2 * depth * attenuation (factor
/// of 2 for pulse-echo round-trip).
///
/// `depth_mm` is the reflector depth in millimetres, `attenuation_db_per_mm`
/// is the material's attenuation coefficient.
pub fn tcg_gain(depth_mm: f64, attenuation_db_per_mm: f64) -> f64 {
    // Round-trip path: 2 * depth
    2.0 * depth_mm * attenuation_db_per_mm
}

/// Apply Snell's law for refraction at a material boundary.
///
/// Given an incident angle `angle_deg` (in degrees), velocities `v1` and `v2`
/// (in m/s) for the two media, returns the refracted angle in degrees.
///
/// Returns `None` if total internal reflection occurs (sin(refracted) > 1).
pub fn snell_law_angle(angle_deg: f64, v1: f64, v2: f64) -> Option<f64> {
    let sin_incident = (angle_deg * PI / 180.0).sin();
    let sin_refracted = sin_incident * v2 / v1;
    if sin_refracted.abs() > 1.0 {
        None // Total internal reflection
    } else {
        Some(sin_refracted.asin() * 180.0 / PI)
    }
}

/// Calculate the near-field length (Fresnel zone) for a circular transducer.
///
/// N = D^2 * f / (4 * v)
///
/// where D is the element diameter in mm, f is frequency in MHz, and v is
/// the sound velocity in m/s. Returns the near-field length in millimetres.
pub fn near_field_length(diameter_mm: f64, freq_mhz: f64, velocity_mps: f64) -> f64 {
    // Convert to consistent units:
    //   D in metres = diameter_mm * 1e-3
    //   f in Hz = freq_mhz * 1e6
    //   N_m = D_m^2 * f_Hz / (4 * v_mps)
    //   N_mm = N_m * 1e3
    let d_m = diameter_mm * 1e-3;
    let f_hz = freq_mhz * 1e6;
    let n_m = d_m * d_m * f_hz / (4.0 * velocity_mps);
    n_m * 1e3
}

/// Calculate the half-angle beam divergence for a circular transducer.
///
/// sin(theta) = 1.22 * lambda / D
///
/// where lambda = v / f. Returns the half-angle in degrees.
pub fn beam_divergence_deg(diameter_mm: f64, freq_mhz: f64, velocity_mps: f64) -> f64 {
    let lambda_m = velocity_mps / (freq_mhz * 1e6);
    let d_m = diameter_mm * 1e-3;
    let sin_theta = 1.22 * lambda_m / d_m;
    // Clamp to valid asin range for very small transducers
    let sin_theta = sin_theta.min(1.0);
    sin_theta.asin() * 180.0 / PI
}

/// Envelope detection via a discrete Hilbert-transform approximation.
///
/// Computes the analytic signal envelope |x + j*H(x)| where H(x) is the
/// Hilbert transform. Uses a frequency-domain approach with an internal DFT
/// (Bluestein / radix-2 style) to produce the envelope of `rf_signal`.
///
/// This is a pure-Rust implementation with no external FFT dependency.
pub fn envelope_detection(rf_signal: &[f64]) -> Vec<f64> {
    let n = rf_signal.len();
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![rf_signal[0].abs()];
    }

    // Zero-pad to next power of 2 for efficient DFT
    let nfft = n.next_power_of_two();
    let mut re = vec![0.0; nfft];
    let mut im = vec![0.0; nfft];
    for (i, &v) in rf_signal.iter().enumerate() {
        re[i] = v;
    }

    // Forward DFT (Cooley-Tukey radix-2)
    fft_in_place(&mut re, &mut im, false);

    // Apply Hilbert spectral mask:
    //   DC and Nyquist: unchanged (multiply by 1)
    //   Positive frequencies: multiply by 2
    //   Negative frequencies: multiply by 0
    // Index 0 = DC
    // Index 1..nfft/2 = positive frequencies
    // Index nfft/2 = Nyquist
    // Index nfft/2+1..nfft-1 = negative frequencies
    let half = nfft / 2;
    for i in 1..half {
        re[i] *= 2.0;
        im[i] *= 2.0;
    }
    // Nyquist (index half) stays as-is
    for i in (half + 1)..nfft {
        re[i] = 0.0;
        im[i] = 0.0;
    }

    // Inverse DFT
    fft_in_place(&mut re, &mut im, true);

    // Envelope = magnitude of analytic signal
    let mut envelope = Vec::with_capacity(n);
    for i in 0..n {
        envelope.push((re[i] * re[i] + im[i] * im[i]).sqrt());
    }
    envelope
}

/// Generate a synthetic A-scan signal for testing purposes.
///
/// Creates a realistic ultrasonic RF waveform with:
/// - An initial pulse (bang) near t=0
/// - Flaw echoes at specified (depth_mm, amplitude) locations
/// - A back-wall echo at `back_wall_depth_mm`
/// - Background noise
///
/// The returned signal is normalised so the back-wall echo has amplitude ~1.0.
pub fn generate_ascan_signal(
    back_wall_depth_mm: f64,
    flaws: &[(f64, f64)],
    config: &NdtConfig,
) -> Vec<f64> {
    let total_tof_us = depth_to_time(back_wall_depth_mm * 1.2, config.sound_velocity_mps);
    let n_samples = (total_tof_us * 1e-6 * config.sample_rate_hz).ceil() as usize;
    let n_samples = n_samples.max(256);

    let mut signal = vec![0.0f64; n_samples];

    let freq_hz = config.transducer_freq_mhz * 1e6;
    let dt = 1.0 / config.sample_rate_hz;

    // Helper: add a tone-burst (windowed sinusoid) centred at a given depth
    let add_echo = |sig: &mut Vec<f64>, depth_mm: f64, amplitude: f64| {
        let tof_s = 2.0 * (depth_mm / 1000.0) / config.sound_velocity_mps;
        let centre_sample = (tof_s / dt).round() as i64;
        // Number of cycles in the burst
        let n_cycles = 3.0;
        let burst_duration_s = n_cycles / freq_hz;
        let half_burst_samples = (burst_duration_s / dt / 2.0).round() as i64;

        let start = (centre_sample - half_burst_samples).max(0) as usize;
        let end = ((centre_sample + half_burst_samples) as usize).min(sig.len());

        for i in start..end {
            let t = i as f64 * dt;
            let t_rel = t - tof_s;
            // Gaussian window
            let sigma = burst_duration_s / 4.0;
            let window = (-0.5 * (t_rel / sigma).powi(2)).exp();
            sig[i] += amplitude * window * (2.0 * PI * freq_hz * t_rel).sin();
        }
    };

    // Initial pulse in dead zone
    add_echo(&mut signal, config.dead_zone_mm * 0.3, 0.8);

    // Flaw echoes
    for &(depth, amp) in flaws {
        add_echo(&mut signal, depth, amp);
    }

    // Back-wall echo
    add_echo(&mut signal, back_wall_depth_mm, 1.0);

    // Add a small amount of noise (deterministic PRNG for reproducibility)
    let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_1234;
    for sample in signal.iter_mut() {
        // Simple xorshift64
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 7;
        rng_state ^= rng_state << 17;
        let noise = (rng_state as f64 / u64::MAX as f64) * 2.0 - 1.0;
        *sample += noise * 0.01;
    }

    signal
}

// ---------------------------------------------------------------------------
// NdtProcessor
// ---------------------------------------------------------------------------

/// Ultrasonic NDT signal processor.
///
/// Processes digitised RF A-scan waveforms to detect and characterise flaws.
pub struct NdtProcessor {
    /// Processor configuration.
    pub config: NdtConfig,
}

impl NdtProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: NdtConfig) -> Self {
        Self { config }
    }

    /// Process an A-scan RF signal with the specified gates.
    ///
    /// Performs envelope detection, gate evaluation, back-wall detection, and
    /// flaw classification. Returns an [`AScanResult`] with all findings.
    pub fn process_ascan(&self, rf_signal: &[f64], gates: &[Gate]) -> AScanResult {
        if rf_signal.is_empty() {
            return AScanResult {
                gates: vec![],
                back_wall_depth_mm: 0.0,
                back_wall_amplitude: 0.0,
                indications: vec![],
            };
        }

        let envelope = envelope_detection(rf_signal);
        let dt = 1.0 / self.config.sample_rate_hz;

        // Find global maximum for normalisation
        let max_env = envelope.iter().cloned().fold(0.0f64, f64::max);
        let norm = if max_env > 1e-15 { max_env } else { 1.0 };

        // Normalised envelope
        let env_norm: Vec<f64> = envelope.iter().map(|&v| v / norm).collect();

        // --- Back-wall detection ---
        // The back-wall echo is expected to be the strongest echo after the
        // dead zone. Find the peak in the normalised envelope beyond the dead zone.
        let dead_zone_samples = self.depth_to_sample(self.config.dead_zone_mm, dt);
        let mut bw_sample = dead_zone_samples;
        let mut bw_amp = 0.0;
        for i in dead_zone_samples..env_norm.len() {
            if env_norm[i] > bw_amp {
                bw_amp = env_norm[i];
                bw_sample = i;
            }
        }
        let bw_tof_us = bw_sample as f64 * dt * 1e6;
        let bw_depth_mm = time_to_depth(bw_tof_us, self.config.sound_velocity_mps);

        // --- Gate evaluation ---
        let mut gate_results = Vec::with_capacity(gates.len());
        let mut indications = Vec::new();

        for gate in gates {
            let start_sample = self.depth_to_sample(gate.start_mm, dt);
            let end_sample = self.depth_to_sample(gate.end_mm, dt).min(env_norm.len());

            let mut peak_amp = 0.0;
            let mut peak_sample = start_sample;

            for i in start_sample..end_sample {
                if env_norm[i] > peak_amp {
                    peak_amp = env_norm[i];
                    peak_sample = i;
                }
            }

            let peak_tof_us = peak_sample as f64 * dt * 1e6;
            let peak_depth = time_to_depth(peak_tof_us, self.config.sound_velocity_mps);

            gate_results.push(GateResult {
                gate_id: gate.id,
                peak_amplitude: peak_amp,
                peak_depth_mm: peak_depth,
                tof_us: peak_tof_us,
            });

            // If peak exceeds the gate threshold and is not the back-wall echo,
            // record a flaw indication.
            if peak_amp > gate.threshold && (peak_depth - bw_depth_mm).abs() > 2.0 {
                let flaw_type = self.classify_flaw(&env_norm, peak_sample);
                let eq_size = self.estimate_equivalent_size(peak_amp, peak_depth);

                indications.push(FlawIndication {
                    depth_mm: peak_depth,
                    amplitude_percent: peak_amp * 100.0,
                    equivalent_size_mm: eq_size,
                    flaw_type,
                });
            }
        }

        AScanResult {
            gates: gate_results,
            back_wall_depth_mm: bw_depth_mm,
            back_wall_amplitude: bw_amp,
            indications,
        }
    }

    // --- Internal helpers ---

    /// Convert a depth in mm to a sample index.
    fn depth_to_sample(&self, depth_mm: f64, dt: f64) -> usize {
        let tof_s = 2.0 * (depth_mm / 1000.0) / self.config.sound_velocity_mps;
        (tof_s / dt).round() as usize
    }

    /// Classify a flaw based on the local echo shape in the envelope.
    fn classify_flaw(&self, envelope: &[f64], peak_idx: usize) -> FlawType {
        let n = envelope.len();
        if n < 3 || peak_idx == 0 || peak_idx >= n - 1 {
            return FlawType::Unknown;
        }

        // Measure the echo width at -6 dB (50% amplitude)
        let half_amp = envelope[peak_idx] * 0.5;

        // Search left
        let mut left = peak_idx;
        while left > 0 && envelope[left] > half_amp {
            left -= 1;
        }

        // Search right
        let mut right = peak_idx;
        while right < n - 1 && envelope[right] > half_amp {
            right += 1;
        }

        let width = right - left;
        let peak_amp = envelope[peak_idx];

        // Rise sharpness: how quickly does the echo rise?
        let rise_len = peak_idx - left;
        let fall_len = right - peak_idx;

        // Heuristic classification
        if peak_amp > 0.8 && width < 10 && rise_len <= 3 {
            // Sharp, high-amplitude echo: likely a crack
            FlawType::Crack
        } else if width > 30 && peak_amp < 0.5 {
            // Broad, moderate-amplitude: likely porosity (scattered reflections)
            FlawType::Porosity
        } else if rise_len > 0 && fall_len > 0 {
            let asymmetry = (rise_len as f64 - fall_len as f64).abs()
                / (rise_len as f64 + fall_len as f64);
            if asymmetry > 0.4 {
                // Asymmetric echo: could be lack of fusion
                FlawType::LackOfFusion
            } else if peak_amp > 0.5 {
                // Symmetric moderate echo
                FlawType::Inclusion
            } else {
                FlawType::Unknown
            }
        } else {
            FlawType::Unknown
        }
    }

    /// Estimate equivalent reflector size using a simplified DGS/AVG approach.
    ///
    /// This is a rough estimate based on the amplitude and depth. In practice,
    /// a full DGS diagram (Distance-Gain-Size) from the transducer manufacturer
    /// would be used.
    fn estimate_equivalent_size(&self, amplitude: f64, depth_mm: f64) -> f64 {
        // Simplified model: size ~ amplitude * (depth / near_field)^0.5 * beam_diameter
        let nf = near_field_length(
            self.config.beam_diameter_mm,
            self.config.transducer_freq_mhz,
            self.config.sound_velocity_mps,
        );
        let depth_ratio = (depth_mm / nf).max(0.1);
        amplitude * depth_ratio.sqrt() * self.config.beam_diameter_mm * 0.1
    }
}

// ---------------------------------------------------------------------------
// Internal FFT (Cooley-Tukey radix-2, in-place)
// ---------------------------------------------------------------------------

/// In-place radix-2 FFT / IFFT.
///
/// `re` and `im` must have length equal to a power of 2.
/// Set `inverse` to `true` for IFFT (includes 1/N scaling).
fn fft_in_place(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");
    assert_eq!(re.len(), im.len());

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey butterfly
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_sign = if inverse { 1.0 } else { -1.0 };
        let w_step = angle_sign * 2.0 * PI / size as f64;

        let mut k = 0;
        while k < n {
            for j_idx in 0..half {
                let angle = w_step * j_idx as f64;
                let wr = angle.cos();
                let wi = angle.sin();

                let a = k + j_idx;
                let b = a + half;

                let tr = wr * re[b] - wi * im[b];
                let ti = wr * im[b] + wi * re[b];

                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
            k += size;
        }
        size <<= 1;
    }

    // Scale for inverse
    if inverse {
        let scale = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= scale;
            im[i] *= scale;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn default_config() -> NdtConfig {
        NdtConfig::default()
    }

    // --- time_to_depth / depth_to_time ---

    #[test]
    fn test_time_to_depth_steel() {
        // 10 us TOF in steel (5900 m/s) => depth = 5900 * 10e-6 / 2 * 1000 = 29.5 mm
        let depth = time_to_depth(10.0, 5900.0);
        assert!((depth - 29.5).abs() < EPSILON, "depth = {depth}");
    }

    #[test]
    fn test_depth_to_time_steel() {
        // 29.5 mm depth in steel => TOF = 2 * 0.0295 / 5900 * 1e6 = 10.0 us
        let tof = depth_to_time(29.5, 5900.0);
        assert!((tof - 10.0).abs() < EPSILON, "tof = {tof}");
    }

    #[test]
    fn test_time_depth_roundtrip() {
        let depth = 42.0;
        let vel = 5900.0;
        let tof = depth_to_time(depth, vel);
        let recovered = time_to_depth(tof, vel);
        assert!(
            (recovered - depth).abs() < EPSILON,
            "roundtrip failed: {recovered} != {depth}"
        );
    }

    #[test]
    fn test_time_to_depth_zero() {
        assert!((time_to_depth(0.0, 5900.0)).abs() < EPSILON);
    }

    #[test]
    fn test_time_to_depth_water() {
        // Water: v = 1480 m/s, TOF = 20 us => depth = 1480 * 20e-6 / 2 * 1000 = 14.8 mm
        let depth = time_to_depth(20.0, 1480.0);
        assert!((depth - 14.8).abs() < EPSILON, "depth = {depth}");
    }

    // --- DAC curve ---

    #[test]
    fn test_dac_curve_interpolation() {
        let refs = vec![(10.0, 0.9), (30.0, 0.6), (50.0, 0.3)];
        let val = dac_curve(&refs, 20.0);
        // Linear interp between (10, 0.9) and (30, 0.6): 0.9 + (0.6-0.9)*(10/20) = 0.75
        assert!((val - 0.75).abs() < EPSILON, "dac = {val}");
    }

    #[test]
    fn test_dac_curve_at_calibration_point() {
        let refs = vec![(10.0, 0.9), (30.0, 0.6)];
        let val = dac_curve(&refs, 10.0);
        assert!((val - 0.9).abs() < EPSILON, "dac = {val}");
    }

    #[test]
    fn test_dac_curve_extrapolation() {
        let refs = vec![(10.0, 0.9), (30.0, 0.6)];
        // Extrapolate beyond 30: slope = (0.6-0.9)/(30-10) = -0.015/mm
        // At 40 mm: 0.6 + (-0.015)*(40-30) = 0.45
        let val = dac_curve(&refs, 40.0);
        assert!((val - 0.45).abs() < EPSILON, "dac = {val}");
    }

    #[test]
    fn test_dac_curve_single_point() {
        let refs = vec![(20.0, 0.8)];
        assert!((dac_curve(&refs, 20.0) - 0.8).abs() < EPSILON);
        assert!((dac_curve(&refs, 50.0) - 0.8).abs() < EPSILON);
    }

    #[test]
    fn test_dac_curve_empty() {
        assert!((dac_curve(&[], 10.0)).abs() < EPSILON);
    }

    // --- TCG gain ---

    #[test]
    fn test_tcg_gain_basic() {
        // 50 mm depth, 0.01 dB/mm attenuation => gain = 2 * 50 * 0.01 = 1.0 dB
        let gain = tcg_gain(50.0, 0.01);
        assert!((gain - 1.0).abs() < EPSILON, "gain = {gain}");
    }

    #[test]
    fn test_tcg_gain_zero_depth() {
        assert!((tcg_gain(0.0, 0.05)).abs() < EPSILON);
    }

    #[test]
    fn test_tcg_gain_zero_attenuation() {
        assert!((tcg_gain(100.0, 0.0)).abs() < EPSILON);
    }

    // --- Snell's law ---

    #[test]
    fn test_snell_law_normal_incidence() {
        // 0 degree incidence => 0 degree refraction
        let angle = snell_law_angle(0.0, 5900.0, 3230.0).unwrap();
        assert!(angle.abs() < EPSILON, "angle = {angle}");
    }

    #[test]
    fn test_snell_law_plexiglass_to_steel() {
        // Plexiglass (2730 m/s) to steel long. (5900 m/s), 20 deg incident
        let angle = snell_law_angle(20.0, 2730.0, 5900.0).unwrap();
        // sin(refracted) = sin(20) * 5900/2730 = 0.3420 * 2.1612 = 0.7392
        let expected = (0.3420 * 5900.0 / 2730.0f64).asin() * 180.0 / PI;
        assert!(
            (angle - expected).abs() < 0.1,
            "angle = {angle}, expected = {expected}"
        );
    }

    #[test]
    fn test_snell_law_total_internal_reflection() {
        // Large angle from slow to fast medium => total internal reflection
        let result = snell_law_angle(60.0, 2730.0, 5900.0);
        assert!(result.is_none(), "expected total internal reflection");
    }

    #[test]
    fn test_snell_law_equal_velocities() {
        let angle = snell_law_angle(30.0, 5900.0, 5900.0).unwrap();
        assert!((angle - 30.0).abs() < EPSILON, "angle = {angle}");
    }

    // --- Near field ---

    #[test]
    fn test_near_field_length_calculation() {
        // D = 10 mm, f = 5 MHz, v = 5900 m/s
        // N = (0.010)^2 * 5e6 / (4 * 5900) = 100e-6 * 5e6 / 23600 = 500 / 23600 = 0.02119 m = 21.19 mm
        let nf = near_field_length(10.0, 5.0, 5900.0);
        let expected = (0.010f64.powi(2) * 5e6) / (4.0 * 5900.0) * 1e3;
        assert!((nf - expected).abs() < 0.01, "nf = {nf}, expected = {expected}");
    }

    #[test]
    fn test_near_field_increases_with_frequency() {
        let nf_5 = near_field_length(10.0, 5.0, 5900.0);
        let nf_10 = near_field_length(10.0, 10.0, 5900.0);
        assert!(nf_10 > nf_5, "near field should increase with frequency");
        assert!(
            (nf_10 / nf_5 - 2.0).abs() < EPSILON,
            "should double with double frequency"
        );
    }

    #[test]
    fn test_near_field_increases_with_diameter() {
        let nf_10 = near_field_length(10.0, 5.0, 5900.0);
        let nf_20 = near_field_length(20.0, 5.0, 5900.0);
        assert!(
            (nf_20 / nf_10 - 4.0).abs() < EPSILON,
            "should quadruple with double diameter"
        );
    }

    // --- Beam divergence ---

    #[test]
    fn test_beam_divergence_calculation() {
        // D = 10mm, f = 5MHz, v = 5900 m/s
        // lambda = 5900 / 5e6 = 0.00118 m
        // sin(theta) = 1.22 * 0.00118 / 0.010 = 0.14396
        // theta = 8.28 deg
        let div = beam_divergence_deg(10.0, 5.0, 5900.0);
        let lambda = 5900.0 / 5e6;
        let expected = (1.22 * lambda / 0.010f64).asin() * 180.0 / PI;
        assert!(
            (div - expected).abs() < 0.01,
            "div = {div}, expected = {expected}"
        );
    }

    #[test]
    fn test_beam_divergence_decreases_with_frequency() {
        let div_5 = beam_divergence_deg(10.0, 5.0, 5900.0);
        let div_10 = beam_divergence_deg(10.0, 10.0, 5900.0);
        assert!(
            div_10 < div_5,
            "divergence should decrease with higher frequency"
        );
    }

    #[test]
    fn test_beam_divergence_decreases_with_diameter() {
        let div_10 = beam_divergence_deg(10.0, 5.0, 5900.0);
        let div_20 = beam_divergence_deg(20.0, 5.0, 5900.0);
        assert!(
            div_20 < div_10,
            "divergence should decrease with larger diameter"
        );
    }

    // --- Envelope detection ---

    #[test]
    fn test_envelope_of_pure_sinusoid() {
        // Envelope of sin(2*pi*f*t) should be approximately 1.0
        let n = 256;
        let freq = 10.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / n as f64).sin())
            .collect();

        let env = envelope_detection(&signal);
        assert_eq!(env.len(), n);

        // Check that the middle portion of the envelope is close to 1.0
        // (edges may have artefacts from windowing)
        let mid_start = n / 4;
        let mid_end = 3 * n / 4;
        for i in mid_start..mid_end {
            assert!(
                (env[i] - 1.0).abs() < 0.15,
                "envelope[{i}] = {}, expected ~1.0",
                env[i]
            );
        }
    }

    #[test]
    fn test_envelope_of_am_signal() {
        // AM signal: (1 + 0.5*cos(2*pi*fm*t)) * sin(2*pi*fc*t)
        // Envelope should follow (1 + 0.5*cos(2*pi*fm*t))
        let n = 512;
        let fc = 50.0;
        let fm = 3.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                (1.0 + 0.5 * (2.0 * PI * fm * t).cos()) * (2.0 * PI * fc * t).sin()
            })
            .collect();

        let env = envelope_detection(&signal);
        assert_eq!(env.len(), n);

        // Check that the envelope tracks the modulation in the middle portion
        let mid_start = n / 4;
        let mid_end = 3 * n / 4;
        for i in mid_start..mid_end {
            let t = i as f64 / n as f64;
            let expected = (1.0 + 0.5 * (2.0 * PI * fm * t).cos()).abs();
            assert!(
                (env[i] - expected).abs() < 0.2,
                "envelope[{i}] = {}, expected ~{expected}",
                env[i]
            );
        }
    }

    #[test]
    fn test_envelope_empty() {
        let env = envelope_detection(&[]);
        assert!(env.is_empty());
    }

    #[test]
    fn test_envelope_single_sample() {
        let env = envelope_detection(&[0.75]);
        assert_eq!(env.len(), 1);
        assert!((env[0] - 0.75).abs() < EPSILON);
    }

    // --- NdtProcessor ---

    #[test]
    fn test_processor_creation() {
        let config = default_config();
        let processor = NdtProcessor::new(config.clone());
        assert!((processor.config.transducer_freq_mhz - 5.0).abs() < EPSILON);
        assert!((processor.config.sound_velocity_mps - 5900.0).abs() < EPSILON);
    }

    #[test]
    fn test_process_ascan_empty_signal() {
        let processor = NdtProcessor::new(default_config());
        let result = processor.process_ascan(&[], &[]);
        assert_eq!(result.gates.len(), 0);
        assert!((result.back_wall_depth_mm).abs() < EPSILON);
    }

    #[test]
    fn test_process_ascan_back_wall_detection() {
        let config = default_config();
        let processor = NdtProcessor::new(config.clone());
        let signal = generate_ascan_signal(50.0, &[], &config);

        let result = processor.process_ascan(&signal, &[]);
        // Back wall should be detected near 50 mm
        assert!(
            (result.back_wall_depth_mm - 50.0).abs() < 5.0,
            "back wall at {} mm, expected ~50 mm",
            result.back_wall_depth_mm
        );
        assert!(
            result.back_wall_amplitude > 0.5,
            "back wall amplitude = {}",
            result.back_wall_amplitude
        );
    }

    #[test]
    fn test_process_ascan_flaw_detection() {
        let config = default_config();
        let processor = NdtProcessor::new(config.clone());
        let signal = generate_ascan_signal(50.0, &[(25.0, 0.7)], &config);

        let gates = vec![Gate {
            id: 1,
            start_mm: 15.0,
            end_mm: 40.0,
            threshold: 0.2,
        }];
        let result = processor.process_ascan(&signal, &gates);

        assert_eq!(result.gates.len(), 1);
        assert!(
            result.gates[0].peak_amplitude > 0.2,
            "gate peak = {}",
            result.gates[0].peak_amplitude
        );
        // The flaw at ~25 mm should be detected
        assert!(
            (result.gates[0].peak_depth_mm - 25.0).abs() < 5.0,
            "flaw depth = {} mm, expected ~25 mm",
            result.gates[0].peak_depth_mm
        );
    }

    #[test]
    fn test_process_ascan_flaw_indication() {
        let config = default_config();
        let processor = NdtProcessor::new(config.clone());
        let signal = generate_ascan_signal(60.0, &[(30.0, 0.6)], &config);

        let gates = vec![Gate {
            id: 1,
            start_mm: 20.0,
            end_mm: 45.0,
            threshold: 0.15,
        }];
        let result = processor.process_ascan(&signal, &gates);

        // Should have at least one indication (the flaw, not the back wall)
        assert!(
            !result.indications.is_empty(),
            "expected at least one flaw indication"
        );
        let ind = &result.indications[0];
        assert!(ind.amplitude_percent > 15.0);
        assert!(ind.equivalent_size_mm > 0.0);
    }

    // --- Synthetic signal generation ---

    #[test]
    fn test_generate_ascan_signal_length() {
        let config = default_config();
        let signal = generate_ascan_signal(50.0, &[], &config);
        assert!(signal.len() >= 256, "signal too short: {}", signal.len());
    }

    #[test]
    fn test_generate_ascan_signal_has_energy() {
        let config = default_config();
        let signal = generate_ascan_signal(50.0, &[(25.0, 0.5)], &config);
        let energy: f64 = signal.iter().map(|&s| s * s).sum();
        assert!(energy > 0.0, "signal has no energy");
    }

    #[test]
    fn test_generate_ascan_multiple_flaws() {
        let config = default_config();
        let signal = generate_ascan_signal(80.0, &[(20.0, 0.3), (40.0, 0.5), (60.0, 0.7)], &config);
        let energy: f64 = signal.iter().map(|&s| s * s).sum();
        assert!(energy > 0.0);
        // Signal should be longer for deeper back wall
        let short = generate_ascan_signal(30.0, &[], &config);
        assert!(signal.len() > short.len());
    }

    // --- Edge cases ---

    #[test]
    fn test_snell_law_grazing_incidence() {
        // 90 degrees incidence
        let result = snell_law_angle(90.0, 2730.0, 5900.0);
        // sin(90)*5900/2730 = 2.16 > 1 => total internal reflection
        assert!(result.is_none());
    }

    #[test]
    fn test_snell_law_fast_to_slow() {
        // Going from fast to slow medium: should always refract (no TIR)
        let angle = snell_law_angle(45.0, 5900.0, 2730.0);
        assert!(angle.is_some());
        let a = angle.unwrap();
        assert!(a < 45.0, "refracted angle {a} should be less than incident 45 deg");
    }

    #[test]
    fn test_depth_to_time_large_depth() {
        // 500 mm in steel
        let tof = depth_to_time(500.0, 5900.0);
        let expected = 2.0 * 0.5 / 5900.0 * 1e6; // ~169.49 us
        assert!(
            (tof - expected).abs() < 0.01,
            "tof = {tof}, expected = {expected}"
        );
    }

    #[test]
    fn test_near_field_zero_diameter() {
        let nf = near_field_length(0.0, 5.0, 5900.0);
        assert!(nf.abs() < EPSILON);
    }

    #[test]
    fn test_fft_in_place_roundtrip() {
        let n = 16;
        let mut re: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).sin()).collect();
        let mut im = vec![0.0; n];
        let original = re.clone();

        fft_in_place(&mut re, &mut im, false);
        fft_in_place(&mut re, &mut im, true);

        for i in 0..n {
            assert!(
                (re[i] - original[i]).abs() < 1e-10,
                "FFT roundtrip failed at index {i}: {} != {}",
                re[i],
                original[i]
            );
        }
    }

    #[test]
    fn test_config_default() {
        let config = NdtConfig::default();
        assert!((config.transducer_freq_mhz - 5.0).abs() < EPSILON);
        assert!((config.sample_rate_hz - 100_000_000.0).abs() < EPSILON);
        assert!((config.sound_velocity_mps - 5900.0).abs() < EPSILON);
        assert!((config.beam_diameter_mm - 10.0).abs() < EPSILON);
        assert!((config.dead_zone_mm - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_flaw_type_enum() {
        // Ensure all variants exist and are distinct
        let types = [
            FlawType::Crack,
            FlawType::Porosity,
            FlawType::Inclusion,
            FlawType::LackOfFusion,
            FlawType::Delamination,
            FlawType::Unknown,
        ];
        for i in 0..types.len() {
            for j in (i + 1)..types.len() {
                assert_ne!(types[i], types[j], "variants should be distinct");
            }
        }
    }
}
