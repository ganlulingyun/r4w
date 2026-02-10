//! Ultrasound Beam Synthesizer — beamforming and focusing for medical/industrial
//! ultrasound transducer arrays.
//!
//! This module provides delay-and-sum transmit/receive beamforming, dynamic aperture
//! control, apodization windowing (Hann, Hamming, Tukey, rectangular), envelope
//! detection via Hilbert transform, log compression for B-mode display, tissue
//! harmonic imaging (THI) second-harmonic extraction, and sector/linear scan
//! conversion to Cartesian images.
//!
//! # Example
//!
//! ```
//! use r4w_core::ultrasound_beam_synthesizer::{
//!     UltrasoundConfig, UltrasoundBeamSynthesizer, ScanLine, scan_convert,
//!     compute_tx_delays, envelope_detect, log_compress,
//! };
//!
//! let config = UltrasoundConfig {
//!     num_elements: 64,
//!     element_pitch_m: 0.0003,
//!     center_freq_hz: 5.0e6,
//!     sample_rate_hz: 40.0e6,
//!     sound_speed_mps: 1540.0,
//! };
//!
//! let synth = UltrasoundBeamSynthesizer::new(config);
//!
//! // Compute transmit delays for a focus at (0, 0.03) metres
//! let positions = synth.element_positions();
//! let delays = compute_tx_delays((0.0, 0.03), &positions, 1540.0);
//! assert_eq!(delays.len(), 64);
//!
//! // Envelope detection and log compression
//! let signal = vec![0.0_f64; 128];
//! let env = envelope_detect(&signal);
//! let compressed = log_compress(&env, 60.0);
//! assert_eq!(compressed.len(), env.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & data types
// ---------------------------------------------------------------------------

/// Configuration for an ultrasound transducer array and acquisition system.
#[derive(Debug, Clone)]
pub struct UltrasoundConfig {
    /// Number of transducer elements in the array.
    pub num_elements: usize,
    /// Centre-to-centre spacing between adjacent elements (metres).
    pub element_pitch_m: f64,
    /// Transducer centre frequency (Hz).
    pub center_freq_hz: f64,
    /// Sampling rate of the digitiser (Hz).
    pub sample_rate_hz: f64,
    /// Speed of sound in the medium (m/s).  Typical soft tissue ~ 1540 m/s.
    pub sound_speed_mps: f64,
}

/// A single scan line of RF or processed echo data.
#[derive(Debug, Clone)]
pub struct ScanLine {
    /// Sample values along the line (amplitude or envelope).
    pub samples: Vec<f64>,
    /// Steering angle of this line (radians, 0 = straight ahead).
    pub angle_rad: f64,
    /// Maximum imaging depth (metres).
    pub depth_m: f64,
}

/// A B-mode image in Cartesian coordinates, stored row-major.
#[derive(Debug, Clone)]
pub struct BmodeImage {
    /// Pixel intensities, row-major, length = width * height.
    pub pixels: Vec<f64>,
    /// Image width in pixels.
    pub width: usize,
    /// Image height in pixels.
    pub height: usize,
}

// ---------------------------------------------------------------------------
// Main synthesizer struct
// ---------------------------------------------------------------------------

/// Ultrasound beam synthesizer that encapsulates array geometry and acquisition
/// parameters.
#[derive(Debug, Clone)]
pub struct UltrasoundBeamSynthesizer {
    config: UltrasoundConfig,
}

impl UltrasoundBeamSynthesizer {
    /// Create a new synthesizer from the given configuration.
    pub fn new(config: UltrasoundConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the underlying configuration.
    pub fn config(&self) -> &UltrasoundConfig {
        &self.config
    }

    /// Compute the lateral positions of each element (metres), centred around
    /// zero.
    ///
    /// Element 0 is at the most-negative x position.
    pub fn element_positions(&self) -> Vec<f64> {
        let n = self.config.num_elements;
        let pitch = self.config.element_pitch_m;
        let offset = (n as f64 - 1.0) / 2.0 * pitch;
        (0..n).map(|i| i as f64 * pitch - offset).collect()
    }
}

// ---------------------------------------------------------------------------
// Transmit focusing
// ---------------------------------------------------------------------------

/// Compute per-element transmit delays so that all wavefronts arrive at
/// `focus_point` simultaneously.
///
/// `focus_point` is `(x, z)` in metres (lateral, axial).  `element_positions`
/// are lateral positions.  Returns delays in **seconds**.
pub fn compute_tx_delays(
    focus_point: (f64, f64),
    element_positions: &[f64],
    sound_speed: f64,
) -> Vec<f64> {
    let (fx, fz) = focus_point;
    // Distance from each element to focus
    let distances: Vec<f64> = element_positions
        .iter()
        .map(|&ex| ((ex - fx).powi(2) + fz.powi(2)).sqrt())
        .collect();
    let max_dist = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    // Delay = (max_distance - element_distance) / c  so that the furthest
    // element fires first (zero delay) and closer elements are delayed.
    distances
        .iter()
        .map(|&d| (max_dist - d) / sound_speed)
        .collect()
}

// ---------------------------------------------------------------------------
// Receive focusing
// ---------------------------------------------------------------------------

/// Compute per-element receive delays for dynamic focusing at a given pixel
/// location `(x, z)` (metres).  Returns delays in **seconds**.
pub fn compute_rx_delays(
    pixel: (f64, f64),
    element_positions: &[f64],
    sound_speed: f64,
) -> Vec<f64> {
    let (px, pz) = pixel;
    let distances: Vec<f64> = element_positions
        .iter()
        .map(|&ex| ((ex - px).powi(2) + pz.powi(2)).sqrt())
        .collect();
    let min_dist = distances.iter().cloned().fold(f64::INFINITY, f64::min);
    // Align all elements to the nearest element's arrival time.
    distances
        .iter()
        .map(|&d| (d - min_dist) / sound_speed)
        .collect()
}

// ---------------------------------------------------------------------------
// Delay-and-sum beamforming
// ---------------------------------------------------------------------------

/// Delay-and-sum beamformer.
///
/// `rf_data` contains one `Vec<f64>` per element (all same length).
/// `delays` are per-element delays in **seconds**.
/// `sample_rate` is the digitiser sample rate (Hz).
///
/// Returns the beamformed line whose length equals that of the individual
/// element traces.
pub fn beamform_line(
    rf_data: &[Vec<f64>],
    delays: &[f64],
    sample_rate: f64,
) -> Vec<f64> {
    assert!(
        !rf_data.is_empty(),
        "rf_data must contain at least one element"
    );
    let num_samples = rf_data[0].len();
    let num_elements = rf_data.len();
    assert_eq!(
        delays.len(),
        num_elements,
        "delays length must match rf_data element count"
    );

    let mut output = vec![0.0_f64; num_samples];

    for (ch, delay_s) in rf_data.iter().zip(delays.iter()) {
        let delay_samples = delay_s * sample_rate;
        let delay_int = delay_samples.floor() as isize;
        let frac = delay_samples - delay_samples.floor();

        for i in 0..num_samples {
            let idx = i as isize - delay_int;
            if idx >= 0 && (idx as usize) < num_samples {
                let s0 = ch[idx as usize];
                // Linear interpolation for fractional delay
                let s1 = if (idx as usize + 1) < num_samples {
                    ch[idx as usize + 1]
                } else {
                    s0
                };
                output[i] += s0 * (1.0 - frac) + s1 * frac;
            }
        }
    }
    output
}

// ---------------------------------------------------------------------------
// Dynamic aperture
// ---------------------------------------------------------------------------

/// Compute the number of active elements for a given depth and f-number.
///
/// `f_number` = depth / aperture.  Returns at least 1 element.
pub fn dynamic_aperture(depth_m: f64, f_number: f64, element_pitch_m: f64) -> usize {
    if f_number <= 0.0 || element_pitch_m <= 0.0 {
        return 1;
    }
    let aperture = depth_m / f_number;
    let n = (aperture / element_pitch_m).round() as usize;
    n.max(1)
}

// ---------------------------------------------------------------------------
// Apodization
// ---------------------------------------------------------------------------

/// Apply an apodization window **in-place** to `weights`.
///
/// Supported window names (case-insensitive): `"hann"`, `"hamming"`,
/// `"tukey"` (alpha = 0.5), and `"rect"` (no change).
pub fn apply_apodization(weights: &mut [f64], window: &str) {
    let n = weights.len();
    if n == 0 {
        return;
    }
    let nm1 = (n as f64) - 1.0;
    match window.to_ascii_lowercase().as_str() {
        "hann" => {
            for (i, w) in weights.iter_mut().enumerate() {
                *w *= 0.5 * (1.0 - (2.0 * PI * i as f64 / nm1).cos());
            }
        }
        "hamming" => {
            for (i, w) in weights.iter_mut().enumerate() {
                *w *= 0.54 - 0.46 * (2.0 * PI * i as f64 / nm1).cos();
            }
        }
        "tukey" => {
            let alpha = 0.5_f64;
            let boundary = (alpha * nm1 / 2.0) as usize;
            for (i, w) in weights.iter_mut().enumerate() {
                let t = if i <= boundary {
                    0.5 * (1.0 - (2.0 * PI * i as f64 / (alpha * nm1)).cos())
                } else if i >= n - 1 - boundary {
                    0.5 * (1.0 - (2.0 * PI * (nm1 - i as f64) / (alpha * nm1)).cos())
                } else {
                    1.0
                };
                *w *= t;
            }
        }
        "rect" | _ => { /* unity - no change */ }
    }
}

// ---------------------------------------------------------------------------
// Envelope detection (Hilbert-based)
// ---------------------------------------------------------------------------

/// Detect the envelope of a real-valued signal using the analytic signal
/// (Hilbert transform via DFT).
///
/// Returns the instantaneous amplitude at each sample.
pub fn envelope_detect(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    // DFT
    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];
    dft_in_place(&mut re, &mut im, false);

    // Build analytic signal in frequency domain:
    //  DC and Nyquist bins unchanged; positive freqs x2; negative freqs = 0.
    if n > 1 {
        // Positive frequencies: indices 1..n/2  (multiply by 2)
        let half = n / 2;
        for i in 1..half {
            re[i] *= 2.0;
            im[i] *= 2.0;
        }
        // Negative frequencies: indices n/2+1..n-1 (set to 0)
        for i in (half + 1)..n {
            re[i] = 0.0;
            im[i] = 0.0;
        }
        // DC (index 0) and Nyquist (index n/2 if even) stay as-is.
    }

    // Inverse DFT
    dft_in_place(&mut re, &mut im, true);

    // Envelope = magnitude of analytic signal
    re.iter()
        .zip(im.iter())
        .map(|(&r, &i)| (r * r + i * i).sqrt())
        .collect()
}

// ---------------------------------------------------------------------------
// Log compression
// ---------------------------------------------------------------------------

/// Logarithmic compression for B-mode display.
///
/// Maps envelope values into the range [0, 1] with the specified dynamic
/// range in dB.  Values below the noise floor are clamped to 0.
pub fn log_compress(envelope: &[f64], dynamic_range_db: f64) -> Vec<f64> {
    if envelope.is_empty() {
        return Vec::new();
    }
    let max_val = envelope
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    if max_val <= 0.0 {
        return vec![0.0; envelope.len()];
    }
    let floor = 10.0_f64.powf(-dynamic_range_db / 20.0); // voltage floor
    envelope
        .iter()
        .map(|&v| {
            let normalised = (v / max_val).max(floor);
            let db = 20.0 * normalised.log10(); // negative dB
            // Map [-dynamic_range_db, 0] -> [0, 1]
            ((db + dynamic_range_db) / dynamic_range_db).clamp(0.0, 1.0)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tissue Harmonic Imaging (THI)
// ---------------------------------------------------------------------------

/// Extract the second-harmonic band from an RF signal for tissue-harmonic
/// imaging.
///
/// A simple frequency-domain bandpass filter centred at `2 * fundamental_hz`
/// with bandwidth equal to `fundamental_hz` is applied.
pub fn extract_harmonic(
    signal: &[f64],
    fundamental_hz: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];
    dft_in_place(&mut re, &mut im, false);

    let harmonic = 2.0 * fundamental_hz;
    let half_bw = fundamental_hz / 2.0;
    let freq_res = sample_rate / n as f64;

    for k in 0..n {
        // Map bin index to frequency (handle negative half)
        let freq = if k <= n / 2 {
            k as f64 * freq_res
        } else {
            (k as f64 - n as f64) * freq_res
        };
        let f_abs = freq.abs();
        if f_abs < (harmonic - half_bw) || f_abs > (harmonic + half_bw) {
            re[k] = 0.0;
            im[k] = 0.0;
        }
    }

    dft_in_place(&mut re, &mut im, true);
    re
}

// ---------------------------------------------------------------------------
// Scan conversion
// ---------------------------------------------------------------------------

/// Convert a set of sector-scan lines into a Cartesian [`BmodeImage`].
///
/// Each [`ScanLine`] is assumed to originate from the top-centre of the image
/// (the probe face).  The mapping uses nearest-neighbour interpolation over
/// the `(angle, range)` to `(x, z)` polar grid.
pub fn scan_convert(
    lines: &[ScanLine],
    output_width: usize,
    output_height: usize,
) -> BmodeImage {
    if lines.is_empty() || output_width == 0 || output_height == 0 {
        return BmodeImage {
            pixels: vec![0.0; output_width * output_height],
            width: output_width,
            height: output_height,
        };
    }

    let max_depth = lines
        .iter()
        .map(|l| l.depth_m)
        .fold(0.0_f64, f64::max);

    // Determine angular extent
    let min_angle = lines
        .iter()
        .map(|l| l.angle_rad)
        .fold(f64::INFINITY, f64::min);
    let max_angle = lines
        .iter()
        .map(|l| l.angle_rad)
        .fold(f64::NEG_INFINITY, f64::max);

    // Physical extent of the image
    let half_width_m = max_depth * ((max_angle.abs()).max(min_angle.abs())).sin().max(0.01);
    let x_extent = 2.0 * half_width_m;
    let z_extent = max_depth;

    let mut pixels = vec![0.0_f64; output_width * output_height];

    for row in 0..output_height {
        let z = (row as f64 + 0.5) / output_height as f64 * z_extent;
        for col in 0..output_width {
            let x = (col as f64 + 0.5) / output_width as f64 * x_extent - half_width_m;
            let range = (x * x + z * z).sqrt();
            let angle = x.atan2(z); // angle from z-axis

            // Find nearest scan line
            let mut best_line = 0;
            let mut best_diff = f64::INFINITY;
            for (li, line) in lines.iter().enumerate() {
                let diff = (line.angle_rad - angle).abs();
                if diff < best_diff {
                    best_diff = diff;
                    best_line = li;
                }
            }

            let line = &lines[best_line];
            if range <= line.depth_m && !line.samples.is_empty() {
                let sample_idx =
                    (range / line.depth_m * (line.samples.len() as f64 - 1.0)).round() as usize;
                let sample_idx = sample_idx.min(line.samples.len() - 1);
                pixels[row * output_width + col] = line.samples[sample_idx];
            }
        }
    }

    BmodeImage {
        pixels,
        width: output_width,
        height: output_height,
    }
}

// ---------------------------------------------------------------------------
// Internal: Naive DFT / IDFT  (std-only, no external FFT crate)
// ---------------------------------------------------------------------------

/// In-place DFT (forward) or IDFT (inverse) using the naive O(N^2) algorithm.
fn dft_in_place(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert_eq!(n, im.len());
    if n <= 1 {
        return;
    }

    let sign = if inverse { 1.0 } else { -1.0 };
    let mut out_re = vec![0.0_f64; n];
    let mut out_im = vec![0.0_f64; n];

    for k in 0..n {
        let mut sr = 0.0_f64;
        let mut si = 0.0_f64;
        for t in 0..n {
            let angle = sign * 2.0 * PI * (k as f64) * (t as f64) / (n as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            sr += re[t] * cos_a - im[t] * sin_a;
            si += re[t] * sin_a + im[t] * cos_a;
        }
        out_re[k] = sr;
        out_im[k] = si;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for k in 0..n {
            re[k] = out_re[k] * inv_n;
            im[k] = out_im[k] * inv_n;
        }
    } else {
        re.copy_from_slice(&out_re);
        im.copy_from_slice(&out_im);
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    // Helper: simple config for tests
    fn test_config() -> UltrasoundConfig {
        UltrasoundConfig {
            num_elements: 16,
            element_pitch_m: 0.0003,
            center_freq_hz: 5.0e6,
            sample_rate_hz: 40.0e6,
            sound_speed_mps: 1540.0,
        }
    }

    // --- 1. UltrasoundBeamSynthesizer::new and element_positions ----------

    #[test]
    fn test_element_positions_count() {
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        assert_eq!(pos.len(), 16);
    }

    #[test]
    fn test_element_positions_symmetry() {
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        // Positions should be symmetric about zero
        let first = pos[0];
        let last = pos[pos.len() - 1];
        assert!((first + last).abs() < EPS, "first + last = {}", first + last);
    }

    #[test]
    fn test_element_positions_spacing() {
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        for i in 1..pos.len() {
            let diff = pos[i] - pos[i - 1];
            assert!(
                (diff - 0.0003).abs() < EPS,
                "spacing at {} = {}",
                i,
                diff
            );
        }
    }

    // --- 2. compute_tx_delays --------------------------------------------

    #[test]
    fn test_tx_delays_on_axis() {
        // Focus directly ahead of centre - delays should be symmetric
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        let delays = compute_tx_delays((0.0, 0.03), &pos, 1540.0);
        assert_eq!(delays.len(), 16);
        // Outermost elements should have smallest delay (fire first)
        let mid = delays.len() / 2;
        assert!(delays[0] < delays[mid], "outer should fire before centre");
    }

    #[test]
    fn test_tx_delays_all_non_negative() {
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        let delays = compute_tx_delays((0.005, 0.02), &pos, 1540.0);
        for &d in &delays {
            assert!(d >= -EPS, "negative delay: {}", d);
        }
    }

    #[test]
    fn test_tx_delays_symmetry_for_on_axis_focus() {
        let synth = UltrasoundBeamSynthesizer::new(test_config());
        let pos = synth.element_positions();
        let delays = compute_tx_delays((0.0, 0.05), &pos, 1540.0);
        let n = delays.len();
        for i in 0..n / 2 {
            assert!(
                (delays[i] - delays[n - 1 - i]).abs() < EPS,
                "asymmetry at {}: {} vs {}",
                i,
                delays[i],
                delays[n - 1 - i]
            );
        }
    }

    // --- 3. compute_rx_delays --------------------------------------------

    #[test]
    fn test_rx_delays_length() {
        let pos = vec![-0.001, 0.0, 0.001];
        let d = compute_rx_delays((0.0, 0.02), &pos, 1540.0);
        assert_eq!(d.len(), 3);
    }

    #[test]
    fn test_rx_delays_non_negative() {
        let pos = vec![-0.002, -0.001, 0.0, 0.001, 0.002];
        let d = compute_rx_delays((0.001, 0.02), &pos, 1540.0);
        for &v in &d {
            assert!(v >= -EPS, "negative rx delay {}", v);
        }
    }

    // --- 4. beamform_line ------------------------------------------------

    #[test]
    fn test_beamform_line_basic() {
        // Two channels, zero delay - output = sum
        let rf = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
        let delays = vec![0.0, 0.0];
        let out = beamform_line(&rf, &delays, 1.0);
        assert_eq!(out.len(), 3);
        assert!((out[0] - 5.0).abs() < EPS);
        assert!((out[1] - 7.0).abs() < EPS);
        assert!((out[2] - 9.0).abs() < EPS);
    }

    #[test]
    fn test_beamform_line_with_delay() {
        // Single channel with 1-sample delay: output[1] should come from input[0]
        let rf = vec![vec![10.0, 20.0, 30.0, 40.0]];
        let delays = vec![1.0]; // 1 second at sample_rate=1 => 1 sample delay
        let out = beamform_line(&rf, &delays, 1.0);
        // output[1] <- input[0]
        assert!((out[1] - 10.0).abs() < EPS);
        assert!((out[2] - 20.0).abs() < EPS);
    }

    // --- 5. dynamic_aperture ---------------------------------------------

    #[test]
    fn test_dynamic_aperture_basic() {
        // depth=0.03, f_number=2.0 => aperture=0.015, pitch=0.0003 => 50 elements
        let n = dynamic_aperture(0.03, 2.0, 0.0003);
        assert_eq!(n, 50);
    }

    #[test]
    fn test_dynamic_aperture_minimum() {
        // Very shallow depth should still return at least 1
        let n = dynamic_aperture(0.0001, 10.0, 0.001);
        assert!(n >= 1);
    }

    #[test]
    fn test_dynamic_aperture_zero_f_number() {
        let n = dynamic_aperture(0.03, 0.0, 0.0003);
        assert_eq!(n, 1);
    }

    // --- 6. apply_apodization --------------------------------------------

    #[test]
    fn test_apodization_rect() {
        let mut w = vec![1.0; 8];
        apply_apodization(&mut w, "rect");
        for &v in &w {
            assert!((v - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn test_apodization_hann_endpoints() {
        let mut w = vec![1.0; 64];
        apply_apodization(&mut w, "hann");
        // Hann window is zero at endpoints
        assert!(w[0].abs() < EPS, "hann w[0] = {}", w[0]);
        assert!(w[63].abs() < EPS, "hann w[63] = {}", w[63]);
    }

    #[test]
    fn test_apodization_hamming_endpoints() {
        let mut w = vec![1.0; 64];
        apply_apodization(&mut w, "hamming");
        // Hamming minimum is ~0.08
        assert!(w[0] > 0.07 && w[0] < 0.09, "hamming w[0] = {}", w[0]);
    }

    #[test]
    fn test_apodization_tukey_centre() {
        let mut w = vec![1.0; 64];
        apply_apodization(&mut w, "tukey");
        // Centre of Tukey window should be 1.0
        assert!((w[32] - 1.0).abs() < EPS, "tukey w[32] = {}", w[32]);
    }

    // --- 7. envelope_detect ----------------------------------------------

    #[test]
    fn test_envelope_detect_dc() {
        // DC signal -> constant envelope
        let sig = vec![3.0; 32];
        let env = envelope_detect(&sig);
        assert_eq!(env.len(), 32);
        for &v in &env {
            assert!((v - 3.0).abs() < 0.1, "env val = {}", v);
        }
    }

    #[test]
    fn test_envelope_detect_sine() {
        // Pure sine -> envelope approx amplitude
        let n = 64;
        let amplitude = 5.0;
        let sig: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * 4.0 * i as f64 / n as f64).sin())
            .collect();
        let env = envelope_detect(&sig);
        // Check a few samples away from edges
        for i in 8..56 {
            assert!(
                (env[i] - amplitude).abs() < 0.5,
                "env[{}] = {} (expected ~{})",
                i,
                env[i],
                amplitude
            );
        }
    }

    #[test]
    fn test_envelope_detect_empty() {
        let env = envelope_detect(&[]);
        assert!(env.is_empty());
    }

    // --- 8. log_compress -------------------------------------------------

    #[test]
    fn test_log_compress_peak() {
        let env = vec![0.0, 0.5, 1.0];
        let lc = log_compress(&env, 60.0);
        // Peak value should be 1.0
        assert!((lc[2] - 1.0).abs() < EPS);
    }

    #[test]
    fn test_log_compress_range() {
        let env: Vec<f64> = (0..100).map(|i| i as f64 / 99.0).collect();
        let lc = log_compress(&env, 40.0);
        for &v in &lc {
            assert!(v >= 0.0 && v <= 1.0, "out of range: {}", v);
        }
    }

    #[test]
    fn test_log_compress_empty() {
        let lc = log_compress(&[], 60.0);
        assert!(lc.is_empty());
    }

    // --- 9. extract_harmonic ---------------------------------------------

    #[test]
    fn test_extract_harmonic_suppresses_fundamental() {
        // Signal with fundamental at 5 MHz and harmonic at 10 MHz
        let n = 256;
        let fs = 40.0e6;
        let f0 = 5.0e6;
        let sig: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * f0 * t).sin() + 0.3 * (2.0 * PI * 2.0 * f0 * t).sin()
            })
            .collect();
        let harm = extract_harmonic(&sig, f0, fs);
        assert_eq!(harm.len(), n);

        // Energy at fundamental should be much less than in original
        let orig_fund_energy: f64 = sig.iter().map(|s| s * s).sum::<f64>();
        let harm_energy: f64 = harm.iter().map(|s| s * s).sum::<f64>();
        assert!(
            harm_energy < orig_fund_energy * 0.5,
            "harmonic energy {} should be less than half original {}",
            harm_energy,
            orig_fund_energy
        );
    }

    #[test]
    fn test_extract_harmonic_empty() {
        let h = extract_harmonic(&[], 5.0e6, 40.0e6);
        assert!(h.is_empty());
    }

    // --- 10. scan_convert ------------------------------------------------

    #[test]
    fn test_scan_convert_dimensions() {
        let lines = vec![
            ScanLine {
                samples: vec![0.5; 100],
                angle_rad: -0.2,
                depth_m: 0.05,
            },
            ScanLine {
                samples: vec![0.8; 100],
                angle_rad: 0.0,
                depth_m: 0.05,
            },
            ScanLine {
                samples: vec![0.5; 100],
                angle_rad: 0.2,
                depth_m: 0.05,
            },
        ];
        let img = scan_convert(&lines, 80, 60);
        assert_eq!(img.width, 80);
        assert_eq!(img.height, 60);
        assert_eq!(img.pixels.len(), 80 * 60);
    }

    #[test]
    fn test_scan_convert_empty_lines() {
        let img = scan_convert(&[], 10, 10);
        assert_eq!(img.pixels.len(), 100);
        assert!(img.pixels.iter().all(|&v| v == 0.0));
    }

    // --- 11. Internal DFT round-trip -------------------------------------

    #[test]
    fn test_dft_roundtrip() {
        let original = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut re = original.clone();
        let mut im = vec![0.0; 8];
        dft_in_place(&mut re, &mut im, false);
        dft_in_place(&mut re, &mut im, true);
        for (i, (&r, &o)) in re.iter().zip(original.iter()).enumerate() {
            assert!(
                (r - o).abs() < 1e-6,
                "DFT roundtrip mismatch at {}: {} vs {}",
                i,
                r,
                o
            );
        }
        for (i, &v) in im.iter().enumerate() {
            assert!(v.abs() < 1e-6, "imaginary residual at {}: {}", i, v);
        }
    }

    // --- 12. Config accessor ---------------------------------------------

    #[test]
    fn test_config_accessor() {
        let cfg = test_config();
        let synth = UltrasoundBeamSynthesizer::new(cfg.clone());
        assert_eq!(synth.config().num_elements, 16);
        assert!((synth.config().center_freq_hz - 5.0e6).abs() < 1.0);
    }
}
