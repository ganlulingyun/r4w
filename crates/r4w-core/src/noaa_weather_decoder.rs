//! NOAA APT (Automatic Picture Transmission) Decoder
//!
//! Decodes APT signals from NOAA polar-orbiting weather satellites
//! (NOAA-15, NOAA-18, NOAA-19). APT transmits two simultaneous
//! channels (visible + infrared) as AM-modulated audio at 2400 Hz
//! subcarrier on 137 MHz FM downlinks. Each image line is 2080 pixels
//! wide (two 1040-pixel channels) transmitted at 2 lines/second.
//!
//! The APT line format (2080 pixels total):
//! - Channel A (1040 px): Sync A (39) + Space A / Visible (909) + Telemetry A (92)
//! - Channel B (1040 px): Sync B (39) + Space B / Infrared (909) + Telemetry B (92)
//!
//! Reference: NOAA KLM User's Guide, Section 4.2 (APT format).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noaa_weather_decoder::{AptDecoder, AptConfig, ImageChannel};
//!
//! let config = AptConfig::default();
//! assert_eq!(config.line_length, 2080);
//!
//! let decoder = AptDecoder::new(config);
//! // Generate a synthetic APT-like AM signal (carrier at 2400 Hz)
//! let sample_rate = decoder.config().sample_rate;
//! let samples_per_line = decoder.config().samples_per_line();
//! let num_samples = samples_per_line;
//!
//! // Create a simple AM test signal with known envelope
//! let signal: Vec<f64> = (0..num_samples)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let carrier = (2.0 * std::f64::consts::PI * 2400.0 * t).sin();
//!         let envelope = 0.5; // constant brightness
//!         envelope * carrier
//!     })
//!     .collect();
//!
//! let demodulated = decoder.am_demodulate(&signal);
//! assert!(!demodulated.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// APT line length in pixels (both channels combined).
pub const APT_LINE_PIXELS: usize = 2080;

/// Pixels per channel (A or B).
pub const APT_CHANNEL_PIXELS: usize = 1040;

/// Samples per line at the standard APT rate (4160 samples/line, 2 samples/pixel).
pub const APT_SAMPLES_PER_LINE: usize = APT_LINE_PIXELS * 2;

/// APT line rate: 2 lines per second.
pub const APT_LINE_RATE: f64 = 2.0;

/// Standard APT AM subcarrier frequency in Hz.
pub const APT_CARRIER_HZ: f64 = 2400.0;

/// Sync A word length in pixels.
pub const SYNC_A_PIXELS: usize = 39;

/// Space A (visible channel) length in pixels.
pub const SPACE_A_PIXELS: usize = 909;

/// Telemetry A length in pixels (92 px per half-line).
pub const TELEMETRY_A_PIXELS: usize = 92;

/// Sync B word length in pixels.
pub const SYNC_B_PIXELS: usize = 39;

/// Space B (infrared channel) length in pixels.
pub const SPACE_B_PIXELS: usize = 909;

/// Telemetry B length in pixels (92 px per half-line).
pub const TELEMETRY_B_PIXELS: usize = 92;

/// Number of sync pulses in the Sync A pattern.
pub const SYNC_A_PULSES: usize = 7;

/// Sync A pattern frequency in Hz (7 pulses in 39 pixels at 4160 samples/sec).
pub const SYNC_A_FREQ_HZ: f64 = 832.0;

/// Number of telemetry calibration wedges per frame (16 wedges).
pub const TELEMETRY_WEDGES: usize = 16;

/// Lines per telemetry frame (one wedge set = 128 lines).
pub const TELEMETRY_FRAME_LINES: usize = 128;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Image channel selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageChannel {
    /// Channel A -- typically visible light.
    VisibleA,
    /// Channel B -- typically infrared.
    InfraredB,
}

/// Configuration for the APT decoder.
#[derive(Debug, Clone)]
pub struct AptConfig {
    /// Input sample rate in Hz (typically 11025 or 20800).
    pub sample_rate: f64,
    /// Correlation threshold for sync word detection (0.0-1.0).
    pub sync_threshold: f64,
    /// Line length in pixels (default 2080).
    pub line_length: usize,
}

impl Default for AptConfig {
    fn default() -> Self {
        Self {
            sample_rate: 20800.0,
            sync_threshold: 0.65,
            line_length: APT_LINE_PIXELS,
        }
    }
}

impl AptConfig {
    /// Number of input samples per APT line at the configured sample rate.
    pub fn samples_per_line(&self) -> usize {
        (self.sample_rate / APT_LINE_RATE) as usize
    }
}

/// A single decoded APT line (2080 pixels).
#[derive(Debug, Clone)]
pub struct AptLine {
    /// Sync A marker pixels (39 px).
    pub sync_a: Vec<u8>,
    /// Space A / visible channel image data (909 px).
    pub space_a: Vec<u8>,
    /// Telemetry A strip (92 px).
    pub telemetry_a: Vec<u8>,
    /// Sync B marker pixels (39 px).
    pub sync_b: Vec<u8>,
    /// Space B / infrared channel image data (909 px).
    pub space_b: Vec<u8>,
    /// Telemetry B strip (92 px).
    pub telemetry_b: Vec<u8>,
}

impl AptLine {
    /// Total pixel count for this line.
    pub fn total_pixels(&self) -> usize {
        self.sync_a.len()
            + self.space_a.len()
            + self.telemetry_a.len()
            + self.sync_b.len()
            + self.space_b.len()
            + self.telemetry_b.len()
    }

    /// Return all 2080 pixels as a flat row.
    pub fn as_pixels(&self) -> Vec<u8> {
        let mut row = Vec::with_capacity(APT_LINE_PIXELS);
        row.extend_from_slice(&self.sync_a);
        row.extend_from_slice(&self.space_a);
        row.extend_from_slice(&self.telemetry_a);
        row.extend_from_slice(&self.sync_b);
        row.extend_from_slice(&self.space_b);
        row.extend_from_slice(&self.telemetry_b);
        row
    }
}

/// Telemetry wedge data extracted from calibration strips.
#[derive(Debug, Clone)]
pub struct TelemetryData {
    /// Wedge values (16 wedges, each an average pixel brightness 0-255).
    pub wedges: Vec<u8>,
    /// Channel identifier (A or B).
    pub channel: ImageChannel,
}

/// Decoded APT image composed of multiple scan lines.
#[derive(Debug, Clone)]
pub struct AptImage {
    /// Width in pixels.
    pub width: usize,
    /// Height in pixels (number of lines).
    pub height: usize,
    /// Row-major pixel data (grayscale 0-255).
    pub pixels: Vec<u8>,
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// NOAA APT signal decoder.
///
/// Accepts AM-demodulated (or raw audio) samples and produces APT
/// image lines. The typical workflow is:
///
/// 1. AM-demodulate the FM-detected 137 MHz signal.
/// 2. Resample to 4160 samples/line (if needed).
/// 3. Detect sync words to find line boundaries.
/// 4. Decode lines and assemble an image.
#[derive(Debug, Clone)]
pub struct AptDecoder {
    config: AptConfig,
}

impl AptDecoder {
    /// Create a new decoder with the given configuration.
    pub fn new(config: AptConfig) -> Self {
        Self { config }
    }

    /// Create a decoder with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(AptConfig::default())
    }

    /// Return the current configuration.
    pub fn config(&self) -> &AptConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // AM demodulation
    // -----------------------------------------------------------------------

    /// AM envelope detection via moving-window RMS.
    ///
    /// Computes the envelope of the input signal using a short-window
    /// RMS estimator scaled to approximate the peak amplitude.
    /// The output length equals the input length.
    pub fn am_demodulate(&self, samples: &[f64]) -> Vec<f64> {
        if samples.is_empty() {
            return Vec::new();
        }
        am_demodulate(samples)
    }

    // -----------------------------------------------------------------------
    // Resampling
    // -----------------------------------------------------------------------

    /// Resample the demodulated signal so that one APT line spans
    /// exactly [`APT_SAMPLES_PER_LINE`] (4160) samples.
    ///
    /// Uses linear interpolation for simplicity.
    pub fn resample_to_apt_rate(&self, samples: &[f64]) -> Vec<f64> {
        let input_samples_per_line = self.config.samples_per_line();
        if input_samples_per_line == APT_SAMPLES_PER_LINE {
            return samples.to_vec();
        }
        let ratio = APT_SAMPLES_PER_LINE as f64 / input_samples_per_line as f64;
        let num_lines_f = samples.len() as f64 / input_samples_per_line as f64;
        let out_len = (num_lines_f * APT_SAMPLES_PER_LINE as f64) as usize;
        let mut out = Vec::with_capacity(out_len);
        for i in 0..out_len {
            let src_idx = i as f64 / ratio;
            let idx0 = src_idx.floor() as usize;
            let frac = src_idx - idx0 as f64;
            let idx1 = (idx0 + 1).min(samples.len().saturating_sub(1));
            out.push(samples[idx0] * (1.0 - frac) + samples[idx1] * frac);
        }
        out
    }

    // -----------------------------------------------------------------------
    // Sync word detection
    // -----------------------------------------------------------------------

    /// Detect APT Sync-A patterns in a resampled (4160 samp/line) signal.
    ///
    /// The Sync-A word is a 7-pulse AM tone at ~832 Hz occupying the
    /// first 39 pixels (78 samples at 2 samples/pixel). Returns sample
    /// offsets where sync correlation exceeds `sync_threshold`.
    pub fn detect_sync_word(&self, samples: &[f64]) -> Vec<usize> {
        detect_sync_word(samples, self.config.sync_threshold)
    }

    // -----------------------------------------------------------------------
    // Line decoding
    // -----------------------------------------------------------------------

    /// Decode a single APT line from 4160 resampled samples.
    ///
    /// Each pixel is the average of 2 consecutive samples, quantised to
    /// 0-255. Returns `None` if the slice is too short.
    pub fn decode_line(&self, samples: &[f64]) -> Option<AptLine> {
        decode_line(samples)
    }

    // -----------------------------------------------------------------------
    // Image assembly
    // -----------------------------------------------------------------------

    /// Assemble a grayscale image from decoded APT lines.
    ///
    /// `channel` selects either the visible (A) or infrared (B) channel.
    pub fn assemble_image(&self, lines: &[AptLine], channel: ImageChannel) -> AptImage {
        assemble_image(lines, channel)
    }

    // -----------------------------------------------------------------------
    // Telemetry extraction
    // -----------------------------------------------------------------------

    /// Extract telemetry calibration wedges from a decoded line.
    ///
    /// The telemetry strip is 92 pixels. The first 80 pixels form
    /// 16 wedges of 5 pixels each; the remaining 12 pixels are padding.
    pub fn extract_telemetry(&self, line: &AptLine, channel: ImageChannel) -> TelemetryData {
        extract_telemetry(line, channel)
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// AM envelope detection (free function).
///
/// Uses a simple moving-window RMS approach as an envelope detector,
/// with a window size tuned for the 2400 Hz APT subcarrier.
pub fn am_demodulate(samples: &[f64]) -> Vec<f64> {
    if samples.is_empty() {
        return Vec::new();
    }
    // Use half-cycle of 2400 Hz carrier as window: at typical 20800 Hz
    // sample rate that is ~4 samples. We use a minimum window of 3.
    let window = 4usize.max(3);
    let half_w = window / 2;
    let len = samples.len();
    let mut envelope = vec![0.0f64; len];

    for i in 0..len {
        let start = if i >= half_w { i - half_w } else { 0 };
        let end = (i + half_w + 1).min(len);
        let mut sum_sq = 0.0;
        for j in start..end {
            sum_sq += samples[j] * samples[j];
        }
        // RMS envelope x sqrt(2) to approximate peak
        envelope[i] = (sum_sq / (end - start) as f64).sqrt() * std::f64::consts::SQRT_2;
    }
    envelope
}

/// Generate the reference Sync-A waveform (78 samples at 2 samp/px, 39 px).
///
/// Seven cycles of an ~832 Hz tone normalised to +/-1.
fn generate_sync_a_reference() -> Vec<f64> {
    let num_samples = SYNC_A_PIXELS * 2; // 78 samples
    let freq = SYNC_A_FREQ_HZ;
    let sample_rate = APT_SAMPLES_PER_LINE as f64 * APT_LINE_RATE; // 8320 Hz effective
    (0..num_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            (2.0 * PI * freq * t).sin()
        })
        .collect()
}

/// Detect Sync-A offsets in resampled samples.
pub fn detect_sync_word(samples: &[f64], threshold: f64) -> Vec<usize> {
    let reference = generate_sync_a_reference();
    let ref_len = reference.len();
    if samples.len() < ref_len {
        return Vec::new();
    }

    // Normalised cross-correlation
    let ref_energy: f64 = reference.iter().map(|x| x * x).sum::<f64>().sqrt();
    if ref_energy < 1e-12 {
        return Vec::new();
    }

    let mut peaks = Vec::new();
    let search_len = samples.len() - ref_len + 1;

    for i in 0..search_len {
        let mut cross = 0.0f64;
        let mut sig_energy = 0.0f64;
        for j in 0..ref_len {
            cross += samples[i + j] * reference[j];
            sig_energy += samples[i + j] * samples[i + j];
        }
        let sig_e = sig_energy.sqrt();
        let correlation = if sig_e > 1e-12 {
            cross / (sig_e * ref_energy)
        } else {
            0.0
        };
        if correlation >= threshold {
            peaks.push(i);
        }
    }

    // Cluster nearby detections -- keep only the best per line region
    coalesce_peaks(&peaks, APT_SAMPLES_PER_LINE / 2)
}

/// Remove duplicate detections that are within `min_distance` of each other,
/// keeping the first in each cluster.
fn coalesce_peaks(peaks: &[usize], min_distance: usize) -> Vec<usize> {
    if peaks.is_empty() {
        return Vec::new();
    }
    let mut result = vec![peaks[0]];
    for &p in &peaks[1..] {
        if p - *result.last().unwrap() >= min_distance {
            result.push(p);
        }
    }
    result
}

/// Decode one APT line from exactly `APT_SAMPLES_PER_LINE` samples.
pub fn decode_line(samples: &[f64]) -> Option<AptLine> {
    if samples.len() < APT_SAMPLES_PER_LINE {
        return None;
    }

    let pixels = samples_to_pixels(&samples[..APT_SAMPLES_PER_LINE]);

    let mut offset = 0;
    let sync_a = pixels[offset..offset + SYNC_A_PIXELS].to_vec();
    offset += SYNC_A_PIXELS;
    let space_a = pixels[offset..offset + SPACE_A_PIXELS].to_vec();
    offset += SPACE_A_PIXELS;
    let telemetry_a = pixels[offset..offset + TELEMETRY_A_PIXELS].to_vec();
    offset += TELEMETRY_A_PIXELS;
    let sync_b = pixels[offset..offset + SYNC_B_PIXELS].to_vec();
    offset += SYNC_B_PIXELS;
    let space_b = pixels[offset..offset + SPACE_B_PIXELS].to_vec();
    offset += SPACE_B_PIXELS;
    let telemetry_b = pixels[offset..offset + TELEMETRY_B_PIXELS].to_vec();

    Some(AptLine {
        sync_a,
        space_a,
        telemetry_a,
        sync_b,
        space_b,
        telemetry_b,
    })
}

/// Convert resampled samples (2 per pixel) to 0-255 pixel values.
fn samples_to_pixels(samples: &[f64]) -> Vec<u8> {
    samples
        .chunks(2)
        .map(|pair| {
            let avg = if pair.len() == 2 {
                (pair[0] + pair[1]) / 2.0
            } else {
                pair[0]
            };
            // Clamp to 0-255
            (avg * 255.0).round().clamp(0.0, 255.0) as u8
        })
        .collect()
}

/// Assemble a single-channel image from decoded lines.
pub fn assemble_image(lines: &[AptLine], channel: ImageChannel) -> AptImage {
    let width = match channel {
        ImageChannel::VisibleA => SPACE_A_PIXELS,
        ImageChannel::InfraredB => SPACE_B_PIXELS,
    };
    let height = lines.len();
    let mut pixels = Vec::with_capacity(width * height);
    for line in lines {
        match channel {
            ImageChannel::VisibleA => pixels.extend_from_slice(&line.space_a),
            ImageChannel::InfraredB => pixels.extend_from_slice(&line.space_b),
        }
    }
    AptImage {
        width,
        height,
        pixels,
    }
}

/// Extract telemetry calibration wedge values from a line.
pub fn extract_telemetry(line: &AptLine, channel: ImageChannel) -> TelemetryData {
    let strip = match channel {
        ImageChannel::VisibleA => &line.telemetry_a,
        ImageChannel::InfraredB => &line.telemetry_b,
    };

    // 92 pixels: first 80 form 16 wedges of 5 px each; remaining 12 are padding
    let pixels_per_wedge = 5;
    let mut wedges = Vec::with_capacity(TELEMETRY_WEDGES);
    for w in 0..TELEMETRY_WEDGES {
        let start = w * pixels_per_wedge;
        let end = (start + pixels_per_wedge).min(strip.len());
        if start >= strip.len() {
            wedges.push(0);
            continue;
        }
        let sum: u32 = strip[start..end].iter().map(|&p| p as u32).sum();
        let count = (end - start) as u32;
        wedges.push(if count > 0 { (sum / count) as u8 } else { 0 });
    }

    TelemetryData { wedges, channel }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let cfg = AptConfig::default();
        assert_eq!(cfg.line_length, 2080);
        assert_eq!(cfg.sample_rate, 20800.0);
        assert!((cfg.sync_threshold - 0.65).abs() < 1e-6);
    }

    #[test]
    fn test_samples_per_line() {
        let cfg = AptConfig::default();
        // 20800 Hz / 2 lines per second = 10400 samples per line
        assert_eq!(cfg.samples_per_line(), 10400);

        let cfg2 = AptConfig {
            sample_rate: 11025.0,
            ..AptConfig::default()
        };
        assert_eq!(cfg2.samples_per_line(), 5512);
    }

    #[test]
    fn test_apt_line_pixel_counts() {
        // Verify the APT format adds up to 2080 pixels
        let total = SYNC_A_PIXELS + SPACE_A_PIXELS + TELEMETRY_A_PIXELS
            + SYNC_B_PIXELS + SPACE_B_PIXELS + TELEMETRY_B_PIXELS;
        assert_eq!(total, APT_LINE_PIXELS);
        assert_eq!(total, 2080);
    }

    #[test]
    fn test_am_demodulate_empty() {
        let result = am_demodulate(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_am_demodulate_dc_signal() {
        // Constant signal should produce constant envelope
        let signal = vec![0.5; 100];
        let env = am_demodulate(&signal);
        assert_eq!(env.len(), 100);
        // RMS of constant 0.5 x sqrt(2) ~ 0.707
        for &v in &env[2..98] {
            assert!(
                (v - 0.707).abs() < 0.1,
                "Expected ~0.707, got {}",
                v
            );
        }
    }

    #[test]
    fn test_am_demodulate_sine() {
        // AM-modulated carrier: envelope should track the modulation
        let n = 1000;
        let fs = 20800.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let carrier = (2.0 * PI * 2400.0 * t).sin();
                // Constant envelope of 1.0
                carrier
            })
            .collect();
        let env = am_demodulate(&signal);
        assert_eq!(env.len(), n);
        // The average envelope should be roughly near 1.0
        let mean: f64 = env.iter().sum::<f64>() / env.len() as f64;
        assert!(
            mean > 0.5 && mean < 1.5,
            "Mean envelope {} out of range",
            mean
        );
    }

    #[test]
    fn test_samples_to_pixels() {
        let samples = vec![0.0, 0.0, 1.0, 1.0, 0.5, 0.5];
        let pixels = samples_to_pixels(&samples);
        assert_eq!(pixels.len(), 3);
        assert_eq!(pixels[0], 0);
        assert_eq!(pixels[1], 255);
        assert_eq!(pixels[2], 128);
    }

    #[test]
    fn test_decode_line_too_short() {
        let samples = vec![0.5; 100];
        assert!(decode_line(&samples).is_none());
    }

    #[test]
    fn test_decode_line_correct_lengths() {
        // Create 4160 samples with constant value 0.5 -> pixel value 128
        let samples = vec![0.5; APT_SAMPLES_PER_LINE];
        let line = decode_line(&samples).unwrap();

        assert_eq!(line.sync_a.len(), SYNC_A_PIXELS);
        assert_eq!(line.space_a.len(), SPACE_A_PIXELS);
        assert_eq!(line.telemetry_a.len(), TELEMETRY_A_PIXELS);
        assert_eq!(line.sync_b.len(), SYNC_B_PIXELS);
        assert_eq!(line.space_b.len(), SPACE_B_PIXELS);
        assert_eq!(line.telemetry_b.len(), TELEMETRY_B_PIXELS);
        assert_eq!(line.total_pixels(), APT_LINE_PIXELS);

        // All pixels should be 128 (0.5 * 255 = 127.5 -> round to 128)
        for &px in &line.space_a {
            assert_eq!(px, 128);
        }
    }

    #[test]
    fn test_assemble_image_visible() {
        let samples_a = vec![0.75; APT_SAMPLES_PER_LINE];
        let samples_b = vec![0.25; APT_SAMPLES_PER_LINE];
        let line_a = decode_line(&samples_a).unwrap();
        let line_b = decode_line(&samples_b).unwrap();
        let lines = vec![line_a, line_b];

        let img = assemble_image(&lines, ImageChannel::VisibleA);
        assert_eq!(img.width, SPACE_A_PIXELS);
        assert_eq!(img.height, 2);
        assert_eq!(img.pixels.len(), SPACE_A_PIXELS * 2);
        // First row: 0.75 * 255 = 191.25 -> 191
        assert_eq!(img.pixels[0], 191);
        // Second row: 0.25 * 255 = 63.75 -> 64
        assert_eq!(img.pixels[SPACE_A_PIXELS], 64);
    }

    #[test]
    fn test_assemble_image_infrared() {
        let samples = vec![0.5; APT_SAMPLES_PER_LINE];
        let line = decode_line(&samples).unwrap();
        let img = assemble_image(&[line], ImageChannel::InfraredB);
        assert_eq!(img.width, SPACE_B_PIXELS);
        assert_eq!(img.height, 1);
    }

    #[test]
    fn test_extract_telemetry() {
        // Create a line with a gradient in the telemetry strips
        let mut samples = vec![0.0; APT_SAMPLES_PER_LINE];
        // Fill telemetry A region with increasing values
        // Telemetry A starts at pixel (39 + 909) = 948, sample offset = 948 * 2 = 1896
        let telem_a_start = (SYNC_A_PIXELS + SPACE_A_PIXELS) * 2;
        for i in 0..(TELEMETRY_A_PIXELS * 2) {
            let px_idx = i / 2;
            let wedge_idx = px_idx / 5; // 5 pixels per wedge
            // Scale so wedge 0 = 0.0, wedge 15 = 1.0
            let val = (wedge_idx as f64) / 15.0;
            samples[telem_a_start + i] = val;
        }

        let line = decode_line(&samples).unwrap();
        let telem = extract_telemetry(&line, ImageChannel::VisibleA);
        assert_eq!(telem.wedges.len(), TELEMETRY_WEDGES);
        assert_eq!(telem.channel, ImageChannel::VisibleA);

        // First wedge should be ~0, last should be ~255
        assert!(telem.wedges[0] < 10, "First wedge should be near 0, got {}", telem.wedges[0]);
        assert!(telem.wedges[15] > 240, "Last wedge should be near 255, got {}", telem.wedges[15]);
    }

    #[test]
    fn test_resample_identity() {
        // When sample rate already gives APT_SAMPLES_PER_LINE, no resampling needed
        let cfg = AptConfig {
            sample_rate: APT_SAMPLES_PER_LINE as f64 * APT_LINE_RATE,
            ..AptConfig::default()
        };
        assert_eq!(cfg.samples_per_line(), APT_SAMPLES_PER_LINE);
        let decoder = AptDecoder::new(cfg);
        let input = vec![1.0; APT_SAMPLES_PER_LINE * 2];
        let output = decoder.resample_to_apt_rate(&input);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_resample_downconvert() {
        // 20800 Hz -> 10400 samples/line, need to resample to 4160
        let decoder = AptDecoder::with_defaults();
        let input: Vec<f64> = (0..10400).map(|i| (i as f64) / 10400.0).collect();
        let output = decoder.resample_to_apt_rate(&input);
        assert_eq!(output.len(), APT_SAMPLES_PER_LINE);
        // First sample should be ~0.0, last should be near 1.0
        assert!(output[0].abs() < 0.01);
        assert!((output[APT_SAMPLES_PER_LINE - 1] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_detect_sync_word_with_tone() {
        // Generate a short signal with a sync-A-like tone embedded
        let effective_rate = APT_SAMPLES_PER_LINE as f64 * APT_LINE_RATE; // 8320 Hz
        let num_samples = APT_SAMPLES_PER_LINE;
        let mut signal = vec![0.0f64; num_samples];

        // Insert sync pattern at offset 0: 7 cycles of 832 Hz
        for i in 0..(SYNC_A_PIXELS * 2) {
            let t = i as f64 / effective_rate;
            signal[i] = (2.0 * PI * SYNC_A_FREQ_HZ * t).sin();
        }

        let peaks = detect_sync_word(&signal, 0.5);
        // Should find at least one sync near offset 0
        assert!(
            !peaks.is_empty(),
            "Should detect sync word in synthetic signal"
        );
        assert!(
            peaks[0] < 20,
            "First sync detection should be near offset 0, got {}",
            peaks[0]
        );
    }

    #[test]
    fn test_coalesce_peaks() {
        let peaks = vec![10, 12, 15, 5000, 5002, 10000];
        let result = coalesce_peaks(&peaks, 2000);
        assert_eq!(result, vec![10, 5000, 10000]);
    }

    #[test]
    fn test_apt_line_as_pixels_roundtrip() {
        let samples = vec![0.5; APT_SAMPLES_PER_LINE];
        let line = decode_line(&samples).unwrap();
        let pixels = line.as_pixels();
        assert_eq!(pixels.len(), APT_LINE_PIXELS);
        // All should be 128
        assert!(pixels.iter().all(|&p| p == 128));
    }

    #[test]
    fn test_image_channel_enum() {
        assert_ne!(ImageChannel::VisibleA, ImageChannel::InfraredB);
        let ch = ImageChannel::VisibleA;
        let ch2 = ch; // Copy
        assert_eq!(ch, ch2);
    }

    #[test]
    fn test_decoder_full_pipeline() {
        let decoder = AptDecoder::with_defaults();

        // Generate synthetic APT signal: constant AM envelope
        let samples_per_line = decoder.config().samples_per_line();
        let num_lines = 3;
        let total_samples = samples_per_line * num_lines;
        let fs = decoder.config().sample_rate;

        let signal: Vec<f64> = (0..total_samples)
            .map(|i| {
                let t = i as f64 / fs;
                let carrier = (2.0 * PI * APT_CARRIER_HZ * t).sin();
                0.6 * carrier // constant brightness ~0.6
            })
            .collect();

        // Demodulate
        let envelope = decoder.am_demodulate(&signal);
        assert_eq!(envelope.len(), total_samples);

        // Resample
        let resampled = decoder.resample_to_apt_rate(&envelope);
        assert!(
            resampled.len() >= APT_SAMPLES_PER_LINE,
            "Resampled signal too short"
        );

        // Decode lines
        let mut lines = Vec::new();
        for chunk in resampled.chunks(APT_SAMPLES_PER_LINE) {
            if let Some(line) = decoder.decode_line(chunk) {
                lines.push(line);
            }
        }
        assert!(!lines.is_empty(), "Should decode at least one line");

        // Assemble image
        let img = decoder.assemble_image(&lines, ImageChannel::VisibleA);
        assert_eq!(img.width, SPACE_A_PIXELS);
        assert_eq!(img.height, lines.len());
    }
}
