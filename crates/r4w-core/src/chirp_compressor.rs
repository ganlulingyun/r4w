//! Chirp (LFM) Pulse Compression
//!
//! Implements matched-filter pulse compression for Linear Frequency Modulated
//! (LFM) chirp signals used in radar and sonar applications. Pulse compression
//! allows a long-duration, low-peak-power transmitted pulse to achieve the range
//! resolution of a short pulse while retaining the energy advantage of the long
//! pulse. The processing gain equals the time-bandwidth product (BT).
//!
//! # Features
//!
//! - Up-sweep and down-sweep LFM chirp generation
//! - Matched-filter compression (time-domain correlation)
//! - Windowed compression for sidelobe reduction (Hamming, Hann, Kaiser)
//! - Peak sidelobe ratio (PSR) and mainlobe width measurement utilities
//! - Radar-friendly factory with MHz / microsecond parameters
//!
//! # Example
//!
//! ```rust
//! use r4w_core::chirp_compressor::{ChirpCompressor, generate_chirp};
//!
//! let bandwidth = 1e6;   // 1 MHz bandwidth
//! let duration  = 10e-6; // 10 us pulse
//! let fs        = 10e6;  // 10 MHz sample rate
//!
//! let tx = generate_chirp(bandwidth, duration, fs);
//! let compressor = ChirpCompressor::new(bandwidth, duration, fs);
//!
//! // Compress the transmitted chirp against itself (auto-correlation).
//! let compressed = compressor.compress(&tx);
//! assert!(!compressed.is_empty());
//!
//! // The time-bandwidth product gives the compression ratio.
//! let bt = compressor.compression_ratio();
//! assert!((bt - 10.0).abs() < 1e-9);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper arithmetic on (f64, f64) "complex" tuples
// ---------------------------------------------------------------------------

#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn cmag(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

// ---------------------------------------------------------------------------
// Window functions
// ---------------------------------------------------------------------------

/// Window function type for sidelobe control during pulse compression.
#[derive(Debug, Clone)]
pub enum WindowType {
    /// No windowing (rectangular).
    Rectangular,
    /// Hamming window — good sidelobe suppression with moderate mainlobe widening.
    Hamming,
    /// Hann (Hanning) window — zero-endpoint, smooth taper.
    Hann,
    /// Kaiser window with shape parameter beta.
    Kaiser(f64),
}

/// Compute window coefficients of the given type and length.
fn window_coefficients(wtype: &WindowType, len: usize) -> Vec<f64> {
    match wtype {
        WindowType::Rectangular => vec![1.0; len],
        WindowType::Hamming => (0..len)
            .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / (len as f64 - 1.0)).cos())
            .collect(),
        WindowType::Hann => (0..len)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (len as f64 - 1.0)).cos()))
            .collect(),
        WindowType::Kaiser(beta) => kaiser_window(len, *beta),
    }
}

/// Zeroth-order modified Bessel function of the first kind, I0(x).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    for k in 1..=50 {
        term *= (x / (2.0 * k as f64)) * (x / (2.0 * k as f64));
        sum += term;
        if term < 1e-15 * sum {
            break;
        }
    }
    sum
}

/// Generate a Kaiser window of length `len` with shape parameter `beta`.
fn kaiser_window(len: usize, beta: f64) -> Vec<f64> {
    let denom = bessel_i0(beta);
    let m = len as f64 - 1.0;
    (0..len)
        .map(|n| {
            let arg = beta * (1.0 - ((2.0 * n as f64 / m) - 1.0).powi(2)).sqrt();
            bessel_i0(arg) / denom
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Chirp generation
// ---------------------------------------------------------------------------

/// Generate a linear frequency modulated (LFM) up-sweep chirp signal.
///
/// The instantaneous frequency sweeps from `-bandwidth/2` to `+bandwidth/2`
/// over the given `duration` at the specified `sample_rate`.
///
/// Returns a vector of `(re, im)` complex samples with unit magnitude.
pub fn generate_chirp(bandwidth: f64, duration: f64, sample_rate: f64) -> Vec<(f64, f64)> {
    let n = (duration * sample_rate).round() as usize;
    let chirp_rate = bandwidth / duration;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * (-bandwidth / 2.0 * t + 0.5 * chirp_rate * t * t);
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Generate a linear frequency modulated (LFM) down-sweep chirp signal.
///
/// The instantaneous frequency sweeps from `+bandwidth/2` to `-bandwidth/2`
/// over the given `duration` at the specified `sample_rate`.
///
/// Returns a vector of `(re, im)` complex samples with unit magnitude.
pub fn generate_down_chirp(bandwidth: f64, duration: f64, sample_rate: f64) -> Vec<(f64, f64)> {
    let n = (duration * sample_rate).round() as usize;
    let chirp_rate = bandwidth / duration;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * (bandwidth / 2.0 * t - 0.5 * chirp_rate * t * t);
            (phase.cos(), phase.sin())
        })
        .collect()
}

// ---------------------------------------------------------------------------
// ChirpCompressor
// ---------------------------------------------------------------------------

/// Matched-filter pulse compressor for LFM chirp signals.
///
/// Performs time-domain cross-correlation of a received signal with an
/// internally generated reference chirp. The output peak location corresponds
/// to the target delay and the peak amplitude reflects the processing gain
/// (equal to the time-bandwidth product).
pub struct ChirpCompressor {
    bandwidth: f64,
    pulse_duration: f64,
    sample_rate: f64,
    reference: Vec<(f64, f64)>,
}

impl ChirpCompressor {
    /// Create a new compressor for an LFM up-chirp with the given parameters.
    ///
    /// * `bandwidth_hz` — sweep bandwidth in Hz
    /// * `pulse_duration_s` — pulse duration in seconds
    /// * `sample_rate` — sample rate in Hz
    pub fn new(bandwidth_hz: f64, pulse_duration_s: f64, sample_rate: f64) -> Self {
        let reference = generate_chirp(bandwidth_hz, pulse_duration_s, sample_rate);
        Self {
            bandwidth: bandwidth_hz,
            pulse_duration: pulse_duration_s,
            sample_rate,
            reference,
        }
    }

    /// Compress a received signal and return the magnitude envelope.
    ///
    /// This performs cross-correlation of `received` with the reference chirp
    /// (conjugate of the reference used as matched filter) and returns the
    /// magnitude at each lag. The output length is
    /// `received.len() + reference.len() - 1`.
    pub fn compress(&self, received: &[(f64, f64)]) -> Vec<f64> {
        self.compress_complex(received)
            .iter()
            .map(|s| cmag(*s))
            .collect()
    }

    /// Compress a received signal and return the complex correlation output.
    ///
    /// Output length is `received.len() + reference.len() - 1`.
    pub fn compress_complex(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        correlate(received, &self.reference)
    }

    /// Return the time-bandwidth product (compression ratio).
    pub fn compression_ratio(&self) -> f64 {
        self.bandwidth * self.pulse_duration
    }

    /// Return the range resolution in metres: `c / (2 * B)`.
    pub fn range_resolution_m(&self) -> f64 {
        const C: f64 = 299_792_458.0; // speed of light, m/s
        C / (2.0 * self.bandwidth)
    }
}

// ---------------------------------------------------------------------------
// WindowedCompressor
// ---------------------------------------------------------------------------

/// Windowed pulse compressor for LFM chirp signals.
///
/// Applies a spectral window to the matched filter reference in order to
/// suppress range sidelobes at the cost of a slightly wider mainlobe.
pub struct WindowedCompressor {
    reference: Vec<(f64, f64)>,
}

impl WindowedCompressor {
    /// Create a windowed compressor.
    ///
    /// The reference chirp is element-wise multiplied by the chosen window
    /// before being used as the matched filter kernel.
    pub fn new(bandwidth: f64, duration: f64, sample_rate: f64, window: WindowType) -> Self {
        let chirp = generate_chirp(bandwidth, duration, sample_rate);
        let win = window_coefficients(&window, chirp.len());
        let reference: Vec<(f64, f64)> = chirp
            .iter()
            .zip(win.iter())
            .map(|(&(re, im), &w)| (re * w, im * w))
            .collect();
        Self { reference }
    }

    /// Compress using the windowed matched filter and return magnitude output.
    pub fn compress(&self, received: &[(f64, f64)]) -> Vec<f64> {
        correlate(received, &self.reference)
            .iter()
            .map(|s| cmag(*s))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Cross-correlation (time domain)
// ---------------------------------------------------------------------------

/// Compute the full cross-correlation of `x` with `h` (matched filter).
///
/// This computes `sum_k x[n+k] * conj(h[k])` for all valid lags, returning
/// a vector of length `x.len() + h.len() - 1`.
fn correlate(x: &[(f64, f64)], h: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let out_len = x.len() + h.len() - 1;
    let mut out = vec![(0.0, 0.0); out_len];
    for (n, sample) in out.iter_mut().enumerate() {
        let mut acc = (0.0, 0.0);
        for (k, hk) in h.iter().enumerate() {
            let xi = n as isize - k as isize;
            if xi >= 0 && (xi as usize) < x.len() {
                acc = cadd(acc, cmul(x[xi as usize], conj(*hk)));
            }
        }
        *sample = acc;
    }
    out
}

// ---------------------------------------------------------------------------
// Analysis utilities
// ---------------------------------------------------------------------------

/// Measure the peak sidelobe ratio (PSR) in dB.
///
/// PSR = 20 * log10(mainlobe_peak / max_sidelobe).
///
/// The mainlobe is identified as the global peak. Sidelobes are all samples
/// outside the contiguous region around the peak that stays above the first
/// null (the first points where the magnitude rises after falling from the
/// peak).
pub fn peak_sidelobe_ratio(compressed: &[f64]) -> f64 {
    if compressed.is_empty() {
        return 0.0;
    }

    // Find global peak
    let (peak_idx, peak_val) = compressed
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap();

    if *peak_val == 0.0 {
        return 0.0;
    }

    // Walk left from peak to find first null (local minimum)
    let mut left = peak_idx;
    while left > 0 && compressed[left - 1] <= compressed[left] {
        left -= 1;
    }

    // Walk right from peak to find first null (local minimum)
    let mut right = peak_idx;
    while right < compressed.len() - 1 && compressed[right + 1] <= compressed[right] {
        right += 1;
    }

    // Maximum sidelobe is the max value outside the mainlobe region
    let max_sidelobe = compressed[..left]
        .iter()
        .chain(compressed[right + 1..].iter())
        .cloned()
        .fold(0.0_f64, f64::max);

    if max_sidelobe <= 0.0 {
        return f64::INFINITY;
    }

    20.0 * (peak_val / max_sidelobe).log10()
}

/// Measure the mainlobe width (number of samples) at the given threshold in dB
/// below the peak (e.g., -3 dB).
///
/// Returns the number of contiguous samples around the peak that are above
/// `peak - |threshold_db|`.
pub fn mainlobe_width(compressed: &[f64], threshold_db: f64) -> usize {
    if compressed.is_empty() {
        return 0;
    }

    let peak_val = compressed.iter().cloned().fold(0.0_f64, f64::max);
    if peak_val == 0.0 {
        return 0;
    }

    // Threshold is in dB below the peak (caller may pass -3.0 or 3.0; we
    // treat the absolute value).
    let threshold_linear = peak_val * 10.0_f64.powf(-threshold_db.abs() / 20.0);

    let peak_idx = compressed
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap()
        .0;

    let mut left = peak_idx;
    while left > 0 && compressed[left - 1] >= threshold_linear {
        left -= 1;
    }

    let mut right = peak_idx;
    while right < compressed.len() - 1 && compressed[right + 1] >= threshold_linear {
        right += 1;
    }

    right - left + 1
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// Create a [`ChirpCompressor`] using radar-friendly units.
///
/// * `bandwidth_mhz` — bandwidth in **MHz**
/// * `pulse_us` — pulse duration in **microseconds**
/// * `sample_rate` — sample rate in Hz
pub fn radar_compressor(bandwidth_mhz: f64, pulse_us: f64, sample_rate: f64) -> ChirpCompressor {
    ChirpCompressor::new(bandwidth_mhz * 1e6, pulse_us * 1e-6, sample_rate)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const BW: f64 = 1e6;      // 1 MHz
    const DUR: f64 = 10e-6;   // 10 us
    const FS: f64 = 10e6;     // 10 MHz

    /// 1. Generate chirp — output length equals round(duration * sample_rate).
    #[test]
    fn test_chirp_output_length() {
        let chirp = generate_chirp(BW, DUR, FS);
        let expected = (DUR * FS).round() as usize;
        assert_eq!(chirp.len(), expected);
    }

    /// 2. Chirp has constant (unit) envelope — all samples have magnitude ~1.
    #[test]
    fn test_chirp_unit_magnitude() {
        let chirp = generate_chirp(BW, DUR, FS);
        for &(re, im) in &chirp {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-12,
                "Expected unit magnitude, got {}",
                mag
            );
        }
    }

    /// 3. Compression produces peak at correct delay.
    #[test]
    fn test_compression_peak_at_correct_delay() {
        let tx = generate_chirp(BW, DUR, FS);
        let n = tx.len(); // 100
        let compressor = ChirpCompressor::new(BW, DUR, FS);

        // Place the chirp at a known offset inside a longer receive buffer.
        let delay = 50_usize;
        let mut received = vec![(0.0, 0.0); n + delay + 20];
        for (i, &s) in tx.iter().enumerate() {
            received[delay + i] = s;
        }

        let compressed = compressor.compress(&received);
        let peak_idx = compressed
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // The correlation peak should appear near delay + n - 1 (the point
        // where the reference is fully aligned with the embedded chirp).
        let expected = delay + n - 1;
        assert!(
            (peak_idx as isize - expected as isize).unsigned_abs() <= 1,
            "Peak at {} but expected near {}",
            peak_idx,
            expected
        );
    }

    /// 4. Compression ratio equals the time-bandwidth product.
    #[test]
    fn test_compression_ratio() {
        let compressor = ChirpCompressor::new(BW, DUR, FS);
        let bt = compressor.compression_ratio();
        assert!(
            (bt - (BW * DUR)).abs() < 1e-9,
            "Expected BT={}, got {}",
            BW * DUR,
            bt
        );
    }

    /// 5. Range resolution = c / (2 * B).
    #[test]
    fn test_range_resolution() {
        let compressor = ChirpCompressor::new(BW, DUR, FS);
        let res = compressor.range_resolution_m();
        let expected = 299_792_458.0 / (2.0 * BW);
        assert!(
            (res - expected).abs() < 1e-3,
            "Expected {:.3} m, got {:.3} m",
            expected,
            res
        );
    }

    /// 6. Windowed compression (Hamming) produces lower sidelobes than rectangular.
    #[test]
    fn test_windowed_compression_reduces_sidelobes() {
        let tx = generate_chirp(BW, DUR, FS);

        let rect = ChirpCompressor::new(BW, DUR, FS);
        let rect_out = rect.compress(&tx);
        let rect_psr = peak_sidelobe_ratio(&rect_out);

        let hamming = WindowedCompressor::new(BW, DUR, FS, WindowType::Hamming);
        let hamming_out = hamming.compress(&tx);
        let hamming_psr = peak_sidelobe_ratio(&hamming_out);

        // Hamming should have a higher PSR (better sidelobe suppression)
        assert!(
            hamming_psr > rect_psr,
            "Hamming PSR ({:.1} dB) should exceed rectangular PSR ({:.1} dB)",
            hamming_psr,
            rect_psr
        );
    }

    /// 7. Peak sidelobe ratio is positive and reasonable for auto-correlation.
    #[test]
    fn test_peak_sidelobe_ratio_measurement() {
        let tx = generate_chirp(BW, DUR, FS);
        let compressor = ChirpCompressor::new(BW, DUR, FS);
        let compressed = compressor.compress(&tx);
        let psr = peak_sidelobe_ratio(&compressed);

        // For BT=10 rectangular matched filter, PSR is typically ~13 dB
        assert!(
            psr > 5.0 && psr < 30.0,
            "PSR should be in a reasonable range, got {:.1} dB",
            psr
        );
    }

    /// 8. Mainlobe width at -3 dB is narrow relative to full correlation length.
    #[test]
    fn test_mainlobe_width_measurement() {
        let tx = generate_chirp(BW, DUR, FS);
        let compressor = ChirpCompressor::new(BW, DUR, FS);
        let compressed = compressor.compress(&tx);
        let width = mainlobe_width(&compressed, -3.0);

        // Width should be a small fraction of the total output length and > 0
        assert!(width > 0, "Mainlobe width must be > 0");
        assert!(
            width < compressed.len() / 2,
            "Mainlobe width ({}) should be much less than output length ({})",
            width,
            compressed.len()
        );
    }

    /// 9. Down chirp generation: correct length and unit magnitude.
    #[test]
    fn test_down_chirp_generation() {
        let down = generate_down_chirp(BW, DUR, FS);
        let expected_len = (DUR * FS).round() as usize;
        assert_eq!(down.len(), expected_len);

        for &(re, im) in &down {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-12,
                "Down chirp should have unit magnitude, got {}",
                mag
            );
        }

        // Down chirp should differ from up chirp (they are complex conjugates,
        // so the imaginary parts have opposite signs for non-zero phases).
        let up = generate_chirp(BW, DUR, FS);
        let differs = up
            .iter()
            .zip(down.iter())
            .any(|(a, b)| (a.0 - b.0).abs() > 1e-10 || (a.1 - b.1).abs() > 1e-10);
        assert!(differs, "Down chirp should differ from up chirp");
    }

    /// 10. Radar factory creates a compressor with correct converted parameters.
    #[test]
    fn test_radar_factory() {
        let rc = radar_compressor(1.0, 10.0, FS);
        // 1 MHz bandwidth, 10 us pulse -> BT = 10
        assert!(
            (rc.compression_ratio() - 10.0).abs() < 1e-9,
            "Radar factory BT should be 10, got {}",
            rc.compression_ratio()
        );

        // Range resolution should match 1 MHz
        let expected_res = 299_792_458.0 / (2.0 * 1e6);
        assert!(
            (rc.range_resolution_m() - expected_res).abs() < 1e-3,
            "Range resolution mismatch"
        );

        // Should produce the same reference length as direct construction
        let direct = ChirpCompressor::new(1e6, 10e-6, FS);
        assert_eq!(rc.reference.len(), direct.reference.len());
    }
}
