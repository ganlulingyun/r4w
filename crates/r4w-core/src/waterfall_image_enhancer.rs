//! Waterfall / spectrogram image enhancement for SDR visualization.
//!
//! This module operates on 2-D magnitude arrays (rows = time, columns = frequency)
//! and provides contrast enhancement, noise removal, colour-mapping, and peak
//! highlighting suitable for real-time or post-processing SDR displays.
//!
//! Only the Rust standard library is used — no external crates.
//!
//! # Quick start
//!
//! ```
//! use r4w_core::waterfall_image_enhancer::{WaterfallImageEnhancer, ContrastMode, ColormapKind};
//!
//! // 4 time-rows × 8 frequency-bins of linear magnitude data
//! let data: Vec<Vec<f64>> = vec![
//!     vec![0.0, 0.1, 0.5, 1.0, 0.5, 0.1, 0.0, 0.0],
//!     vec![0.0, 0.2, 0.6, 1.2, 0.6, 0.2, 0.0, 0.0],
//!     vec![0.0, 0.1, 0.4, 0.9, 0.4, 0.1, 0.0, 0.0],
//!     vec![0.0, 0.3, 0.7, 1.5, 0.7, 0.3, 0.0, 0.0],
//! ];
//!
//! let mut enh = WaterfallImageEnhancer::new(data);
//!
//! // Convert to dB, stretch contrast, and map to colour
//! enh.to_db_scale(-60.0);
//! enh.contrast_stretch(ContrastMode::Logarithmic);
//! let rgb = enh.apply_colormap(ColormapKind::Viridis);
//!
//! assert_eq!(rgb.len(), 4);       // 4 rows
//! assert_eq!(rgb[0].len(), 8);    // 8 columns
//! // Each element is an (R, G, B) tuple in 0..=255
//! let (r, g, b) = rgb[0][0];
//! assert!(r <= 255 && g <= 255 && b <= 255);
//! ```

use std::f64::consts::LN_10;

// ── Public types ────────────────────────────────────────────────────────────

/// Contrast stretching mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContrastMode {
    /// Linear min-max stretch to \[0, 1\].
    Linear,
    /// Logarithmic stretch: `log10(1 + value)` then normalise.
    Logarithmic,
    /// Power-law (gamma) stretch with exponent γ.
    Gamma(f64),
}

/// Available colour maps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColormapKind {
    Grayscale,
    Jet,
    Viridis,
    Plasma,
}

/// A detected peak in the waterfall.
#[derive(Debug, Clone, PartialEq)]
pub struct PeakLocation {
    /// Row (time) index.
    pub row: usize,
    /// Column (frequency) index.
    pub col: usize,
    /// Value at the peak (in the current scale of the data).
    pub value: f64,
}

/// AGC axis selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgcAxis {
    /// Normalise each row independently.
    Row,
    /// Normalise each column independently.
    Column,
}

// ── Main struct ─────────────────────────────────────────────────────────────

/// 2-D waterfall / spectrogram image enhancer.
///
/// Internally stores a row-major `Vec<Vec<f64>>` where each inner vector is a
/// frequency row (one time slice).  All enhancement methods mutate the data
/// in-place so they can be freely chained.
#[derive(Debug, Clone)]
pub struct WaterfallImageEnhancer {
    /// Row-major magnitude data: `data[time][freq]`.
    data: Vec<Vec<f64>>,
}

impl WaterfallImageEnhancer {
    // ── Construction ────────────────────────────────────────────────────

    /// Create a new enhancer from a 2-D magnitude array.
    ///
    /// `data[row][col]` — rows are time slices, columns are frequency bins.
    /// All rows must have the same length (panics otherwise).
    pub fn new(data: Vec<Vec<f64>>) -> Self {
        if !data.is_empty() {
            let cols = data[0].len();
            assert!(
                data.iter().all(|r| r.len() == cols),
                "All rows must have the same number of columns"
            );
        }
        Self { data }
    }

    /// Number of time rows.
    pub fn rows(&self) -> usize {
        self.data.len()
    }

    /// Number of frequency columns (0 if empty).
    pub fn cols(&self) -> usize {
        self.data.first().map_or(0, |r| r.len())
    }

    /// Borrow the inner data.
    pub fn data(&self) -> &Vec<Vec<f64>> {
        &self.data
    }

    /// Consume the enhancer and return the inner data.
    pub fn into_data(self) -> Vec<Vec<f64>> {
        self.data
    }

    // ── dB scale conversion ─────────────────────────────────────────────

    /// Convert linear magnitudes to dB scale: `20 * log10(value)`, clamped to
    /// `floor_db` from below. Values ≤ 0 are mapped to `floor_db`.
    pub fn to_db_scale(&mut self, floor_db: f64) {
        let factor = 20.0 / LN_10; // 20*log10(x) = 20/ln(10)*ln(x)
        for row in &mut self.data {
            for v in row.iter_mut() {
                if *v <= 0.0 {
                    *v = floor_db;
                } else {
                    *v = (factor * v.ln()).max(floor_db);
                }
            }
        }
    }

    // ── Contrast stretching ─────────────────────────────────────────────

    /// Apply a contrast stretching transform to normalise the data into
    /// approximately the \[0, 1\] range.
    pub fn contrast_stretch(&mut self, mode: ContrastMode) {
        let (min, max) = self.min_max();
        if (max - min).abs() < 1e-15 {
            // Flat image — map everything to 0.
            for row in &mut self.data {
                for v in row.iter_mut() {
                    *v = 0.0;
                }
            }
            return;
        }
        match mode {
            ContrastMode::Linear => {
                let range = max - min;
                for row in &mut self.data {
                    for v in row.iter_mut() {
                        *v = (*v - min) / range;
                    }
                }
            }
            ContrastMode::Logarithmic => {
                // Shift to [0, ..] then log10(1 + x)
                let range = max - min;
                let log_max = (1.0 + range).log10();
                for row in &mut self.data {
                    for v in row.iter_mut() {
                        *v = (1.0 + (*v - min)).log10() / log_max;
                    }
                }
            }
            ContrastMode::Gamma(gamma) => {
                let range = max - min;
                for row in &mut self.data {
                    for v in row.iter_mut() {
                        let norm = (*v - min) / range;
                        *v = norm.powf(gamma);
                    }
                }
            }
        }
    }

    // ── Histogram equalisation ──────────────────────────────────────────

    /// Histogram equalisation — redistributes intensities so that the
    /// cumulative distribution is approximately uniform.  Values are mapped
    /// to \[0, 1\].  `num_bins` controls the histogram resolution.
    pub fn histogram_equalize(&mut self, num_bins: usize) {
        let num_bins = num_bins.max(2);
        let (min, max) = self.min_max();
        if (max - min).abs() < 1e-15 {
            return;
        }

        let range = max - min;
        let total = (self.rows() * self.cols()) as f64;

        // Build histogram
        let mut hist = vec![0usize; num_bins];
        for row in &self.data {
            for &v in row {
                let idx = ((v - min) / range * (num_bins - 1) as f64).round() as usize;
                let idx = idx.min(num_bins - 1);
                hist[idx] += 1;
            }
        }

        // Cumulative distribution
        let mut cdf = vec![0.0f64; num_bins];
        cdf[0] = hist[0] as f64 / total;
        for i in 1..num_bins {
            cdf[i] = cdf[i - 1] + hist[i] as f64 / total;
        }

        // Map values
        for row in &mut self.data {
            for v in row.iter_mut() {
                let idx = ((*v - min) / range * (num_bins - 1) as f64).round() as usize;
                let idx = idx.min(num_bins - 1);
                *v = cdf[idx];
            }
        }
    }

    // ── Background subtraction ──────────────────────────────────────────

    /// Subtract the temporal median from each frequency bin.
    ///
    /// For every column (frequency bin) the median across all rows (time) is
    /// computed and subtracted.  Negative results are clamped to 0.
    pub fn subtract_background(&mut self) {
        let nrows = self.rows();
        let ncols = self.cols();
        if nrows == 0 || ncols == 0 {
            return;
        }

        for c in 0..ncols {
            let mut col_vals: Vec<f64> = (0..nrows).map(|r| self.data[r][c]).collect();
            col_vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = if nrows % 2 == 1 {
                col_vals[nrows / 2]
            } else {
                (col_vals[nrows / 2 - 1] + col_vals[nrows / 2]) / 2.0
            };
            for r in 0..nrows {
                self.data[r][c] = (self.data[r][c] - median).max(0.0);
            }
        }
    }

    // ── Noise floor estimation & removal ────────────────────────────────

    /// Estimate the noise floor per frequency bin as the `percentile`-th value
    /// across time, then subtract it.  `percentile` is in \[0, 100\].
    /// Negative results are clamped to 0.
    pub fn remove_noise_floor(&mut self, percentile: f64) {
        let percentile = percentile.clamp(0.0, 100.0);
        let nrows = self.rows();
        let ncols = self.cols();
        if nrows == 0 || ncols == 0 {
            return;
        }

        for c in 0..ncols {
            let mut col_vals: Vec<f64> = (0..nrows).map(|r| self.data[r][c]).collect();
            col_vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let idx = ((percentile / 100.0) * (nrows - 1) as f64).round() as usize;
            let idx = idx.min(nrows - 1);
            let floor = col_vals[idx];
            for r in 0..nrows {
                self.data[r][c] = (self.data[r][c] - floor).max(0.0);
            }
        }
    }

    // ── Automatic gain control ──────────────────────────────────────────

    /// Per-row or per-column automatic gain control.
    ///
    /// Each row (or column) is normalised to \[0, 1\] independently.
    pub fn agc(&mut self, axis: AgcAxis) {
        match axis {
            AgcAxis::Row => {
                for row in &mut self.data {
                    let (min, max) = row_min_max(row);
                    let range = max - min;
                    if range.abs() < 1e-15 {
                        for v in row.iter_mut() {
                            *v = 0.0;
                        }
                    } else {
                        for v in row.iter_mut() {
                            *v = (*v - min) / range;
                        }
                    }
                }
            }
            AgcAxis::Column => {
                let ncols = self.cols();
                let nrows = self.rows();
                for c in 0..ncols {
                    let mut min = f64::INFINITY;
                    let mut max = f64::NEG_INFINITY;
                    for r in 0..nrows {
                        let v = self.data[r][c];
                        if v < min {
                            min = v;
                        }
                        if v > max {
                            max = v;
                        }
                    }
                    let range = max - min;
                    if range.abs() < 1e-15 {
                        for r in 0..nrows {
                            self.data[r][c] = 0.0;
                        }
                    } else {
                        for r in 0..nrows {
                            self.data[r][c] = (self.data[r][c] - min) / range;
                        }
                    }
                }
            }
        }
    }

    // ── 2-D median filter ───────────────────────────────────────────────

    /// Apply a 2-D median filter with the given `radius`.
    ///
    /// The window size is `(2*radius+1) × (2*radius+1)`.  Border pixels use a
    /// smaller window (the filter never reads out-of-bounds).
    pub fn median_filter_2d(&mut self, radius: usize) {
        if radius == 0 {
            return;
        }
        let nrows = self.rows();
        let ncols = self.cols();
        if nrows == 0 || ncols == 0 {
            return;
        }

        let mut out = vec![vec![0.0f64; ncols]; nrows];
        let mut window = Vec::new();

        for r in 0..nrows {
            for c in 0..ncols {
                window.clear();
                let r_lo = r.saturating_sub(radius);
                let r_hi = (r + radius).min(nrows - 1);
                let c_lo = c.saturating_sub(radius);
                let c_hi = (c + radius).min(ncols - 1);
                for rr in r_lo..=r_hi {
                    for cc in c_lo..=c_hi {
                        window.push(self.data[rr][cc]);
                    }
                }
                window.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let mid = window.len() / 2;
                out[r][c] = if window.len() % 2 == 1 {
                    window[mid]
                } else {
                    (window[mid - 1] + window[mid]) / 2.0
                };
            }
        }
        self.data = out;
    }

    // ── Peak highlighting ───────────────────────────────────────────────

    /// Detect peaks that exceed `threshold` (in the current data scale).
    ///
    /// A sample is considered a peak if it is greater than `threshold` **and**
    /// greater than or equal to all of its 8-connected neighbours.
    pub fn detect_peaks(&self, threshold: f64) -> Vec<PeakLocation> {
        let nrows = self.rows();
        let ncols = self.cols();
        let mut peaks = Vec::new();

        for r in 0..nrows {
            for c in 0..ncols {
                let v = self.data[r][c];
                if v < threshold {
                    continue;
                }
                let mut is_peak = true;
                'nbr: for dr in [-1i64, 0, 1] {
                    for dc in [-1i64, 0, 1] {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        let nr = r as i64 + dr;
                        let nc = c as i64 + dc;
                        if nr >= 0 && nr < nrows as i64 && nc >= 0 && nc < ncols as i64 {
                            if self.data[nr as usize][nc as usize] > v {
                                is_peak = false;
                                break 'nbr;
                            }
                        }
                    }
                }
                if is_peak {
                    peaks.push(PeakLocation { row: r, col: c, value: v });
                }
            }
        }
        peaks
    }

    /// Produce a peak-highlight overlay.
    ///
    /// Returns a 2-D array of the same shape as the data, where peak pixels
    /// are `1.0` and all others are `0.0`.
    pub fn peak_overlay(&self, threshold: f64) -> Vec<Vec<f64>> {
        let nrows = self.rows();
        let ncols = self.cols();
        let mut overlay = vec![vec![0.0f64; ncols]; nrows];
        for p in self.detect_peaks(threshold) {
            overlay[p.row][p.col] = 1.0;
        }
        overlay
    }

    // ── Colour-map application ──────────────────────────────────────────

    /// Map the current (normalised) data to RGB triples using the chosen
    /// colour map.
    ///
    /// The data should ideally be in \[0, 1\] (run `contrast_stretch` first).
    /// Values outside that range are clamped.
    pub fn apply_colormap(&self, kind: ColormapKind) -> Vec<Vec<(u8, u8, u8)>> {
        self.data
            .iter()
            .map(|row| {
                row.iter()
                    .map(|&v| {
                        let t = v.clamp(0.0, 1.0);
                        match kind {
                            ColormapKind::Grayscale => colormap_grayscale(t),
                            ColormapKind::Jet => colormap_jet(t),
                            ColormapKind::Viridis => colormap_viridis(t),
                            ColormapKind::Plasma => colormap_plasma(t),
                        }
                    })
                    .collect()
            })
            .collect()
    }

    // ── Helpers (private) ───────────────────────────────────────────────

    /// Global (min, max) of the data.
    fn min_max(&self) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for row in &self.data {
            for &v in row {
                if v < min {
                    min = v;
                }
                if v > max {
                    max = v;
                }
            }
        }
        if min.is_infinite() {
            (0.0, 0.0)
        } else {
            (min, max)
        }
    }
}

// ── Free helper functions ───────────────────────────────────────────────────

fn row_min_max(row: &[f64]) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for &v in row {
        if v < min {
            min = v;
        }
        if v > max {
            max = v;
        }
    }
    if min.is_infinite() {
        (0.0, 0.0)
    } else {
        (min, max)
    }
}

// ── Colour-map implementations ──────────────────────────────────────────────

fn to_u8(x: f64) -> u8 {
    (x * 255.0).round().clamp(0.0, 255.0) as u8
}

fn colormap_grayscale(t: f64) -> (u8, u8, u8) {
    let v = to_u8(t);
    (v, v, v)
}

/// Classic "jet" colour map (blue → cyan → green → yellow → red).
fn colormap_jet(t: f64) -> (u8, u8, u8) {
    let r = if t < 0.375 {
        0.0
    } else if t < 0.625 {
        (t - 0.375) / 0.25
    } else {
        1.0
    };

    let g = if t < 0.125 {
        0.0
    } else if t < 0.375 {
        (t - 0.125) / 0.25
    } else if t < 0.625 {
        1.0
    } else if t < 0.875 {
        1.0 - (t - 0.625) / 0.25
    } else {
        0.0
    };

    let b = if t < 0.125 {
        0.5 + t / 0.125 * 0.5
    } else if t < 0.375 {
        1.0
    } else if t < 0.625 {
        1.0 - (t - 0.375) / 0.25
    } else {
        0.0
    };

    (to_u8(r), to_u8(g), to_u8(b))
}

/// Attempt at perceptually uniform "viridis" map — piecewise-linear through
/// a handful of anchor colours sampled from the matplotlib original.
fn colormap_viridis(t: f64) -> (u8, u8, u8) {
    // 5 anchor points (t, R, G, B) in [0,1]
    const ANCHORS: [(f64, f64, f64, f64); 5] = [
        (0.00, 0.267, 0.004, 0.329),
        (0.25, 0.282, 0.141, 0.458),
        (0.50, 0.127, 0.567, 0.551),
        (0.75, 0.454, 0.820, 0.322),
        (1.00, 0.993, 0.906, 0.144),
    ];
    lerp_anchors(&ANCHORS, t)
}

/// Approximate "plasma" map — piecewise-linear through anchor colours.
fn colormap_plasma(t: f64) -> (u8, u8, u8) {
    const ANCHORS: [(f64, f64, f64, f64); 5] = [
        (0.00, 0.050, 0.030, 0.530),
        (0.25, 0.417, 0.001, 0.658),
        (0.50, 0.798, 0.125, 0.424),
        (0.75, 0.973, 0.434, 0.098),
        (1.00, 0.940, 0.975, 0.131),
    ];
    lerp_anchors(&ANCHORS, t)
}

fn lerp_anchors(anchors: &[(f64, f64, f64, f64)], t: f64) -> (u8, u8, u8) {
    let t = t.clamp(0.0, 1.0);
    // Find the surrounding segment.
    let mut i = 0;
    while i + 1 < anchors.len() - 1 && anchors[i + 1].0 < t {
        i += 1;
    }
    let (t0, r0, g0, b0) = anchors[i];
    let (t1, r1, g1, b1) = anchors[i + 1];
    let frac = if (t1 - t0).abs() < 1e-15 {
        0.0
    } else {
        (t - t0) / (t1 - t0)
    };
    let r = r0 + frac * (r1 - r0);
    let g = g0 + frac * (g1 - g0);
    let b = b0 + frac * (b1 - b0);
    (to_u8(r), to_u8(g), to_u8(b))
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a small 4×4 test grid.
    fn sample_data() -> Vec<Vec<f64>> {
        vec![
            vec![0.0, 1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0, 7.0],
            vec![8.0, 9.0, 10.0, 11.0],
            vec![12.0, 13.0, 14.0, 15.0],
        ]
    }

    #[test]
    fn test_new_and_dimensions() {
        let enh = WaterfallImageEnhancer::new(sample_data());
        assert_eq!(enh.rows(), 4);
        assert_eq!(enh.cols(), 4);
    }

    #[test]
    fn test_empty() {
        let enh = WaterfallImageEnhancer::new(vec![]);
        assert_eq!(enh.rows(), 0);
        assert_eq!(enh.cols(), 0);
    }

    #[test]
    #[should_panic(expected = "same number of columns")]
    fn test_ragged_rows_panic() {
        WaterfallImageEnhancer::new(vec![vec![1.0, 2.0], vec![3.0]]);
    }

    #[test]
    fn test_to_db_scale() {
        let data = vec![vec![0.0, 1.0, 10.0, 100.0]];
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.to_db_scale(-80.0);
        let d = &enh.data()[0];
        assert!((d[0] - (-80.0)).abs() < 1e-6, "zero maps to floor");
        assert!((d[1] - 0.0).abs() < 1e-6, "1.0 -> 0 dB");
        assert!((d[2] - 20.0).abs() < 1e-6, "10.0 -> 20 dB");
        assert!((d[3] - 40.0).abs() < 1e-6, "100.0 -> 40 dB");
    }

    #[test]
    fn test_contrast_linear() {
        let mut enh = WaterfallImageEnhancer::new(vec![vec![2.0, 4.0, 6.0]]);
        enh.contrast_stretch(ContrastMode::Linear);
        let d = &enh.data()[0];
        assert!((d[0] - 0.0).abs() < 1e-12);
        assert!((d[1] - 0.5).abs() < 1e-12);
        assert!((d[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_contrast_logarithmic() {
        let mut enh = WaterfallImageEnhancer::new(vec![vec![0.0, 5.0, 10.0]]);
        enh.contrast_stretch(ContrastMode::Logarithmic);
        let d = &enh.data()[0];
        assert!((d[0] - 0.0).abs() < 1e-12);
        assert!((d[2] - 1.0).abs() < 1e-12);
        // Middle should be between 0 and 1 and less than linear midpoint
        assert!(d[1] > 0.0 && d[1] < 1.0);
    }

    #[test]
    fn test_contrast_gamma() {
        let mut enh = WaterfallImageEnhancer::new(vec![vec![0.0, 5.0, 10.0]]);
        enh.contrast_stretch(ContrastMode::Gamma(2.0));
        let d = &enh.data()[0];
        assert!((d[0] - 0.0).abs() < 1e-12);
        assert!((d[2] - 1.0).abs() < 1e-12);
        // 0.5^2 = 0.25
        assert!((d[1] - 0.25).abs() < 1e-12);
    }

    #[test]
    fn test_contrast_flat_image() {
        let mut enh = WaterfallImageEnhancer::new(vec![vec![5.0, 5.0, 5.0]]);
        enh.contrast_stretch(ContrastMode::Linear);
        // All values should be 0 (constant image)
        for &v in &enh.data()[0] {
            assert!((v - 0.0).abs() < 1e-12);
        }
    }

    #[test]
    fn test_histogram_equalize() {
        let mut enh = WaterfallImageEnhancer::new(sample_data());
        enh.histogram_equalize(16);
        let d = enh.data();
        // First element should be close to 0, last close to 1.
        assert!(d[0][0] < 0.15);
        assert!(d[3][3] > 0.85);
    }

    #[test]
    fn test_subtract_background() {
        // Constant background of 10, signal spike at (1,1)
        let data = vec![
            vec![10.0, 10.0, 10.0],
            vec![10.0, 50.0, 10.0],
            vec![10.0, 10.0, 10.0],
        ];
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.subtract_background();
        // Background columns should be ~0
        assert!((enh.data()[0][0] - 0.0).abs() < 1e-12);
        assert!((enh.data()[2][2] - 0.0).abs() < 1e-12);
        // Signal should remain (50 - median(10,50,10) = 50 - 10 = 40)
        assert!((enh.data()[1][1] - 40.0).abs() < 1e-12);
    }

    #[test]
    fn test_remove_noise_floor() {
        let data = vec![
            vec![1.0, 2.0],
            vec![1.0, 2.0],
            vec![1.0, 100.0],
        ];
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.remove_noise_floor(50.0); // median percentile
        // Col 0: all 1.0, median = 1.0 -> all become 0
        assert!((enh.data()[0][0] - 0.0).abs() < 1e-12);
        // Col 1: sorted = [2, 2, 100], 50th %ile -> idx=1 -> 2.0, so top row becomes 98
        assert!((enh.data()[2][1] - 98.0).abs() < 1e-12);
    }

    #[test]
    fn test_agc_row() {
        let data = vec![
            vec![10.0, 20.0, 30.0],
            vec![100.0, 200.0, 300.0],
        ];
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.agc(AgcAxis::Row);
        // Each row independently normalised to [0,1]
        assert!((enh.data()[0][0] - 0.0).abs() < 1e-12);
        assert!((enh.data()[0][2] - 1.0).abs() < 1e-12);
        assert!((enh.data()[1][0] - 0.0).abs() < 1e-12);
        assert!((enh.data()[1][2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_agc_column() {
        let data = vec![
            vec![1.0, 100.0],
            vec![3.0, 300.0],
            vec![5.0, 500.0],
        ];
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.agc(AgcAxis::Column);
        // Col 0: [1,3,5] -> [0, 0.5, 1]
        assert!((enh.data()[0][0] - 0.0).abs() < 1e-12);
        assert!((enh.data()[1][0] - 0.5).abs() < 1e-12);
        assert!((enh.data()[2][0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_median_filter_2d_radius_0() {
        let original = sample_data();
        let mut enh = WaterfallImageEnhancer::new(original.clone());
        enh.median_filter_2d(0);
        assert_eq!(*enh.data(), original, "radius 0 should be identity");
    }

    #[test]
    fn test_median_filter_2d_removes_spike() {
        let mut data = vec![vec![1.0; 5]; 5];
        data[2][2] = 100.0; // spike
        let mut enh = WaterfallImageEnhancer::new(data);
        enh.median_filter_2d(1);
        // Centre should now be 1.0 (the median of mostly-1s)
        assert!((enh.data()[2][2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_detect_peaks_basic() {
        let data = vec![
            vec![0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.5, 0.0, 0.0],
            vec![0.0, 0.5, 1.0, 0.5, 0.0],
            vec![0.0, 0.0, 0.5, 0.0, 0.0],
            vec![0.0, 0.0, 0.0, 0.0, 0.0],
        ];
        let enh = WaterfallImageEnhancer::new(data);
        let peaks = enh.detect_peaks(0.8);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].row, 2);
        assert_eq!(peaks[0].col, 2);
        assert!((peaks[0].value - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_peak_overlay_shape() {
        let enh = WaterfallImageEnhancer::new(sample_data());
        let ov = enh.peak_overlay(100.0);
        assert_eq!(ov.len(), enh.rows());
        assert_eq!(ov[0].len(), enh.cols());
    }

    #[test]
    fn test_colormap_grayscale_endpoints() {
        let (r, g, b) = colormap_grayscale(0.0);
        assert_eq!((r, g, b), (0, 0, 0));
        let (r, g, b) = colormap_grayscale(1.0);
        assert_eq!((r, g, b), (255, 255, 255));
    }

    #[test]
    fn test_colormap_jet_endpoints() {
        let (r, _, b) = colormap_jet(0.0);
        // At t=0 jet is blue-ish (blue > red)
        assert!(b > r);
        let (r, _, b) = colormap_jet(1.0);
        // At t=1 jet is red-ish (red > blue)
        assert!(r > b);
    }

    #[test]
    fn test_apply_colormap_shape() {
        let enh = WaterfallImageEnhancer::new(vec![vec![0.0, 0.5, 1.0]; 3]);
        for kind in [
            ColormapKind::Grayscale,
            ColormapKind::Jet,
            ColormapKind::Viridis,
            ColormapKind::Plasma,
        ] {
            let rgb = enh.apply_colormap(kind);
            assert_eq!(rgb.len(), 3);
            assert_eq!(rgb[0].len(), 3);
        }
    }

    #[test]
    fn test_into_data_roundtrip() {
        let original = sample_data();
        let enh = WaterfallImageEnhancer::new(original.clone());
        let recovered = enh.into_data();
        assert_eq!(recovered, original);
    }
}
