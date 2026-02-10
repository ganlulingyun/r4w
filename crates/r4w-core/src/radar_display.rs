//! Radar Display Data Formatting — PPI, B-scope, and A-scope
//!
//! Provides data structures and rasterization routines for the three
//! classic radar display types:
//!
//! - **A-scope**: amplitude vs. range for a single azimuth (oscilloscope-style)
//! - **B-scope**: rectangular azimuth (x) vs. range (y) intensity display
//! - **PPI (Plan Position Indicator)**: polar range-azimuth map rendered onto
//!   a Cartesian grid, the familiar rotating-sweep radar display
//!
//! Also includes colormap functions for rendering radar imagery and a
//! range-to-sample-index utility.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::radar_display::{PpiData, colormap_jet, colormap_green};
//!
//! let mut ppi = PpiData::new(100_000.0);
//! let samples = vec![0.5; 256];
//! ppi.add_sweep(0.0, samples.clone());
//! ppi.add_sweep(std::f64::consts::FRAC_PI_2, samples);
//!
//! let grid = ppi.to_cartesian(64);
//! assert_eq!(grid.len(), 64);
//! assert_eq!(grid[0].len(), 64);
//!
//! let (r, g, b) = colormap_jet(0.5);
//! assert!(g > r && g > b); // green dominates at midpoint
//!
//! let (r2, g2, b2) = colormap_green(0.8);
//! assert_eq!(r2, 0);
//! assert_eq!(b2, 0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Display type enum
// ---------------------------------------------------------------------------

/// Radar display type selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayType {
    /// A-scope: amplitude vs. range for one azimuth.
    AScope,
    /// B-scope: rectangular azimuth vs. range intensity map.
    BScope,
    /// PPI: Plan Position Indicator (polar sweep on Cartesian grid).
    PpiScope,
}

// ---------------------------------------------------------------------------
// A-scope
// ---------------------------------------------------------------------------

/// A-scope display data: amplitude vs. range for a single azimuth direction.
#[derive(Debug, Clone)]
pub struct AScopeData {
    /// Range bin centers in metres.
    pub range_bins: Vec<f64>,
    /// Amplitude (linear) for each range bin.
    pub amplitude: Vec<f64>,
    /// Antenna azimuth in degrees for this sweep.
    pub azimuth_deg: f64,
}

impl AScopeData {
    /// Convert a raw pulse (time-domain amplitude samples) into A-scope data.
    ///
    /// Each sample index is mapped to range via the radar range equation
    /// `R = c * t / 2`, where `t = index / sample_rate`.  The returned
    /// [`AScopeData`] has `azimuth_deg` set to 0.0; callers may overwrite it.
    pub fn from_pulse(pulse: &[f64], sample_rate: f64) -> Self {
        let c = 299_792_458.0_f64;
        let range_bins: Vec<f64> = (0..pulse.len())
            .map(|i| c * (i as f64) / (2.0 * sample_rate))
            .collect();
        let amplitude = pulse.to_vec();
        Self {
            range_bins,
            amplitude,
            azimuth_deg: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// B-scope
// ---------------------------------------------------------------------------

/// B-scope display data: rectangular azimuth (columns) vs. range (rows).
#[derive(Debug, Clone)]
pub struct BScopeData {
    /// Each inner Vec is one sweep (amplitude per range bin).  Sweeps are
    /// stored in the order they were added.
    pub data: Vec<Vec<f64>>,
    /// Azimuth angle in degrees for each sweep, matching `data` indices.
    pub azimuths_deg: Vec<f64>,
    /// Maximum displayed range in metres (used for axis scaling).
    pub max_range_m: f64,
}

impl BScopeData {
    /// Create a new empty B-scope with the given maximum range.
    pub fn new(max_range_m: f64) -> Self {
        Self {
            data: Vec::new(),
            azimuths_deg: Vec::new(),
            max_range_m,
        }
    }

    /// Append a single sweep at `azimuth_deg` with the given amplitude data.
    pub fn add_sweep(&mut self, azimuth_deg: f64, amplitude: &[f64]) {
        self.azimuths_deg.push(azimuth_deg);
        self.data.push(amplitude.to_vec());
    }

    /// Rasterize the B-scope data onto a rectangular grid of `width` columns
    /// (azimuth) and `height` rows (range).
    ///
    /// Azimuth sweeps are mapped to columns via nearest-neighbour indexing.
    /// Range bins within each sweep are linearly interpolated to `height` rows.
    /// Returns a row-major 2D grid (`[row][col]`) of normalised amplitude
    /// values, or an all-zeros grid if no sweeps have been added.
    pub fn as_image(&self, width: usize, height: usize) -> Vec<Vec<f64>> {
        if self.data.is_empty() || width == 0 || height == 0 {
            return vec![vec![0.0; width]; height];
        }

        let num_sweeps = self.data.len();
        let mut image = vec![vec![0.0; width]; height];

        for col in 0..width {
            // Map column to nearest sweep index.
            let sweep_idx = (col * num_sweeps / width).min(num_sweeps - 1);
            let sweep = &self.data[sweep_idx];
            if sweep.is_empty() {
                continue;
            }
            let num_bins = sweep.len();

            for row in 0..height {
                // Map row to fractional range-bin index (row 0 = near range).
                let frac = (row as f64) * (num_bins as f64 - 1.0) / (height as f64 - 1.0).max(1.0);
                let lo = (frac as usize).min(num_bins - 1);
                let hi = (lo + 1).min(num_bins - 1);
                let t = frac - lo as f64;
                image[row][col] = sweep[lo] * (1.0 - t) + sweep[hi] * t;
            }
        }

        image
    }
}

// ---------------------------------------------------------------------------
// PPI
// ---------------------------------------------------------------------------

/// A single radial sweep in a PPI display.
#[derive(Debug, Clone)]
pub struct PpiSweep {
    /// Azimuth angle in radians (0 = north / up, increasing clockwise in
    /// display convention, but stored as mathematical angle here).
    pub azimuth_rad: f64,
    /// Amplitude samples from near range to far range.
    pub samples: Vec<f64>,
}

/// PPI (Plan Position Indicator) display data.
#[derive(Debug, Clone)]
pub struct PpiData {
    /// Collected radial sweeps.
    pub sweeps: Vec<PpiSweep>,
    /// Maximum displayed range in metres.
    pub max_range_m: f64,
}

impl PpiData {
    /// Create a new empty PPI display with the given maximum range.
    pub fn new(max_range_m: f64) -> Self {
        Self {
            sweeps: Vec::new(),
            max_range_m,
        }
    }

    /// Add a radial sweep at `azimuth_rad` with the given samples.
    pub fn add_sweep(&mut self, azimuth_rad: f64, samples: Vec<f64>) {
        self.sweeps.push(PpiSweep {
            azimuth_rad,
            samples,
        });
    }

    /// Rasterize the PPI polar data onto a square Cartesian grid of
    /// `grid_size x grid_size` pixels.
    ///
    /// The centre of the grid corresponds to the radar location.  Each pixel
    /// is mapped back to polar coordinates `(r, theta)` and the nearest sweep
    /// and range-bin value is looked up.  Pixels outside the maximum range or
    /// with no nearby sweep data are left at 0.0.
    pub fn to_cartesian(&self, grid_size: usize) -> Vec<Vec<f64>> {
        let mut grid = vec![vec![0.0; grid_size]; grid_size];

        if self.sweeps.is_empty() || grid_size == 0 {
            return grid;
        }

        let half = grid_size as f64 / 2.0;

        for row in 0..grid_size {
            for col in 0..grid_size {
                // Pixel centre in normalised coordinates, origin at centre.
                let x = (col as f64 + 0.5 - half) / half; // -1..+1
                let y = (half - row as f64 - 0.5) / half; // -1..+1  (y up)

                let r = (x * x + y * y).sqrt();
                if r > 1.0 {
                    continue; // outside the display circle
                }

                let theta = y.atan2(x); // -PI..PI

                // Find the sweep with the closest azimuth.
                let (best_idx, _) = self
                    .sweeps
                    .iter()
                    .enumerate()
                    .map(|(i, s)| {
                        let mut diff = (s.azimuth_rad - theta).abs();
                        if diff > PI {
                            diff = 2.0 * PI - diff;
                        }
                        (i, diff)
                    })
                    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                    .unwrap();

                let sweep = &self.sweeps[best_idx];
                if sweep.samples.is_empty() {
                    continue;
                }

                let bin_f = r * (sweep.samples.len() as f64 - 1.0);
                let bin = (bin_f.round() as usize).min(sweep.samples.len() - 1);
                grid[row][col] = sweep.samples[bin];
            }
        }

        grid
    }
}

// ---------------------------------------------------------------------------
// Colormaps
// ---------------------------------------------------------------------------

/// Jet colormap: maps a normalised value in `[0, 1]` to an (R, G, B) triplet.
///
/// The mapping follows the classic "jet" scheme:
/// 0.0 = blue, 0.25 = cyan, 0.5 = green, 0.75 = yellow, 1.0 = red.
/// Values outside `[0, 1]` are clamped.
pub fn colormap_jet(value: f64) -> (u8, u8, u8) {
    let v = value.clamp(0.0, 1.0);

    // Piecewise-linear ramps for each channel.
    let r = if v < 0.375 {
        0.0
    } else if v < 0.625 {
        (v - 0.375) / 0.25
    } else {
        1.0
    };

    let g = if v < 0.125 {
        0.0
    } else if v < 0.375 {
        (v - 0.125) / 0.25
    } else if v < 0.625 {
        1.0
    } else if v < 0.875 {
        1.0 - (v - 0.625) / 0.25
    } else {
        0.0
    };

    let b = if v < 0.375 {
        1.0
    } else if v < 0.625 {
        1.0 - (v - 0.375) / 0.25
    } else {
        0.0
    };

    (
        (r * 255.0).round() as u8,
        (g * 255.0).round() as u8,
        (b * 255.0).round() as u8,
    )
}

/// Classic green phosphor radar-scope colormap.
///
/// Maps a normalised value in `[0, 1]` to shades of green with R=0, B=0.
/// Values outside `[0, 1]` are clamped.
pub fn colormap_green(value: f64) -> (u8, u8, u8) {
    let v = value.clamp(0.0, 1.0);
    (0, (v * 255.0).round() as u8, 0)
}

// ---------------------------------------------------------------------------
// Range utility
// ---------------------------------------------------------------------------

/// Convert a slant range in metres to the corresponding sample index.
///
/// Uses the two-way propagation delay: `index = 2 * range_m * sample_rate / c`.
pub fn range_ring_samples(range_m: f64, sample_rate: f64) -> usize {
    let c = 299_792_458.0_f64;
    ((2.0 * range_m * sample_rate) / c).round() as usize
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn test_ascope_data() {
        let pulse = vec![0.0, 0.5, 1.0, 0.7, 0.2];
        let fs = 1e6;
        let a = AScopeData::from_pulse(&pulse, fs);

        assert_eq!(a.amplitude.len(), 5);
        assert_eq!(a.range_bins.len(), 5);
        assert!((a.azimuth_deg - 0.0).abs() < 1e-12);

        // First range bin should be 0 m.
        assert!((a.range_bins[0] - 0.0).abs() < 1e-6);

        // Range increases monotonically.
        for i in 1..a.range_bins.len() {
            assert!(a.range_bins[i] > a.range_bins[i - 1]);
        }

        // Amplitude preserved.
        assert!((a.amplitude[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_bscope_add_sweep() {
        let mut bs = BScopeData::new(50_000.0);
        assert!(bs.data.is_empty());

        bs.add_sweep(0.0, &[1.0, 2.0, 3.0]);
        bs.add_sweep(10.0, &[4.0, 5.0, 6.0]);

        assert_eq!(bs.data.len(), 2);
        assert_eq!(bs.azimuths_deg.len(), 2);
        assert!((bs.azimuths_deg[1] - 10.0).abs() < 1e-12);
        assert!((bs.data[1][0] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_bscope_rasterize() {
        let mut bs = BScopeData::new(10_000.0);
        // Two identical sweeps of constant amplitude 0.75.
        bs.add_sweep(0.0, &[0.75; 32]);
        bs.add_sweep(90.0, &[0.75; 32]);

        let img = bs.as_image(8, 16);
        assert_eq!(img.len(), 16);
        assert_eq!(img[0].len(), 8);

        // All values should be 0.75 since input is constant.
        for row in &img {
            for &v in row {
                assert!((v - 0.75).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_ppi_add_sweep() {
        let mut ppi = PpiData::new(100_000.0);
        assert!(ppi.sweeps.is_empty());

        ppi.add_sweep(0.0, vec![1.0, 2.0, 3.0]);
        ppi.add_sweep(FRAC_PI_2, vec![4.0, 5.0, 6.0]);

        assert_eq!(ppi.sweeps.len(), 2);
        assert!((ppi.sweeps[0].azimuth_rad - 0.0).abs() < 1e-12);
        assert!((ppi.sweeps[1].azimuth_rad - FRAC_PI_2).abs() < 1e-12);
        assert_eq!(ppi.sweeps[1].samples.len(), 3);
    }

    #[test]
    fn test_ppi_cartesian() {
        // Create a PPI with four sweeps at cardinal directions, constant value.
        let mut ppi = PpiData::new(1000.0);
        let samps = vec![0.5; 64];
        for angle in &[0.0, FRAC_PI_2, PI, 3.0 * FRAC_PI_2] {
            ppi.add_sweep(*angle, samps.clone());
        }

        let grid = ppi.to_cartesian(32);
        assert_eq!(grid.len(), 32);
        assert_eq!(grid[0].len(), 32);

        // Centre pixel should be 0.5 (within the circle, nearest sweep bin 0).
        let mid = 32 / 2;
        assert!((grid[mid][mid] - 0.5).abs() < 1e-6);

        // Corner pixels outside radius should be 0.0.
        assert!((grid[0][0] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_colormap_jet() {
        // v = 0.0 -> blue
        let (r, g, b) = colormap_jet(0.0);
        assert_eq!(r, 0);
        assert_eq!(g, 0);
        assert!(b > 200);

        // v = 1.0 -> red
        let (r, g, b) = colormap_jet(1.0);
        assert!(r > 200);
        assert_eq!(g, 0);
        assert_eq!(b, 0);

        // v = 0.5 -> green dominates
        let (r, g, b) = colormap_jet(0.5);
        assert!(g >= r);
        assert!(g >= b);

        // Clamping: negative stays blue, >1 stays red.
        assert_eq!(colormap_jet(-0.5), colormap_jet(0.0));
        assert_eq!(colormap_jet(2.0), colormap_jet(1.0));
    }

    #[test]
    fn test_colormap_green() {
        let (r, g, b) = colormap_green(0.0);
        assert_eq!((r, g, b), (0, 0, 0));

        let (r, g, b) = colormap_green(1.0);
        assert_eq!((r, g, b), (0, 255, 0));

        let (r, g, b) = colormap_green(0.5);
        assert_eq!(r, 0);
        assert_eq!(b, 0);
        assert!((g as f64 - 128.0).abs() < 2.0);

        // Clamping.
        assert_eq!(colormap_green(-1.0), (0, 0, 0));
        assert_eq!(colormap_green(5.0), (0, 255, 0));
    }

    #[test]
    fn test_range_ring() {
        let fs = 10e6; // 10 MHz
        let c: f64 = 299_792_458.0;
        let range_m: f64 = 1000.0;
        let expected = ((2.0 * range_m * fs) / c).round() as usize;
        assert_eq!(range_ring_samples(range_m, fs), expected);

        // Zero range -> bin 0.
        assert_eq!(range_ring_samples(0.0, fs), 0);
    }

    #[test]
    fn test_empty_displays() {
        // A-scope from empty pulse.
        let a = AScopeData::from_pulse(&[], 1e6);
        assert!(a.amplitude.is_empty());
        assert!(a.range_bins.is_empty());

        // B-scope rasterize with no data.
        let bs = BScopeData::new(1000.0);
        let img = bs.as_image(10, 10);
        assert_eq!(img.len(), 10);
        for row in &img {
            assert!(row.iter().all(|&v| v == 0.0));
        }

        // PPI to_cartesian with no sweeps.
        let ppi = PpiData::new(1000.0);
        let grid = ppi.to_cartesian(8);
        assert_eq!(grid.len(), 8);
        for row in &grid {
            assert!(row.iter().all(|&v| v == 0.0));
        }

        // Zero-size outputs.
        let img0 = bs.as_image(0, 0);
        assert!(img0.is_empty());
        let grid0 = ppi.to_cartesian(0);
        assert!(grid0.is_empty());
    }

    #[test]
    fn test_single_sweep_ppi() {
        // Single sweep along positive x-axis (theta = 0).
        let mut ppi = PpiData::new(500.0);
        let samps: Vec<f64> = (0..128).map(|i| i as f64 / 127.0).collect();
        ppi.add_sweep(0.0, samps);

        let grid = ppi.to_cartesian(64);
        let mid = 64 / 2;

        // Pixel to the right of centre should have a positive value
        // (it lies near azimuth 0 at some range).
        assert!(grid[mid][mid + 10] > 0.0);

        // Pixel to the left of centre should also pick the single sweep
        // (closest azimuth is still 0, but it will be from the "wrong" side;
        // however with only one sweep it's the only option).
        // Just verify no panic and grid is populated.
        assert!(grid[mid][mid - 10] >= 0.0);
    }
}
