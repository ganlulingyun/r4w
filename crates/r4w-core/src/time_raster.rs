//! # Time Raster
//!
//! Maps a 1D sample stream into a 2D raster (matrix) for display.
//! Each row represents one time slice of configurable width.
//! Used for creating waterfall displays, eye diagrams, and
//! other time-vs-amplitude visualizations.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::time_raster::{TimeRaster, RasterData};
//!
//! let mut raster = TimeRaster::new(100, 50); // 100 cols x 50 rows
//! let samples: Vec<f64> = (0..500).map(|i| (i as f64 * 0.1).sin()).collect();
//! raster.feed(&samples);
//! let data = raster.get_raster();
//! assert_eq!(data.rows(), 5);
//! assert_eq!(data.cols(), 100);
//! ```

/// 2D raster data.
#[derive(Debug, Clone)]
pub struct RasterData {
    data: Vec<Vec<f64>>,
    cols: usize,
}

impl RasterData {
    /// Create empty raster.
    pub fn new(cols: usize) -> Self {
        Self {
            data: Vec::new(),
            cols,
        }
    }

    /// Number of rows.
    pub fn rows(&self) -> usize {
        self.data.len()
    }

    /// Number of columns.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Get a specific row.
    pub fn row(&self, index: usize) -> Option<&[f64]> {
        self.data.get(index).map(|r| r.as_slice())
    }

    /// Get a specific cell.
    pub fn get(&self, row: usize, col: usize) -> Option<f64> {
        self.data.get(row).and_then(|r| r.get(col)).copied()
    }

    /// Get all data as flat slice (row-major).
    pub fn as_flat(&self) -> Vec<f64> {
        self.data.iter().flatten().copied().collect()
    }

    /// Get min/max values across all cells.
    pub fn min_max(&self) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for row in &self.data {
            for &v in row {
                if v < min { min = v; }
                if v > max { max = v; }
            }
        }
        (min, max)
    }

    /// Add a row.
    pub fn push_row(&mut self, row: Vec<f64>) {
        self.data.push(row);
    }
}

/// Time raster: converts 1D stream to 2D display matrix.
#[derive(Debug, Clone)]
pub struct TimeRaster {
    /// Number of columns (samples per row).
    cols: usize,
    /// Maximum number of rows to keep.
    max_rows: usize,
    /// Accumulated rows.
    rows: Vec<Vec<f64>>,
    /// Partial row buffer.
    buffer: Vec<f64>,
    /// Total rows produced.
    total_rows: u64,
}

impl TimeRaster {
    /// Create a new time raster.
    ///
    /// * `cols` - Samples per row (width)
    /// * `max_rows` - Maximum rows to keep (height)
    pub fn new(cols: usize, max_rows: usize) -> Self {
        Self {
            cols: cols.max(1),
            max_rows: max_rows.max(1),
            rows: Vec::new(),
            buffer: Vec::new(),
            total_rows: 0,
        }
    }

    /// Feed samples into the raster.
    pub fn feed(&mut self, samples: &[f64]) {
        self.buffer.extend_from_slice(samples);
        while self.buffer.len() >= self.cols {
            let row: Vec<f64> = self.buffer.drain(..self.cols).collect();
            self.rows.push(row);
            self.total_rows += 1;
            if self.rows.len() > self.max_rows {
                self.rows.remove(0);
            }
        }
    }

    /// Feed with overlap (for symbol-sync eye diagrams).
    pub fn feed_overlapped(&mut self, samples: &[f64], stride: usize) {
        let stride = stride.max(1);
        let mut offset = 0;
        while offset + self.cols <= samples.len() {
            let row = samples[offset..offset + self.cols].to_vec();
            self.rows.push(row);
            self.total_rows += 1;
            if self.rows.len() > self.max_rows {
                self.rows.remove(0);
            }
            offset += stride;
        }
    }

    /// Get the current raster data.
    pub fn get_raster(&self) -> RasterData {
        let mut rd = RasterData::new(self.cols);
        for row in &self.rows {
            rd.push_row(row.clone());
        }
        rd
    }

    /// Get the most recent N rows.
    pub fn recent_rows(&self, n: usize) -> Vec<&[f64]> {
        let start = self.rows.len().saturating_sub(n);
        self.rows[start..].iter().map(|r| r.as_slice()).collect()
    }

    /// Get number of stored rows.
    pub fn num_rows(&self) -> usize {
        self.rows.len()
    }

    /// Get total rows produced.
    pub fn total_rows(&self) -> u64 {
        self.total_rows
    }

    /// Get column count.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Reset the raster.
    pub fn reset(&mut self) {
        self.rows.clear();
        self.buffer.clear();
        self.total_rows = 0;
    }
}

/// Normalize raster data to [0, 1] range.
pub fn normalize_raster(raster: &RasterData) -> RasterData {
    let (min, max) = raster.min_max();
    let range = max - min;
    let mut normalized = RasterData::new(raster.cols());
    for i in 0..raster.rows() {
        if let Some(row) = raster.row(i) {
            let norm_row: Vec<f64> = if range > 1e-30 {
                row.iter().map(|&v| (v - min) / range).collect()
            } else {
                vec![0.5; row.len()]
            };
            normalized.push_row(norm_row);
        }
    }
    normalized
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_raster() {
        let mut raster = TimeRaster::new(4, 10);
        raster.feed(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
        let data = raster.get_raster();
        assert_eq!(data.rows(), 2);
        assert_eq!(data.cols(), 4);
        assert_eq!(data.row(0), Some(&[1.0, 2.0, 3.0, 4.0][..]));
        assert_eq!(data.row(1), Some(&[5.0, 6.0, 7.0, 8.0][..]));
    }

    #[test]
    fn test_partial_row() {
        let mut raster = TimeRaster::new(4, 10);
        raster.feed(&[1.0, 2.0]); // Not enough for a row.
        assert_eq!(raster.num_rows(), 0);
        raster.feed(&[3.0, 4.0, 5.0]); // Now we have 5 → 1 row + 1 leftover.
        assert_eq!(raster.num_rows(), 1);
    }

    #[test]
    fn test_max_rows() {
        let mut raster = TimeRaster::new(2, 3);
        raster.feed(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]);
        assert_eq!(raster.num_rows(), 3); // Keeps only 3 most recent.
        let data = raster.get_raster();
        assert_eq!(data.row(0), Some(&[5.0, 6.0][..])); // Oldest kept.
    }

    #[test]
    fn test_overlapped_feed() {
        let mut raster = TimeRaster::new(4, 10);
        let samples = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        raster.feed_overlapped(&samples, 2);
        // Row 0: [1,2,3,4], Row 1: [3,4,5,6] (stride=2, overlap=2).
        assert_eq!(raster.num_rows(), 2);
        let data = raster.get_raster();
        assert_eq!(data.row(0), Some(&[1.0, 2.0, 3.0, 4.0][..]));
        assert_eq!(data.row(1), Some(&[3.0, 4.0, 5.0, 6.0][..]));
    }

    #[test]
    fn test_raster_data_accessors() {
        let mut rd = RasterData::new(3);
        rd.push_row(vec![1.0, 2.0, 3.0]);
        rd.push_row(vec![4.0, 5.0, 6.0]);
        assert_eq!(rd.get(0, 1), Some(2.0));
        assert_eq!(rd.get(1, 2), Some(6.0));
        assert_eq!(rd.get(2, 0), None);
    }

    #[test]
    fn test_min_max() {
        let mut rd = RasterData::new(2);
        rd.push_row(vec![-3.0, 5.0]);
        rd.push_row(vec![1.0, -1.0]);
        let (min, max) = rd.min_max();
        assert_eq!(min, -3.0);
        assert_eq!(max, 5.0);
    }

    #[test]
    fn test_normalize() {
        let mut rd = RasterData::new(2);
        rd.push_row(vec![0.0, 10.0]);
        rd.push_row(vec![5.0, 10.0]);
        let norm = normalize_raster(&rd);
        assert!((norm.get(0, 0).unwrap() - 0.0).abs() < 1e-10);
        assert!((norm.get(0, 1).unwrap() - 1.0).abs() < 1e-10);
        assert!((norm.get(1, 0).unwrap() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_recent_rows() {
        let mut raster = TimeRaster::new(2, 10);
        raster.feed(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let recent = raster.recent_rows(2);
        assert_eq!(recent.len(), 2);
        assert_eq!(recent[0], &[3.0, 4.0]);
        assert_eq!(recent[1], &[5.0, 6.0]);
    }

    #[test]
    fn test_as_flat() {
        let mut rd = RasterData::new(2);
        rd.push_row(vec![1.0, 2.0]);
        rd.push_row(vec![3.0, 4.0]);
        assert_eq!(rd.as_flat(), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_reset() {
        let mut raster = TimeRaster::new(4, 10);
        raster.feed(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(raster.num_rows(), 1);
        raster.reset();
        assert_eq!(raster.num_rows(), 0);
        assert_eq!(raster.total_rows(), 0);
    }
}
