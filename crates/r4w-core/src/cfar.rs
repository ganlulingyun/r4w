//! CFAR — Constant False Alarm Rate Detector
//!
//! Sliding-window detection algorithm that adaptively sets detection
//! thresholds based on local noise estimates. Maintains a constant
//! false-alarm probability across varying noise floors. Essential for
//! radar, spectrum sensing, and blind signal detection.
//! GNU Radio equivalent: gr-radar CFAR blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cfar::{CfarDetector, CfarAlgorithm};
//!
//! let mut detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 4, 8, 5.0);
//! let power: Vec<f64> = vec![0.1; 50];
//! let detections = detector.detect(&power);
//! assert!(detections.iter().all(|&d| !d)); // No detections in flat noise
//! ```

use std::collections::BinaryHeap;
use std::cmp::Reverse;

/// CFAR algorithm variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CfarAlgorithm {
    /// Cell-Averaging CFAR — averages all reference cells.
    /// Good for homogeneous clutter.
    CellAveraging,
    /// Greatest-Of CFAR — uses the greater of leading/lagging averages.
    /// Robust at clutter edges.
    GreatestOf,
    /// Smallest-Of CFAR — uses the lesser of leading/lagging averages.
    /// Better detection in clutter transitions but higher false-alarm rate.
    SmallestOf,
    /// Ordered-Statistics CFAR — selects the k-th ranked reference cell.
    /// Robust to interfering targets in the reference window.
    OrderedStatistics { rank: usize },
}

/// Detection result with index and estimated SNR.
#[derive(Debug, Clone, Copy)]
pub struct CfarDetection {
    /// Index of the detected cell.
    pub index: usize,
    /// Power level of the cell under test.
    pub power: f64,
    /// Estimated noise level from reference cells.
    pub noise_estimate: f64,
    /// Test statistic (power / noise_estimate).
    pub test_statistic: f64,
}

/// Constant False Alarm Rate detector.
#[derive(Debug, Clone)]
pub struct CfarDetector {
    algorithm: CfarAlgorithm,
    guard_cells: usize,
    reference_cells: usize,
    threshold_factor: f64,
}

impl CfarDetector {
    /// Create a new CFAR detector.
    ///
    /// * `algorithm` — CFAR variant
    /// * `guard_cells` — number of guard cells on each side of the CUT
    /// * `reference_cells` — number of reference cells on each side
    /// * `threshold_factor` — multiplier on noise estimate (higher = fewer false alarms)
    pub fn new(
        algorithm: CfarAlgorithm,
        guard_cells: usize,
        reference_cells: usize,
        threshold_factor: f64,
    ) -> Self {
        assert!(reference_cells > 0, "Must have at least 1 reference cell per side");
        assert!(threshold_factor > 0.0, "Threshold factor must be positive");
        Self {
            algorithm,
            guard_cells,
            reference_cells,
            threshold_factor,
        }
    }

    /// Total window half-width (guard + reference).
    fn half_window(&self) -> usize {
        self.guard_cells + self.reference_cells
    }

    /// Estimate the noise level for cell at `index` using reference cells.
    fn estimate_noise(&self, power: &[f64], index: usize) -> f64 {
        let n = power.len();
        let hw = self.half_window();

        // Collect leading reference cells
        let mut leading = Vec::new();
        for i in 1..=self.reference_cells {
            let offset = self.guard_cells + i;
            if index >= offset {
                leading.push(power[index - offset]);
            }
        }

        // Collect lagging reference cells
        let mut lagging = Vec::new();
        for i in 1..=self.reference_cells {
            let offset = self.guard_cells + i;
            if index + offset < n {
                lagging.push(power[index + offset]);
            }
        }

        match self.algorithm {
            CfarAlgorithm::CellAveraging => {
                let all: Vec<f64> = leading.iter().chain(lagging.iter()).copied().collect();
                if all.is_empty() {
                    return f64::MAX;
                }
                all.iter().sum::<f64>() / all.len() as f64
            }
            CfarAlgorithm::GreatestOf => {
                let lead_avg = if leading.is_empty() {
                    0.0
                } else {
                    leading.iter().sum::<f64>() / leading.len() as f64
                };
                let lag_avg = if lagging.is_empty() {
                    0.0
                } else {
                    lagging.iter().sum::<f64>() / lagging.len() as f64
                };
                if leading.is_empty() && lagging.is_empty() {
                    return f64::MAX;
                }
                lead_avg.max(lag_avg)
            }
            CfarAlgorithm::SmallestOf => {
                let lead_avg = if leading.is_empty() {
                    f64::MAX
                } else {
                    leading.iter().sum::<f64>() / leading.len() as f64
                };
                let lag_avg = if lagging.is_empty() {
                    f64::MAX
                } else {
                    lagging.iter().sum::<f64>() / lagging.len() as f64
                };
                lead_avg.min(lag_avg)
            }
            CfarAlgorithm::OrderedStatistics { rank } => {
                let mut all: Vec<f64> = leading.iter().chain(lagging.iter()).copied().collect();
                if all.is_empty() {
                    return f64::MAX;
                }
                all.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let k = rank.min(all.len() - 1);
                all[k]
            }
        }
    }

    /// Run detection on power-domain input.
    ///
    /// Returns a boolean vector: `true` where a detection occurred.
    pub fn detect(&self, power: &[f64]) -> Vec<bool> {
        let n = power.len();
        let mut result = vec![false; n];

        for i in 0..n {
            let noise = self.estimate_noise(power, i);
            let threshold = noise * self.threshold_factor;
            result[i] = power[i] > threshold;
        }

        result
    }

    /// Run detection and return detailed results for each detected cell.
    pub fn detect_detailed(&self, power: &[f64]) -> Vec<CfarDetection> {
        let n = power.len();
        let mut detections = Vec::new();

        for i in 0..n {
            let noise = self.estimate_noise(power, i);
            let threshold = noise * self.threshold_factor;
            if power[i] > threshold && noise > 0.0 {
                detections.push(CfarDetection {
                    index: i,
                    power: power[i],
                    noise_estimate: noise,
                    test_statistic: power[i] / noise,
                });
            }
        }

        detections
    }

    /// Set threshold factor.
    pub fn set_threshold_factor(&mut self, factor: f64) {
        assert!(factor > 0.0);
        self.threshold_factor = factor;
    }

    /// Get threshold factor.
    pub fn threshold_factor(&self) -> f64 {
        self.threshold_factor
    }

    /// Get algorithm.
    pub fn algorithm(&self) -> CfarAlgorithm {
        self.algorithm
    }

    /// Get guard cells (per side).
    pub fn guard_cells(&self) -> usize {
        self.guard_cells
    }

    /// Get reference cells (per side).
    pub fn reference_cells(&self) -> usize {
        self.reference_cells
    }
}

/// 2-D CFAR detector for range-Doppler maps.
pub struct Cfar2D {
    guard_rows: usize,
    guard_cols: usize,
    ref_rows: usize,
    ref_cols: usize,
    threshold_factor: f64,
}

impl Cfar2D {
    /// Create a 2-D CFAR detector.
    pub fn new(
        guard_rows: usize,
        guard_cols: usize,
        ref_rows: usize,
        ref_cols: usize,
        threshold_factor: f64,
    ) -> Self {
        Self {
            guard_rows,
            guard_cols,
            ref_rows,
            ref_cols,
            threshold_factor,
        }
    }

    /// Detect targets in a 2-D power map (row-major).
    ///
    /// `rows` x `cols` grid in `power`.
    pub fn detect(&self, power: &[f64], rows: usize, cols: usize) -> Vec<(usize, usize)> {
        assert_eq!(power.len(), rows * cols);
        let mut detections = Vec::new();

        for r in 0..rows {
            for c in 0..cols {
                let noise = self.estimate_noise_2d(power, rows, cols, r, c);
                if power[r * cols + c] > noise * self.threshold_factor {
                    detections.push((r, c));
                }
            }
        }

        detections
    }

    fn estimate_noise_2d(
        &self,
        power: &[f64],
        rows: usize,
        cols: usize,
        row: usize,
        col: usize,
    ) -> f64 {
        let mut sum = 0.0;
        let mut count = 0usize;

        let r_start = row.saturating_sub(self.guard_rows + self.ref_rows);
        let r_end = (row + self.guard_rows + self.ref_rows + 1).min(rows);
        let c_start = col.saturating_sub(self.guard_cols + self.ref_cols);
        let c_end = (col + self.guard_cols + self.ref_cols + 1).min(cols);

        for r in r_start..r_end {
            for c in c_start..c_end {
                // Skip guard region and CUT
                let dr = if r > row { r - row } else { row - r };
                let dc = if c > col { c - col } else { col - c };
                if dr <= self.guard_rows && dc <= self.guard_cols {
                    continue;
                }
                sum += power[r * cols + c];
                count += 1;
            }
        }

        if count == 0 {
            f64::MAX
        } else {
            sum / count as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ca_cfar_no_detection_in_flat_noise() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 2, 4, 3.0);
        let power = vec![1.0; 50];
        let detections = detector.detect(&power);
        // Flat noise: threshold = 3 * 1.0 = 3.0, all cells are 1.0 → no detections
        assert!(detections.iter().all(|&d| !d));
    }

    #[test]
    fn test_ca_cfar_detects_strong_signal() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 2, 4, 3.0);
        let mut power = vec![1.0; 50];
        power[25] = 10.0; // Strong target
        let detections = detector.detect(&power);
        assert!(detections[25], "Should detect strong signal");
    }

    #[test]
    fn test_go_cfar_clutter_edge() {
        let detector = CfarDetector::new(CfarAlgorithm::GreatestOf, 2, 4, 3.0);
        let mut power = vec![0.0; 50];
        // Step clutter: low on left, high on right
        for i in 25..50 {
            power[i] = 5.0;
        }
        // GO-CFAR should use the greater average, so target at clutter edge
        // should not false-alarm as easily
        let detections = detector.detect(&power);
        // In the transition region, GO-CFAR should be conservative
        // (fewer false alarms than CA-CFAR at clutter edges)
        let false_alarms_near_edge: usize = detections[20..30]
            .iter()
            .filter(|&&d| d)
            .count();
        // Check that GO-CFAR is not triggering everywhere
        assert!(false_alarms_near_edge < 10);
    }

    #[test]
    fn test_so_cfar_detects_in_transition() {
        let detector = CfarDetector::new(CfarAlgorithm::SmallestOf, 2, 4, 2.0);
        let mut power = vec![1.0; 50];
        power[25] = 10.0;
        let detections = detector.detect(&power);
        assert!(detections[25]);
    }

    #[test]
    fn test_os_cfar_robust_to_interferers() {
        let detector = CfarDetector::new(
            CfarAlgorithm::OrderedStatistics { rank: 6 },
            2,
            8,
            3.0,
        );
        let mut power = vec![1.0; 50];
        power[25] = 20.0; // Target
        // Place interferers in the reference window
        power[20] = 15.0;
        power[30] = 15.0;
        let detections = detector.detect(&power);
        // OS-CFAR should still detect the main target despite interferers
        assert!(detections[25], "OS-CFAR should detect target despite interferers");
    }

    #[test]
    fn test_cfar_detailed_results() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 2, 4, 3.0);
        let mut power = vec![1.0; 50];
        power[25] = 10.0;
        let detections = detector.detect_detailed(&power);
        let target = detections.iter().find(|d| d.index == 25);
        assert!(target.is_some(), "Should detect at index 25");
        let t = target.unwrap();
        assert!(t.test_statistic > 3.0, "Test statistic should exceed threshold");
        assert!((t.noise_estimate - 1.0).abs() < 0.5, "Noise estimate should be ~1.0");
    }

    #[test]
    fn test_cfar_multiple_targets() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 2, 4, 3.0);
        let mut power = vec![1.0; 100];
        power[30] = 15.0;
        power[70] = 12.0;
        let detections = detector.detect(&power);
        assert!(detections[30]);
        assert!(detections[70]);
    }

    #[test]
    fn test_cfar_threshold_factor() {
        let mut detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 2, 4, 2.0);
        let mut power = vec![1.0; 50];
        power[25] = 3.5;
        // With factor 2.0: threshold = 2.0, 3.5 > 2.0 → detect
        assert!(detector.detect(&power)[25]);
        // With factor 5.0: threshold = 5.0, 3.5 < 5.0 → no detect
        detector.set_threshold_factor(5.0);
        assert!(!detector.detect(&power)[25]);
    }

    #[test]
    fn test_cfar_short_input() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 1, 2, 3.0);
        let power = vec![1.0, 1.0, 10.0, 1.0, 1.0];
        let detections = detector.detect(&power);
        assert!(detections[2], "Should detect center peak even in short input");
    }

    #[test]
    fn test_2d_cfar_no_targets() {
        let detector = Cfar2D::new(1, 1, 2, 2, 3.0);
        let power = vec![1.0; 100]; // 10x10 flat
        let detections = detector.detect(&power, 10, 10);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_2d_cfar_single_target() {
        let detector = Cfar2D::new(1, 1, 2, 2, 3.0);
        let mut power = vec![1.0; 100]; // 10x10
        power[5 * 10 + 5] = 10.0; // Target at (5,5)
        let detections = detector.detect(&power, 10, 10);
        assert!(detections.contains(&(5, 5)), "Should detect target at (5,5)");
    }

    #[test]
    fn test_cfar_accessors() {
        let detector = CfarDetector::new(CfarAlgorithm::CellAveraging, 3, 6, 4.5);
        assert_eq!(detector.guard_cells(), 3);
        assert_eq!(detector.reference_cells(), 6);
        assert!((detector.threshold_factor() - 4.5).abs() < 1e-10);
        assert_eq!(detector.algorithm(), CfarAlgorithm::CellAveraging);
    }
}
