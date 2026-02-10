//! 2-D CFAR -- Constant False Alarm Rate Detection for Range-Doppler Maps
//!
//! Two-dimensional sliding-window detection algorithm that adaptively sets
//! detection thresholds based on local noise estimates in both range and
//! Doppler dimensions. Maintains a constant false-alarm probability across
//! varying noise floors in range-Doppler maps produced by pulsed radar
//! signal processors.
//!
//! Supports four CFAR variants:
//! - **CA-CFAR** (Cell-Averaging): averages all training cells in the 2-D window.
//! - **GO-CFAR** (Greatest-Of): uses the greater of range-leading/lagging averages.
//! - **SO-CFAR** (Smallest-Of): uses the lesser -- better sensitivity, higher Pfa.
//! - **OS-CFAR** (Ordered-Statistics): selects the k-th ranked training cell value;
//!   robust to interfering targets in the reference window.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cfar_2d::{Cfar2d, CfarType2d, Detection2d};
//!
//! let cfar = Cfar2d::new(1, 1, 3, 3, 4.0, CfarType2d::CaCell);
//! // 16x16 range-Doppler map with uniform noise
//! let mut rdm = vec![vec![1.0_f64; 16]; 16];
//! // Inject a target at (8, 8)
//! rdm[8][8] = 20.0;
//! let detections = cfar.detect(&rdm);
//! assert!(detections.iter().any(|d| d.range_bin == 8 && d.doppler_bin == 8));
//! ```

/// CFAR algorithm variant for 2-D detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CfarType2d {
    /// Cell-Averaging CFAR -- averages all training cells in the 2-D annulus.
    /// Best for homogeneous clutter.
    CaCell,
    /// Greatest-Of CFAR -- takes the maximum of leading/lagging noise
    /// estimates along the range dimension.  Robust at clutter edges.
    GoCell,
    /// Smallest-Of CFAR -- takes the minimum of leading/lagging noise
    /// estimates along the range dimension.  Better sensitivity but higher
    /// false-alarm rate at clutter boundaries.
    SoCell,
    /// Ordered-Statistics CFAR -- selects the `rank`-th smallest training
    /// cell value.  Robust to interfering targets in the reference window.
    OsCfar { rank: usize },
}

/// A single 2-D CFAR detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection2d {
    /// Range bin index of the detection.
    pub range_bin: usize,
    /// Doppler bin index of the detection.
    pub doppler_bin: usize,
    /// Power level of the cell under test.
    pub power: f64,
    /// CFAR threshold that was exceeded.
    pub threshold: f64,
    /// Estimated SNR in dB: 10 * log10(power / noise).
    pub snr_db: f64,
}

/// Two-dimensional CFAR detector for range-Doppler maps.
///
/// The detector slides a rectangular window over the map.  Around each
/// cell-under-test (CUT) there is a guard region that excludes target
/// leakage, surrounded by training cells used to estimate the local noise
/// floor.
#[derive(Debug, Clone)]
pub struct Cfar2d {
    /// Guard cells on each side of the CUT in the range dimension.
    pub guard_cells_range: usize,
    /// Guard cells on each side of the CUT in the Doppler dimension.
    pub guard_cells_doppler: usize,
    /// Training cells on each side of the CUT in the range dimension.
    pub training_cells_range: usize,
    /// Training cells on each side of the CUT in the Doppler dimension.
    pub training_cells_doppler: usize,
    /// Multiplicative threshold factor applied to the noise estimate.
    pub threshold_factor: f64,
    /// CFAR algorithm variant.
    pub cfar_type: CfarType2d,
}

impl Cfar2d {
    /// Create a new 2-D CFAR detector.
    ///
    /// * `guard_range`   -- guard cells per side in range
    /// * `guard_doppler` -- guard cells per side in Doppler
    /// * `train_range`   -- training cells per side in range
    /// * `train_doppler` -- training cells per side in Doppler
    /// * `threshold_factor` -- multiplier on the noise estimate
    /// * `cfar_type`     -- algorithm variant
    pub fn new(
        guard_range: usize,
        guard_doppler: usize,
        train_range: usize,
        train_doppler: usize,
        threshold_factor: f64,
        cfar_type: CfarType2d,
    ) -> Self {
        assert!(train_range > 0, "Must have at least 1 training cell in range");
        assert!(train_doppler > 0, "Must have at least 1 training cell in Doppler");
        assert!(threshold_factor > 0.0, "Threshold factor must be positive");
        Self {
            guard_cells_range: guard_range,
            guard_cells_doppler: guard_doppler,
            training_cells_range: train_range,
            training_cells_doppler: train_doppler,
            threshold_factor,
            cfar_type,
        }
    }

    /// Estimate the local noise level at a given cell using the training
    /// cells surrounding the guard region.
    ///
    /// The range-Doppler map `rdm` is indexed as `rdm[range_bin][doppler_bin]`.
    pub fn estimate_noise(
        &self,
        rdm: &[Vec<f64>],
        range_bin: usize,
        doppler_bin: usize,
    ) -> f64 {
        if rdm.is_empty() || rdm[0].is_empty() {
            return f64::MAX;
        }
        let num_range = rdm.len();
        let num_doppler = rdm[0].len();

        let outer_r = self.guard_cells_range + self.training_cells_range;
        let outer_d = self.guard_cells_doppler + self.training_cells_doppler;

        let r_lo = range_bin.saturating_sub(outer_r);
        let r_hi = (range_bin + outer_r + 1).min(num_range);
        let d_lo = doppler_bin.saturating_sub(outer_d);
        let d_hi = (doppler_bin + outer_d + 1).min(num_doppler);

        match self.cfar_type {
            CfarType2d::CaCell => {
                let mut sum = 0.0;
                let mut count = 0usize;
                for r in r_lo..r_hi {
                    for d in d_lo..d_hi {
                        if self.is_in_guard_or_cut(r, d, range_bin, doppler_bin) {
                            continue;
                        }
                        sum += rdm[r][d];
                        count += 1;
                    }
                }
                if count == 0 { f64::MAX } else { sum / count as f64 }
            }
            CfarType2d::GoCell => {
                // Leading = range bins below CUT, lagging = range bins above CUT
                let (lead_sum, lead_count) =
                    self.half_sum(rdm, range_bin, doppler_bin, r_lo, range_bin, d_lo, d_hi);
                let (lag_sum, lag_count) = self.half_sum(
                    rdm,
                    range_bin,
                    doppler_bin,
                    range_bin + 1,
                    r_hi,
                    d_lo,
                    d_hi,
                );
                let lead_avg = if lead_count == 0 { 0.0 } else { lead_sum / lead_count as f64 };
                let lag_avg = if lag_count == 0 { 0.0 } else { lag_sum / lag_count as f64 };
                if lead_count == 0 && lag_count == 0 {
                    f64::MAX
                } else {
                    lead_avg.max(lag_avg)
                }
            }
            CfarType2d::SoCell => {
                let (lead_sum, lead_count) =
                    self.half_sum(rdm, range_bin, doppler_bin, r_lo, range_bin, d_lo, d_hi);
                let (lag_sum, lag_count) = self.half_sum(
                    rdm,
                    range_bin,
                    doppler_bin,
                    range_bin + 1,
                    r_hi,
                    d_lo,
                    d_hi,
                );
                let lead_avg =
                    if lead_count == 0 { f64::MAX } else { lead_sum / lead_count as f64 };
                let lag_avg =
                    if lag_count == 0 { f64::MAX } else { lag_sum / lag_count as f64 };
                lead_avg.min(lag_avg)
            }
            CfarType2d::OsCfar { rank } => {
                let mut vals = Vec::new();
                for r in r_lo..r_hi {
                    for d in d_lo..d_hi {
                        if self.is_in_guard_or_cut(r, d, range_bin, doppler_bin) {
                            continue;
                        }
                        vals.push(rdm[r][d]);
                    }
                }
                if vals.is_empty() {
                    return f64::MAX;
                }
                vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let k = rank.min(vals.len() - 1);
                vals[k]
            }
        }
    }

    /// Detect all cells that exceed the CFAR threshold.
    ///
    /// Returns a list of [`Detection2d`] entries sorted by descending power.
    pub fn detect(&self, rdm: &[Vec<f64>]) -> Vec<Detection2d> {
        if rdm.is_empty() {
            return Vec::new();
        }
        let num_range = rdm.len();
        let num_doppler = rdm[0].len();
        let mut detections = Vec::new();

        for r in 0..num_range {
            for d in 0..num_doppler {
                let noise = self.estimate_noise(rdm, r, d);
                let thresh = noise * self.threshold_factor;
                let power = rdm[r][d];
                if power > thresh && noise < f64::MAX {
                    let snr_db = if noise > 0.0 {
                        10.0 * (power / noise).log10()
                    } else {
                        f64::INFINITY
                    };
                    detections.push(Detection2d {
                        range_bin: r,
                        doppler_bin: d,
                        power,
                        threshold: thresh,
                        snr_db,
                    });
                }
            }
        }

        detections.sort_by(|a, b| {
            b.power
                .partial_cmp(&a.power)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        detections
    }

    /// Compute the CFAR threshold for every cell in the map.
    ///
    /// Returns a 2-D grid (same dimensions as input) where each element
    /// is the threshold that the corresponding cell must exceed for detection.
    pub fn threshold_map(&self, rdm: &[Vec<f64>]) -> Vec<Vec<f64>> {
        if rdm.is_empty() {
            return Vec::new();
        }
        let num_range = rdm.len();
        let num_doppler = rdm[0].len();
        let mut tmap = vec![vec![0.0; num_doppler]; num_range];

        for r in 0..num_range {
            for d in 0..num_doppler {
                let noise = self.estimate_noise(rdm, r, d);
                tmap[r][d] = noise * self.threshold_factor;
            }
        }
        tmap
    }

    /// Produce a binary detection map.
    ///
    /// `true` at each cell where the power exceeds the CFAR threshold.
    pub fn detection_map(&self, rdm: &[Vec<f64>]) -> Vec<Vec<bool>> {
        if rdm.is_empty() {
            return Vec::new();
        }
        let num_range = rdm.len();
        let num_doppler = rdm[0].len();
        let mut dmap = vec![vec![false; num_doppler]; num_range];

        for r in 0..num_range {
            for d in 0..num_doppler {
                let noise = self.estimate_noise(rdm, r, d);
                let thresh = noise * self.threshold_factor;
                dmap[r][d] = rdm[r][d] > thresh && noise < f64::MAX;
            }
        }
        dmap
    }

    // ---- internal helpers ------------------------------------------------

    /// Returns true if `(r, d)` is inside the guard region or is the CUT.
    fn is_in_guard_or_cut(
        &self,
        r: usize,
        d: usize,
        cut_r: usize,
        cut_d: usize,
    ) -> bool {
        let dr = if r > cut_r { r - cut_r } else { cut_r - r };
        let dd = if d > cut_d { d - cut_d } else { cut_d - d };
        dr <= self.guard_cells_range && dd <= self.guard_cells_doppler
    }

    /// Sum training cells in a rectangular sub-region, excluding guard/CUT.
    fn half_sum(
        &self,
        rdm: &[Vec<f64>],
        cut_r: usize,
        cut_d: usize,
        r_lo: usize,
        r_hi: usize,
        d_lo: usize,
        d_hi: usize,
    ) -> (f64, usize) {
        let mut sum = 0.0;
        let mut count = 0usize;
        for r in r_lo..r_hi {
            for d in d_lo..d_hi {
                if self.is_in_guard_or_cut(r, d, cut_r, cut_d) {
                    continue;
                }
                sum += rdm[r][d];
                count += 1;
            }
        }
        (sum, count)
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Merge adjacent detections into clusters.
///
/// Detections whose range bins differ by at most `range_merge` and whose
/// Doppler bins differ by at most `doppler_merge` are grouped.  Each cluster
/// is represented by the detection with the highest power, with `snr_db`
/// recomputed as a weighted average.
pub fn cluster_detections(
    detections: &[Detection2d],
    range_merge: usize,
    doppler_merge: usize,
) -> Vec<Detection2d> {
    if detections.is_empty() {
        return Vec::new();
    }

    // Union-find style: assign each detection to a cluster id
    let n = detections.len();
    let mut cluster_id: Vec<usize> = (0..n).collect();

    // Find root with path compression
    fn find(ids: &mut [usize], mut i: usize) -> usize {
        while ids[i] != i {
            ids[i] = ids[ids[i]];
            i = ids[i];
        }
        i
    }

    // Union detections that are within merge distance
    for i in 0..n {
        for j in (i + 1)..n {
            let dr = if detections[i].range_bin > detections[j].range_bin {
                detections[i].range_bin - detections[j].range_bin
            } else {
                detections[j].range_bin - detections[i].range_bin
            };
            let dd = if detections[i].doppler_bin > detections[j].doppler_bin {
                detections[i].doppler_bin - detections[j].doppler_bin
            } else {
                detections[j].doppler_bin - detections[i].doppler_bin
            };
            if dr <= range_merge && dd <= doppler_merge {
                let ri = find(&mut cluster_id, i);
                let rj = find(&mut cluster_id, j);
                if ri != rj {
                    cluster_id[ri] = rj;
                }
            }
        }
    }

    // Group by cluster
    let mut groups: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for i in 0..n {
        let root = find(&mut cluster_id, i);
        groups.entry(root).or_default().push(i);
    }

    // For each cluster, pick the peak-power detection
    let mut result: Vec<Detection2d> = groups
        .values()
        .map(|members| {
            let peak_idx = *members
                .iter()
                .max_by(|&&a, &&b| {
                    detections[a]
                        .power
                        .partial_cmp(&detections[b].power)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();
            detections[peak_idx]
        })
        .collect();

    result.sort_by(|a, b| {
        b.power
            .partial_cmp(&a.power)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    result
}

/// Probability of false alarm for CA-CFAR given `threshold_factor` (alpha)
/// and the number of training cells `N`.
///
/// For CA-CFAR with `N` independent training cells the Pfa is:
///
///   Pfa = (1 + alpha / N) ^ (-N)
///
/// This is exact for exponentially distributed (power-detected) noise.
pub fn probability_of_false_alarm(threshold_factor: f64, num_training_cells: usize) -> f64 {
    let n = num_training_cells as f64;
    (1.0 + threshold_factor / n).powf(-n)
}

/// Compute the threshold factor (alpha) required to achieve a desired
/// probability of false alarm `pfa` with `num_training_cells` training cells.
///
/// Inverse of [`probability_of_false_alarm`]:
///
///   alpha = N * (Pfa ^ (-1/N) - 1)
pub fn threshold_from_pfa(pfa: f64, num_training_cells: usize) -> f64 {
    assert!(pfa > 0.0 && pfa < 1.0, "Pfa must be in (0, 1)");
    assert!(num_training_cells > 0, "Must have at least 1 training cell");
    let n = num_training_cells as f64;
    n * (pfa.powf(-1.0 / n) - 1.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a uniform-noise range-Doppler map.
    fn uniform_rdm(rows: usize, cols: usize, value: f64) -> Vec<Vec<f64>> {
        vec![vec![value; cols]; rows]
    }

    #[test]
    fn test_single_target() {
        let cfar = Cfar2d::new(1, 1, 3, 3, 4.0, CfarType2d::CaCell);
        let mut rdm = uniform_rdm(20, 20, 1.0);
        rdm[10][10] = 30.0;
        let dets = cfar.detect(&rdm);
        assert!(!dets.is_empty(), "Should detect the injected target");
        let hit = dets.iter().find(|d| d.range_bin == 10 && d.doppler_bin == 10);
        assert!(hit.is_some(), "Detection should be at (10, 10)");
        let h = hit.unwrap();
        assert!(h.snr_db > 10.0, "SNR should be substantial");
        assert!(h.power > h.threshold, "Power must exceed threshold");
    }

    #[test]
    fn test_no_target() {
        let cfar = Cfar2d::new(1, 1, 3, 3, 4.0, CfarType2d::CaCell);
        let rdm = uniform_rdm(20, 20, 1.0);
        let dets = cfar.detect(&rdm);
        assert!(dets.is_empty(), "Flat noise should produce no detections");
    }

    #[test]
    fn test_ca_cfar() {
        // CA-CFAR averages all training cells equally
        let cfar = Cfar2d::new(1, 1, 2, 2, 3.0, CfarType2d::CaCell);
        let mut rdm = uniform_rdm(16, 16, 1.0);
        rdm[8][8] = 10.0;
        let noise = cfar.estimate_noise(&rdm, 8, 8);
        // Noise estimate should be close to 1.0 (the background level)
        assert!(
            (noise - 1.0).abs() < 0.2,
            "CA noise estimate should be ~1.0, got {}",
            noise
        );
        let dets = cfar.detect(&rdm);
        assert!(
            dets.iter().any(|d| d.range_bin == 8 && d.doppler_bin == 8),
            "CA-CFAR should detect target at (8,8)"
        );
    }

    #[test]
    fn test_go_cfar() {
        // GO-CFAR uses the greater of leading/lagging -- more conservative
        let cfar = Cfar2d::new(1, 1, 2, 2, 3.0, CfarType2d::GoCell);
        let mut rdm = uniform_rdm(16, 16, 1.0);
        // Elevated clutter on one side (range bins 0..7)
        for r in 0..7 {
            for d in 0..16 {
                rdm[r][d] = 5.0;
            }
        }
        // Target right at the clutter edge
        rdm[8][8] = 12.0;
        let noise = cfar.estimate_noise(&rdm, 8, 8);
        // GO-CFAR takes the max half-average, so noise > 1.0
        assert!(noise > 1.0, "GO-CFAR noise should reflect higher clutter side");
    }

    #[test]
    fn test_so_cfar() {
        // SO-CFAR uses the lesser of leading/lagging -- more sensitive
        let cfar = Cfar2d::new(1, 1, 2, 2, 3.0, CfarType2d::SoCell);
        let mut rdm = uniform_rdm(16, 16, 1.0);
        // Elevated clutter on one side only
        for r in 0..7 {
            for d in 0..16 {
                rdm[r][d] = 5.0;
            }
        }
        rdm[8][8] = 8.0;
        let noise = cfar.estimate_noise(&rdm, 8, 8);
        // SO-CFAR takes the min half-average, so noise should be closer to 1.0
        assert!(
            noise < 3.0,
            "SO-CFAR noise should be driven by the lower-power side, got {}",
            noise
        );
        let dets = cfar.detect(&rdm);
        assert!(
            dets.iter().any(|d| d.range_bin == 8 && d.doppler_bin == 8),
            "SO-CFAR should detect target at clutter boundary"
        );
    }

    #[test]
    fn test_threshold_map() {
        let cfar = Cfar2d::new(1, 1, 2, 2, 5.0, CfarType2d::CaCell);
        let rdm = uniform_rdm(10, 10, 2.0);
        let tmap = cfar.threshold_map(&rdm);
        assert_eq!(tmap.len(), 10);
        assert_eq!(tmap[0].len(), 10);
        // In the interior, noise estimate = 2.0, threshold = 5.0 * 2.0 = 10.0
        let t = tmap[5][5];
        assert!(
            (t - 10.0).abs() < 1.0,
            "Interior threshold should be ~10.0, got {}",
            t
        );
    }

    #[test]
    fn test_detection_map() {
        let cfar = Cfar2d::new(1, 1, 2, 2, 4.0, CfarType2d::CaCell);
        let mut rdm = uniform_rdm(12, 12, 1.0);
        rdm[6][6] = 20.0;
        let dmap = cfar.detection_map(&rdm);
        assert_eq!(dmap.len(), 12);
        assert_eq!(dmap[0].len(), 12);
        assert!(dmap[6][6], "Target cell should be detected");
        // Most non-target cells should be false
        let total_true: usize = dmap.iter().flat_map(|row| row.iter()).filter(|&&v| v).count();
        assert!(total_true < 5, "Very few cells should be detected, got {}", total_true);
    }

    #[test]
    fn test_clustering() {
        // Two clusters of detections
        let detections = vec![
            Detection2d { range_bin: 5, doppler_bin: 5, power: 20.0, threshold: 4.0, snr_db: 7.0 },
            Detection2d { range_bin: 5, doppler_bin: 6, power: 18.0, threshold: 4.0, snr_db: 6.5 },
            Detection2d { range_bin: 6, doppler_bin: 5, power: 15.0, threshold: 4.0, snr_db: 5.7 },
            // Separate cluster far away
            Detection2d { range_bin: 15, doppler_bin: 15, power: 25.0, threshold: 4.0, snr_db: 8.0 },
            Detection2d { range_bin: 15, doppler_bin: 16, power: 22.0, threshold: 4.0, snr_db: 7.4 },
        ];
        let clustered = cluster_detections(&detections, 2, 2);
        assert_eq!(clustered.len(), 2, "Should merge into 2 clusters");
        // First cluster (highest power) should be at (15,15)
        assert_eq!(clustered[0].range_bin, 15);
        assert_eq!(clustered[0].doppler_bin, 15);
        // Second cluster at (5,5)
        assert_eq!(clustered[1].range_bin, 5);
        assert_eq!(clustered[1].doppler_bin, 5);
    }

    #[test]
    fn test_pfa_calculation() {
        // Round-trip: threshold -> Pfa -> threshold
        let n = 48; // typical 2-D training cell count
        let alpha = 6.0;
        let pfa = probability_of_false_alarm(alpha, n);
        assert!(pfa > 0.0 && pfa < 1.0, "Pfa should be in (0,1), got {}", pfa);

        let recovered_alpha = threshold_from_pfa(pfa, n);
        assert!(
            (recovered_alpha - alpha).abs() < 1e-6,
            "Round-trip threshold: expected {}, got {}",
            alpha,
            recovered_alpha
        );

        // Higher threshold factor -> lower Pfa
        let pfa_high = probability_of_false_alarm(10.0, n);
        assert!(pfa_high < pfa, "Higher threshold should give lower Pfa");

        // More training cells with same factor -> lower Pfa
        let pfa_more = probability_of_false_alarm(alpha, 96);
        assert!(
            pfa_more < pfa,
            "More training cells should give lower Pfa"
        );
    }

    #[test]
    fn test_empty_map() {
        let cfar = Cfar2d::new(1, 1, 2, 2, 4.0, CfarType2d::CaCell);
        let rdm: Vec<Vec<f64>> = Vec::new();
        let dets = cfar.detect(&rdm);
        assert!(dets.is_empty(), "Empty map should produce no detections");
        let tmap = cfar.threshold_map(&rdm);
        assert!(tmap.is_empty(), "Threshold map of empty input should be empty");
        let dmap = cfar.detection_map(&rdm);
        assert!(dmap.is_empty(), "Detection map of empty input should be empty");
        let noise = cfar.estimate_noise(&rdm, 0, 0);
        assert_eq!(noise, f64::MAX, "Noise estimate on empty map should be MAX");
    }
}
