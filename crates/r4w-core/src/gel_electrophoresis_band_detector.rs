//! # Gel Electrophoresis Band Detector
//!
//! Gel electrophoresis image analysis for detecting and quantifying DNA/protein bands,
//! molecular weight estimation, and lane analysis.
//!
//! ## Science Background
//!
//! - Electrophoretic mobility is proportional to log(1/MW) for SDS-PAGE proteins
//! - DNA migration: log(size_bp) is linear with distance for mid-range fragments
//! - Gaussian peak shape for well-resolved bands
//! - Band resolution R = 2(d2 - d1) / (w1 + w2)
//!
//! ## Key Components
//!
//! - **LaneProfile**: 1D intensity profile from a gel lane
//! - **BandDetector**: Peak finding with prominence/width constraints
//! - **MolecularWeightCalibration**: Standard ladder calibration (log MW vs mobility)
//! - **LaneAnalyzer**: Multi-lane gel analysis with similarity and clustering
//! - **GaussianFitter**: Fit Gaussian peaks for deconvolution of overlapping bands
//! - **QuantificationEngine**: Band area integration and relative quantification
//! - **RfCalculator**: Relative front (Rf) calculation
//! - **GelSimulator**: Synthetic gel profile generation for testing
//! - **BaselineEstimator**: Background subtraction methods
//! - **FragmentSizer**: DNA fragment size estimation and Ferguson plots

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Method for baseline correction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaselineMethod {
    /// Rolling ball algorithm with given radius.
    RollingBall(usize),
    /// Linear baseline between first and last points.
    Linear,
    /// Polynomial fit of given degree.
    Polynomial(usize),
}

/// Normalization method for lane profiles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NormMethod {
    /// Normalize so peak intensity = 1.0.
    Peak,
    /// Normalize so total area = 1.0.
    TotalArea,
    /// Normalize relative to a reference band at given position index.
    ReferenceBand(usize),
}

// ---------------------------------------------------------------------------
// LaneProfile
// ---------------------------------------------------------------------------

/// A 1D intensity profile extracted from a gel lane.
#[derive(Debug, Clone)]
pub struct LaneProfile {
    /// Intensity values along the lane.
    pub intensities: Vec<f64>,
    /// Physical spacing between pixels in mm.
    pub pixel_spacing_mm: f64,
}

impl LaneProfile {
    /// Create a new lane profile.
    pub fn new(intensities: Vec<f64>, pixel_spacing_mm: f64) -> Self {
        Self {
            intensities,
            pixel_spacing_mm,
        }
    }

    /// Length of the profile in pixels.
    pub fn len(&self) -> usize {
        self.intensities.len()
    }

    /// Whether the profile is empty.
    pub fn is_empty(&self) -> bool {
        self.intensities.is_empty()
    }

    /// Total length in mm.
    pub fn length_mm(&self) -> f64 {
        if self.intensities.is_empty() {
            return 0.0;
        }
        (self.intensities.len() - 1) as f64 * self.pixel_spacing_mm
    }

    /// Apply moving average smoothing.
    pub fn smooth(&self, window_size: usize) -> LaneProfile {
        let w = if window_size == 0 { 1 } else { window_size };
        let half = w / 2;
        let n = self.intensities.len();
        if n == 0 {
            return self.clone();
        }
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half } else { n - 1 };
            let count = (hi - lo + 1) as f64;
            let sum: f64 = self.intensities[lo..=hi].iter().sum();
            smoothed[i] = sum / count;
        }
        LaneProfile::new(smoothed, self.pixel_spacing_mm)
    }

    /// Apply baseline correction.
    pub fn baseline_correct(&self, method: BaselineMethod) -> LaneProfile {
        let baseline = match method {
            BaselineMethod::RollingBall(radius) => {
                BaselineEstimator::rolling_ball(&self.intensities, radius)
            }
            BaselineMethod::Linear => {
                BaselineEstimator::linear_baseline(&self.intensities)
            }
            BaselineMethod::Polynomial(degree) => {
                BaselineEstimator::polynomial_baseline(&self.intensities, degree)
            }
        };
        let corrected: Vec<f64> = self
            .intensities
            .iter()
            .zip(baseline.iter())
            .map(|(v, b)| (v - b).max(0.0))
            .collect();
        LaneProfile::new(corrected, self.pixel_spacing_mm)
    }

    /// Normalize the profile.
    pub fn normalize(&self, method: NormMethod) -> LaneProfile {
        let n = self.intensities.len();
        if n == 0 {
            return self.clone();
        }
        let scale = match method {
            NormMethod::Peak => {
                let mx = self
                    .intensities
                    .iter()
                    .cloned()
                    .fold(f64::NEG_INFINITY, f64::max);
                if mx.abs() < 1e-30 {
                    1.0
                } else {
                    mx
                }
            }
            NormMethod::TotalArea => {
                let area: f64 = self.intensities.iter().sum::<f64>() * self.pixel_spacing_mm;
                if area.abs() < 1e-30 {
                    1.0
                } else {
                    area
                }
            }
            NormMethod::ReferenceBand(idx) => {
                let v = if idx < n {
                    self.intensities[idx]
                } else {
                    1.0
                };
                if v.abs() < 1e-30 {
                    1.0
                } else {
                    v
                }
            }
        };
        let normed: Vec<f64> = self.intensities.iter().map(|v| v / scale).collect();
        LaneProfile::new(normed, self.pixel_spacing_mm)
    }
}

// ---------------------------------------------------------------------------
// Band
// ---------------------------------------------------------------------------

/// A detected band in a gel lane.
#[derive(Debug, Clone)]
pub struct Band {
    /// Position in pixels (sub-pixel from Gaussian fit).
    pub position_px: f64,
    /// Position in millimetres from lane start.
    pub position_mm: f64,
    /// Peak height (intensity).
    pub height: f64,
    /// Full width at half maximum in pixels.
    pub width: f64,
    /// Integrated area under the band.
    pub area: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
}

// ---------------------------------------------------------------------------
// BandDetector
// ---------------------------------------------------------------------------

/// Detect bands (peaks) in lane intensity profiles.
pub struct BandDetector {
    /// Minimum peak height to qualify.
    pub min_height: f64,
    /// Minimum peak width (FWHM) in pixels.
    pub min_width: f64,
    /// Minimum prominence.
    pub min_prominence: f64,
}

impl BandDetector {
    /// Create a new band detector with given thresholds.
    pub fn new(min_height: f64, min_width: f64, min_prominence: f64) -> Self {
        Self {
            min_height,
            min_width,
            min_prominence,
        }
    }

    /// Detect bands in the given lane profile.
    pub fn detect(&self, profile: &LaneProfile) -> Vec<Band> {
        let data = &profile.intensities;
        let n = data.len();
        if n < 3 {
            return vec![];
        }

        // Find local maxima.
        let mut peak_indices: Vec<usize> = Vec::new();
        for i in 1..n - 1 {
            if data[i] > data[i - 1] && data[i] > data[i + 1] {
                peak_indices.push(i);
            }
        }

        let noise = background_noise_level_raw(data);

        let mut bands = Vec::new();
        for &pi in &peak_indices {
            let height = data[pi];
            if height < self.min_height {
                continue;
            }

            // Prominence: how much the peak stands above the higher of the two
            // nearest valleys on either side.
            let left_min = find_valley_left(data, pi);
            let right_min = find_valley_right(data, pi);
            let prominence = height - left_min.max(right_min);
            if prominence < self.min_prominence {
                continue;
            }

            // Width at half prominence.
            let half_level = height - prominence / 2.0;
            let left_w = half_width_left(data, pi, half_level);
            let right_w = half_width_right(data, pi, half_level);
            let fwhm = left_w + right_w;
            if fwhm < self.min_width {
                continue;
            }

            // Gaussian fit for sub-pixel centre.
            let gp = GaussianFitter::fit_single(data, pi);

            // Area via Gaussian: A = amplitude * sigma * sqrt(2*pi).
            let area = gp.amplitude * gp.sigma * (2.0 * PI).sqrt();

            let snr = if noise > 1e-30 {
                height / noise
            } else {
                f64::INFINITY
            };

            bands.push(Band {
                position_px: gp.center,
                position_mm: gp.center * profile.pixel_spacing_mm,
                height,
                width: fwhm,
                area,
                snr,
            });
        }

        // Sort by position.
        bands.sort_by(|a, b| a.position_px.partial_cmp(&b.position_px).unwrap());
        bands
    }
}

/// Find the minimum value to the left of index `peak` before hitting another peak.
fn find_valley_left(data: &[f64], peak: usize) -> f64 {
    let mut min_val = data[peak];
    for i in (0..peak).rev() {
        if data[i] < min_val {
            min_val = data[i];
        }
        // Stop if we hit a rising edge (another peak).
        if i > 0 && data[i] > data[i - 1] && data[i] > min_val + 1e-12 {
            break;
        }
    }
    min_val
}

/// Find the minimum value to the right of index `peak` before hitting another peak.
fn find_valley_right(data: &[f64], peak: usize) -> f64 {
    let n = data.len();
    let mut min_val = data[peak];
    for i in (peak + 1)..n {
        if data[i] < min_val {
            min_val = data[i];
        }
        if i + 1 < n && data[i] > data[i + 1] && data[i] > min_val + 1e-12 {
            break;
        }
    }
    min_val
}

/// Half-width to the left at a given level.
fn half_width_left(data: &[f64], peak: usize, level: f64) -> f64 {
    for i in (0..peak).rev() {
        if data[i] <= level {
            // Linear interpolation.
            let frac = (level - data[i]) / (data[i + 1] - data[i]).max(1e-30);
            return (peak as f64) - (i as f64 + frac);
        }
    }
    peak as f64
}

/// Half-width to the right at a given level.
fn half_width_right(data: &[f64], peak: usize, level: f64) -> f64 {
    let n = data.len();
    for i in (peak + 1)..n {
        if data[i] <= level {
            let frac = (level - data[i]) / (data[i - 1] - data[i]).max(1e-30);
            return (i as f64 - frac) - peak as f64;
        }
    }
    (n - 1 - peak) as f64
}

/// Estimate background noise level from the profile (MAD-based).
fn background_noise_level_raw(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    // Median absolute deviation of first differences.
    let mut diffs: Vec<f64> = data.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    if diffs.is_empty() {
        return 0.0;
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = diffs[diffs.len() / 2];
    // MAD to sigma: sigma ~ 1.4826 * MAD, and diff has sqrt(2) factor.
    median * 1.4826 / 1.4142
}

// ---------------------------------------------------------------------------
// GaussianParams / GaussianFitter
// ---------------------------------------------------------------------------

/// Parameters of a Gaussian peak: y = amplitude * exp(-(x-center)^2 / (2*sigma^2)).
#[derive(Debug, Clone, Copy)]
pub struct GaussianParams {
    /// Peak centre position.
    pub center: f64,
    /// Peak amplitude.
    pub amplitude: f64,
    /// Standard deviation (width parameter).
    pub sigma: f64,
}

/// Fit Gaussian peaks to band profiles.
pub struct GaussianFitter;

impl GaussianFitter {
    /// Fit a single Gaussian peak near `center_guess` using parabolic log interpolation.
    pub fn fit_single(profile: &[f64], center_guess: usize) -> GaussianParams {
        let n = profile.len();
        if n < 3 || center_guess == 0 || center_guess >= n - 1 {
            return GaussianParams {
                center: center_guess as f64,
                amplitude: if center_guess < n {
                    profile[center_guess]
                } else {
                    0.0
                },
                sigma: 1.0,
            };
        }

        let y0 = profile[center_guess - 1].max(1e-30);
        let y1 = profile[center_guess].max(1e-30);
        let y2 = profile[center_guess + 1].max(1e-30);

        let ln0 = y0.ln();
        let ln1 = y1.ln();
        let ln2 = y2.ln();

        // Parabolic interpolation in log domain.
        // For parabola through (-1,ln0), (0,ln1), (1,ln2):
        //   delta = (ln0 - ln2) / (2*(ln0 - 2*ln1 + ln2))
        let denom = 2.0 * (ln0 - 2.0 * ln1 + ln2);
        let center = if denom.abs() > 1e-30 {
            center_guess as f64 + (ln0 - ln2) / denom
        } else {
            center_guess as f64
        };

        let amplitude = y1;

        // Estimate sigma from FWHM via the three points.
        let sigma = if denom.abs() > 1e-30 {
            (-1.0 / denom).abs().sqrt()
        } else {
            1.0
        };

        GaussianParams {
            center,
            amplitude,
            sigma: sigma.max(0.1),
        }
    }

    /// Fit multiple Gaussian peaks using iterative residual fitting.
    /// `num_peaks` is the number of peaks to find.
    pub fn fit_multiple(profile: &[f64], num_peaks: usize) -> Vec<GaussianParams> {
        let n = profile.len();
        if n < 3 || num_peaks == 0 {
            return vec![];
        }

        let mut residual = profile.to_vec();
        let mut results = Vec::new();

        for _ in 0..num_peaks {
            // Find max in residual.
            let mut max_idx = 0;
            let mut max_val = f64::NEG_INFINITY;
            for (i, &v) in residual.iter().enumerate() {
                if v > max_val {
                    max_val = v;
                    max_idx = i;
                }
            }
            if max_val <= 0.0 {
                break;
            }

            let gp = Self::fit_single(&residual, max_idx);
            // Subtract fitted Gaussian from residual.
            for (i, r) in residual.iter_mut().enumerate() {
                let x = i as f64 - gp.center;
                let g = gp.amplitude * (-x * x / (2.0 * gp.sigma * gp.sigma)).exp();
                *r = (*r - g).max(0.0);
            }
            results.push(gp);
        }
        results
    }

    /// Evaluate a Gaussian at given x values.
    pub fn evaluate(params: &GaussianParams, x: &[f64]) -> Vec<f64> {
        x.iter()
            .map(|&xi| {
                let d = xi - params.center;
                params.amplitude * (-d * d / (2.0 * params.sigma * params.sigma)).exp()
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// MolecularWeightCalibration
// ---------------------------------------------------------------------------

/// A point on the MW calibration curve.
#[derive(Debug, Clone, Copy)]
struct CalPoint {
    position_mm: f64,
    log_mw: f64,
}

/// Calibration from standard ladder: log(MW) = a * position + b.
#[derive(Debug, Clone)]
pub struct MwCalibration {
    /// Slope of log(MW) vs position.
    pub slope: f64,
    /// Intercept.
    pub intercept: f64,
    /// Coefficient of determination.
    pub r_squared: f64,
}

impl MwCalibration {
    /// Estimate molecular weight from position in mm.
    pub fn estimate_mw(&self, position_mm: f64) -> f64 {
        let log_mw = self.slope * position_mm + self.intercept;
        (10.0_f64).powf(log_mw)
    }
}

/// Molecular weight calibration using standard ladders.
pub struct MolecularWeightCalibration {
    /// (position_mm, molecular_weight_da) pairs.
    standards: Vec<(f64, f64)>,
}

impl MolecularWeightCalibration {
    /// Create from (position_mm, mw_daltons) pairs.
    pub fn new(standard_bands: Vec<(f64, f64)>) -> Self {
        Self {
            standards: standard_bands,
        }
    }

    /// Perform linear regression of log10(MW) vs position.
    pub fn calibrate(&self) -> MwCalibration {
        let points: Vec<CalPoint> = self
            .standards
            .iter()
            .map(|&(pos, mw)| CalPoint {
                position_mm: pos,
                log_mw: mw.log10(),
            })
            .collect();

        let n = points.len() as f64;
        if n < 2.0 {
            return MwCalibration {
                slope: 0.0,
                intercept: 0.0,
                r_squared: 0.0,
            };
        }

        let sum_x: f64 = points.iter().map(|p| p.position_mm).sum();
        let sum_y: f64 = points.iter().map(|p| p.log_mw).sum();
        let sum_xy: f64 = points.iter().map(|p| p.position_mm * p.log_mw).sum();
        let sum_xx: f64 = points.iter().map(|p| p.position_mm * p.position_mm).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        let (slope, intercept) = if denom.abs() > 1e-30 {
            let s = (n * sum_xy - sum_x * sum_y) / denom;
            let i = (sum_y - s * sum_x) / n;
            (s, i)
        } else {
            (0.0, sum_y / n)
        };

        // R^2.
        let y_mean = sum_y / n;
        let ss_tot: f64 = points.iter().map(|p| (p.log_mw - y_mean).powi(2)).sum();
        let ss_res: f64 = points
            .iter()
            .map(|p| {
                let pred = slope * p.position_mm + intercept;
                (p.log_mw - pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        MwCalibration {
            slope,
            intercept,
            r_squared,
        }
    }

    /// R-squared of the calibration.
    pub fn r_squared(&self) -> f64 {
        self.calibrate().r_squared
    }

    /// Common DNA ladders.
    pub fn dna_1kb_ladder() -> Self {
        // Typical positions for 1kb ladder (positions in mm, MW in bp).
        Self::new(vec![
            (5.0, 10000.0),
            (10.0, 8000.0),
            (15.0, 6000.0),
            (22.0, 4000.0),
            (30.0, 3000.0),
            (38.0, 2000.0),
            (48.0, 1500.0),
            (55.0, 1000.0),
            (65.0, 750.0),
            (75.0, 500.0),
            (90.0, 250.0),
        ])
    }

    /// Common 100bp DNA ladder.
    pub fn dna_100bp_ladder() -> Self {
        Self::new(vec![
            (10.0, 1500.0),
            (18.0, 1000.0),
            (26.0, 900.0),
            (30.0, 800.0),
            (34.0, 700.0),
            (39.0, 600.0),
            (45.0, 500.0),
            (52.0, 400.0),
            (62.0, 300.0),
            (75.0, 200.0),
            (95.0, 100.0),
        ])
    }

    /// Protein MW marker ladder (approximate SDS-PAGE positions).
    pub fn protein_mw_markers() -> Self {
        Self::new(vec![
            (8.0, 250000.0),
            (14.0, 150000.0),
            (22.0, 100000.0),
            (30.0, 75000.0),
            (42.0, 50000.0),
            (55.0, 37000.0),
            (68.0, 25000.0),
            (82.0, 15000.0),
            (95.0, 10000.0),
        ])
    }
}

// ---------------------------------------------------------------------------
// DendrogramNode
// ---------------------------------------------------------------------------

/// A node in an agglomerative clustering dendrogram.
#[derive(Debug, Clone)]
pub struct DendrogramNode {
    /// Left child index (leaf index or previous merge index + n_leaves).
    pub left: usize,
    /// Right child index.
    pub right: usize,
    /// Distance at which the merge occurred.
    pub distance: f64,
    /// Number of leaves under this node.
    pub size: usize,
}

// ---------------------------------------------------------------------------
// LaneAnalyzer
// ---------------------------------------------------------------------------

/// Multi-lane gel analysis.
pub struct LaneAnalyzer {
    lanes: Vec<Option<LaneProfile>>,
}

impl LaneAnalyzer {
    /// Create for `num_lanes` lanes.
    pub fn new(num_lanes: usize) -> Self {
        Self {
            lanes: vec![None; num_lanes],
        }
    }

    /// Number of lanes.
    pub fn num_lanes(&self) -> usize {
        self.lanes.len()
    }

    /// Add a lane profile at the given index.
    pub fn add_lane(&mut self, index: usize, profile: LaneProfile) {
        if index < self.lanes.len() {
            self.lanes[index] = Some(profile);
        }
    }

    /// Detect bands in all lanes.
    pub fn detect_all_bands(&self, detector: &BandDetector) -> Vec<Vec<Band>> {
        self.lanes
            .iter()
            .map(|lane_opt| {
                if let Some(lane) = lane_opt {
                    detector.detect(lane)
                } else {
                    vec![]
                }
            })
            .collect()
    }

    /// Dice similarity coefficient between two lanes based on detected band positions.
    /// Uses a tolerance window for position matching.
    pub fn lane_similarity(
        &self,
        lane_a: usize,
        lane_b: usize,
        detector: &BandDetector,
        tolerance_mm: f64,
    ) -> f64 {
        let bands_a = if let Some(ref p) = self.lanes.get(lane_a).and_then(|o| o.as_ref()) {
            detector.detect(p)
        } else {
            return 0.0;
        };
        let bands_b = if let Some(ref p) = self.lanes.get(lane_b).and_then(|o| o.as_ref()) {
            detector.detect(p)
        } else {
            return 0.0;
        };

        if bands_a.is_empty() && bands_b.is_empty() {
            return 1.0;
        }
        if bands_a.is_empty() || bands_b.is_empty() {
            return 0.0;
        }

        // Count matching bands (Dice coefficient).
        let mut matches = 0usize;
        let mut used_b = vec![false; bands_b.len()];
        for ba in &bands_a {
            for (j, bb) in bands_b.iter().enumerate() {
                if !used_b[j] && (ba.position_mm - bb.position_mm).abs() <= tolerance_mm {
                    matches += 1;
                    used_b[j] = true;
                    break;
                }
            }
        }

        (2.0 * matches as f64) / (bands_a.len() + bands_b.len()) as f64
    }

    /// Jaccard similarity coefficient between two lanes.
    pub fn lane_similarity_jaccard(
        &self,
        lane_a: usize,
        lane_b: usize,
        detector: &BandDetector,
        tolerance_mm: f64,
    ) -> f64 {
        let bands_a = if let Some(ref p) = self.lanes.get(lane_a).and_then(|o| o.as_ref()) {
            detector.detect(p)
        } else {
            return 0.0;
        };
        let bands_b = if let Some(ref p) = self.lanes.get(lane_b).and_then(|o| o.as_ref()) {
            detector.detect(p)
        } else {
            return 0.0;
        };

        if bands_a.is_empty() && bands_b.is_empty() {
            return 1.0;
        }
        if bands_a.is_empty() || bands_b.is_empty() {
            return 0.0;
        }

        let mut matches = 0usize;
        let mut used_b = vec![false; bands_b.len()];
        for ba in &bands_a {
            for (j, bb) in bands_b.iter().enumerate() {
                if !used_b[j] && (ba.position_mm - bb.position_mm).abs() <= tolerance_mm {
                    matches += 1;
                    used_b[j] = true;
                    break;
                }
            }
        }

        let union = bands_a.len() + bands_b.len() - matches;
        if union == 0 {
            1.0
        } else {
            matches as f64 / union as f64
        }
    }

    /// UPGMA hierarchical clustering dendrogram from a similarity matrix.
    pub fn dendrogram(similarity_matrix: &[Vec<f64>]) -> Vec<DendrogramNode> {
        let n = similarity_matrix.len();
        if n < 2 {
            return vec![];
        }

        // Convert similarity to distance.
        let mut dist: Vec<Vec<f64>> = similarity_matrix
            .iter()
            .map(|row| row.iter().map(|s| 1.0 - s).collect())
            .collect();

        let mut active: Vec<bool> = vec![true; n];
        let mut sizes: Vec<usize> = vec![1; n];
        let mut nodes: Vec<DendrogramNode> = Vec::new();
        // Map from original/merged index to current cluster label.
        let mut labels: Vec<usize> = (0..n).collect();

        for _ in 0..(n - 1) {
            // Find minimum distance pair among active clusters.
            let mut best_d = f64::INFINITY;
            let mut best_i = 0;
            let mut best_j = 0;
            for i in 0..dist.len() {
                if !active[i] {
                    continue;
                }
                for j in (i + 1)..dist.len() {
                    if !active[j] {
                        continue;
                    }
                    if dist[i][j] < best_d {
                        best_d = dist[i][j];
                        best_i = i;
                        best_j = j;
                    }
                }
            }

            let new_label = n + nodes.len();
            let ni = sizes[best_i];
            let nj = sizes[best_j];
            let new_size = ni + nj;

            nodes.push(DendrogramNode {
                left: labels[best_i],
                right: labels[best_j],
                distance: best_d,
                size: new_size,
            });

            // UPGMA: update distances.
            for k in 0..dist.len() {
                if !active[k] || k == best_i || k == best_j {
                    continue;
                }
                let d_new = (dist[best_i][k] * ni as f64 + dist[best_j][k] * nj as f64)
                    / new_size as f64;
                dist[best_i][k] = d_new;
                dist[k][best_i] = d_new;
            }

            active[best_j] = false;
            sizes[best_i] = new_size;
            labels[best_i] = new_label;
        }

        nodes
    }
}

// ---------------------------------------------------------------------------
// QuantificationEngine
// ---------------------------------------------------------------------------

/// Band quantification utilities.
pub struct QuantificationEngine;

impl QuantificationEngine {
    /// Integrate band area from the profile using trapezoidal rule around the band.
    pub fn band_area(profile: &LaneProfile, band: &Band) -> f64 {
        let center = band.position_px;
        let hw = band.width; // Use full width.
        let lo = ((center - hw).floor().max(0.0)) as usize;
        let hi = ((center + hw).ceil() as usize).min(profile.intensities.len().saturating_sub(1));

        if hi <= lo {
            return 0.0;
        }

        let mut area = 0.0;
        for i in lo..hi {
            area += (profile.intensities[i] + profile.intensities[i + 1]) / 2.0
                * profile.pixel_spacing_mm;
        }
        area
    }

    /// Compute relative quantities (fraction of total area) for a set of bands.
    pub fn relative_quantity(bands: &[Band]) -> Vec<f64> {
        let total: f64 = bands.iter().map(|b| b.area).sum();
        if total.abs() < 1e-30 {
            return vec![0.0; bands.len()];
        }
        bands.iter().map(|b| b.area / total).collect()
    }

    /// Estimate concentration from a standard curve (area, concentration) pairs.
    pub fn concentration_from_standard(band_area: f64, std_curve: &[(f64, f64)]) -> f64 {
        if std_curve.len() < 2 {
            return 0.0;
        }

        // Linear regression of concentration vs area.
        let n = std_curve.len() as f64;
        let sum_x: f64 = std_curve.iter().map(|p| p.0).sum();
        let sum_y: f64 = std_curve.iter().map(|p| p.1).sum();
        let sum_xy: f64 = std_curve.iter().map(|p| p.0 * p.1).sum();
        let sum_xx: f64 = std_curve.iter().map(|p| p.0 * p.0).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return sum_y / n;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        (slope * band_area + intercept).max(0.0)
    }

    /// Estimate background noise level from a lane profile.
    pub fn background_noise_level(profile: &LaneProfile) -> f64 {
        background_noise_level_raw(&profile.intensities)
    }
}

// ---------------------------------------------------------------------------
// RfCalculator
// ---------------------------------------------------------------------------

/// Relative front (Rf) calculation for gel electrophoresis.
pub struct RfCalculator;

impl RfCalculator {
    /// Calculate Rf = (band_position - well_position) / (tracking_dye_position - well_position).
    pub fn calculate_rf(
        band_position: f64,
        tracking_dye_position: f64,
        well_position: f64,
    ) -> f64 {
        let total = tracking_dye_position - well_position;
        if total.abs() < 1e-30 {
            return 0.0;
        }
        (band_position - well_position) / total
    }

    /// Convert Rf to estimated molecular weight using a calibration.
    pub fn rf_to_mw(rf: f64, calibration: &MwCalibration, gel_length_mm: f64) -> f64 {
        let position_mm = rf * gel_length_mm;
        calibration.estimate_mw(position_mm)
    }

    /// Band resolution between two adjacent bands.
    /// R = 2(d2 - d1) / (w1 + w2)
    pub fn band_resolution(band1: &Band, band2: &Band) -> f64 {
        let d = (band2.position_mm - band1.position_mm).abs();
        let w_sum = band1.width + band2.width;
        if w_sum < 1e-30 {
            return 0.0;
        }
        2.0 * d / w_sum
    }
}

// ---------------------------------------------------------------------------
// GelSimulator
// ---------------------------------------------------------------------------

/// Generate synthetic gel lane profiles for testing.
pub struct GelSimulator;

impl GelSimulator {
    /// Simulate a single lane with given bands (position_px, height, width_sigma).
    pub fn simulate_lane(
        bands: &[(f64, f64, f64)],
        num_pixels: usize,
        pixel_spacing_mm: f64,
        noise_level: f64,
    ) -> LaneProfile {
        let mut intensities = vec![0.0; num_pixels];

        // Add Gaussian peaks.
        for &(pos, height, sigma) in bands {
            for (i, val) in intensities.iter_mut().enumerate() {
                let x = i as f64 - pos;
                *val += height * (-x * x / (2.0 * sigma * sigma)).exp();
            }
        }

        // Add noise using a simple LCG PRNG.
        if noise_level > 0.0 {
            let mut seed: u64 = 42;
            for val in intensities.iter_mut() {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                // Convert to approximate Gaussian using Box-Muller-like approach.
                let u1 = ((seed >> 32) as f64) / (u32::MAX as f64);
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u2 = ((seed >> 32) as f64) / (u32::MAX as f64);
                let u1c = u1.max(1e-10);
                let gauss = (-2.0 * u1c.ln()).sqrt() * (2.0 * PI * u2).cos();
                *val += noise_level * gauss;
                *val = val.max(0.0);
            }
        }

        LaneProfile::new(intensities, pixel_spacing_mm)
    }

    /// Simulate a standard molecular weight ladder.
    /// `mw_sizes` in Da/bp, `gel_percentage` affects spacing.
    pub fn simulate_ladder(
        mw_sizes: &[f64],
        gel_percentage: f64,
        num_pixels: usize,
        pixel_spacing_mm: f64,
    ) -> LaneProfile {
        // Mobility ~ -log10(MW) * gel_factor.
        let gel_factor = gel_percentage / 1.0; // Normalize to 1%.
        let mut bands = Vec::new();

        if mw_sizes.is_empty() {
            return LaneProfile::new(vec![0.0; num_pixels], pixel_spacing_mm);
        }

        let max_log = mw_sizes
            .iter()
            .map(|mw| mw.log10())
            .fold(f64::NEG_INFINITY, f64::max);
        let min_log = mw_sizes
            .iter()
            .map(|mw| mw.log10())
            .fold(f64::INFINITY, f64::min);
        let range = (max_log - min_log).max(0.1);

        for &mw in mw_sizes {
            let log_mw = mw.log10();
            // Larger MW = less mobility = closer to well (lower position).
            let normalized = (max_log - log_mw) / range;
            let position = 10.0 + normalized * (num_pixels as f64 - 20.0) * gel_factor.min(1.0);
            // Wider bands for larger fragments.
            let sigma = 2.0 + (log_mw - min_log) / range * 3.0;
            let height = 1.0;
            bands.push((position, height, sigma));
        }

        Self::simulate_lane(&bands, num_pixels, pixel_spacing_mm, 0.02)
    }

    /// Apply a gel smile effect (parabolic position shift across lanes).
    pub fn add_smile_effect(profiles: &mut [LaneProfile], curvature: f64) {
        let n_lanes = profiles.len();
        if n_lanes < 2 {
            return;
        }
        let center = (n_lanes - 1) as f64 / 2.0;

        for (lane_idx, profile) in profiles.iter_mut().enumerate() {
            let offset = lane_idx as f64 - center;
            let shift = curvature * offset * offset;
            let shift_px = (shift / profile.pixel_spacing_mm).round() as i64;

            if shift_px == 0 {
                continue;
            }

            let n = profile.intensities.len();
            let mut new_data = vec![0.0; n];
            for i in 0..n {
                let src = i as i64 - shift_px;
                if src >= 0 && (src as usize) < n {
                    new_data[i] = profile.intensities[src as usize];
                }
            }
            profile.intensities = new_data;
        }
    }

    /// Apply uneven loading variation (intensity scaling across lanes).
    pub fn add_loading_variation(profiles: &mut [LaneProfile], variation: f64) {
        let n_lanes = profiles.len();
        if n_lanes == 0 {
            return;
        }
        let mut seed: u64 = 12345;
        for profile in profiles.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u = ((seed >> 32) as f64) / (u32::MAX as f64);
            let factor = 1.0 + variation * (2.0 * u - 1.0);
            for v in profile.intensities.iter_mut() {
                *v *= factor;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BaselineEstimator
// ---------------------------------------------------------------------------

/// Background estimation and subtraction methods.
pub struct BaselineEstimator;

impl BaselineEstimator {
    /// Rolling ball baseline estimation.
    /// Finds the minimum in a sliding window of given radius.
    pub fn rolling_ball(profile: &[f64], radius: usize) -> Vec<f64> {
        let n = profile.len();
        if n == 0 {
            return vec![];
        }
        let r = radius.max(1);
        let mut baseline = vec![0.0; n];

        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = if i + r < n { i + r } else { n - 1 };
            let min_val = profile[lo..=hi]
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);
            baseline[i] = min_val;
        }

        // Smooth the baseline with a second pass (morphological closing).
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = if i + r < n { i + r } else { n - 1 };
            let max_val = baseline[lo..=hi]
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            smoothed[i] = max_val;
        }

        smoothed
    }

    /// Linear baseline between first and last intensity values.
    pub fn linear_baseline(profile: &[f64]) -> Vec<f64> {
        let n = profile.len();
        if n == 0 {
            return vec![];
        }
        if n == 1 {
            return vec![profile[0]];
        }

        let y0 = profile[0];
        let y1 = profile[n - 1];
        (0..n)
            .map(|i| y0 + (y1 - y0) * i as f64 / (n - 1) as f64)
            .collect()
    }

    /// Polynomial baseline fit of given degree.
    /// Uses least-squares fitting to a polynomial, constrained to fit the lower
    /// envelope by iteratively rejecting points above the fit.
    pub fn polynomial_baseline(profile: &[f64], degree: usize) -> Vec<f64> {
        let n = profile.len();
        if n == 0 {
            return vec![];
        }
        let deg = degree.min(n.saturating_sub(1)).min(5);
        if deg == 0 {
            let mean = profile.iter().sum::<f64>() / n as f64;
            return vec![mean; n];
        }

        // Simple polynomial fit using normal equations.
        let mut weights = vec![1.0; n];

        // Iterative: fit, then down-weight points well above the fit.
        for _iter in 0..3 {
            let coeffs = poly_fit(profile, deg, &weights);
            let fitted: Vec<f64> = (0..n).map(|i| poly_eval(&coeffs, i as f64)).collect();

            // Down-weight points that are significantly above the fit.
            let residuals: Vec<f64> = profile
                .iter()
                .zip(fitted.iter())
                .map(|(p, f)| p - f)
                .collect();
            let mut abs_res: Vec<f64> = residuals.iter().map(|r| r.abs()).collect();
            abs_res.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med_res = abs_res[abs_res.len() / 2].max(1e-10);

            for i in 0..n {
                if residuals[i] > 2.0 * med_res {
                    weights[i] *= 0.1;
                }
            }
        }

        let coeffs = poly_fit(profile, deg, &weights);
        (0..n).map(|i| poly_eval(&coeffs, i as f64)).collect()
    }

    /// Asymmetric least squares baseline estimation.
    /// `lambda` controls smoothness (typically 1e5-1e7), `p` is asymmetry (0.001-0.01).
    pub fn asymmetric_least_squares(profile: &[f64], lambda: f64, p: f64) -> Vec<f64> {
        let n = profile.len();
        if n < 3 {
            return profile.to_vec();
        }

        // Simplified iterative version using Whittaker smoother approach.
        let mut z = profile.to_vec();
        let mut w = vec![1.0; n];

        for _iter in 0..10 {
            // Solve tridiagonal system: (W + lambda * D'D) z = W * y.
            // where D is the 2nd difference matrix.
            // Simplified: use iterative smoothing.
            let mut new_z = vec![0.0; n];
            for i in 0..n {
                let left = if i > 0 { z[i - 1] } else { z[i] };
                let right = if i + 1 < n { z[i + 1] } else { z[i] };
                let smooth_term = lambda * (left + right);
                let data_term = w[i] * profile[i];
                new_z[i] = (data_term + smooth_term) / (w[i] + 2.0 * lambda);
            }
            z = new_z;

            // Update weights asymmetrically.
            for i in 0..n {
                w[i] = if profile[i] > z[i] { p } else { 1.0 - p };
            }
        }

        z
    }
}

/// Simple weighted polynomial fitting via normal equations.
fn poly_fit(y: &[f64], degree: usize, weights: &[f64]) -> Vec<f64> {
    let n = y.len();
    let m = degree + 1;

    // Build normal equations: A^T W A c = A^T W y.
    let mut ata = vec![vec![0.0; m]; m];
    let mut aty = vec![0.0; m];

    for i in 0..n {
        let x = i as f64;
        let w = weights[i];
        let mut xpow = vec![1.0; m];
        for j in 1..m {
            xpow[j] = xpow[j - 1] * x;
        }
        for j in 0..m {
            aty[j] += w * xpow[j] * y[i];
            for k in 0..m {
                ata[j][k] += w * xpow[j] * xpow[k];
            }
        }
    }

    // Solve via Gaussian elimination.
    gauss_solve(&mut ata, &mut aty)
}

/// Solve Ax = b by Gaussian elimination with partial pivoting.
fn gauss_solve(a: &mut [Vec<f64>], b: &mut [f64]) -> Vec<f64> {
    let n = b.len();

    for col in 0..n {
        // Partial pivoting.
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution.
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i][j] * x[j];
        }
        x[i] = if a[i][i].abs() > 1e-30 {
            sum / a[i][i]
        } else {
            0.0
        };
    }
    x
}

/// Evaluate polynomial coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ...
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xpow = 1.0;
    for &c in coeffs {
        result += c * xpow;
        xpow *= x;
    }
    result
}

// ---------------------------------------------------------------------------
// FragmentSizer
// ---------------------------------------------------------------------------

/// Ferguson plot result.
#[derive(Debug, Clone)]
pub struct FergusonResult {
    /// Free mobility (y-intercept in log space).
    pub free_mobility: f64,
    /// Retardation coefficient (slope).
    pub retardation_coefficient: f64,
    /// R-squared of the fit.
    pub r_squared: f64,
}

/// DNA fragment size estimation utilities.
pub struct FragmentSizer;

impl FragmentSizer {
    /// Ferguson plot analysis: log(mobility) vs gel percentage.
    /// `mobility` and `gel_percentages` must have the same length.
    pub fn ferguson_plot(mobility: &[f64], gel_percentages: &[f64]) -> FergusonResult {
        let n = mobility.len().min(gel_percentages.len());
        if n < 2 {
            return FergusonResult {
                free_mobility: 0.0,
                retardation_coefficient: 0.0,
                r_squared: 0.0,
            };
        }

        // Linear regression: ln(mobility) = a + b * gel_percentage.
        let log_mob: Vec<f64> = mobility[..n]
            .iter()
            .map(|m| m.max(1e-30).ln())
            .collect();
        let x = &gel_percentages[..n];

        let n_f = n as f64;
        let sum_x: f64 = x.iter().sum();
        let sum_y: f64 = log_mob.iter().sum();
        let sum_xy: f64 = x.iter().zip(log_mob.iter()).map(|(xi, yi)| xi * yi).sum();
        let sum_xx: f64 = x.iter().map(|xi| xi * xi).sum();

        let denom = n_f * sum_xx - sum_x * sum_x;
        let (slope, intercept) = if denom.abs() > 1e-30 {
            let s = (n_f * sum_xy - sum_x * sum_y) / denom;
            let i = (sum_y - s * sum_x) / n_f;
            (s, i)
        } else {
            (0.0, sum_y / n_f)
        };

        let y_mean = sum_y / n_f;
        let ss_tot: f64 = log_mob.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = x
            .iter()
            .zip(log_mob.iter())
            .map(|(xi, yi)| {
                let pred = slope * xi + intercept;
                (yi - pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        FergusonResult {
            free_mobility: intercept.exp(),
            retardation_coefficient: -slope,
            r_squared,
        }
    }

    /// Effective length correction for DNA topology.
    /// Linear DNA migrates at its true size; supercoiled migrates faster.
    pub fn effective_length(mw_daltons: f64, is_linear: bool) -> f64 {
        // Average MW of a nucleotide pair is ~660 Da.
        let bp = mw_daltons / 660.0;
        if is_linear {
            bp
        } else {
            // Supercoiled DNA typically migrates as if ~80% of its linear size.
            bp * 0.8
        }
    }

    /// Estimate DNA size in base pairs from molecular weight in Daltons.
    pub fn mw_to_bp(mw_daltons: f64) -> f64 {
        mw_daltons / 660.0
    }

    /// Convert base pairs to approximate molecular weight in Daltons.
    pub fn bp_to_mw(bp: f64) -> f64 {
        bp * 660.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- LaneProfile tests ----

    #[test]
    fn test_lane_profile_new() {
        let lp = LaneProfile::new(vec![1.0, 2.0, 3.0], 0.1);
        assert_eq!(lp.len(), 3);
        assert!(!lp.is_empty());
        assert!((lp.pixel_spacing_mm - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_lane_profile_empty() {
        let lp = LaneProfile::new(vec![], 0.1);
        assert!(lp.is_empty());
        assert_eq!(lp.len(), 0);
        assert!((lp.length_mm() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_lane_profile_length_mm() {
        let lp = LaneProfile::new(vec![0.0; 101], 0.5);
        assert!((lp.length_mm() - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_smooth_identity_window_1() {
        let lp = LaneProfile::new(vec![1.0, 5.0, 1.0, 5.0, 1.0], 1.0);
        let smoothed = lp.smooth(1);
        // Window of 1 should leave values unchanged.
        for (a, b) in smoothed.intensities.iter().zip(lp.intensities.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_smooth_reduces_variation() {
        let lp = LaneProfile::new(vec![0.0, 10.0, 0.0, 10.0, 0.0], 1.0);
        let smoothed = lp.smooth(3);
        let orig_var: f64 = lp.intensities.iter().map(|v| (v - 4.0).powi(2)).sum();
        let smooth_var: f64 = smoothed
            .intensities
            .iter()
            .map(|v| (v - 4.0).powi(2))
            .sum();
        assert!(smooth_var < orig_var);
    }

    #[test]
    fn test_smooth_preserves_length() {
        let lp = LaneProfile::new(vec![1.0; 50], 0.1);
        let smoothed = lp.smooth(5);
        assert_eq!(smoothed.len(), lp.len());
    }

    #[test]
    fn test_baseline_correct_linear() {
        let data: Vec<f64> = (0..100)
            .map(|i| {
                let x = i as f64;
                0.5 * x / 100.0 + if (i as i64 - 50).abs() < 5 { 1.0 } else { 0.0 }
            })
            .collect();
        let lp = LaneProfile::new(data, 1.0);
        let corrected = lp.baseline_correct(BaselineMethod::Linear);
        // The linear baseline should remove the slope.
        assert!(corrected.intensities[50] > corrected.intensities[0]);
    }

    #[test]
    fn test_normalize_peak() {
        let lp = LaneProfile::new(vec![0.0, 5.0, 10.0, 3.0], 1.0);
        let normed = lp.normalize(NormMethod::Peak);
        let mx = normed
            .intensities
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((mx - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_total_area() {
        let lp = LaneProfile::new(vec![1.0, 2.0, 3.0, 4.0], 0.5);
        let normed = lp.normalize(NormMethod::TotalArea);
        let area: f64 = normed.intensities.iter().sum::<f64>() * normed.pixel_spacing_mm;
        assert!((area - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_reference_band() {
        let lp = LaneProfile::new(vec![2.0, 4.0, 8.0, 1.0], 1.0);
        let normed = lp.normalize(NormMethod::ReferenceBand(2));
        assert!((normed.intensities[2] - 1.0).abs() < 1e-10);
        assert!((normed.intensities[1] - 0.5).abs() < 1e-10);
    }

    // ---- BandDetector tests ----

    #[test]
    fn test_detect_single_band() {
        let bands_spec = vec![(50.0, 10.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert_eq!(bands.len(), 1);
        assert!((bands[0].position_px - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_detect_two_separated_bands() {
        let bands_spec = vec![(30.0, 8.0, 3.0), (70.0, 6.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert_eq!(bands.len(), 2);
        assert!((bands[0].position_px - 30.0).abs() < 1.0);
        assert!((bands[1].position_px - 70.0).abs() < 1.0);
    }

    #[test]
    fn test_detect_height_threshold() {
        let bands_spec = vec![(50.0, 0.1, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(1.0, 1.0, 0.0);
        let bands = detector.detect(&profile);
        assert_eq!(bands.len(), 0);
    }

    #[test]
    fn test_detect_width_threshold() {
        // Very narrow peak.
        let mut data = vec![0.0; 100];
        data[50] = 10.0;
        data[49] = 0.1;
        data[51] = 0.1;
        let profile = LaneProfile::new(data, 1.0);
        let detector = BandDetector::new(0.5, 5.0, 0.0);
        let bands = detector.detect(&profile);
        // Should be filtered out by minimum width.
        assert_eq!(bands.len(), 0);
    }

    #[test]
    fn test_detect_prominence_threshold() {
        // Peak on a high baseline (low prominence).
        let mut data = vec![9.0; 100];
        data[50] = 10.0;
        data[49] = 9.5;
        data[51] = 9.5;
        let profile = LaneProfile::new(data, 1.0);
        let detector = BandDetector::new(0.5, 0.0, 5.0);
        let bands = detector.detect(&profile);
        assert_eq!(bands.len(), 0);
    }

    #[test]
    fn test_detect_bands_sorted_by_position() {
        let bands_spec = vec![(70.0, 5.0, 3.0), (30.0, 8.0, 3.0), (50.0, 6.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        for w in bands.windows(2) {
            assert!(w[0].position_px < w[1].position_px);
        }
    }

    #[test]
    fn test_band_snr() {
        let bands_spec = vec![(50.0, 10.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert!(!bands.is_empty());
        assert!(bands[0].snr > 1.0);
    }

    #[test]
    fn test_band_area_positive() {
        let bands_spec = vec![(50.0, 10.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert!(!bands.is_empty());
        assert!(bands[0].area > 0.0);
    }

    #[test]
    fn test_detect_empty_profile() {
        let profile = LaneProfile::new(vec![], 1.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert!(bands.is_empty());
    }

    #[test]
    fn test_detect_flat_profile() {
        let profile = LaneProfile::new(vec![5.0; 100], 1.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert!(bands.is_empty());
    }

    // ---- GaussianFitter tests ----

    #[test]
    fn test_gaussian_fit_single_centered() {
        // Create a perfect Gaussian peak at position 50 with sigma=4.
        let mut data = vec![0.0; 100];
        for i in 0..100 {
            let x = i as f64 - 50.0;
            data[i] = 10.0 * (-x * x / (2.0 * 16.0)).exp();
        }
        let gp = GaussianFitter::fit_single(&data, 50);
        assert!((gp.center - 50.0).abs() < 0.1);
        assert!((gp.amplitude - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_gaussian_fit_off_center() {
        let mut data = vec![0.0; 100];
        let true_center = 50.3;
        let sigma = 5.0;
        for i in 0..100 {
            let x = i as f64 - true_center;
            data[i] = 8.0 * (-x * x / (2.0 * sigma * sigma)).exp();
        }
        // The parabolic log-interpolation fit should find sub-pixel center near 50.3.
        let gp = GaussianFitter::fit_single(&data, 50);
        assert!(
            (gp.center - true_center).abs() < 0.5,
            "Expected center ~{}, got {}",
            true_center,
            gp.center
        );
    }

    #[test]
    fn test_gaussian_fit_multiple() {
        let mut data = vec![0.0; 200];
        for i in 0..200 {
            let x = i as f64;
            data[i] = 10.0 * (-(x - 50.0).powi(2) / 32.0).exp()
                + 7.0 * (-(x - 120.0).powi(2) / 50.0).exp();
        }
        let peaks = GaussianFitter::fit_multiple(&data, 2);
        assert_eq!(peaks.len(), 2);
    }

    #[test]
    fn test_gaussian_evaluate() {
        let gp = GaussianParams {
            center: 0.0,
            amplitude: 1.0,
            sigma: 1.0,
        };
        let x = vec![0.0, 1.0, -1.0];
        let y = GaussianFitter::evaluate(&gp, &x);
        assert!((y[0] - 1.0).abs() < 1e-10);
        assert!((y[1] - y[2]).abs() < 1e-10); // Symmetric.
        assert!((y[1] - (-0.5_f64).exp()).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_fit_edge_case() {
        let data = vec![1.0, 2.0, 3.0];
        let gp = GaussianFitter::fit_single(&data, 0);
        // Edge case: center_guess = 0, should fallback.
        assert!((gp.center - 0.0).abs() < 1e-10);
    }

    // ---- MolecularWeightCalibration tests ----

    #[test]
    fn test_mw_calibration_basic() {
        let cal = MolecularWeightCalibration::new(vec![
            (10.0, 10000.0),
            (30.0, 1000.0),
            (50.0, 100.0),
        ]);
        let result = cal.calibrate();
        // log(MW) should decrease with position (negative slope).
        assert!(result.slope < 0.0);
        assert!(result.r_squared > 0.9);
    }

    #[test]
    fn test_mw_estimation() {
        let cal = MolecularWeightCalibration::new(vec![
            (10.0, 10000.0),
            (30.0, 1000.0),
            (50.0, 100.0),
        ]);
        let result = cal.calibrate();
        // At position 10 should be near 10000.
        let mw10 = result.estimate_mw(10.0);
        assert!((mw10 - 10000.0).abs() / 10000.0 < 0.2);
        // MW should decrease with distance.
        let mw50 = result.estimate_mw(50.0);
        assert!(mw50 < mw10);
    }

    #[test]
    fn test_mw_calibration_perfect_fit() {
        // Perfectly log-linear data.
        let cal = MolecularWeightCalibration::new(vec![
            (0.0, 10000.0),
            (10.0, 1000.0),
            (20.0, 100.0),
            (30.0, 10.0),
        ]);
        let result = cal.calibrate();
        assert!((result.r_squared - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_dna_1kb_ladder() {
        let cal = MolecularWeightCalibration::dna_1kb_ladder();
        let result = cal.calibrate();
        assert!(result.r_squared > 0.9);
    }

    #[test]
    fn test_dna_100bp_ladder() {
        let cal = MolecularWeightCalibration::dna_100bp_ladder();
        let result = cal.calibrate();
        assert!(result.r_squared > 0.9);
    }

    #[test]
    fn test_protein_mw_markers() {
        let cal = MolecularWeightCalibration::protein_mw_markers();
        let result = cal.calibrate();
        assert!(result.r_squared > 0.9);
    }

    #[test]
    fn test_mw_calibration_single_point() {
        let cal = MolecularWeightCalibration::new(vec![(10.0, 1000.0)]);
        let result = cal.calibrate();
        assert!((result.slope - 0.0).abs() < 1e-10);
    }

    // ---- LaneAnalyzer tests ----

    #[test]
    fn test_lane_analyzer_new() {
        let la = LaneAnalyzer::new(5);
        assert_eq!(la.num_lanes(), 5);
    }

    #[test]
    fn test_lane_analyzer_add_lane() {
        let mut la = LaneAnalyzer::new(3);
        let profile = GelSimulator::simulate_lane(&[(50.0, 10.0, 3.0)], 100, 1.0, 0.0);
        la.add_lane(0, profile);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let all_bands = la.detect_all_bands(&detector);
        assert_eq!(all_bands.len(), 3);
        assert!(!all_bands[0].is_empty());
        assert!(all_bands[1].is_empty()); // No lane added.
    }

    #[test]
    fn test_lane_similarity_identical() {
        let mut la = LaneAnalyzer::new(2);
        let profile = GelSimulator::simulate_lane(&[(30.0, 10.0, 3.0), (60.0, 8.0, 3.0)], 100, 1.0, 0.0);
        la.add_lane(0, profile.clone());
        la.add_lane(1, profile);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let sim = la.lane_similarity(0, 1, &detector, 2.0);
        assert!((sim - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_lane_similarity_different() {
        let mut la = LaneAnalyzer::new(2);
        let p1 = GelSimulator::simulate_lane(&[(20.0, 10.0, 3.0)], 100, 1.0, 0.0);
        let p2 = GelSimulator::simulate_lane(&[(80.0, 10.0, 3.0)], 100, 1.0, 0.0);
        la.add_lane(0, p1);
        la.add_lane(1, p2);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let sim = la.lane_similarity(0, 1, &detector, 2.0);
        assert!(sim < 0.5);
    }

    #[test]
    fn test_lane_similarity_jaccard() {
        let mut la = LaneAnalyzer::new(2);
        let profile = GelSimulator::simulate_lane(&[(30.0, 10.0, 3.0), (60.0, 8.0, 3.0)], 100, 1.0, 0.0);
        la.add_lane(0, profile.clone());
        la.add_lane(1, profile);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let sim = la.lane_similarity_jaccard(0, 1, &detector, 2.0);
        assert!((sim - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_dendrogram_basic() {
        let sim = vec![
            vec![1.0, 0.8, 0.2],
            vec![0.8, 1.0, 0.3],
            vec![0.2, 0.3, 1.0],
        ];
        let nodes = LaneAnalyzer::dendrogram(&sim);
        assert_eq!(nodes.len(), 2); // n-1 merges.
        // First merge should be the most similar pair (0, 1).
        assert!(nodes[0].distance < nodes[1].distance);
    }

    #[test]
    fn test_dendrogram_two_items() {
        let sim = vec![vec![1.0, 0.5], vec![0.5, 1.0]];
        let nodes = LaneAnalyzer::dendrogram(&sim);
        assert_eq!(nodes.len(), 1);
        assert!((nodes[0].distance - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_dendrogram_single_item() {
        let sim = vec![vec![1.0]];
        let nodes = LaneAnalyzer::dendrogram(&sim);
        assert!(nodes.is_empty());
    }

    // ---- QuantificationEngine tests ----

    #[test]
    fn test_band_area_integration() {
        let bands_spec = vec![(50.0, 10.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&profile);
        assert!(!bands.is_empty());
        let area = QuantificationEngine::band_area(&profile, &bands[0]);
        assert!(area > 0.0);
    }

    #[test]
    fn test_relative_quantity() {
        let b1 = Band {
            position_px: 10.0,
            position_mm: 10.0,
            height: 5.0,
            width: 3.0,
            area: 30.0,
            snr: 10.0,
        };
        let b2 = Band {
            position_px: 50.0,
            position_mm: 50.0,
            height: 5.0,
            width: 3.0,
            area: 70.0,
            snr: 10.0,
        };
        let rq = QuantificationEngine::relative_quantity(&[b1, b2]);
        assert!((rq[0] - 0.3).abs() < 1e-10);
        assert!((rq[1] - 0.7).abs() < 1e-10);
    }

    #[test]
    fn test_relative_quantity_sum_to_one() {
        let bands: Vec<Band> = (0..5)
            .map(|i| Band {
                position_px: i as f64 * 10.0,
                position_mm: i as f64 * 10.0,
                height: 5.0,
                width: 3.0,
                area: (i + 1) as f64 * 10.0,
                snr: 10.0,
            })
            .collect();
        let rq = QuantificationEngine::relative_quantity(&bands);
        let sum: f64 = rq.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_concentration_from_standard() {
        let std_curve = vec![(10.0, 1.0), (20.0, 2.0), (30.0, 3.0), (40.0, 4.0)];
        let conc = QuantificationEngine::concentration_from_standard(25.0, &std_curve);
        assert!((conc - 2.5).abs() < 0.1);
    }

    #[test]
    fn test_background_noise_level() {
        let profile = LaneProfile::new(vec![0.0; 100], 1.0);
        let noise = QuantificationEngine::background_noise_level(&profile);
        assert!((noise - 0.0).abs() < 1e-10);
    }

    // ---- RfCalculator tests ----

    #[test]
    fn test_rf_calculation() {
        // Band at 30mm, dye at 100mm, well at 0mm => Rf = 0.3.
        let rf = RfCalculator::calculate_rf(30.0, 100.0, 0.0);
        assert!((rf - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_rf_zero_distance() {
        let rf = RfCalculator::calculate_rf(0.0, 0.0, 0.0);
        assert!((rf - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_rf_at_dye_front() {
        let rf = RfCalculator::calculate_rf(100.0, 100.0, 0.0);
        assert!((rf - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_rf_at_well() {
        let rf = RfCalculator::calculate_rf(5.0, 100.0, 5.0);
        assert!((rf - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_band_resolution() {
        let b1 = Band {
            position_px: 30.0,
            position_mm: 30.0,
            height: 5.0,
            width: 4.0,
            area: 10.0,
            snr: 10.0,
        };
        let b2 = Band {
            position_px: 40.0,
            position_mm: 40.0,
            height: 5.0,
            width: 4.0,
            area: 10.0,
            snr: 10.0,
        };
        let r = RfCalculator::band_resolution(&b1, &b2);
        // R = 2*10/(4+4) = 2.5
        assert!((r - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_rf_to_mw() {
        let cal = MwCalibration {
            slope: -0.05,
            intercept: 4.5,
            r_squared: 0.99,
        };
        let mw = RfCalculator::rf_to_mw(0.5, &cal, 100.0);
        // Position = 50mm, log_mw = -0.05*50 + 4.5 = 2.0, MW = 100.
        assert!((mw - 100.0).abs() < 1.0);
    }

    // ---- GelSimulator tests ----

    #[test]
    fn test_simulate_lane_no_noise() {
        let profile = GelSimulator::simulate_lane(&[(50.0, 10.0, 3.0)], 100, 1.0, 0.0);
        assert_eq!(profile.len(), 100);
        // Peak should be at position 50.
        let max_idx = profile
            .intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 50);
    }

    #[test]
    fn test_simulate_lane_with_noise() {
        let profile = GelSimulator::simulate_lane(&[(50.0, 10.0, 3.0)], 100, 1.0, 0.5);
        assert_eq!(profile.len(), 100);
        // Peak should still be near 50.
        let max_idx = profile
            .intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!((max_idx as i64 - 50).abs() < 5);
    }

    #[test]
    fn test_simulate_ladder() {
        let mw_sizes = vec![10000.0, 5000.0, 2000.0, 1000.0, 500.0];
        let profile = GelSimulator::simulate_ladder(&mw_sizes, 1.0, 200, 0.5);
        assert_eq!(profile.len(), 200);
        // Should have detectable peaks.
        let detector = BandDetector::new(0.1, 1.0, 0.05);
        let bands = detector.detect(&profile);
        assert!(bands.len() >= 3);
    }

    #[test]
    fn test_smile_effect() {
        let mut profiles: Vec<LaneProfile> = (0..5)
            .map(|_| GelSimulator::simulate_lane(&[(50.0, 10.0, 3.0)], 100, 1.0, 0.0))
            .collect();
        GelSimulator::add_smile_effect(&mut profiles, 0.5);
        // Edge lanes should have shifted peak positions.
        let center_max = profiles[2]
            .intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let edge_max = profiles[0]
            .intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // Center should be closer to 50 than edge.
        assert!((center_max as i64 - 50).abs() <= (edge_max as i64 - 50).abs());
    }

    #[test]
    fn test_loading_variation() {
        let mut profiles: Vec<LaneProfile> = (0..5)
            .map(|_| GelSimulator::simulate_lane(&[(50.0, 10.0, 3.0)], 100, 1.0, 0.0))
            .collect();
        GelSimulator::add_loading_variation(&mut profiles, 0.3);
        // Not all lanes should have same peak height.
        let heights: Vec<f64> = profiles
            .iter()
            .map(|p| {
                p.intensities
                    .iter()
                    .cloned()
                    .fold(f64::NEG_INFINITY, f64::max)
            })
            .collect();
        let all_same = heights.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-10);
        assert!(!all_same);
    }

    // ---- BaselineEstimator tests ----

    #[test]
    fn test_rolling_ball_flat() {
        let data = vec![5.0; 50];
        let baseline = BaselineEstimator::rolling_ball(&data, 10);
        for v in &baseline {
            assert!((v - 5.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_rolling_ball_with_peak() {
        let mut data = vec![1.0; 100];
        data[50] = 10.0;
        let baseline = BaselineEstimator::rolling_ball(&data, 10);
        // Baseline at peak position should be close to 1.0 (the background).
        assert!(baseline[50] < 5.0);
    }

    #[test]
    fn test_linear_baseline() {
        let data: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let baseline = BaselineEstimator::linear_baseline(&data);
        for (i, v) in baseline.iter().enumerate() {
            assert!((v - data[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_polynomial_baseline_flat() {
        let data = vec![3.0; 50];
        let baseline = BaselineEstimator::polynomial_baseline(&data, 1);
        for v in &baseline {
            assert!((v - 3.0).abs() < 0.5);
        }
    }

    #[test]
    fn test_polynomial_baseline_degree_0() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let baseline = BaselineEstimator::polynomial_baseline(&data, 0);
        let mean = 3.0;
        for v in &baseline {
            assert!((v - mean).abs() < 1e-10);
        }
    }

    #[test]
    fn test_asymmetric_least_squares() {
        let mut data = vec![1.0; 100];
        // Add a peak.
        for i in 40..60 {
            data[i] = 1.0 + 5.0 * (-(((i as f64) - 50.0).powi(2)) / 20.0).exp();
        }
        let baseline = BaselineEstimator::asymmetric_least_squares(&data, 100.0, 0.01);
        // Baseline at peak should be lower than the raw peak value.
        assert!(
            baseline[50] < data[50],
            "Baseline {} should be less than data {}",
            baseline[50],
            data[50]
        );
        // And baseline in flat regions should be reasonable.
        assert!(
            baseline[5] < 2.0,
            "Baseline at flat region {} should be close to 1.0",
            baseline[5]
        );
    }

    #[test]
    fn test_baseline_empty() {
        let baseline = BaselineEstimator::rolling_ball(&[], 5);
        assert!(baseline.is_empty());
    }

    // ---- FragmentSizer tests ----

    #[test]
    fn test_ferguson_plot() {
        // Synthetic data: mobility decreases with gel percentage.
        let mobility = vec![10.0, 5.0, 2.5, 1.25];
        let gel_pct = vec![0.5, 1.0, 1.5, 2.0];
        let result = FragmentSizer::ferguson_plot(&mobility, &gel_pct);
        assert!(result.retardation_coefficient > 0.0);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_effective_length_linear() {
        let bp = FragmentSizer::effective_length(6600.0, true);
        assert!((bp - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_effective_length_supercoiled() {
        let bp_linear = FragmentSizer::effective_length(6600.0, true);
        let bp_sc = FragmentSizer::effective_length(6600.0, false);
        assert!(bp_sc < bp_linear);
        assert!((bp_sc / bp_linear - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_mw_to_bp() {
        assert!((FragmentSizer::mw_to_bp(660.0) - 1.0).abs() < 1e-10);
        assert!((FragmentSizer::mw_to_bp(6600.0) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_bp_to_mw() {
        assert!((FragmentSizer::bp_to_mw(1.0) - 660.0).abs() < 1e-10);
        assert!((FragmentSizer::bp_to_mw(1000.0) - 660000.0).abs() < 1e-10);
    }

    #[test]
    fn test_mw_bp_roundtrip() {
        let bp = 500.0;
        let mw = FragmentSizer::bp_to_mw(bp);
        let bp2 = FragmentSizer::mw_to_bp(mw);
        assert!((bp - bp2).abs() < 1e-10);
    }

    // ---- Integration / end-to-end tests ----

    #[test]
    fn test_full_pipeline_simulate_detect_calibrate() {
        // Simulate a ladder.
        let mw_sizes = vec![10000.0, 5000.0, 2000.0, 1000.0, 500.0];
        let profile = GelSimulator::simulate_ladder(&mw_sizes, 1.0, 200, 0.01);

        // Detect bands.
        let detector = BandDetector::new(0.05, 1.0, 0.03);
        let bands = detector.detect(&profile);
        assert!(bands.len() >= 3, "Expected >= 3 bands, got {}", bands.len());

        // Quantify.
        let rq = QuantificationEngine::relative_quantity(&bands);
        let sum: f64 = rq.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_multi_lane_analysis() {
        let mut analyzer = LaneAnalyzer::new(3);

        // Lane 0 and 1 have similar bands.
        let p0 = GelSimulator::simulate_lane(
            &[(30.0, 10.0, 3.0), (60.0, 8.0, 3.0)],
            100,
            1.0,
            0.0,
        );
        let p1 = GelSimulator::simulate_lane(
            &[(30.0, 9.0, 3.0), (60.0, 7.0, 3.0)],
            100,
            1.0,
            0.0,
        );
        // Lane 2 has different bands.
        let p2 = GelSimulator::simulate_lane(
            &[(20.0, 10.0, 3.0), (80.0, 8.0, 3.0)],
            100,
            1.0,
            0.0,
        );

        analyzer.add_lane(0, p0);
        analyzer.add_lane(1, p1);
        analyzer.add_lane(2, p2);

        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let all_bands = analyzer.detect_all_bands(&detector);
        assert_eq!(all_bands.len(), 3);

        // Lanes 0,1 should be more similar than 0,2.
        let sim_01 = analyzer.lane_similarity(0, 1, &detector, 3.0);
        let sim_02 = analyzer.lane_similarity(0, 2, &detector, 3.0);
        assert!(sim_01 > sim_02);
    }

    #[test]
    fn test_baseline_correction_preserves_peaks() {
        let bands_spec = vec![(50.0, 10.0, 3.0)];
        let profile = GelSimulator::simulate_lane(&bands_spec, 100, 1.0, 0.0);
        let corrected = profile.baseline_correct(BaselineMethod::RollingBall(15));
        // Peak should still be present.
        let detector = BandDetector::new(0.5, 1.0, 0.5);
        let bands = detector.detect(&corrected);
        assert!(!bands.is_empty());
    }

    #[test]
    fn test_smoothing_then_detection() {
        // Noisy signal.
        let profile = GelSimulator::simulate_lane(
            &[(30.0, 10.0, 3.0), (70.0, 8.0, 3.0)],
            100,
            1.0,
            1.0,
        );
        let smoothed = profile.smooth(5);
        let detector = BandDetector::new(2.0, 1.0, 1.0);
        let bands = detector.detect(&smoothed);
        assert!(bands.len() >= 2);
    }

    #[test]
    fn test_dendrogram_preserves_sizes() {
        let sim = vec![
            vec![1.0, 0.9, 0.1, 0.2],
            vec![0.9, 1.0, 0.2, 0.15],
            vec![0.1, 0.2, 1.0, 0.8],
            vec![0.2, 0.15, 0.8, 1.0],
        ];
        let nodes = LaneAnalyzer::dendrogram(&sim);
        assert_eq!(nodes.len(), 3);
        // Last merge should contain all 4 items.
        assert_eq!(nodes.last().unwrap().size, 4);
    }

    #[test]
    fn test_gel_simulator_empty_bands() {
        let profile = GelSimulator::simulate_lane(&[], 100, 1.0, 0.0);
        assert_eq!(profile.len(), 100);
        assert!(profile.intensities.iter().all(|v| *v == 0.0));
    }

    #[test]
    fn test_noise_level_estimation() {
        // Pure noise should give non-zero noise estimate.
        let profile = GelSimulator::simulate_lane(&[], 1000, 1.0, 1.0);
        let noise = QuantificationEngine::background_noise_level(&profile);
        assert!(noise > 0.0);
    }
}
