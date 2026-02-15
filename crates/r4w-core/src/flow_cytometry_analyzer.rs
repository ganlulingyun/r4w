//! # Flow Cytometry Signal Analyzer
//!
//! This module implements flow cytometry signal processing for analyzing
//! optical and fluorescence properties of cells and particles as they pass
//! through a laser beam in a flow cytometer.
//!
//! ## Background
//!
//! Flow cytometry measures multiple physical characteristics of single cells
//! or particles in a fluid stream. As cells pass through focused laser beams,
//! scattered and fluorescent light is collected by detectors:
//!
//! - **Forward Scatter (FSC)**: Proportional to cell size (diffraction)
//! - **Side Scatter (SSC)**: Proportional to internal complexity/granularity
//! - **Fluorescence Channels**: Detect fluorochrome-labeled antibodies or
//!   intrinsic fluorescence (e.g., FITC, PE, APC, PerCP)
//!
//! ## Applications
//!
//! - Immunophenotyping (CD4/CD8 T-cell counts for HIV monitoring)
//! - Cancer diagnosis (DNA content analysis, minimal residual disease)
//! - Hematology (complete blood count, reticulocyte counting)
//! - Cell cycle analysis (G0/G1, S, G2/M phases)
//! - Apoptosis detection (Annexin V / PI staining)
//!
//! ## Signal Processing Pipeline
//!
//! ```text
//! Raw PMT signal -> Pulse Detection -> Parameter Extraction -> Compensation
//!                                          |
//!                     Gating -> Clustering -> Population Statistics
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::flow_cytometry_analyzer::*;
//!
//! let config = CytometryConfig {
//!     sample_rate_hz: 10_000_000.0,
//!     num_fluorescence_channels: 4,
//!     laser_wavelength_nm: 488.0,
//!     trigger_channel: TriggerChannel::ForwardScatter,
//!     trigger_threshold: 0.1,
//! };
//!
//! let processor = CytometryProcessor::new(config);
//!
//! // Detect pulse events in raw PMT signal
//! let signal = vec![0.0; 100]; // would be real PMT data
//! let events = processor.detect_events(&signal, 0.1);
//! ```

/// Configuration for the flow cytometry processor.
#[derive(Debug, Clone)]
pub struct CytometryConfig {
    /// Digitizer sample rate in Hz (typically 10-100 MHz).
    pub sample_rate_hz: f64,
    /// Number of fluorescence detection channels (typically 2-18).
    pub num_fluorescence_channels: usize,
    /// Excitation laser wavelength in nanometers (e.g., 488, 561, 633).
    pub laser_wavelength_nm: f64,
    /// Which channel triggers event acquisition.
    pub trigger_channel: TriggerChannel,
    /// Minimum signal level to trigger event detection.
    pub trigger_threshold: f64,
}

/// Selects the channel used for event triggering.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TriggerChannel {
    /// Trigger on forward scatter (most common).
    ForwardScatter,
    /// Trigger on side scatter.
    SideScatter,
    /// Trigger on a specific fluorescence channel index.
    Fluorescence(usize),
}

/// Represents a single cell or particle event detected by the cytometer.
///
/// Each event contains scatter parameters, fluorescence intensities, and
/// pulse shape characteristics extracted from the raw PMT signals.
#[derive(Debug, Clone)]
pub struct CellEvent {
    /// Forward scatter intensity (proportional to cell size).
    pub fsc: f64,
    /// Side scatter intensity (proportional to granularity/complexity).
    pub ssc: f64,
    /// Fluorescence channel intensities (one per configured channel).
    pub fluorescence: Vec<f64>,
    /// Time of event in microseconds from acquisition start.
    pub time_us: f64,
    /// Pulse width at half-maximum in sample units.
    pub pulse_width: f64,
    /// Peak pulse height (maximum signal value).
    pub pulse_height: f64,
    /// Integrated pulse area (sum of signal over pulse duration).
    pub pulse_area: f64,
}

/// Statistics for a population of gated events.
#[derive(Debug, Clone)]
pub struct PopulationStats {
    /// Number of events in the population.
    pub count: usize,
    /// Mean forward scatter.
    pub mean_fsc: f64,
    /// Mean side scatter.
    pub mean_ssc: f64,
    /// Standard deviation of forward scatter.
    pub std_fsc: f64,
    /// Standard deviation of side scatter.
    pub std_ssc: f64,
    /// Mean fluorescence per channel.
    pub mean_fluorescence: Vec<f64>,
    /// Standard deviation of fluorescence per channel.
    pub std_fluorescence: Vec<f64>,
    /// Coefficient of variation for FSC (%).
    pub cv_fsc: f64,
    /// Coefficient of variation for SSC (%).
    pub cv_ssc: f64,
}

/// Main flow cytometry signal processor.
///
/// Handles event detection from raw PMT signals, pulse parameter extraction,
/// gating, and spectral compensation.
pub struct CytometryProcessor {
    config: CytometryConfig,
}

impl CytometryProcessor {
    /// Creates a new cytometry processor with the given configuration.
    ///
    /// # Arguments
    /// * `config` - Instrument configuration parameters
    pub fn new(config: CytometryConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the current configuration.
    pub fn config(&self) -> &CytometryConfig {
        &self.config
    }

    /// Detects pulse events in a raw PMT signal by threshold crossing.
    ///
    /// Scans the signal for regions where the amplitude exceeds the given
    /// threshold. Each contiguous above-threshold region is reported as a
    /// `(start, end)` sample index pair (end is exclusive).
    ///
    /// # Arguments
    /// * `signal` - Raw digitized PMT signal
    /// * `threshold` - Minimum amplitude for event detection
    ///
    /// # Returns
    /// Vector of `(start_index, end_index)` for each detected pulse
    pub fn detect_events(&self, signal: &[f64], threshold: f64) -> Vec<(usize, usize)> {
        let mut events = Vec::new();
        let mut in_pulse = false;
        let mut start = 0;

        for (i, &sample) in signal.iter().enumerate() {
            if !in_pulse && sample >= threshold {
                in_pulse = true;
                start = i;
            } else if in_pulse && sample < threshold {
                in_pulse = false;
                events.push((start, i));
            }
        }

        // Handle pulse that extends to end of signal
        if in_pulse {
            events.push((start, signal.len()));
        }

        events
    }

    /// Extracts pulse shape parameters from a single pulse waveform.
    ///
    /// Computes pulse height (peak value), pulse width (number of samples
    /// at or above half-maximum), and pulse area (trapezoidal integral).
    ///
    /// # Arguments
    /// * `pulse` - Slice of samples for one detected pulse
    ///
    /// # Returns
    /// Tuple of `(height, width, area)` where:
    /// - `height`: Maximum amplitude in the pulse
    /// - `width`: Number of samples at or above half-maximum
    /// - `area`: Sum of all sample values (trapezoidal approximation)
    pub fn extract_pulse_parameters(&self, pulse: &[f64]) -> (f64, f64, f64) {
        if pulse.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        // Height: peak value
        let height = pulse.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Width: count samples at or above half-maximum
        let half_max = height / 2.0;
        let width = pulse.iter().filter(|&&v| v >= half_max).count() as f64;

        // Area: trapezoidal integration (approximated as sum of samples)
        let area: f64 = pulse.iter().sum();

        (height, width, area)
    }

    /// Applies polygon gating to select events within a 2D region.
    ///
    /// Uses the ray-casting algorithm to determine which events fall inside
    /// the polygon defined by the given vertices in the coordinate space
    /// defined by `param_x` and `param_y` accessor functions.
    ///
    /// # Arguments
    /// * `events` - Slice of cell events to gate
    /// * `vertices` - Polygon vertices as `(x, y)` pairs (closed automatically)
    /// * `param_x` - Function to extract X coordinate from a CellEvent
    /// * `param_y` - Function to extract Y coordinate from a CellEvent
    ///
    /// # Returns
    /// Indices of events that fall inside the polygon gate
    pub fn gate_polygon(
        &self,
        events: &[CellEvent],
        vertices: &[(f64, f64)],
        param_x: fn(&CellEvent) -> f64,
        param_y: fn(&CellEvent) -> f64,
    ) -> Vec<usize> {
        events
            .iter()
            .enumerate()
            .filter(|(_, evt)| {
                let x = param_x(evt);
                let y = param_y(evt);
                point_in_polygon(x, y, vertices)
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// Applies rectangular gating to select events within an axis-aligned region.
    ///
    /// Selects events where both the X and Y parameter values fall within
    /// the specified ranges.
    ///
    /// # Arguments
    /// * `events` - Slice of cell events to gate
    /// * `x_range` - `(min, max)` range for the X parameter
    /// * `y_range` - `(min, max)` range for the Y parameter
    /// * `param_x` - Function to extract X coordinate from a CellEvent
    /// * `param_y` - Function to extract Y coordinate from a CellEvent
    ///
    /// # Returns
    /// Indices of events that fall inside the rectangular gate
    pub fn gate_rectangle(
        &self,
        events: &[CellEvent],
        x_range: (f64, f64),
        y_range: (f64, f64),
        param_x: fn(&CellEvent) -> f64,
        param_y: fn(&CellEvent) -> f64,
    ) -> Vec<usize> {
        events
            .iter()
            .enumerate()
            .filter(|(_, evt)| {
                let x = param_x(evt);
                let y = param_y(evt);
                x >= x_range.0 && x <= x_range.1 && y >= y_range.0 && y <= y_range.1
            })
            .map(|(i, _)| i)
            .collect()
    }

    /// Applies spectral compensation (spillover correction) to fluorescence data.
    ///
    /// In multi-color flow cytometry, fluorochrome emission spectra overlap,
    /// causing spillover between detection channels. The compensation matrix
    /// corrects this by subtracting the spillover contributions.
    ///
    /// The compensation matrix should be the inverse of the spillover matrix.
    /// Each row `i` gives the coefficients to apply to the raw fluorescence
    /// values to produce the corrected value for channel `i`:
    ///
    /// `corrected[i] = sum_j(matrix[i][j] * raw[j])`
    ///
    /// # Arguments
    /// * `events` - Mutable slice of cell events to compensate
    /// * `matrix` - Compensation matrix (N x N where N = number of fluorescence channels)
    pub fn compensation_matrix_apply(&self, events: &mut [CellEvent], matrix: &[Vec<f64>]) {
        let n = self.config.num_fluorescence_channels;

        for event in events.iter_mut() {
            if event.fluorescence.len() != n {
                continue;
            }

            let raw = event.fluorescence.clone();
            for i in 0..n {
                if i >= matrix.len() || matrix[i].len() != n {
                    continue;
                }
                let mut val = 0.0;
                for j in 0..n {
                    val += matrix[i][j] * raw[j];
                }
                event.fluorescence[i] = val;
            }
        }
    }
}

/// Histogram analysis tools for single-parameter distributions.
///
/// Provides methods for building histograms, finding peaks, and computing
/// statistical measures commonly used in flow cytometry data analysis
/// (e.g., DNA content histograms for cell cycle analysis).
pub struct HistogramAnalyzer;

impl HistogramAnalyzer {
    /// Builds a histogram from a slice of values.
    ///
    /// Divides the range `[min, max]` of the input values into `num_bins`
    /// equal-width bins and counts the number of values falling into each bin.
    ///
    /// # Arguments
    /// * `values` - Input data values
    /// * `num_bins` - Number of histogram bins
    ///
    /// # Returns
    /// Tuple of `(bin_edges, counts)` where `bin_edges` has length `num_bins + 1`
    /// and `counts` has length `num_bins`.
    pub fn build_histogram(values: &[f64], num_bins: usize) -> (Vec<f64>, Vec<usize>) {
        if values.is_empty() || num_bins == 0 {
            return (vec![], vec![]);
        }

        let min_val = values.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Handle case where all values are identical
        let range = if (max_val - min_val).abs() < f64::EPSILON {
            1.0
        } else {
            max_val - min_val
        };

        let bin_width = range / num_bins as f64;

        // Create bin edges
        let bin_edges: Vec<f64> = (0..=num_bins)
            .map(|i| min_val + i as f64 * bin_width)
            .collect();

        // Count values in each bin
        let mut counts = vec![0usize; num_bins];
        for &v in values {
            let mut bin = ((v - min_val) / bin_width) as usize;
            // Clamp the last value to fall in the last bin
            if bin >= num_bins {
                bin = num_bins - 1;
            }
            counts[bin] += 1;
        }

        (bin_edges, counts)
    }

    /// Finds peaks in a histogram by looking for local maxima above a minimum height.
    ///
    /// A peak is a bin whose count is greater than both its neighbors and
    /// exceeds the specified minimum height threshold.
    ///
    /// # Arguments
    /// * `histogram` - Bin counts from `build_histogram`
    /// * `min_height` - Minimum count for a bin to be considered a peak
    ///
    /// # Returns
    /// Indices of bins that are local maxima above the threshold
    pub fn find_peaks(histogram: &[usize], min_height: usize) -> Vec<usize> {
        let mut peaks = Vec::new();

        if histogram.len() < 2 {
            if histogram.len() == 1 && histogram[0] >= min_height {
                peaks.push(0);
            }
            return peaks;
        }

        // Check first bin
        if histogram[0] >= min_height && histogram[0] > histogram[1] {
            peaks.push(0);
        }

        // Check interior bins
        for i in 1..histogram.len() - 1 {
            if histogram[i] >= min_height
                && histogram[i] > histogram[i - 1]
                && histogram[i] > histogram[i + 1]
            {
                peaks.push(i);
            }
        }

        // Check last bin
        let last = histogram.len() - 1;
        if histogram[last] >= min_height && histogram[last] > histogram[last - 1] {
            peaks.push(last);
        }

        peaks
    }

    /// Computes the coefficient of variation (CV) of a set of values.
    ///
    /// CV = (standard deviation / mean) * 100%, commonly used to assess
    /// instrument precision (e.g., CV of fluorescent beads should be < 3%).
    ///
    /// # Arguments
    /// * `values` - Input data values
    ///
    /// # Returns
    /// CV as a percentage (0-100+), or 0.0 if mean is zero
    pub fn coefficient_of_variation(values: &[f64]) -> f64 {
        if values.is_empty() {
            return 0.0;
        }

        let n = values.len() as f64;
        let mean = values.iter().sum::<f64>() / n;

        if mean.abs() < f64::EPSILON {
            return 0.0;
        }

        let variance = values.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>() / n;
        let std_dev = variance.sqrt();

        (std_dev / mean.abs()) * 100.0
    }

    /// Computes the median of a set of values.
    ///
    /// # Arguments
    /// * `values` - Input data values
    ///
    /// # Returns
    /// The median value, or 0.0 if the input is empty
    pub fn median(values: &[f64]) -> f64 {
        if values.is_empty() {
            return 0.0;
        }

        let mut sorted = values.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let n = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        } else {
            sorted[n / 2]
        }
    }

    /// Computes the p-th percentile of a set of values.
    ///
    /// Uses linear interpolation between adjacent data points when the
    /// percentile falls between two values.
    ///
    /// # Arguments
    /// * `values` - Input data values
    /// * `p` - Percentile (0.0 to 100.0)
    ///
    /// # Returns
    /// The interpolated percentile value, or 0.0 if input is empty
    pub fn percentile(values: &[f64], p: f64) -> f64 {
        if values.is_empty() {
            return 0.0;
        }

        let mut sorted = values.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let p_clamped = p.clamp(0.0, 100.0);
        let n = sorted.len();

        if n == 1 {
            return sorted[0];
        }

        // Compute rank using the C = 1 interpolation method
        let rank = (p_clamped / 100.0) * (n - 1) as f64;
        let lower = rank.floor() as usize;
        let upper = rank.ceil() as usize;
        let frac = rank - lower as f64;

        if upper >= n {
            sorted[n - 1]
        } else {
            sorted[lower] * (1.0 - frac) + sorted[upper] * frac
        }
    }

    /// Computes the geometric mean of a set of positive values.
    ///
    /// The geometric mean is commonly used in flow cytometry for fluorescence
    /// intensity because fluorescence data is typically log-normally distributed.
    ///
    /// # Arguments
    /// * `values` - Input data values (must be positive)
    ///
    /// # Returns
    /// The geometric mean, or 0.0 if input is empty or contains non-positive values
    pub fn geometric_mean(values: &[f64]) -> f64 {
        if values.is_empty() {
            return 0.0;
        }

        // Use log-sum-exp for numerical stability
        let log_sum: f64 = values
            .iter()
            .map(|&v| {
                if v > 0.0 {
                    v.ln()
                } else {
                    return f64::NEG_INFINITY;
                }
            })
            .sum();

        if log_sum.is_infinite() {
            return 0.0;
        }

        (log_sum / values.len() as f64).exp()
    }
}

/// Population analysis tools for multi-parameter clustering and statistics.
pub struct PopulationAnalyzer;

impl PopulationAnalyzer {
    /// Performs k-means clustering on cell events using specified feature extractors.
    ///
    /// Partitions events into `k` clusters by iteratively assigning events to the
    /// nearest cluster centroid and recomputing centroids until convergence or a
    /// maximum number of iterations is reached.
    ///
    /// # Arguments
    /// * `events` - Slice of cell events to cluster
    /// * `k` - Number of clusters
    /// * `features` - Feature extraction functions (each maps CellEvent -> f64)
    ///
    /// # Returns
    /// Vector of cluster assignments (0..k) for each event
    pub fn kmeans_cluster(
        events: &[CellEvent],
        k: usize,
        features: &[fn(&CellEvent) -> f64],
    ) -> Vec<usize> {
        if events.is_empty() || k == 0 || features.is_empty() {
            return vec![];
        }

        let n = events.len();
        let d = features.len();
        let k = k.min(n);

        // Extract feature matrix
        let data: Vec<Vec<f64>> = events
            .iter()
            .map(|evt| features.iter().map(|f| f(evt)).collect())
            .collect();

        // Initialize centroids using evenly spaced events
        let mut centroids: Vec<Vec<f64>> = (0..k)
            .map(|i| {
                let idx = i * n / k;
                data[idx].clone()
            })
            .collect();

        let mut assignments = vec![0usize; n];
        let max_iter = 100;

        for _ in 0..max_iter {
            let mut changed = false;

            // Assignment step: assign each point to nearest centroid
            for i in 0..n {
                let mut best_cluster = 0;
                let mut best_dist = f64::INFINITY;

                for c in 0..k {
                    let dist: f64 = (0..d)
                        .map(|j| {
                            let diff = data[i][j] - centroids[c][j];
                            diff * diff
                        })
                        .sum();

                    if dist < best_dist {
                        best_dist = dist;
                        best_cluster = c;
                    }
                }

                if assignments[i] != best_cluster {
                    assignments[i] = best_cluster;
                    changed = true;
                }
            }

            if !changed {
                break;
            }

            // Update step: recompute centroids
            let mut new_centroids = vec![vec![0.0; d]; k];
            let mut counts = vec![0usize; k];

            for i in 0..n {
                let c = assignments[i];
                counts[c] += 1;
                for j in 0..d {
                    new_centroids[c][j] += data[i][j];
                }
            }

            for c in 0..k {
                if counts[c] > 0 {
                    for j in 0..d {
                        new_centroids[c][j] /= counts[c] as f64;
                    }
                } else {
                    // Keep old centroid if no points assigned
                    new_centroids[c] = centroids[c].clone();
                }
            }

            centroids = new_centroids;
        }

        assignments
    }

    /// Computes population statistics for a subset of events.
    ///
    /// Calculates mean, standard deviation, and CV for scatter and fluorescence
    /// parameters of the events identified by the given indices.
    ///
    /// # Arguments
    /// * `events` - Full slice of cell events
    /// * `indices` - Indices of events in this population
    ///
    /// # Returns
    /// Population statistics summary
    pub fn population_statistics(events: &[CellEvent], indices: &[usize]) -> PopulationStats {
        if indices.is_empty() {
            return PopulationStats {
                count: 0,
                mean_fsc: 0.0,
                mean_ssc: 0.0,
                std_fsc: 0.0,
                std_ssc: 0.0,
                mean_fluorescence: vec![],
                std_fluorescence: vec![],
                cv_fsc: 0.0,
                cv_ssc: 0.0,
            };
        }

        let n = indices.len() as f64;

        // Compute FSC stats
        let fsc_values: Vec<f64> = indices.iter().map(|&i| events[i].fsc).collect();
        let mean_fsc = fsc_values.iter().sum::<f64>() / n;
        let var_fsc = fsc_values.iter().map(|&v| (v - mean_fsc).powi(2)).sum::<f64>() / n;
        let std_fsc = var_fsc.sqrt();

        // Compute SSC stats
        let ssc_values: Vec<f64> = indices.iter().map(|&i| events[i].ssc).collect();
        let mean_ssc = ssc_values.iter().sum::<f64>() / n;
        let var_ssc = ssc_values.iter().map(|&v| (v - mean_ssc).powi(2)).sum::<f64>() / n;
        let std_ssc = var_ssc.sqrt();

        // Compute fluorescence stats per channel
        let num_fl = if let Some(first) = indices.first() {
            events[*first].fluorescence.len()
        } else {
            0
        };

        let mut mean_fluorescence = vec![0.0; num_fl];
        let mut std_fluorescence = vec![0.0; num_fl];

        for ch in 0..num_fl {
            let fl_vals: Vec<f64> = indices.iter().map(|&i| events[i].fluorescence[ch]).collect();
            let mean = fl_vals.iter().sum::<f64>() / n;
            let var = fl_vals.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n;
            mean_fluorescence[ch] = mean;
            std_fluorescence[ch] = var.sqrt();
        }

        let cv_fsc = if mean_fsc.abs() > f64::EPSILON {
            (std_fsc / mean_fsc.abs()) * 100.0
        } else {
            0.0
        };

        let cv_ssc = if mean_ssc.abs() > f64::EPSILON {
            (std_ssc / mean_ssc.abs()) * 100.0
        } else {
            0.0
        };

        PopulationStats {
            count: indices.len(),
            mean_fsc,
            mean_ssc,
            std_fsc,
            std_ssc,
            mean_fluorescence,
            std_fluorescence,
            cv_fsc,
            cv_ssc,
        }
    }

    /// Discriminates doublets from singlets using pulse width/area ratio.
    ///
    /// When two cells pass through the laser simultaneously (doublets), the
    /// pulse width increases relative to pulse area. True singlets have a
    /// consistent height-to-area ratio.
    ///
    /// Events with `pulse_width / pulse_area` below the threshold are
    /// considered singlets (returned).
    ///
    /// # Arguments
    /// * `events` - Slice of cell events
    /// * `ratio_threshold` - Maximum width/area ratio for singlet classification
    ///
    /// # Returns
    /// Indices of events classified as singlets (not doublets)
    pub fn doublet_discrimination(events: &[CellEvent], ratio_threshold: f64) -> Vec<usize> {
        events
            .iter()
            .enumerate()
            .filter(|(_, evt)| {
                if evt.pulse_area.abs() < f64::EPSILON {
                    return false;
                }
                let ratio = evt.pulse_width / evt.pulse_area;
                ratio <= ratio_threshold
            })
            .map(|(i, _)| i)
            .collect()
    }
}

/// Tests whether a point lies inside a polygon using the ray-casting algorithm.
///
/// Casts a horizontal ray from the test point to the right and counts
/// intersections with polygon edges. An odd count means the point is inside.
///
/// # Arguments
/// * `x` - Test point X coordinate
/// * `y` - Test point Y coordinate
/// * `vertices` - Polygon vertices as `(x, y)` pairs
///
/// # Returns
/// `true` if the point is inside the polygon
pub fn point_in_polygon(x: f64, y: f64, vertices: &[(f64, f64)]) -> bool {
    let n = vertices.len();
    if n < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = n - 1;

    for i in 0..n {
        let (xi, yi) = vertices[i];
        let (xj, yj) = vertices[j];

        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }

        j = i;
    }

    inside
}

/// Applies a logarithmic transform (base 10) to a value.
///
/// Commonly used in flow cytometry to display fluorescence data on a
/// logarithmic scale. Values <= 0 are mapped to a configurable floor.
///
/// # Arguments
/// * `value` - Input value
/// * `floor` - Minimum output value for non-positive inputs
///
/// # Returns
/// `log10(value)` if value > 0, otherwise `floor`
pub fn log_transform(value: f64, floor: f64) -> f64 {
    if value > 0.0 {
        value.log10()
    } else {
        floor
    }
}

/// Applies a biexponential (logicle-like) transform to a value.
///
/// The biexponential transform handles both positive and negative values
/// (common with compensated fluorescence data) by transitioning smoothly
/// from a linear region near zero to logarithmic scaling at larger values.
///
/// This is a simplified approximation:
/// - For `|x| > transition`: `sign(x) * (log10(|x|) + log10(transition))`
/// - For `|x| <= transition`: `x / transition`
///
/// The `transition` parameter controls where the linear-to-log crossover
/// occurs.
///
/// # Arguments
/// * `value` - Input value (can be negative after compensation)
/// * `transition` - Crossover point between linear and log regions
///
/// # Returns
/// The biexponentially transformed value
pub fn biexponential_transform(value: f64, transition: f64) -> f64 {
    let t = transition.abs().max(f64::EPSILON);

    if value.abs() <= t {
        // Linear region near zero
        value / t
    } else {
        // Logarithmic region
        let sign = if value >= 0.0 { 1.0 } else { -1.0 };
        let log_val = value.abs().log10();
        let log_t = t.log10();
        sign * (log_val - log_t + 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> CytometryConfig {
        CytometryConfig {
            sample_rate_hz: 10_000_000.0,
            num_fluorescence_channels: 4,
            laser_wavelength_nm: 488.0,
            trigger_channel: TriggerChannel::ForwardScatter,
            trigger_threshold: 0.1,
        }
    }

    fn make_event(fsc: f64, ssc: f64, fl: Vec<f64>) -> CellEvent {
        CellEvent {
            fsc,
            ssc,
            fluorescence: fl,
            time_us: 0.0,
            pulse_width: 5.0,
            pulse_height: fsc,
            pulse_area: fsc * 5.0,
        }
    }

    // --- Event Detection Tests ---

    #[test]
    fn test_detect_events_single_pulse() {
        let proc = CytometryProcessor::new(make_config());
        let signal = vec![0.0, 0.0, 0.5, 0.8, 0.6, 0.0, 0.0];
        let events = proc.detect_events(&signal, 0.3);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0], (2, 5));
    }

    #[test]
    fn test_detect_events_multiple_pulses() {
        let proc = CytometryProcessor::new(make_config());
        let signal = vec![0.0, 0.5, 0.0, 0.0, 0.7, 0.9, 0.0];
        let events = proc.detect_events(&signal, 0.3);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0], (1, 2));
        assert_eq!(events[1], (4, 6));
    }

    #[test]
    fn test_detect_events_no_pulses() {
        let proc = CytometryProcessor::new(make_config());
        let signal = vec![0.0, 0.1, 0.2, 0.1, 0.0];
        let events = proc.detect_events(&signal, 0.3);
        assert_eq!(events.len(), 0);
    }

    #[test]
    fn test_detect_events_pulse_at_end() {
        let proc = CytometryProcessor::new(make_config());
        let signal = vec![0.0, 0.0, 0.5, 0.8];
        let events = proc.detect_events(&signal, 0.3);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0], (2, 4));
    }

    #[test]
    fn test_detect_events_empty_signal() {
        let proc = CytometryProcessor::new(make_config());
        let events = proc.detect_events(&[], 0.3);
        assert_eq!(events.len(), 0);
    }

    // --- Pulse Parameter Extraction Tests ---

    #[test]
    fn test_pulse_area_is_integral() {
        let proc = CytometryProcessor::new(make_config());
        let pulse = vec![0.1, 0.5, 1.0, 0.8, 0.3];
        let (height, _width, area) = proc.extract_pulse_parameters(&pulse);
        let expected_area: f64 = pulse.iter().sum();
        assert!((area - expected_area).abs() < 1e-10);
        assert!((height - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_pulse_height_is_max() {
        let proc = CytometryProcessor::new(make_config());
        let pulse = vec![0.2, 0.8, 1.5, 0.6];
        let (height, _, _) = proc.extract_pulse_parameters(&pulse);
        assert!((height - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_pulse_width_half_max() {
        let proc = CytometryProcessor::new(make_config());
        // Peak = 1.0, half max = 0.5, samples >= 0.5: [0.5, 1.0, 0.7]
        let pulse = vec![0.1, 0.5, 1.0, 0.7, 0.2];
        let (_, width, _) = proc.extract_pulse_parameters(&pulse);
        assert!((width - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_pulse_parameters_empty() {
        let proc = CytometryProcessor::new(make_config());
        let (h, w, a) = proc.extract_pulse_parameters(&[]);
        assert_eq!(h, 0.0);
        assert_eq!(w, 0.0);
        assert_eq!(a, 0.0);
    }

    // --- Polygon Gating Tests ---

    #[test]
    fn test_polygon_gate_inside() {
        let proc = CytometryProcessor::new(make_config());
        let events = vec![
            make_event(5.0, 5.0, vec![1.0; 4]),  // inside
            make_event(15.0, 15.0, vec![1.0; 4]), // outside
        ];
        // Square gate: (0,0) to (10,10)
        let vertices = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let inside =
            proc.gate_polygon(&events, &vertices, |e| e.fsc, |e| e.ssc);
        assert_eq!(inside, vec![0]);
    }

    #[test]
    fn test_polygon_gate_outside() {
        let proc = CytometryProcessor::new(make_config());
        let events = vec![make_event(15.0, 15.0, vec![1.0; 4])];
        let vertices = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let inside =
            proc.gate_polygon(&events, &vertices, |e| e.fsc, |e| e.ssc);
        assert!(inside.is_empty());
    }

    #[test]
    fn test_polygon_gate_triangle() {
        let proc = CytometryProcessor::new(make_config());
        let events = vec![
            make_event(2.0, 1.0, vec![]),  // inside triangle
            make_event(5.0, 8.0, vec![]),  // outside triangle
        ];
        // Triangle with vertices at (0,0), (6,0), (3,6)
        let vertices = vec![(0.0, 0.0), (6.0, 0.0), (3.0, 6.0)];
        let inside =
            proc.gate_polygon(&events, &vertices, |e| e.fsc, |e| e.ssc);
        assert_eq!(inside, vec![0]);
    }

    // --- Rectangle Gating Tests ---

    #[test]
    fn test_rectangle_gate() {
        let proc = CytometryProcessor::new(make_config());
        let events = vec![
            make_event(5.0, 5.0, vec![]),
            make_event(15.0, 5.0, vec![]),
            make_event(5.0, 15.0, vec![]),
        ];
        let inside = proc.gate_rectangle(
            &events,
            (0.0, 10.0),
            (0.0, 10.0),
            |e| e.fsc,
            |e| e.ssc,
        );
        assert_eq!(inside, vec![0]);
    }

    #[test]
    fn test_rectangle_gate_on_boundary() {
        let proc = CytometryProcessor::new(make_config());
        let events = vec![make_event(10.0, 10.0, vec![])];
        let inside = proc.gate_rectangle(
            &events,
            (0.0, 10.0),
            (0.0, 10.0),
            |e| e.fsc,
            |e| e.ssc,
        );
        assert_eq!(inside, vec![0]); // boundary is inclusive
    }

    // --- Compensation Tests ---

    #[test]
    fn test_compensation_identity_matrix_unchanged() {
        let proc = CytometryProcessor::new(CytometryConfig {
            num_fluorescence_channels: 3,
            ..make_config()
        });
        let mut events = vec![make_event(1.0, 1.0, vec![100.0, 200.0, 300.0])];
        let identity = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        proc.compensation_matrix_apply(&mut events, &identity);
        assert!((events[0].fluorescence[0] - 100.0).abs() < 1e-10);
        assert!((events[0].fluorescence[1] - 200.0).abs() < 1e-10);
        assert!((events[0].fluorescence[2] - 300.0).abs() < 1e-10);
    }

    #[test]
    fn test_compensation_spillover_correction() {
        let proc = CytometryProcessor::new(CytometryConfig {
            num_fluorescence_channels: 2,
            ..make_config()
        });
        let mut events = vec![make_event(1.0, 1.0, vec![100.0, 50.0])];
        // Compensation: subtract 20% spillover from channel 1 into channel 0
        let comp = vec![vec![1.0, -0.2], vec![0.0, 1.0]];
        proc.compensation_matrix_apply(&mut events, &comp);
        assert!((events[0].fluorescence[0] - 90.0).abs() < 1e-10);
        assert!((events[0].fluorescence[1] - 50.0).abs() < 1e-10);
    }

    // --- Histogram Tests ---

    #[test]
    fn test_histogram_bin_counts_sum_to_total() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let (_, counts) = HistogramAnalyzer::build_histogram(&values, 5);
        let total: usize = counts.iter().sum();
        assert_eq!(total, values.len());
    }

    #[test]
    fn test_histogram_correct_bins() {
        let values = vec![1.0, 1.5, 2.0, 3.0, 3.5, 4.0];
        let (edges, counts) = HistogramAnalyzer::build_histogram(&values, 3);
        assert_eq!(edges.len(), 4); // num_bins + 1
        assert_eq!(counts.len(), 3);
        let total: usize = counts.iter().sum();
        assert_eq!(total, 6);
    }

    #[test]
    fn test_histogram_empty() {
        let (edges, counts) = HistogramAnalyzer::build_histogram(&[], 10);
        assert!(edges.is_empty());
        assert!(counts.is_empty());
    }

    #[test]
    fn test_histogram_single_value() {
        let values = vec![5.0, 5.0, 5.0];
        let (_, counts) = HistogramAnalyzer::build_histogram(&values, 4);
        let total: usize = counts.iter().sum();
        assert_eq!(total, 3);
    }

    #[test]
    fn test_find_peaks_single_peak() {
        let histogram = vec![1, 3, 5, 8, 5, 3, 1];
        let peaks = HistogramAnalyzer::find_peaks(&histogram, 2);
        assert_eq!(peaks, vec![3]);
    }

    #[test]
    fn test_find_peaks_two_peaks() {
        let histogram = vec![1, 5, 2, 1, 2, 7, 1];
        let peaks = HistogramAnalyzer::find_peaks(&histogram, 3);
        assert_eq!(peaks, vec![1, 5]);
    }

    // --- Statistical Tests ---

    #[test]
    fn test_cv_of_uniform_distribution() {
        // CV of a uniform distribution [0, 1] is about 57.7%
        // But for evenly spaced values 1..=100, CV = sqrt(var)/mean * 100
        let values: Vec<f64> = (1..=100).map(|x| x as f64).collect();
        let cv = HistogramAnalyzer::coefficient_of_variation(&values);
        // Mean = 50.5, std ~= 28.87
        assert!(cv > 50.0 && cv < 65.0, "CV = {}", cv);
    }

    #[test]
    fn test_cv_constant_values() {
        let values = vec![5.0, 5.0, 5.0, 5.0];
        let cv = HistogramAnalyzer::coefficient_of_variation(&values);
        assert!((cv - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_odd() {
        let values = vec![3.0, 1.0, 2.0];
        let med = HistogramAnalyzer::median(&values);
        assert!((med - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_even() {
        let values = vec![1.0, 2.0, 3.0, 4.0];
        let med = HistogramAnalyzer::median(&values);
        assert!((med - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_percentile_50_equals_median() {
        let values = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        let p50 = HistogramAnalyzer::percentile(&values, 50.0);
        let med = HistogramAnalyzer::median(&values);
        assert!((p50 - med).abs() < 1e-10, "p50={}, median={}", p50, med);
    }

    #[test]
    fn test_percentile_extremes() {
        let values = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        let p0 = HistogramAnalyzer::percentile(&values, 0.0);
        let p100 = HistogramAnalyzer::percentile(&values, 100.0);
        assert!((p0 - 10.0).abs() < 1e-10);
        assert!((p100 - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_geometric_mean_equal_values() {
        let values = vec![7.0, 7.0, 7.0, 7.0];
        let gm = HistogramAnalyzer::geometric_mean(&values);
        assert!((gm - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_geometric_mean_known() {
        // GM(2, 8) = sqrt(16) = 4
        let values = vec![2.0, 8.0];
        let gm = HistogramAnalyzer::geometric_mean(&values);
        assert!((gm - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_geometric_mean_empty() {
        let gm = HistogramAnalyzer::geometric_mean(&[]);
        assert_eq!(gm, 0.0);
    }

    // --- K-Means Clustering Tests ---

    #[test]
    fn test_kmeans_two_clusters() {
        // Two well-separated groups
        let mut events = Vec::new();
        // Group 1: low FSC, low SSC
        for _ in 0..20 {
            events.push(make_event(1.0, 1.0, vec![]));
        }
        // Group 2: high FSC, high SSC
        for _ in 0..20 {
            events.push(make_event(100.0, 100.0, vec![]));
        }

        let features: Vec<fn(&CellEvent) -> f64> = vec![|e| e.fsc, |e| e.ssc];
        let assignments = PopulationAnalyzer::kmeans_cluster(&events, 2, &features);

        assert_eq!(assignments.len(), 40);
        // All events in the first group should have the same cluster
        let first_cluster = assignments[0];
        assert!(assignments[..20].iter().all(|&a| a == first_cluster));
        // All events in the second group should have a different cluster
        let second_cluster = assignments[20];
        assert!(assignments[20..].iter().all(|&a| a == second_cluster));
        assert_ne!(first_cluster, second_cluster);
    }

    #[test]
    fn test_kmeans_single_cluster() {
        let events = vec![
            make_event(5.0, 5.0, vec![]),
            make_event(5.1, 4.9, vec![]),
            make_event(4.9, 5.1, vec![]),
        ];
        let features: Vec<fn(&CellEvent) -> f64> = vec![|e| e.fsc, |e| e.ssc];
        let assignments = PopulationAnalyzer::kmeans_cluster(&events, 1, &features);
        assert_eq!(assignments.len(), 3);
        assert!(assignments.iter().all(|&a| a == 0));
    }

    #[test]
    fn test_kmeans_empty() {
        let events: Vec<CellEvent> = vec![];
        let features: Vec<fn(&CellEvent) -> f64> = vec![|e| e.fsc];
        let assignments = PopulationAnalyzer::kmeans_cluster(&events, 2, &features);
        assert!(assignments.is_empty());
    }

    // --- Population Statistics Tests ---

    #[test]
    fn test_population_statistics_basic() {
        let events = vec![
            make_event(10.0, 20.0, vec![100.0]),
            make_event(20.0, 40.0, vec![200.0]),
            make_event(30.0, 60.0, vec![300.0]),
        ];
        let indices: Vec<usize> = vec![0, 1, 2];
        let stats = PopulationAnalyzer::population_statistics(&events, &indices);
        assert_eq!(stats.count, 3);
        assert!((stats.mean_fsc - 20.0).abs() < 1e-10);
        assert!((stats.mean_ssc - 40.0).abs() < 1e-10);
        assert!((stats.mean_fluorescence[0] - 200.0).abs() < 1e-10);
    }

    #[test]
    fn test_population_statistics_empty() {
        let events = vec![make_event(1.0, 1.0, vec![])];
        let stats = PopulationAnalyzer::population_statistics(&events, &[]);
        assert_eq!(stats.count, 0);
    }

    // --- Doublet Discrimination Tests ---

    #[test]
    fn test_doublet_discrimination_filters_high_ratio() {
        let events = vec![
            CellEvent {
                fsc: 10.0,
                ssc: 5.0,
                fluorescence: vec![],
                time_us: 0.0,
                pulse_width: 5.0,
                pulse_height: 10.0,
                pulse_area: 50.0,
            }, // ratio = 5/50 = 0.1 (singlet)
            CellEvent {
                fsc: 10.0,
                ssc: 5.0,
                fluorescence: vec![],
                time_us: 1.0,
                pulse_width: 15.0,
                pulse_height: 10.0,
                pulse_area: 60.0,
            }, // ratio = 15/60 = 0.25 (doublet)
        ];
        let singlets = PopulationAnalyzer::doublet_discrimination(&events, 0.15);
        assert_eq!(singlets, vec![0]);
    }

    #[test]
    fn test_doublet_discrimination_all_pass() {
        let events = vec![
            CellEvent {
                fsc: 10.0,
                ssc: 5.0,
                fluorescence: vec![],
                time_us: 0.0,
                pulse_width: 5.0,
                pulse_height: 10.0,
                pulse_area: 50.0,
            },
            CellEvent {
                fsc: 12.0,
                ssc: 6.0,
                fluorescence: vec![],
                time_us: 1.0,
                pulse_width: 4.0,
                pulse_height: 12.0,
                pulse_area: 48.0,
            },
        ];
        let singlets = PopulationAnalyzer::doublet_discrimination(&events, 0.5);
        assert_eq!(singlets, vec![0, 1]);
    }

    // --- Helper Function Tests ---

    #[test]
    fn test_point_in_polygon_square() {
        let square = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        assert!(point_in_polygon(5.0, 5.0, &square));
        assert!(!point_in_polygon(15.0, 5.0, &square));
    }

    #[test]
    fn test_point_in_polygon_triangle() {
        let triangle = vec![(0.0, 0.0), (10.0, 0.0), (5.0, 10.0)];
        assert!(point_in_polygon(5.0, 3.0, &triangle));
        assert!(!point_in_polygon(0.0, 10.0, &triangle));
    }

    #[test]
    fn test_point_in_polygon_degenerate() {
        // Fewer than 3 vertices
        assert!(!point_in_polygon(0.0, 0.0, &[(0.0, 0.0), (1.0, 1.0)]));
    }

    #[test]
    fn test_log_transform_positive() {
        let result = log_transform(100.0, -1.0);
        assert!((result - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_log_transform_zero() {
        let result = log_transform(0.0, -3.0);
        assert_eq!(result, -3.0);
    }

    #[test]
    fn test_log_transform_negative() {
        let result = log_transform(-5.0, -3.0);
        assert_eq!(result, -3.0);
    }

    #[test]
    fn test_biexponential_linear_region() {
        let result = biexponential_transform(0.5, 10.0);
        assert!((result - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_biexponential_log_region() {
        // For value=1000, transition=10:
        // sign=1, log10(1000)=3, log10(10)=1, result = 3-1+1 = 3.0
        let result = biexponential_transform(1000.0, 10.0);
        assert!((result - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_biexponential_negative() {
        let result_pos = biexponential_transform(1000.0, 10.0);
        let result_neg = biexponential_transform(-1000.0, 10.0);
        assert!((result_pos + result_neg).abs() < 1e-10, "Should be antisymmetric");
    }

    #[test]
    fn test_biexponential_zero() {
        let result = biexponential_transform(0.0, 10.0);
        assert!((result - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_config_accessor() {
        let config = make_config();
        let proc = CytometryProcessor::new(config.clone());
        assert_eq!(proc.config().sample_rate_hz, 10_000_000.0);
        assert_eq!(proc.config().num_fluorescence_channels, 4);
        assert!((proc.config().laser_wavelength_nm - 488.0).abs() < 1e-10);
    }

    #[test]
    fn test_full_pipeline() {
        // Simulate a complete flow cytometry analysis pipeline
        let config = CytometryConfig {
            sample_rate_hz: 10_000_000.0,
            num_fluorescence_channels: 2,
            laser_wavelength_nm: 488.0,
            trigger_channel: TriggerChannel::ForwardScatter,
            trigger_threshold: 0.1,
        };
        let proc = CytometryProcessor::new(config);

        // Generate synthetic signal with two pulses
        let mut signal = vec![0.0; 100];
        // Pulse 1: samples 10-20
        for i in 10..20 {
            signal[i] = 0.5 + 0.5 * ((i - 10) as f64 * std::f64::consts::PI / 10.0).sin();
        }
        // Pulse 2: samples 50-65
        for i in 50..65 {
            signal[i] = 0.3 + 0.7 * ((i - 50) as f64 * std::f64::consts::PI / 15.0).sin();
        }

        // Detect events
        let pulses = proc.detect_events(&signal, 0.2);
        assert_eq!(pulses.len(), 2);

        // Extract parameters for each pulse
        for &(start, end) in &pulses {
            let (h, w, a) = proc.extract_pulse_parameters(&signal[start..end]);
            assert!(h > 0.0);
            assert!(w > 0.0);
            assert!(a > 0.0);
        }
    }
}
