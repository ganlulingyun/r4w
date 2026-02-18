//! Chromatographic Peak Resolver
//!
//! Chromatography data analysis for resolving overlapping peaks in HPLC, GC, and CE
//! chromatograms, including peak detection, deconvolution, baseline correction, and
//! quantification.
//!
//! # Key Components
//!
//! - [`Chromatogram`] - Time-series intensity data container with smoothing and interpolation
//! - [`BaselineCorrector`] - Baseline drift removal (rolling ball, rubber band, polynomial)
//! - [`PeakDetector`] - Derivative-based peak finding with prominence and width criteria
//! - [`PeakDeconvolution`] - Gaussian and EMG fitting for overlapping peak resolution
//! - [`PlateCount`] - Theoretical plate calculations and van Deemter analysis
//! - [`AsymmetryAnalysis`] - USP tailing factor and asymmetry measurements
//! - [`QuantificationEngine`] - External/internal standard quantification and calibration
//! - [`SystemSuitability`] - USP/EP system suitability test generation
//! - [`RetentionIndexCalculator`] - Kovats and linear retention indices
//! - [`NoiseEstimator`] - RMS, peak-to-peak, and ASTM noise estimation
//!
//! # Science
//!
//! - Gaussian peak: y = A exp(-(t - tr)^2 / (2 sigma^2))
//! - EMG: convolution of Gaussian with exponential decay (tailing)
//! - Plate count: N = 5.545 (tr / w_half)^2 measures column efficiency
//! - Resolution: Rs = 2 * delta_tr / (w1 + w2), baseline resolved if Rs >= 1.5
//! - Van Deemter: H = A + B/u + C*u optimizes mobile phase flow rate

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Smoothing method enum
// ---------------------------------------------------------------------------

/// Smoothing method for chromatogram data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SmoothMethod {
    /// Moving average smoothing.
    MovingAverage,
    /// Savitzky-Golay quadratic smoothing.
    SavitzkyGolay,
}

// ---------------------------------------------------------------------------
// Chromatogram
// ---------------------------------------------------------------------------

/// Time-series intensity data for a chromatographic run.
#[derive(Debug, Clone)]
pub struct Chromatogram {
    /// Retention time in minutes.
    pub time_min: Vec<f64>,
    /// Detector intensity (absorbance, counts, mV, etc.).
    pub intensity: Vec<f64>,
}

impl Chromatogram {
    /// Create a new chromatogram from time and intensity vectors.
    ///
    /// # Panics
    /// Panics if the vectors have different lengths or are empty.
    pub fn new(time_min: Vec<f64>, intensity: Vec<f64>) -> Self {
        assert!(!time_min.is_empty(), "Chromatogram must have data");
        assert_eq!(
            time_min.len(),
            intensity.len(),
            "Time and intensity must have equal length"
        );
        Self {
            time_min,
            intensity,
        }
    }

    /// Return (t_min, t_max) retention time range.
    pub fn retention_time_range(&self) -> (f64, f64) {
        let tmin = self.time_min.first().copied().unwrap_or(0.0);
        let tmax = self.time_min.last().copied().unwrap_or(0.0);
        (tmin, tmax)
    }

    /// Maximum intensity in the chromatogram.
    pub fn max_intensity(&self) -> f64 {
        self.intensity
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.time_min.len()
    }

    /// Whether the chromatogram is empty.
    pub fn is_empty(&self) -> bool {
        self.time_min.is_empty()
    }

    /// Slice the chromatogram to a time window [t_start, t_end].
    pub fn slice(&self, t_start: f64, t_end: f64) -> Chromatogram {
        let mut t = Vec::new();
        let mut y = Vec::new();
        for i in 0..self.time_min.len() {
            if self.time_min[i] >= t_start && self.time_min[i] <= t_end {
                t.push(self.time_min[i]);
                y.push(self.intensity[i]);
            }
        }
        if t.is_empty() {
            // Return single point at t_start with interpolated value
            t.push(t_start);
            y.push(self.interpolate(t_start));
        }
        Chromatogram::new(t, y)
    }

    /// Linear interpolation of intensity at arbitrary time t.
    pub fn interpolate(&self, t: f64) -> f64 {
        let n = self.time_min.len();
        if n == 0 {
            return 0.0;
        }
        if t <= self.time_min[0] {
            return self.intensity[0];
        }
        if t >= self.time_min[n - 1] {
            return self.intensity[n - 1];
        }
        // Binary search for interval
        let mut lo = 0;
        let mut hi = n - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if self.time_min[mid] <= t {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let dt = self.time_min[hi] - self.time_min[lo];
        if dt.abs() < 1e-15 {
            return self.intensity[lo];
        }
        let frac = (t - self.time_min[lo]) / dt;
        self.intensity[lo] + frac * (self.intensity[hi] - self.intensity[lo])
    }

    /// Smooth the chromatogram using the specified method and window size.
    pub fn smooth(&self, method: SmoothMethod, window: usize) -> Chromatogram {
        let n = self.intensity.len();
        let w = if window < 3 { 3 } else { window | 1 }; // ensure odd
        let half = w / 2;
        let mut smoothed = vec![0.0; n];

        match method {
            SmoothMethod::MovingAverage => {
                for i in 0..n {
                    let lo = if i >= half { i - half } else { 0 };
                    let hi = if i + half < n { i + half } else { n - 1 };
                    let count = (hi - lo + 1) as f64;
                    let sum: f64 = self.intensity[lo..=hi].iter().sum();
                    smoothed[i] = sum / count;
                }
            }
            SmoothMethod::SavitzkyGolay => {
                // Quadratic Savitzky-Golay (simplified coefficients for window sizes)
                // For general window, use convolution approach with least-squares coefficients
                let coeffs = savitzky_golay_coeffs(w);
                for i in 0..n {
                    let mut val = 0.0;
                    for (j, &c) in coeffs.iter().enumerate() {
                        let idx = i as isize + j as isize - half as isize;
                        let idx = idx.max(0).min(n as isize - 1) as usize;
                        val += c * self.intensity[idx];
                    }
                    smoothed[i] = val;
                }
            }
        }

        Chromatogram::new(self.time_min.clone(), smoothed)
    }
}

/// Compute Savitzky-Golay quadratic smoothing coefficients for window size w (must be odd).
fn savitzky_golay_coeffs(w: usize) -> Vec<f64> {
    let m = (w / 2) as isize;
    // Quadratic fit: for smoothing (0th derivative), the weights are:
    // c_i = (3*m*(m+1) - 1 - 5*i^2) / ((2*m+3)*(2*m+1)*(2*m-1)/3)
    // We compute using least-squares directly for robustness
    let n = w;
    // Build the normal equations for polynomial degree 2
    // We want to find c_j such that sum_i c_i * y_i approximates the polynomial fit at i=0
    let mut coeffs = vec![0.0; n];
    // For a degree-2 polynomial fit through points -m..m, the smoothing weights at center are:
    // Use the closed-form: c_i = (3M(M+1) - 1 - 5i^2) / ((2M+3)(2M+1)(2M-1)/3)
    // where M = m, i runs from -m to m
    let m_f = m as f64;
    let denom = (2.0 * m_f + 3.0) * (2.0 * m_f + 1.0) * (2.0 * m_f - 1.0) / 3.0;
    if denom.abs() < 1e-15 {
        // fallback to uniform
        for c in coeffs.iter_mut() {
            *c = 1.0 / n as f64;
        }
        return coeffs;
    }
    for j in 0..n {
        let i = j as isize - m;
        let i_f = i as f64;
        coeffs[j] = (3.0 * m_f * (m_f + 1.0) - 1.0 - 5.0 * i_f * i_f) / denom;
    }
    coeffs
}

// ---------------------------------------------------------------------------
// BaselineCorrector
// ---------------------------------------------------------------------------

/// Baseline correction methods for chromatographic data.
pub struct BaselineCorrector;

impl BaselineCorrector {
    /// Asymmetric least squares baseline estimation.
    ///
    /// Uses iterative reweighted least squares with asymmetric penalty.
    /// `lambda` controls smoothness (typically 1e4 to 1e7).
    /// `p` controls asymmetry (typically 0.001 to 0.01, smaller = more below signal).
    pub fn asymmetric_least_squares(chrom: &Chromatogram, lambda: f64, p: f64) -> Vec<f64> {
        let n = chrom.intensity.len();
        if n < 3 {
            return chrom.intensity.clone();
        }
        // Simplified ALS: iteratively smooth with weights
        // Use tridiagonal system D'WD + lambda*D2'D2 ~ penalized smoothing
        // For simplicity, use iterative weighted moving average approach
        let max_iter = 20;
        let mut z = chrom.intensity.clone();
        let mut w = vec![1.0; n];

        for _ in 0..max_iter {
            // Smooth z with lambda-dependent window
            let win = ((lambda.sqrt() / 10.0) as usize).max(3).min(n / 2);
            let win = win | 1; // odd
            let half = win / 2;
            let mut z_new = vec![0.0; n];
            for i in 0..n {
                let lo = if i >= half { i - half } else { 0 };
                let hi = if i + half < n { i + half } else { n - 1 };
                let mut sum_w = 0.0;
                let mut sum_wy = 0.0;
                for j in lo..=hi {
                    sum_w += w[j];
                    sum_wy += w[j] * chrom.intensity[j];
                }
                z_new[i] = if sum_w > 0.0 {
                    sum_wy / sum_w
                } else {
                    chrom.intensity[i]
                };
            }
            // Update weights: asymmetric
            for i in 0..n {
                if chrom.intensity[i] > z_new[i] {
                    w[i] = p;
                } else {
                    w[i] = 1.0 - p;
                }
            }
            z = z_new;
        }
        z
    }

    /// Rolling ball baseline estimation.
    ///
    /// Morphological approach: roll a ball of given radius underneath the signal.
    pub fn rolling_ball(chrom: &Chromatogram, radius: usize) -> Vec<f64> {
        let n = chrom.intensity.len();
        if n == 0 {
            return vec![];
        }
        let r = radius.max(1);
        // Erosion (minimum filter) followed by dilation (maximum filter)
        let mut eroded = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = if i + r < n { i + r } else { n - 1 };
            let mut min_val = f64::INFINITY;
            for j in lo..=hi {
                // Subtract ball curvature
                let dx = (j as f64 - i as f64) / r as f64;
                let ball_offset = if dx.abs() <= 1.0 {
                    r as f64 * (1.0 - (1.0 - dx * dx).sqrt())
                } else {
                    r as f64
                };
                let adj = chrom.intensity[j] + ball_offset;
                if adj < min_val {
                    min_val = adj;
                }
            }
            eroded[i] = min_val;
        }
        // Dilation
        let mut baseline = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = if i + r < n { i + r } else { n - 1 };
            let mut max_val = f64::NEG_INFINITY;
            for j in lo..=hi {
                let dx = (j as f64 - i as f64) / r as f64;
                let ball_offset = if dx.abs() <= 1.0 {
                    r as f64 * (1.0 - (1.0 - dx * dx).sqrt())
                } else {
                    r as f64
                };
                let adj = eroded[j] - ball_offset;
                if adj > max_val {
                    max_val = adj;
                }
            }
            baseline[i] = max_val;
        }
        baseline
    }

    /// Rubber band baseline using convex hull lower envelope.
    pub fn rubber_band(chrom: &Chromatogram) -> Vec<f64> {
        let n = chrom.intensity.len();
        if n < 2 {
            return chrom.intensity.clone();
        }
        // Build lower convex hull
        let mut hull: Vec<usize> = Vec::new();
        for i in 0..n {
            while hull.len() >= 2 {
                let a = hull[hull.len() - 2];
                let b = hull[hull.len() - 1];
                // Cross product to check if turning right (below)
                let cross = (chrom.time_min[b] - chrom.time_min[a])
                    * (chrom.intensity[i] - chrom.intensity[a])
                    - (chrom.intensity[b] - chrom.intensity[a])
                        * (chrom.time_min[i] - chrom.time_min[a]);
                if cross <= 0.0 {
                    hull.pop();
                } else {
                    break;
                }
            }
            hull.push(i);
        }
        // Interpolate baseline from hull points
        let mut baseline = vec![0.0; n];
        let mut hi = 0;
        for i in 0..n {
            while hi + 1 < hull.len() && hull[hi + 1] <= i {
                hi += 1;
            }
            if hi + 1 < hull.len() {
                let a = hull[hi];
                let b = hull[hi + 1];
                let dt = chrom.time_min[b] - chrom.time_min[a];
                if dt.abs() < 1e-15 {
                    baseline[i] = chrom.intensity[a];
                } else {
                    let frac = (chrom.time_min[i] - chrom.time_min[a]) / dt;
                    baseline[i] =
                        chrom.intensity[a] + frac * (chrom.intensity[b] - chrom.intensity[a]);
                }
            } else {
                baseline[i] = chrom.intensity[*hull.last().unwrap()];
            }
        }
        baseline
    }

    /// Polynomial baseline fit of given degree.
    pub fn polynomial(chrom: &Chromatogram, degree: usize) -> Vec<f64> {
        let n = chrom.intensity.len();
        let deg = degree.min(n - 1);
        let coeffs = polyfit(&chrom.time_min, &chrom.intensity, deg);
        let mut baseline = vec![0.0; n];
        for i in 0..n {
            baseline[i] = polyeval(&coeffs, chrom.time_min[i]);
        }
        baseline
    }

    /// Subtract a baseline from a chromatogram.
    pub fn subtract_baseline(chrom: &Chromatogram, baseline: &[f64]) -> Chromatogram {
        let mut corrected = vec![0.0; chrom.intensity.len()];
        for i in 0..corrected.len() {
            let bl = if i < baseline.len() { baseline[i] } else { 0.0 };
            corrected[i] = (chrom.intensity[i] - bl).max(0.0);
        }
        Chromatogram::new(chrom.time_min.clone(), corrected)
    }
}

/// Polynomial least-squares fit of degree `deg` to data (x, y).
fn polyfit(x: &[f64], y: &[f64], deg: usize) -> Vec<f64> {
    let n = x.len();
    let m = deg + 1;
    // Build normal equations: (X'X) a = X'y
    // X is n x m Vandermonde matrix
    let mut xtx = vec![0.0; m * m];
    let mut xty = vec![0.0; m];
    for i in 0..n {
        let mut xi_pow = vec![1.0; m];
        for j in 1..m {
            xi_pow[j] = xi_pow[j - 1] * x[i];
        }
        for r in 0..m {
            for c in 0..m {
                xtx[r * m + c] += xi_pow[r] * xi_pow[c];
            }
            xty[r] += xi_pow[r] * y[i];
        }
    }
    // Solve via Gauss elimination
    gauss_solve(m, &mut xtx, &mut xty)
}

/// Evaluate polynomial at x: coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ...
fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xp = 1.0;
    for &c in coeffs {
        result += c * xp;
        xp *= x;
    }
    result
}

/// Solve Ax = b via Gaussian elimination with partial pivoting. Returns x.
fn gauss_solve(n: usize, a: &mut [f64], b: &mut [f64]) -> Vec<f64> {
    // Forward elimination
    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            b.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            x[col] = 0.0;
            continue;
        }
        let mut sum = b[col];
        for k in (col + 1)..n {
            sum -= a[col * n + k] * x[k];
        }
        x[col] = sum / pivot;
    }
    x
}

// ---------------------------------------------------------------------------
// ChromPeak
// ---------------------------------------------------------------------------

/// Detected chromatographic peak with quality metrics.
#[derive(Debug, Clone)]
pub struct ChromPeak {
    /// Retention time at peak apex (minutes).
    pub retention_time: f64,
    /// Peak height (intensity units).
    pub height: f64,
    /// Peak area (intensity * minutes).
    pub area: f64,
    /// Full width at half maximum (minutes).
    pub width_at_half: f64,
    /// Asymmetry factor (b/a at 10% height).
    pub asymmetry: f64,
    /// Theoretical plate count from this peak.
    pub plates: f64,
    /// Start time of peak (minutes).
    pub start_time: f64,
    /// End time of peak (minutes).
    pub end_time: f64,
}

// ---------------------------------------------------------------------------
// PeakDetector
// ---------------------------------------------------------------------------

/// Derivative-based chromatographic peak detector.
pub struct PeakDetector {
    /// Minimum peak height.
    pub min_height: f64,
    /// Minimum peak prominence.
    pub min_prominence: f64,
    /// Minimum peak width in minutes.
    pub min_width_min: f64,
}

impl PeakDetector {
    /// Create a new peak detector.
    pub fn new(min_height: f64, min_prominence: f64, min_width_min: f64) -> Self {
        Self {
            min_height,
            min_prominence,
            min_width_min,
        }
    }

    /// Detect peaks in a chromatogram using first-derivative zero crossings.
    pub fn detect(&self, chrom: &Chromatogram) -> Vec<ChromPeak> {
        let n = chrom.intensity.len();
        if n < 3 {
            return vec![];
        }

        let mut peaks = Vec::new();

        // Compute numerical first derivative
        let mut deriv = vec![0.0; n];
        for i in 1..n - 1 {
            let dt = chrom.time_min[i + 1] - chrom.time_min[i - 1];
            if dt.abs() > 1e-15 {
                deriv[i] =
                    (chrom.intensity[i + 1] - chrom.intensity[i - 1]) / dt;
            }
        }

        // Find zero crossings of derivative (positive to negative = peak)
        for i in 1..n - 1 {
            if deriv[i - 1] > 0.0 && deriv[i + 1] < 0.0 || (deriv[i - 1] > 0.0 && deriv[i] <= 0.0 && i > 0) {
                let height = chrom.intensity[i];
                if height < self.min_height {
                    continue;
                }

                // Find peak boundaries (descent to local minimum on each side)
                let start_idx = self.find_peak_start(chrom, i);
                let end_idx = self.find_peak_end(chrom, i);

                let base_level =
                    (chrom.intensity[start_idx] + chrom.intensity[end_idx]) / 2.0;
                let prominence = height - base_level;
                if prominence < self.min_prominence {
                    continue;
                }

                let start_time = chrom.time_min[start_idx];
                let end_time = chrom.time_min[end_idx];
                let width = end_time - start_time;
                if width < self.min_width_min {
                    continue;
                }

                // FWHM
                let half_height = base_level + prominence / 2.0;
                let (t_left, t_right) =
                    find_half_height_times(chrom, i, start_idx, end_idx, half_height);
                let fwhm = t_right - t_left;

                // Area (trapezoidal integration)
                let area = trapz(
                    &chrom.time_min[start_idx..=end_idx],
                    &chrom.intensity[start_idx..=end_idx],
                );

                // Plate count from FWHM
                let plates = if fwhm > 1e-12 {
                    5.545 * (chrom.time_min[i] / fwhm).powi(2)
                } else {
                    0.0
                };

                // Asymmetry at 10% height
                let ten_pct = base_level + prominence * 0.1;
                let (t_l10, t_r10) =
                    find_half_height_times(chrom, i, start_idx, end_idx, ten_pct);
                let a_front = chrom.time_min[i] - t_l10;
                let b_back = t_r10 - chrom.time_min[i];
                let asymmetry = if a_front > 1e-12 {
                    b_back / a_front
                } else {
                    1.0
                };

                peaks.push(ChromPeak {
                    retention_time: chrom.time_min[i],
                    height,
                    area,
                    width_at_half: fwhm,
                    asymmetry,
                    plates,
                    start_time,
                    end_time,
                });
            }
        }

        peaks
    }

    fn find_peak_start(&self, chrom: &Chromatogram, peak_idx: usize) -> usize {
        let mut idx = peak_idx;
        while idx > 0 && chrom.intensity[idx - 1] <= chrom.intensity[idx] {
            idx -= 1;
        }
        idx
    }

    fn find_peak_end(&self, chrom: &Chromatogram, peak_idx: usize) -> usize {
        let n = chrom.intensity.len();
        let mut idx = peak_idx;
        while idx + 1 < n && chrom.intensity[idx + 1] <= chrom.intensity[idx] {
            idx += 1;
        }
        idx
    }
}

/// Find the times where the chromatogram crosses a given height level on each side of a peak.
fn find_half_height_times(
    chrom: &Chromatogram,
    peak_idx: usize,
    start_idx: usize,
    end_idx: usize,
    level: f64,
) -> (f64, f64) {
    // Search left side
    let mut t_left = chrom.time_min[start_idx];
    for i in (start_idx..peak_idx).rev() {
        if chrom.intensity[i] <= level && chrom.intensity[i + 1] > level {
            let dy = chrom.intensity[i + 1] - chrom.intensity[i];
            if dy.abs() > 1e-15 {
                let frac = (level - chrom.intensity[i]) / dy;
                t_left = chrom.time_min[i] + frac * (chrom.time_min[i + 1] - chrom.time_min[i]);
            } else {
                t_left = chrom.time_min[i];
            }
            break;
        }
    }
    // Search right side
    let mut t_right = chrom.time_min[end_idx];
    for i in peak_idx..end_idx {
        if chrom.intensity[i] > level && chrom.intensity[i + 1] <= level {
            let dy = chrom.intensity[i] - chrom.intensity[i + 1];
            if dy.abs() > 1e-15 {
                let frac = (chrom.intensity[i] - level) / dy;
                t_right = chrom.time_min[i] + frac * (chrom.time_min[i + 1] - chrom.time_min[i]);
            } else {
                t_right = chrom.time_min[i + 1];
            }
            break;
        }
    }
    (t_left, t_right)
}

/// Trapezoidal integration.
fn trapz(x: &[f64], y: &[f64]) -> f64 {
    let mut area = 0.0;
    for i in 1..x.len().min(y.len()) {
        area += 0.5 * (y[i] + y[i - 1]) * (x[i] - x[i - 1]);
    }
    area
}

// ---------------------------------------------------------------------------
// GaussianPeak / EmgPeak
// ---------------------------------------------------------------------------

/// Gaussian peak model parameters.
#[derive(Debug, Clone, Copy)]
pub struct GaussianPeak {
    /// Peak center (retention time, minutes).
    pub center: f64,
    /// Peak amplitude.
    pub amplitude: f64,
    /// Standard deviation (minutes).
    pub sigma: f64,
}

impl GaussianPeak {
    /// Evaluate the Gaussian at time t.
    pub fn eval(&self, t: f64) -> f64 {
        self.amplitude * (-0.5 * ((t - self.center) / self.sigma).powi(2)).exp()
    }

    /// Area under the Gaussian peak.
    pub fn area(&self) -> f64 {
        self.amplitude * self.sigma * (2.0 * PI).sqrt()
    }
}

/// Exponentially Modified Gaussian peak model parameters.
#[derive(Debug, Clone, Copy)]
pub struct EmgPeak {
    /// Gaussian center (retention time, minutes).
    pub center: f64,
    /// Amplitude.
    pub amplitude: f64,
    /// Gaussian standard deviation.
    pub sigma: f64,
    /// Exponential tailing parameter (tau > 0 for tailing).
    pub tau: f64,
}

impl EmgPeak {
    /// Evaluate the EMG peak at time t.
    ///
    /// EMG = (A * sigma / tau) * sqrt(pi/2) * exp(sigma^2/(2*tau^2) - (t-center)/tau)
    ///       * erfc((sigma^2 - tau*(t-center)) / (sqrt(2)*sigma*tau))
    pub fn eval(&self, t: f64) -> f64 {
        if self.tau.abs() < 1e-15 {
            // Degenerate to pure Gaussian
            return self.amplitude * (-0.5 * ((t - self.center) / self.sigma).powi(2)).exp();
        }
        let s = self.sigma;
        let tau = self.tau;
        let z = s / tau;
        let u = (t - self.center) / tau;
        let arg = 0.5 * z * z - u;
        // Avoid overflow
        if arg > 500.0 {
            return 0.0;
        }
        let exp_part = arg.exp();
        let erfc_arg = (z * z - u) / (std::f64::consts::SQRT_2 * z);
        let erfc_val = erfc_approx(erfc_arg);
        self.amplitude * (PI / 2.0).sqrt() * (s / tau) * exp_part * erfc_val
    }
}

/// Approximate erfc(x) using Abramowitz and Stegun formula 7.1.26.
fn erfc_approx(x: f64) -> f64 {
    if x >= 0.0 {
        erfc_positive(x)
    } else {
        2.0 - erfc_positive(-x)
    }
}

fn erfc_positive(x: f64) -> f64 {
    // Rational approximation for erfc (max error ~1.5e-7)
    let p = 0.3275911;
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let t = 1.0 / (1.0 + p * x);
    let poly = t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
    poly * (-x * x).exp()
}

// ---------------------------------------------------------------------------
// PeakDeconvolution
// ---------------------------------------------------------------------------

/// Peak deconvolution for resolving overlapping chromatographic peaks.
pub struct PeakDeconvolution;

impl PeakDeconvolution {
    /// Fit multiple Gaussian peaks to a chromatogram.
    ///
    /// Uses initial estimates from the largest local maxima, then iteratively
    /// refines via coordinate descent.
    pub fn gaussian_fit(chrom: &Chromatogram, num_peaks: usize) -> Vec<GaussianPeak> {
        if num_peaks == 0 || chrom.is_empty() {
            return vec![];
        }
        // Initial estimates: find top num_peaks local maxima
        let mut initial = find_initial_peaks(chrom, num_peaks);

        // Iterative refinement (coordinate descent)
        let max_iter = 50;
        for _ in 0..max_iter {
            for p in 0..initial.len() {
                // Compute residual excluding this peak
                let mut residual = chrom.intensity.clone();
                for (q, pk) in initial.iter().enumerate() {
                    if q != p {
                        for i in 0..residual.len() {
                            residual[i] -= pk.eval(chrom.time_min[i]);
                        }
                    }
                }
                // Fit single Gaussian to residual
                let fitted = fit_single_gaussian(&chrom.time_min, &residual, &initial[p]);
                initial[p] = fitted;
            }
        }
        initial
    }

    /// Fit multiple EMG peaks to a chromatogram.
    pub fn emg_fit(chrom: &Chromatogram, num_peaks: usize) -> Vec<EmgPeak> {
        if num_peaks == 0 || chrom.is_empty() {
            return vec![];
        }
        let gaussians = Self::gaussian_fit(chrom, num_peaks);
        // Convert Gaussian fits to EMG with small initial tau
        let mut emg_peaks: Vec<EmgPeak> = gaussians
            .iter()
            .map(|g| EmgPeak {
                center: g.center,
                amplitude: g.amplitude,
                sigma: g.sigma,
                tau: g.sigma * 0.3, // initial tailing estimate
            })
            .collect();

        // Refine tau by grid search
        for p in 0..emg_peaks.len() {
            let mut best_sse = f64::INFINITY;
            let mut best_tau = emg_peaks[p].tau;
            let sigma = emg_peaks[p].sigma;
            for k in 0..20 {
                let trial_tau = sigma * (0.05 + k as f64 * 0.1);
                emg_peaks[p].tau = trial_tau;
                let sse = Self::residual_emg(chrom, &emg_peaks);
                if sse < best_sse {
                    best_sse = sse;
                    best_tau = trial_tau;
                }
            }
            emg_peaks[p].tau = best_tau;
        }
        emg_peaks
    }

    /// Sum of squared residuals for Gaussian peak model.
    pub fn residual(chrom: &Chromatogram, peaks: &[GaussianPeak]) -> f64 {
        let mut sse = 0.0;
        for i in 0..chrom.intensity.len() {
            let t = chrom.time_min[i];
            let model: f64 = peaks.iter().map(|p| p.eval(t)).sum();
            let diff = chrom.intensity[i] - model;
            sse += diff * diff;
        }
        sse
    }

    /// Sum of squared residuals for EMG peak model.
    pub fn residual_emg(chrom: &Chromatogram, peaks: &[EmgPeak]) -> f64 {
        let mut sse = 0.0;
        for i in 0..chrom.intensity.len() {
            let t = chrom.time_min[i];
            let model: f64 = peaks.iter().map(|p| p.eval(t)).sum();
            let diff = chrom.intensity[i] - model;
            sse += diff * diff;
        }
        sse
    }
}

/// Find initial peak estimates as the top N local maxima.
fn find_initial_peaks(chrom: &Chromatogram, n: usize) -> Vec<GaussianPeak> {
    let len = chrom.intensity.len();
    let mut maxima = Vec::new();
    for i in 1..len - 1 {
        if chrom.intensity[i] > chrom.intensity[i - 1]
            && chrom.intensity[i] > chrom.intensity[i + 1]
        {
            maxima.push((i, chrom.intensity[i]));
        }
    }
    maxima.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    let count = n.min(maxima.len());
    let mut peaks = Vec::with_capacity(count);
    for k in 0..count {
        let idx = maxima[k].0;
        let amp = chrom.intensity[idx];
        // Estimate sigma from width at half height
        let half = amp / 2.0;
        let mut left_t = chrom.time_min[idx];
        for i in (0..idx).rev() {
            if chrom.intensity[i] <= half {
                let dy = chrom.intensity[i + 1] - chrom.intensity[i];
                if dy.abs() > 1e-15 {
                    let frac = (half - chrom.intensity[i]) / dy;
                    left_t = chrom.time_min[i] + frac * (chrom.time_min[i + 1] - chrom.time_min[i]);
                }
                break;
            }
        }
        let mut right_t = chrom.time_min[idx];
        for i in idx..len - 1 {
            if chrom.intensity[i + 1] <= half {
                let dy = chrom.intensity[i] - chrom.intensity[i + 1];
                if dy.abs() > 1e-15 {
                    let frac = (chrom.intensity[i] - half) / dy;
                    right_t =
                        chrom.time_min[i] + frac * (chrom.time_min[i + 1] - chrom.time_min[i]);
                }
                break;
            }
        }
        let fwhm = right_t - left_t;
        let sigma = if fwhm > 1e-12 {
            fwhm / 2.355 // FWHM = 2*sqrt(2*ln(2))*sigma
        } else {
            0.1
        };
        peaks.push(GaussianPeak {
            center: chrom.time_min[idx],
            amplitude: amp,
            sigma,
        });
    }
    // If we need more peaks than maxima found, add evenly spaced estimates
    while peaks.len() < n {
        let (tmin, tmax) = chrom.retention_time_range();
        let frac = (peaks.len() as f64 + 1.0) / (n as f64 + 1.0);
        let t = tmin + frac * (tmax - tmin);
        peaks.push(GaussianPeak {
            center: t,
            amplitude: chrom.max_intensity() * 0.1,
            sigma: (tmax - tmin) / (n as f64 * 4.0),
        });
    }
    peaks
}

/// Fit a single Gaussian to data, starting from an initial estimate.
/// Uses simple parameter updates.
fn fit_single_gaussian(time: &[f64], data: &[f64], init: &GaussianPeak) -> GaussianPeak {
    let n = time.len();
    let mut center = init.center;
    let mut amplitude = init.amplitude;
    let mut sigma = init.sigma.max(1e-6);

    for _ in 0..20 {
        // Update amplitude: least-squares fit for fixed center/sigma
        let mut sum_g2 = 0.0;
        let mut sum_gd = 0.0;
        for i in 0..n {
            let g = (-0.5 * ((time[i] - center) / sigma).powi(2)).exp();
            sum_g2 += g * g;
            sum_gd += g * data[i].max(0.0);
        }
        if sum_g2 > 1e-15 {
            amplitude = (sum_gd / sum_g2).max(0.0);
        }

        // Update center: weighted mean
        let mut sum_w = 0.0;
        let mut sum_wt = 0.0;
        for i in 0..n {
            let g = amplitude * (-0.5 * ((time[i] - center) / sigma).powi(2)).exp();
            let w = g.max(0.0);
            sum_w += w;
            sum_wt += w * time[i];
        }
        if sum_w > 1e-15 {
            center = sum_wt / sum_w;
        }

        // Update sigma: weighted std dev
        let mut sum_w2 = 0.0;
        let mut sum_wd2 = 0.0;
        for i in 0..n {
            let g = amplitude * (-0.5 * ((time[i] - center) / sigma).powi(2)).exp();
            let w = g.max(0.0);
            sum_w2 += w;
            sum_wd2 += w * (time[i] - center).powi(2);
        }
        if sum_w2 > 1e-15 {
            let new_sigma = (sum_wd2 / sum_w2).sqrt();
            if new_sigma > 1e-6 {
                sigma = new_sigma;
            }
        }
    }

    GaussianPeak {
        center,
        amplitude,
        sigma,
    }
}

// ---------------------------------------------------------------------------
// PlateCount
// ---------------------------------------------------------------------------

/// Theoretical plate count calculations for column efficiency.
pub struct PlateCount;

impl PlateCount {
    /// Plate count from retention time and peak width at base.
    ///
    /// N = 16 * (tr / w)^2
    pub fn from_retention_and_width(tr: f64, w: f64) -> f64 {
        if w.abs() < 1e-15 {
            return 0.0;
        }
        16.0 * (tr / w).powi(2)
    }

    /// Plate count from retention time and FWHM.
    ///
    /// N = 5.545 * (tr / fwhm)^2
    pub fn from_fwhm(tr: f64, fwhm: f64) -> f64 {
        if fwhm.abs() < 1e-15 {
            return 0.0;
        }
        5.545 * (tr / fwhm).powi(2)
    }

    /// Height Equivalent to a Theoretical Plate.
    ///
    /// HETP = L / N (cm/plate)
    pub fn height_equivalent(column_length_cm: f64, plates: f64) -> f64 {
        if plates.abs() < 1e-15 {
            return f64::INFINITY;
        }
        column_length_cm / plates
    }

    /// Chromatographic resolution between two adjacent peaks.
    ///
    /// Rs = 2*(tr2 - tr1) / (w1 + w2)
    pub fn resolution(tr1: f64, tr2: f64, w1: f64, w2: f64) -> f64 {
        let denom = w1 + w2;
        if denom.abs() < 1e-15 {
            return 0.0;
        }
        2.0 * (tr2 - tr1).abs() / denom
    }

    /// Van Deemter equation for plate height vs. linear velocity.
    ///
    /// H = A + B/u + C*u
    pub fn van_deemter(u: f64, a: f64, b: f64, c: f64) -> f64 {
        if u.abs() < 1e-15 {
            return f64::INFINITY;
        }
        a + b / u + c * u
    }

    /// Optimal linear velocity (minimum H) from Van Deemter.
    ///
    /// u_opt = sqrt(B/C)
    pub fn optimal_velocity(b: f64, c: f64) -> f64 {
        if c.abs() < 1e-15 {
            return 0.0;
        }
        (b / c).sqrt()
    }

    /// Minimum plate height from Van Deemter.
    ///
    /// H_min = A + 2*sqrt(B*C)
    pub fn minimum_plate_height(a: f64, b: f64, c: f64) -> f64 {
        a + 2.0 * (b * c).sqrt()
    }
}

// ---------------------------------------------------------------------------
// AsymmetryAnalysis
// ---------------------------------------------------------------------------

/// Peak asymmetry (tailing) analysis per USP/EP methods.
pub struct AsymmetryAnalysis;

impl AsymmetryAnalysis {
    /// USP tailing factor T = (a+b)/(2a) measured at 5% peak height.
    pub fn usp_tailing_factor(peak: &ChromPeak, chrom: &Chromatogram) -> f64 {
        let base = chrom.interpolate(peak.start_time);
        let prominence = peak.height - base;
        let level = base + 0.05 * prominence;
        let peak_idx = find_closest_index(&chrom.time_min, peak.retention_time);
        let start_idx = find_closest_index(&chrom.time_min, peak.start_time);
        let end_idx = find_closest_index(&chrom.time_min, peak.end_time);
        let (t_left, t_right) = find_half_height_times(chrom, peak_idx, start_idx, end_idx, level);
        let a = peak.retention_time - t_left;
        let b = t_right - peak.retention_time;
        if a.abs() < 1e-15 {
            return 1.0;
        }
        (a + b) / (2.0 * a)
    }

    /// Asymmetry factor As = b/a at 10% peak height.
    pub fn asymmetry_factor(peak: &ChromPeak, chrom: &Chromatogram) -> f64 {
        let base = chrom.interpolate(peak.start_time);
        let prominence = peak.height - base;
        let level = base + 0.1 * prominence;
        let peak_idx = find_closest_index(&chrom.time_min, peak.retention_time);
        let start_idx = find_closest_index(&chrom.time_min, peak.start_time);
        let end_idx = find_closest_index(&chrom.time_min, peak.end_time);
        let (t_left, t_right) = find_half_height_times(chrom, peak_idx, start_idx, end_idx, level);
        let a = peak.retention_time - t_left;
        let b = t_right - peak.retention_time;
        if a.abs() < 1e-15 {
            return 1.0;
        }
        b / a
    }

    /// Estimate EMG tau from asymmetry factor and sigma.
    pub fn emg_tau_from_asymmetry(asymmetry: f64, sigma: f64) -> f64 {
        // Approximate relationship: As ~ 1 + (tau/sigma)^2 for moderate tailing
        let ratio_sq = (asymmetry - 1.0).max(0.0);
        sigma * ratio_sq.sqrt()
    }

    /// Check if tailing factor is within USP acceptance (T <= 2.0).
    pub fn is_acceptable(tailing: f64) -> bool {
        tailing <= 2.0
    }
}

/// Find the index of the closest time value.
fn find_closest_index(times: &[f64], target: f64) -> usize {
    let mut best_idx = 0;
    let mut best_diff = f64::INFINITY;
    for (i, &t) in times.iter().enumerate() {
        let diff = (t - target).abs();
        if diff < best_diff {
            best_diff = diff;
            best_idx = i;
        }
    }
    best_idx
}

// ---------------------------------------------------------------------------
// QuantificationEngine
// ---------------------------------------------------------------------------

/// Calibration curve result.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Slope of linear regression.
    pub slope: f64,
    /// Intercept of linear regression.
    pub intercept: f64,
    /// Coefficient of determination (R^2).
    pub r_squared: f64,
}

/// Peak area quantification for chromatographic analysis.
pub struct QuantificationEngine;

impl QuantificationEngine {
    /// External standard quantification.
    ///
    /// Builds a calibration from standard concentrations and areas,
    /// then applies to unknown peak areas.
    pub fn external_standard(
        peak_areas: &[f64],
        std_concs: &[f64],
        std_areas: &[f64],
    ) -> Vec<f64> {
        let cal = Self::calibration_curve(std_concs, std_areas);
        peak_areas
            .iter()
            .map(|&a| {
                if cal.slope.abs() < 1e-15 {
                    0.0
                } else {
                    (a - cal.intercept) / cal.slope
                }
            })
            .collect()
    }

    /// Internal standard quantification.
    pub fn internal_standard(
        peak_areas: &[f64],
        is_area: f64,
        is_conc: f64,
        response_factors: &[f64],
    ) -> Vec<f64> {
        if is_area.abs() < 1e-15 {
            return vec![0.0; peak_areas.len()];
        }
        peak_areas
            .iter()
            .enumerate()
            .map(|(i, &a)| {
                let rf = if i < response_factors.len() {
                    response_factors[i]
                } else {
                    1.0
                };
                (a / is_area) * is_conc / rf
            })
            .collect()
    }

    /// Area percent normalization.
    pub fn area_percent(areas: &[f64]) -> Vec<f64> {
        let total: f64 = areas.iter().sum();
        if total.abs() < 1e-15 {
            return vec![0.0; areas.len()];
        }
        areas.iter().map(|&a| a / total * 100.0).collect()
    }

    /// Linear calibration curve (linear regression).
    pub fn calibration_curve(concs: &[f64], areas: &[f64]) -> CalibrationResult {
        let n = concs.len().min(areas.len());
        if n < 2 {
            return CalibrationResult {
                slope: 0.0,
                intercept: 0.0,
                r_squared: 0.0,
            };
        }
        let n_f = n as f64;
        let sum_x: f64 = concs[..n].iter().sum();
        let sum_y: f64 = areas[..n].iter().sum();
        let sum_xy: f64 = concs[..n].iter().zip(areas[..n].iter()).map(|(&x, &y)| x * y).sum();
        let sum_x2: f64 = concs[..n].iter().map(|&x| x * x).sum();
        let sum_y2: f64 = areas[..n].iter().map(|&y| y * y).sum();

        let denom = n_f * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-15 {
            return CalibrationResult {
                slope: 0.0,
                intercept: sum_y / n_f,
                r_squared: 0.0,
            };
        }
        let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n_f;

        // R^2
        let ss_tot = sum_y2 - sum_y * sum_y / n_f;
        let ss_res: f64 = concs[..n]
            .iter()
            .zip(areas[..n].iter())
            .map(|(&x, &y)| {
                let pred = slope * x + intercept;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot.abs() > 1e-15 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        CalibrationResult {
            slope,
            intercept,
            r_squared,
        }
    }

    /// Limit of Detection (LOD = 3.3 * sigma_noise / slope).
    pub fn limit_of_detection(slope: f64, noise_std: f64) -> f64 {
        if slope.abs() < 1e-15 {
            return f64::INFINITY;
        }
        3.3 * noise_std / slope
    }

    /// Limit of Quantification (LOQ = 10 * sigma_noise / slope).
    pub fn limit_of_quantification(slope: f64, noise_std: f64) -> f64 {
        if slope.abs() < 1e-15 {
            return f64::INFINITY;
        }
        10.0 * noise_std / slope
    }
}

// ---------------------------------------------------------------------------
// SystemSuitability
// ---------------------------------------------------------------------------

/// System suitability test result.
#[derive(Debug, Clone)]
pub struct SuitabilityResult {
    /// Test name.
    pub name: String,
    /// Measured value.
    pub value: f64,
    /// Acceptance limit.
    pub limit: f64,
    /// Whether the test passed.
    pub passed: bool,
}

/// System suitability report.
#[derive(Debug, Clone)]
pub struct SuitabilityReport {
    /// Individual test results.
    pub results: Vec<SuitabilityResult>,
    /// Overall pass/fail.
    pub overall_pass: bool,
}

/// USP/EP system suitability tests.
pub struct SystemSuitability;

impl SystemSuitability {
    /// Check resolution (Rs >= threshold, typically 2.0 for baseline or 1.5).
    pub fn check_resolution(rs: f64) -> bool {
        rs >= 1.5
    }

    /// Check repeatability of replicate injections.
    ///
    /// Returns (RSD%, passed) where passed if RSD <= 1.0%.
    pub fn check_repeatability(areas: &[f64]) -> (f64, bool) {
        let n = areas.len();
        if n < 2 {
            return (0.0, true);
        }
        let mean = areas.iter().sum::<f64>() / n as f64;
        if mean.abs() < 1e-15 {
            return (0.0, true);
        }
        let variance =
            areas.iter().map(|&a| (a - mean).powi(2)).sum::<f64>() / (n as f64 - 1.0);
        let std_dev = variance.sqrt();
        let rsd = std_dev / mean * 100.0;
        (rsd, rsd <= 1.0)
    }

    /// Check plate count (N >= min).
    pub fn check_plates(n: f64, min: f64) -> bool {
        n >= min
    }

    /// Check tailing factor (T <= 2.0).
    pub fn check_tailing(t: f64) -> bool {
        t <= 2.0
    }

    /// Generate a system suitability report from test results.
    pub fn generate_report(results: &[SuitabilityResult]) -> SuitabilityReport {
        let overall_pass = results.iter().all(|r| r.passed);
        SuitabilityReport {
            results: results.to_vec(),
            overall_pass,
        }
    }
}

// ---------------------------------------------------------------------------
// RetentionIndexCalculator
// ---------------------------------------------------------------------------

/// Retention index calculations (Kovats and linear).
pub struct RetentionIndexCalculator;

impl RetentionIndexCalculator {
    /// Kovats retention index for isothermal analysis.
    ///
    /// I = 100 * [n + (log(tr_x) - log(tr_n)) / (log(tr_n1) - log(tr_n))]
    pub fn kovats_index(tr_x: f64, tr_n: f64, tr_n1: f64, n: usize) -> f64 {
        let log_x = tr_x.ln();
        let log_n = tr_n.ln();
        let log_n1 = tr_n1.ln();
        let denom = log_n1 - log_n;
        if denom.abs() < 1e-15 {
            return 100.0 * n as f64;
        }
        100.0 * (n as f64 + (log_x - log_n) / denom)
    }

    /// Linear retention index for temperature-programmed analysis.
    ///
    /// I = 100 * [n + (tr_x - tr_n) / (tr_n1 - tr_n)]
    pub fn linear_index(tr_x: f64, tr_n: f64, tr_n1: f64, n: usize) -> f64 {
        let denom = tr_n1 - tr_n;
        if denom.abs() < 1e-15 {
            return 100.0 * n as f64;
        }
        100.0 * (n as f64 + (tr_x - tr_n) / denom)
    }

    /// Identify a compound by matching its retention index to a database.
    ///
    /// Returns the name of the closest match within a tolerance of 5 index units.
    pub fn identify_by_index<'a>(
        index: f64,
        database: &'a [(f64, &str)],
    ) -> Option<&'a str> {
        let tolerance = 5.0;
        let mut best_match: Option<&'a str> = None;
        let mut best_diff = f64::INFINITY;
        for &(db_index, name) in database {
            let diff = (index - db_index).abs();
            if diff < best_diff && diff <= tolerance {
                best_diff = diff;
                best_match = Some(name);
            }
        }
        best_match
    }
}

// ---------------------------------------------------------------------------
// NoiseEstimator
// ---------------------------------------------------------------------------

/// Signal-to-noise and noise estimation for chromatographic data.
pub struct NoiseEstimator;

impl NoiseEstimator {
    /// RMS noise in a blank region of the chromatogram.
    pub fn rms_noise(chrom: &Chromatogram, t_start: f64, t_end: f64) -> f64 {
        let region = chrom.slice(t_start, t_end);
        if region.len() < 2 {
            return 0.0;
        }
        let mean = region.intensity.iter().sum::<f64>() / region.len() as f64;
        let variance = region
            .intensity
            .iter()
            .map(|&y| (y - mean).powi(2))
            .sum::<f64>()
            / region.len() as f64;
        variance.sqrt()
    }

    /// Peak-to-peak noise in a blank region.
    pub fn peak_to_peak_noise(chrom: &Chromatogram, t_start: f64, t_end: f64) -> f64 {
        let region = chrom.slice(t_start, t_end);
        if region.len() < 2 {
            return 0.0;
        }
        let max = region
            .intensity
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let min = region
            .intensity
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        max - min
    }

    /// Signal-to-noise ratio per USP: S/N = 2H/h.
    ///
    /// H = peak height, h = peak-to-peak noise in baseline region.
    pub fn signal_to_noise(peak_height: f64, noise: f64) -> f64 {
        if noise.abs() < 1e-15 {
            return f64::INFINITY;
        }
        2.0 * peak_height / noise
    }

    /// ASTM noise estimation: standard deviation across segments.
    pub fn astm_noise(chrom: &Chromatogram, segment_width: f64) -> f64 {
        let (t_start, t_end) = chrom.retention_time_range();
        let total_width = t_end - t_start;
        if total_width <= 0.0 || segment_width <= 0.0 {
            return 0.0;
        }
        let num_segments = (total_width / segment_width).floor() as usize;
        if num_segments < 2 {
            return Self::rms_noise(chrom, t_start, t_end);
        }
        let mut segment_means = Vec::with_capacity(num_segments);
        for k in 0..num_segments {
            let seg_start = t_start + k as f64 * segment_width;
            let seg_end = seg_start + segment_width;
            let region = chrom.slice(seg_start, seg_end);
            let mean = region.intensity.iter().sum::<f64>() / region.len().max(1) as f64;
            segment_means.push(mean);
        }
        let grand_mean =
            segment_means.iter().sum::<f64>() / segment_means.len() as f64;
        let variance = segment_means
            .iter()
            .map(|&m| (m - grand_mean).powi(2))
            .sum::<f64>()
            / (segment_means.len() as f64 - 1.0);
        variance.sqrt()
    }
}

// ---------------------------------------------------------------------------
// Helper: generate synthetic chromatogram for testing
// ---------------------------------------------------------------------------

/// Generate a synthetic chromatogram with Gaussian peaks for testing.
pub fn generate_test_chromatogram(
    time_range: (f64, f64),
    num_points: usize,
    peaks: &[(f64, f64, f64)], // (center, amplitude, sigma)
    noise_level: f64,
) -> Chromatogram {
    let dt = (time_range.1 - time_range.0) / (num_points - 1).max(1) as f64;
    let mut time = Vec::with_capacity(num_points);
    let mut intensity = Vec::with_capacity(num_points);

    // Simple LCG PRNG for reproducible noise
    let mut seed: u64 = 42;
    let lcg_next = |s: &mut u64| -> f64 {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
        (*s >> 33) as f64 / (1u64 << 31) as f64 - 0.5
    };

    for i in 0..num_points {
        let t = time_range.0 + i as f64 * dt;
        let mut y = 0.0;
        for &(center, amp, sigma) in peaks {
            y += amp * (-0.5 * ((t - center) / sigma).powi(2)).exp();
        }
        y += noise_level * lcg_next(&mut seed);
        time.push(t);
        intensity.push(y.max(0.0));
    }
    Chromatogram::new(time, intensity)
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Chromatogram basics --

    #[test]
    fn test_chromatogram_new() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![10.0, 20.0, 15.0]);
        assert_eq!(c.len(), 3);
        assert!(!c.is_empty());
    }

    #[test]
    #[should_panic]
    fn test_chromatogram_empty() {
        Chromatogram::new(vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_chromatogram_length_mismatch() {
        Chromatogram::new(vec![0.0, 1.0], vec![10.0]);
    }

    #[test]
    fn test_retention_time_range() {
        let c = Chromatogram::new(vec![1.0, 2.0, 3.0, 4.0], vec![0.0; 4]);
        let (tmin, tmax) = c.retention_time_range();
        assert!((tmin - 1.0).abs() < 1e-10);
        assert!((tmax - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_max_intensity() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![5.0, 100.0, 3.0]);
        assert!((c.max_intensity() - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_slice() {
        let c = Chromatogram::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
        );
        let s = c.slice(1.0, 3.0);
        assert_eq!(s.len(), 3);
        assert!((s.time_min[0] - 1.0).abs() < 1e-10);
        assert!((s.time_min[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_exact() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![0.0, 10.0, 20.0]);
        assert!((c.interpolate(1.0) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_midpoint() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![0.0, 10.0, 20.0]);
        assert!((c.interpolate(0.5) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_extrapolate_left() {
        let c = Chromatogram::new(vec![1.0, 2.0], vec![10.0, 20.0]);
        assert!((c.interpolate(0.0) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_extrapolate_right() {
        let c = Chromatogram::new(vec![1.0, 2.0], vec![10.0, 20.0]);
        assert!((c.interpolate(5.0) - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_smooth_moving_average() {
        let c = Chromatogram::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.0, 100.0, 0.0, 0.0],
        );
        let s = c.smooth(SmoothMethod::MovingAverage, 3);
        // Center point should be averaged with neighbors
        assert!(s.intensity[2] < 100.0);
        assert!(s.intensity[2] > 0.0);
    }

    #[test]
    fn test_smooth_savitzky_golay() {
        let c = Chromatogram::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.0, 100.0, 0.0, 0.0],
        );
        let s = c.smooth(SmoothMethod::SavitzkyGolay, 3);
        assert!(s.intensity[2] > 0.0);
    }

    // -- Baseline correction --

    #[test]
    fn test_baseline_als() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[(5.0, 100.0, 0.5)], 0.0);
        let baseline = BaselineCorrector::asymmetric_least_squares(&c, 1e4, 0.01);
        assert_eq!(baseline.len(), c.len());
        // Baseline should be lower than peak
        let peak_idx = 100;
        assert!(baseline[peak_idx] < c.intensity[peak_idx]);
    }

    #[test]
    fn test_baseline_rolling_ball() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[(5.0, 100.0, 0.5)], 0.0);
        let baseline = BaselineCorrector::rolling_ball(&c, 20);
        assert_eq!(baseline.len(), c.len());
        // Baseline at edges should be near zero
        assert!(baseline[0].abs() < 5.0);
    }

    #[test]
    fn test_baseline_rubber_band() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[(5.0, 100.0, 0.5)], 0.0);
        let baseline = BaselineCorrector::rubber_band(&c);
        assert_eq!(baseline.len(), c.len());
        // Rubber band should be at or below the signal everywhere
        for i in 0..c.len() {
            assert!(baseline[i] <= c.intensity[i] + 1e-10);
        }
    }

    #[test]
    fn test_baseline_polynomial() {
        // Linear drift: y = 2t + noise
        let mut time = Vec::new();
        let mut intensity = Vec::new();
        for i in 0..100 {
            let t = i as f64 * 0.1;
            time.push(t);
            intensity.push(2.0 * t + 1.0);
        }
        let c = Chromatogram::new(time, intensity);
        let baseline = BaselineCorrector::polynomial(&c, 1);
        // Should recover the linear baseline closely
        for i in 0..c.len() {
            let expected = 2.0 * c.time_min[i] + 1.0;
            assert!((baseline[i] - expected).abs() < 0.1);
        }
    }

    #[test]
    fn test_subtract_baseline() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![10.0, 20.0, 15.0]);
        let bl = vec![5.0, 5.0, 5.0];
        let corrected = BaselineCorrector::subtract_baseline(&c, &bl);
        assert!((corrected.intensity[0] - 5.0).abs() < 1e-10);
        assert!((corrected.intensity[1] - 15.0).abs() < 1e-10);
        assert!((corrected.intensity[2] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_subtract_baseline_clamps_negative() {
        let c = Chromatogram::new(vec![0.0, 1.0], vec![3.0, 5.0]);
        let bl = vec![10.0, 2.0];
        let corrected = BaselineCorrector::subtract_baseline(&c, &bl);
        assert!((corrected.intensity[0] - 0.0).abs() < 1e-10); // clamped
        assert!((corrected.intensity[1] - 3.0).abs() < 1e-10);
    }

    // -- Peak detection --

    #[test]
    fn test_detect_single_peak() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        assert!((peaks[0].retention_time - 5.0).abs() < 0.1);
        assert!((peaks[0].height - 100.0).abs() < 5.0);
    }

    #[test]
    fn test_detect_two_peaks() {
        let c = generate_test_chromatogram(
            (0.0, 10.0),
            500,
            &[(3.0, 80.0, 0.3), (7.0, 60.0, 0.4)],
            0.0,
        );
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_detect_no_peaks_below_threshold() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[(5.0, 2.0, 0.3)], 0.0);
        let detector = PeakDetector::new(10.0, 10.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_area_positive() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        assert!(peaks[0].area > 0.0);
    }

    #[test]
    fn test_peak_width_positive() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        assert!(peaks[0].width_at_half > 0.0);
    }

    #[test]
    fn test_peak_plates_positive() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        assert!(peaks[0].plates > 0.0);
    }

    #[test]
    fn test_peak_start_before_end() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        assert!(peaks[0].start_time < peaks[0].end_time);
    }

    // -- Gaussian peak model --

    #[test]
    fn test_gaussian_eval_center() {
        let g = GaussianPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 1.0,
        };
        assert!((g.eval(5.0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_eval_off_center() {
        let g = GaussianPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 1.0,
        };
        let v = g.eval(6.0);
        let expected = 100.0 * (-0.5f64).exp();
        assert!((v - expected).abs() < 1e-6);
    }

    #[test]
    fn test_gaussian_area() {
        let g = GaussianPeak {
            center: 0.0,
            amplitude: 1.0,
            sigma: 1.0,
        };
        let expected = (2.0 * PI).sqrt();
        assert!((g.area() - expected).abs() < 1e-6);
    }

    #[test]
    fn test_gaussian_symmetric() {
        let g = GaussianPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 0.5,
        };
        assert!((g.eval(4.5) - g.eval(5.5)).abs() < 1e-10);
    }

    // -- EMG peak model --

    #[test]
    fn test_emg_degenerates_to_gaussian() {
        let e = EmgPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 1.0,
            tau: 0.0,
        };
        let g = GaussianPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 1.0,
        };
        assert!((e.eval(5.0) - g.eval(5.0)).abs() < 1.0);
    }

    #[test]
    fn test_emg_positive() {
        let e = EmgPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 0.5,
            tau: 0.3,
        };
        for t_val in [4.0, 5.0, 6.0, 7.0] {
            assert!(e.eval(t_val) >= 0.0);
        }
    }

    #[test]
    fn test_emg_tailing() {
        let e = EmgPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 0.5,
            tau: 0.5,
        };
        // With tailing, the peak maximum shifts to the right of center
        let val_at_center = e.eval(5.0);
        let val_right = e.eval(5.5);
        // The sum of both should be substantial
        assert!(val_at_center > 0.0);
        assert!(val_right > 0.0);
    }

    // -- Peak deconvolution --

    #[test]
    fn test_gaussian_fit_single() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.5)], 0.0);
        let fits = PeakDeconvolution::gaussian_fit(&c, 1);
        assert_eq!(fits.len(), 1);
        assert!((fits[0].center - 5.0).abs() < 0.5);
        assert!((fits[0].amplitude - 100.0).abs() < 50.0);
    }

    #[test]
    fn test_gaussian_fit_two_peaks() {
        let c = generate_test_chromatogram(
            (0.0, 10.0),
            500,
            &[(3.0, 80.0, 0.4), (7.0, 60.0, 0.5)],
            0.0,
        );
        let fits = PeakDeconvolution::gaussian_fit(&c, 2);
        assert_eq!(fits.len(), 2);
    }

    #[test]
    fn test_gaussian_residual() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[(5.0, 100.0, 0.5)], 0.0);
        let perfect = vec![GaussianPeak {
            center: 5.0,
            amplitude: 100.0,
            sigma: 0.5,
        }];
        let res = PeakDeconvolution::residual(&c, &perfect);
        assert!(res < 10.0); // should be very close to zero for noiseless data
    }

    #[test]
    fn test_emg_fit() {
        let c = generate_test_chromatogram((0.0, 10.0), 300, &[(5.0, 100.0, 0.5)], 0.0);
        let fits = PeakDeconvolution::emg_fit(&c, 1);
        assert_eq!(fits.len(), 1);
        assert!(fits[0].sigma > 0.0);
        assert!(fits[0].tau >= 0.0);
    }

    #[test]
    fn test_gaussian_fit_empty() {
        let c = Chromatogram::new(vec![0.0], vec![0.0]);
        let fits = PeakDeconvolution::gaussian_fit(&c, 0);
        assert!(fits.is_empty());
    }

    // -- PlateCount --

    #[test]
    fn test_plates_from_width() {
        let n = PlateCount::from_retention_and_width(10.0, 0.5);
        let expected = 16.0 * (10.0 / 0.5f64).powi(2);
        assert!((n - expected).abs() < 1e-6);
    }

    #[test]
    fn test_plates_from_fwhm() {
        let n = PlateCount::from_fwhm(10.0, 0.5);
        let expected = 5.545 * (10.0 / 0.5f64).powi(2);
        assert!((n - expected).abs() < 1e-6);
    }

    #[test]
    fn test_plates_zero_width() {
        assert!((PlateCount::from_retention_and_width(10.0, 0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_hetp() {
        let hetp = PlateCount::height_equivalent(25.0, 10000.0);
        assert!((hetp - 0.0025).abs() < 1e-6);
    }

    #[test]
    fn test_resolution() {
        let rs = PlateCount::resolution(5.0, 6.0, 0.4, 0.4);
        let expected = 2.0 * 1.0 / 0.8;
        assert!((rs - expected).abs() < 1e-6);
    }

    #[test]
    fn test_van_deemter() {
        let h = PlateCount::van_deemter(1.0, 0.1, 0.5, 0.05);
        let expected = 0.1 + 0.5 / 1.0 + 0.05 * 1.0;
        assert!((h - expected).abs() < 1e-10);
    }

    #[test]
    fn test_optimal_velocity() {
        let u_opt = PlateCount::optimal_velocity(0.5, 0.05);
        let expected = (0.5 / 0.05f64).sqrt();
        assert!((u_opt - expected).abs() < 1e-6);
    }

    #[test]
    fn test_minimum_plate_height() {
        let h_min = PlateCount::minimum_plate_height(0.1, 0.5, 0.05);
        let expected = 0.1 + 2.0 * (0.5 * 0.05f64).sqrt();
        assert!((h_min - expected).abs() < 1e-6);
    }

    // -- AsymmetryAnalysis --

    #[test]
    fn test_usp_tailing_symmetric() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        let t = AsymmetryAnalysis::usp_tailing_factor(&peaks[0], &c);
        // Symmetric Gaussian should have T ~ 1.0
        assert!((t - 1.0).abs() < 0.3);
    }

    #[test]
    fn test_asymmetry_factor_symmetric() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[(5.0, 100.0, 0.3)], 0.0);
        let detector = PeakDetector::new(5.0, 5.0, 0.01);
        let peaks = detector.detect(&c);
        assert!(!peaks.is_empty());
        let a = AsymmetryAnalysis::asymmetry_factor(&peaks[0], &c);
        assert!((a - 1.0).abs() < 0.3);
    }

    #[test]
    fn test_emg_tau_from_asymmetry() {
        let tau = AsymmetryAnalysis::emg_tau_from_asymmetry(2.0, 1.0);
        assert!(tau > 0.0);
    }

    #[test]
    fn test_is_acceptable() {
        assert!(AsymmetryAnalysis::is_acceptable(1.5));
        assert!(AsymmetryAnalysis::is_acceptable(2.0));
        assert!(!AsymmetryAnalysis::is_acceptable(2.1));
    }

    // -- QuantificationEngine --

    #[test]
    fn test_calibration_curve() {
        let concs = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let areas = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let cal = QuantificationEngine::calibration_curve(&concs, &areas);
        assert!((cal.slope - 100.0).abs() < 1e-6);
        assert!(cal.intercept.abs() < 1e-6);
        assert!((cal.r_squared - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_external_standard() {
        let std_concs = vec![1.0, 2.0, 3.0];
        let std_areas = vec![100.0, 200.0, 300.0];
        let peak_areas = vec![150.0, 250.0];
        let concs =
            QuantificationEngine::external_standard(&peak_areas, &std_concs, &std_areas);
        assert!((concs[0] - 1.5).abs() < 0.1);
        assert!((concs[1] - 2.5).abs() < 0.1);
    }

    #[test]
    fn test_internal_standard() {
        let peak_areas = vec![500.0, 300.0];
        let is_area = 1000.0;
        let is_conc = 10.0;
        let rf = vec![1.0, 1.0];
        let concs =
            QuantificationEngine::internal_standard(&peak_areas, is_area, is_conc, &rf);
        assert!((concs[0] - 5.0).abs() < 1e-6);
        assert!((concs[1] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_area_percent() {
        let areas = vec![100.0, 200.0, 300.0];
        let pcts = QuantificationEngine::area_percent(&areas);
        assert!((pcts[0] - 100.0 / 6.0).abs() < 0.01);
        assert!((pcts[2] - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_area_percent_zero_total() {
        let areas = vec![0.0, 0.0];
        let pcts = QuantificationEngine::area_percent(&areas);
        assert!(pcts.iter().all(|&p| p == 0.0));
    }

    #[test]
    fn test_lod() {
        let lod = QuantificationEngine::limit_of_detection(100.0, 1.0);
        assert!((lod - 0.033).abs() < 0.001);
    }

    #[test]
    fn test_loq() {
        let loq = QuantificationEngine::limit_of_quantification(100.0, 1.0);
        assert!((loq - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_calibration_r_squared() {
        let concs = vec![1.0, 2.0, 3.0, 4.0];
        let areas = vec![105.0, 198.0, 302.0, 397.0]; // slight noise
        let cal = QuantificationEngine::calibration_curve(&concs, &areas);
        assert!(cal.r_squared > 0.99);
    }

    // -- SystemSuitability --

    #[test]
    fn test_check_resolution_pass() {
        assert!(SystemSuitability::check_resolution(2.0));
        assert!(SystemSuitability::check_resolution(1.5));
    }

    #[test]
    fn test_check_resolution_fail() {
        assert!(!SystemSuitability::check_resolution(1.4));
    }

    #[test]
    fn test_check_repeatability() {
        let areas = vec![100.0, 100.5, 99.5, 100.2, 99.8];
        let (rsd, passed) = SystemSuitability::check_repeatability(&areas);
        assert!(rsd < 1.0);
        assert!(passed);
    }

    #[test]
    fn test_check_repeatability_fail() {
        let areas = vec![100.0, 110.0, 90.0, 120.0, 80.0];
        let (rsd, passed) = SystemSuitability::check_repeatability(&areas);
        assert!(rsd > 1.0);
        assert!(!passed);
    }

    #[test]
    fn test_check_plates() {
        assert!(SystemSuitability::check_plates(10000.0, 5000.0));
        assert!(!SystemSuitability::check_plates(3000.0, 5000.0));
    }

    #[test]
    fn test_check_tailing() {
        assert!(SystemSuitability::check_tailing(1.5));
        assert!(!SystemSuitability::check_tailing(2.5));
    }

    #[test]
    fn test_generate_report_pass() {
        let results = vec![
            SuitabilityResult {
                name: "Resolution".to_string(),
                value: 2.5,
                limit: 1.5,
                passed: true,
            },
            SuitabilityResult {
                name: "Plates".to_string(),
                value: 10000.0,
                limit: 5000.0,
                passed: true,
            },
        ];
        let report = SystemSuitability::generate_report(&results);
        assert!(report.overall_pass);
    }

    #[test]
    fn test_generate_report_fail() {
        let results = vec![
            SuitabilityResult {
                name: "Resolution".to_string(),
                value: 2.5,
                limit: 1.5,
                passed: true,
            },
            SuitabilityResult {
                name: "Tailing".to_string(),
                value: 2.5,
                limit: 2.0,
                passed: false,
            },
        ];
        let report = SystemSuitability::generate_report(&results);
        assert!(!report.overall_pass);
    }

    // -- RetentionIndexCalculator --

    #[test]
    fn test_kovats_index() {
        // tr_n=100, tr_n1=200 for n=10, tr_x=150
        let idx = RetentionIndexCalculator::kovats_index(150.0, 100.0, 200.0, 10);
        // I = 100*(10 + (ln150-ln100)/(ln200-ln100))
        let expected = 100.0 * (10.0 + (150.0f64.ln() - 100.0f64.ln()) / (200.0f64.ln() - 100.0f64.ln()));
        assert!((idx - expected).abs() < 0.01);
    }

    #[test]
    fn test_linear_index() {
        let idx = RetentionIndexCalculator::linear_index(150.0, 100.0, 200.0, 10);
        let expected = 100.0 * (10.0 + (150.0 - 100.0) / (200.0 - 100.0));
        assert!((idx - expected).abs() < 0.01);
    }

    #[test]
    fn test_linear_index_at_bracket() {
        let idx = RetentionIndexCalculator::linear_index(100.0, 100.0, 200.0, 10);
        assert!((idx - 1000.0).abs() < 0.01);
    }

    #[test]
    fn test_identify_by_index_found() {
        let db = vec![(1000.0, "decane"), (1100.0, "undecane"), (1200.0, "dodecane")];
        let result = RetentionIndexCalculator::identify_by_index(1001.0, &db);
        assert_eq!(result, Some("decane"));
    }

    #[test]
    fn test_identify_by_index_not_found() {
        let db = vec![(1000.0, "decane"), (1100.0, "undecane")];
        let result = RetentionIndexCalculator::identify_by_index(1050.0, &db);
        assert_eq!(result, None);
    }

    // -- NoiseEstimator --

    #[test]
    fn test_rms_noise() {
        let c = Chromatogram::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, -1.0, 1.0, -1.0, 1.0],
        );
        let noise = NoiseEstimator::rms_noise(&c, 0.0, 4.0);
        assert!(noise > 0.0);
    }

    #[test]
    fn test_peak_to_peak_noise() {
        let c = Chromatogram::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, 3.0, 2.0, 0.0, 1.5],
        );
        let pp = NoiseEstimator::peak_to_peak_noise(&c, 0.0, 4.0);
        assert!((pp - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_signal_to_noise() {
        let sn = NoiseEstimator::signal_to_noise(100.0, 2.0);
        assert!((sn - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_signal_to_noise_zero_noise() {
        let sn = NoiseEstimator::signal_to_noise(100.0, 0.0);
        assert!(sn.is_infinite());
    }

    #[test]
    fn test_astm_noise() {
        let c = generate_test_chromatogram((0.0, 10.0), 500, &[], 1.0);
        let noise = NoiseEstimator::astm_noise(&c, 1.0);
        assert!(noise > 0.0);
    }

    // -- Trapz helper --

    #[test]
    fn test_trapz_linear() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 1.0, 2.0];
        let area = trapz(&x, &y);
        assert!((area - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_trapz_constant() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 5.0, 5.0, 5.0];
        let area = trapz(&x, &y);
        assert!((area - 15.0).abs() < 1e-10);
    }

    // -- Polyfit / polyeval --

    #[test]
    fn test_polyfit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = polyfit(&x, &y, 1);
        assert!((coeffs[0] - 1.0).abs() < 1e-6); // intercept
        assert!((coeffs[1] - 2.0).abs() < 1e-6); // slope
    }

    #[test]
    fn test_polyeval() {
        let coeffs = vec![1.0, 2.0, 3.0]; // 1 + 2x + 3x^2
        let v = polyeval(&coeffs, 2.0);
        assert!((v - 17.0).abs() < 1e-10);
    }

    // -- erfc approximation --

    #[test]
    fn test_erfc_zero() {
        assert!((erfc_approx(0.0) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_erfc_large_positive() {
        assert!(erfc_approx(5.0) < 0.001);
    }

    #[test]
    fn test_erfc_large_negative() {
        assert!((erfc_approx(-5.0) - 2.0).abs() < 0.001);
    }

    // -- generate_test_chromatogram --

    #[test]
    fn test_generate_test_chromatogram() {
        let c = generate_test_chromatogram(
            (0.0, 10.0),
            100,
            &[(5.0, 50.0, 0.5)],
            0.0,
        );
        assert_eq!(c.len(), 100);
        assert!(c.max_intensity() > 40.0);
    }

    #[test]
    fn test_generate_test_chromatogram_with_noise() {
        let c = generate_test_chromatogram((0.0, 10.0), 200, &[], 1.0);
        assert_eq!(c.len(), 200);
        // All intensities clamped to >= 0
        assert!(c.intensity.iter().all(|&y| y >= 0.0));
    }

    // -- SavitzkyGolay coefficients --

    #[test]
    fn test_savitzky_golay_coeffs_sum_to_one() {
        let coeffs = savitzky_golay_coeffs(5);
        let sum: f64 = coeffs.iter().sum();
        assert!((sum - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_savitzky_golay_coeffs_symmetric() {
        let coeffs = savitzky_golay_coeffs(7);
        let n = coeffs.len();
        for i in 0..n / 2 {
            assert!((coeffs[i] - coeffs[n - 1 - i]).abs() < 1e-10);
        }
    }

    // -- find_closest_index --

    #[test]
    fn test_find_closest_index() {
        let times = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        assert_eq!(find_closest_index(&times, 2.3), 2);
        assert_eq!(find_closest_index(&times, 2.7), 3);
    }

    // -- Edge cases --

    #[test]
    fn test_rolling_ball_empty() {
        let c = Chromatogram::new(vec![0.0], vec![5.0]);
        let bl = BaselineCorrector::rolling_ball(&c, 5);
        assert_eq!(bl.len(), 1);
    }

    #[test]
    fn test_rubber_band_two_points() {
        let c = Chromatogram::new(vec![0.0, 1.0], vec![5.0, 10.0]);
        let bl = BaselineCorrector::rubber_band(&c);
        assert_eq!(bl.len(), 2);
    }

    #[test]
    fn test_polynomial_degree_zero() {
        let c = Chromatogram::new(vec![0.0, 1.0, 2.0], vec![5.0, 5.0, 5.0]);
        let bl = BaselineCorrector::polynomial(&c, 0);
        for &v in &bl {
            assert!((v - 5.0).abs() < 1e-6);
        }
    }
}
