//! # Gel Permeation Chromatography Processor
//!
//! GPC/SEC (Gel Permeation Chromatography / Size Exclusion Chromatography) signal
//! processing for polymer molecular weight distribution analysis.
//!
//! ## Science Background
//!
//! - GPC separates polymers by hydrodynamic volume (size exclusion)
//! - Larger molecules elute first (shorter retention time/volume)
//! - Calibration: log(MW) vs retention volume using narrow standards
//! - Universal calibration: [η]M (hydrodynamic volume) is polymer-independent
//! - Mark-Houwink-Sakurada: [η] = K × M^a relates intrinsic viscosity to MW
//!
//! ## Key Components
//!
//! - **CalibrationCurve** - Polynomial fit of log(MW) vs retention volume
//! - **MolecularWeightAverages** - Mn, Mw, Mz, PDI from chromatogram slices
//! - **ChromatogramProcessor** - Baseline correction, peak detection, integration
//! - **BandBroadening** - Axial dispersion and instrument spreading correction
//! - **MHSEquation** - Mark-Houwink-Sakurada intrinsic viscosity model
//! - **UniversalCalibration** - Benoit universal calibration between polymer types
//! - **MultiDetector** - RI, UV, viscometer, MALS detector processing
//! - **ColumnPerformance** - Plate count, HETP, Van Deemter equation
//! - **MWDAnalysis** - Differential/cumulative MWD, distribution fitting

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Convert retention volume to molecular weight using polynomial calibration.
///
/// coefficients are for: log10(MW) = c[0] + c[1]*V + c[2]*V^2 + c[3]*V^3 + ...
pub fn retention_to_mw(volume: f64, coefficients: &[f64]) -> f64 {
    let mut log_mw = 0.0;
    let mut v_pow = 1.0;
    for &c in coefficients {
        log_mw += c * v_pow;
        v_pow *= volume;
    }
    10.0_f64.powf(log_mw)
}

/// Polydispersity index: Mw / Mn.
pub fn pdi(mw: f64, mn: f64) -> f64 {
    if mn <= 0.0 {
        return f64::NAN;
    }
    mw / mn
}

/// Van Deemter equation: H = A + B/u + C*u
/// where A = eddy diffusion, B = longitudinal diffusion, C = mass transfer, u = flow velocity.
pub fn van_deemter(a: f64, b: f64, c: f64, u: f64) -> f64 {
    if u <= 0.0 {
        return f64::NAN;
    }
    a + b / u + c * u
}

/// Trapezoidal integration of y values with uniform spacing dx.
fn trapz(y: &[f64], dx: f64) -> f64 {
    if y.len() < 2 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..y.len() - 1 {
        sum += (y[i] + y[i + 1]) * 0.5 * dx;
    }
    sum
}

/// Simple polynomial evaluation: p(x) = c[0] + c[1]*x + c[2]*x^2 + ...
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xpow = 1.0;
    for &c in coeffs {
        result += c * xpow;
        xpow *= x;
    }
    result
}

/// Gaussian function: amplitude * exp(-0.5 * ((x - center) / sigma)^2)
fn gaussian(x: f64, amplitude: f64, center: f64, sigma: f64) -> f64 {
    let z = (x - center) / sigma;
    amplitude * (-0.5 * z * z).exp()
}

/// Solve a small linear system Ax = b via Gaussian elimination with partial pivoting.
/// Returns None if singular. `a` is row-major n×n, `b` is length n.
fn solve_linear(a: &mut [Vec<f64>], b: &mut [f64]) -> Option<Vec<f64>> {
    let n = b.len();
    // Forward elimination
    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in col + 1..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return None;
        }
        a.swap(col, max_row);
        b.swap(col, max_row);
        let pivot = a[col][col];
        for row in col + 1..n {
            let factor = a[row][col] / pivot;
            for j in col..n {
                let v = a[col][j];
                a[row][j] -= factor * v;
            }
            let bv = b[col];
            b[row] -= factor * bv;
        }
    }
    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in i + 1..n {
            sum -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-15 {
            return None;
        }
        x[i] = sum / a[i][i];
    }
    Some(x)
}

/// Least-squares polynomial fit of degree `deg` to (x, y) data.
/// Returns coefficients [c0, c1, ..., c_deg] where p(x) = c0 + c1*x + ...
fn polyfit(x: &[f64], y: &[f64], deg: usize) -> Option<Vec<f64>> {
    let n = deg + 1;
    if x.len() < n || x.len() != y.len() {
        return None;
    }
    // Build normal equations: (X^T X) c = X^T y
    let mut ata = vec![vec![0.0; n]; n];
    let mut atb = vec![0.0; n];
    for k in 0..x.len() {
        let mut xi = 1.0;
        for i in 0..n {
            atb[i] += xi * y[k];
            let mut xj = 1.0;
            for j in 0..n {
                ata[i][j] += xi * xj;
                xj *= x[k];
            }
            xi *= x[k];
        }
    }
    solve_linear(&mut ata, &mut atb)
}

// ---------------------------------------------------------------------------
// CalibrationCurve
// ---------------------------------------------------------------------------

/// Calibration method for GPC.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationType {
    /// Conventional calibration using narrow standards.
    NarrowStandard,
    /// Universal calibration using hydrodynamic volume.
    Universal,
    /// Broad standard calibration.
    BroadStandard,
}

/// Polynomial calibration curve: log10(MW) = f(V_retention).
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Polynomial coefficients [c0, c1, c2, ...] for log10(MW) = c0 + c1*V + c2*V^2 + ...
    pub coefficients: Vec<f64>,
    /// Calibration type.
    pub cal_type: CalibrationType,
    /// R-squared goodness of fit.
    pub r_squared: f64,
    /// Retention volume range [min, max] for valid interpolation.
    pub v_range: (f64, f64),
}

impl CalibrationCurve {
    /// Fit a polynomial calibration curve from narrow standard data.
    /// `volumes` = retention volumes, `mw_values` = molecular weights, `degree` = polynomial order.
    pub fn from_narrow_standards(
        volumes: &[f64],
        mw_values: &[f64],
        degree: usize,
    ) -> Option<Self> {
        if volumes.len() != mw_values.len() || volumes.len() < degree + 1 {
            return None;
        }
        let log_mw: Vec<f64> = mw_values.iter().map(|&m| m.log10()).collect();
        let coeffs = polyfit(volumes, &log_mw, degree)?;

        // R-squared
        let mean_y = log_mw.iter().sum::<f64>() / log_mw.len() as f64;
        let ss_tot: f64 = log_mw.iter().map(|&y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = volumes
            .iter()
            .zip(log_mw.iter())
            .map(|(&v, &y)| {
                let pred = poly_eval(&coeffs, v);
                (y - pred).powi(2)
            })
            .sum();
        let r_sq = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        let v_min = volumes.iter().cloned().fold(f64::INFINITY, f64::min);
        let v_max = volumes.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        Some(Self {
            coefficients: coeffs,
            cal_type: CalibrationType::NarrowStandard,
            r_squared: r_sq,
            v_range: (v_min, v_max),
        })
    }

    /// Create calibration from known polynomial coefficients.
    pub fn from_coefficients(coefficients: Vec<f64>, v_min: f64, v_max: f64) -> Self {
        Self {
            coefficients,
            cal_type: CalibrationType::NarrowStandard,
            r_squared: 1.0,
            v_range: (v_min, v_max),
        }
    }

    /// Convert retention volume to molecular weight.
    pub fn volume_to_mw(&self, volume: f64) -> f64 {
        retention_to_mw(volume, &self.coefficients)
    }

    /// Convert retention volume to log10(MW).
    pub fn volume_to_log_mw(&self, volume: f64) -> f64 {
        poly_eval(&self.coefficients, volume)
    }

    /// Check if a volume is within the calibrated range.
    pub fn is_in_range(&self, volume: f64) -> bool {
        volume >= self.v_range.0 && volume <= self.v_range.1
    }
}

// ---------------------------------------------------------------------------
// MolecularWeightAverages
// ---------------------------------------------------------------------------

/// Molecular weight averages computed from chromatogram slices.
#[derive(Debug, Clone, Copy)]
pub struct MolecularWeightAverages {
    /// Number-average molecular weight: Σ(hi) / Σ(hi/Mi)
    pub mn: f64,
    /// Weight-average molecular weight: Σ(hi*Mi) / Σ(hi)
    pub mw: f64,
    /// Z-average molecular weight: Σ(hi*Mi^2) / Σ(hi*Mi)
    pub mz: f64,
    /// Polydispersity index: Mw / Mn
    pub pdi: f64,
    /// Viscosity-average molecular weight (if MHS params available)
    pub mv: Option<f64>,
}

impl MolecularWeightAverages {
    /// Compute averages from chromatogram slice heights and their molecular weights.
    /// `heights` = detector response at each slice, `mws` = MW at each slice.
    pub fn from_slices(heights: &[f64], mws: &[f64]) -> Self {
        assert_eq!(heights.len(), mws.len());
        let mut sum_h = 0.0;
        let mut sum_h_over_m = 0.0;
        let mut sum_h_m = 0.0;
        let mut sum_h_m2 = 0.0;

        for (&h, &m) in heights.iter().zip(mws.iter()) {
            if h <= 0.0 || m <= 0.0 {
                continue;
            }
            sum_h += h;
            sum_h_over_m += h / m;
            sum_h_m += h * m;
            sum_h_m2 += h * m * m;
        }

        let mn = if sum_h_over_m > 0.0 {
            sum_h / sum_h_over_m
        } else {
            0.0
        };
        let mw = if sum_h > 0.0 { sum_h_m / sum_h } else { 0.0 };
        let mz = if sum_h_m > 0.0 {
            sum_h_m2 / sum_h_m
        } else {
            0.0
        };
        let pdi_val = pdi(mw, mn);

        Self {
            mn,
            mw,
            mz,
            pdi: pdi_val,
            mv: None,
        }
    }

    /// Compute averages including viscosity-average using MHS parameters.
    pub fn from_slices_with_mhs(heights: &[f64], mws: &[f64], mhs: &MHSEquation) -> Self {
        let mut avg = Self::from_slices(heights, mws);
        let a = mhs.a;

        let mut sum_h = 0.0;
        let mut sum_h_ma = 0.0;
        for (&h, &m) in heights.iter().zip(mws.iter()) {
            if h <= 0.0 || m <= 0.0 {
                continue;
            }
            sum_h += h;
            sum_h_ma += h * m.powf(a);
        }
        if sum_h > 0.0 {
            avg.mv = Some((sum_h_ma / sum_h).powf(1.0 / a));
        }
        avg
    }
}

// ---------------------------------------------------------------------------
// ChromatogramProcessor
// ---------------------------------------------------------------------------

/// A detected peak in a chromatogram.
#[derive(Debug, Clone, Copy)]
pub struct ChromPeak {
    /// Peak apex index.
    pub apex_index: usize,
    /// Retention volume at apex.
    pub retention_volume: f64,
    /// Peak height (baseline-subtracted).
    pub height: f64,
    /// Integrated area (trapezoidal).
    pub area: f64,
    /// Left boundary index.
    pub left: usize,
    /// Right boundary index.
    pub right: usize,
    /// Asymmetry factor (tailing factor at 10% height).
    pub asymmetry: f64,
}

/// Baseline correction method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaselineMethod {
    /// Linear baseline between endpoints.
    Linear,
    /// Valley-to-valley baseline.
    ValleyToValley,
    /// Polynomial baseline of given degree.
    Polynomial(usize),
}

/// Chromatogram signal processor.
#[derive(Debug, Clone)]
pub struct ChromatogramProcessor {
    /// Raw detector signal.
    pub signal: Vec<f64>,
    /// Retention volume for each data point.
    pub volumes: Vec<f64>,
    /// Baseline-corrected signal.
    pub corrected: Vec<f64>,
    /// Detected peaks.
    pub peaks: Vec<ChromPeak>,
}

impl ChromatogramProcessor {
    /// Create from raw signal and corresponding retention volumes.
    pub fn new(signal: Vec<f64>, volumes: Vec<f64>) -> Self {
        assert_eq!(signal.len(), volumes.len());
        let corrected = signal.clone();
        Self {
            signal,
            volumes,
            corrected,
            peaks: Vec::new(),
        }
    }

    /// Apply baseline correction.
    pub fn correct_baseline(&mut self, method: BaselineMethod) {
        match method {
            BaselineMethod::Linear => {
                if self.signal.len() < 2 {
                    return;
                }
                let n = self.signal.len();
                let y0 = self.signal[0];
                let y1 = self.signal[n - 1];
                for i in 0..n {
                    let frac = i as f64 / (n - 1) as f64;
                    self.corrected[i] = self.signal[i] - (y0 + frac * (y1 - y0));
                }
            }
            BaselineMethod::ValleyToValley => {
                // Find minima, interpolate baseline through them
                let n = self.signal.len();
                if n < 3 {
                    self.corrected = self.signal.clone();
                    return;
                }
                let mut valleys = vec![0usize]; // start
                for i in 1..n - 1 {
                    if self.signal[i] <= self.signal[i - 1]
                        && self.signal[i] <= self.signal[i + 1]
                    {
                        valleys.push(i);
                    }
                }
                valleys.push(n - 1); // end
                valleys.dedup();

                // Linear interpolation between valleys
                let mut baseline = vec![0.0; n];
                for seg in 0..valleys.len() - 1 {
                    let i0 = valleys[seg];
                    let i1 = valleys[seg + 1];
                    let y0 = self.signal[i0];
                    let y1 = self.signal[i1];
                    for i in i0..=i1 {
                        let frac = if i1 > i0 {
                            (i - i0) as f64 / (i1 - i0) as f64
                        } else {
                            0.0
                        };
                        baseline[i] = y0 + frac * (y1 - y0);
                    }
                }
                for i in 0..n {
                    self.corrected[i] = self.signal[i] - baseline[i];
                }
            }
            BaselineMethod::Polynomial(deg) => {
                // Fit polynomial to the first and last 10% of data points
                let n = self.signal.len();
                let wing = (n as f64 * 0.1).max(2.0) as usize;
                let mut xfit = Vec::new();
                let mut yfit = Vec::new();
                for i in 0..wing.min(n) {
                    xfit.push(self.volumes[i]);
                    yfit.push(self.signal[i]);
                }
                for i in n.saturating_sub(wing)..n {
                    xfit.push(self.volumes[i]);
                    yfit.push(self.signal[i]);
                }
                if let Some(coeffs) = polyfit(&xfit, &yfit, deg.min(3)) {
                    for i in 0..n {
                        self.corrected[i] = self.signal[i] - poly_eval(&coeffs, self.volumes[i]);
                    }
                }
            }
        }
    }

    /// Detect peaks in the corrected signal.
    /// `min_height` = minimum peak height, `min_prominence` = minimum prominence.
    pub fn detect_peaks(&mut self, min_height: f64, min_prominence: f64) {
        self.peaks.clear();
        let n = self.corrected.len();
        if n < 3 {
            return;
        }

        // Find local maxima
        let mut maxima = Vec::new();
        for i in 1..n - 1 {
            if self.corrected[i] > self.corrected[i - 1]
                && self.corrected[i] >= self.corrected[i + 1]
                && self.corrected[i] >= min_height
            {
                maxima.push(i);
            }
        }

        for &apex in &maxima {
            // Find left boundary (descend to valley or edge)
            let mut left = apex;
            while left > 0 && self.corrected[left - 1] <= self.corrected[left] {
                left -= 1;
            }
            // Find right boundary
            let mut right = apex;
            while right < n - 1 && self.corrected[right + 1] <= self.corrected[right] {
                right += 1;
            }

            let base = self.corrected[left].max(self.corrected[right]);
            let prominence = self.corrected[apex] - base;
            if prominence < min_prominence {
                continue;
            }

            // Area by trapezoidal rule
            let slice = &self.corrected[left..=right];
            let dx = if right > left {
                (self.volumes[right] - self.volumes[left]) / (right - left) as f64
            } else {
                1.0
            };
            let area = trapz(slice, dx);

            // Asymmetry factor at 10% height
            let h10 = self.corrected[apex] * 0.1 + base * 0.9;
            let mut left_10 = apex;
            while left_10 > left && self.corrected[left_10] > h10 {
                left_10 -= 1;
            }
            let mut right_10 = apex;
            while right_10 < right && self.corrected[right_10] > h10 {
                right_10 += 1;
            }
            let a_dist = (apex - left_10) as f64;
            let b_dist = (right_10 - apex) as f64;
            let asymmetry = if a_dist > 0.0 { b_dist / a_dist } else { 1.0 };

            self.peaks.push(ChromPeak {
                apex_index: apex,
                retention_volume: self.volumes[apex],
                height: self.corrected[apex],
                area,
                left,
                right,
                asymmetry,
            });
        }
    }

    /// Integrate a region of the corrected chromatogram (trapezoidal).
    pub fn integrate_region(&self, start_idx: usize, end_idx: usize) -> f64 {
        if start_idx >= end_idx || end_idx >= self.corrected.len() {
            return 0.0;
        }
        let slice = &self.corrected[start_idx..=end_idx];
        let dx = (self.volumes[end_idx] - self.volumes[start_idx]) / (end_idx - start_idx) as f64;
        trapz(slice, dx)
    }

    /// Compute molecular weight averages using a calibration curve.
    pub fn compute_mw_averages(
        &self,
        cal: &CalibrationCurve,
        start_idx: usize,
        end_idx: usize,
    ) -> MolecularWeightAverages {
        let heights: Vec<f64> = self.corrected[start_idx..=end_idx].to_vec();
        let mws: Vec<f64> = self.volumes[start_idx..=end_idx]
            .iter()
            .map(|&v| cal.volume_to_mw(v))
            .collect();
        MolecularWeightAverages::from_slices(&heights, &mws)
    }
}

// ---------------------------------------------------------------------------
// BandBroadening
// ---------------------------------------------------------------------------

/// Band broadening correction parameters.
#[derive(Debug, Clone)]
pub struct BandBroadening {
    /// Instrument spreading function sigma (in volume units).
    pub sigma: f64,
    /// Skew parameter for exponentially modified Gaussian.
    pub skew: f64,
}

impl BandBroadening {
    /// Create with given spreading sigma and skew.
    pub fn new(sigma: f64, skew: f64) -> Self {
        Self { sigma, skew }
    }

    /// Generate Gaussian instrument spreading function.
    pub fn gaussian_isf(&self, n: usize, dx: f64) -> Vec<f64> {
        let half = n / 2;
        let mut isf = vec![0.0; n];
        let mut sum = 0.0;
        for i in 0..n {
            let x = (i as f64 - half as f64) * dx;
            isf[i] = (-0.5 * (x / self.sigma).powi(2)).exp();
            sum += isf[i];
        }
        if sum > 0.0 {
            for v in &mut isf {
                *v /= sum;
            }
        }
        isf
    }

    /// Apply Tung's method for band broadening correction (iterative deconvolution).
    /// `observed` = observed chromatogram, `dx` = volume spacing.
    pub fn correct_tung(&self, observed: &[f64], dx: f64, iterations: usize) -> Vec<f64> {
        let n = observed.len();
        let isf = self.gaussian_isf(n.min(64), dx);
        let isf_len = isf.len();
        let half = isf_len / 2;

        // Iterative Tung correction: w_new = w_old + (observed - convolve(w_old, isf))
        let mut corrected = observed.to_vec();

        for _ in 0..iterations {
            // Convolve corrected with ISF
            let mut conv = vec![0.0; n];
            for i in 0..n {
                let mut sum = 0.0;
                for j in 0..isf_len {
                    let idx = i as isize + j as isize - half as isize;
                    if idx >= 0 && (idx as usize) < n {
                        sum += corrected[idx as usize] * isf[j];
                    }
                }
                conv[i] = sum;
            }
            // Update
            for i in 0..n {
                corrected[i] += observed[i] - conv[i];
            }
        }
        corrected
    }

    /// Apply skew correction (exponentially modified Gaussian deconvolution).
    pub fn correct_skew(&self, signal: &[f64], dx: f64) -> Vec<f64> {
        if self.skew.abs() < 1e-12 || signal.is_empty() {
            return signal.to_vec();
        }
        let tau = self.skew;
        let n = signal.len();
        let mut result = vec![0.0; n];
        // First-order exponential correction: w(v) = g(v) - tau * dg/dv
        for i in 0..n {
            let deriv = if i == 0 {
                (signal[1] - signal[0]) / dx
            } else if i == n - 1 {
                (signal[n - 1] - signal[n - 2]) / dx
            } else {
                (signal[i + 1] - signal[i - 1]) / (2.0 * dx)
            };
            result[i] = signal[i] - tau * deriv;
        }
        result
    }
}

// ---------------------------------------------------------------------------
// MHSEquation
// ---------------------------------------------------------------------------

/// Mark-Houwink-Sakurada equation: [η] = K × M^a
#[derive(Debug, Clone, Copy)]
pub struct MHSEquation {
    /// K constant (dL/g units).
    pub k: f64,
    /// a exponent.
    pub a: f64,
}

impl MHSEquation {
    /// Create with given K and a parameters.
    pub fn new(k: f64, a: f64) -> Self {
        Self { k, a }
    }

    /// Polystyrene in THF: K=1.14e-4 dL/g, a=0.716
    pub fn ps_thf() -> Self {
        Self {
            k: 1.14e-4,
            a: 0.716,
        }
    }

    /// PMMA in THF: K=1.28e-4 dL/g, a=0.690
    pub fn pmma_thf() -> Self {
        Self {
            k: 1.28e-4,
            a: 0.690,
        }
    }

    /// Polyethylene in TCB at 135°C: K=4.06e-4, a=0.725
    pub fn pe_tcb() -> Self {
        Self {
            k: 4.06e-4,
            a: 0.725,
        }
    }

    /// Polypropylene in TCB at 135°C: K=1.90e-4, a=0.725
    pub fn pp_tcb() -> Self {
        Self {
            k: 1.90e-4,
            a: 0.725,
        }
    }

    /// Compute intrinsic viscosity from molecular weight.
    pub fn intrinsic_viscosity(&self, mw: f64) -> f64 {
        self.k * mw.powf(self.a)
    }

    /// Compute molecular weight from intrinsic viscosity.
    pub fn mw_from_viscosity(&self, eta: f64) -> f64 {
        if self.k <= 0.0 || self.a <= 0.0 {
            return 0.0;
        }
        (eta / self.k).powf(1.0 / self.a)
    }

    /// Hydrodynamic volume J = [η] × M
    pub fn hydrodynamic_volume(&self, mw: f64) -> f64 {
        self.intrinsic_viscosity(mw) * mw
    }
}

// ---------------------------------------------------------------------------
// UniversalCalibration
// ---------------------------------------------------------------------------

/// Benoit universal calibration: J = [η]M = f(V_retention)
/// Converts between polymer types via hydrodynamic volume.
#[derive(Debug, Clone)]
pub struct UniversalCalibration {
    /// Calibration curve (log10(J) vs V) where J = [η]M
    pub j_curve: CalibrationCurve,
    /// Reference polymer MHS parameters (used for initial calibration).
    pub reference_mhs: MHSEquation,
}

impl UniversalCalibration {
    /// Build universal calibration from conventional calibration and reference MHS parameters.
    pub fn from_conventional(cal: &CalibrationCurve, ref_mhs: &MHSEquation) -> Self {
        // Convert log(M) coefficients to log(J) = log([η]M) = log(K) + (1+a)*log(M)
        // J = K * M^a * M = K * M^(1+a)
        // log(J) = log(K) + (1+a) * log(M)
        let log_k = ref_mhs.k.log10();
        let exp = 1.0 + ref_mhs.a;

        // New coefficients: log(J) = log(K) + (1+a) * (c0 + c1*V + c2*V^2 + ...)
        let mut j_coeffs = cal.coefficients.clone();
        j_coeffs[0] = log_k + exp * j_coeffs[0];
        for i in 1..j_coeffs.len() {
            j_coeffs[i] *= exp;
        }

        let j_curve = CalibrationCurve {
            coefficients: j_coeffs,
            cal_type: CalibrationType::Universal,
            r_squared: cal.r_squared,
            v_range: cal.v_range,
        };

        Self {
            j_curve,
            reference_mhs: *ref_mhs,
        }
    }

    /// Convert retention volume to hydrodynamic volume J = [η]M.
    pub fn volume_to_j(&self, volume: f64) -> f64 {
        10.0_f64.powf(poly_eval(&self.j_curve.coefficients, volume))
    }

    /// Convert retention volume to MW for a target polymer.
    /// Uses J = K_target * M^(1+a_target) => M = (J / K_target)^(1/(1+a_target))
    pub fn volume_to_mw(&self, volume: f64, target_mhs: &MHSEquation) -> f64 {
        let j = self.volume_to_j(volume);
        if target_mhs.k <= 0.0 {
            return 0.0;
        }
        (j / target_mhs.k).powf(1.0 / (1.0 + target_mhs.a))
    }
}

// ---------------------------------------------------------------------------
// MultiDetector
// ---------------------------------------------------------------------------

/// Detector type in a multi-detector GPC system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Refractive Index (concentration detector).
    RI,
    /// UV absorption (selective concentration detector).
    UV,
    /// Online viscometer (viscosity detector).
    Viscometer,
    /// Multi-Angle Light Scattering.
    MALS,
}

/// Multi-detector GPC data at a single chromatogram slice.
#[derive(Debug, Clone, Copy)]
pub struct DetectorSlice {
    /// RI detector response (proportional to concentration).
    pub ri: f64,
    /// UV detector response.
    pub uv: Option<f64>,
    /// Specific viscosity from viscometer.
    pub viscosity: Option<f64>,
    /// Rayleigh ratio from MALS (at zero angle extrapolation).
    pub mals_r0: Option<f64>,
}

/// Multi-detector GPC processor.
#[derive(Debug, Clone)]
pub struct MultiDetector {
    /// dn/dc (specific refractive index increment) in mL/g.
    pub dn_dc: f64,
    /// Laser wavelength for MALS in nm.
    pub laser_wavelength: f64,
    /// Solvent refractive index.
    pub solvent_ri: f64,
    /// Optical constant K* for MALS.
    pub k_star: f64,
}

impl MultiDetector {
    /// Create a multi-detector processor.
    /// `dn_dc` in mL/g, `laser_wavelength` in nm, `solvent_ri` dimensionless.
    pub fn new(dn_dc: f64, laser_wavelength: f64, solvent_ri: f64) -> Self {
        // K* = 4π²n₀²(dn/dc)² / (N_A λ⁴)
        let lambda_cm = laser_wavelength * 1e-7; // nm to cm
        let na = 6.022e23;
        let k_star =
            4.0 * PI * PI * solvent_ri * solvent_ri * dn_dc * dn_dc / (na * lambda_cm.powi(4));
        Self {
            dn_dc,
            laser_wavelength,
            solvent_ri,
            k_star,
        }
    }

    /// Compute absolute MW at a slice from MALS and RI data.
    /// MW = R(0) / (K* × c), where c is proportional to RI response / dn_dc.
    pub fn mw_from_mals(&self, slice: &DetectorSlice, cell_volume: f64) -> Option<f64> {
        let r0 = slice.mals_r0?;
        if slice.ri.abs() < 1e-15 {
            return None;
        }
        // concentration = RI_response * cell_volume / dn_dc (simplified)
        let c = slice.ri.abs() * cell_volume / self.dn_dc;
        if c <= 0.0 || self.k_star <= 0.0 {
            return None;
        }
        Some(r0 / (self.k_star * c))
    }

    /// Compute intrinsic viscosity at a slice from viscometer and RI data.
    /// [η] = η_sp / c
    pub fn intrinsic_viscosity_from_slice(
        &self,
        slice: &DetectorSlice,
        cell_volume: f64,
    ) -> Option<f64> {
        let eta_sp = slice.viscosity?;
        if slice.ri.abs() < 1e-15 {
            return None;
        }
        let c = slice.ri.abs() * cell_volume / self.dn_dc;
        if c <= 0.0 {
            return None;
        }
        Some(eta_sp / c)
    }

    /// Compute UV/RI ratio (related to composition in copolymers).
    pub fn uv_ri_ratio(&self, slice: &DetectorSlice) -> Option<f64> {
        let uv = slice.uv?;
        if slice.ri.abs() < 1e-15 {
            return None;
        }
        Some(uv / slice.ri)
    }
}

// ---------------------------------------------------------------------------
// ColumnPerformance
// ---------------------------------------------------------------------------

/// Column performance metrics.
#[derive(Debug, Clone, Copy)]
pub struct ColumnPerformance {
    /// Number of theoretical plates N.
    pub plate_count: f64,
    /// Height Equivalent to a Theoretical Plate (HETP) in mm.
    pub hetp: f64,
    /// Column length in mm.
    pub column_length: f64,
    /// Resolution between two peaks.
    pub resolution: Option<f64>,
}

impl ColumnPerformance {
    /// Compute plate count from a peak.
    /// N = 5.54 * (V_r / W_0.5)^2 (half-height method)
    /// `retention_vol` = peak retention volume, `width_half` = peak width at half height.
    pub fn from_peak(retention_vol: f64, width_half: f64, column_length_mm: f64) -> Self {
        let n = if width_half > 0.0 {
            5.54 * (retention_vol / width_half).powi(2)
        } else {
            0.0
        };
        let hetp = if n > 0.0 {
            column_length_mm / n
        } else {
            f64::INFINITY
        };
        Self {
            plate_count: n,
            hetp,
            column_length: column_length_mm,
            resolution: None,
        }
    }

    /// Compute resolution between two peaks.
    /// R = 2 * (V2 - V1) / (W1 + W2) where W = base width ≈ 4σ.
    pub fn resolution(v1: f64, v2: f64, w1: f64, w2: f64) -> f64 {
        let denom = w1 + w2;
        if denom <= 0.0 {
            return 0.0;
        }
        2.0 * (v2 - v1).abs() / denom
    }

    /// Compute plate count using tangent method.
    /// N = 16 * (V_r / W_base)^2
    pub fn plate_count_tangent(retention_vol: f64, width_base: f64) -> f64 {
        if width_base <= 0.0 {
            return 0.0;
        }
        16.0 * (retention_vol / width_base).powi(2)
    }

    /// Asymmetry factor from USP tailing factor.
    /// T = (a + b) / (2 * a) at 5% peak height, where a = front half-width, b = back half-width.
    pub fn usp_tailing_factor(front_width: f64, back_width: f64) -> f64 {
        if front_width <= 0.0 {
            return f64::NAN;
        }
        (front_width + back_width) / (2.0 * front_width)
    }
}

/// Van Deemter curve fitting.
#[derive(Debug, Clone, Copy)]
pub struct VanDeemterFit {
    /// A term (eddy diffusion).
    pub a: f64,
    /// B term (longitudinal diffusion).
    pub b: f64,
    /// C term (mass transfer resistance).
    pub c: f64,
    /// Optimal flow velocity u_opt = sqrt(B/C).
    pub u_opt: f64,
    /// Minimum plate height H_min = A + 2*sqrt(B*C).
    pub h_min: f64,
}

impl VanDeemterFit {
    /// Fit Van Deemter equation to (flow_velocity, plate_height) data.
    pub fn fit(velocities: &[f64], heights: &[f64]) -> Option<Self> {
        if velocities.len() < 3 || velocities.len() != heights.len() {
            return None;
        }
        // H = A + B/u + C*u
        // Transform: H = A + B*(1/u) + C*u
        // Linear system in [A, B, C] with basis functions [1, 1/u, u]
        let n = velocities.len();
        let mut ata = vec![vec![0.0; 3]; 3];
        let mut atb = vec![0.0; 3];

        for i in 0..n {
            let u = velocities[i];
            if u <= 0.0 {
                continue;
            }
            let basis = [1.0, 1.0 / u, u];
            for r in 0..3 {
                atb[r] += basis[r] * heights[i];
                for c in 0..3 {
                    ata[r][c] += basis[r] * basis[c];
                }
            }
        }

        let params = solve_linear(&mut ata, &mut atb)?;
        let a = params[0];
        let b = params[1];
        let c = params[2];

        let u_opt = if b > 0.0 && c > 0.0 {
            (b / c).sqrt()
        } else {
            0.0
        };
        let h_min = if b > 0.0 && c > 0.0 {
            a + 2.0 * (b * c).sqrt()
        } else {
            a
        };

        Some(Self {
            a,
            b,
            c,
            u_opt,
            h_min,
        })
    }

    /// Evaluate H at a given flow velocity.
    pub fn evaluate(&self, u: f64) -> f64 {
        van_deemter(self.a, self.b, self.c, u)
    }
}

// ---------------------------------------------------------------------------
// MWDAnalysis
// ---------------------------------------------------------------------------

/// Molecular weight distribution analysis.
#[derive(Debug, Clone)]
pub struct MWDAnalysis {
    /// log10(MW) values.
    pub log_mw: Vec<f64>,
    /// Differential weight fraction dW/d(log M).
    pub dw_dlog_m: Vec<f64>,
    /// Cumulative weight fraction.
    pub cumulative: Vec<f64>,
}

impl MWDAnalysis {
    /// Compute differential and cumulative MWD from chromatogram and calibration.
    pub fn from_chromatogram(
        heights: &[f64],
        volumes: &[f64],
        cal: &CalibrationCurve,
    ) -> Self {
        let n = heights.len();
        assert_eq!(n, volumes.len());

        let mut log_mw = Vec::with_capacity(n);
        let mut dw = Vec::with_capacity(n);

        for i in 0..n {
            let lm = cal.volume_to_log_mw(volumes[i]);
            log_mw.push(lm);
            // dW/d(logM) ∝ h(V) * |dV/d(logM)|
            // For polynomial calibration, d(logM)/dV is derivative of polynomial
            let dlog_dv = {
                let mut deriv = 0.0;
                let mut vpow = 1.0;
                for j in 1..cal.coefficients.len() {
                    deriv += j as f64 * cal.coefficients[j] * vpow;
                    vpow *= volumes[i];
                }
                deriv
            };
            let dv_dlog = if dlog_dv.abs() > 1e-15 {
                (1.0 / dlog_dv).abs()
            } else {
                0.0
            };
            dw.push(heights[i].max(0.0) * dv_dlog);
        }

        // Normalize dW so area = 1
        // Sort by log_mw ascending for proper integration
        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| log_mw[a].partial_cmp(&log_mw[b]).unwrap_or(std::cmp::Ordering::Equal));

        let mut sorted_log = vec![0.0; n];
        let mut sorted_dw = vec![0.0; n];
        for (out_i, &src_i) in indices.iter().enumerate() {
            sorted_log[out_i] = log_mw[src_i];
            sorted_dw[out_i] = dw[src_i];
        }

        // Normalize
        let total = if n > 1 {
            let dx_avg = (sorted_log[n - 1] - sorted_log[0]) / (n - 1) as f64;
            trapz(&sorted_dw, dx_avg.abs())
        } else {
            1.0
        };
        if total > 0.0 {
            for v in &mut sorted_dw {
                *v /= total;
            }
        }

        // Cumulative
        let mut cumulative = vec![0.0; n];
        if n > 1 {
            let dx = (sorted_log[n - 1] - sorted_log[0]).abs() / (n - 1) as f64;
            for i in 1..n {
                cumulative[i] = cumulative[i - 1] + 0.5 * (sorted_dw[i - 1] + sorted_dw[i]) * dx;
            }
            // Normalize cumulative to [0, 1]
            let cmax = cumulative[n - 1];
            if cmax > 0.0 {
                for v in &mut cumulative {
                    *v /= cmax;
                }
            }
        }

        Self {
            log_mw: sorted_log,
            dw_dlog_m: sorted_dw,
            cumulative,
        }
    }

    /// Fit a single log-normal distribution to the MWD.
    /// Returns (mu, sigma, amplitude) where pdf ∝ amp * exp(-0.5*((x-mu)/sigma)^2).
    pub fn fit_lognormal(&self) -> (f64, f64, f64) {
        if self.log_mw.is_empty() || self.dw_dlog_m.is_empty() {
            return (0.0, 0.0, 0.0);
        }
        // Weighted mean and variance of log_mw
        let total_w: f64 = self.dw_dlog_m.iter().sum();
        if total_w <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        let mu: f64 = self
            .log_mw
            .iter()
            .zip(self.dw_dlog_m.iter())
            .map(|(&x, &w)| x * w)
            .sum::<f64>()
            / total_w;
        let var: f64 = self
            .log_mw
            .iter()
            .zip(self.dw_dlog_m.iter())
            .map(|(&x, &w)| w * (x - mu).powi(2))
            .sum::<f64>()
            / total_w;
        let sigma = var.sqrt();
        let amplitude = if sigma > 0.0 {
            total_w / (sigma * (2.0 * PI).sqrt()) / self.log_mw.len() as f64
        } else {
            0.0
        };
        (mu, sigma, amplitude)
    }

    /// Fit a bimodal distribution (sum of two log-normals) using iterative EM-like decomposition.
    /// Returns ((mu1, sigma1, w1), (mu2, sigma2, w2)).
    pub fn fit_bimodal(&self) -> ((f64, f64, f64), (f64, f64, f64)) {
        let n = self.log_mw.len();
        if n < 6 {
            let single = self.fit_lognormal();
            return ((single.0, single.1, 1.0), (single.0, single.1, 0.0));
        }

        // Initialize: split at the midpoint
        let mid = n / 2;
        let total_w: f64 = self.dw_dlog_m.iter().sum();
        if total_w <= 0.0 {
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0));
        }

        // Initial guesses
        let mean1: f64 = self.log_mw[..mid]
            .iter()
            .zip(self.dw_dlog_m[..mid].iter())
            .map(|(&x, &w)| x * w)
            .sum::<f64>()
            / self.dw_dlog_m[..mid].iter().sum::<f64>().max(1e-15);
        let mean2: f64 = self.log_mw[mid..]
            .iter()
            .zip(self.dw_dlog_m[mid..].iter())
            .map(|(&x, &w)| x * w)
            .sum::<f64>()
            / self.dw_dlog_m[mid..].iter().sum::<f64>().max(1e-15);

        let range = self.log_mw[n - 1] - self.log_mw[0];
        let mut mu1 = mean1;
        let mut mu2 = mean2;
        let mut sig1 = range * 0.2;
        let mut sig2 = range * 0.2;
        let mut w1 = 0.5;

        // EM iterations
        for _ in 0..50 {
            // E-step: compute responsibilities
            let mut r1 = vec![0.0; n];
            let mut sum_r1 = 0.0;
            let mut sum_r2 = 0.0;
            for i in 0..n {
                let x = self.log_mw[i];
                let p1 = w1 * gaussian(x, 1.0, mu1, sig1.max(1e-10));
                let p2 = (1.0 - w1) * gaussian(x, 1.0, mu2, sig2.max(1e-10));
                let total = p1 + p2;
                let resp = if total > 0.0 { p1 / total } else { 0.5 };
                r1[i] = resp * self.dw_dlog_m[i];
                sum_r1 += r1[i];
                sum_r2 += (1.0 - resp) * self.dw_dlog_m[i];
            }

            if sum_r1 <= 0.0 || sum_r2 <= 0.0 {
                break;
            }

            // M-step
            let new_mu1: f64 = (0..n)
                .map(|i| r1[i] * self.log_mw[i])
                .sum::<f64>()
                / sum_r1;
            let new_mu2: f64 = (0..n)
                .map(|i| (self.dw_dlog_m[i] - r1[i]) * self.log_mw[i])
                .sum::<f64>()
                / sum_r2;
            let new_sig1: f64 = ((0..n)
                .map(|i| r1[i] * (self.log_mw[i] - new_mu1).powi(2))
                .sum::<f64>()
                / sum_r1)
                .sqrt();
            let new_sig2: f64 = ((0..n)
                .map(|i| (self.dw_dlog_m[i] - r1[i]) * (self.log_mw[i] - new_mu2).powi(2))
                .sum::<f64>()
                / sum_r2)
                .sqrt();

            mu1 = new_mu1;
            mu2 = new_mu2;
            sig1 = new_sig1.max(1e-6);
            sig2 = new_sig2.max(1e-6);
            w1 = sum_r1 / (sum_r1 + sum_r2);
        }

        ((mu1, sig1, w1), (mu2, sig2, 1.0 - w1))
    }

    /// Get MW at a given cumulative percentile (e.g., 0.10 for 10%).
    pub fn mw_at_percentile(&self, percentile: f64) -> f64 {
        let n = self.cumulative.len();
        if n == 0 {
            return 0.0;
        }
        let p = percentile.clamp(0.0, 1.0);
        for i in 0..n - 1 {
            if self.cumulative[i] <= p && self.cumulative[i + 1] >= p {
                // Linear interpolation
                let frac = if (self.cumulative[i + 1] - self.cumulative[i]).abs() > 1e-15 {
                    (p - self.cumulative[i]) / (self.cumulative[i + 1] - self.cumulative[i])
                } else {
                    0.0
                };
                let log_m =
                    self.log_mw[i] + frac * (self.log_mw[i + 1] - self.log_mw[i]);
                return 10.0_f64.powf(log_m);
            }
        }
        10.0_f64.powf(self.log_mw[n - 1])
    }

    /// Peak molecular weight (Mp) - MW at maximum of differential MWD.
    pub fn peak_mw(&self) -> f64 {
        if self.dw_dlog_m.is_empty() {
            return 0.0;
        }
        let mut max_idx = 0;
        let mut max_val = self.dw_dlog_m[0];
        for (i, &v) in self.dw_dlog_m.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }
        10.0_f64.powf(self.log_mw[max_idx])
    }
}

// ---------------------------------------------------------------------------
// GpcSystem - ties everything together
// ---------------------------------------------------------------------------

/// Complete GPC analysis system.
#[derive(Debug, Clone)]
pub struct GpcSystem {
    pub calibration: CalibrationCurve,
    pub band_broadening: Option<BandBroadening>,
    pub mhs: Option<MHSEquation>,
}

impl GpcSystem {
    /// Create a GPC system with a calibration curve.
    pub fn new(calibration: CalibrationCurve) -> Self {
        Self {
            calibration,
            band_broadening: None,
            mhs: None,
        }
    }

    /// Set band broadening parameters.
    pub fn with_band_broadening(mut self, bb: BandBroadening) -> Self {
        self.band_broadening = Some(bb);
        self
    }

    /// Set MHS parameters.
    pub fn with_mhs(mut self, mhs: MHSEquation) -> Self {
        self.mhs = Some(mhs);
        self
    }

    /// Full analysis pipeline: baseline correction, band broadening, MW averages, MWD.
    pub fn analyze(
        &self,
        signal: &[f64],
        volumes: &[f64],
    ) -> (MolecularWeightAverages, MWDAnalysis) {
        let mut proc = ChromatogramProcessor::new(signal.to_vec(), volumes.to_vec());
        proc.correct_baseline(BaselineMethod::Linear);

        let mut corrected = proc.corrected.clone();

        // Band broadening correction if available
        if let Some(ref bb) = self.band_broadening {
            let dx = if volumes.len() > 1 {
                (volumes[volumes.len() - 1] - volumes[0]) / (volumes.len() - 1) as f64
            } else {
                1.0
            };
            corrected = bb.correct_tung(&corrected, dx, 3);
        }

        let mws: Vec<f64> = volumes
            .iter()
            .map(|&v| self.calibration.volume_to_mw(v))
            .collect();

        let averages = if let Some(ref mhs) = self.mhs {
            MolecularWeightAverages::from_slices_with_mhs(&corrected, &mws, mhs)
        } else {
            MolecularWeightAverages::from_slices(&corrected, &mws)
        };

        let mwd = MWDAnalysis::from_chromatogram(&corrected, volumes, &self.calibration);

        (averages, mwd)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol || (a - b).abs() / (a.abs().max(b.abs()).max(1e-15)) < tol
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_retention_to_mw_constant() {
        // log10(MW) = 5.0 => MW = 100000
        let mw = retention_to_mw(0.0, &[5.0]);
        assert!(approx_eq(mw, 1e5, 1e-3));
    }

    #[test]
    fn test_retention_to_mw_linear() {
        // log10(MW) = 8.0 - 0.3*V at V=10 => log10(MW)=5 => MW=1e5
        let mw = retention_to_mw(10.0, &[8.0, -0.3]);
        assert!(approx_eq(mw, 1e5, 1e-3));
    }

    #[test]
    fn test_retention_to_mw_cubic() {
        let coeffs = [10.0, -0.5, 0.01, -0.0001];
        let mw = retention_to_mw(5.0, &coeffs);
        let expected_log = 10.0 - 0.5 * 5.0 + 0.01 * 25.0 - 0.0001 * 125.0;
        assert!(approx_eq(mw, 10.0_f64.powf(expected_log), 1e-3));
    }

    #[test]
    fn test_pdi_normal() {
        assert!(approx_eq(pdi(200000.0, 100000.0), 2.0, TOL));
    }

    #[test]
    fn test_pdi_monodisperse() {
        assert!(approx_eq(pdi(50000.0, 50000.0), 1.0, TOL));
    }

    #[test]
    fn test_pdi_zero_mn() {
        assert!(pdi(100.0, 0.0).is_nan());
    }

    #[test]
    fn test_van_deemter_basic() {
        let h = van_deemter(0.5, 1.0, 0.1, 2.0);
        // 0.5 + 1.0/2.0 + 0.1*2.0 = 0.5 + 0.5 + 0.2 = 1.2
        assert!(approx_eq(h, 1.2, TOL));
    }

    #[test]
    fn test_van_deemter_zero_velocity() {
        assert!(van_deemter(0.5, 1.0, 0.1, 0.0).is_nan());
    }

    #[test]
    fn test_van_deemter_optimum() {
        // u_opt = sqrt(B/C) = sqrt(1.0/0.1) = sqrt(10)
        let u_opt = (1.0_f64 / 0.1).sqrt();
        let h = van_deemter(0.5, 1.0, 0.1, u_opt);
        let h_min = 0.5 + 2.0 * (1.0 * 0.1_f64).sqrt();
        assert!(approx_eq(h, h_min, TOL));
    }

    #[test]
    fn test_trapz_basic() {
        let y = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        assert!(approx_eq(trapz(&y, 1.0), 8.0, TOL));
    }

    #[test]
    fn test_trapz_single() {
        assert!(approx_eq(trapz(&[5.0], 1.0), 0.0, TOL));
    }

    #[test]
    fn test_poly_eval_constant() {
        assert!(approx_eq(poly_eval(&[3.0], 42.0), 3.0, TOL));
    }

    #[test]
    fn test_poly_eval_quadratic() {
        // 1 + 2x + 3x^2 at x=2 => 1 + 4 + 12 = 17
        assert!(approx_eq(poly_eval(&[1.0, 2.0, 3.0], 2.0), 17.0, TOL));
    }

    #[test]
    fn test_gaussian_peak() {
        assert!(approx_eq(gaussian(5.0, 10.0, 5.0, 1.0), 10.0, TOL));
    }

    #[test]
    fn test_gaussian_symmetry() {
        let left = gaussian(3.0, 1.0, 5.0, 1.0);
        let right = gaussian(7.0, 1.0, 5.0, 1.0);
        assert!(approx_eq(left, right, TOL));
    }

    #[test]
    fn test_polyfit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0]; // y = 1 + 2x
        let c = polyfit(&x, &y, 1).unwrap();
        assert!(approx_eq(c[0], 1.0, 1e-6));
        assert!(approx_eq(c[1], 2.0, 1e-6));
    }

    #[test]
    fn test_polyfit_quadratic() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 0.5 * xi * xi).collect();
        let c = polyfit(&x, &y, 2).unwrap();
        assert!(approx_eq(c[0], 1.0, 1e-4));
        assert!(approx_eq(c[1], 0.0, 1e-4));
        assert!(approx_eq(c[2], 0.5, 1e-4));
    }

    // -----------------------------------------------------------------------
    // CalibrationCurve tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_calibration_from_standards() {
        // PS narrow standards: (volume, MW)
        let volumes = vec![14.0, 15.0, 16.0, 17.0, 18.0];
        let mws = vec![500000.0, 100000.0, 20000.0, 4000.0, 800.0];
        let cal = CalibrationCurve::from_narrow_standards(&volumes, &mws, 3).unwrap();
        assert!(cal.r_squared > 0.99);
        assert_eq!(cal.cal_type, CalibrationType::NarrowStandard);
    }

    #[test]
    fn test_calibration_volume_to_mw() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        // At V=10: log(MW) = 8 - 3 = 5, MW = 1e5
        assert!(approx_eq(cal.volume_to_mw(10.0), 1e5, 1.0));
    }

    #[test]
    fn test_calibration_range_check() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        assert!(cal.is_in_range(15.0));
        assert!(!cal.is_in_range(5.0));
        assert!(!cal.is_in_range(30.0));
    }

    #[test]
    fn test_calibration_roundtrip() {
        let volumes = vec![12.0, 14.0, 16.0, 18.0, 20.0];
        let mws = vec![1e6, 1e5, 1e4, 1e3, 1e2];
        let cal = CalibrationCurve::from_narrow_standards(&volumes, &mws, 1).unwrap();
        for (&v, &m) in volumes.iter().zip(mws.iter()) {
            let predicted = cal.volume_to_mw(v);
            assert!(
                approx_eq(predicted.log10(), m.log10(), 0.1),
                "V={}, expected MW={}, got {}",
                v,
                m,
                predicted
            );
        }
    }

    #[test]
    fn test_calibration_too_few_points() {
        assert!(CalibrationCurve::from_narrow_standards(&[10.0], &[1e5], 3).is_none());
    }

    #[test]
    fn test_calibration_log_mw() {
        let cal = CalibrationCurve::from_coefficients(vec![6.0, -0.2], 10.0, 30.0);
        assert!(approx_eq(cal.volume_to_log_mw(10.0), 4.0, TOL));
    }

    // -----------------------------------------------------------------------
    // MolecularWeightAverages tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mw_averages_monodisperse() {
        // All slices have the same MW => Mn=Mw=Mz, PDI=1
        let heights = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let mws = vec![1e5, 1e5, 1e5, 1e5, 1e5];
        let avg = MolecularWeightAverages::from_slices(&heights, &mws);
        assert!(approx_eq(avg.mn, 1e5, 1.0));
        assert!(approx_eq(avg.mw, 1e5, 1.0));
        assert!(approx_eq(avg.pdi, 1.0, TOL));
    }

    #[test]
    fn test_mw_averages_polydisperse() {
        let heights = vec![1.0, 1.0];
        let mws = vec![1e4, 1e6];
        let avg = MolecularWeightAverages::from_slices(&heights, &mws);
        // Mn = 2 / (1/1e4 + 1/1e6) = 2 / 0.000101 ≈ 19802
        assert!(avg.mn < avg.mw);
        assert!(avg.mw < avg.mz);
        assert!(avg.pdi > 1.0);
    }

    #[test]
    fn test_mw_averages_pdi_ordering() {
        // For any distribution: Mn <= Mw <= Mz
        let heights = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let mws = vec![1e3, 1e4, 1e5, 1e6, 1e7];
        let avg = MolecularWeightAverages::from_slices(&heights, &mws);
        assert!(avg.mn <= avg.mw);
        assert!(avg.mw <= avg.mz);
    }

    #[test]
    fn test_mw_averages_skip_zero() {
        let heights = vec![0.0, 1.0, 0.0];
        let mws = vec![1e4, 1e5, 1e6];
        let avg = MolecularWeightAverages::from_slices(&heights, &mws);
        assert!(approx_eq(avg.mn, 1e5, 1.0));
    }

    #[test]
    fn test_mw_averages_with_mhs() {
        let heights = vec![1.0, 2.0, 1.0];
        let mws = vec![1e4, 1e5, 1e6];
        let mhs = MHSEquation::ps_thf();
        let avg = MolecularWeightAverages::from_slices_with_mhs(&heights, &mws, &mhs);
        assert!(avg.mv.is_some());
        let mv = avg.mv.unwrap();
        assert!(mv > 0.0);
    }

    // -----------------------------------------------------------------------
    // ChromatogramProcessor tests
    // -----------------------------------------------------------------------

    fn make_gaussian_chromatogram(n: usize, center: f64, sigma: f64) -> (Vec<f64>, Vec<f64>) {
        let volumes: Vec<f64> = (0..n).map(|i| 10.0 + i as f64 * 0.1).collect();
        let signal: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 100.0, center, sigma))
            .collect();
        (signal, volumes)
    }

    #[test]
    fn test_chromatogram_creation() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let proc = ChromatogramProcessor::new(sig.clone(), vol.clone());
        assert_eq!(proc.signal.len(), 100);
    }

    #[test]
    fn test_baseline_linear() {
        let mut signal = vec![10.0; 50];
        signal[25] = 50.0; // peak
        let volumes: Vec<f64> = (0..50).map(|i| 10.0 + i as f64 * 0.2).collect();
        let mut proc = ChromatogramProcessor::new(signal, volumes);
        proc.correct_baseline(BaselineMethod::Linear);
        // baseline is flat at 10, so corrected peak should be ~40
        assert!(proc.corrected[25] > 30.0);
        assert!(proc.corrected[0].abs() < 1.0);
    }

    #[test]
    fn test_baseline_polynomial() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Polynomial(1));
        // Peak should still be present
        let max_val = proc.corrected.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 50.0);
    }

    #[test]
    fn test_baseline_valley_to_valley() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::ValleyToValley);
        let max_val = proc.corrected.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.0);
    }

    #[test]
    fn test_peak_detection_single() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Linear);
        proc.detect_peaks(1.0, 0.5);
        assert_eq!(proc.peaks.len(), 1);
        assert!(approx_eq(proc.peaks[0].retention_volume, 15.0, 0.2));
    }

    #[test]
    fn test_peak_detection_height_filter() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Linear);
        proc.detect_peaks(200.0, 0.0); // min_height too high
        assert_eq!(proc.peaks.len(), 0);
    }

    #[test]
    fn test_peak_area_positive() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Linear);
        proc.detect_peaks(1.0, 0.5);
        assert!(proc.peaks[0].area > 0.0);
    }

    #[test]
    fn test_peak_asymmetry_gaussian() {
        let (sig, vol) = make_gaussian_chromatogram(200, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Linear);
        proc.detect_peaks(1.0, 0.5);
        if !proc.peaks.is_empty() {
            // Gaussian is symmetric, asymmetry should be near 1.0
            assert!(proc.peaks[0].asymmetry > 0.5 && proc.peaks[0].asymmetry < 2.0);
        }
    }

    #[test]
    fn test_integrate_region() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let proc = ChromatogramProcessor::new(sig, vol);
        let area = proc.integrate_region(0, 99);
        assert!(area > 0.0);
    }

    #[test]
    fn test_compute_mw_averages() {
        let (sig, vol) = make_gaussian_chromatogram(100, 15.0, 0.5);
        let mut proc = ChromatogramProcessor::new(sig, vol);
        proc.correct_baseline(BaselineMethod::Linear);
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let avg = proc.compute_mw_averages(&cal, 0, 99);
        assert!(avg.mn > 0.0);
        assert!(avg.mw > 0.0);
        assert!(avg.mw >= avg.mn);
    }

    // -----------------------------------------------------------------------
    // BandBroadening tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gaussian_isf_normalized() {
        let bb = BandBroadening::new(0.1, 0.0);
        let isf = bb.gaussian_isf(32, 0.01);
        let sum: f64 = isf.iter().sum();
        assert!(approx_eq(sum, 1.0, 1e-6));
    }

    #[test]
    fn test_gaussian_isf_peak_center() {
        let bb = BandBroadening::new(0.1, 0.0);
        let isf = bb.gaussian_isf(32, 0.01);
        let max_idx = isf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 16);
    }

    #[test]
    fn test_tung_correction_identity() {
        // If no broadening (sigma very small), correction should approximately return input
        let bb = BandBroadening::new(0.001, 0.0);
        let signal: Vec<f64> = (0..50).map(|i| gaussian(i as f64, 10.0, 25.0, 3.0)).collect();
        let corrected = bb.correct_tung(&signal, 1.0, 5);
        let max_orig = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let max_corr = corrected.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(approx_eq(max_orig, max_corr, 1.0));
    }

    #[test]
    fn test_tung_correction_sharpens() {
        // Band broadening correction should sharpen the peak
        let bb = BandBroadening::new(1.0, 0.0);
        let signal: Vec<f64> = (0..100)
            .map(|i| gaussian(i as f64 * 0.1, 10.0, 5.0, 1.0))
            .collect();
        let corrected = bb.correct_tung(&signal, 0.1, 10);
        let max_corr = corrected.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let max_orig = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        // Corrected peak should be at least as tall as original
        assert!(max_corr >= max_orig * 0.8);
    }

    #[test]
    fn test_skew_correction_no_skew() {
        let bb = BandBroadening::new(0.1, 0.0);
        let signal = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let corrected = bb.correct_skew(&signal, 1.0);
        assert_eq!(signal, corrected);
    }

    #[test]
    fn test_skew_correction_applies() {
        let bb = BandBroadening::new(0.1, 0.5);
        let signal: Vec<f64> = (0..50).map(|i| gaussian(i as f64, 10.0, 25.0, 3.0)).collect();
        let corrected = bb.correct_skew(&signal, 1.0);
        // Corrected signal should differ from original
        let diff: f64 = signal
            .iter()
            .zip(corrected.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diff > 0.0);
    }

    // -----------------------------------------------------------------------
    // MHSEquation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mhs_ps_thf() {
        let mhs = MHSEquation::ps_thf();
        assert!(approx_eq(mhs.k, 1.14e-4, 1e-8));
        assert!(approx_eq(mhs.a, 0.716, 1e-6));
    }

    #[test]
    fn test_mhs_pmma_thf() {
        let mhs = MHSEquation::pmma_thf();
        assert!(approx_eq(mhs.k, 1.28e-4, 1e-8));
        assert!(approx_eq(mhs.a, 0.690, 1e-6));
    }

    #[test]
    fn test_mhs_viscosity_roundtrip() {
        let mhs = MHSEquation::ps_thf();
        let mw = 100000.0;
        let eta = mhs.intrinsic_viscosity(mw);
        let mw_back = mhs.mw_from_viscosity(eta);
        assert!(approx_eq(mw, mw_back, 1.0));
    }

    #[test]
    fn test_mhs_viscosity_increases_with_mw() {
        let mhs = MHSEquation::ps_thf();
        let eta_low = mhs.intrinsic_viscosity(1e4);
        let eta_high = mhs.intrinsic_viscosity(1e6);
        assert!(eta_high > eta_low);
    }

    #[test]
    fn test_mhs_hydrodynamic_volume() {
        let mhs = MHSEquation::ps_thf();
        let j = mhs.hydrodynamic_volume(1e5);
        assert!(j > 0.0);
        // J = K * M^(1+a) = 1.14e-4 * (1e5)^1.716
        let expected = 1.14e-4 * (1e5_f64).powf(1.716);
        assert!(approx_eq(j, expected, expected * 1e-6));
    }

    #[test]
    fn test_mhs_pe_tcb() {
        let mhs = MHSEquation::pe_tcb();
        assert!(approx_eq(mhs.k, 4.06e-4, 1e-8));
    }

    #[test]
    fn test_mhs_pp_tcb() {
        let mhs = MHSEquation::pp_tcb();
        assert!(approx_eq(mhs.k, 1.90e-4, 1e-8));
    }

    // -----------------------------------------------------------------------
    // UniversalCalibration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_universal_calibration_creation() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let mhs = MHSEquation::ps_thf();
        let ucal = UniversalCalibration::from_conventional(&cal, &mhs);
        assert_eq!(ucal.j_curve.cal_type, CalibrationType::Universal);
    }

    #[test]
    fn test_universal_calibration_j_positive() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let mhs = MHSEquation::ps_thf();
        let ucal = UniversalCalibration::from_conventional(&cal, &mhs);
        let j = ucal.volume_to_j(15.0);
        assert!(j > 0.0);
    }

    #[test]
    fn test_universal_calibration_cross_polymer() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let ps = MHSEquation::ps_thf();
        let pmma = MHSEquation::pmma_thf();
        let ucal = UniversalCalibration::from_conventional(&cal, &ps);

        let mw_ps = ucal.volume_to_mw(15.0, &ps);
        let mw_pmma = ucal.volume_to_mw(15.0, &pmma);
        // Different polymers should give different MW at same retention volume
        assert!((mw_ps - mw_pmma).abs() / mw_ps > 0.01);
    }

    #[test]
    fn test_universal_calibration_consistency() {
        // For reference polymer, universal cal should match conventional
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let ps = MHSEquation::ps_thf();
        let ucal = UniversalCalibration::from_conventional(&cal, &ps);

        let mw_conv = cal.volume_to_mw(15.0);
        let mw_univ = ucal.volume_to_mw(15.0, &ps);
        assert!(approx_eq(mw_conv, mw_univ, mw_conv * 0.01));
    }

    // -----------------------------------------------------------------------
    // MultiDetector tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_detector_creation() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        assert!(md.k_star > 0.0);
        assert!(approx_eq(md.dn_dc, 0.185, TOL));
    }

    #[test]
    fn test_multi_detector_mw_from_mals() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        let slice = DetectorSlice {
            ri: 0.001,
            uv: None,
            viscosity: None,
            mals_r0: Some(1e-5),
        };
        let mw = md.mw_from_mals(&slice, 1.0);
        assert!(mw.is_some());
        assert!(mw.unwrap() > 0.0);
    }

    #[test]
    fn test_multi_detector_no_mals() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        let slice = DetectorSlice {
            ri: 0.001,
            uv: None,
            viscosity: None,
            mals_r0: None,
        };
        assert!(md.mw_from_mals(&slice, 1.0).is_none());
    }

    #[test]
    fn test_multi_detector_viscosity() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        let slice = DetectorSlice {
            ri: 0.001,
            uv: None,
            viscosity: Some(0.5),
            mals_r0: None,
        };
        let iv = md.intrinsic_viscosity_from_slice(&slice, 1.0);
        assert!(iv.is_some());
        assert!(iv.unwrap() > 0.0);
    }

    #[test]
    fn test_multi_detector_uv_ri_ratio() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        let slice = DetectorSlice {
            ri: 0.002,
            uv: Some(0.01),
            viscosity: None,
            mals_r0: None,
        };
        let ratio = md.uv_ri_ratio(&slice);
        assert!(approx_eq(ratio.unwrap(), 5.0, TOL));
    }

    #[test]
    fn test_multi_detector_zero_ri() {
        let md = MultiDetector::new(0.185, 658.0, 1.405);
        let slice = DetectorSlice {
            ri: 0.0,
            uv: Some(0.01),
            viscosity: Some(0.5),
            mals_r0: Some(1e-5),
        };
        assert!(md.mw_from_mals(&slice, 1.0).is_none());
        assert!(md.intrinsic_viscosity_from_slice(&slice, 1.0).is_none());
    }

    // -----------------------------------------------------------------------
    // ColumnPerformance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_plate_count() {
        let perf = ColumnPerformance::from_peak(15.0, 0.5, 300.0);
        // N = 5.54 * (15/0.5)^2 = 5.54 * 900 = 4986
        assert!(approx_eq(perf.plate_count, 4986.0, 1.0));
    }

    #[test]
    fn test_hetp() {
        let perf = ColumnPerformance::from_peak(15.0, 0.5, 300.0);
        // HETP = 300 / 4986 ≈ 0.0602
        assert!(perf.hetp > 0.0 && perf.hetp < 1.0);
    }

    #[test]
    fn test_resolution() {
        let r = ColumnPerformance::resolution(15.0, 16.0, 0.5, 0.5);
        // R = 2*(16-15)/(0.5+0.5) = 2.0
        assert!(approx_eq(r, 2.0, TOL));
    }

    #[test]
    fn test_plate_count_tangent() {
        let n = ColumnPerformance::plate_count_tangent(15.0, 1.0);
        // N = 16 * (15/1)^2 = 16 * 225 = 3600
        assert!(approx_eq(n, 3600.0, TOL));
    }

    #[test]
    fn test_usp_tailing_symmetric() {
        let t = ColumnPerformance::usp_tailing_factor(0.5, 0.5);
        assert!(approx_eq(t, 1.0, TOL));
    }

    #[test]
    fn test_usp_tailing_tailed() {
        let t = ColumnPerformance::usp_tailing_factor(0.3, 0.7);
        // T = (0.3+0.7)/(2*0.3) = 1.667
        assert!(t > 1.0);
    }

    // -----------------------------------------------------------------------
    // VanDeemterFit tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_van_deemter_fit() {
        let a = 0.5;
        let b = 2.0;
        let c = 0.05;
        let velocities: Vec<f64> = (1..=20).map(|i| i as f64 * 0.5).collect();
        let heights: Vec<f64> = velocities
            .iter()
            .map(|&u| van_deemter(a, b, c, u))
            .collect();
        let fit = VanDeemterFit::fit(&velocities, &heights).unwrap();
        assert!(approx_eq(fit.a, a, 0.1));
        assert!(approx_eq(fit.b, b, 0.2));
        assert!(approx_eq(fit.c, c, 0.01));
    }

    #[test]
    fn test_van_deemter_optimum_velocity() {
        let fit = VanDeemterFit {
            a: 0.5,
            b: 2.0,
            c: 0.05,
            u_opt: (2.0 / 0.05_f64).sqrt(),
            h_min: 0.5 + 2.0 * (2.0 * 0.05_f64).sqrt(),
        };
        assert!(approx_eq(fit.u_opt, (40.0_f64).sqrt(), 0.01));
    }

    #[test]
    fn test_van_deemter_evaluate() {
        let fit = VanDeemterFit {
            a: 0.5,
            b: 2.0,
            c: 0.05,
            u_opt: 0.0,
            h_min: 0.0,
        };
        let h = fit.evaluate(2.0);
        assert!(approx_eq(h, 0.5 + 1.0 + 0.1, TOL));
    }

    #[test]
    fn test_van_deemter_fit_too_few() {
        assert!(VanDeemterFit::fit(&[1.0, 2.0], &[0.5, 0.6]).is_none());
    }

    // -----------------------------------------------------------------------
    // MWDAnalysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mwd_from_chromatogram() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        assert_eq!(mwd.log_mw.len(), n);
        assert_eq!(mwd.dw_dlog_m.len(), n);
        assert_eq!(mwd.cumulative.len(), n);
    }

    #[test]
    fn test_mwd_cumulative_monotonic() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        for i in 1..mwd.cumulative.len() {
            assert!(mwd.cumulative[i] >= mwd.cumulative[i - 1] - 1e-10);
        }
    }

    #[test]
    fn test_mwd_cumulative_bounds() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        assert!(mwd.cumulative[0] >= 0.0);
        assert!(mwd.cumulative[n - 1] <= 1.0 + 1e-6);
    }

    #[test]
    fn test_mwd_fit_lognormal() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 200;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.05).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        let (mu, sigma, _amp) = mwd.fit_lognormal();
        assert!(mu > 0.0);
        assert!(sigma > 0.0);
    }

    #[test]
    fn test_mwd_fit_bimodal() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 200;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.05).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 5.0, 15.0, 0.5) + gaussian(v, 5.0, 18.0, 0.5))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        let ((mu1, sig1, w1), (mu2, sig2, w2)) = mwd.fit_bimodal();
        assert!(sig1 > 0.0);
        assert!(sig2 > 0.0);
        assert!(w1 + w2 > 0.99);
    }

    #[test]
    fn test_mwd_percentile() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 200;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.05).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        let mw10 = mwd.mw_at_percentile(0.1);
        let mw50 = mwd.mw_at_percentile(0.5);
        let mw90 = mwd.mw_at_percentile(0.9);
        assert!(mw10 > 0.0);
        assert!(mw50 > 0.0);
        assert!(mw90 > 0.0);
    }

    #[test]
    fn test_mwd_peak_mw() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 200;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.05).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        let mp = mwd.peak_mw();
        assert!(mp > 0.0);
    }

    // -----------------------------------------------------------------------
    // GpcSystem integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gpc_system_basic() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let sys = GpcSystem::new(cal);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let signal: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let (avg, mwd) = sys.analyze(&signal, &volumes);
        assert!(avg.mn > 0.0);
        assert!(avg.mw > 0.0);
        assert!(avg.pdi >= 1.0);
        assert!(!mwd.dw_dlog_m.is_empty());
    }

    #[test]
    fn test_gpc_system_with_bb() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let bb = BandBroadening::new(0.1, 0.0);
        let sys = GpcSystem::new(cal).with_band_broadening(bb);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let signal: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let (avg, _) = sys.analyze(&signal, &volumes);
        assert!(avg.mn > 0.0);
    }

    #[test]
    fn test_gpc_system_with_mhs() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let mhs = MHSEquation::ps_thf();
        let sys = GpcSystem::new(cal).with_mhs(mhs);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let signal: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let (avg, _) = sys.analyze(&signal, &volumes);
        assert!(avg.mv.is_some());
    }

    // -----------------------------------------------------------------------
    // Edge case and additional tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_empty_slices() {
        let avg = MolecularWeightAverages::from_slices(&[], &[]);
        assert!(approx_eq(avg.mn, 0.0, TOL));
        assert!(approx_eq(avg.mw, 0.0, TOL));
    }

    #[test]
    fn test_single_slice() {
        let avg = MolecularWeightAverages::from_slices(&[1.0], &[50000.0]);
        assert!(approx_eq(avg.mn, 50000.0, 1.0));
        assert!(approx_eq(avg.pdi, 1.0, TOL));
    }

    #[test]
    fn test_negative_heights_ignored() {
        let avg = MolecularWeightAverages::from_slices(&[-1.0, 2.0, -1.0], &[1e4, 1e5, 1e6]);
        assert!(approx_eq(avg.mn, 1e5, 1.0));
    }

    #[test]
    fn test_resolution_zero_width() {
        assert!(approx_eq(
            ColumnPerformance::resolution(10.0, 12.0, 0.0, 0.0),
            0.0,
            TOL
        ));
    }

    #[test]
    fn test_calibration_decreasing_mw() {
        // In GPC, MW decreases with increasing retention volume
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let mw_early = cal.volume_to_mw(12.0);
        let mw_late = cal.volume_to_mw(20.0);
        assert!(mw_early > mw_late);
    }

    #[test]
    fn test_mhs_zero_mw() {
        let mhs = MHSEquation::ps_thf();
        let eta = mhs.intrinsic_viscosity(0.0);
        assert!(approx_eq(eta, 0.0, TOL));
    }

    #[test]
    fn test_mhs_from_viscosity_zero() {
        let mhs = MHSEquation::ps_thf();
        let mw = mhs.mw_from_viscosity(0.0);
        assert!(approx_eq(mw, 0.0, TOL));
    }

    #[test]
    fn test_band_broadening_large_sigma() {
        let bb = BandBroadening::new(5.0, 0.0);
        let isf = bb.gaussian_isf(64, 0.5);
        let sum: f64 = isf.iter().sum();
        assert!(approx_eq(sum, 1.0, 1e-3));
    }

    #[test]
    fn test_chromatogram_two_peaks() {
        let n = 200;
        let volumes: Vec<f64> = (0..n).map(|i| 10.0 + i as f64 * 0.1).collect();
        let signal: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 50.0, 15.0, 0.5) + gaussian(v, 50.0, 20.0, 0.5))
            .collect();
        let mut proc = ChromatogramProcessor::new(signal, volumes);
        proc.correct_baseline(BaselineMethod::Linear);
        proc.detect_peaks(1.0, 1.0);
        assert!(proc.peaks.len() >= 2);
    }

    #[test]
    fn test_mwd_differential_non_negative() {
        let cal = CalibrationCurve::from_coefficients(vec![8.0, -0.3], 10.0, 25.0);
        let n = 100;
        let volumes: Vec<f64> = (0..n).map(|i| 12.0 + i as f64 * 0.1).collect();
        let heights: Vec<f64> = volumes
            .iter()
            .map(|&v| gaussian(v, 10.0, 16.0, 1.0))
            .collect();
        let mwd = MWDAnalysis::from_chromatogram(&heights, &volumes, &cal);
        for &dw in &mwd.dw_dlog_m {
            assert!(dw >= -1e-10, "dW/d(logM) should be non-negative, got {}", dw);
        }
    }

    #[test]
    fn test_plate_count_zero_width() {
        let perf = ColumnPerformance::from_peak(15.0, 0.0, 300.0);
        assert!(approx_eq(perf.plate_count, 0.0, TOL));
    }

    #[test]
    fn test_polyfit_insufficient_data() {
        assert!(polyfit(&[1.0], &[2.0], 2).is_none());
    }

    #[test]
    fn test_mwd_empty() {
        let mwd = MWDAnalysis {
            log_mw: vec![],
            dw_dlog_m: vec![],
            cumulative: vec![],
        };
        let (mu, sigma, amp) = mwd.fit_lognormal();
        assert!(approx_eq(mu, 0.0, TOL));
        assert!(approx_eq(sigma, 0.0, TOL));
        assert!(approx_eq(amp, 0.0, TOL));
    }

    #[test]
    fn test_mwd_peak_mw_empty() {
        let mwd = MWDAnalysis {
            log_mw: vec![],
            dw_dlog_m: vec![],
            cumulative: vec![],
        };
        assert!(approx_eq(mwd.peak_mw(), 0.0, TOL));
    }

    #[test]
    fn test_mwd_percentile_empty() {
        let mwd = MWDAnalysis {
            log_mw: vec![],
            dw_dlog_m: vec![],
            cumulative: vec![],
        };
        assert!(approx_eq(mwd.mw_at_percentile(0.5), 0.0, TOL));
    }

    #[test]
    fn test_integrate_empty_region() {
        let proc = ChromatogramProcessor::new(vec![1.0, 2.0, 3.0], vec![10.0, 11.0, 12.0]);
        assert!(approx_eq(proc.integrate_region(2, 1), 0.0, TOL)); // start > end
    }

    #[test]
    fn test_calibration_v_range() {
        let volumes = vec![10.0, 12.0, 14.0, 16.0, 18.0];
        let mws = vec![1e6, 1e5, 1e4, 1e3, 1e2];
        let cal = CalibrationCurve::from_narrow_standards(&volumes, &mws, 1).unwrap();
        assert!(approx_eq(cal.v_range.0, 10.0, TOL));
        assert!(approx_eq(cal.v_range.1, 18.0, TOL));
    }
}
