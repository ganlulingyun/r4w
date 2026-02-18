//! Ion Chromatography (IC) Signal Processing
//!
//! Implements signal processing for quantitative analysis of ionic species in
//! solution using ion chromatography.  Covers eluent suppression, ion
//! separation modelling, chromatogram peak detection/fitting, calibration,
//! column efficiency metrics, gradient elution, matrix elimination, EPA water
//! quality compliance, and system suitability testing.
//!
//! ## Principles
//!
//! Ion chromatography separates ionic analytes on an ion-exchange column.
//! Eluent suppression (chemical or electrolytic) removes background
//! conductivity, dramatically enhancing analyte detection.  Peaks are
//! identified by retention time, and concentrations are determined via
//! external-standard or standard-addition calibration.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ion_chromatography_analyzer::{
//!     ChromatogramPeakFinder, CalibrationProcessor, CalibrationMode,
//!     retention_factor, resolution, plates_from_peak,
//! };
//!
//! // Build a synthetic chromatogram with two Gaussian peaks
//! let n = 1000;
//! let dt = 20.0 / n as f64;
//! let time: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
//! let signal: Vec<f64> = time.iter().map(|&t| {
//!     500.0 * (-((t - 5.0f64).powi(2)) / (2.0 * 0.15f64.powi(2))).exp()
//!   + 300.0 * (-((t - 8.0f64).powi(2)) / (2.0 * 0.2f64.powi(2))).exp()
//! }).collect();
//!
//! let finder = ChromatogramPeakFinder::new(0.01, 3);
//! let peaks = finder.find_peaks(&time, &signal);
//! assert!(peaks.len() >= 2);
//!
//! // Retention factor with dead time t0 = 1.5 min
//! let k = retention_factor(5.0, 1.5);
//! assert!((k - 2.333).abs() < 0.01);
//! ```

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// ln(2)
const LN2: f64 = 0.693147180559945;

/// sqrt(2 * pi)
const SQRT_2PI: f64 = 2.5066282746310002;

/// sqrt(2 * ln(2)) -- for FWHM <-> sigma conversion
const SQRT_2LN2: f64 = 1.1774100225154747;

/// 2 * sqrt(2 * ln(2))
const FWHM_FACTOR: f64 = 2.3548200450309493;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Retention factor k' = (tR - t0) / t0.
///
/// * `tr` - retention time of the analyte (minutes).
/// * `t0` - void (dead) time of the column (minutes).
pub fn retention_factor(tr: f64, t0: f64) -> f64 {
    assert!(t0 > 0.0, "dead time must be positive");
    (tr - t0) / t0
}

/// Resolution between two adjacent peaks.
///
/// Rs = 2 (tR2 - tR1) / (W1 + W2)
///
/// * `tr1`, `tr2` - retention times.
/// * `w1`, `w2` - peak widths at base.
pub fn resolution(tr1: f64, tr2: f64, w1: f64, w2: f64) -> f64 {
    2.0 * (tr2 - tr1).abs() / (w1 + w2)
}

/// Theoretical plates from peak width at half-height.
///
/// N = 5.545 * (tR / w_half)^2
pub fn plates_from_peak(tr: f64, w_half: f64) -> f64 {
    assert!(w_half > 0.0, "peak width must be positive");
    5.545 * (tr / w_half).powi(2)
}

/// Theoretical plates using base width.
///
/// N = 16 * (tR / W)^2
pub fn plates_from_base_width(tr: f64, w_base: f64) -> f64 {
    assert!(w_base > 0.0, "base width must be positive");
    16.0 * (tr / w_base).powi(2)
}

/// Plate height H = L / N.
pub fn plate_height(column_length_mm: f64, n_plates: f64) -> f64 {
    assert!(n_plates > 0.0, "plate count must be positive");
    column_length_mm / n_plates
}

/// Asymmetry factor As = b / a measured at 10 % peak height.
/// `a` is the front half-width, `b` is the back half-width.
pub fn asymmetry_factor(a: f64, b: f64) -> f64 {
    assert!(a > 0.0, "front half-width must be positive");
    b / a
}

/// Trapezoidal integration of evenly-spaced data.
fn trapz(y: &[f64], dx: f64) -> f64 {
    if y.len() < 2 {
        return 0.0;
    }
    let mut sum = 0.5 * (y[0] + y[y.len() - 1]);
    for v in &y[1..y.len() - 1] {
        sum += v;
    }
    sum * dx
}

/// Gaussian function: A * exp(-(x-mu)^2 / (2*sigma^2))
fn gaussian(x: f64, amplitude: f64, mu: f64, sigma: f64) -> f64 {
    amplitude * (-((x - mu).powi(2)) / (2.0 * sigma * sigma)).exp()
}

/// Simple linear regression y = a + b*x.  Returns (a, b, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxx: f64 = x.iter().map(|v| v * v).sum();
    let sxy: f64 = x.iter().zip(y).map(|(a, b)| a * b).sum();
    let syy: f64 = y.iter().map(|v| v * v).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, 0.0, 0.0);
    }
    let b_coeff = (n * sxy - sx * sy) / denom;
    let a_coeff = (sy - b_coeff * sx) / n;
    let ss_res: f64 = x
        .iter()
        .zip(y)
        .map(|(xi, yi)| {
            let pred = a_coeff + b_coeff * xi;
            (yi - pred).powi(2)
        })
        .sum();
    let y_mean = sy / n;
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
    let r2 = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };
    (a_coeff, b_coeff, r2)
}

/// Quadratic regression y = a + b*x + c*x^2.  Returns (a, b, c, r_squared).
fn quadratic_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64, f64) {
    // Normal equations for polynomial fit degree 2:
    // [n     Sx   Sx2 ] [a]   [Sy  ]
    // [Sx    Sx2  Sx3 ] [b] = [Sxy ]
    // [Sx2   Sx3  Sx4 ] [c]   [Sx2y]
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sx2: f64 = x.iter().map(|v| v.powi(2)).sum();
    let sx3: f64 = x.iter().map(|v| v.powi(3)).sum();
    let sx4: f64 = x.iter().map(|v| v.powi(4)).sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y).map(|(a, b)| a * b).sum();
    let sx2y: f64 = x.iter().zip(y).map(|(a, b)| a * a * b).sum();

    // Solve 3x3 system via Cramer's rule
    let det = |m: [[f64; 3]; 3]| -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };

    let mat = [[n, sx, sx2], [sx, sx2, sx3], [sx2, sx3, sx4]];
    let d = det(mat);
    if d.abs() < 1e-30 {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let mat_a = [[sy, sx, sx2], [sxy, sx2, sx3], [sx2y, sx3, sx4]];
    let mat_b = [[n, sy, sx2], [sx, sxy, sx3], [sx2, sx2y, sx4]];
    let mat_c = [[n, sx, sy], [sx, sx2, sxy], [sx2, sx3, sx2y]];
    let a = det(mat_a) / d;
    let b = det(mat_b) / d;
    let c = det(mat_c) / d;

    let y_mean = sy / n;
    let ss_res: f64 = x
        .iter()
        .zip(y)
        .map(|(xi, yi)| {
            let pred = a + b * xi + c * xi * xi;
            (yi - pred).powi(2)
        })
        .sum();
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
    let r2 = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };
    (a, b, c, r2)
}

/// Standard deviation of a slice.
fn std_dev(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    let var = data.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
    var.sqrt()
}

/// Mean of a slice.
fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

/// Relative standard deviation (coefficient of variation) in percent.
fn rsd_percent(data: &[f64]) -> f64 {
    let m = mean(data);
    if m.abs() < 1e-30 {
        return 0.0;
    }
    (std_dev(data) / m.abs()) * 100.0
}

// ---------------------------------------------------------------------------
// 1. SuppressedConductivity
// ---------------------------------------------------------------------------

/// Eluent suppression model for ion chromatography.
///
/// Chemical suppression removes eluent conductance, enhancing analyte signal.
/// The suppressor converts the eluent counter-ion to a weakly conducting form
/// (e.g., NaOH -> H2O for anion analysis).
#[derive(Debug, Clone)]
pub struct SuppressedConductivity {
    /// Background conductivity of suppressed eluent (uS/cm).
    pub background_conductivity: f64,
    /// Suppressor capacity in milliequivalents (meq).
    pub suppressor_capacity_meq: f64,
    /// Current eluent consumption of suppressor capacity (meq).
    pub consumed_capacity_meq: f64,
    /// Suppressor efficiency (0.0 .. 1.0, typically > 0.98).
    pub efficiency: f64,
}

impl SuppressedConductivity {
    /// Create a new suppressor model.
    pub fn new(background: f64, capacity_meq: f64, efficiency: f64) -> Self {
        Self {
            background_conductivity: background,
            suppressor_capacity_meq: capacity_meq,
            consumed_capacity_meq: 0.0,
            efficiency: efficiency.clamp(0.0, 1.0),
        }
    }

    /// Default suppressor (Metrohm MSM-style).
    pub fn default_anion() -> Self {
        Self::new(0.5, 100.0, 0.99)
    }

    /// Default cation suppressor.
    pub fn default_cation() -> Self {
        Self::new(0.3, 80.0, 0.98)
    }

    /// Subtract background conductivity from raw signal.
    pub fn suppress(&self, raw_signal: &[f64]) -> Vec<f64> {
        raw_signal
            .iter()
            .map(|&v| {
                let suppressed = v - self.background_conductivity * self.efficiency;
                suppressed.max(0.0)
            })
            .collect()
    }

    /// Consume suppressor capacity for a given eluent volume and normality.
    /// Returns remaining capacity fraction.
    pub fn consume(&mut self, volume_ml: f64, normality_meq_per_ml: f64) -> f64 {
        self.consumed_capacity_meq += volume_ml * normality_meq_per_ml;
        self.remaining_fraction()
    }

    /// Fraction of suppressor capacity remaining.
    pub fn remaining_fraction(&self) -> f64 {
        let remaining = self.suppressor_capacity_meq - self.consumed_capacity_meq;
        (remaining / self.suppressor_capacity_meq).clamp(0.0, 1.0)
    }

    /// Check if suppressor is exhausted (< 5% capacity).
    pub fn is_exhausted(&self) -> bool {
        self.remaining_fraction() < 0.05
    }

    /// Regenerate (reset) suppressor.
    pub fn regenerate(&mut self) {
        self.consumed_capacity_meq = 0.0;
    }

    /// Estimate analyte conductivity from concentration.
    /// Approximate: conductivity ~ equivalent_conductance * concentration_meq_per_L.
    pub fn estimated_conductivity(&self, equiv_conductance: f64, concentration_meq_l: f64) -> f64 {
        equiv_conductance * concentration_meq_l
    }
}

// ---------------------------------------------------------------------------
// 2. IonSeparation
// ---------------------------------------------------------------------------

/// Standard anion species.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AnionSpecies {
    Fluoride,
    Chloride,
    Nitrite,
    Bromide,
    Nitrate,
    Phosphate,
    Sulfate,
}

/// Standard cation species.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CationSpecies {
    Lithium,
    Sodium,
    Ammonium,
    Potassium,
    Magnesium,
    Calcium,
}

/// Ion separation model: selectivity coefficients and exchange equilibrium.
#[derive(Debug, Clone)]
pub struct IonSeparation {
    /// Selectivity coefficients for anions (relative to Cl-).
    pub anion_selectivity: [(AnionSpecies, f64); 7],
    /// Selectivity coefficients for cations (relative to Na+).
    pub cation_selectivity: [(CationSpecies, f64); 6],
    /// Donnan exclusion factor (0..1; higher = better exclusion of co-ions).
    pub donnan_exclusion: f64,
}

impl IonSeparation {
    /// Default selectivity coefficients for a typical AS22 anion column.
    pub fn default_anion() -> Self {
        Self {
            anion_selectivity: [
                (AnionSpecies::Fluoride, 0.10),
                (AnionSpecies::Chloride, 1.00),
                (AnionSpecies::Nitrite, 1.30),
                (AnionSpecies::Bromide, 2.80),
                (AnionSpecies::Nitrate, 3.28),
                (AnionSpecies::Phosphate, 7.30),
                (AnionSpecies::Sulfate, 8.70),
            ],
            cation_selectivity: Self::default_cation_coefficients(),
            donnan_exclusion: 0.95,
        }
    }

    /// Default selectivity coefficients for a typical CS12A cation column.
    pub fn default_cation() -> Self {
        Self {
            anion_selectivity: Self::default_anion_coefficients(),
            cation_selectivity: [
                (CationSpecies::Lithium, 0.30),
                (CationSpecies::Sodium, 1.00),
                (CationSpecies::Ammonium, 1.30),
                (CationSpecies::Potassium, 1.75),
                (CationSpecies::Magnesium, 2.50),
                (CationSpecies::Calcium, 3.10),
            ],
            donnan_exclusion: 0.93,
        }
    }

    fn default_anion_coefficients() -> [(AnionSpecies, f64); 7] {
        [
            (AnionSpecies::Fluoride, 0.10),
            (AnionSpecies::Chloride, 1.00),
            (AnionSpecies::Nitrite, 1.30),
            (AnionSpecies::Bromide, 2.80),
            (AnionSpecies::Nitrate, 3.28),
            (AnionSpecies::Phosphate, 7.30),
            (AnionSpecies::Sulfate, 8.70),
        ]
    }

    fn default_cation_coefficients() -> [(CationSpecies, f64); 6] {
        [
            (CationSpecies::Lithium, 0.30),
            (CationSpecies::Sodium, 1.00),
            (CationSpecies::Ammonium, 1.30),
            (CationSpecies::Potassium, 1.75),
            (CationSpecies::Magnesium, 2.50),
            (CationSpecies::Calcium, 3.10),
        ]
    }

    /// Get selectivity coefficient for an anion species.
    pub fn anion_coefficient(&self, species: AnionSpecies) -> Option<f64> {
        self.anion_selectivity
            .iter()
            .find(|(s, _)| *s == species)
            .map(|(_, v)| *v)
    }

    /// Get selectivity coefficient for a cation species.
    pub fn cation_coefficient(&self, species: CationSpecies) -> Option<f64> {
        self.cation_selectivity
            .iter()
            .find(|(s, _)| *s == species)
            .map(|(_, v)| *v)
    }

    /// Predict retention factor from selectivity coefficient.
    /// k' ~ alpha * column_capacity / [eluent]
    pub fn predicted_retention_factor(
        &self,
        selectivity: f64,
        column_capacity_meq: f64,
        eluent_concentration_mm: f64,
    ) -> f64 {
        if eluent_concentration_mm <= 0.0 {
            return 0.0;
        }
        selectivity * column_capacity_meq / eluent_concentration_mm
    }

    /// Ion exchange distribution ratio Kd from selectivity.
    /// Kd = alpha * [eluent_ion_on_resin] / [eluent_ion_in_solution]
    pub fn distribution_ratio(&self, selectivity: f64, resin_capacity: f64, solution_conc: f64) -> f64 {
        if solution_conc <= 0.0 {
            return 0.0;
        }
        selectivity * resin_capacity / solution_conc
    }

    /// Donnan exclusion: fraction of co-ions excluded from the resin phase.
    pub fn effective_exclusion(&self, charge_ratio: f64) -> f64 {
        // Higher charge ratio = better exclusion
        (self.donnan_exclusion * charge_ratio).min(1.0)
    }

    /// Predict elution order from selectivity coefficients (ascending).
    pub fn anion_elution_order(&self) -> Vec<AnionSpecies> {
        let mut sorted = self.anion_selectivity;
        sorted.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        sorted.iter().map(|(s, _)| *s).collect()
    }

    /// Predict cation elution order (ascending).
    pub fn cation_elution_order(&self) -> Vec<CationSpecies> {
        let mut sorted = self.cation_selectivity;
        sorted.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        sorted.iter().map(|(s, _)| *s).collect()
    }
}

// ---------------------------------------------------------------------------
// 3. ChromatogramPeakFinder
// ---------------------------------------------------------------------------

/// A detected chromatographic peak.
#[derive(Debug, Clone)]
pub struct ChromPeak {
    /// Retention time (minutes).
    pub retention_time: f64,
    /// Peak height (signal units).
    pub height: f64,
    /// Peak area (signal * time units).
    pub area: f64,
    /// Peak width at half-height (minutes).
    pub width_half_height: f64,
    /// Peak width at base (minutes), estimated as 4*sigma.
    pub width_base: f64,
    /// Gaussian fit parameters: (amplitude, mu, sigma).
    pub gaussian_fit: (f64, f64, f64),
    /// Theoretical plates for this peak.
    pub plates: f64,
    /// Asymmetry factor (1.0 = perfectly symmetric).
    pub asymmetry: f64,
    /// Start index in the chromatogram.
    pub start_idx: usize,
    /// End index in the chromatogram.
    pub end_idx: usize,
    /// Index of the peak apex.
    pub apex_idx: usize,
}

/// Chromatogram peak detection and fitting.
#[derive(Debug, Clone)]
pub struct ChromatogramPeakFinder {
    /// Minimum peak height as fraction of maximum signal.
    pub threshold_fraction: f64,
    /// Minimum number of points for a valid peak.
    pub min_peak_width_points: usize,
}

impl ChromatogramPeakFinder {
    /// Create a new peak finder.
    ///
    /// * `threshold_fraction` - minimum height fraction (0..1) to consider a peak.
    /// * `min_peak_width_points` - minimum number of contiguous points above threshold.
    pub fn new(threshold_fraction: f64, min_peak_width_points: usize) -> Self {
        Self {
            threshold_fraction: threshold_fraction.clamp(0.0, 1.0),
            min_peak_width_points: min_peak_width_points.max(1),
        }
    }

    /// Find peaks in a chromatogram using first-derivative zero-crossings.
    pub fn find_peaks(&self, time: &[f64], signal: &[f64]) -> Vec<ChromPeak> {
        assert_eq!(time.len(), signal.len(), "time and signal must have same length");
        if signal.len() < 5 {
            return Vec::new();
        }

        let max_signal = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_signal <= 0.0 {
            return Vec::new();
        }
        let threshold = self.threshold_fraction * max_signal;

        // First derivative (central difference)
        let n = signal.len();
        let mut deriv = vec![0.0f64; n];
        for i in 1..n - 1 {
            deriv[i] = (signal[i + 1] - signal[i - 1]) / 2.0;
        }
        deriv[0] = signal[1] - signal[0];
        deriv[n - 1] = signal[n - 1] - signal[n - 2];

        // Find zero-crossings of the derivative (positive -> negative = peak)
        let mut raw_peaks = Vec::new();
        for i in 1..n - 1 {
            if deriv[i - 1] > 0.0 && deriv[i + 1] < 0.0 && signal[i] >= threshold {
                raw_peaks.push(i);
            } else if deriv[i - 1] > 0.0 && deriv[i].abs() < 1e-15 && i + 2 < n && deriv[i + 1] < 0.0 && signal[i] >= threshold {
                raw_peaks.push(i);
            }
        }
        // Deduplicate: merge consecutive indices, keep highest signal
        let mut peak_indices = Vec::new();
        let mut i_raw = 0;
        while i_raw < raw_peaks.len() {
            let mut best = raw_peaks[i_raw];
            let mut j = i_raw + 1;
            while j < raw_peaks.len() && raw_peaks[j] - raw_peaks[j - 1] <= 2 {
                if signal[raw_peaks[j]] > signal[best] {
                    best = raw_peaks[j];
                }
                j += 1;
            }
            peak_indices.push(best);
            i_raw = j;
        }

        let mut peaks = Vec::new();
        let dt = if time.len() >= 2 {
            (time[time.len() - 1] - time[0]) / (time.len() - 1) as f64
        } else {
            1.0
        };

        for &apex in &peak_indices {
            let height = signal[apex];
            let half_height = height / 2.0;

            // Find half-height boundaries
            let mut left = apex;
            while left > 0 && signal[left] > half_height {
                left -= 1;
            }
            let mut right = apex;
            while right < n - 1 && signal[right] > half_height {
                right += 1;
            }

            // Interpolate for more precise half-height positions
            let left_t = if left < apex && (signal[left + 1] - signal[left]).abs() > 1e-30 {
                let frac = (half_height - signal[left]) / (signal[left + 1] - signal[left]);
                time[left] + frac * (time[left + 1] - time[left])
            } else {
                time[left]
            };
            let right_t = if right > apex && (signal[right - 1] - signal[right]).abs() > 1e-30 {
                let frac = (half_height - signal[right]) / (signal[right - 1] - signal[right]);
                time[right] + frac * (time[right - 1] - time[right])
            } else {
                time[right]
            };

            let w_half = (right_t - left_t).abs();
            if (right - left) < self.min_peak_width_points {
                continue;
            }

            // Sigma from FWHM
            let sigma = w_half / FWHM_FACTOR;

            // Find peak base boundaries (where signal drops to ~baseline)
            let baseline = threshold * 0.1;
            let mut start = apex;
            while start > 0 && signal[start] > baseline {
                start -= 1;
            }
            let mut end = apex;
            while end < n - 1 && signal[end] > baseline {
                end += 1;
            }

            // Area by trapezoidal integration
            let area = trapz(&signal[start..=end], dt);

            // Width at base ~ 4 sigma
            let w_base = 4.0 * sigma;

            // Plates
            let plates = plates_from_peak(time[apex], w_half);

            // Asymmetry: measure widths at 10% peak height
            let ten_pct = height * 0.10;
            let mut asym_left = apex;
            while asym_left > 0 && signal[asym_left] > ten_pct {
                asym_left -= 1;
            }
            let mut asym_right = apex;
            while asym_right < n - 1 && signal[asym_right] > ten_pct {
                asym_right += 1;
            }
            let a_front = time[apex] - time[asym_left];
            let b_back = time[asym_right] - time[apex];
            let asym = if a_front > 1e-15 { b_back / a_front } else { 1.0 };

            peaks.push(ChromPeak {
                retention_time: time[apex],
                height,
                area,
                width_half_height: w_half,
                width_base: w_base,
                gaussian_fit: (height, time[apex], sigma),
                plates,
                asymmetry: asym,
                start_idx: start,
                end_idx: end,
                apex_idx: apex,
            });
        }

        peaks
    }

    /// Generate a Gaussian fit for a detected peak.
    pub fn gaussian_fit_signal(
        &self,
        time: &[f64],
        peak: &ChromPeak,
    ) -> Vec<f64> {
        let (amp, mu, sigma) = peak.gaussian_fit;
        time.iter().map(|&t| gaussian(t, amp, mu, sigma)).collect()
    }

    /// Second derivative peak detection (inflection points).
    pub fn find_peaks_second_derivative(&self, time: &[f64], signal: &[f64]) -> Vec<usize> {
        if signal.len() < 5 {
            return Vec::new();
        }
        let n = signal.len();
        let max_signal = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let threshold = self.threshold_fraction * max_signal;

        // Second derivative (central difference)
        let mut d2 = vec![0.0f64; n];
        for i in 1..n - 1 {
            d2[i] = signal[i + 1] - 2.0 * signal[i] + signal[i - 1];
        }

        // Peaks correspond to negative second derivative minima
        let mut peaks = Vec::new();
        for i in 2..n - 2 {
            if d2[i] < d2[i - 1] && d2[i] < d2[i + 1] && d2[i] < 0.0 && signal[i] >= threshold {
                peaks.push(i);
            }
        }
        peaks
    }
}

// ---------------------------------------------------------------------------
// 4. CalibrationProcessor
// ---------------------------------------------------------------------------

/// Calibration mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationMode {
    /// Linear: y = a + b*x
    Linear,
    /// Quadratic: y = a + b*x + c*x^2
    Quadratic,
    /// Single-point: response factor = area / concentration
    SinglePoint,
}

/// Calibration result.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    pub mode: CalibrationMode,
    /// Coefficients: [intercept, slope] for linear; [a, b, c] for quadratic.
    pub coefficients: Vec<f64>,
    /// R-squared (coefficient of determination).
    pub r_squared: f64,
    /// Response factor (area / concentration) for single-point.
    pub response_factor: f64,
    /// Instrument Detection Limit (IDL) in concentration units.
    pub idl: f64,
    /// Method Detection Limit (MDL) in concentration units.
    pub mdl: f64,
}

/// Multi-level calibration processor.
#[derive(Debug, Clone)]
pub struct CalibrationProcessor {
    /// Calibration concentrations.
    pub concentrations: Vec<f64>,
    /// Corresponding peak areas (or heights).
    pub responses: Vec<f64>,
    /// Calibration mode.
    pub mode: CalibrationMode,
}

impl CalibrationProcessor {
    /// Create a new calibration processor.
    pub fn new(mode: CalibrationMode) -> Self {
        Self {
            concentrations: Vec::new(),
            responses: Vec::new(),
            mode,
        }
    }

    /// Add a calibration point.
    pub fn add_point(&mut self, concentration: f64, response: f64) {
        self.concentrations.push(concentration);
        self.responses.push(response);
    }

    /// Add multiple calibration points.
    pub fn add_points(&mut self, concentrations: &[f64], responses: &[f64]) {
        assert_eq!(concentrations.len(), responses.len());
        self.concentrations.extend_from_slice(concentrations);
        self.responses.extend_from_slice(responses);
    }

    /// Perform calibration and return result.
    pub fn calibrate(&self) -> CalibrationResult {
        match self.mode {
            CalibrationMode::Linear => self.calibrate_linear(),
            CalibrationMode::Quadratic => self.calibrate_quadratic(),
            CalibrationMode::SinglePoint => self.calibrate_single_point(),
        }
    }

    fn calibrate_linear(&self) -> CalibrationResult {
        let (a, b, r2) = linear_regression(&self.concentrations, &self.responses);
        let rf = if !self.concentrations.is_empty() {
            let mean_r = mean(&self.responses);
            let mean_c = mean(&self.concentrations);
            if mean_c.abs() > 1e-30 { mean_r / mean_c } else { 0.0 }
        } else {
            0.0
        };

        // IDL: 3 * std_dev(residuals) / slope
        // MDL: 3.143 * std_dev(replicates) -- simplified: use residual std
        let residuals: Vec<f64> = self
            .concentrations
            .iter()
            .zip(&self.responses)
            .map(|(c, r)| r - (a + b * c))
            .collect();
        let res_std = std_dev(&residuals);
        let idl = if b.abs() > 1e-30 { 3.0 * res_std / b } else { 0.0 };
        let mdl = if b.abs() > 1e-30 { 3.143 * res_std / b } else { 0.0 };

        CalibrationResult {
            mode: CalibrationMode::Linear,
            coefficients: vec![a, b],
            r_squared: r2,
            response_factor: rf,
            idl: idl.abs(),
            mdl: mdl.abs(),
        }
    }

    fn calibrate_quadratic(&self) -> CalibrationResult {
        let (a, b, c, r2) = quadratic_regression(&self.concentrations, &self.responses);
        let rf = if !self.concentrations.is_empty() {
            let mean_r = mean(&self.responses);
            let mean_c = mean(&self.concentrations);
            if mean_c.abs() > 1e-30 { mean_r / mean_c } else { 0.0 }
        } else {
            0.0
        };

        let residuals: Vec<f64> = self
            .concentrations
            .iter()
            .zip(&self.responses)
            .map(|(ci, ri)| ri - (a + b * ci + c * ci * ci))
            .collect();
        let res_std = std_dev(&residuals);
        let slope_at_zero = b;
        let idl = if slope_at_zero.abs() > 1e-30 {
            3.0 * res_std / slope_at_zero
        } else {
            0.0
        };
        let mdl = if slope_at_zero.abs() > 1e-30 {
            3.143 * res_std / slope_at_zero
        } else {
            0.0
        };

        CalibrationResult {
            mode: CalibrationMode::Quadratic,
            coefficients: vec![a, b, c],
            r_squared: r2,
            response_factor: rf,
            idl: idl.abs(),
            mdl: mdl.abs(),
        }
    }

    fn calibrate_single_point(&self) -> CalibrationResult {
        let rf = if !self.concentrations.is_empty() && self.concentrations[0].abs() > 1e-30 {
            self.responses[0] / self.concentrations[0]
        } else {
            0.0
        };
        CalibrationResult {
            mode: CalibrationMode::SinglePoint,
            coefficients: vec![0.0, rf],
            r_squared: 1.0,
            response_factor: rf,
            idl: 0.0,
            mdl: 0.0,
        }
    }

    /// Predict concentration from a measured response.
    pub fn predict_concentration(&self, response: f64) -> f64 {
        let cal = self.calibrate();
        match cal.mode {
            CalibrationMode::Linear => {
                let a = cal.coefficients[0];
                let b = cal.coefficients[1];
                if b.abs() < 1e-30 { 0.0 } else { (response - a) / b }
            }
            CalibrationMode::Quadratic => {
                // Solve a + b*x + c*x^2 = response
                let a = cal.coefficients[0] - response;
                let b = cal.coefficients[1];
                let c = cal.coefficients[2];
                if c.abs() < 1e-30 {
                    if b.abs() < 1e-30 { 0.0 } else { -a / b }
                } else {
                    let disc = b * b - 4.0 * c * a;
                    if disc < 0.0 {
                        0.0
                    } else {
                        let x1 = (-b + disc.sqrt()) / (2.0 * c);
                        let x2 = (-b - disc.sqrt()) / (2.0 * c);
                        // Return the positive root
                        if x1 >= 0.0 { x1 } else { x2 }
                    }
                }
            }
            CalibrationMode::SinglePoint => {
                if cal.response_factor.abs() < 1e-30 {
                    0.0
                } else {
                    response / cal.response_factor
                }
            }
        }
    }

    /// Verify calibration: check that a known standard is within +/- tolerance_pct.
    pub fn verify(&self, known_conc: f64, measured_response: f64, tolerance_pct: f64) -> bool {
        let predicted = self.predict_concentration(measured_response);
        let pct_diff = if known_conc.abs() > 1e-30 {
            ((predicted - known_conc) / known_conc * 100.0).abs()
        } else {
            0.0
        };
        pct_diff <= tolerance_pct
    }
}

// ---------------------------------------------------------------------------
// 5. ColumnEfficiency
// ---------------------------------------------------------------------------

/// Column efficiency metrics.
#[derive(Debug, Clone)]
pub struct ColumnEfficiency {
    /// Column length in mm.
    pub column_length_mm: f64,
}

impl ColumnEfficiency {
    pub fn new(column_length_mm: f64) -> Self {
        Self { column_length_mm }
    }

    /// Theoretical plates from base width: N = 16 (tR/W)^2.
    pub fn plates_base(&self, tr: f64, w_base: f64) -> f64 {
        plates_from_base_width(tr, w_base)
    }

    /// Theoretical plates from half-height width: N = 5.545 (tR/W_0.5)^2.
    pub fn plates_half_height(&self, tr: f64, w_half: f64) -> f64 {
        plates_from_peak(tr, w_half)
    }

    /// HETP (Height Equivalent to a Theoretical Plate): H = L / N.
    pub fn hetp(&self, n_plates: f64) -> f64 {
        plate_height(self.column_length_mm, n_plates)
    }

    /// Van Deemter equation: H = A + B/u + C*u
    ///
    /// * `a` - eddy diffusion term (mm).
    /// * `b` - longitudinal diffusion coefficient.
    /// * `c` - mass transfer coefficient.
    /// * `u` - linear velocity (mm/s).
    pub fn van_deemter(a: f64, b: f64, c: f64, u: f64) -> f64 {
        a + b / u + c * u
    }

    /// Optimal linear velocity from Van Deemter: u_opt = sqrt(B/C).
    pub fn optimal_velocity(b: f64, c: f64) -> f64 {
        assert!(c > 0.0, "C term must be positive");
        (b / c).sqrt()
    }

    /// Minimum plate height at optimal velocity: H_min = A + 2*sqrt(B*C).
    pub fn min_plate_height(a: f64, b: f64, c: f64) -> f64 {
        a + 2.0 * (b * c).sqrt()
    }

    /// Resolution between two peaks.
    pub fn resolution(&self, tr1: f64, tr2: f64, w1: f64, w2: f64) -> f64 {
        resolution(tr1, tr2, w1, w2)
    }

    /// Fundamental resolution equation: Rs = (sqrt(N)/4) * (alpha-1)/alpha * (k'2/(1+k'2))
    pub fn resolution_fundamental(n: f64, alpha: f64, k2: f64) -> f64 {
        (n.sqrt() / 4.0) * ((alpha - 1.0) / alpha) * (k2 / (1.0 + k2))
    }

    /// Selectivity factor alpha = k'2 / k'1 (k'2 > k'1).
    pub fn selectivity(k1: f64, k2: f64) -> f64 {
        if k1.abs() < 1e-30 {
            return 0.0;
        }
        k2 / k1
    }
}

// ---------------------------------------------------------------------------
// 6. GradientElution
// ---------------------------------------------------------------------------

/// Gradient profile type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GradientProfileType {
    Isocratic,
    Linear,
    StepGradient,
    Convex,
    Concave,
}

/// A gradient step (start_time, end_time, start_fraction, end_fraction).
#[derive(Debug, Clone)]
pub struct GradientStep {
    pub start_time_min: f64,
    pub end_time_min: f64,
    pub start_fraction_b: f64,
    pub end_fraction_b: f64,
}

/// Gradient elution programming.
#[derive(Debug, Clone)]
pub struct GradientElution {
    pub profile: GradientProfileType,
    pub steps: Vec<GradientStep>,
    /// Gradient delay volume (mL) - system dwell volume.
    pub delay_volume_ml: f64,
    /// Flow rate (mL/min).
    pub flow_rate_ml_min: f64,
}

impl GradientElution {
    /// Create an isocratic (constant composition) method.
    pub fn isocratic(fraction_b: f64, flow_rate: f64) -> Self {
        Self {
            profile: GradientProfileType::Isocratic,
            steps: vec![GradientStep {
                start_time_min: 0.0,
                end_time_min: 30.0,
                start_fraction_b: fraction_b,
                end_fraction_b: fraction_b,
            }],
            delay_volume_ml: 0.0,
            flow_rate_ml_min: flow_rate,
        }
    }

    /// Create a linear gradient.
    pub fn linear(start_b: f64, end_b: f64, duration_min: f64, flow_rate: f64) -> Self {
        Self {
            profile: GradientProfileType::Linear,
            steps: vec![GradientStep {
                start_time_min: 0.0,
                end_time_min: duration_min,
                start_fraction_b: start_b,
                end_fraction_b: end_b,
            }],
            delay_volume_ml: 0.0,
            flow_rate_ml_min: flow_rate,
        }
    }

    /// Set gradient delay volume.
    pub fn with_delay_volume(mut self, delay_ml: f64) -> Self {
        self.delay_volume_ml = delay_ml;
        self
    }

    /// Add a gradient step.
    pub fn add_step(&mut self, step: GradientStep) {
        self.steps.push(step);
    }

    /// Gradient delay time (minutes) = delay_volume / flow_rate.
    pub fn delay_time_min(&self) -> f64 {
        if self.flow_rate_ml_min > 0.0 {
            self.delay_volume_ml / self.flow_rate_ml_min
        } else {
            0.0
        }
    }

    /// Eluent composition at time t (fraction B), with delay correction.
    pub fn composition_at(&self, t_min: f64) -> f64 {
        let effective_t = t_min - self.delay_time_min();
        if self.steps.is_empty() {
            return 0.0;
        }

        // Find the applicable step
        for step in &self.steps {
            if effective_t >= step.start_time_min && effective_t <= step.end_time_min {
                let duration = step.end_time_min - step.start_time_min;
                if duration <= 0.0 {
                    return step.start_fraction_b;
                }
                let frac = (effective_t - step.start_time_min) / duration;
                return step.start_fraction_b + frac * (step.end_fraction_b - step.start_fraction_b);
            }
        }

        // Before first step or after last step
        if effective_t < self.steps[0].start_time_min {
            self.steps[0].start_fraction_b
        } else {
            self.steps.last().unwrap().end_fraction_b
        }
    }

    /// Retention factor in gradient mode (approximate).
    /// k_gradient ~ t_g * F / (delta_C * Vm * S)
    /// where t_g = gradient time, F = flow rate, delta_C = concentration change,
    /// Vm = column void volume, S = slope of log(k) vs composition.
    pub fn gradient_retention_factor(
        &self,
        gradient_time_min: f64,
        void_volume_ml: f64,
        s_factor: f64,
    ) -> f64 {
        if self.steps.is_empty() || void_volume_ml <= 0.0 || s_factor <= 0.0 {
            return 0.0;
        }
        let delta_c = (self.steps[0].end_fraction_b - self.steps[0].start_fraction_b).abs();
        if delta_c < 1e-15 {
            return 0.0;
        }
        (gradient_time_min * self.flow_rate_ml_min) / (delta_c * void_volume_ml * s_factor)
    }

    /// Check if method is isocratic.
    pub fn is_isocratic(&self) -> bool {
        self.profile == GradientProfileType::Isocratic
    }
}

// ---------------------------------------------------------------------------
// 7. MatrixElimination
// ---------------------------------------------------------------------------

/// Matrix elimination strategies for complex sample matrices.
#[derive(Debug, Clone)]
pub struct MatrixElimination;

impl MatrixElimination {
    /// Heart-cutting: extract a time window from one chromatogram for re-injection.
    /// Returns the portion of the signal between start and end times.
    pub fn heart_cut(
        time: &[f64],
        signal: &[f64],
        start_min: f64,
        end_min: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut cut_time = Vec::new();
        let mut cut_signal = Vec::new();
        for (i, &t) in time.iter().enumerate() {
            if t >= start_min && t <= end_min {
                cut_time.push(t);
                cut_signal.push(signal[i]);
            }
        }
        (cut_time, cut_signal)
    }

    /// Dilution correction: multiply measured concentration by dilution factor.
    pub fn dilution_correction(measured_conc: f64, dilution_factor: f64) -> f64 {
        measured_conc * dilution_factor
    }

    /// Standard addition method.
    ///
    /// Given paired (added_concentration, measured_response), extrapolate to
    /// find the native concentration via x-intercept of the regression line.
    pub fn standard_addition(
        added_concentrations: &[f64],
        measured_responses: &[f64],
    ) -> f64 {
        let (a, b, _r2) = linear_regression(added_concentrations, measured_responses);
        // x-intercept = -a/b
        if b.abs() < 1e-30 {
            return 0.0;
        }
        -(a / b)
    }

    /// Inline dilution (Metrohm-style): calculate the volume of diluent needed.
    /// Returns required diluent volume in uL.
    pub fn inline_dilution_volume(
        sample_volume_ul: f64,
        target_dilution_factor: f64,
    ) -> f64 {
        sample_volume_ul * (target_dilution_factor - 1.0)
    }

    /// Matrix spike recovery: assess method accuracy in the sample matrix.
    /// recovery_pct = (spiked_result - unspiked_result) / spike_amount * 100
    pub fn spike_recovery(
        unspiked_result: f64,
        spiked_result: f64,
        spike_amount: f64,
    ) -> f64 {
        if spike_amount.abs() < 1e-30 {
            return 0.0;
        }
        ((spiked_result - unspiked_result) / spike_amount) * 100.0
    }

    /// Relative percent difference between duplicate analyses.
    pub fn rpd(result1: f64, result2: f64) -> f64 {
        let avg = (result1 + result2) / 2.0;
        if avg.abs() < 1e-30 {
            return 0.0;
        }
        ((result1 - result2).abs() / avg) * 100.0
    }
}

// ---------------------------------------------------------------------------
// 8. WaterAnalysis
// ---------------------------------------------------------------------------

/// EPA drinking water Maximum Contaminant Levels (MCLs) in mg/L.
#[derive(Debug, Clone)]
pub struct DrinkingWaterMcl {
    pub fluoride: f64,
    pub chloride: f64,
    pub nitrite: f64,
    pub bromide: f64,
    pub nitrate: f64,
    pub phosphate: f64,
    pub sulfate: f64,
}

impl Default for DrinkingWaterMcl {
    fn default() -> Self {
        Self {
            fluoride: 4.0,    // EPA MCL
            chloride: 250.0,  // EPA SMCL (secondary)
            nitrite: 1.0,     // EPA MCL as N
            bromide: 10.0,    // No federal MCL; guideline
            nitrate: 10.0,    // EPA MCL as N
            phosphate: 10.0,  // No federal MCL; guideline
            sulfate: 250.0,   // EPA SMCL
        }
    }
}

/// Water analysis per EPA 300.0 / 300.1.
#[derive(Debug, Clone)]
pub struct WaterAnalysis {
    pub mcl: DrinkingWaterMcl,
}

/// Result of an anion analysis.
#[derive(Debug, Clone)]
pub struct AnionResult {
    pub species: AnionSpecies,
    pub concentration_mg_l: f64,
    pub exceeds_mcl: bool,
    pub mcl_value: f64,
}

/// Ion balance result.
#[derive(Debug, Clone)]
pub struct IonBalance {
    /// Sum of cation equivalents (meq/L).
    pub cation_sum_meq: f64,
    /// Sum of anion equivalents (meq/L).
    pub anion_sum_meq: f64,
    /// Percent difference.
    pub percent_difference: f64,
    /// Acceptable balance (< 10% difference for < 5 meq/L, < 5% for > 5 meq/L).
    pub acceptable: bool,
}

impl WaterAnalysis {
    pub fn new() -> Self {
        Self {
            mcl: DrinkingWaterMcl::default(),
        }
    }

    pub fn with_mcl(mcl: DrinkingWaterMcl) -> Self {
        Self { mcl }
    }

    /// Check a single anion against EPA MCL.
    pub fn check_anion(&self, species: AnionSpecies, concentration: f64) -> AnionResult {
        let mcl_val = match species {
            AnionSpecies::Fluoride => self.mcl.fluoride,
            AnionSpecies::Chloride => self.mcl.chloride,
            AnionSpecies::Nitrite => self.mcl.nitrite,
            AnionSpecies::Bromide => self.mcl.bromide,
            AnionSpecies::Nitrate => self.mcl.nitrate,
            AnionSpecies::Phosphate => self.mcl.phosphate,
            AnionSpecies::Sulfate => self.mcl.sulfate,
        };
        AnionResult {
            species,
            concentration_mg_l: concentration,
            exceeds_mcl: concentration > mcl_val,
            mcl_value: mcl_val,
        }
    }

    /// Check all common anions.
    pub fn check_all_anions(&self, concentrations: &[(AnionSpecies, f64)]) -> Vec<AnionResult> {
        concentrations
            .iter()
            .map(|(sp, conc)| self.check_anion(*sp, *conc))
            .collect()
    }

    /// Ion balance check (cation/anion sum ratio).
    ///
    /// Equivalent weights used for conversion:
    /// - Anions: F- 19, Cl- 35.45, NO2- 46.0, Br- 79.9, NO3- 62.0, PO4^3- 31.66, SO4^2- 48.03
    /// - Cations: Li+ 6.94, Na+ 23.0, NH4+ 18.04, K+ 39.1, Mg2+ 12.15, Ca2+ 20.04
    pub fn ion_balance(
        &self,
        anion_concs: &[(AnionSpecies, f64)],
        cation_concs: &[(CationSpecies, f64)],
    ) -> IonBalance {
        let anion_eq_weights = |s: AnionSpecies| -> f64 {
            match s {
                AnionSpecies::Fluoride => 19.0,
                AnionSpecies::Chloride => 35.45,
                AnionSpecies::Nitrite => 46.0,
                AnionSpecies::Bromide => 79.9,
                AnionSpecies::Nitrate => 62.0,
                AnionSpecies::Phosphate => 31.66,
                AnionSpecies::Sulfate => 48.03,
            }
        };

        let cation_eq_weights = |s: CationSpecies| -> f64 {
            match s {
                CationSpecies::Lithium => 6.94,
                CationSpecies::Sodium => 23.0,
                CationSpecies::Ammonium => 18.04,
                CationSpecies::Potassium => 39.1,
                CationSpecies::Magnesium => 12.15,
                CationSpecies::Calcium => 20.04,
            }
        };

        let anion_sum: f64 = anion_concs
            .iter()
            .map(|(s, c)| c / anion_eq_weights(*s))
            .sum();
        let cation_sum: f64 = cation_concs
            .iter()
            .map(|(s, c)| c / cation_eq_weights(*s))
            .sum();

        let total = anion_sum + cation_sum;
        let pct_diff = if total > 1e-15 {
            ((cation_sum - anion_sum).abs() / total) * 200.0
        } else {
            0.0
        };

        // Acceptance criteria: <10% for low TDS, <5% for high TDS
        let acceptable = if total < 5.0 {
            pct_diff < 10.0
        } else {
            pct_diff < 5.0
        };

        IonBalance {
            cation_sum_meq: cation_sum,
            anion_sum_meq: anion_sum,
            percent_difference: pct_diff,
            acceptable,
        }
    }

    /// EPA 300.0 method compliance: check that required parameters are met.
    pub fn epa_300_compliance(
        &self,
        initial_cal_r2: f64,
        check_std_recovery_pct: f64,
        spike_recovery_pct: f64,
        duplicate_rpd: f64,
    ) -> Vec<(String, bool)> {
        vec![
            ("Initial calibration R^2 >= 0.995".to_string(), initial_cal_r2 >= 0.995),
            (
                "Check standard recovery 90-110%".to_string(),
                check_std_recovery_pct >= 90.0 && check_std_recovery_pct <= 110.0,
            ),
            (
                "Matrix spike recovery 80-120%".to_string(),
                spike_recovery_pct >= 80.0 && spike_recovery_pct <= 120.0,
            ),
            ("Duplicate RPD <= 20%".to_string(), duplicate_rpd <= 20.0),
        ]
    }
}

// ---------------------------------------------------------------------------
// 9. SystemSuitability
// ---------------------------------------------------------------------------

/// System suitability test results.
#[derive(Debug, Clone)]
pub struct SuitabilityResult {
    /// Injection precision RSD (%).
    pub injection_rsd: f64,
    /// Injection precision passes (RSD < 1%).
    pub injection_pass: bool,
    /// Retention time RSD (%).
    pub rt_rsd: f64,
    /// Retention time passes (RSD < 1%).
    pub rt_pass: bool,
    /// Peak resolution (between critical pair).
    pub peak_resolution: f64,
    /// Resolution passes (Rs > 1.5 for baseline separation).
    pub resolution_pass: bool,
    /// System blank maximum signal.
    pub blank_max: f64,
    /// Blank passes (< threshold).
    pub blank_pass: bool,
    /// Carryover percent.
    pub carryover_pct: f64,
    /// Carryover passes (< 0.1%).
    pub carryover_pass: bool,
    /// Overall pass.
    pub overall_pass: bool,
}

/// System suitability testing per USP/EP guidelines.
#[derive(Debug, Clone)]
pub struct SystemSuitability {
    /// Maximum allowed injection RSD (%).
    pub max_injection_rsd: f64,
    /// Maximum allowed retention time RSD (%).
    pub max_rt_rsd: f64,
    /// Minimum required resolution.
    pub min_resolution: f64,
    /// Maximum blank signal (detector units).
    pub max_blank_signal: f64,
    /// Maximum carryover (%).
    pub max_carryover_pct: f64,
}

impl SystemSuitability {
    /// Default system suitability criteria.
    pub fn new() -> Self {
        Self {
            max_injection_rsd: 1.0,
            max_rt_rsd: 1.0,
            min_resolution: 1.5,
            max_blank_signal: 10.0,
            max_carryover_pct: 0.1,
        }
    }

    /// Custom criteria.
    pub fn with_criteria(
        max_injection_rsd: f64,
        max_rt_rsd: f64,
        min_resolution: f64,
        max_blank: f64,
        max_carryover: f64,
    ) -> Self {
        Self {
            max_injection_rsd,
            max_rt_rsd,
            min_resolution,
            max_blank_signal: max_blank,
            max_carryover_pct: max_carryover,
        }
    }

    /// Evaluate system suitability.
    ///
    /// * `injection_areas` - replicate injection areas.
    /// * `retention_times` - replicate retention times.
    /// * `resolution` - resolution between critical pair.
    /// * `blank_signal` - maximum signal in system blank.
    /// * `high_std_area` - area of high standard (for carryover calc).
    /// * `blank_after_high_area` - area of blank run after high standard.
    pub fn evaluate(
        &self,
        injection_areas: &[f64],
        retention_times: &[f64],
        peak_resolution: f64,
        blank_signal: f64,
        high_std_area: f64,
        blank_after_high_area: f64,
    ) -> SuitabilityResult {
        let inj_rsd = rsd_percent(injection_areas);
        let rt_rsd = rsd_percent(retention_times);
        let carryover = if high_std_area.abs() > 1e-30 {
            (blank_after_high_area / high_std_area) * 100.0
        } else {
            0.0
        };

        let inj_pass = inj_rsd < self.max_injection_rsd;
        let rt_pass = rt_rsd < self.max_rt_rsd;
        let res_pass = peak_resolution >= self.min_resolution;
        let blank_pass = blank_signal < self.max_blank_signal;
        let carry_pass = carryover < self.max_carryover_pct;

        let overall = inj_pass && rt_pass && res_pass && blank_pass && carry_pass;

        SuitabilityResult {
            injection_rsd: inj_rsd,
            injection_pass: inj_pass,
            rt_rsd,
            rt_pass,
            peak_resolution,
            resolution_pass: res_pass,
            blank_max: blank_signal,
            blank_pass,
            carryover_pct: carryover,
            carryover_pass: carry_pass,
            overall_pass: overall,
        }
    }

    /// Quick check: injection precision only (5+ replicates recommended).
    pub fn check_injection_precision(&self, areas: &[f64]) -> (f64, bool) {
        let rsd = rsd_percent(areas);
        (rsd, rsd < self.max_injection_rsd)
    }

    /// Quick check: retention time stability.
    pub fn check_rt_stability(&self, rts: &[f64]) -> (f64, bool) {
        let rsd = rsd_percent(rts);
        (rsd, rsd < self.max_rt_rsd)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_retention_factor_basic() {
        let k = retention_factor(5.0, 1.5);
        assert!((k - 2.3333).abs() < 0.01);
    }

    #[test]
    fn test_retention_factor_zero() {
        let k = retention_factor(1.5, 1.5);
        assert!((k - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_retention_factor_large() {
        let k = retention_factor(20.0, 2.0);
        assert!((k - 9.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic]
    fn test_retention_factor_zero_dead_time() {
        retention_factor(5.0, 0.0);
    }

    #[test]
    fn test_resolution_basic() {
        let rs = resolution(5.0, 7.0, 0.5, 0.6);
        assert!((rs - 3.6363).abs() < 0.01);
    }

    #[test]
    fn test_resolution_identical_peaks() {
        let rs = resolution(5.0, 5.0, 0.5, 0.5);
        assert!((rs - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_plates_from_peak_basic() {
        let n = plates_from_peak(10.0, 0.5);
        // N = 5.545 * (10/0.5)^2 = 5.545 * 400 = 2218
        assert!((n - 2218.0).abs() < 1.0);
    }

    #[test]
    fn test_plates_from_base_width() {
        let n = plates_from_base_width(10.0, 1.0);
        // N = 16 * (10/1)^2 = 1600
        assert!((n - 1600.0).abs() < 1e-10);
    }

    #[test]
    fn test_plate_height() {
        let h = plate_height(250.0, 10000.0);
        assert!((h - 0.025).abs() < 1e-10);
    }

    #[test]
    fn test_asymmetry_factor_symmetric() {
        let as_f = asymmetry_factor(1.0, 1.0);
        assert!((as_f - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_asymmetry_factor_tailing() {
        let as_f = asymmetry_factor(0.5, 1.0);
        assert!((as_f - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_trapz_constant() {
        let y = vec![3.0; 100];
        let area = trapz(&y, 1.0);
        assert!((area - 297.0).abs() < 1e-10);
    }

    #[test]
    fn test_trapz_linear() {
        // y = x from 0 to 10 in 11 steps
        let y: Vec<f64> = (0..=10).map(|i| i as f64).collect();
        let area = trapz(&y, 1.0);
        assert!((area - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_peak() {
        let v = gaussian(5.0, 100.0, 5.0, 1.0);
        assert!((v - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_off_center() {
        let v = gaussian(6.0, 100.0, 5.0, 1.0);
        let expected = 100.0 * (-0.5f64).exp();
        assert!((v - expected).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (a, b, r2) = linear_regression(&x, &y);
        assert!((a - 0.0).abs() < 1e-10);
        assert!((b - 2.0).abs() < 1e-10);
        assert!((r2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![3.0, 5.0, 7.0, 9.0, 11.0];
        let (a, b, r2) = linear_regression(&x, &y);
        assert!((a - 3.0).abs() < 1e-10);
        assert!((b - 2.0).abs() < 1e-10);
        assert!((r2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_quadratic_regression_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|xi| 1.0 + 2.0 * xi + 0.5 * xi * xi).collect();
        let (a, b, c, r2) = quadratic_regression(&x, &y);
        assert!((a - 1.0).abs() < 1e-6);
        assert!((b - 2.0).abs() < 1e-6);
        assert!((c - 0.5).abs() < 1e-6);
        assert!((r2 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_std_dev_basic() {
        let data = vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let sd = std_dev(&data);
        // Sample std dev with Bessel's correction: sqrt(32/7) ~ 2.138
        assert!((sd - 2.138).abs() < 0.01);
    }

    #[test]
    fn test_mean_basic() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((mean(&data) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_rsd_percent() {
        let data = vec![100.0, 101.0, 99.0, 100.5, 99.5];
        let rsd = rsd_percent(&data);
        assert!(rsd < 2.0); // should be very small
        assert!(rsd > 0.0);
    }

    // -----------------------------------------------------------------------
    // SuppressedConductivity tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_suppressor_creation() {
        let sup = SuppressedConductivity::new(0.5, 100.0, 0.99);
        assert!((sup.background_conductivity - 0.5).abs() < 1e-10);
        assert!((sup.suppressor_capacity_meq - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_suppressor_default_anion() {
        let sup = SuppressedConductivity::default_anion();
        assert!((sup.efficiency - 0.99).abs() < 1e-10);
    }

    #[test]
    fn test_suppressor_default_cation() {
        let sup = SuppressedConductivity::default_cation();
        assert!((sup.efficiency - 0.98).abs() < 1e-10);
    }

    #[test]
    fn test_suppressor_suppress() {
        let sup = SuppressedConductivity::new(1.0, 100.0, 1.0);
        let raw = vec![5.0, 3.0, 1.0, 0.5, 10.0];
        let suppressed = sup.suppress(&raw);
        assert!((suppressed[0] - 4.0).abs() < 1e-10);
        assert!((suppressed[2] - 0.0).abs() < 1e-10);
        assert!((suppressed[3] - 0.0).abs() < 1e-10); // clamped to 0
    }

    #[test]
    fn test_suppressor_consume() {
        let mut sup = SuppressedConductivity::new(0.5, 100.0, 0.99);
        let remaining = sup.consume(10.0, 5.0);
        // consumed 50 meq, remaining = 50/100 = 0.5
        assert!((remaining - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_suppressor_exhaustion() {
        let mut sup = SuppressedConductivity::new(0.5, 100.0, 0.99);
        sup.consume(20.0, 5.0); // consume 100 meq = capacity
        assert!(sup.is_exhausted());
    }

    #[test]
    fn test_suppressor_regenerate() {
        let mut sup = SuppressedConductivity::new(0.5, 100.0, 0.99);
        sup.consume(20.0, 5.0);
        sup.regenerate();
        assert!((sup.remaining_fraction() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_suppressor_estimated_conductivity() {
        let sup = SuppressedConductivity::default_anion();
        let cond = sup.estimated_conductivity(76.3, 0.1);
        assert!((cond - 7.63).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // IonSeparation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_anion_selectivity_default() {
        let sep = IonSeparation::default_anion();
        let cl = sep.anion_coefficient(AnionSpecies::Chloride).unwrap();
        assert!((cl - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_anion_selectivity_fluoride() {
        let sep = IonSeparation::default_anion();
        let f = sep.anion_coefficient(AnionSpecies::Fluoride).unwrap();
        assert!(f < 1.0); // fluoride elutes before chloride
    }

    #[test]
    fn test_anion_selectivity_sulfate() {
        let sep = IonSeparation::default_anion();
        let so4 = sep.anion_coefficient(AnionSpecies::Sulfate).unwrap();
        assert!(so4 > 1.0); // sulfate is strongly retained
    }

    #[test]
    fn test_cation_selectivity_default() {
        let sep = IonSeparation::default_cation();
        let na = sep.cation_coefficient(CationSpecies::Sodium).unwrap();
        assert!((na - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cation_selectivity_calcium() {
        let sep = IonSeparation::default_cation();
        let ca = sep.cation_coefficient(CationSpecies::Calcium).unwrap();
        assert!(ca > 1.0);
    }

    #[test]
    fn test_predicted_retention_factor() {
        let sep = IonSeparation::default_anion();
        let k = sep.predicted_retention_factor(8.7, 0.1, 4.5);
        assert!(k > 0.0);
    }

    #[test]
    fn test_distribution_ratio() {
        let sep = IonSeparation::default_anion();
        let kd = sep.distribution_ratio(1.0, 10.0, 5.0);
        assert!((kd - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_donnan_exclusion() {
        let sep = IonSeparation::default_anion();
        let excl = sep.effective_exclusion(1.0);
        assert!((excl - 0.95).abs() < 1e-10);
    }

    #[test]
    fn test_donnan_exclusion_divalent() {
        let sep = IonSeparation::default_anion();
        let excl = sep.effective_exclusion(2.0);
        assert!((excl - 1.0).abs() < 1e-10); // clamped to 1.0
    }

    #[test]
    fn test_anion_elution_order() {
        let sep = IonSeparation::default_anion();
        let order = sep.anion_elution_order();
        assert_eq!(order[0], AnionSpecies::Fluoride);
        assert_eq!(order[order.len() - 1], AnionSpecies::Sulfate);
    }

    #[test]
    fn test_cation_elution_order() {
        let sep = IonSeparation::default_cation();
        let order = sep.cation_elution_order();
        assert_eq!(order[0], CationSpecies::Lithium);
        assert_eq!(order[order.len() - 1], CationSpecies::Calcium);
    }

    // -----------------------------------------------------------------------
    // ChromatogramPeakFinder tests
    // -----------------------------------------------------------------------

    fn make_chromatogram(peaks: &[(f64, f64, f64)], n: usize, t_max: f64) -> (Vec<f64>, Vec<f64>) {
        let dt = t_max / n as f64;
        let time: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let signal: Vec<f64> = time
            .iter()
            .map(|&t| {
                peaks
                    .iter()
                    .map(|(amp, mu, sigma)| gaussian(t, *amp, *mu, *sigma))
                    .sum()
            })
            .collect();
        (time, signal)
    }

    #[test]
    fn test_find_single_peak() {
        let (time, signal) = make_chromatogram(&[(100.0, 5.0, 0.2)], 1000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert_eq!(peaks.len(), 1);
        assert!((peaks[0].retention_time - 5.0).abs() < 0.05);
    }

    #[test]
    fn test_find_two_peaks() {
        let (time, signal) = make_chromatogram(
            &[(100.0, 5.0, 0.5), (80.0, 8.0, 0.5)],
            5000,
            15.0,
        );
        let finder = ChromatogramPeakFinder::new(0.05, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert_eq!(peaks.len(), 2);
        let rts: Vec<f64> = peaks.iter().map(|p| p.retention_time).collect();
        assert!(rts.iter().any(|&t| (t - 5.0).abs() < 0.5));
        assert!(rts.iter().any(|&t| (t - 8.0).abs() < 0.5));
    }

    #[test]
    fn test_peak_height() {
        let (time, signal) = make_chromatogram(&[(200.0, 5.0, 0.15)], 1000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(!peaks.is_empty());
        assert!((peaks[0].height - 200.0).abs() < 5.0);
    }

    #[test]
    fn test_peak_area_gaussian() {
        let amp = 100.0;
        let sigma = 0.3;
        let (time, signal) = make_chromatogram(&[(amp, 5.0, sigma)], 2000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.001, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(!peaks.is_empty());
        // Theoretical area of Gaussian = amp * sigma * sqrt(2*pi)
        let theoretical_area = amp * sigma * SQRT_2PI;
        // Allow 10% tolerance for numerical integration
        assert!((peaks[0].area - theoretical_area).abs() / theoretical_area < 0.10);
    }

    #[test]
    fn test_peak_width_half_height() {
        let sigma = 0.2;
        let (time, signal) = make_chromatogram(&[(100.0, 5.0, sigma)], 2000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(!peaks.is_empty());
        let expected_fwhm = FWHM_FACTOR * sigma;
        assert!((peaks[0].width_half_height - expected_fwhm).abs() < 0.05);
    }

    #[test]
    fn test_peak_plates() {
        let (time, signal) = make_chromatogram(&[(100.0, 10.0, 0.2)], 4000, 20.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(!peaks.is_empty());
        assert!(peaks[0].plates > 1000.0); // should be large for narrow peak at tR=10
    }

    #[test]
    fn test_gaussian_fit_signal() {
        let (time, signal) = make_chromatogram(&[(100.0, 5.0, 0.2)], 1000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(!peaks.is_empty());
        let fitted = finder.gaussian_fit_signal(&time, &peaks[0]);
        assert_eq!(fitted.len(), time.len());
        // Peak of fitted signal should be near original
        let max_fitted = fitted.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((max_fitted - 100.0).abs() < 5.0);
    }

    #[test]
    fn test_second_derivative_detection() {
        let (time, signal) = make_chromatogram(&[(100.0, 5.0, 0.2)], 1000, 10.0);
        let finder = ChromatogramPeakFinder::new(0.01, 3);
        let peaks_d2 = finder.find_peaks_second_derivative(&time, &signal);
        assert!(!peaks_d2.is_empty());
        // The detected index should be near the peak location
        let peak_time = time[peaks_d2[0]];
        assert!((peak_time - 5.0).abs() < 0.2);
    }

    #[test]
    fn test_no_peaks_in_noise() {
        let signal = vec![0.001; 100];
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let finder = ChromatogramPeakFinder::new(0.5, 5);
        let peaks = finder.find_peaks(&time, &signal);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_threshold_filtering() {
        // One tall peak and one short peak
        let (time, signal) = make_chromatogram(
            &[(100.0, 5.0, 0.5), (5.0, 8.0, 0.5)],
            5000,
            15.0,
        );
        let finder = ChromatogramPeakFinder::new(0.1, 3); // 10% threshold
        let peaks = finder.find_peaks(&time, &signal);
        // Small peak (5.0 amplitude) is 5% of 100.0, should be filtered out at 10% threshold
        // Only the main peak at t=5.0 with amplitude 100.0 should be found
        assert_eq!(peaks.len(), 1);
        assert!((peaks[0].retention_time - 5.0).abs() < 0.5);
    }

    // -----------------------------------------------------------------------
    // CalibrationProcessor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_calibration() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Linear);
        cal.add_points(&[1.0, 2.0, 5.0, 10.0, 20.0], &[100.0, 200.0, 500.0, 1000.0, 2000.0]);
        let result = cal.calibrate();
        assert!((result.r_squared - 1.0).abs() < 1e-6);
        assert_eq!(result.coefficients.len(), 2);
    }

    #[test]
    fn test_linear_calibration_predict() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Linear);
        cal.add_points(&[0.0, 1.0, 2.0, 5.0, 10.0], &[0.0, 100.0, 200.0, 500.0, 1000.0]);
        let conc = cal.predict_concentration(300.0);
        assert!((conc - 3.0).abs() < 0.1);
    }

    #[test]
    fn test_quadratic_calibration() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Quadratic);
        let concs = vec![0.0, 1.0, 2.0, 5.0, 10.0, 20.0];
        let resps: Vec<f64> = concs.iter().map(|c| 10.0 * c + 0.5 * c * c).collect();
        cal.add_points(&concs, &resps);
        let result = cal.calibrate();
        assert!((result.r_squared - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_single_point_calibration() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::SinglePoint);
        cal.add_point(10.0, 5000.0);
        let result = cal.calibrate();
        assert!((result.response_factor - 500.0).abs() < 1e-10);
    }

    #[test]
    fn test_single_point_predict() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::SinglePoint);
        cal.add_point(10.0, 5000.0);
        let conc = cal.predict_concentration(2500.0);
        assert!((conc - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_calibration_verify_pass() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Linear);
        cal.add_points(&[0.0, 5.0, 10.0, 20.0], &[0.0, 500.0, 1000.0, 2000.0]);
        assert!(cal.verify(10.0, 1000.0, 10.0));
    }

    #[test]
    fn test_calibration_verify_fail() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Linear);
        cal.add_points(&[0.0, 5.0, 10.0, 20.0], &[0.0, 500.0, 1000.0, 2000.0]);
        // 50% off should fail at 10% tolerance
        assert!(!cal.verify(10.0, 500.0, 10.0));
    }

    #[test]
    fn test_calibration_idl_mdl() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Linear);
        cal.add_points(
            &[0.0, 0.5, 1.0, 2.0, 5.0, 10.0],
            &[5.0, 55.0, 95.0, 210.0, 505.0, 1010.0],
        );
        let result = cal.calibrate();
        // IDL and MDL should be finite positive
        assert!(result.idl >= 0.0);
        assert!(result.mdl >= 0.0);
        assert!(result.mdl >= result.idl);
    }

    #[test]
    fn test_quadratic_predict() {
        let mut cal = CalibrationProcessor::new(CalibrationMode::Quadratic);
        let concs = vec![0.0, 1.0, 2.0, 5.0, 10.0];
        let resps: Vec<f64> = concs.iter().map(|c| 10.0 * c + 0.5 * c * c).collect();
        cal.add_points(&concs, &resps);
        let predicted = cal.predict_concentration(55.0); // 10*5 + 0.5*25 = 62.5 nope, let's use real value
        // For c=5: resp = 50+12.5=62.5
        let p5 = cal.predict_concentration(62.5);
        assert!((p5 - 5.0).abs() < 0.5);
    }

    // -----------------------------------------------------------------------
    // ColumnEfficiency tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_column_plates_base() {
        let col = ColumnEfficiency::new(250.0);
        let n = col.plates_base(10.0, 1.0);
        assert!((n - 1600.0).abs() < 1e-10);
    }

    #[test]
    fn test_column_plates_half_height() {
        let col = ColumnEfficiency::new(250.0);
        let n = col.plates_half_height(10.0, 0.5);
        assert!((n - 2218.0).abs() < 1.0);
    }

    #[test]
    fn test_column_hetp() {
        let col = ColumnEfficiency::new(250.0);
        let h = col.hetp(10000.0);
        assert!((h - 0.025).abs() < 1e-10);
    }

    #[test]
    fn test_van_deemter() {
        let h = ColumnEfficiency::van_deemter(0.5, 5.0, 0.01, 10.0);
        // H = 0.5 + 5/10 + 0.01*10 = 0.5 + 0.5 + 0.1 = 1.1
        assert!((h - 1.1).abs() < 1e-10);
    }

    #[test]
    fn test_optimal_velocity() {
        let u_opt = ColumnEfficiency::optimal_velocity(5.0, 0.01);
        // sqrt(5/0.01) = sqrt(500) ~ 22.36
        assert!((u_opt - 22.36).abs() < 0.1);
    }

    #[test]
    fn test_min_plate_height() {
        let h_min = ColumnEfficiency::min_plate_height(0.5, 5.0, 0.01);
        // 0.5 + 2*sqrt(0.05) = 0.5 + 0.4472 = 0.9472
        assert!((h_min - 0.9472).abs() < 0.01);
    }

    #[test]
    fn test_column_resolution() {
        let col = ColumnEfficiency::new(250.0);
        let rs = col.resolution(5.0, 7.0, 0.5, 0.6);
        assert!((rs - 3.6363).abs() < 0.01);
    }

    #[test]
    fn test_selectivity() {
        let alpha = ColumnEfficiency::selectivity(2.0, 4.0);
        assert!((alpha - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_resolution_fundamental() {
        let rs = ColumnEfficiency::resolution_fundamental(10000.0, 1.05, 5.0);
        assert!(rs > 0.0);
    }

    // -----------------------------------------------------------------------
    // GradientElution tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_isocratic_method() {
        let ge = GradientElution::isocratic(0.5, 1.0);
        assert!(ge.is_isocratic());
        assert!((ge.composition_at(5.0) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_linear_gradient() {
        let ge = GradientElution::linear(0.1, 0.9, 20.0, 1.0);
        assert!(!ge.is_isocratic());
        // At midpoint (t=10), composition should be 0.5
        assert!((ge.composition_at(10.0) - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_gradient_endpoints() {
        let ge = GradientElution::linear(0.1, 0.9, 20.0, 1.0);
        assert!((ge.composition_at(0.0) - 0.1).abs() < 0.01);
        assert!((ge.composition_at(20.0) - 0.9).abs() < 0.01);
    }

    #[test]
    fn test_gradient_delay() {
        let ge = GradientElution::linear(0.1, 0.9, 20.0, 1.0)
            .with_delay_volume(2.0);
        assert!((ge.delay_time_min() - 2.0).abs() < 1e-10);
        // Before delay, composition should be start value
        assert!((ge.composition_at(1.0) - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_gradient_retention_factor() {
        let ge = GradientElution::linear(0.1, 0.9, 20.0, 1.0);
        let k = ge.gradient_retention_factor(20.0, 0.5, 5.0);
        assert!(k > 0.0);
    }

    #[test]
    fn test_gradient_add_step() {
        let mut ge = GradientElution::isocratic(0.5, 1.0);
        ge.add_step(GradientStep {
            start_time_min: 10.0,
            end_time_min: 20.0,
            start_fraction_b: 0.5,
            end_fraction_b: 1.0,
        });
        assert_eq!(ge.steps.len(), 2);
    }

    // -----------------------------------------------------------------------
    // MatrixElimination tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_heart_cut() {
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let signal: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let (ct, cs) = MatrixElimination::heart_cut(&time, &signal, 2.0, 5.0);
        assert!(!ct.is_empty());
        assert_eq!(ct.len(), cs.len());
        assert!(ct[0] >= 2.0);
        assert!(*ct.last().unwrap() <= 5.0);
    }

    #[test]
    fn test_dilution_correction() {
        let corrected = MatrixElimination::dilution_correction(5.0, 10.0);
        assert!((corrected - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_standard_addition() {
        // y = 100 + 50*x, x-intercept = -100/50 = -2.0 -> native conc = 2.0
        let added = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let measured = vec![100.0, 150.0, 200.0, 250.0, 300.0];
        let native = MatrixElimination::standard_addition(&added, &measured);
        assert!((native - (-2.0)).abs() < 0.01);
    }

    #[test]
    fn test_inline_dilution() {
        let diluent = MatrixElimination::inline_dilution_volume(100.0, 5.0);
        assert!((diluent - 400.0).abs() < 1e-10);
    }

    #[test]
    fn test_spike_recovery() {
        let recovery = MatrixElimination::spike_recovery(10.0, 19.5, 10.0);
        assert!((recovery - 95.0).abs() < 1e-10);
    }

    #[test]
    fn test_rpd() {
        let rpd = MatrixElimination::rpd(100.0, 105.0);
        // |100-105| / 102.5 * 100 = 4.878
        assert!((rpd - 4.878).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // WaterAnalysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_water_analysis_default_mcl() {
        let wa = WaterAnalysis::new();
        assert!((wa.mcl.fluoride - 4.0).abs() < 1e-10);
        assert!((wa.mcl.nitrate - 10.0).abs() < 1e-10);
        assert!((wa.mcl.sulfate - 250.0).abs() < 1e-10);
    }

    #[test]
    fn test_check_anion_pass() {
        let wa = WaterAnalysis::new();
        let result = wa.check_anion(AnionSpecies::Fluoride, 2.0);
        assert!(!result.exceeds_mcl);
    }

    #[test]
    fn test_check_anion_fail() {
        let wa = WaterAnalysis::new();
        let result = wa.check_anion(AnionSpecies::Fluoride, 5.0);
        assert!(result.exceeds_mcl);
    }

    #[test]
    fn test_check_all_anions() {
        let wa = WaterAnalysis::new();
        let results = wa.check_all_anions(&[
            (AnionSpecies::Fluoride, 1.0),
            (AnionSpecies::Nitrate, 15.0),
        ]);
        assert_eq!(results.len(), 2);
        assert!(!results[0].exceeds_mcl);
        assert!(results[1].exceeds_mcl);
    }

    #[test]
    fn test_ion_balance_acceptable() {
        let wa = WaterAnalysis::new();
        let balance = wa.ion_balance(
            &[
                (AnionSpecies::Chloride, 35.45),
                (AnionSpecies::Sulfate, 48.03),
            ],
            &[
                (CationSpecies::Sodium, 23.0),
                (CationSpecies::Calcium, 20.04),
            ],
        );
        // Each should contribute ~1 meq/L, so sums should be roughly equal
        assert!(balance.acceptable);
    }

    #[test]
    fn test_ion_balance_unacceptable() {
        let wa = WaterAnalysis::new();
        let balance = wa.ion_balance(
            &[(AnionSpecies::Chloride, 1000.0)], // ~28 meq/L
            &[(CationSpecies::Sodium, 10.0)],    // ~0.43 meq/L
        );
        assert!(!balance.acceptable);
    }

    #[test]
    fn test_epa_300_compliance_pass() {
        let wa = WaterAnalysis::new();
        let checks = wa.epa_300_compliance(0.999, 100.0, 95.0, 5.0);
        assert!(checks.iter().all(|(_, pass)| *pass));
    }

    #[test]
    fn test_epa_300_compliance_fail_r2() {
        let wa = WaterAnalysis::new();
        let checks = wa.epa_300_compliance(0.990, 100.0, 95.0, 5.0);
        assert!(!checks[0].1); // R^2 fails
    }

    #[test]
    fn test_epa_300_compliance_fail_recovery() {
        let wa = WaterAnalysis::new();
        let checks = wa.epa_300_compliance(0.999, 115.0, 95.0, 5.0);
        assert!(!checks[1].1); // check std recovery fails
    }

    // -----------------------------------------------------------------------
    // SystemSuitability tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_system_suitability_pass() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1002.0, 998.0, 1001.0, 999.0, 1003.0];
        let rts = vec![5.001, 5.002, 4.999, 5.000, 5.001, 5.003];
        let result = ss.evaluate(&areas, &rts, 2.5, 0.5, 10000.0, 0.5);
        assert!(result.injection_pass);
        assert!(result.rt_pass);
        assert!(result.resolution_pass);
        assert!(result.blank_pass);
        assert!(result.carryover_pass);
        assert!(result.overall_pass);
    }

    #[test]
    fn test_system_suitability_fail_injection() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1100.0, 900.0, 1050.0, 950.0]; // high variability
        let rts = vec![5.0, 5.0, 5.0, 5.0, 5.0];
        let result = ss.evaluate(&areas, &rts, 2.5, 0.5, 10000.0, 0.5);
        assert!(!result.injection_pass);
        assert!(!result.overall_pass);
    }

    #[test]
    fn test_system_suitability_fail_resolution() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1001.0, 999.0];
        let rts = vec![5.0, 5.0, 5.0];
        let result = ss.evaluate(&areas, &rts, 1.0, 0.5, 10000.0, 0.5);
        assert!(!result.resolution_pass);
    }

    #[test]
    fn test_system_suitability_fail_blank() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1001.0, 999.0];
        let rts = vec![5.0, 5.0, 5.0];
        let result = ss.evaluate(&areas, &rts, 2.5, 50.0, 10000.0, 0.5);
        assert!(!result.blank_pass);
    }

    #[test]
    fn test_system_suitability_fail_carryover() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1001.0, 999.0];
        let rts = vec![5.0, 5.0, 5.0];
        let result = ss.evaluate(&areas, &rts, 2.5, 0.5, 10000.0, 20.0);
        assert!(!result.carryover_pass);
    }

    #[test]
    fn test_injection_precision_check() {
        let ss = SystemSuitability::new();
        let areas = vec![1000.0, 1001.0, 999.0, 1000.5, 999.5];
        let (rsd, pass) = ss.check_injection_precision(&areas);
        assert!(pass);
        assert!(rsd < 1.0);
    }

    #[test]
    fn test_rt_stability_check() {
        let ss = SystemSuitability::new();
        let rts = vec![5.0, 5.001, 4.999, 5.002, 4.998];
        let (rsd, pass) = ss.check_rt_stability(&rts);
        assert!(pass);
        assert!(rsd < 1.0);
    }

    #[test]
    fn test_custom_suitability_criteria() {
        let ss = SystemSuitability::with_criteria(2.0, 2.0, 1.0, 20.0, 0.5);
        assert!((ss.max_injection_rsd - 2.0).abs() < 1e-10);
        assert!((ss.min_resolution - 1.0).abs() < 1e-10);
    }
}
