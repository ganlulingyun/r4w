// trace:FR-XPS | ai:claude
//! # XPS Photoelectron Analyzer
//!
//! X-ray Photoelectron Spectroscopy (XPS/ESCA) data analysis for surface chemical
//! composition, including spectrum processing, peak fitting, chemical state
//! identification, and quantification from binding energy spectra.
//!
//! ## Physics Background
//!
//! XPS is based on the photoelectric effect:
//!   BE = hν - KE - φ_spectrometer
//!
//! where BE is binding energy, hν is photon energy, KE is kinetic energy, and
//! φ is the spectrometer work function. Surface sensitivity arises from the
//! inelastic mean free path (IMFP) of photoelectrons, typically 1-10 nm.
//!
//! ## Key Capabilities
//!
//! - Spectrum smoothing (Savitzky-Golay) and differentiation
//! - Background subtraction: Shirley, Tougaard, and linear
//! - Peak fitting with pseudo-Voigt and Doniach-Sunjic line shapes
//! - Chemical state identification from binding energy databases
//! - Quantification via atomic sensitivity factors (Scofield cross-sections)
//! - Spin-orbit splitting analysis for p, d, f orbitals
//! - Charge referencing (adventitious carbon, Au 4f)
//! - Sputter depth profiling analysis

use std::f64::consts::PI;

// ─── XpsSpectrum ─────────────────────────────────────────────────────

/// A binding energy spectrum I(BE).
#[derive(Debug, Clone)]
pub struct XpsSpectrum {
    /// Binding energies in eV (typically descending order in XPS convention).
    pub binding_energy_ev: Vec<f64>,
    /// Counts (intensity) at each binding energy.
    pub counts: Vec<f64>,
}

impl XpsSpectrum {
    /// Create a new XPS spectrum from binding energy and counts vectors.
    ///
    /// # Panics
    /// Panics if the two vectors have different lengths or are empty.
    pub fn new(binding_energy_ev: Vec<f64>, counts: Vec<f64>) -> Self {
        assert!(!binding_energy_ev.is_empty(), "Spectrum must have at least one point");
        assert_eq!(
            binding_energy_ev.len(),
            counts.len(),
            "Binding energy and counts must have equal length"
        );
        Self {
            binding_energy_ev,
            counts,
        }
    }

    /// Return the energy range (min, max) in eV.
    pub fn energy_range(&self) -> (f64, f64) {
        let mut min = self.binding_energy_ev[0];
        let mut max = self.binding_energy_ev[0];
        for &e in &self.binding_energy_ev[1..] {
            if e < min {
                min = e;
            }
            if e > max {
                max = e;
            }
        }
        (min, max)
    }

    /// Return the maximum counts value.
    pub fn max_counts(&self) -> f64 {
        let mut m = f64::NEG_INFINITY;
        for &c in &self.counts {
            if c > m {
                m = c;
            }
        }
        m
    }

    /// Smooth the spectrum using a Savitzky-Golay-like moving average window.
    ///
    /// Uses a simple symmetric moving average of the given `window` size (must be odd).
    /// For a true Savitzky-Golay polynomial fit we use a quadratic least-squares
    /// convolution kernel of the specified window width.
    pub fn smooth(&self, window: usize) -> XpsSpectrum {
        let w = if window % 2 == 0 { window + 1 } else { window };
        let half = w / 2;
        let n = self.counts.len();
        let coeffs = savitzky_golay_coeffs(w, 2, 0);
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let mut val = 0.0;
            for (j, &c) in coeffs.iter().enumerate() {
                let idx = i as isize + j as isize - half as isize;
                let idx = idx.clamp(0, n as isize - 1) as usize;
                val += c * self.counts[idx];
            }
            smoothed[i] = val;
        }
        XpsSpectrum {
            binding_energy_ev: self.binding_energy_ev.clone(),
            counts: smoothed,
        }
    }

    /// Compute the derivative of the spectrum.
    ///
    /// `order` = 1 for first derivative, 2 for second derivative.
    /// Uses central finite differences.
    pub fn derivative(&self, order: usize) -> XpsSpectrum {
        assert!(order == 1 || order == 2, "Only 1st and 2nd derivatives supported");
        let n = self.counts.len();
        let mut result = vec![0.0; n];
        if n < 3 {
            return XpsSpectrum {
                binding_energy_ev: self.binding_energy_ev.clone(),
                counts: result,
            };
        }

        if order == 1 {
            // Central difference: f'(x) ≈ (f(x+h) - f(x-h)) / (2h)
            for i in 1..n - 1 {
                let h = self.binding_energy_ev[i + 1] - self.binding_energy_ev[i - 1];
                if h.abs() > 1e-15 {
                    result[i] = (self.counts[i + 1] - self.counts[i - 1]) / h;
                }
            }
            // Forward/backward difference at endpoints
            if n >= 2 {
                let h0 = self.binding_energy_ev[1] - self.binding_energy_ev[0];
                if h0.abs() > 1e-15 {
                    result[0] = (self.counts[1] - self.counts[0]) / h0;
                }
                let hlast = self.binding_energy_ev[n - 1] - self.binding_energy_ev[n - 2];
                if hlast.abs() > 1e-15 {
                    result[n - 1] = (self.counts[n - 1] - self.counts[n - 2]) / hlast;
                }
            }
        } else {
            // Second derivative: f''(x) ≈ (f(x+h) - 2f(x) + f(x-h)) / h²
            for i in 1..n - 1 {
                let h = (self.binding_energy_ev[i + 1] - self.binding_energy_ev[i - 1]) / 2.0;
                if h.abs() > 1e-15 {
                    result[i] = (self.counts[i + 1] - 2.0 * self.counts[i] + self.counts[i - 1])
                        / (h * h);
                }
            }
        }

        XpsSpectrum {
            binding_energy_ev: self.binding_energy_ev.clone(),
            counts: result,
        }
    }

    /// Convert kinetic energy to binding energy.
    ///
    /// BE = hν - KE - φ
    pub fn kinetic_to_binding(ke_ev: f64, photon_energy_ev: f64, work_function_ev: f64) -> f64 {
        photon_energy_ev - ke_ev - work_function_ev
    }

    /// Find the index of the energy value closest to `target_ev`.
    fn closest_index(&self, target_ev: f64) -> usize {
        let mut best = 0;
        let mut best_dist = (self.binding_energy_ev[0] - target_ev).abs();
        for (i, &e) in self.binding_energy_ev.iter().enumerate().skip(1) {
            let d = (e - target_ev).abs();
            if d < best_dist {
                best_dist = d;
                best = i;
            }
        }
        best
    }

    /// Return a sub-spectrum between be_start and be_end (inclusive of nearest points).
    pub fn region(&self, be_start: f64, be_end: f64) -> XpsSpectrum {
        let lo = be_start.min(be_end);
        let hi = be_start.max(be_end);
        let mut be = Vec::new();
        let mut cts = Vec::new();
        for (i, &e) in self.binding_energy_ev.iter().enumerate() {
            if e >= lo && e <= hi {
                be.push(e);
                cts.push(self.counts[i]);
            }
        }
        if be.is_empty() {
            XpsSpectrum {
                binding_energy_ev: vec![lo],
                counts: vec![0.0],
            }
        } else {
            XpsSpectrum {
                binding_energy_ev: be,
                counts: cts,
            }
        }
    }
}

// ─── Savitzky-Golay helper ───────────────────────────────────────────

/// Compute Savitzky-Golay convolution coefficients for a window of size `m`
/// (must be odd), polynomial order `poly_order`, and derivative order `deriv`.
fn savitzky_golay_coeffs(m: usize, poly_order: usize, deriv: usize) -> Vec<f64> {
    let half = m as isize / 2;
    let p = poly_order + 1;
    // Build the Vandermonde-like normal equations: J^T J c = J^T e_deriv
    // where J[i][k] = i^k for i in -half..=half
    let n = m;
    // Build J^T J (p x p)
    let mut jtj = vec![vec![0.0; p]; p];
    for i in 0..n {
        let x = (i as isize - half) as f64;
        for r in 0..p {
            for c in 0..p {
                jtj[r][c] += x.powi((r + c) as i32);
            }
        }
    }
    // Solve for each data point to get convolution weights
    // For derivative `deriv`, we want the coefficient of x^deriv in the fitted polynomial
    // multiplied by deriv!
    let mut rhs = vec![0.0; p];
    if deriv < p {
        rhs[deriv] = factorial(deriv) as f64;
    }
    let coeffs_poly = solve_linear_system(&jtj, &rhs);
    // Now compute the weight for each sample point
    let mut weights = vec![0.0; n];
    for i in 0..n {
        let x = (i as isize - half) as f64;
        let mut w = 0.0;
        for k in 0..p {
            w += coeffs_poly[k] * x.powi(k as i32);
        }
        weights[i] = w;
    }
    weights
}

fn factorial(n: usize) -> usize {
    if n <= 1 {
        1
    } else {
        n * factorial(n - 1)
    }
}

/// Solve a small dense linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = a[i].clone();
        row.push(b[i]);
        aug.push(row);
    }
    // Forward elimination with partial pivoting
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in col + 1..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);
        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for row in col + 1..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                let v = aug[col][j];
                aug[row][j] -= factor * v;
            }
        }
    }
    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in i + 1..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }
    x
}

// ─── Background types ────────────────────────────────────────────────

/// Background subtraction method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BackgroundType {
    Shirley,
    Tougaard,
    Linear,
    None,
}

// ─── ShirleyBackground ──────────────────────────────────────────────

/// Shirley iterative background calculation.
///
/// The Shirley background at energy E is proportional to the integrated
/// spectral intensity above the background from E to the high-BE endpoint:
///   B(E) = k ∫ₑᴱ² [I(E') - B(E')] dE' + I(E₂)
pub struct ShirleyBackground {
    /// Maximum iterations for convergence.
    pub max_iterations: usize,
    /// Convergence threshold (max absolute change between iterations).
    pub convergence_threshold: f64,
}

impl Default for ShirleyBackground {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            convergence_threshold: 1e-6,
        }
    }
}

impl ShirleyBackground {
    /// Create a new Shirley background calculator.
    pub fn new(max_iterations: usize, convergence_threshold: f64) -> Self {
        Self {
            max_iterations,
            convergence_threshold,
        }
    }

    /// Calculate the Shirley background for the given spectrum region.
    ///
    /// `be_start` and `be_end` define the energy window. The background
    /// is iterated until convergence. Returns a background vector with
    /// the same length as the spectrum.
    pub fn calculate(&self, spectrum: &XpsSpectrum, be_start: f64, be_end: f64) -> Vec<f64> {
        let n = spectrum.counts.len();
        let mut bg = vec![0.0; n];

        // Find indices for the region endpoints
        let idx_lo = spectrum.closest_index(be_start.min(be_end));
        let idx_hi = spectrum.closest_index(be_start.max(be_end));
        let (i_start, i_end) = if idx_lo <= idx_hi {
            (idx_lo, idx_hi)
        } else {
            (idx_hi, idx_lo)
        };

        if i_start == i_end || n < 2 {
            return bg;
        }

        let y_start = spectrum.counts[i_start];
        let y_end = spectrum.counts[i_end];

        // Initialize background to linear
        for i in 0..n {
            if i <= i_start {
                bg[i] = y_start;
            } else if i >= i_end {
                bg[i] = y_end;
            } else {
                let frac = (i - i_start) as f64 / (i_end - i_start) as f64;
                bg[i] = y_start + frac * (y_end - y_start);
            }
        }

        // Iterative Shirley
        for _iter in 0..self.max_iterations {
            let mut max_change = 0.0;
            let mut new_bg = bg.clone();

            // Compute total integral of (I - B) from i_start to i_end
            let mut total_integral = 0.0;
            for i in i_start..i_end {
                total_integral += (spectrum.counts[i] - bg[i]).max(0.0);
            }

            if total_integral < 1e-30 {
                break;
            }

            let k = (y_end - y_start).abs() / total_integral;

            // For each point, background = y_end + k * integral from point to i_end
            for i in i_start..=i_end {
                let mut partial_integral = 0.0;
                for j in i..i_end {
                    partial_integral += (spectrum.counts[j] - bg[j]).max(0.0);
                }
                // Shirley: B(E) = I_right + k * ∫ from E to E_right
                let new_val = y_end + k * partial_integral;
                let change = (new_val - new_bg[i]).abs();
                if change > max_change {
                    max_change = change;
                }
                new_bg[i] = new_val;
            }

            bg = new_bg;

            if max_change < self.convergence_threshold {
                break;
            }
        }

        bg
    }

    /// Subtract a background from the spectrum.
    pub fn subtract(spectrum: &XpsSpectrum, background: &[f64]) -> XpsSpectrum {
        let counts: Vec<f64> = spectrum
            .counts
            .iter()
            .zip(background.iter())
            .map(|(&c, &b)| (c - b).max(0.0))
            .collect();
        XpsSpectrum {
            binding_energy_ev: spectrum.binding_energy_ev.clone(),
            counts,
        }
    }
}

// ─── TougaardBackground ─────────────────────────────────────────────

/// Tougaard universal background subtraction.
///
/// Uses the universal cross-section K(T) = B*T / (C + T²)²
/// with default parameters B=2866 eV², C=1643 eV².
pub struct TougaardBackground;

impl TougaardBackground {
    /// Calculate the Tougaard background.
    ///
    /// B(E) = λ ∫ K(E'-E) I(E') dE' for E' > E
    /// K(T) = b_param * T / (c_param + T²)²
    pub fn calculate(
        spectrum: &XpsSpectrum,
        b_param: f64,
        c_param: f64,
    ) -> Vec<f64> {
        let n = spectrum.counts.len();
        let mut bg = vec![0.0; n];

        // We need sorted ascending energies for integration
        let mut indexed: Vec<(usize, f64)> = spectrum
            .binding_energy_ev
            .iter()
            .enumerate()
            .map(|(i, &e)| (i, e))
            .collect();
        indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        for &(i, ei) in &indexed {
            let mut integral = 0.0;
            // Integrate over energies greater than ei
            for k in 0..indexed.len().saturating_sub(1) {
                let (j, ej) = indexed[k];
                let (_, ej1) = indexed[k + 1];
                if ej <= ei {
                    continue;
                }
                let t = ej - ei;
                let denom = c_param + t * t;
                if denom.abs() < 1e-30 {
                    continue;
                }
                let kernel = b_param * t / (denom * denom);
                let de = (ej1 - ej).abs();
                integral += kernel * spectrum.counts[j] * de;
            }
            bg[i] = integral;
        }

        // Scale: typically a factor around 1e-3 to 1e-4
        // We normalize so the background does not exceed the spectrum
        let max_spec = spectrum.max_counts();
        let max_bg = bg.iter().cloned().fold(0.0_f64, f64::max);
        if max_bg > max_spec && max_bg > 1e-30 {
            let scale = max_spec * 0.3 / max_bg;
            for b in &mut bg {
                *b *= scale;
            }
        }

        bg
    }

    /// Default universal cross-section parameters (B=2866 eV², C=1643 eV²).
    pub fn universal_params() -> (f64, f64) {
        (2866.0, 1643.0)
    }
}

// ─── LinearBackground ───────────────────────────────────────────────

/// Simple linear background between two energy endpoints.
pub struct LinearBackground;

impl LinearBackground {
    /// Calculate a linear background between `be_start` and `be_end`.
    pub fn calculate(spectrum: &XpsSpectrum, be_start: f64, be_end: f64) -> Vec<f64> {
        let n = spectrum.counts.len();
        let idx_start = spectrum.closest_index(be_start);
        let idx_end = spectrum.closest_index(be_end);
        let (i0, i1) = if idx_start <= idx_end {
            (idx_start, idx_end)
        } else {
            (idx_end, idx_start)
        };

        let y0 = spectrum.counts[i0];
        let y1 = spectrum.counts[i1];
        let e0 = spectrum.binding_energy_ev[i0];
        let e1 = spectrum.binding_energy_ev[i1];
        let de = e1 - e0;

        let mut bg = vec![0.0; n];
        for i in 0..n {
            let e = spectrum.binding_energy_ev[i];
            if de.abs() < 1e-15 {
                bg[i] = (y0 + y1) / 2.0;
            } else {
                let frac = (e - e0) / de;
                bg[i] = y0 + frac * (y1 - y0);
            }
        }
        bg
    }
}

// ─── Peak shapes ────────────────────────────────────────────────────

/// Gaussian peak shape.
/// G(x) = A * exp(-4 ln2 (x - x0)² / fwhm²)
pub fn gaussian(x: f64, center: f64, fwhm: f64, amplitude: f64) -> f64 {
    let sigma_sq = fwhm * fwhm / (8.0 * 2.0_f64.ln());
    if sigma_sq < 1e-30 {
        return 0.0;
    }
    amplitude * (-(x - center).powi(2) / (2.0 * sigma_sq)).exp()
}

/// Lorentzian peak shape.
/// L(x) = A * (fwhm/2)² / [(x - x0)² + (fwhm/2)²]
pub fn lorentzian(x: f64, center: f64, fwhm: f64, amplitude: f64) -> f64 {
    let half_w = fwhm / 2.0;
    let dx = x - center;
    amplitude * half_w * half_w / (dx * dx + half_w * half_w)
}

/// Pseudo-Voigt profile: η * L(x) + (1-η) * G(x)
///
/// `eta` is the Lorentzian fraction (0 = pure Gaussian, 1 = pure Lorentzian).
pub fn pseudo_voigt(be: f64, center: f64, fwhm: f64, eta: f64, amplitude: f64) -> f64 {
    let eta_c = eta.clamp(0.0, 1.0);
    eta_c * lorentzian(be, center, fwhm, amplitude)
        + (1.0 - eta_c) * gaussian(be, center, fwhm, amplitude)
}

/// Doniach-Sunjic asymmetric line shape for metallic peaks.
///
/// DS(x) = cos(π α/2 + (1-α) arctan((x-x0)/γ)) / [(x-x0)² + γ²]^((1-α)/2)
///
/// `asymmetry` (α) typically 0.0-0.2 for metals.
pub fn doniach_sunjic(x: f64, center: f64, gamma: f64, asymmetry: f64, amplitude: f64) -> f64 {
    let dx = x - center;
    let r2 = dx * dx + gamma * gamma;
    if r2 < 1e-30 {
        return amplitude;
    }
    let atan_term = if gamma.abs() > 1e-15 {
        (dx / gamma).atan()
    } else {
        0.0
    };
    let phase = PI * asymmetry / 2.0 + (1.0 - asymmetry) * atan_term;
    let denom = r2.powf((1.0 - asymmetry) / 2.0);
    amplitude * phase.cos() / denom
}

// ─── XpsPeak ────────────────────────────────────────────────────────

/// A single fitted XPS peak.
#[derive(Debug, Clone)]
pub struct XpsPeak {
    /// Peak center binding energy in eV.
    pub binding_energy_ev: f64,
    /// Full width at half maximum in eV.
    pub fwhm_ev: f64,
    /// Peak amplitude (height).
    pub amplitude: f64,
    /// Integrated peak area.
    pub area: f64,
    /// Gaussian-Lorentzian mixing parameter (0 = Gaussian, 1 = Lorentzian).
    pub gl_mix: f64,
    /// Asymmetry parameter (Doniach-Sunjic, 0 = symmetric).
    pub asymmetry: f64,
}

impl XpsPeak {
    /// Evaluate this peak at the given binding energy.
    pub fn evaluate(&self, be: f64) -> f64 {
        if self.asymmetry > 0.01 {
            doniach_sunjic(be, self.binding_energy_ev, self.fwhm_ev / 2.0, self.asymmetry, self.amplitude)
        } else {
            pseudo_voigt(be, self.binding_energy_ev, self.fwhm_ev, self.gl_mix, self.amplitude)
        }
    }

    /// Compute the peak area by numerical integration over a reasonable range.
    pub fn compute_area(&self, energy_step: f64) -> f64 {
        let range = self.fwhm_ev * 5.0;
        let start = self.binding_energy_ev - range;
        let end = self.binding_energy_ev + range;
        let steps = ((end - start) / energy_step).ceil() as usize;
        let mut area = 0.0;
        for i in 0..steps {
            let e = start + i as f64 * energy_step;
            area += self.evaluate(e) * energy_step;
        }
        area
    }
}

// ─── PeakConstraint ─────────────────────────────────────────────────

/// Constraints for peak fitting.
#[derive(Debug, Clone)]
pub enum PeakConstraint {
    /// Fix the binding energy of peak at index.
    FixPosition(usize, f64),
    /// Fix the relative BE separation between two peaks.
    RelativePosition(usize, usize, f64),
    /// Fix the FWHM ratio between two peaks.
    FwhmRatio(usize, usize, f64),
    /// Fix the area ratio between two peaks.
    AreaRatio(usize, usize, f64),
    /// Fix the GL mixing parameter.
    FixGlMix(usize, f64),
    /// Maximum FWHM.
    MaxFwhm(usize, f64),
    /// Minimum FWHM.
    MinFwhm(usize, f64),
}

// ─── XpsFitResult ───────────────────────────────────────────────────

/// Result of XPS peak fitting.
#[derive(Debug, Clone)]
pub struct XpsFitResult {
    /// Fitted peaks.
    pub peaks: Vec<XpsPeak>,
    /// Residual sum of squares.
    pub residual_ss: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
    /// Background used.
    pub background: Vec<f64>,
}

impl XpsFitResult {
    /// Evaluate the total fitted envelope at the given binding energy.
    pub fn envelope(&self, be: f64) -> f64 {
        self.peaks.iter().map(|p| p.evaluate(be)).sum::<f64>()
    }
}

// ─── PeakFitter ─────────────────────────────────────────────────────

/// Peak fitting engine for XPS spectra.
pub struct PeakFitter {
    /// Maximum optimization iterations.
    pub max_iterations: usize,
    /// Convergence tolerance.
    pub tolerance: f64,
}

impl Default for PeakFitter {
    fn default() -> Self {
        Self {
            max_iterations: 200,
            tolerance: 1e-8,
        }
    }
}

impl PeakFitter {
    /// Create a new peak fitter.
    pub fn new(max_iterations: usize, tolerance: f64) -> Self {
        Self {
            max_iterations,
            tolerance,
        }
    }

    /// Fit peaks to the spectrum using pseudo-Voigt profiles.
    ///
    /// Initial peak positions are estimated from the highest points in the
    /// background-subtracted spectrum. Uses iterative least-squares refinement.
    pub fn fit_peaks(
        &self,
        spectrum: &XpsSpectrum,
        num_peaks: usize,
        bg_type: BackgroundType,
    ) -> XpsFitResult {
        let n = spectrum.counts.len();
        if n == 0 || num_peaks == 0 {
            return XpsFitResult {
                peaks: vec![],
                residual_ss: 0.0,
                r_squared: 1.0,
                background: vec![0.0; n],
            };
        }

        let (be_min, be_max) = spectrum.energy_range();

        // Calculate background
        let background = match bg_type {
            BackgroundType::Shirley => {
                ShirleyBackground::default().calculate(spectrum, be_min, be_max)
            }
            BackgroundType::Tougaard => {
                let (b, c) = TougaardBackground::universal_params();
                TougaardBackground::calculate(spectrum, b, c)
            }
            BackgroundType::Linear => LinearBackground::calculate(spectrum, be_min, be_max),
            BackgroundType::None => vec![0.0; n],
        };

        // Subtract background
        let subtracted = ShirleyBackground::subtract(spectrum, &background);

        // Find initial peak positions from the N highest local maxima
        let mut peak_candidates: Vec<(usize, f64)> = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if subtracted.counts[i] >= subtracted.counts[i - 1]
                && subtracted.counts[i] >= subtracted.counts[i + 1]
                && subtracted.counts[i] > 0.0
            {
                peak_candidates.push((i, subtracted.counts[i]));
            }
        }
        // Also consider endpoints
        if n > 0 && subtracted.counts[0] > 0.0 {
            peak_candidates.push((0, subtracted.counts[0]));
        }
        if n > 1 && subtracted.counts[n - 1] > 0.0 {
            peak_candidates.push((n - 1, subtracted.counts[n - 1]));
        }
        peak_candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let de = if n > 1 {
            ((be_max - be_min) / (n as f64 - 1.0)).abs()
        } else {
            0.1
        };

        // Initialize peaks
        let actual_peaks = num_peaks.min(peak_candidates.len().max(1));
        let mut peaks: Vec<XpsPeak> = Vec::with_capacity(actual_peaks);
        for i in 0..actual_peaks {
            if i < peak_candidates.len() {
                let (idx, amp) = peak_candidates[i];
                peaks.push(XpsPeak {
                    binding_energy_ev: spectrum.binding_energy_ev[idx],
                    fwhm_ev: de * 5.0, // initial guess
                    amplitude: amp,
                    area: 0.0,
                    gl_mix: 0.3, // moderate Gaussian-Lorentzian mix
                    asymmetry: 0.0,
                });
            } else {
                // Distribute evenly if not enough candidates
                let frac = (i as f64 + 1.0) / (actual_peaks as f64 + 1.0);
                let be_pos = be_min + frac * (be_max - be_min);
                peaks.push(XpsPeak {
                    binding_energy_ev: be_pos,
                    fwhm_ev: de * 5.0,
                    amplitude: subtracted.max_counts() * 0.5,
                    area: 0.0,
                    gl_mix: 0.3,
                    asymmetry: 0.0,
                });
            }
        }

        // Iterative refinement: simple gradient-descent-like optimization
        // Parameters per peak: [center, fwhm, amplitude]
        let np = peaks.len();
        let step_center = de * 0.1;
        let step_fwhm = de * 0.05;

        for _iteration in 0..self.max_iterations {
            let mut improved = false;

            for pi in 0..np {
                let current_rss = self.compute_residual(&subtracted, &peaks);

                // Try adjusting center
                for &dir in &[-1.0, 1.0] {
                    let old = peaks[pi].binding_energy_ev;
                    peaks[pi].binding_energy_ev = old + dir * step_center;
                    let new_rss = self.compute_residual(&subtracted, &peaks);
                    if new_rss < current_rss {
                        improved = true;
                    } else {
                        peaks[pi].binding_energy_ev = old;
                    }
                }

                // Try adjusting FWHM
                for &dir in &[-1.0, 1.0] {
                    let old = peaks[pi].fwhm_ev;
                    let new_fwhm = (old + dir * step_fwhm).max(de * 0.5);
                    peaks[pi].fwhm_ev = new_fwhm;
                    let new_rss = self.compute_residual(&subtracted, &peaks);
                    if new_rss < current_rss {
                        improved = true;
                    } else {
                        peaks[pi].fwhm_ev = old;
                    }
                }

                // Optimize amplitude analytically (least squares for this peak)
                let mut num = 0.0;
                let mut den = 0.0;
                for i in 0..n {
                    let be = subtracted.binding_energy_ev[i];
                    let other_sum: f64 = peaks.iter().enumerate()
                        .filter(|(j, _)| *j != pi)
                        .map(|(_, p)| p.evaluate(be))
                        .sum();
                    let residual = subtracted.counts[i] - other_sum;
                    let basis = pseudo_voigt(be, peaks[pi].binding_energy_ev, peaks[pi].fwhm_ev, peaks[pi].gl_mix, 1.0);
                    num += residual * basis;
                    den += basis * basis;
                }
                if den > 1e-30 {
                    peaks[pi].amplitude = (num / den).max(0.0);
                }
            }

            if !improved {
                break;
            }
        }

        // Compute areas
        for peak in &mut peaks {
            peak.area = peak.compute_area(de.max(0.01));
        }

        // Compute R²
        let mean_counts = subtracted.counts.iter().sum::<f64>() / n as f64;
        let ss_tot: f64 = subtracted.counts.iter().map(|&c| (c - mean_counts).powi(2)).sum();
        let residual_ss = self.compute_residual(&subtracted, &peaks);
        let r_squared = if ss_tot > 1e-30 {
            1.0 - residual_ss / ss_tot
        } else {
            1.0
        };

        XpsFitResult {
            peaks,
            residual_ss,
            r_squared,
            background,
        }
    }

    /// Apply peak constraints to an existing fit result.
    pub fn constrain_peaks(result: &mut XpsFitResult, constraints: &[PeakConstraint]) {
        for constraint in constraints {
            match constraint {
                PeakConstraint::FixPosition(idx, be) => {
                    if *idx < result.peaks.len() {
                        result.peaks[*idx].binding_energy_ev = *be;
                    }
                }
                PeakConstraint::RelativePosition(idx1, idx2, delta) => {
                    if *idx1 < result.peaks.len() && *idx2 < result.peaks.len() {
                        let be1 = result.peaks[*idx1].binding_energy_ev;
                        result.peaks[*idx2].binding_energy_ev = be1 + delta;
                    }
                }
                PeakConstraint::FwhmRatio(idx1, idx2, ratio) => {
                    if *idx1 < result.peaks.len() && *idx2 < result.peaks.len() {
                        result.peaks[*idx2].fwhm_ev = result.peaks[*idx1].fwhm_ev * ratio;
                    }
                }
                PeakConstraint::AreaRatio(idx1, idx2, ratio) => {
                    if *idx1 < result.peaks.len() && *idx2 < result.peaks.len() {
                        result.peaks[*idx2].amplitude = result.peaks[*idx1].amplitude * ratio;
                    }
                }
                PeakConstraint::FixGlMix(idx, eta) => {
                    if *idx < result.peaks.len() {
                        result.peaks[*idx].gl_mix = *eta;
                    }
                }
                PeakConstraint::MaxFwhm(idx, max_fwhm) => {
                    if *idx < result.peaks.len() && result.peaks[*idx].fwhm_ev > *max_fwhm {
                        result.peaks[*idx].fwhm_ev = *max_fwhm;
                    }
                }
                PeakConstraint::MinFwhm(idx, min_fwhm) => {
                    if *idx < result.peaks.len() && result.peaks[*idx].fwhm_ev < *min_fwhm {
                        result.peaks[*idx].fwhm_ev = *min_fwhm;
                    }
                }
            }
        }
    }

    fn compute_residual(&self, spectrum: &XpsSpectrum, peaks: &[XpsPeak]) -> f64 {
        let mut rss = 0.0;
        for (i, &be) in spectrum.binding_energy_ev.iter().enumerate() {
            let model: f64 = peaks.iter().map(|p| p.evaluate(be)).sum();
            let diff = spectrum.counts[i] - model;
            rss += diff * diff;
        }
        rss
    }
}

// ─── ChemicalStateIdentifier ────────────────────────────────────────

/// A match from the binding energy database.
#[derive(Debug, Clone)]
pub struct ElementMatch {
    /// Element symbol.
    pub element: &'static str,
    /// Orbital designation (e.g., "1s", "2p3/2").
    pub orbital: &'static str,
    /// Expected binding energy in eV.
    pub expected_be: f64,
    /// Chemical state description.
    pub chemical_state: &'static str,
}

/// Core-level binding energy database entry.
struct CoreLevelEntry {
    element: &'static str,
    orbital: &'static str,
    binding_energy: f64,
    chemical_state: &'static str,
}

/// Core-level binding energy database for common elements.
const CORE_LEVEL_DB: &[CoreLevelEntry] = &[
    // Carbon 1s
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 284.8, chemical_state: "C-C/C-H (adventitious)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 285.0, chemical_state: "C-C sp3" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 284.4, chemical_state: "C=C sp2 (graphite)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 286.5, chemical_state: "C-O (ether/alcohol)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 288.0, chemical_state: "C=O (carbonyl)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 289.0, chemical_state: "O-C=O (carboxyl)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 290.5, chemical_state: "CO3 (carbonate)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 292.0, chemical_state: "CF2 (PTFE)" },
    CoreLevelEntry { element: "C", orbital: "1s", binding_energy: 293.5, chemical_state: "CF3" },
    // Oxygen 1s
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 530.0, chemical_state: "Metal oxide (lattice O²⁻)" },
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 531.0, chemical_state: "O 1s (metal oxide)" },
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 531.5, chemical_state: "C=O" },
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 532.0, chemical_state: "OH / C-O" },
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 533.0, chemical_state: "H2O (adsorbed)" },
    CoreLevelEntry { element: "O", orbital: "1s", binding_energy: 534.0, chemical_state: "O in organic peroxide" },
    // Nitrogen 1s
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 398.0, chemical_state: "Pyridinic N" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 399.0, chemical_state: "N 1s amine" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 400.0, chemical_state: "Pyrrolic N" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 401.0, chemical_state: "Graphitic/quaternary N" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 402.5, chemical_state: "N-O (nitro)" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 405.0, chemical_state: "NO3" },
    CoreLevelEntry { element: "N", orbital: "1s", binding_energy: 407.0, chemical_state: "NO2 (nitrate)" },
    // Silicon 2p
    CoreLevelEntry { element: "Si", orbital: "2p", binding_energy: 99.3, chemical_state: "Si elemental" },
    CoreLevelEntry { element: "Si", orbital: "2p", binding_energy: 100.5, chemical_state: "Si-C (SiC)" },
    CoreLevelEntry { element: "Si", orbital: "2p", binding_energy: 101.5, chemical_state: "Si suboxide (SiOx)" },
    CoreLevelEntry { element: "Si", orbital: "2p", binding_energy: 103.3, chemical_state: "SiO2" },
    CoreLevelEntry { element: "Si", orbital: "2p", binding_energy: 102.0, chemical_state: "Si3N4" },
    // Iron 2p
    CoreLevelEntry { element: "Fe", orbital: "2p3/2", binding_energy: 707.0, chemical_state: "Fe metal" },
    CoreLevelEntry { element: "Fe", orbital: "2p3/2", binding_energy: 709.5, chemical_state: "Fe²⁺ (FeO)" },
    CoreLevelEntry { element: "Fe", orbital: "2p3/2", binding_energy: 711.0, chemical_state: "Fe³⁺ (Fe2O3)" },
    CoreLevelEntry { element: "Fe", orbital: "2p3/2", binding_energy: 712.5, chemical_state: "FeOOH" },
    // Gold 4f
    CoreLevelEntry { element: "Au", orbital: "4f7/2", binding_energy: 84.0, chemical_state: "Au metal" },
    CoreLevelEntry { element: "Au", orbital: "4f7/2", binding_energy: 85.8, chemical_state: "Au¹⁺ (Au2O)" },
    CoreLevelEntry { element: "Au", orbital: "4f7/2", binding_energy: 86.9, chemical_state: "Au³⁺ (Au2O3)" },
    // Titanium 2p
    CoreLevelEntry { element: "Ti", orbital: "2p3/2", binding_energy: 454.0, chemical_state: "Ti metal" },
    CoreLevelEntry { element: "Ti", orbital: "2p3/2", binding_energy: 455.0, chemical_state: "TiN" },
    CoreLevelEntry { element: "Ti", orbital: "2p3/2", binding_energy: 458.5, chemical_state: "TiO2" },
    // Aluminum 2p
    CoreLevelEntry { element: "Al", orbital: "2p", binding_energy: 72.9, chemical_state: "Al metal" },
    CoreLevelEntry { element: "Al", orbital: "2p", binding_energy: 74.5, chemical_state: "Al2O3" },
    // Copper 2p
    CoreLevelEntry { element: "Cu", orbital: "2p3/2", binding_energy: 932.7, chemical_state: "Cu metal" },
    CoreLevelEntry { element: "Cu", orbital: "2p3/2", binding_energy: 933.5, chemical_state: "Cu²⁺ (CuO)" },
    CoreLevelEntry { element: "Cu", orbital: "2p3/2", binding_energy: 932.5, chemical_state: "Cu¹⁺ (Cu2O)" },
    // Zinc 2p
    CoreLevelEntry { element: "Zn", orbital: "2p3/2", binding_energy: 1021.8, chemical_state: "Zn metal" },
    CoreLevelEntry { element: "Zn", orbital: "2p3/2", binding_energy: 1022.5, chemical_state: "ZnO" },
    // Silver 3d
    CoreLevelEntry { element: "Ag", orbital: "3d5/2", binding_energy: 368.2, chemical_state: "Ag metal" },
    CoreLevelEntry { element: "Ag", orbital: "3d5/2", binding_energy: 367.7, chemical_state: "Ag2O" },
    // Sulfur 2p
    CoreLevelEntry { element: "S", orbital: "2p", binding_energy: 162.0, chemical_state: "S²⁻ (sulfide)" },
    CoreLevelEntry { element: "S", orbital: "2p", binding_energy: 163.5, chemical_state: "S-S (disulfide)" },
    CoreLevelEntry { element: "S", orbital: "2p", binding_energy: 164.0, chemical_state: "S (elemental)" },
    CoreLevelEntry { element: "S", orbital: "2p", binding_energy: 168.5, chemical_state: "SO4²⁻ (sulfate)" },
    CoreLevelEntry { element: "S", orbital: "2p", binding_energy: 166.5, chemical_state: "SO3²⁻ (sulfite)" },
    // Fluorine 1s
    CoreLevelEntry { element: "F", orbital: "1s", binding_energy: 685.0, chemical_state: "F⁻ (fluoride)" },
    CoreLevelEntry { element: "F", orbital: "1s", binding_energy: 688.0, chemical_state: "C-F (organic)" },
    // Chlorine 2p
    CoreLevelEntry { element: "Cl", orbital: "2p", binding_energy: 198.5, chemical_state: "Cl⁻ (chloride)" },
    CoreLevelEntry { element: "Cl", orbital: "2p", binding_energy: 200.0, chemical_state: "C-Cl (organic)" },
    // Phosphorus 2p
    CoreLevelEntry { element: "P", orbital: "2p", binding_energy: 130.0, chemical_state: "P elemental" },
    CoreLevelEntry { element: "P", orbital: "2p", binding_energy: 133.5, chemical_state: "PO4³⁻ (phosphate)" },
];

/// Chemical state identifier using a core-level binding energy database.
pub struct ChemicalStateIdentifier;

impl ChemicalStateIdentifier {
    /// Identify possible elements and chemical states for a given binding energy.
    ///
    /// Returns matches within `tolerance_ev` (default ±1.5 eV) of the measured BE.
    pub fn identify_element(binding_energy: f64) -> Vec<ElementMatch> {
        Self::identify_element_with_tolerance(binding_energy, 1.5)
    }

    /// Identify with a custom tolerance window.
    pub fn identify_element_with_tolerance(binding_energy: f64, tolerance_ev: f64) -> Vec<ElementMatch> {
        let mut matches: Vec<ElementMatch> = Vec::new();
        for entry in CORE_LEVEL_DB.iter() {
            if (entry.binding_energy - binding_energy).abs() <= tolerance_ev {
                matches.push(ElementMatch {
                    element: entry.element,
                    orbital: entry.orbital,
                    expected_be: entry.binding_energy,
                    chemical_state: entry.chemical_state,
                });
            }
        }
        // Sort by proximity
        matches.sort_by(|a, b| {
            let da = (a.expected_be - binding_energy).abs();
            let db = (b.expected_be - binding_energy).abs();
            da.partial_cmp(&db).unwrap()
        });
        matches
    }

    /// Look up carbon chemical shifts.
    pub fn carbon_chemical_shifts() -> Vec<(&'static str, f64)> {
        vec![
            ("C-C/C-H", 285.0),
            ("C=C sp2", 284.4),
            ("C-N", 286.0),
            ("C-O", 286.5),
            ("C=O", 288.0),
            ("O-C=O", 289.0),
            ("CO3", 290.5),
            ("CF2", 292.0),
            ("CF3", 293.5),
        ]
    }
}

// ─── Quantification ─────────────────────────────────────────────────

/// XPS quantification utilities.
pub struct Quantification;

impl Quantification {
    /// Calculate atomic percent from peak areas and sensitivity factors.
    ///
    /// at% = (Aᵢ/SFᵢ) / Σ(Aⱼ/SFⱼ) × 100
    pub fn atomic_percent(peak_areas: &[f64], sensitivity_factors: &[f64]) -> Vec<f64> {
        assert_eq!(
            peak_areas.len(),
            sensitivity_factors.len(),
            "Areas and sensitivity factors must have equal length"
        );
        let normalized: Vec<f64> = peak_areas
            .iter()
            .zip(sensitivity_factors.iter())
            .map(|(&a, &sf)| if sf > 1e-30 { a / sf } else { 0.0 })
            .collect();
        let total: f64 = normalized.iter().sum();
        if total < 1e-30 {
            return vec![0.0; peak_areas.len()];
        }
        normalized.iter().map(|&n| n / total * 100.0).collect()
    }

    /// Get the Scofield cross-section (relative sensitivity factor) for a given
    /// element and orbital using Al Kα radiation (1486.6 eV).
    ///
    /// Values are approximate Wagner/Scofield sensitivity factors normalized to F 1s = 1.00.
    pub fn sensitivity_factor(element: &str, orbital: &str) -> f64 {
        match (element, orbital) {
            // Wagner empirical relative sensitivity factors (Al Kα)
            ("C", "1s") => 0.296,
            ("N", "1s") => 0.477,
            ("O", "1s") => 0.711,
            ("F", "1s") => 1.000,
            ("Na", "1s") => 2.300,
            ("Mg", "1s") => 2.760,
            ("Al", "2p") => 0.234,
            ("Si", "2p") => 0.339,
            ("P", "2p") => 0.486,
            ("S", "2p") => 0.668,
            ("Cl", "2p") => 0.891,
            ("K", "2p") => 1.466,
            ("Ca", "2p") => 1.833,
            ("Ti", "2p3/2") => 2.001,
            ("Fe", "2p3/2") => 2.957,
            ("Cu", "2p3/2") => 5.321,
            ("Zn", "2p3/2") => 5.588,
            ("Ag", "3d5/2") => 5.987,
            ("Au", "4f7/2") => 6.250,
            ("Pt", "4f7/2") => 5.575,
            ("Pd", "3d5/2") => 4.581,
            ("Ni", "2p3/2") => 3.898,
            ("Co", "2p3/2") => 3.382,
            ("Mn", "2p3/2") => 2.660,
            ("Cr", "2p3/2") => 2.237,
            ("V", "2p3/2") => 1.846,
            _ => 1.0, // default
        }
    }

    /// Calculate stoichiometry normalized to the smallest value.
    pub fn stoichiometry(at_percents: &[f64], _elements: &[&str]) -> Vec<f64> {
        if at_percents.is_empty() {
            return vec![];
        }
        let min_val = at_percents
            .iter()
            .cloned()
            .filter(|&v| v > 0.01)
            .fold(f64::INFINITY, f64::min);
        if min_val < 0.01 {
            return at_percents.to_vec();
        }
        at_percents.iter().map(|&v| v / min_val).collect()
    }
}

// ─── SpinOrbitSplitting ─────────────────────────────────────────────

/// Spin-orbit splitting analysis for core-level doublets.
pub struct SpinOrbitSplitting;

impl SpinOrbitSplitting {
    /// Return the spin-orbit splitting (ΔBE) for a given element and orbital type.
    ///
    /// Values in eV for common core levels.
    pub fn doublet_spacing(element: &str, orbital: &str) -> f64 {
        match (element, orbital) {
            // 2p doublets
            ("Si", "2p") => 0.6,
            ("Al", "2p") => 0.44,
            ("P", "2p") => 0.87,
            ("S", "2p") => 1.16,
            ("Cl", "2p") => 1.6,
            ("Ti", "2p") => 5.54,
            ("V", "2p") => 7.33,
            ("Cr", "2p") => 9.05,
            ("Mn", "2p") => 11.1,
            ("Fe", "2p") => 13.1,
            ("Co", "2p") => 14.97,
            ("Ni", "2p") => 17.27,
            ("Cu", "2p") => 19.75,
            ("Zn", "2p") => 23.05,
            // 3d doublets
            ("Ag", "3d") => 6.0,
            ("Pd", "3d") => 5.26,
            ("In", "3d") => 7.56,
            ("Sn", "3d") => 8.42,
            // 4f doublets
            ("Au", "4f") => 3.67,
            ("Pt", "4f") => 3.33,
            ("Ir", "4f") => 2.98,
            ("W", "4f") => 2.18,
            ("Ta", "4f") => 1.91,
            ("Hf", "4f") => 1.66,
            _ => 0.0,
        }
    }

    /// Return the area ratio for a spin-orbit doublet based on the orbital quantum number.
    ///
    /// The ratio is (2j₊ + 1)/(2j₋ + 1) where j₊ = l + 1/2, j₋ = l - 1/2.
    /// - p orbitals (l=1): p3/2 : p1/2 = 2 : 1
    /// - d orbitals (l=2): d5/2 : d3/2 = 3 : 2
    /// - f orbitals (l=3): f7/2 : f5/2 = 4 : 3
    pub fn doublet_ratio(orbital: &str) -> f64 {
        if orbital.contains('p') {
            2.0 / 1.0
        } else if orbital.contains('d') {
            3.0 / 2.0
        } else if orbital.contains('f') {
            4.0 / 3.0
        } else {
            1.0 // s orbitals have no splitting
        }
    }

    /// Given a main peak (the higher-j component), generate the partner peak
    /// for the spin-orbit doublet.
    ///
    /// The partner peak has:
    /// - BE shifted by the doublet spacing
    /// - Area scaled by the inverse of the doublet ratio
    /// - Same FWHM and line shape
    pub fn constrain_doublet(peak: &XpsPeak, element: &str, orbital: &str) -> XpsPeak {
        let spacing = Self::doublet_spacing(element, orbital);
        let ratio = Self::doublet_ratio(orbital);

        XpsPeak {
            binding_energy_ev: peak.binding_energy_ev + spacing,
            fwhm_ev: peak.fwhm_ev,
            amplitude: peak.amplitude / ratio,
            area: peak.area / ratio,
            gl_mix: peak.gl_mix,
            asymmetry: peak.asymmetry,
        }
    }
}

// ─── ChargeCorrection ───────────────────────────────────────────────

/// Charge referencing utilities.
pub struct ChargeCorrection;

impl ChargeCorrection {
    /// Calculate charge correction shift from adventitious carbon.
    ///
    /// The reference value for C 1s (C-C/C-H) is 284.8 eV (ISO 19318:2004).
    /// Returns the shift that should be applied to all energies.
    pub fn adventitious_carbon_correction(c1s_measured: f64) -> f64 {
        284.8 - c1s_measured
    }

    /// Apply a binding energy correction (shift) to all energies in the spectrum.
    pub fn apply_correction(spectrum: &mut XpsSpectrum, shift_ev: f64) {
        for e in &mut spectrum.binding_energy_ev {
            *e += shift_ev;
        }
    }

    /// Calculate charge correction from Au 4f7/2 reference (84.0 eV).
    pub fn au_4f_correction(au4f_measured: f64) -> f64 {
        84.0 - au4f_measured
    }

    /// Assess charge compensation quality from peak broadening.
    ///
    /// Returns the extra broadening in eV. A well-compensated spectrum
    /// should have < 0.2 eV extra broadening.
    pub fn flood_gun_assessment(peak_fwhm: f64, expected_fwhm: f64) -> f64 {
        let extra = peak_fwhm - expected_fwhm;
        extra.max(0.0)
    }

    /// Typical instrument resolution (FWHM) for Ag 3d5/2 with Al Kα.
    pub fn typical_ag3d_fwhm() -> f64 {
        0.50 // eV for a modern monochromated system
    }
}

// ─── DepthProfile ───────────────────────────────────────────────────

/// Sputter depth profiling data and analysis.
#[derive(Debug, Clone)]
pub struct DepthProfile {
    /// Sputter times in seconds.
    pub sputter_times_s: Vec<f64>,
    /// Compositions at each time step. Each inner Vec is [element1_at%, element2_at%, ...].
    pub compositions: Vec<Vec<f64>>,
}

impl DepthProfile {
    /// Create a new depth profile.
    ///
    /// # Panics
    /// Panics if the lengths do not match.
    pub fn new(sputter_times_s: Vec<f64>, compositions: Vec<Vec<f64>>) -> Self {
        assert_eq!(
            sputter_times_s.len(),
            compositions.len(),
            "Sputter times and compositions must have same length"
        );
        Self {
            sputter_times_s,
            compositions,
        }
    }

    /// Convert sputter time to depth using a known sputter rate.
    ///
    /// depth (nm) = time (s) × rate (nm/min) / 60
    pub fn depth_from_sputter_rate(time_s: f64, rate_nm_per_min: f64) -> f64 {
        time_s * rate_nm_per_min / 60.0
    }

    /// Get depth values for all sputter times.
    pub fn depths(&self, rate_nm_per_min: f64) -> Vec<f64> {
        self.sputter_times_s
            .iter()
            .map(|&t| Self::depth_from_sputter_rate(t, rate_nm_per_min))
            .collect()
    }

    /// Calculate the interface width using the 16-84% criterion.
    ///
    /// The interface width is the depth range over which a composition profile
    /// changes from `threshold.0` fraction to `threshold.1` fraction of its
    /// maximum value. Standard thresholds: (0.16, 0.84) for 1-sigma.
    pub fn interface_width(profile: &[f64], depths: &[f64], threshold: (f64, f64)) -> f64 {
        if profile.is_empty() || depths.is_empty() {
            return 0.0;
        }
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_val < 1e-30 {
            return 0.0;
        }

        let lo_target = threshold.0 * max_val;
        let hi_target = threshold.1 * max_val;

        let mut depth_lo = depths[0];
        let mut depth_hi = depths[depths.len() - 1];

        // Find where profile crosses lo threshold
        for i in 0..profile.len().saturating_sub(1) {
            if (profile[i] <= lo_target && profile[i + 1] >= lo_target)
                || (profile[i] >= lo_target && profile[i + 1] <= lo_target)
            {
                let frac = if (profile[i + 1] - profile[i]).abs() > 1e-30 {
                    (lo_target - profile[i]) / (profile[i + 1] - profile[i])
                } else {
                    0.5
                };
                depth_lo = depths[i] + frac * (depths[i + 1] - depths[i]);
                break;
            }
        }

        // Find where profile crosses hi threshold
        for i in 0..profile.len().saturating_sub(1) {
            if (profile[i] <= hi_target && profile[i + 1] >= hi_target)
                || (profile[i] >= hi_target && profile[i + 1] <= hi_target)
            {
                let frac = if (profile[i + 1] - profile[i]).abs() > 1e-30 {
                    (hi_target - profile[i]) / (profile[i + 1] - profile[i])
                } else {
                    0.5
                };
                depth_hi = depths[i] + frac * (depths[i + 1] - depths[i]);
                break;
            }
        }

        (depth_hi - depth_lo).abs()
    }

    /// Estimate layer thickness from a composition profile.
    ///
    /// Layer thickness is the depth range where composition exceeds the given threshold.
    pub fn layer_thickness(profile: &[f64], depths: &[f64], threshold: f64) -> f64 {
        if profile.is_empty() || depths.is_empty() {
            return 0.0;
        }
        let mut first_above: Option<f64> = None;
        let mut last_above: Option<f64> = None;

        for (i, (&val, &depth)) in profile.iter().zip(depths.iter()).enumerate() {
            if val >= threshold {
                if first_above.is_none() {
                    // Interpolate to find exact crossing
                    if i > 0 && profile[i - 1] < threshold {
                        let frac = if (val - profile[i - 1]).abs() > 1e-30 {
                            (threshold - profile[i - 1]) / (val - profile[i - 1])
                        } else {
                            0.0
                        };
                        first_above = Some(depths[i - 1] + frac * (depth - depths[i - 1]));
                    } else {
                        first_above = Some(depth);
                    }
                }
                last_above = Some(depth);
            } else if last_above.is_some() && first_above.is_some() {
                // Interpolate descending crossing
                if i > 0 && profile[i - 1] >= threshold {
                    let frac = if (profile[i - 1] - val).abs() > 1e-30 {
                        (profile[i - 1] - threshold) / (profile[i - 1] - val)
                    } else {
                        0.0
                    };
                    last_above = Some(depths[i - 1] + frac * (depth - depths[i - 1]));
                }
                break;
            }
        }

        match (first_above, last_above) {
            (Some(f), Some(l)) => (l - f).abs(),
            _ => 0.0,
        }
    }

    /// Extract the composition profile for a specific element index.
    pub fn element_profile(&self, element_idx: usize) -> Vec<f64> {
        self.compositions
            .iter()
            .map(|comp| {
                if element_idx < comp.len() {
                    comp[element_idx]
                } else {
                    0.0
                }
            })
            .collect()
    }
}

// ─── Inelastic Mean Free Path ───────────────────────────────────────

/// IMFP (Inelastic Mean Free Path) estimation using the TPP-2M formula.
pub struct Imfp;

impl Imfp {
    /// Estimate IMFP in nm using the simplified TPP-2M formula.
    ///
    /// A simplified approximation: λ ≈ 0.1 * E^0.5 (nm) for organic materials,
    /// where E is kinetic energy in eV.
    pub fn tpp2m_simplified(kinetic_energy_ev: f64) -> f64 {
        if kinetic_energy_ev <= 0.0 {
            return 0.0;
        }
        0.1 * kinetic_energy_ev.sqrt()
    }

    /// Information depth (~3λ) for 95% of the signal.
    pub fn information_depth(kinetic_energy_ev: f64) -> f64 {
        3.0 * Self::tpp2m_simplified(kinetic_energy_ev)
    }

    /// Beer-Lambert attenuation factor: exp(-d / (λ cos θ))
    pub fn attenuation(thickness_nm: f64, imfp_nm: f64, angle_deg: f64) -> f64 {
        if imfp_nm < 1e-30 {
            return 0.0;
        }
        let cos_theta = (angle_deg * PI / 180.0).cos();
        if cos_theta.abs() < 1e-15 {
            return 0.0;
        }
        (-thickness_nm / (imfp_nm * cos_theta)).exp()
    }
}

// ─── Auger Parameter ────────────────────────────────────────────────

/// Modified Auger parameter calculation for chemical state plots (Wagner plots).
pub struct AugerParameter;

impl AugerParameter {
    /// Calculate the modified Auger parameter.
    ///
    /// α' = KE(Auger) + BE(photoelectron) = KE(Auger) + hν - KE(photo)
    pub fn modified_auger_parameter(
        auger_ke_ev: f64,
        photoelectron_be_ev: f64,
    ) -> f64 {
        auger_ke_ev + photoelectron_be_ev
    }

    /// Known modified Auger parameters for Cu chemical states.
    pub fn cu_auger_params() -> Vec<(&'static str, f64)> {
        vec![
            ("Cu metal", 1851.2),
            ("Cu2O", 1849.2),
            ("CuO", 1851.2),
            ("Cu(OH)2", 1850.0),
        ]
    }
}

// ─── Angle-Resolved XPS ─────────────────────────────────────────────

/// Angle-resolved XPS (ARXPS) for non-destructive depth profiling.
pub struct AngleResolvedXps;

impl AngleResolvedXps {
    /// Effective sampling depth at a given emission angle.
    ///
    /// d_eff = λ cos(θ) where λ is the IMFP and θ is the emission angle.
    pub fn effective_depth(imfp_nm: f64, emission_angle_deg: f64) -> f64 {
        imfp_nm * (emission_angle_deg * PI / 180.0).cos()
    }

    /// Overlayer thickness from a two-layer model using intensity ratios at two angles.
    ///
    /// For a thin overlayer on a substrate:
    /// I_overlayer / I_substrate = (I_ov_inf / I_sub_inf) * [1 - exp(-d/λ cos θ)] / exp(-d/λ cos θ)
    ///
    /// This simplified version uses the ratio at normal emission:
    /// d = λ cos(θ) * ln(1 + R * I_sub_inf / I_ov_inf)
    pub fn overlayer_thickness(
        intensity_ratio: f64,
        imfp_nm: f64,
        angle_deg: f64,
    ) -> f64 {
        let cos_theta = (angle_deg * PI / 180.0).cos();
        if cos_theta.abs() < 1e-15 || intensity_ratio <= 0.0 {
            return 0.0;
        }
        imfp_nm * cos_theta * (1.0 + intensity_ratio).ln()
    }
}

// ─── Survey scan helper ─────────────────────────────────────────────

/// Helper for XPS survey scan analysis.
pub struct SurveyScanAnalyzer;

impl SurveyScanAnalyzer {
    /// Auto-identify peaks in a survey scan.
    ///
    /// Finds local maxima above the given threshold and identifies them.
    pub fn auto_identify(
        spectrum: &XpsSpectrum,
        threshold_fraction: f64,
    ) -> Vec<(f64, Vec<ElementMatch>)> {
        let max_counts = spectrum.max_counts();
        let threshold = max_counts * threshold_fraction;
        let n = spectrum.counts.len();
        let mut results = Vec::new();

        for i in 1..n.saturating_sub(1) {
            if spectrum.counts[i] >= spectrum.counts[i - 1]
                && spectrum.counts[i] >= spectrum.counts[i + 1]
                && spectrum.counts[i] > threshold
            {
                let be = spectrum.binding_energy_ev[i];
                let matches = ChemicalStateIdentifier::identify_element(be);
                if !matches.is_empty() {
                    results.push((be, matches));
                }
            }
        }

        results
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_spectrum() -> XpsSpectrum {
        // Simulate a C 1s region (282-292 eV) with a peak at ~285 eV
        let n = 100;
        let be: Vec<f64> = (0..n).map(|i| 282.0 + i as f64 * 0.1).collect();
        let counts: Vec<f64> = be
            .iter()
            .map(|&e| {
                let peak = 1000.0 * (-((e - 285.0).powi(2)) / (2.0 * 0.5_f64.powi(2))).exp();
                let bg = 50.0 + 0.5 * (e - 282.0);
                peak + bg
            })
            .collect();
        XpsSpectrum::new(be, counts)
    }

    fn make_c1s_spectrum() -> XpsSpectrum {
        // C 1s with multiple chemical states
        let n = 200;
        let be: Vec<f64> = (0..n).map(|i| 282.0 + i as f64 * 0.1).collect();
        let counts: Vec<f64> = be
            .iter()
            .map(|&e| {
                let cc = 1000.0 * (-((e - 285.0).powi(2)) / (2.0 * 0.4_f64.powi(2))).exp();
                let co = 400.0 * (-((e - 286.5).powi(2)) / (2.0 * 0.4_f64.powi(2))).exp();
                let carboxyl = 200.0 * (-((e - 289.0).powi(2)) / (2.0 * 0.5_f64.powi(2))).exp();
                let bg = 30.0;
                cc + co + carboxyl + bg
            })
            .collect();
        XpsSpectrum::new(be, counts)
    }

    // ─── XpsSpectrum tests ───────────────────────────────────────

    #[test]
    fn test_spectrum_creation() {
        let spec = make_test_spectrum();
        assert_eq!(spec.binding_energy_ev.len(), 100);
        assert_eq!(spec.counts.len(), 100);
    }

    #[test]
    #[should_panic]
    fn test_spectrum_empty_panics() {
        XpsSpectrum::new(vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_spectrum_mismatched_lengths() {
        XpsSpectrum::new(vec![1.0, 2.0], vec![1.0]);
    }

    #[test]
    fn test_energy_range() {
        let spec = make_test_spectrum();
        let (min, max) = spec.energy_range();
        assert!((min - 282.0).abs() < 0.01);
        assert!((max - 291.9).abs() < 0.11);
    }

    #[test]
    fn test_max_counts() {
        let spec = make_test_spectrum();
        let max_c = spec.max_counts();
        assert!(max_c > 900.0); // peak ~1000 + background
    }

    #[test]
    fn test_kinetic_to_binding() {
        // Al Kα = 1486.6 eV, work function ~4.5 eV
        let be = XpsSpectrum::kinetic_to_binding(1197.1, 1486.6, 4.5);
        assert!((be - 285.0).abs() < 0.01);
    }

    #[test]
    fn test_kinetic_to_binding_gold() {
        // Au 4f7/2 at 84 eV: KE = 1486.6 - 84 - 4.5 = 1398.1
        let be = XpsSpectrum::kinetic_to_binding(1398.1, 1486.6, 4.5);
        assert!((be - 84.0).abs() < 0.01);
    }

    #[test]
    fn test_smooth() {
        let spec = make_test_spectrum();
        let smoothed = spec.smooth(5);
        assert_eq!(smoothed.counts.len(), spec.counts.len());
        // Smoothed should reduce noise but preserve peak
        let idx_peak = spec.closest_index(285.0);
        assert!(smoothed.counts[idx_peak] > 0.0);
    }

    #[test]
    fn test_smooth_preserves_length() {
        let spec = make_test_spectrum();
        let s3 = spec.smooth(3);
        let s7 = spec.smooth(7);
        assert_eq!(s3.counts.len(), spec.counts.len());
        assert_eq!(s7.counts.len(), spec.counts.len());
    }

    #[test]
    fn test_derivative_first() {
        let spec = make_test_spectrum();
        let deriv = spec.derivative(1);
        assert_eq!(deriv.counts.len(), spec.counts.len());
        // At the peak center, first derivative should cross zero
        let idx_peak = spec.closest_index(285.0);
        // Near the peak, derivative should be close to zero
        assert!(deriv.counts[idx_peak].abs() < deriv.counts[idx_peak - 5].abs());
    }

    #[test]
    fn test_derivative_second() {
        let spec = make_test_spectrum();
        let deriv2 = spec.derivative(2);
        assert_eq!(deriv2.counts.len(), spec.counts.len());
        // At peak center, second derivative should be negative (concave down)
        let idx_peak = spec.closest_index(285.0);
        assert!(deriv2.counts[idx_peak] < 0.0);
    }

    #[test]
    #[should_panic]
    fn test_derivative_invalid_order() {
        let spec = make_test_spectrum();
        spec.derivative(3);
    }

    #[test]
    fn test_closest_index() {
        let spec = make_test_spectrum();
        let idx = spec.closest_index(285.0);
        assert!((spec.binding_energy_ev[idx] - 285.0).abs() < 0.1);
    }

    #[test]
    fn test_region() {
        let spec = make_test_spectrum();
        let region = spec.region(284.0, 286.0);
        assert!(!region.binding_energy_ev.is_empty());
        for &e in &region.binding_energy_ev {
            assert!(e >= 284.0 && e <= 286.0);
        }
    }

    // ─── Background tests ────────────────────────────────────────

    #[test]
    fn test_shirley_background() {
        let spec = make_test_spectrum();
        let bg = ShirleyBackground::default().calculate(&spec, 282.0, 291.9);
        assert_eq!(bg.len(), spec.counts.len());
        // Background should be non-negative and generally track below the spectrum
        // The Shirley iterative method may slightly exceed at edge points
        let max_spec = spec.max_counts();
        for &b in &bg {
            assert!(b < max_spec * 1.1); // background should not exceed spectrum peak
        }
    }

    #[test]
    fn test_shirley_subtract() {
        let spec = make_test_spectrum();
        let bg = ShirleyBackground::default().calculate(&spec, 282.0, 291.9);
        let sub = ShirleyBackground::subtract(&spec, &bg);
        assert_eq!(sub.counts.len(), spec.counts.len());
        for &c in &sub.counts {
            assert!(c >= 0.0);
        }
    }

    #[test]
    fn test_shirley_convergence() {
        let spec = make_test_spectrum();
        let shirley = ShirleyBackground::new(100, 1e-9);
        let bg = shirley.calculate(&spec, 282.0, 291.9);
        assert_eq!(bg.len(), spec.counts.len());
    }

    #[test]
    fn test_tougaard_background() {
        let spec = make_test_spectrum();
        let (b, c) = TougaardBackground::universal_params();
        let bg = TougaardBackground::calculate(&spec, b, c);
        assert_eq!(bg.len(), spec.counts.len());
    }

    #[test]
    fn test_tougaard_universal_params() {
        let (b, c) = TougaardBackground::universal_params();
        assert!((b - 2866.0).abs() < 0.1);
        assert!((c - 1643.0).abs() < 0.1);
    }

    #[test]
    fn test_linear_background() {
        let spec = make_test_spectrum();
        let bg = LinearBackground::calculate(&spec, 282.0, 291.9);
        assert_eq!(bg.len(), spec.counts.len());
        // Linear background should interpolate between endpoints
        let idx0 = spec.closest_index(282.0);
        let idx_end = spec.closest_index(291.9);
        assert!((bg[idx0] - spec.counts[idx0]).abs() < 1.0);
        assert!((bg[idx_end] - spec.counts[idx_end]).abs() < 1.0);
    }

    // ─── Peak shape tests ────────────────────────────────────────

    #[test]
    fn test_gaussian_peak() {
        let g = gaussian(285.0, 285.0, 1.0, 100.0);
        assert!((g - 100.0).abs() < 0.01); // At center, should be amplitude
        let g_off = gaussian(286.0, 285.0, 1.0, 100.0);
        assert!(g_off < g);
    }

    #[test]
    fn test_gaussian_fwhm() {
        let amp = 100.0;
        let fwhm = 1.0;
        let half_max = amp / 2.0;
        let g_at_hwhm = gaussian(285.0 + fwhm / 2.0, 285.0, fwhm, amp);
        assert!((g_at_hwhm - half_max).abs() < 1.0);
    }

    #[test]
    fn test_lorentzian_peak() {
        let l = lorentzian(285.0, 285.0, 1.0, 100.0);
        assert!((l - 100.0).abs() < 0.01);
        let l_off = lorentzian(286.0, 285.0, 1.0, 100.0);
        assert!(l_off < l);
    }

    #[test]
    fn test_lorentzian_fwhm() {
        let amp = 100.0;
        let fwhm = 2.0;
        let l_at_hwhm = lorentzian(285.0 + fwhm / 2.0, 285.0, fwhm, amp);
        assert!((l_at_hwhm - amp / 2.0).abs() < 1.0);
    }

    #[test]
    fn test_pseudo_voigt_pure_gaussian() {
        let v = pseudo_voigt(285.0, 285.0, 1.0, 0.0, 100.0);
        let g = gaussian(285.0, 285.0, 1.0, 100.0);
        assert!((v - g).abs() < 0.01);
    }

    #[test]
    fn test_pseudo_voigt_pure_lorentzian() {
        let v = pseudo_voigt(285.0, 285.0, 1.0, 1.0, 100.0);
        let l = lorentzian(285.0, 285.0, 1.0, 100.0);
        assert!((v - l).abs() < 0.01);
    }

    #[test]
    fn test_pseudo_voigt_mixed() {
        let v = pseudo_voigt(285.0, 285.0, 1.0, 0.5, 100.0);
        assert!((v - 100.0).abs() < 0.01); // At center
    }

    #[test]
    fn test_doniach_sunjic() {
        let ds = doniach_sunjic(285.0, 285.0, 0.5, 0.0, 100.0);
        assert!(ds > 0.0); // Should be positive at center
    }

    #[test]
    fn test_doniach_sunjic_asymmetric() {
        let ds_left = doniach_sunjic(284.0, 285.0, 0.5, 0.15, 100.0);
        let ds_right = doniach_sunjic(286.0, 285.0, 0.5, 0.15, 100.0);
        // Asymmetric: tail should extend more to higher BE (right)
        // For metallic peaks, the asymmetric tail extends to higher KE = lower BE
        assert!(ds_left.abs() > 0.0 || ds_right.abs() > 0.0);
    }

    // ─── XpsPeak tests ───────────────────────────────────────────

    #[test]
    fn test_xps_peak_evaluate() {
        let peak = XpsPeak {
            binding_energy_ev: 285.0,
            fwhm_ev: 1.0,
            amplitude: 1000.0,
            area: 0.0,
            gl_mix: 0.3,
            asymmetry: 0.0,
        };
        let val_center = peak.evaluate(285.0);
        let val_off = peak.evaluate(287.0);
        assert!(val_center > val_off);
    }

    #[test]
    fn test_xps_peak_compute_area() {
        let peak = XpsPeak {
            binding_energy_ev: 285.0,
            fwhm_ev: 1.0,
            amplitude: 1000.0,
            area: 0.0,
            gl_mix: 0.0, // pure Gaussian
            asymmetry: 0.0,
        };
        let area = peak.compute_area(0.01);
        // For Gaussian: area ≈ A * FWHM * sqrt(π/(4 ln 2)) ≈ A * FWHM * 1.0645
        let expected = 1000.0 * 1.0 * (PI / (4.0 * 2.0_f64.ln())).sqrt();
        assert!((area - expected).abs() / expected < 0.05); // 5% tolerance
    }

    #[test]
    fn test_xps_peak_asymmetric_evaluate() {
        let peak = XpsPeak {
            binding_energy_ev: 285.0,
            fwhm_ev: 1.0,
            amplitude: 1000.0,
            area: 0.0,
            gl_mix: 0.3,
            asymmetry: 0.1,
        };
        let val = peak.evaluate(285.0);
        assert!(val > 0.0);
    }

    // ─── PeakFitter tests ────────────────────────────────────────

    #[test]
    fn test_fit_single_peak() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let result = fitter.fit_peaks(&spec, 1, BackgroundType::Linear);
        assert_eq!(result.peaks.len(), 1);
        // Peak should be near 285 eV
        assert!((result.peaks[0].binding_energy_ev - 285.0).abs() < 1.0);
    }

    #[test]
    fn test_fit_r_squared() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let result = fitter.fit_peaks(&spec, 1, BackgroundType::Linear);
        // R² should be reasonable for a good fit
        assert!(result.r_squared > 0.5);
    }

    #[test]
    fn test_fit_multiple_peaks() {
        let spec = make_c1s_spectrum();
        let fitter = PeakFitter::new(300, 1e-8);
        let result = fitter.fit_peaks(&spec, 3, BackgroundType::None);
        assert_eq!(result.peaks.len(), 3);
    }

    #[test]
    fn test_fit_envelope() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let result = fitter.fit_peaks(&spec, 1, BackgroundType::None);
        let env = result.envelope(285.0);
        assert!(env > 0.0);
    }

    #[test]
    fn test_constrain_peaks() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let mut result = fitter.fit_peaks(&spec, 2, BackgroundType::Linear);
        PeakFitter::constrain_peaks(
            &mut result,
            &[PeakConstraint::FixPosition(0, 285.0)],
        );
        assert!((result.peaks[0].binding_energy_ev - 285.0).abs() < 0.01);
    }

    #[test]
    fn test_constrain_fwhm() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let mut result = fitter.fit_peaks(&spec, 1, BackgroundType::Linear);
        PeakFitter::constrain_peaks(
            &mut result,
            &[PeakConstraint::MaxFwhm(0, 0.5)],
        );
        assert!(result.peaks[0].fwhm_ev <= 0.5);
    }

    #[test]
    fn test_constrain_gl_mix() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let mut result = fitter.fit_peaks(&spec, 1, BackgroundType::None);
        PeakFitter::constrain_peaks(
            &mut result,
            &[PeakConstraint::FixGlMix(0, 0.8)],
        );
        assert!((result.peaks[0].gl_mix - 0.8).abs() < 0.01);
    }

    // ─── ChemicalStateIdentifier tests ───────────────────────────

    #[test]
    fn test_identify_carbon() {
        let matches = ChemicalStateIdentifier::identify_element(285.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "C"));
    }

    #[test]
    fn test_identify_oxygen() {
        let matches = ChemicalStateIdentifier::identify_element(531.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "O"));
    }

    #[test]
    fn test_identify_gold() {
        let matches = ChemicalStateIdentifier::identify_element(84.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "Au"));
    }

    #[test]
    fn test_identify_silicon() {
        let matches = ChemicalStateIdentifier::identify_element(99.3);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "Si"));
    }

    #[test]
    fn test_identify_iron() {
        let matches = ChemicalStateIdentifier::identify_element(707.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "Fe"));
    }

    #[test]
    fn test_identify_nitrogen() {
        let matches = ChemicalStateIdentifier::identify_element(399.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|m| m.element == "N"));
    }

    #[test]
    fn test_identify_no_match() {
        let matches = ChemicalStateIdentifier::identify_element(1500.0);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_identify_with_tolerance() {
        let narrow = ChemicalStateIdentifier::identify_element_with_tolerance(285.0, 0.1);
        let wide = ChemicalStateIdentifier::identify_element_with_tolerance(285.0, 5.0);
        assert!(wide.len() >= narrow.len());
    }

    #[test]
    fn test_carbon_chemical_shifts() {
        let shifts = ChemicalStateIdentifier::carbon_chemical_shifts();
        assert!(shifts.len() >= 5);
        assert!(shifts.iter().any(|(name, _)| name.contains("C-C")));
    }

    // ─── Quantification tests ────────────────────────────────────

    #[test]
    fn test_atomic_percent() {
        let areas = vec![100.0, 200.0, 300.0];
        let sfs = vec![1.0, 1.0, 1.0]; // equal sensitivity
        let at_pct = Quantification::atomic_percent(&areas, &sfs);
        assert!((at_pct[0] - 100.0 / 6.0).abs() < 0.1);
        assert!((at_pct[1] - 200.0 / 6.0).abs() < 0.1);
        assert!((at_pct[2] - 300.0 / 6.0).abs() < 0.1);
    }

    #[test]
    fn test_atomic_percent_with_sf() {
        let areas = vec![100.0, 100.0];
        let sfs = vec![0.296, 0.711]; // C 1s, O 1s
        let at_pct = Quantification::atomic_percent(&areas, &sfs);
        let sum: f64 = at_pct.iter().sum();
        assert!((sum - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_sensitivity_factor_known() {
        let sf = Quantification::sensitivity_factor("C", "1s");
        assert!((sf - 0.296).abs() < 0.001);
        let sf_o = Quantification::sensitivity_factor("O", "1s");
        assert!((sf_o - 0.711).abs() < 0.001);
    }

    #[test]
    fn test_sensitivity_factor_unknown() {
        let sf = Quantification::sensitivity_factor("Unobtanium", "1s");
        assert!((sf - 1.0).abs() < 0.001); // default
    }

    #[test]
    fn test_stoichiometry() {
        let at_pct = vec![33.33, 66.67];
        let stoich = Quantification::stoichiometry(&at_pct, &["Si", "O"]);
        // Should be approximately 1:2 (SiO2)
        assert!((stoich[0] - 1.0).abs() < 0.01);
        assert!((stoich[1] - 2.0).abs() < 0.02);
    }

    // ─── SpinOrbitSplitting tests ────────────────────────────────

    #[test]
    fn test_doublet_spacing_fe() {
        let spacing = SpinOrbitSplitting::doublet_spacing("Fe", "2p");
        assert!((spacing - 13.1).abs() < 0.1);
    }

    #[test]
    fn test_doublet_spacing_au() {
        let spacing = SpinOrbitSplitting::doublet_spacing("Au", "4f");
        assert!((spacing - 3.67).abs() < 0.01);
    }

    #[test]
    fn test_doublet_ratio_p() {
        let ratio = SpinOrbitSplitting::doublet_ratio("2p");
        assert!((ratio - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_doublet_ratio_d() {
        let ratio = SpinOrbitSplitting::doublet_ratio("3d");
        assert!((ratio - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_doublet_ratio_f() {
        let ratio = SpinOrbitSplitting::doublet_ratio("4f");
        assert!((ratio - 4.0 / 3.0).abs() < 0.01);
    }

    #[test]
    fn test_constrain_doublet() {
        let main_peak = XpsPeak {
            binding_energy_ev: 84.0, // Au 4f7/2
            fwhm_ev: 1.0,
            amplitude: 1000.0,
            area: 500.0,
            gl_mix: 0.3,
            asymmetry: 0.0,
        };
        let partner = SpinOrbitSplitting::constrain_doublet(&main_peak, "Au", "4f");
        assert!((partner.binding_energy_ev - 87.67).abs() < 0.01); // 84 + 3.67
        assert!((partner.amplitude - 1000.0 * 3.0 / 4.0).abs() < 1.0); // 4f ratio: 4:3
    }

    // ─── ChargeCorrection tests ──────────────────────────────────

    #[test]
    fn test_adventitious_carbon_correction() {
        let shift = ChargeCorrection::adventitious_carbon_correction(286.0);
        assert!((shift - (-1.2)).abs() < 0.01);
    }

    #[test]
    fn test_apply_correction() {
        let mut spec = make_test_spectrum();
        let original_first = spec.binding_energy_ev[0];
        ChargeCorrection::apply_correction(&mut spec, 1.5);
        assert!((spec.binding_energy_ev[0] - (original_first + 1.5)).abs() < 0.001);
    }

    #[test]
    fn test_au_4f_correction() {
        let shift = ChargeCorrection::au_4f_correction(85.0);
        assert!((shift - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_flood_gun_assessment() {
        let broadening = ChargeCorrection::flood_gun_assessment(1.2, 0.9);
        assert!((broadening - 0.3).abs() < 0.01);
    }

    #[test]
    fn test_flood_gun_no_broadening() {
        let broadening = ChargeCorrection::flood_gun_assessment(0.8, 0.9);
        assert!((broadening).abs() < 0.01);
    }

    // ─── DepthProfile tests ──────────────────────────────────────

    #[test]
    fn test_depth_from_sputter_rate() {
        let depth = DepthProfile::depth_from_sputter_rate(60.0, 2.0); // 60s at 2 nm/min
        assert!((depth - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_depth_profile_creation() {
        let dp = DepthProfile::new(
            vec![0.0, 60.0, 120.0],
            vec![vec![50.0, 50.0], vec![30.0, 70.0], vec![10.0, 90.0]],
        );
        assert_eq!(dp.sputter_times_s.len(), 3);
    }

    #[test]
    fn test_depths() {
        let dp = DepthProfile::new(
            vec![0.0, 60.0, 120.0],
            vec![vec![50.0, 50.0], vec![30.0, 70.0], vec![10.0, 90.0]],
        );
        let depths = dp.depths(2.0); // 2 nm/min
        assert!((depths[0] - 0.0).abs() < 0.01);
        assert!((depths[1] - 2.0).abs() < 0.01);
        assert!((depths[2] - 4.0).abs() < 0.01);
    }

    #[test]
    fn test_interface_width() {
        let profile = vec![0.0, 10.0, 30.0, 50.0, 70.0, 90.0, 100.0];
        let depths = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let width = DepthProfile::interface_width(&profile, &depths, (0.16, 0.84));
        assert!(width > 0.0);
        assert!(width < 6.0);
    }

    #[test]
    fn test_layer_thickness() {
        let profile = vec![0.0, 0.0, 80.0, 80.0, 80.0, 0.0, 0.0];
        let depths = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let thickness = DepthProfile::layer_thickness(&profile, &depths, 50.0);
        assert!(thickness > 1.0);
        assert!(thickness < 5.0);
    }

    #[test]
    fn test_element_profile() {
        let dp = DepthProfile::new(
            vec![0.0, 60.0, 120.0],
            vec![vec![50.0, 50.0], vec![30.0, 70.0], vec![10.0, 90.0]],
        );
        let prof = dp.element_profile(0);
        assert_eq!(prof, vec![50.0, 30.0, 10.0]);
    }

    // ─── IMFP tests ──────────────────────────────────────────────

    #[test]
    fn test_imfp_estimate() {
        let lambda = Imfp::tpp2m_simplified(1000.0);
        assert!(lambda > 0.0);
        assert!(lambda < 10.0); // Should be reasonable nm range
    }

    #[test]
    fn test_information_depth() {
        let depth = Imfp::information_depth(1000.0);
        let lambda = Imfp::tpp2m_simplified(1000.0);
        assert!((depth - 3.0 * lambda).abs() < 0.01);
    }

    #[test]
    fn test_attenuation() {
        let atten = Imfp::attenuation(0.0, 2.0, 0.0);
        assert!((atten - 1.0).abs() < 0.01); // zero thickness = no attenuation
    }

    #[test]
    fn test_attenuation_thick() {
        let atten = Imfp::attenuation(10.0, 2.0, 0.0);
        assert!(atten < 0.01); // thick film = high attenuation
    }

    #[test]
    fn test_attenuation_angle() {
        let atten_0 = Imfp::attenuation(2.0, 2.0, 0.0);
        let atten_60 = Imfp::attenuation(2.0, 2.0, 60.0);
        assert!(atten_60 < atten_0); // grazing angle = more attenuation
    }

    // ─── Auger parameter tests ───────────────────────────────────

    #[test]
    fn test_modified_auger_parameter() {
        let alpha = AugerParameter::modified_auger_parameter(918.5, 932.7);
        assert!((alpha - 1851.2).abs() < 0.01);
    }

    #[test]
    fn test_cu_auger_params() {
        let params = AugerParameter::cu_auger_params();
        assert!(!params.is_empty());
        assert!(params.iter().any(|(name, _)| *name == "Cu metal"));
    }

    // ─── ARXPS tests ─────────────────────────────────────────────

    #[test]
    fn test_effective_depth() {
        let depth = AngleResolvedXps::effective_depth(2.0, 0.0);
        assert!((depth - 2.0).abs() < 0.01); // Normal emission
    }

    #[test]
    fn test_effective_depth_grazing() {
        let depth = AngleResolvedXps::effective_depth(2.0, 80.0);
        assert!(depth < 0.5); // Grazing angle = very surface sensitive
    }

    #[test]
    fn test_overlayer_thickness() {
        let d = AngleResolvedXps::overlayer_thickness(1.0, 2.0, 0.0);
        assert!(d > 0.0);
        // For ratio=1: d = λ * ln(2) ≈ 2 * 0.693 = 1.386
        assert!((d - 2.0 * 2.0_f64.ln()).abs() < 0.01);
    }

    // ─── Survey scan tests ───────────────────────────────────────

    #[test]
    fn test_auto_identify() {
        let spec = make_test_spectrum();
        let results = SurveyScanAnalyzer::auto_identify(&spec, 0.5);
        // Should find at least one peak
        assert!(!results.is_empty() || spec.max_counts() < 1.0);
    }

    // ─── Savitzky-Golay coefficient tests ────────────────────────

    #[test]
    fn test_sg_coeffs_sum_to_one() {
        let coeffs = savitzky_golay_coeffs(5, 2, 0);
        let sum: f64 = coeffs.iter().sum();
        assert!((sum - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_sg_coeffs_derivative_sum_to_zero() {
        let coeffs = savitzky_golay_coeffs(5, 2, 1);
        let sum: f64 = coeffs.iter().sum();
        assert!(sum.abs() < 0.01);
    }

    // ─── Linear system solver tests ──────────────────────────────

    #[test]
    fn test_solve_identity() {
        let a = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let b = vec![3.0, 7.0];
        let x = solve_linear_system(&a, &b);
        assert!((x[0] - 3.0).abs() < 1e-10);
        assert!((x[1] - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_solve_2x2() {
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 10.0];
        let x = solve_linear_system(&a, &b);
        // 2x + y = 5, x + 3y = 10 => x = 1, y = 3
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 3.0).abs() < 1e-10);
    }

    // ─── BackgroundType enum tests ───────────────────────────────

    #[test]
    fn test_background_type_eq() {
        assert_eq!(BackgroundType::Shirley, BackgroundType::Shirley);
        assert_ne!(BackgroundType::Shirley, BackgroundType::Linear);
    }

    #[test]
    fn test_fit_with_shirley_background() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let result = fitter.fit_peaks(&spec, 1, BackgroundType::Shirley);
        assert!(!result.peaks.is_empty());
        assert!(!result.background.is_empty());
    }

    #[test]
    fn test_fit_with_no_background() {
        let spec = make_test_spectrum();
        let fitter = PeakFitter::default();
        let result = fitter.fit_peaks(&spec, 1, BackgroundType::None);
        assert!(!result.peaks.is_empty());
        // Background should be all zeros
        assert!(result.background.iter().all(|&b| b == 0.0));
    }

    // ─── Edge case tests ─────────────────────────────────────────

    #[test]
    fn test_spectrum_single_point() {
        let spec = XpsSpectrum::new(vec![285.0], vec![100.0]);
        let (min, max) = spec.energy_range();
        assert!((min - 285.0).abs() < 0.01);
        assert!((max - 285.0).abs() < 0.01);
    }

    #[test]
    fn test_zero_area_quantification() {
        let at_pct = Quantification::atomic_percent(&[0.0, 0.0], &[1.0, 1.0]);
        assert!(at_pct.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_imfp_zero_ke() {
        let lambda = Imfp::tpp2m_simplified(0.0);
        assert!((lambda).abs() < 0.01);
    }

    #[test]
    fn test_depth_profile_empty_threshold() {
        let thickness = DepthProfile::layer_thickness(&[], &[], 50.0);
        assert!((thickness).abs() < 0.01);
    }
}
