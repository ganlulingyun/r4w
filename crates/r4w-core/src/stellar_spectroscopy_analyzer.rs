//! # Stellar Spectroscopy Analyzer
//!
//! Signal processing for stellar spectroscopy: spectral line detection, radial velocity
//! measurement via cross-correlation, equivalent width computation, and stellar
//! classification from spectral features.
//!
//! ## Overview
//!
//! Stellar spectroscopy is the primary technique for determining the physical properties
//! of stars. By analyzing absorption and emission lines in a star's spectrum, astronomers
//! can determine:
//!
//! - **Radial velocity** via Doppler shift of known spectral lines
//! - **Effective temperature** from Wien's displacement law or spectral classification
//! - **Chemical composition** from absorption line identification and equivalent widths
//! - **Luminosity class** from line profile widths (pressure broadening)
//!
//! ## Key Algorithms
//!
//! - **Cross-Correlation Function (CCF)**: Measures radial velocity by cross-correlating
//!   an observed spectrum against a template at various velocity shifts. The peak of the
//!   CCF gives the best-fit radial velocity.
//!
//! - **Equivalent Width**: Measures absorption line strength as the width of a rectangle
//!   with the same area as the line profile, relative to the continuum level.
//!
//! - **Gaussian Line Fitting**: Fits absorption/emission lines with Gaussian profiles
//!   to determine center wavelength, depth, and width.
//!
//! - **Planck/Blackbody Function**: Models the continuum emission of stars as thermal
//!   radiators, enabling temperature estimation from spectral shape.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stellar_spectroscopy_analyzer::*;
//!
//! let config = SpectroscopyConfig {
//!     wavelength_min_nm: 380.0,
//!     wavelength_max_nm: 700.0,
//!     spectral_resolution: 10000.0,
//!     snr: 100.0,
//!     radial_velocity_range_km_s: 500.0,
//! };
//!
//! let analyzer = StellarSpectroscopyAnalyzer::new(config);
//!
//! // Compute Doppler shift for a star moving at +100 km/s
//! let shifted = analyzer.doppler_shift(656.28, 100.0);
//! assert!((shifted - 656.499).abs() < 0.01);
//!
//! // Compute radial velocity from observed wavelength
//! let rv = analyzer.radial_velocity(656.28, 656.499);
//! assert!((rv - 100.0).abs() < 1.0);
//! ```

// ─── Physical Constants ───────────────────────────────────────────────────────

/// Speed of light in km/s.
pub const SPEED_OF_LIGHT_KM_S: f64 = 299_792.458;

/// Planck constant in J·s.
pub const PLANCK_CONSTANT: f64 = 6.626_070_15e-34;

/// Boltzmann constant in J/K.
pub const BOLTZMANN_CONSTANT: f64 = 1.380_649e-23;

/// Wien displacement constant in nm·K.
pub const WIEN_CONSTANT: f64 = 2.897_771_955e6;

/// Speed of light in m/s (used internally for Planck function).
const SPEED_OF_LIGHT_M_S: f64 = 2.997_924_58e8;

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the spectroscopy analyzer.
#[derive(Debug, Clone)]
pub struct SpectroscopyConfig {
    /// Minimum wavelength of the spectrograph coverage in nm.
    pub wavelength_min_nm: f64,
    /// Maximum wavelength of the spectrograph coverage in nm.
    pub wavelength_max_nm: f64,
    /// Spectral resolving power R = lambda / delta_lambda.
    pub spectral_resolution: f64,
    /// Signal-to-noise ratio of the spectrum.
    pub snr: f64,
    /// Maximum radial velocity search range in km/s.
    pub radial_velocity_range_km_s: f64,
}

// ─── Spectral Line ────────────────────────────────────────────────────────────

/// A known spectral absorption or emission line.
#[derive(Debug, Clone)]
pub struct SpectralLine {
    /// Rest-frame wavelength in nm.
    pub wavelength_nm: f64,
    /// Element or transition name (e.g., "H-alpha", "Ca II K").
    pub element: String,
    /// Ionization state: 0 = neutral (I), 1 = singly ionized (II), etc.
    pub ionization_state: u8,
}

// ─── Harvard Spectral Classification ──────────────────────────────────────────

/// Harvard spectral classification for main-sequence stars (O through M).
///
/// The sequence runs from hottest (O, ~30,000+ K) to coolest (M, ~2,400-3,700 K).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StellarClassifier {
    /// O-type: T_eff > 30,000 K. Ionized He lines dominant.
    O,
    /// B-type: T_eff ~10,000-30,000 K. Neutral He lines, weak H.
    B,
    /// A-type: T_eff ~7,500-10,000 K. Strongest hydrogen Balmer lines.
    A,
    /// F-type: T_eff ~6,000-7,500 K. Moderate H, Ca II appearing.
    F,
    /// G-type: T_eff ~5,200-6,000 K (Solar type). Ca II strong, Fe lines.
    G,
    /// K-type: T_eff ~3,700-5,200 K. Strong metals, weak H.
    K,
    /// M-type: T_eff ~2,400-3,700 K. TiO molecular bands dominant.
    M,
}

impl core::fmt::Display for StellarClassifier {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = match self {
            StellarClassifier::O => "O",
            StellarClassifier::B => "B",
            StellarClassifier::A => "A",
            StellarClassifier::F => "F",
            StellarClassifier::G => "G",
            StellarClassifier::K => "K",
            StellarClassifier::M => "M",
        };
        write!(f, "{}", s)
    }
}

// ─── Stellar Spectroscopy Analyzer ────────────────────────────────────────────

/// Core analyzer for stellar spectroscopy signal processing.
///
/// Provides methods for Doppler shift computation, radial velocity measurement,
/// cross-correlation velocity determination, equivalent width, continuum
/// normalization, and spectral line detection.
pub struct StellarSpectroscopyAnalyzer {
    config: SpectroscopyConfig,
}

impl StellarSpectroscopyAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: SpectroscopyConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the analyzer configuration.
    pub fn config(&self) -> &SpectroscopyConfig {
        &self.config
    }

    /// Compute the observed wavelength given a rest wavelength and radial velocity.
    ///
    /// Uses the non-relativistic Doppler formula:
    ///
    ///   lambda_obs = lambda_rest * (1 + v / c)
    ///
    /// where v is positive for recession (redshift) and negative for approach (blueshift).
    ///
    /// # Arguments
    /// * `rest_wavelength_nm` - Laboratory rest wavelength in nm.
    /// * `velocity_km_s` - Radial velocity in km/s (positive = receding).
    ///
    /// # Returns
    /// Observed wavelength in nm.
    pub fn doppler_shift(&self, rest_wavelength_nm: f64, velocity_km_s: f64) -> f64 {
        rest_wavelength_nm * (1.0 + velocity_km_s / SPEED_OF_LIGHT_KM_S)
    }

    /// Compute radial velocity from rest and observed wavelengths.
    ///
    ///   v = c * (lambda_obs / lambda_rest - 1)
    ///
    /// # Arguments
    /// * `rest_wavelength_nm` - Laboratory rest wavelength in nm.
    /// * `observed_wavelength_nm` - Observed wavelength in nm.
    ///
    /// # Returns
    /// Radial velocity in km/s (positive = receding).
    pub fn radial_velocity(&self, rest_wavelength_nm: f64, observed_wavelength_nm: f64) -> f64 {
        SPEED_OF_LIGHT_KM_S * (observed_wavelength_nm / rest_wavelength_nm - 1.0)
    }

    /// Compute the Cross-Correlation Function (CCF) between an observed spectrum
    /// and a template spectrum at a series of trial radial velocities.
    ///
    /// For each trial velocity, the template is Doppler-shifted and cross-correlated
    /// with the observation. The resulting CCF peak indicates the best-fit velocity.
    ///
    /// Both `spectrum` and `template` are slices of (wavelength_nm, flux) pairs,
    /// assumed to be sorted by wavelength.
    ///
    /// # Arguments
    /// * `spectrum` - Observed spectrum as (wavelength_nm, flux) pairs.
    /// * `template` - Template spectrum as (wavelength_nm, flux) pairs.
    /// * `velocity_steps` - Trial velocities in km/s to evaluate.
    ///
    /// # Returns
    /// Vec of (velocity_km_s, ccf_value) pairs.
    pub fn cross_correlate_rv(
        &self,
        spectrum: &[(f64, f64)],
        template: &[(f64, f64)],
        velocity_steps: &[f64],
    ) -> Vec<(f64, f64)> {
        let mut ccf = Vec::with_capacity(velocity_steps.len());

        for &v in velocity_steps {
            let shift_factor = 1.0 + v / SPEED_OF_LIGHT_KM_S;
            let mut sum = 0.0;

            // For each template point, shift its wavelength and interpolate into the
            // observed spectrum to compute the correlation.
            for &(t_wl, t_flux) in template {
                let shifted_wl = t_wl * shift_factor;
                // Linear interpolation in the observed spectrum
                if let Some(obs_flux) = linear_interp(spectrum, shifted_wl) {
                    sum += obs_flux * t_flux;
                }
            }
            ccf.push((v, sum));
        }

        ccf
    }

    /// Find the velocity and correlation height at the peak of a CCF.
    ///
    /// Uses parabolic interpolation around the discrete maximum for sub-step
    /// velocity precision.
    ///
    /// # Arguments
    /// * `ccf` - Cross-correlation function as (velocity_km_s, ccf_value) pairs.
    ///
    /// # Returns
    /// (velocity_km_s, peak_height) at the CCF maximum.
    pub fn find_rv_peak(&self, ccf: &[(f64, f64)]) -> (f64, f64) {
        if ccf.is_empty() {
            return (0.0, 0.0);
        }

        // Find the index of the maximum CCF value.
        let mut max_idx = 0;
        let mut max_val = ccf[0].1;
        for (i, &(_, val)) in ccf.iter().enumerate() {
            if val > max_val {
                max_val = val;
                max_idx = i;
            }
        }

        // Parabolic interpolation if we are not at an edge.
        if max_idx > 0 && max_idx < ccf.len() - 1 {
            let v_m1 = ccf[max_idx - 1].0;
            let v_0 = ccf[max_idx].0;
            let v_p1 = ccf[max_idx + 1].0;
            let y_m1 = ccf[max_idx - 1].1;
            let y_0 = ccf[max_idx].1;
            let y_p1 = ccf[max_idx + 1].1;

            let denom = 2.0 * (2.0 * y_0 - y_m1 - y_p1);
            if denom.abs() > 1e-30 {
                let delta = (y_m1 - y_p1) / denom;
                let step = v_p1 - v_0;
                let refined_v = v_0 + delta * step;
                // Evaluate parabolic peak height.
                let _ = v_m1; // used in denominator above
                let peak_h = y_0 + 0.25 * (y_m1 - y_p1) * delta;
                return (refined_v, peak_h);
            }
        }

        (ccf[max_idx].0, max_val)
    }

    /// Compute the equivalent width of an absorption line.
    ///
    /// The equivalent width W is defined as:
    ///
    ///   W = integral[ (1 - F(lambda)/F_c) d_lambda ]
    ///
    /// integrated over a window centred on the line, where F_c is the continuum level.
    /// The result is in nm and is positive for absorption lines.
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs, sorted by wavelength.
    /// * `line_center_nm` - Central wavelength of the line.
    /// * `continuum_level` - Estimated continuum flux level.
    /// * `window_nm` - Half-width of the integration window in nm.
    ///
    /// # Returns
    /// Equivalent width in nm.
    pub fn equivalent_width(
        &self,
        spectrum: &[(f64, f64)],
        line_center_nm: f64,
        continuum_level: f64,
        window_nm: f64,
    ) -> f64 {
        if continuum_level <= 0.0 || spectrum.len() < 2 {
            return 0.0;
        }

        let wl_min = line_center_nm - window_nm;
        let wl_max = line_center_nm + window_nm;

        // Trapezoidal integration over the window.
        let mut ew = 0.0;
        for i in 0..spectrum.len() - 1 {
            let (w0, f0) = spectrum[i];
            let (w1, f1) = spectrum[i + 1];

            // Skip segments entirely outside the window.
            if w1 < wl_min || w0 > wl_max {
                continue;
            }

            // Clamp to window boundaries.
            let a = w0.max(wl_min);
            let b = w1.min(wl_max);
            if b <= a {
                continue;
            }

            // Interpolate flux at clamped boundaries if needed.
            let fa = if a == w0 {
                f0
            } else {
                let t = (a - w0) / (w1 - w0);
                f0 + t * (f1 - f0)
            };
            let fb = if b == w1 {
                f1
            } else {
                let t = (b - w0) / (w1 - w0);
                f0 + t * (f1 - f0)
            };

            let integrand_a = 1.0 - fa / continuum_level;
            let integrand_b = 1.0 - fb / continuum_level;

            ew += 0.5 * (integrand_a + integrand_b) * (b - a);
        }

        ew
    }

    /// Normalize the spectrum by fitting and dividing by a polynomial continuum.
    ///
    /// After normalization the continuum level should be approximately 1.0.
    ///
    /// # Arguments
    /// * `spectrum` - Mutable slice of (wavelength_nm, flux) pairs.
    /// * `order` - Polynomial order for the fit (e.g., 2 for quadratic).
    pub fn continuum_normalize(&self, spectrum: &mut [(f64, f64)], order: usize) {
        if spectrum.len() < order + 1 {
            return;
        }

        let n = spectrum.len();
        let order = order.min(n - 1);

        // Normalise wavelengths to [-1, 1] for numerical stability.
        let wl_min = spectrum[0].0;
        let wl_max = spectrum[n - 1].0;
        let wl_range = wl_max - wl_min;
        if wl_range <= 0.0 {
            return;
        }

        let norm_wl: Vec<f64> = spectrum
            .iter()
            .map(|(w, _)| 2.0 * (w - wl_min) / wl_range - 1.0)
            .collect();

        // Build the normal equations for polynomial least-squares fit.
        // A^T A c = A^T f  where A is the Vandermonde matrix.
        let m = order + 1;
        let mut ata = vec![0.0; m * m];
        let mut atf = vec![0.0; m];

        for i in 0..n {
            let x = norm_wl[i];
            let f = spectrum[i].1;
            let mut xi = 1.0;
            for j in 0..m {
                atf[j] += xi * f;
                let mut xk = 1.0;
                for k in 0..m {
                    ata[j * m + k] += xi * xk;
                    xk *= x;
                }
                xi *= x;
            }
        }

        // Solve via Gaussian elimination with partial pivoting.
        let coeffs = solve_linear_system(&mut ata, &mut atf, m);

        // Divide flux by the fitted continuum.
        for i in 0..n {
            let x = norm_wl[i];
            let mut cont = 0.0;
            let mut xi = 1.0;
            for c in &coeffs {
                cont += c * xi;
                xi *= x;
            }
            if cont.abs() > 1e-30 {
                spectrum[i].1 /= cont;
            }
        }
    }

    /// Detect absorption/emission lines in a spectrum.
    ///
    /// Lines are identified as deviations exceeding `sigma_threshold` standard
    /// deviations below the local mean (absorption) or above (emission).
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs, ideally continuum-normalized.
    /// * `sigma_threshold` - Detection threshold in standard deviations (e.g., 3.0).
    ///
    /// # Returns
    /// Vec of (center_nm, depth, fwhm_nm) for each detected line.
    /// Depth is positive for absorption lines (flux dip below continuum).
    pub fn detect_lines(
        &self,
        spectrum: &[(f64, f64)],
        sigma_threshold: f64,
    ) -> Vec<(f64, f64, f64)> {
        if spectrum.len() < 5 {
            return Vec::new();
        }

        // Estimate noise: median absolute deviation of flux differences.
        let fluxes: Vec<f64> = spectrum.iter().map(|(_, f)| *f).collect();
        let mean_flux = fluxes.iter().sum::<f64>() / fluxes.len() as f64;
        let variance =
            fluxes.iter().map(|f| (f - mean_flux).powi(2)).sum::<f64>() / fluxes.len() as f64;
        let sigma = variance.sqrt();

        if sigma < 1e-30 {
            return Vec::new();
        }

        let threshold = sigma_threshold * sigma;

        // Find contiguous regions below (mean - threshold) for absorption lines.
        let mut lines = Vec::new();
        let mut i = 0;
        while i < spectrum.len() {
            if (mean_flux - spectrum[i].1) > threshold {
                // Start of a line region.
                let start = i;
                while i < spectrum.len() && (mean_flux - spectrum[i].1) > threshold * 0.5 {
                    i += 1;
                }
                let end = i;

                // Find the deepest point.
                let mut min_flux = f64::MAX;
                let mut min_idx = start;
                for j in start..end {
                    if spectrum[j].1 < min_flux {
                        min_flux = spectrum[j].1;
                        min_idx = j;
                    }
                }

                let center_nm = spectrum[min_idx].0;
                let depth = mean_flux - min_flux;
                let width_nm = if end > start {
                    spectrum[end.min(spectrum.len() - 1)].0 - spectrum[start].0
                } else {
                    0.0
                };

                lines.push((center_nm, depth, width_nm));
            } else {
                i += 1;
            }
        }

        lines
    }

    /// Fit a Gaussian profile to a spectral line.
    ///
    /// Uses iterative refinement (Gauss-Newton style) to find the best-fit
    /// Gaussian parameters: center wavelength, amplitude (depth), and sigma (width).
    ///
    /// The model is: f(x) = -amplitude * exp(-(x - center)^2 / (2 * sigma^2))
    /// (negative amplitude for absorption lines).
    ///
    /// # Arguments
    /// * `wavelengths` - Wavelength values in nm.
    /// * `fluxes` - Flux values (should be continuum-subtracted, so absorption is negative).
    ///
    /// # Returns
    /// (center_nm, amplitude, sigma_nm) of the best-fit Gaussian.
    pub fn gaussian_fit(&self, wavelengths: &[f64], fluxes: &[f64]) -> (f64, f64, f64) {
        if wavelengths.len() < 3 || wavelengths.len() != fluxes.len() {
            return (0.0, 0.0, 0.0);
        }

        let n = wavelengths.len();

        // Initial guess: center at minimum flux, amplitude from depth, sigma from width.
        let mut min_idx = 0;
        let mut min_val = fluxes[0];
        for (i, &f) in fluxes.iter().enumerate() {
            if f < min_val {
                min_val = f;
                min_idx = i;
            }
        }

        let mut center = wavelengths[min_idx];
        let mut amplitude = -min_val; // positive amplitude for absorption
        if amplitude < 1e-30 {
            amplitude = 1e-6;
        }
        let wl_range = wavelengths[n - 1] - wavelengths[0];
        let mut sigma = (wl_range / 4.0).max(0.01);

        // Gauss-Newton iterations.
        for _ in 0..50 {
            let mut jt_j = [[0.0f64; 3]; 3];
            let mut jt_r = [0.0f64; 3];

            for i in 0..n {
                let x = wavelengths[i];
                let dx = x - center;
                let s2 = sigma * sigma;
                let exp_term = (-dx * dx / (2.0 * s2)).exp();
                let model = -amplitude * exp_term;
                let residual = fluxes[i] - model;

                // Jacobian: partial derivatives of model w.r.t. (center, amplitude, sigma)
                let d_center = -amplitude * exp_term * dx / s2;
                let d_amplitude = -exp_term;
                let d_sigma = -amplitude * exp_term * dx * dx / (s2 * sigma);

                let j = [d_center, d_amplitude, d_sigma];

                for r in 0..3 {
                    jt_r[r] += j[r] * residual;
                    for c in 0..3 {
                        jt_j[r][c] += j[r] * j[c];
                    }
                }
            }

            // Levenberg-Marquardt damping for stability.
            for k in 0..3 {
                jt_j[k][k] += 1e-6 * jt_j[k][k] + 1e-10;
            }

            // Solve 3x3 system.
            let mut mat = vec![0.0; 9];
            let mut rhs = vec![0.0; 3];
            for r in 0..3 {
                rhs[r] = jt_r[r];
                for c in 0..3 {
                    mat[r * 3 + c] = jt_j[r][c];
                }
            }

            let delta = solve_linear_system(&mut mat, &mut rhs, 3);

            center += delta[0];
            amplitude += delta[1];
            sigma += delta[2];

            // Constrain parameters to physical values.
            if amplitude < 0.0 {
                amplitude = 1e-6;
            }
            if sigma < 1e-6 {
                sigma = 1e-6;
            }

            // Check convergence.
            let step_size = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
            if step_size < 1e-20 {
                break;
            }
        }

        (center, amplitude, sigma)
    }
}

// ─── Blackbody Model ──────────────────────────────────────────────────────────

/// Blackbody (Planck) radiation model for stellar continuum emission.
pub struct BlackbodyModel;

impl BlackbodyModel {
    /// Compute the Planck spectral radiance B(lambda, T).
    ///
    ///   B(lambda, T) = (2 h c^2 / lambda^5) / (exp(h c / (lambda k T)) - 1)
    ///
    /// The result is in W·sr^-1·m^-3 (spectral radiance per unit wavelength).
    ///
    /// # Arguments
    /// * `wavelength_nm` - Wavelength in nm (must be > 0).
    /// * `temperature_k` - Temperature in Kelvin (must be > 0).
    ///
    /// # Returns
    /// Spectral radiance, or 0.0 for invalid inputs.
    pub fn planck_function(wavelength_nm: f64, temperature_k: f64) -> f64 {
        if wavelength_nm <= 0.0 || temperature_k <= 0.0 {
            return 0.0;
        }

        let lambda_m = wavelength_nm * 1e-9;
        let c = SPEED_OF_LIGHT_M_S;
        let h = PLANCK_CONSTANT;
        let k = BOLTZMANN_CONSTANT;

        let numerator = 2.0 * h * c * c / (lambda_m.powi(5));
        let exponent = h * c / (lambda_m * k * temperature_k);

        // Guard against overflow in exp.
        if exponent > 700.0 {
            // Wien approximation: B ~ numerator * exp(-exponent)
            return numerator * (-exponent).exp();
        }

        numerator / (exponent.exp() - 1.0)
    }

    /// Compute the peak wavelength from Wien's displacement law.
    ///
    ///   lambda_max = b / T
    ///
    /// where b = 2.898 x 10^6 nm·K.
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin (must be > 0).
    ///
    /// # Returns
    /// Peak wavelength in nm.
    pub fn wien_peak(temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        WIEN_CONSTANT / temperature_k
    }

    /// Estimate effective temperature from the observed peak wavelength using Wien's law.
    ///
    /// # Arguments
    /// * `peak_wavelength_nm` - Observed peak wavelength in nm (must be > 0).
    ///
    /// # Returns
    /// Effective temperature in Kelvin.
    pub fn effective_temperature(peak_wavelength_nm: f64) -> f64 {
        if peak_wavelength_nm <= 0.0 {
            return 0.0;
        }
        WIEN_CONSTANT / peak_wavelength_nm
    }

    /// Approximate B-V color index from effective temperature.
    ///
    /// Uses the Ballesteros (2012) empirical relation:
    ///
    ///   B-V = -0.865 + 8540 / T
    ///
    /// Valid for roughly 3000-40000 K.
    ///
    /// # Arguments
    /// * `temperature_k` - Effective temperature in Kelvin.
    ///
    /// # Returns
    /// Approximate B-V color index.
    pub fn color_index_bv(temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        // Simplified Ballesteros formula
        -0.865 + 8540.0 / temperature_k
    }

    /// Estimate luminosity class from the full-width-at-half-maximum of a strong line.
    ///
    /// Broader lines indicate higher surface gravity (main-sequence dwarfs), while
    /// narrower lines indicate lower gravity (supergiants/giants). This is a rough
    /// classification based on typical pressure-broadening widths.
    ///
    /// # Arguments
    /// * `line_width_nm` - FWHM of a strong absorption line in nm.
    ///
    /// # Returns
    /// Luminosity class string: "I" (supergiant), "II", "III" (giant), "IV", or "V" (dwarf).
    pub fn luminosity_class_from_line_width(line_width_nm: f64) -> String {
        if line_width_nm < 0.02 {
            "I".to_string() // Supergiant - very narrow lines
        } else if line_width_nm < 0.05 {
            "II".to_string() // Bright giant
        } else if line_width_nm < 0.1 {
            "III".to_string() // Giant
        } else if line_width_nm < 0.2 {
            "IV".to_string() // Subgiant
        } else {
            "V".to_string() // Main-sequence dwarf - broadest lines
        }
    }
}

// ─── Absorption Line Database ─────────────────────────────────────────────────

/// Database of common stellar absorption lines for identification.
///
/// Pre-loaded with hydrogen Balmer series, calcium H and K, sodium D,
/// and other commonly observed stellar lines.
pub struct AbsorptionLineDatabase {
    lines: Vec<SpectralLine>,
}

impl AbsorptionLineDatabase {
    /// Create a new database pre-loaded with common stellar absorption lines.
    pub fn new() -> Self {
        let lines = vec![
            // Hydrogen Balmer series
            SpectralLine {
                wavelength_nm: 656.28,
                element: "H-alpha".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 486.13,
                element: "H-beta".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 434.05,
                element: "H-gamma".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 410.17,
                element: "H-delta".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 397.01,
                element: "H-epsilon".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 388.91,
                element: "H-zeta".to_string(),
                ionization_state: 0,
            },
            // Calcium II H and K
            SpectralLine {
                wavelength_nm: 393.37,
                element: "Ca II K".to_string(),
                ionization_state: 1,
            },
            SpectralLine {
                wavelength_nm: 396.85,
                element: "Ca II H".to_string(),
                ionization_state: 1,
            },
            // Sodium D
            SpectralLine {
                wavelength_nm: 589.00,
                element: "Na I D2".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 589.59,
                element: "Na I D1".to_string(),
                ionization_state: 0,
            },
            // Magnesium
            SpectralLine {
                wavelength_nm: 517.27,
                element: "Mg I b".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 518.36,
                element: "Mg I b".to_string(),
                ionization_state: 0,
            },
            // Iron
            SpectralLine {
                wavelength_nm: 438.36,
                element: "Fe I".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 527.04,
                element: "Fe I".to_string(),
                ionization_state: 0,
            },
            // Helium I
            SpectralLine {
                wavelength_nm: 587.56,
                element: "He I D3".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 447.15,
                element: "He I".to_string(),
                ionization_state: 0,
            },
            // Helium II (ionized)
            SpectralLine {
                wavelength_nm: 468.57,
                element: "He II".to_string(),
                ionization_state: 1,
            },
            // Titanium Oxide bands (M stars)
            SpectralLine {
                wavelength_nm: 705.00,
                element: "TiO".to_string(),
                ionization_state: 0,
            },
            SpectralLine {
                wavelength_nm: 620.00,
                element: "TiO".to_string(),
                ionization_state: 0,
            },
            // Oxygen
            SpectralLine {
                wavelength_nm: 777.19,
                element: "O I".to_string(),
                ionization_state: 0,
            },
        ];

        Self { lines }
    }

    /// Compute the wavelength of a hydrogen Balmer series line.
    ///
    /// The Balmer formula gives:
    ///
    ///   1/lambda = R_H * (1/4 - 1/n^2)
    ///
    /// where n = 3 (H-alpha), 4 (H-beta), 5 (H-gamma), etc.
    ///
    /// # Arguments
    /// * `n` - Upper quantum number (must be >= 3 for Balmer series).
    ///
    /// # Returns
    /// Wavelength in nm.
    pub fn hydrogen_balmer(n: usize) -> f64 {
        if n < 3 {
            return 0.0;
        }
        // Rydberg constant for hydrogen: 1.0973731568539e7 m^-1
        let r_h = 1.097_373_156_8539e7; // m^-1
        let inv_lambda = r_h * (0.25 - 1.0 / (n as f64 * n as f64));
        if inv_lambda <= 0.0 {
            return 0.0;
        }
        1.0e9 / inv_lambda // convert m to nm
    }

    /// Identify a spectral line by matching an observed wavelength against the database.
    ///
    /// # Arguments
    /// * `observed_nm` - Observed wavelength in nm.
    /// * `tolerance_nm` - Maximum allowed difference in nm for a match.
    ///
    /// # Returns
    /// The closest matching spectral line within tolerance, or None.
    pub fn identify_line(&self, observed_nm: f64, tolerance_nm: f64) -> Option<SpectralLine> {
        let mut best: Option<&SpectralLine> = None;
        let mut best_dist = f64::MAX;

        for line in &self.lines {
            let dist = (line.wavelength_nm - observed_nm).abs();
            if dist <= tolerance_nm && dist < best_dist {
                best = Some(line);
                best_dist = dist;
            }
        }

        best.cloned()
    }

    /// Return all known lines within a wavelength range.
    ///
    /// # Arguments
    /// * `min_nm` - Minimum wavelength in nm (inclusive).
    /// * `max_nm` - Maximum wavelength in nm (inclusive).
    ///
    /// # Returns
    /// Vec of matching spectral lines.
    pub fn lines_in_range(&self, min_nm: f64, max_nm: f64) -> Vec<SpectralLine> {
        self.lines
            .iter()
            .filter(|l| l.wavelength_nm >= min_nm && l.wavelength_nm <= max_nm)
            .cloned()
            .collect()
    }
}

impl Default for AbsorptionLineDatabase {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Spectral Classifier ─────────────────────────────────────────────────────

/// Classifies stars into Harvard spectral types based on spectral features.
///
/// Uses the relative strengths of hydrogen Balmer lines, calcium K, helium lines,
/// and TiO molecular bands to determine the spectral class.
pub struct SpectralClassifier;

impl SpectralClassifier {
    /// Classify a star based on its spectrum and detected lines.
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs (ideally continuum-normalized).
    /// * `lines_detected` - Detected lines as (center_nm, depth, width_nm).
    ///
    /// # Returns
    /// Best-fit Harvard spectral class.
    pub fn classify(
        spectrum: &[(f64, f64)],
        lines_detected: &[(f64, f64, f64)],
    ) -> StellarClassifier {
        let h_strength = Self::hydrogen_line_strength(spectrum);
        let ca_strength = Self::calcium_k_strength(spectrum);
        let tio_strength = Self::tio_band_strength(spectrum);

        // Check for helium lines (hot stars).
        let has_he_ii = lines_detected
            .iter()
            .any(|(wl, depth, _)| (wl - 468.57).abs() < 2.0 && *depth > 0.02);
        let has_he_i = lines_detected
            .iter()
            .any(|(wl, depth, _)| (wl - 447.15).abs() < 2.0 && *depth > 0.02);

        // Classification logic based on spectral feature strengths.
        if has_he_ii {
            StellarClassifier::O
        } else if has_he_i && h_strength < 0.3 {
            StellarClassifier::B
        } else if h_strength > 0.5 {
            StellarClassifier::A
        } else if h_strength > 0.3 && ca_strength > 0.05 {
            StellarClassifier::F
        } else if ca_strength > 0.15 && h_strength > 0.1 {
            StellarClassifier::G
        } else if tio_strength > 0.05 {
            StellarClassifier::M
        } else if ca_strength > 0.1 {
            StellarClassifier::K
        } else {
            // Default to G (solar-type) if features are ambiguous.
            StellarClassifier::G
        }
    }

    /// Measure the strength of hydrogen Balmer lines (H-alpha and H-beta depth).
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs.
    ///
    /// # Returns
    /// Average depth of H-alpha and H-beta relative to the local continuum.
    pub fn hydrogen_line_strength(spectrum: &[(f64, f64)]) -> f64 {
        let h_alpha_depth = Self::line_depth_at(spectrum, 656.28, 2.0);
        let h_beta_depth = Self::line_depth_at(spectrum, 486.13, 2.0);
        (h_alpha_depth + h_beta_depth) / 2.0
    }

    /// Measure the depth of the Ca II K line at 393.37 nm.
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs.
    ///
    /// # Returns
    /// Depth of Ca II K relative to the local continuum.
    pub fn calcium_k_strength(spectrum: &[(f64, f64)]) -> f64 {
        Self::line_depth_at(spectrum, 393.37, 2.0)
    }

    /// Measure the strength of TiO molecular absorption bands.
    ///
    /// TiO bands are prominent in cool M-type stars, particularly around 620 nm
    /// and 705 nm.
    ///
    /// # Arguments
    /// * `spectrum` - (wavelength_nm, flux) pairs.
    ///
    /// # Returns
    /// Average depth in the TiO band regions.
    pub fn tio_band_strength(spectrum: &[(f64, f64)]) -> f64 {
        let band1 = Self::line_depth_at(spectrum, 620.0, 10.0);
        let band2 = Self::line_depth_at(spectrum, 705.0, 10.0);
        (band1 + band2) / 2.0
    }

    /// Approximate effective temperature from spectral class.
    ///
    /// Returns a representative T_eff for the middle of each class.
    ///
    /// # Arguments
    /// * `class` - Harvard spectral class.
    ///
    /// # Returns
    /// Approximate effective temperature in Kelvin.
    pub fn temperature_estimate(class: &StellarClassifier) -> f64 {
        match class {
            StellarClassifier::O => 40_000.0,
            StellarClassifier::B => 20_000.0,
            StellarClassifier::A => 8_750.0,
            StellarClassifier::F => 6_750.0,
            StellarClassifier::G => 5_600.0,
            StellarClassifier::K => 4_450.0,
            StellarClassifier::M => 3_050.0,
        }
    }

    /// Measure the depth of a spectral feature at a given wavelength.
    ///
    /// The "depth" is computed as:
    ///   (continuum - line_minimum) / continuum
    ///
    /// where the continuum is estimated from the edges of the window.
    fn line_depth_at(spectrum: &[(f64, f64)], center_nm: f64, half_window_nm: f64) -> f64 {
        let wl_min = center_nm - half_window_nm;
        let wl_max = center_nm + half_window_nm;

        // Collect points within the window.
        let in_window: Vec<(f64, f64)> = spectrum
            .iter()
            .filter(|(w, _)| *w >= wl_min && *w <= wl_max)
            .copied()
            .collect();

        if in_window.is_empty() {
            return 0.0;
        }

        // Estimate continuum from the mean of the outermost quarter of points.
        let quarter = (in_window.len() / 4).max(1);
        let cont_low: f64 = in_window[..quarter].iter().map(|(_, f)| f).sum::<f64>() / quarter as f64;
        let cont_high: f64 = in_window[in_window.len() - quarter..]
            .iter()
            .map(|(_, f)| f)
            .sum::<f64>()
            / quarter as f64;
        let continuum = (cont_low + cont_high) / 2.0;

        if continuum <= 0.0 {
            return 0.0;
        }

        // Find minimum flux in the window.
        let min_flux = in_window
            .iter()
            .map(|(_, f)| *f)
            .fold(f64::MAX, f64::min);

        ((continuum - min_flux) / continuum).max(0.0)
    }
}

// ─── Helper Functions ─────────────────────────────────────────────────────────

/// Linear interpolation in a sorted (x, y) dataset.
///
/// Returns None if `target_x` is outside the data range.
fn linear_interp(data: &[(f64, f64)], target_x: f64) -> Option<f64> {
    if data.is_empty() {
        return None;
    }
    if target_x < data[0].0 || target_x > data[data.len() - 1].0 {
        return None;
    }

    // Binary search for the enclosing interval.
    let mut lo = 0;
    let mut hi = data.len() - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if data[mid].0 <= target_x {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let (x0, y0) = data[lo];
    let (x1, y1) = data[hi];
    if (x1 - x0).abs() < 1e-30 {
        return Some(y0);
    }

    let t = (target_x - x0) / (x1 - x0);
    Some(y0 + t * (y1 - y0))
}

/// Solve a linear system Ax = b via Gaussian elimination with partial pivoting.
///
/// Both `a` (row-major n x n) and `b` (length n) are modified in place.
/// Returns the solution vector x.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination with partial pivoting.
    for col in 0..n {
        // Find pivot.
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows.
        if max_row != col {
            for k in 0..n {
                let tmp = a[col * n + k];
                a[col * n + k] = a[max_row * n + k];
                a[max_row * n + k] = tmp;
            }
            let tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        // Eliminate below.
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution.
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

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SpectroscopyConfig {
        SpectroscopyConfig {
            wavelength_min_nm: 380.0,
            wavelength_max_nm: 750.0,
            spectral_resolution: 10000.0,
            snr: 100.0,
            radial_velocity_range_km_s: 500.0,
        }
    }

    fn make_analyzer() -> StellarSpectroscopyAnalyzer {
        StellarSpectroscopyAnalyzer::new(default_config())
    }

    // ── Doppler shift tests ──

    #[test]
    fn test_doppler_shift_redshift() {
        let a = make_analyzer();
        // +100 km/s should redshift H-alpha by ~0.219 nm
        let shifted = a.doppler_shift(656.28, 100.0);
        let expected_shift = 656.28 * 100.0 / SPEED_OF_LIGHT_KM_S;
        assert!(
            (shifted - 656.28 - expected_shift).abs() < 0.001,
            "H-alpha shift at 100 km/s: expected ~{:.3} nm shift, got {:.3} nm shift",
            expected_shift,
            shifted - 656.28
        );
        // Should be ~0.219 nm
        assert!((shifted - 656.28 - 0.2189).abs() < 0.01);
    }

    #[test]
    fn test_doppler_shift_blueshift() {
        let a = make_analyzer();
        let shifted = a.doppler_shift(656.28, -100.0);
        assert!(shifted < 656.28);
        assert!((656.28 - shifted - 0.2189).abs() < 0.01);
    }

    #[test]
    fn test_doppler_shift_zero_velocity() {
        let a = make_analyzer();
        let shifted = a.doppler_shift(656.28, 0.0);
        assert!((shifted - 656.28).abs() < 1e-10, "Zero velocity should produce no shift");
    }

    #[test]
    fn test_doppler_shift_large_velocity() {
        let a = make_analyzer();
        // 10,000 km/s ~ 3.3% of c
        let shifted = a.doppler_shift(500.0, 10_000.0);
        let expected = 500.0 * (1.0 + 10_000.0 / SPEED_OF_LIGHT_KM_S);
        assert!((shifted - expected).abs() < 1e-6);
    }

    // ── Radial velocity tests ──

    #[test]
    fn test_radial_velocity_from_shift() {
        let a = make_analyzer();
        let rv = a.radial_velocity(656.28, 656.499);
        assert!((rv - 100.0).abs() < 1.5, "RV should be ~100 km/s, got {}", rv);
    }

    #[test]
    fn test_radial_velocity_zero() {
        let a = make_analyzer();
        let rv = a.radial_velocity(656.28, 656.28);
        assert!((rv).abs() < 1e-10, "Identical wavelengths should give zero RV");
    }

    #[test]
    fn test_radial_velocity_roundtrip() {
        let a = make_analyzer();
        let v_in = 42.7;
        let shifted = a.doppler_shift(500.0, v_in);
        let v_out = a.radial_velocity(500.0, shifted);
        assert!((v_out - v_in).abs() < 1e-6, "Roundtrip failed: {} vs {}", v_in, v_out);
    }

    #[test]
    fn test_radial_velocity_negative() {
        let a = make_analyzer();
        let rv = a.radial_velocity(656.28, 656.0);
        assert!(rv < 0.0, "Blueshift should give negative velocity");
    }

    // ── Cross-correlation RV tests ──

    #[test]
    fn test_ccf_peak_at_known_velocity() {
        let a = make_analyzer();

        // Create a simple template with a single line at 500 nm.
        let template: Vec<(f64, f64)> = (0..1000)
            .map(|i| {
                let wl = 480.0 + i as f64 * 0.04;
                let flux = 1.0 - 0.5 * (-(wl - 500.0).powi(2) / (2.0 * 0.1_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        // Create observed spectrum shifted by +50 km/s.
        let v_true = 50.0;
        let shift = 1.0 + v_true / SPEED_OF_LIGHT_KM_S;
        let spectrum: Vec<(f64, f64)> = template
            .iter()
            .map(|(wl, flux)| (wl * shift, *flux))
            .collect();

        let velocities: Vec<f64> = (-100..=100).map(|i| i as f64 * 2.0).collect();
        let ccf = a.cross_correlate_rv(&spectrum, &template, &velocities);
        let (peak_v, _peak_h) = a.find_rv_peak(&ccf);

        assert!(
            (peak_v - v_true).abs() < 5.0,
            "CCF peak should be near {} km/s, got {} km/s",
            v_true,
            peak_v
        );
    }

    #[test]
    fn test_ccf_zero_velocity() {
        let a = make_analyzer();

        let template: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let wl = 490.0 + i as f64 * 0.1;
                let flux = 1.0 - 0.3 * (-(wl - 500.0).powi(2) / (2.0 * 0.2_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let velocities: Vec<f64> = (-50..=50).map(|i| i as f64 * 2.0).collect();
        let ccf = a.cross_correlate_rv(&template, &template, &velocities);
        let (peak_v, _) = a.find_rv_peak(&ccf);

        assert!(
            peak_v.abs() < 4.0,
            "Self-correlation should peak near 0 km/s, got {} km/s",
            peak_v
        );
    }

    #[test]
    fn test_find_rv_peak_empty() {
        let a = make_analyzer();
        let (v, h) = a.find_rv_peak(&[]);
        assert_eq!(v, 0.0);
        assert_eq!(h, 0.0);
    }

    // ── Equivalent width tests ──

    #[test]
    fn test_equivalent_width_absorption() {
        let a = make_analyzer();

        // Spectrum with a Gaussian absorption line at 500 nm.
        let spectrum: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let wl = 490.0 + i as f64 * 0.04;
                let flux = 1.0 - 0.5 * (-(wl - 500.0).powi(2) / (2.0 * 0.2_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let ew = a.equivalent_width(&spectrum, 500.0, 1.0, 2.0);
        assert!(ew > 0.0, "EW should be positive for absorption, got {}", ew);
    }

    #[test]
    fn test_equivalent_width_deeper_line() {
        let a = make_analyzer();

        // Shallow line.
        let shallow: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let wl = 490.0 + i as f64 * 0.04;
                let flux = 1.0 - 0.2 * (-(wl - 500.0).powi(2) / (2.0 * 0.2_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        // Deep line.
        let deep: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let wl = 490.0 + i as f64 * 0.04;
                let flux = 1.0 - 0.8 * (-(wl - 500.0).powi(2) / (2.0 * 0.2_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let ew_shallow = a.equivalent_width(&shallow, 500.0, 1.0, 2.0);
        let ew_deep = a.equivalent_width(&deep, 500.0, 1.0, 2.0);
        assert!(
            ew_deep > ew_shallow,
            "Deeper line should have larger EW: {} vs {}",
            ew_deep,
            ew_shallow
        );
    }

    #[test]
    fn test_equivalent_width_flat_spectrum() {
        let a = make_analyzer();
        let flat: Vec<(f64, f64)> = (0..100)
            .map(|i| (490.0 + i as f64 * 0.2, 1.0))
            .collect();

        let ew = a.equivalent_width(&flat, 500.0, 1.0, 2.0);
        assert!(ew.abs() < 1e-10, "Flat spectrum should have zero EW, got {}", ew);
    }

    // ── Continuum normalization tests ──

    #[test]
    fn test_continuum_normalize_linear() {
        let a = make_analyzer();
        let mut spectrum: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let wl = 400.0 + i as f64 * 3.0;
                // Sloped continuum: 1.0 at 400 nm, 2.0 at 700 nm.
                let flux = 1.0 + (wl - 400.0) / 300.0;
                (wl, flux)
            })
            .collect();

        a.continuum_normalize(&mut spectrum, 1);

        // After normalization, all fluxes should be close to 1.0.
        for &(_, f) in &spectrum {
            assert!(
                (f - 1.0).abs() < 0.01,
                "Normalized flux should be ~1.0, got {}",
                f
            );
        }
    }

    #[test]
    fn test_continuum_normalize_preserves_line() {
        let a = make_analyzer();

        // Spectrum with a line at 500 nm on a flat continuum.
        let mut spectrum: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let wl = 450.0 + i as f64 * 0.5;
                let flux = 2.0 - 1.0 * (-(wl - 500.0).powi(2) / (2.0 * 0.5_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        a.continuum_normalize(&mut spectrum, 2);

        // The line center should still be depressed below 1.0.
        let center_flux = spectrum
            .iter()
            .find(|(w, _)| (*w - 500.0).abs() < 0.3)
            .map(|(_, f)| *f)
            .unwrap();

        assert!(
            center_flux < 0.95,
            "Line center should be below 1.0, got {}",
            center_flux
        );
    }

    // ── Line detection tests ──

    #[test]
    fn test_detect_injected_line() {
        let a = make_analyzer();

        // Continuum-normalized spectrum with injected Gaussian absorption at 550 nm.
        let spectrum: Vec<(f64, f64)> = (0..1000)
            .map(|i| {
                let wl = 400.0 + i as f64 * 0.3;
                let flux = 1.0 - 0.4 * (-(wl - 550.0).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let lines = a.detect_lines(&spectrum, 3.0);
        assert!(
            !lines.is_empty(),
            "Should detect at least one line in the spectrum"
        );

        // The detected line should be near 550 nm.
        let closest = lines
            .iter()
            .min_by(|a, b| {
                (a.0 - 550.0)
                    .abs()
                    .partial_cmp(&(b.0 - 550.0).abs())
                    .unwrap()
            })
            .unwrap();

        assert!(
            (closest.0 - 550.0).abs() < 2.0,
            "Detected line should be near 550 nm, got {} nm",
            closest.0
        );
    }

    #[test]
    fn test_detect_no_lines_in_flat() {
        let a = make_analyzer();
        let flat: Vec<(f64, f64)> = (0..100)
            .map(|i| (400.0 + i as f64 * 3.0, 1.0))
            .collect();

        let lines = a.detect_lines(&flat, 3.0);
        assert!(
            lines.is_empty(),
            "Flat spectrum should have no detected lines"
        );
    }

    #[test]
    fn test_detect_multiple_lines() {
        let a = make_analyzer();

        let spectrum: Vec<(f64, f64)> = (0..2000)
            .map(|i| {
                let wl = 400.0 + i as f64 * 0.15;
                let flux = 1.0
                    - 0.4 * (-(wl - 450.0).powi(2) / (2.0 * 0.3_f64.powi(2))).exp()
                    - 0.3 * (-(wl - 550.0).powi(2) / (2.0 * 0.3_f64.powi(2))).exp()
                    - 0.5 * (-(wl - 650.0).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let lines = a.detect_lines(&spectrum, 2.5);
        assert!(
            lines.len() >= 2,
            "Should detect multiple lines, found {}",
            lines.len()
        );
    }

    // ── Gaussian fit tests ──

    #[test]
    fn test_gaussian_fit_known_line() {
        let a = make_analyzer();

        let center_true: f64 = 500.0;
        let amp_true: f64 = 0.5;
        let sigma_true: f64 = 0.3;

        let wavelengths: Vec<f64> = (0..200).map(|i| 498.0 + i as f64 * 0.02).collect();
        let fluxes: Vec<f64> = wavelengths
            .iter()
            .map(|&w| -amp_true * (-(w - center_true).powi(2) / (2.0 * sigma_true.powi(2))).exp())
            .collect();

        let (c, a_fit, s) = a.gaussian_fit(&wavelengths, &fluxes);

        assert!(
            (c - center_true).abs() < 0.05,
            "Center: expected {}, got {}",
            center_true,
            c
        );
        assert!(
            (a_fit - amp_true).abs() < 0.05,
            "Amplitude: expected {}, got {}",
            amp_true,
            a_fit
        );
        assert!(
            (s - sigma_true).abs() < 0.1,
            "Sigma: expected {}, got {}",
            sigma_true,
            s
        );
    }

    #[test]
    fn test_gaussian_fit_narrow_line() {
        let a = make_analyzer();

        let wavelengths: Vec<f64> = (0..100).map(|i| 499.0 + i as f64 * 0.02).collect();
        let fluxes: Vec<f64> = wavelengths
            .iter()
            .map(|&w| -0.8 * (-(w - 500.0).powi(2) / (2.0 * 0.05_f64.powi(2))).exp())
            .collect();

        let (c, _amp, sigma) = a.gaussian_fit(&wavelengths, &fluxes);
        assert!((c - 500.0).abs() < 0.1, "Center should be ~500 nm");
        assert!(sigma < 0.2, "Sigma should be small for narrow line");
    }

    // ── Blackbody model tests ──

    #[test]
    fn test_wien_peak_sun() {
        // Sun at 5778 K should peak at ~502 nm.
        let peak = BlackbodyModel::wien_peak(5778.0);
        assert!(
            (peak - 501.5).abs() < 2.0,
            "Solar peak should be ~502 nm, got {} nm",
            peak
        );
    }

    #[test]
    fn test_wien_peak_hot_star() {
        // 30,000 K star peaks at ~97 nm (UV).
        let peak = BlackbodyModel::wien_peak(30000.0);
        assert!(
            (peak - 96.6).abs() < 1.0,
            "Hot star peak should be ~97 nm, got {} nm",
            peak
        );
    }

    #[test]
    fn test_effective_temperature_roundtrip() {
        let t_in = 6000.0;
        let peak = BlackbodyModel::wien_peak(t_in);
        let t_out = BlackbodyModel::effective_temperature(peak);
        assert!(
            (t_out - t_in).abs() < 0.1,
            "Roundtrip failed: {} vs {}",
            t_in,
            t_out
        );
    }

    #[test]
    fn test_planck_positive() {
        // Planck function should be positive for all T > 0, lambda > 0.
        for &temp in &[3000.0, 5778.0, 10000.0, 30000.0] {
            for &wl in &[200.0, 400.0, 550.0, 700.0, 1000.0] {
                let b = BlackbodyModel::planck_function(wl, temp);
                assert!(
                    b > 0.0,
                    "Planck should be positive at T={}, lambda={}, got {}",
                    temp,
                    wl,
                    b
                );
            }
        }
    }

    #[test]
    fn test_planck_hotter_is_brighter() {
        // At any wavelength, a hotter blackbody should be brighter.
        let wl = 500.0;
        let b_cool = BlackbodyModel::planck_function(wl, 3000.0);
        let b_hot = BlackbodyModel::planck_function(wl, 10000.0);
        assert!(
            b_hot > b_cool,
            "Hotter star should be brighter: {} vs {}",
            b_hot,
            b_cool
        );
    }

    #[test]
    fn test_planck_invalid_inputs() {
        assert_eq!(BlackbodyModel::planck_function(0.0, 5778.0), 0.0);
        assert_eq!(BlackbodyModel::planck_function(-100.0, 5778.0), 0.0);
        assert_eq!(BlackbodyModel::planck_function(500.0, 0.0), 0.0);
        assert_eq!(BlackbodyModel::planck_function(500.0, -100.0), 0.0);
    }

    #[test]
    fn test_color_index_sun() {
        // Sun (~5778 K) should have B-V ~ 0.65.
        let bv = BlackbodyModel::color_index_bv(5778.0);
        assert!(
            (bv - 0.61).abs() < 0.2,
            "Solar B-V should be ~0.65, got {}",
            bv
        );
    }

    #[test]
    fn test_color_index_hot_star_blue() {
        // Hot star should have negative (blue) B-V.
        let bv = BlackbodyModel::color_index_bv(30000.0);
        assert!(bv < 0.0, "Hot star should be blue (B-V < 0), got {}", bv);
    }

    #[test]
    fn test_luminosity_class_from_width() {
        assert_eq!(BlackbodyModel::luminosity_class_from_line_width(0.01), "I");
        assert_eq!(BlackbodyModel::luminosity_class_from_line_width(0.03), "II");
        assert_eq!(BlackbodyModel::luminosity_class_from_line_width(0.07), "III");
        assert_eq!(BlackbodyModel::luminosity_class_from_line_width(0.15), "IV");
        assert_eq!(BlackbodyModel::luminosity_class_from_line_width(0.3), "V");
    }

    // ── Absorption line database tests ──

    #[test]
    fn test_balmer_h_alpha() {
        let wl = AbsorptionLineDatabase::hydrogen_balmer(3);
        // Rydberg formula gives ~656.11 nm; commonly quoted 656.28 nm includes
        // finite nuclear mass correction. Tolerance 0.3 nm covers both.
        assert!(
            (wl - 656.28).abs() < 0.3,
            "H-alpha should be ~656.28 nm, got {} nm",
            wl
        );
    }

    #[test]
    fn test_balmer_h_beta() {
        let wl = AbsorptionLineDatabase::hydrogen_balmer(4);
        assert!(
            (wl - 486.13).abs() < 0.2,
            "H-beta should be ~486.13 nm, got {} nm",
            wl
        );
    }

    #[test]
    fn test_balmer_h_gamma() {
        let wl = AbsorptionLineDatabase::hydrogen_balmer(5);
        assert!(
            (wl - 434.05).abs() < 0.2,
            "H-gamma should be ~434.05 nm, got {} nm",
            wl
        );
    }

    #[test]
    fn test_balmer_series_converges() {
        // Higher-n lines should converge toward the series limit (364.6 nm).
        let wl_10 = AbsorptionLineDatabase::hydrogen_balmer(10);
        let wl_20 = AbsorptionLineDatabase::hydrogen_balmer(20);
        assert!(wl_20 < wl_10, "Higher n should give shorter wavelength");
        assert!(
            wl_20 > 364.0,
            "Should stay above Balmer limit, got {}",
            wl_20
        );
    }

    #[test]
    fn test_balmer_invalid_n() {
        assert_eq!(AbsorptionLineDatabase::hydrogen_balmer(2), 0.0);
        assert_eq!(AbsorptionLineDatabase::hydrogen_balmer(1), 0.0);
    }

    #[test]
    fn test_identify_h_alpha() {
        let db = AbsorptionLineDatabase::new();
        let line = db.identify_line(656.3, 0.5);
        assert!(line.is_some());
        let l = line.unwrap();
        assert_eq!(l.element, "H-alpha");
    }

    #[test]
    fn test_identify_ca_k() {
        let db = AbsorptionLineDatabase::new();
        let line = db.identify_line(393.4, 0.5);
        assert!(line.is_some());
        let l = line.unwrap();
        assert!(l.element.contains("Ca II K"));
    }

    #[test]
    fn test_identify_no_match() {
        let db = AbsorptionLineDatabase::new();
        let line = db.identify_line(999.0, 0.5);
        assert!(line.is_none(), "No line should be found at 999 nm");
    }

    #[test]
    fn test_lines_in_range() {
        let db = AbsorptionLineDatabase::new();
        let visible = db.lines_in_range(380.0, 700.0);
        assert!(
            visible.len() >= 10,
            "Should find many lines in visible range, found {}",
            visible.len()
        );
    }

    #[test]
    fn test_lines_in_narrow_range() {
        let db = AbsorptionLineDatabase::new();
        let narrow = db.lines_in_range(655.0, 658.0);
        assert!(
            narrow.len() >= 1,
            "Should find H-alpha in 655-658 nm range"
        );
        assert!(narrow.iter().any(|l| l.element == "H-alpha"));
    }

    // ── Spectral classifier tests ──

    #[test]
    fn test_classify_a_star() {
        // A-type star: strong H-alpha and H-beta, weak metals.
        let spectrum: Vec<(f64, f64)> = (0..3000)
            .map(|i| {
                let wl = 380.0 + i as f64 * 0.12;
                let mut flux = 1.0;
                // Strong hydrogen lines.
                flux -= 0.7 * (-(wl - 656.28).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                flux -= 0.6 * (-(wl - 486.13).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                flux -= 0.5 * (-(wl - 434.05).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let a = make_analyzer();
        let lines = a.detect_lines(&spectrum, 2.0);
        let class = SpectralClassifier::classify(&spectrum, &lines);
        assert_eq!(
            class,
            StellarClassifier::A,
            "Star with strong H lines should be A-type"
        );
    }

    #[test]
    fn test_classify_m_star() {
        // M-type star: TiO bands, weak H.
        let spectrum: Vec<(f64, f64)> = (0..3000)
            .map(|i| {
                let wl = 380.0 + i as f64 * 0.12;
                let mut flux = 1.0;
                // TiO bands.
                flux -= 0.2 * (-(wl - 620.0).powi(2) / (2.0 * 5.0_f64.powi(2))).exp();
                flux -= 0.2 * (-(wl - 705.0).powi(2) / (2.0 * 5.0_f64.powi(2))).exp();
                // Weak H-alpha.
                flux -= 0.05 * (-(wl - 656.28).powi(2) / (2.0 * 0.3_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let a = make_analyzer();
        let lines = a.detect_lines(&spectrum, 2.0);
        let class = SpectralClassifier::classify(&spectrum, &lines);
        assert_eq!(
            class,
            StellarClassifier::M,
            "Star with TiO bands should be M-type"
        );
    }

    #[test]
    fn test_temperature_estimates() {
        let t_o = SpectralClassifier::temperature_estimate(&StellarClassifier::O);
        let t_a = SpectralClassifier::temperature_estimate(&StellarClassifier::A);
        let t_g = SpectralClassifier::temperature_estimate(&StellarClassifier::G);
        let t_m = SpectralClassifier::temperature_estimate(&StellarClassifier::M);

        assert!(t_o > t_a, "O should be hotter than A");
        assert!(t_a > t_g, "A should be hotter than G");
        assert!(t_g > t_m, "G should be hotter than M");
    }

    #[test]
    fn test_hydrogen_line_strength_strong() {
        // Spectrum with deep H-alpha and H-beta.
        let spectrum: Vec<(f64, f64)> = (0..3000)
            .map(|i| {
                let wl = 380.0 + i as f64 * 0.12;
                let mut flux = 1.0;
                flux -= 0.6 * (-(wl - 656.28).powi(2) / (2.0 * 0.5_f64.powi(2))).exp();
                flux -= 0.5 * (-(wl - 486.13).powi(2) / (2.0 * 0.5_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let h = SpectralClassifier::hydrogen_line_strength(&spectrum);
        assert!(h > 0.3, "Strong H lines should give high strength, got {}", h);
    }

    #[test]
    fn test_calcium_k_strength() {
        let spectrum: Vec<(f64, f64)> = (0..3000)
            .map(|i| {
                let wl = 380.0 + i as f64 * 0.12;
                let flux = 1.0 - 0.4 * (-(wl - 393.37).powi(2) / (2.0 * 0.5_f64.powi(2))).exp();
                (wl, flux)
            })
            .collect();

        let ca = SpectralClassifier::calcium_k_strength(&spectrum);
        assert!(ca > 0.1, "Should detect Ca II K absorption, got {}", ca);
    }

    // ── Constants tests ──

    #[test]
    fn test_speed_of_light() {
        assert!((SPEED_OF_LIGHT_KM_S - 299792.458).abs() < 0.001);
    }

    #[test]
    fn test_wien_constant() {
        assert!((WIEN_CONSTANT - 2.898e6).abs() < 1e4);
    }

    // ── Linear interpolation tests ──

    #[test]
    fn test_linear_interp_midpoint() {
        let data = vec![(0.0, 0.0), (1.0, 1.0)];
        let y = linear_interp(&data, 0.5).unwrap();
        assert!((y - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_linear_interp_endpoints() {
        let data = vec![(0.0, 10.0), (1.0, 20.0)];
        assert!((linear_interp(&data, 0.0).unwrap() - 10.0).abs() < 1e-10);
        assert!((linear_interp(&data, 1.0).unwrap() - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_interp_out_of_range() {
        let data = vec![(0.0, 0.0), (1.0, 1.0)];
        assert!(linear_interp(&data, -0.1).is_none());
        assert!(linear_interp(&data, 1.1).is_none());
    }

    // ── Edge case tests ──

    #[test]
    fn test_stellar_classifier_display() {
        assert_eq!(format!("{}", StellarClassifier::O), "O");
        assert_eq!(format!("{}", StellarClassifier::G), "G");
        assert_eq!(format!("{}", StellarClassifier::M), "M");
    }

    #[test]
    fn test_config_stored() {
        let config = default_config();
        let a = StellarSpectroscopyAnalyzer::new(config);
        assert!((a.config().spectral_resolution - 10000.0).abs() < 1e-10);
    }

    #[test]
    fn test_equivalent_width_zero_continuum() {
        let a = make_analyzer();
        let spectrum = vec![(500.0, 0.5), (501.0, 0.5)];
        let ew = a.equivalent_width(&spectrum, 500.5, 0.0, 1.0);
        assert_eq!(ew, 0.0);
    }

    #[test]
    fn test_gaussian_fit_too_few_points() {
        let a = make_analyzer();
        let (c, amp, s) = a.gaussian_fit(&[500.0], &[-0.5]);
        assert_eq!(c, 0.0);
        assert_eq!(amp, 0.0);
        assert_eq!(s, 0.0);
    }

    #[test]
    fn test_absorption_line_database_default() {
        let db = AbsorptionLineDatabase::default();
        assert!(db.lines_in_range(380.0, 800.0).len() > 10);
    }
}
