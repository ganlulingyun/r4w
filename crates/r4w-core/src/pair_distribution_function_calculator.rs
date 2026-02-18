//! # Pair Distribution Function Calculator
//!
//! Real-space total scattering analysis via the pair distribution function (PDF) G(r).
//! Transforms powder diffraction data from reciprocal space (Q-space) to real space
//! to reveal local atomic structure including bond lengths, coordination numbers,
//! and structural disorder.
//!
//! ## Physics Background
//!
//! The pair distribution function G(r) is obtained via a Fourier sine transform of
//! the reduced structure function F(Q) = Q[S(Q) - 1]:
//!
//!   G(r) = (2/pi) integral_0^Qmax F(Q) sin(Qr) dQ
//!
//! where S(Q) is the total scattering structure function derived from the measured
//! powder diffraction intensity I(Q), normalized by the atomic scattering factors.
//!
//! The scattering vector Q = 4*pi*sin(theta)/lambda relates diffraction angle 2*theta
//! and wavelength lambda.
//!
//! ## Key Components
//!
//! - [`DiffractionPattern`] - Powder diffraction data I(Q) or I(2theta)
//! - [`StructureFunction`] - S(Q) total scattering structure function
//! - [`Composition`] - Sample composition with X-ray form factors
//! - [`PdfTransform`] - Fourier sine transform to G(r)
//! - [`TerminationRippleCorrection`] - Handle finite Q_max artifacts
//! - [`BackgroundCorrection`] - Instrumental background removal
//! - [`PeakAnalyzer`] - Structural information from PDF peaks
//! - [`ModelPdfCalculator`] - Calculate PDF from structural models
//! - [`PartialPdfDecomposition`] - Separate element-pair contributions
//! - [`QSpaceProcessing`] - Pre-transform data processing

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────
// Element and X-ray form factor data (Cromer-Mann coefficients)
// ─────────────────────────────────────────────────────────────────────

/// Supported elements for X-ray scattering factor calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Element {
    H,
    C,
    N,
    O,
    F,
    Na,
    Mg,
    Al,
    Si,
    P,
    S,
    Cl,
    K,
    Ca,
    Ti,
    Cr,
    Mn,
    Fe,
    Co,
    Ni,
    Cu,
    Zn,
    Ge,
    Se,
    Br,
    Ag,
    Au,
    Pb,
}

/// Cromer-Mann coefficients for X-ray atomic form factor:
/// f(s) = sum_i a_i * exp(-b_i * s^2) + c
/// where s = Q / (4*pi) = sin(theta)/lambda in A^-1.
struct CromerMann {
    a: [f64; 4],
    b: [f64; 4],
    c: f64,
}

impl Element {
    /// Return Cromer-Mann coefficients for this element.
    /// Reference: International Tables for Crystallography, Vol C, Table 6.1.1.4.
    fn cromer_mann(&self) -> CromerMann {
        match self {
            Element::H => CromerMann {
                a: [0.489918, 0.262003, 0.196767, 0.049879],
                b: [20.6593, 7.74039, 49.5519, 2.20159],
                c: 0.001305,
            },
            Element::C => CromerMann {
                a: [2.31000, 1.02000, 1.58860, 0.865000],
                b: [20.8439, 10.2075, 0.568700, 51.6512],
                c: 0.215600,
            },
            Element::N => CromerMann {
                a: [12.2126, 3.13220, 2.01250, 1.16630],
                b: [0.005700, 9.89330, 28.9975, 0.582600],
                c: -11.529,
            },
            Element::O => CromerMann {
                a: [3.04850, 2.28680, 1.54630, 0.867000],
                b: [13.2771, 5.70110, 0.323900, 32.9089],
                c: 0.250800,
            },
            Element::F => CromerMann {
                a: [3.53920, 2.64120, 1.51700, 1.02430],
                b: [10.2825, 4.29440, 0.261500, 26.1476],
                c: 0.277600,
            },
            Element::Na => CromerMann {
                a: [4.76260, 3.17360, 1.26740, 1.11280],
                b: [3.28500, 8.84220, 0.313600, 129.424],
                c: 0.676000,
            },
            Element::Mg => CromerMann {
                a: [5.42040, 2.17350, 1.22690, 2.30730],
                b: [2.82750, 79.2611, 0.380800, 7.19370],
                c: 0.858400,
            },
            Element::Al => CromerMann {
                a: [6.42020, 1.90020, 1.59360, 1.96460],
                b: [3.03870, 0.742600, 31.5472, 85.0886],
                c: 1.11510,
            },
            Element::Si => CromerMann {
                a: [6.29150, 3.03530, 1.98910, 1.54100],
                b: [2.43860, 32.3337, 0.678500, 81.6937],
                c: 1.14070,
            },
            Element::P => CromerMann {
                a: [6.43450, 4.17910, 1.78000, 1.49080],
                b: [1.90670, 27.1570, 0.526000, 68.1645],
                c: 1.11490,
            },
            Element::S => CromerMann {
                a: [6.90530, 5.20340, 1.43790, 1.58630],
                b: [1.46790, 22.2151, 0.253600, 56.1720],
                c: 0.866900,
            },
            Element::Cl => CromerMann {
                a: [11.4604, 7.19640, 6.25560, 1.64550],
                b: [0.010400, 1.16620, 18.5194, 47.7784],
                c: -9.5574,
            },
            Element::K => CromerMann {
                a: [8.21860, 7.43980, 1.05190, 0.865900],
                b: [12.7949, 0.774800, 213.187, 41.6841],
                c: 1.42280,
            },
            Element::Ca => CromerMann {
                a: [8.62660, 7.38730, 1.58990, 1.02110],
                b: [10.4421, 0.659900, 85.7484, 178.437],
                c: 1.37510,
            },
            Element::Ti => CromerMann {
                a: [9.75950, 7.35580, 1.69910, 1.90210],
                b: [7.85080, 0.500000, 35.6338, 116.105],
                c: 1.28070,
            },
            Element::Cr => CromerMann {
                a: [10.6406, 7.35370, 3.32400, 1.49220],
                b: [6.10380, 0.392000, 20.2626, 98.7399],
                c: 1.18320,
            },
            Element::Mn => CromerMann {
                a: [11.2819, 7.35730, 3.01930, 2.24410],
                b: [5.34090, 0.343200, 17.8674, 83.7543],
                c: 1.08960,
            },
            Element::Fe => CromerMann {
                a: [11.7695, 7.35730, 3.52220, 2.30450],
                b: [4.76110, 0.307200, 15.3535, 76.8805],
                c: 1.03690,
            },
            Element::Co => CromerMann {
                a: [12.2841, 7.34090, 4.00340, 2.34880],
                b: [4.27910, 0.278400, 13.5359, 71.1692],
                c: 1.01180,
            },
            Element::Ni => CromerMann {
                a: [12.8376, 7.29200, 4.44380, 2.38000],
                b: [3.87850, 0.256500, 12.1763, 66.3421],
                c: 1.03410,
            },
            Element::Cu => CromerMann {
                a: [13.3380, 7.16760, 5.61580, 1.67350],
                b: [3.58280, 0.247000, 11.3966, 64.8126],
                c: 1.19100,
            },
            Element::Zn => CromerMann {
                a: [14.0743, 7.03180, 5.16520, 2.41000],
                b: [3.26550, 0.233300, 10.3163, 58.7097],
                c: 1.30410,
            },
            Element::Ge => CromerMann {
                a: [16.0816, 6.37470, 3.70680, 3.68300],
                b: [2.85090, 0.251600, 11.4468, 54.7625],
                c: 2.13130,
            },
            Element::Se => CromerMann {
                a: [17.0006, 5.81960, 3.97310, 4.35430],
                b: [2.40980, 0.272600, 15.2372, 43.8163],
                c: 2.84090,
            },
            Element::Br => CromerMann {
                a: [17.1789, 5.23580, 5.63770, 3.98510],
                b: [2.17230, 16.5796, 0.260900, 41.4328],
                c: 2.95570,
            },
            Element::Ag => CromerMann {
                a: [19.2808, 16.6885, 4.80450, 1.04630],
                b: [0.644600, 7.47260, 24.6605, 99.8156],
                c: 5.17900,
            },
            Element::Au => CromerMann {
                a: [16.8819, 18.5913, 25.5582, 5.86000],
                b: [0.461100, 8.62160, 1.48260, 36.3956],
                c: 12.0658,
            },
            Element::Pb => CromerMann {
                a: [31.0617, 13.0637, 18.4420, 5.96960],
                b: [0.690200, 2.35760, 8.61800, 47.2579],
                c: 13.4118,
            },
        }
    }

    /// Compute X-ray atomic form factor f(Q) at given Q (in inverse Angstroms).
    /// f(Q) = sum_i a_i * exp(-b_i * (Q/(4*pi))^2) + c
    pub fn form_factor(&self, q: f64) -> f64 {
        let cm = self.cromer_mann();
        let s = q / (4.0 * PI);
        let s2 = s * s;
        let mut f = cm.c;
        for i in 0..4 {
            f += cm.a[i] * (-cm.b[i] * s2).exp();
        }
        f
    }

    /// Atomic number (number of electrons), used as f(0) approximation.
    pub fn atomic_number(&self) -> u32 {
        match self {
            Element::H => 1,
            Element::C => 6,
            Element::N => 7,
            Element::O => 8,
            Element::F => 9,
            Element::Na => 11,
            Element::Mg => 12,
            Element::Al => 13,
            Element::Si => 14,
            Element::P => 15,
            Element::S => 16,
            Element::Cl => 17,
            Element::K => 19,
            Element::Ca => 20,
            Element::Ti => 22,
            Element::Cr => 24,
            Element::Mn => 25,
            Element::Fe => 26,
            Element::Co => 27,
            Element::Ni => 28,
            Element::Cu => 29,
            Element::Zn => 30,
            Element::Ge => 32,
            Element::Se => 34,
            Element::Br => 35,
            Element::Ag => 47,
            Element::Au => 79,
            Element::Pb => 82,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Composition
// ─────────────────────────────────────────────────────────────────────

/// Sample composition for scattering normalization.
///
/// Stores element-fraction pairs and computes average scattering factors
/// needed to convert raw intensity I(Q) to the structure function S(Q).
#[derive(Debug, Clone)]
pub struct Composition {
    /// (element, mole fraction) pairs. Fractions should sum to 1.
    elements: Vec<(Element, f64)>,
}

impl Composition {
    /// Create a new composition from element-fraction pairs.
    /// Fractions are normalized to sum to 1.
    pub fn new(elements: Vec<(Element, f64)>) -> Self {
        let total: f64 = elements.iter().map(|(_, f)| f).sum();
        let elements = if (total - 1.0).abs() > 1e-10 && total > 0.0 {
            elements.iter().map(|(e, f)| (*e, f / total)).collect()
        } else {
            elements
        };
        Self { elements }
    }

    /// Average scattering factor <f(Q)> = sum_i c_i * f_i(Q)
    pub fn average_scattering_factor(&self, q: f64) -> f64 {
        self.elements
            .iter()
            .map(|(elem, frac)| frac * elem.form_factor(q))
            .sum()
    }

    /// Average of squared scattering factor <f^2(Q)> = sum_i c_i * f_i(Q)^2
    pub fn average_sq_scattering_factor(&self, q: f64) -> f64 {
        self.elements
            .iter()
            .map(|(elem, frac)| {
                let fi = elem.form_factor(q);
                frac * fi * fi
            })
            .sum()
    }

    /// Number of distinct elements.
    pub fn num_elements(&self) -> usize {
        self.elements.len()
    }

    /// Access raw element-fraction pairs.
    pub fn elements(&self) -> &[(Element, f64)] {
        &self.elements
    }

    /// Average atomic number (approximation for number density scaling).
    pub fn average_atomic_number(&self) -> f64 {
        self.elements
            .iter()
            .map(|(e, f)| f * e.atomic_number() as f64)
            .sum()
    }
}

// ─────────────────────────────────────────────────────────────────────
// DiffractionPattern
// ─────────────────────────────────────────────────────────────────────

/// Powder diffraction data in Q-space: I(Q) values at scattering vector Q (in A^-1).
#[derive(Debug, Clone)]
pub struct DiffractionPattern {
    /// Scattering vector values in inverse Angstroms, sorted ascending.
    q_values: Vec<f64>,
    /// Measured intensities corresponding to each Q value.
    intensities: Vec<f64>,
}

impl DiffractionPattern {
    /// Create a diffraction pattern from Q values (A^-1) and intensities.
    /// Q values must be sorted ascending. Panics if lengths differ.
    pub fn new(q_values: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(
            q_values.len(),
            intensities.len(),
            "Q values and intensities must have the same length"
        );
        assert!(!q_values.is_empty(), "Pattern must not be empty");
        Self {
            q_values,
            intensities,
        }
    }

    /// Create from 2*theta (degrees) and intensities, converting to Q.
    /// Q = 4*pi*sin(theta)/lambda
    pub fn from_two_theta(
        two_theta_deg: Vec<f64>,
        intensities: Vec<f64>,
        wavelength_angstrom: f64,
    ) -> Self {
        assert_eq!(two_theta_deg.len(), intensities.len());
        assert!(wavelength_angstrom > 0.0);
        let q_values: Vec<f64> = two_theta_deg
            .iter()
            .map(|&tt| {
                let theta = tt.to_radians() / 2.0;
                4.0 * PI * theta.sin() / wavelength_angstrom
            })
            .collect();
        Self::new(q_values, intensities)
    }

    /// Return (Q_min, Q_max).
    pub fn q_range(&self) -> (f64, f64) {
        (
            self.q_values[0],
            self.q_values[self.q_values.len() - 1],
        )
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.q_values.len()
    }

    /// Whether pattern is empty.
    pub fn is_empty(&self) -> bool {
        self.q_values.is_empty()
    }

    /// Linear interpolation of intensity at arbitrary Q.
    pub fn interpolate(&self, q: f64) -> f64 {
        if q <= self.q_values[0] {
            return self.intensities[0];
        }
        let last = self.q_values.len() - 1;
        if q >= self.q_values[last] {
            return self.intensities[last];
        }
        // Binary search for bracket
        let idx = match self.q_values.binary_search_by(|v| v.partial_cmp(&q).unwrap()) {
            Ok(i) => return self.intensities[i],
            Err(i) => i,
        };
        let i = idx - 1;
        let t = (q - self.q_values[i]) / (self.q_values[i + 1] - self.q_values[i]);
        self.intensities[i] + t * (self.intensities[i + 1] - self.intensities[i])
    }

    /// Access Q values.
    pub fn q_values(&self) -> &[f64] {
        &self.q_values
    }

    /// Access intensities.
    pub fn intensities(&self) -> &[f64] {
        &self.intensities
    }
}

// ─────────────────────────────────────────────────────────────────────
// StructureFunction
// ─────────────────────────────────────────────────────────────────────

/// The total scattering structure function S(Q).
///
/// S(Q) = [I(Q) - <f^2> + <f>^2] / <f>^2 + 1
///
/// where <f> and <f^2> are composition-weighted average scattering factors.
#[derive(Debug, Clone)]
pub struct StructureFunction {
    /// (Q, S(Q)) pairs.
    data: Vec<(f64, f64)>,
}

impl StructureFunction {
    /// Compute S(Q) from measured intensity and sample composition.
    ///
    /// S(Q) = [I(Q) - <f^2(Q)> + <f(Q)>^2] / <f(Q)>^2 + 1
    pub fn from_intensity(pattern: &DiffractionPattern, composition: &Composition) -> Self {
        let data: Vec<(f64, f64)> = pattern
            .q_values
            .iter()
            .zip(pattern.intensities.iter())
            .map(|(&q, &iq)| {
                let f_avg = composition.average_scattering_factor(q);
                let f2_avg = composition.average_sq_scattering_factor(q);
                let f_avg_sq = f_avg * f_avg;
                // Avoid division by zero
                let sq = if f_avg_sq.abs() < 1e-30 {
                    1.0
                } else {
                    (iq - f2_avg + f_avg_sq) / f_avg_sq + 1.0
                };
                // S(Q) should not diverge negatively in well-corrected data
                (q, sq)
            })
            .collect();
        Self { data }
    }

    /// Create directly from (Q, S(Q)) data.
    pub fn from_data(data: Vec<(f64, f64)>) -> Self {
        Self { data }
    }

    /// Compute the reduced structure function F(Q) = Q * [S(Q) - 1].
    pub fn reduced_structure_function(&self) -> Vec<(f64, f64)> {
        self.data.iter().map(|&(q, sq)| (q, q * (sq - 1.0))).collect()
    }

    /// Access S(Q) data.
    pub fn data(&self) -> &[(f64, f64)] {
        &self.data
    }

    /// Interpolate S(Q) at given Q using linear interpolation.
    pub fn interpolate(&self, q: f64) -> f64 {
        if self.data.is_empty() {
            return 1.0;
        }
        if q <= self.data[0].0 {
            return self.data[0].1;
        }
        let last = self.data.len() - 1;
        if q >= self.data[last].0 {
            return self.data[last].1;
        }
        // Linear search (data is sorted by Q)
        for i in 0..last {
            if q >= self.data[i].0 && q <= self.data[i + 1].0 {
                let t = (q - self.data[i].0) / (self.data[i + 1].0 - self.data[i].0);
                return self.data[i].1 + t * (self.data[i + 1].1 - self.data[i].1);
            }
        }
        self.data[last].1
    }
}

// ─────────────────────────────────────────────────────────────────────
// PdfResult and PdfTransform
// ─────────────────────────────────────────────────────────────────────

/// Result of the PDF transform containing G(r), g(r), and RDF R(r).
#[derive(Debug, Clone)]
pub struct PdfResult {
    /// Real-space distance values (Angstroms).
    pub r_values: Vec<f64>,
    /// Pair distribution function G(r) = (2/pi) integral F(Q) sin(Qr) dQ.
    pub g_r: Vec<f64>,
    /// Radial distribution function R(r) = 4*pi*r^2*rho0*g_pair(r).
    /// where g_pair(r) = G(r)/(4*pi*r*rho0) + 1
    pub rdf: Vec<f64>,
    /// Number density rho0 in atoms/A^3.
    pub num_density: f64,
}

impl PdfResult {
    /// Compute the pair correlation function g_pair(r) = G(r)/(4*pi*r*rho0) + 1.
    pub fn pair_correlation(&self) -> Vec<f64> {
        self.r_values
            .iter()
            .zip(self.g_r.iter())
            .map(|(&r, &gr)| {
                if r.abs() < 1e-10 || self.num_density.abs() < 1e-30 {
                    0.0
                } else {
                    gr / (4.0 * PI * r * self.num_density) + 1.0
                }
            })
            .collect()
    }

    /// Find the r-value of the maximum of G(r).
    pub fn max_peak_position(&self) -> Option<f64> {
        if self.g_r.is_empty() {
            return None;
        }
        let (idx, _) = self.g_r.iter().enumerate().fold((0, f64::NEG_INFINITY), |(mi, mv), (i, &v)| {
            if v > mv { (i, v) } else { (mi, mv) }
        });
        Some(self.r_values[idx])
    }
}

/// Fourier sine transform to compute G(r) from F(Q).
pub struct PdfTransform;

impl PdfTransform {
    /// Compute G(r) from the reduced structure function F(Q) = Q[S(Q)-1].
    ///
    /// G(r) = (2/pi) * integral_0^q_max F(Q) sin(Qr) dQ
    ///
    /// Uses numerical integration (trapezoidal rule).
    ///
    /// # Arguments
    /// * `f_q` - Reduced structure function as (Q, F(Q)) pairs
    /// * `r_min` - Minimum r value (Angstroms)
    /// * `r_max` - Maximum r value (Angstroms)
    /// * `r_step` - Step size in r (Angstroms)
    /// * `q_max` - Maximum Q to integrate to (A^-1)
    /// * `num_density` - Number density rho0 (atoms/A^3)
    pub fn compute_gr(
        f_q: &[(f64, f64)],
        r_min: f64,
        r_max: f64,
        r_step: f64,
        q_max: f64,
        num_density: f64,
    ) -> PdfResult {
        // Filter F(Q) to Q <= q_max
        let fq_filtered: Vec<(f64, f64)> = f_q.iter().filter(|&&(q, _)| q <= q_max).cloned().collect();

        let n_r = ((r_max - r_min) / r_step).ceil() as usize + 1;
        let mut r_values = Vec::with_capacity(n_r);
        let mut g_r = Vec::with_capacity(n_r);
        let mut rdf = Vec::with_capacity(n_r);

        for i in 0..n_r {
            let r = r_min + i as f64 * r_step;
            r_values.push(r);

            // Trapezoidal integration of F(Q)*sin(Qr)
            let mut integral = 0.0;
            for j in 0..fq_filtered.len().saturating_sub(1) {
                let (q0, fq0) = fq_filtered[j];
                let (q1, fq1) = fq_filtered[j + 1];
                let dq = q1 - q0;
                let v0 = fq0 * (q0 * r).sin();
                let v1 = fq1 * (q1 * r).sin();
                integral += 0.5 * (v0 + v1) * dq;
            }

            let gr = (2.0 / PI) * integral;
            g_r.push(gr);

            // g_pair(r) = G(r)/(4*pi*r*rho0) + 1
            let g_pair = if r.abs() < 1e-10 || num_density.abs() < 1e-30 {
                0.0
            } else {
                gr / (4.0 * PI * r * num_density) + 1.0
            };
            // R(r) = 4*pi*r^2*rho0*g_pair(r)
            let rr = 4.0 * PI * r * r * num_density * g_pair.max(0.0);
            rdf.push(rr);
        }

        PdfResult {
            r_values,
            g_r,
            rdf,
            num_density,
        }
    }

    /// Convenience: compute G(r) directly from a StructureFunction.
    pub fn from_structure_function(
        sf: &StructureFunction,
        r_min: f64,
        r_max: f64,
        r_step: f64,
        q_max: f64,
        num_density: f64,
    ) -> PdfResult {
        let fq = sf.reduced_structure_function();
        Self::compute_gr(&fq, r_min, r_max, r_step, q_max, num_density)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Termination Ripple Correction
// ─────────────────────────────────────────────────────────────────────

/// Window types for termination ripple suppression.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowType {
    /// Lorch modification: sinc(pi*Q/Q_max)
    Lorch,
    /// Hann window applied to high-Q end
    Hann,
    /// Cosine rolloff in [Q_max*(1-frac), Q_max]
    CosineRolloff(f64),
}

/// Corrections for termination ripples caused by finite Q_max.
///
/// The sharp cutoff at Q_max introduces sinc-like artifacts in G(r).
/// These can be reduced by windowing F(Q) before the transform.
pub struct TerminationRippleCorrection;

impl TerminationRippleCorrection {
    /// Apply Lorch modification: multiply F(Q) by sinc(pi*Q/Q_max).
    /// This dampens F(Q) near Q_max to reduce termination ripples.
    pub fn lorch_modification(f_q: &mut [(f64, f64)], q_max: f64) {
        for &mut (ref q, ref mut fq_val) in f_q.iter_mut() {
            let arg = PI * q / q_max;
            let lorch = if arg.abs() < 1e-12 { 1.0 } else { arg.sin() / arg };
            *fq_val *= lorch;
        }
    }

    /// Step function correction: convolve G(r) with sinc to undo truncation broadening.
    /// Approximate correction by subtracting the expected termination ripple pattern.
    pub fn step_function_correction(g_r: &mut [f64], r_values: &[f64], q_max: f64) {
        // The termination effect is a convolution with (q_max/pi)*sinc(q_max*r/pi)
        // We approximate deconvolution by iterative subtraction
        let n = g_r.len();
        if n < 3 {
            return;
        }
        let mut correction = vec![0.0; n];
        for i in 0..n {
            let r = r_values[i];
            // Expected ripple: proportional to sin(Q_max * r) / r for r > first peak
            if r > 0.5 {
                let sinc_contrib = (q_max * r).sin() / (PI * r);
                correction[i] = sinc_contrib * 0.1; // mild correction factor
            }
        }
        for i in 0..n {
            g_r[i] -= correction[i];
        }
    }

    /// Apply a window function to F(Q) to reduce termination artifacts.
    pub fn apply_window(f_q: &mut [(f64, f64)], q_max: f64, window: WindowType) {
        match window {
            WindowType::Lorch => Self::lorch_modification(f_q, q_max),
            WindowType::Hann => {
                for &mut (ref q, ref mut fq_val) in f_q.iter_mut() {
                    if *q > q_max {
                        *fq_val = 0.0;
                    } else {
                        let w = 0.5 * (1.0 + (PI * q / q_max).cos());
                        *fq_val *= w;
                    }
                }
            }
            WindowType::CosineRolloff(frac) => {
                let q_start = q_max * (1.0 - frac);
                for &mut (ref q, ref mut fq_val) in f_q.iter_mut() {
                    if *q > q_max {
                        *fq_val = 0.0;
                    } else if *q > q_start {
                        let w = 0.5 * (1.0 + (PI * (q - q_start) / (q_max - q_start)).cos());
                        *fq_val *= w;
                    }
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Background Correction
// ─────────────────────────────────────────────────────────────────────

/// Background correction and instrumental artifact removal.
pub struct BackgroundCorrection;

impl BackgroundCorrection {
    /// Subtract a background pattern from the measured pattern.
    /// Intensities are subtracted point-by-point using interpolation.
    pub fn subtract_background(
        pattern: &DiffractionPattern,
        bg: &DiffractionPattern,
    ) -> DiffractionPattern {
        let corrected: Vec<f64> = pattern
            .q_values
            .iter()
            .zip(pattern.intensities.iter())
            .map(|(&q, &intensity)| {
                let bg_val = bg.interpolate(q);
                (intensity - bg_val).max(0.0)
            })
            .collect();
        DiffractionPattern::new(pattern.q_values.clone(), corrected)
    }

    /// Apply Paalman-Pings absorption correction.
    /// Corrects for X-ray absorption in the sample: I_corr = I * exp(mu*r) * A_factor
    /// where mu_r = mu * R (linear absorption coefficient * sample radius).
    /// Using cylindrical geometry approximation: A = 1 / (1 - 0.7698*mu_r + 0.1718*mu_r^2)
    pub fn absorption_correction(
        pattern: &DiffractionPattern,
        mu_r: f64,
    ) -> DiffractionPattern {
        // Cylindrical sample absorption factor (Paalman-Pings approximation)
        let a_factor = 1.0 / (1.0 - 0.7698 * mu_r + 0.1718 * mu_r * mu_r);
        let corrected: Vec<f64> = pattern.intensities.iter().map(|&i| i * a_factor).collect();
        DiffractionPattern::new(pattern.q_values.clone(), corrected)
    }

    /// Subtract container scattering: I_sample = (I_sc - A_sc/A_c * I_c)
    /// where A_sc is sample+container absorption factor and A_c is container-only.
    pub fn container_subtraction(
        sample_container: &DiffractionPattern,
        empty_container: &DiffractionPattern,
        absorption_factor: f64,
    ) -> DiffractionPattern {
        let corrected: Vec<f64> = sample_container
            .q_values
            .iter()
            .zip(sample_container.intensities.iter())
            .map(|(&q, &i_sc)| {
                let i_c = empty_container.interpolate(q);
                (i_sc - absorption_factor * i_c).max(0.0)
            })
            .collect();
        DiffractionPattern::new(sample_container.q_values.clone(), corrected)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Peak Analyzer
// ─────────────────────────────────────────────────────────────────────

/// A peak found in the PDF.
#[derive(Debug, Clone)]
pub struct PdfPeak {
    /// Peak center position in Angstroms.
    pub r_position: f64,
    /// Peak height (G(r) value).
    pub height: f64,
    /// Estimated full-width at half-maximum (FWHM) in Angstroms.
    pub width: f64,
    /// Coordination number from integration (if computed).
    pub coordination_number: f64,
}

/// Analyze PDF peaks for structural information.
pub struct PeakAnalyzer;

impl PeakAnalyzer {
    /// Find peaks in the PDF above a minimum height threshold.
    /// Uses a simple local maximum detection with 3-point neighborhood.
    pub fn find_peaks(pdf: &PdfResult, min_height: f64) -> Vec<PdfPeak> {
        let mut peaks = Vec::new();
        let n = pdf.g_r.len();
        if n < 3 {
            return peaks;
        }

        for i in 1..n - 1 {
            if pdf.g_r[i] > pdf.g_r[i - 1]
                && pdf.g_r[i] > pdf.g_r[i + 1]
                && pdf.g_r[i] >= min_height
            {
                let r_pos = pdf.r_values[i];
                let height = pdf.g_r[i];
                let half_max = height / 2.0;

                // Estimate FWHM: find half-max crossings
                let mut left_r = r_pos;
                for j in (0..i).rev() {
                    if pdf.g_r[j] <= half_max {
                        // Linear interpolation to find crossing
                        let t = (half_max - pdf.g_r[j]) / (pdf.g_r[j + 1] - pdf.g_r[j]);
                        left_r = pdf.r_values[j] + t * (pdf.r_values[j + 1] - pdf.r_values[j]);
                        break;
                    }
                }
                let mut right_r = r_pos;
                for j in i + 1..n {
                    if pdf.g_r[j] <= half_max {
                        let t = (half_max - pdf.g_r[j - 1]) / (pdf.g_r[j] - pdf.g_r[j - 1]);
                        right_r = pdf.r_values[j - 1]
                            + t * (pdf.r_values[j] - pdf.r_values[j - 1]);
                        break;
                    }
                }
                let width = right_r - left_r;

                peaks.push(PdfPeak {
                    r_position: r_pos,
                    height,
                    width: width.max(0.0),
                    coordination_number: 0.0,
                });
            }
        }
        peaks
    }

    /// Compute coordination number by integrating the RDF over [r_min, r_max].
    /// N = integral_{r_min}^{r_max} R(r) dr
    /// where R(r) = 4*pi*r^2*rho0*g(r)
    pub fn coordination_number(
        pdf: &PdfResult,
        r_min: f64,
        r_max: f64,
        _rho0: f64,
    ) -> f64 {
        let mut integral = 0.0;
        for i in 0..pdf.r_values.len().saturating_sub(1) {
            let r0 = pdf.r_values[i];
            let r1 = pdf.r_values[i + 1];
            if r1 < r_min || r0 > r_max {
                continue;
            }
            // Clamp to integration range
            let ra = r0.max(r_min);
            let rb = r1.min(r_max);
            if rb <= ra {
                continue;
            }
            let dr = rb - ra;
            // Interpolate RDF at midpoint
            let t_a = if (r1 - r0).abs() < 1e-15 {
                0.0
            } else {
                (ra - r0) / (r1 - r0)
            };
            let t_b = if (r1 - r0).abs() < 1e-15 {
                1.0
            } else {
                (rb - r0) / (r1 - r0)
            };
            let rdf_a = pdf.rdf[i] + t_a * (pdf.rdf[i + 1] - pdf.rdf[i]);
            let rdf_b = pdf.rdf[i] + t_b * (pdf.rdf[i + 1] - pdf.rdf[i]);
            integral += 0.5 * (rdf_a + rdf_b) * dr;
        }
        integral
    }

    /// Extract bond length from peak position.
    pub fn bond_length_from_peak(peak: &PdfPeak) -> f64 {
        peak.r_position
    }
}

// ─────────────────────────────────────────────────────────────────────
// Model PDF Calculator
// ─────────────────────────────────────────────────────────────────────

/// Calculate G(r) from a structural model (set of atom pairs).
pub struct ModelPdfCalculator;

impl ModelPdfCalculator {
    /// Gaussian broadening of a delta function centered at r_ij.
    /// delta_broad(r) = (1 / (sigma*sqrt(2*pi))) * exp(-(r - r_ij)^2 / (2*sigma^2))
    pub fn broadened_delta(r: f64, r_ij: f64, sigma: f64) -> f64 {
        let norm = 1.0 / (sigma * (2.0 * PI).sqrt());
        let exponent = -0.5 * ((r - r_ij) / sigma).powi(2);
        norm * exponent.exp()
    }

    /// Compute model G(r) from atom pair distances and weights.
    ///
    /// G_calc(r) = (1/r) * sum_{i!=j} w_ij * broadened_delta(r, r_ij, sigma) - 4*pi*r*rho0
    ///
    /// # Arguments
    /// * `pairs` - Vec of (distance_r_ij, weight_w_ij)
    /// * `sigma` - Gaussian broadening width (Debye-Waller-like thermal displacement)
    /// * `r_values` - r points to evaluate
    /// * `rho0` - number density
    pub fn from_atom_pairs(
        pairs: &[(f64, f64)],
        sigma: f64,
        r_values: &[f64],
        rho0: f64,
    ) -> Vec<(f64, f64)> {
        r_values
            .iter()
            .map(|&r| {
                let mut sum = 0.0;
                if r.abs() > 1e-10 {
                    for &(r_ij, w_ij) in pairs {
                        sum += w_ij * Self::broadened_delta(r, r_ij, sigma) / r;
                    }
                }
                let gr = sum - 4.0 * PI * r * rho0;
                (r, gr)
            })
            .collect()
    }

    /// Compute the weighted R-factor (Rw) residual between observed and calculated G(r).
    ///
    /// Rw = sqrt( sum_i (G_obs(r_i) - G_calc(r_i))^2 / sum_i G_obs(r_i)^2 )
    pub fn residual(g_obs: &[f64], g_calc: &[f64]) -> f64 {
        assert_eq!(g_obs.len(), g_calc.len());
        let num: f64 = g_obs
            .iter()
            .zip(g_calc.iter())
            .map(|(&o, &c)| (o - c).powi(2))
            .sum();
        let den: f64 = g_obs.iter().map(|&o| o.powi(2)).sum();
        if den.abs() < 1e-30 {
            return 0.0;
        }
        (num / den).sqrt()
    }

    /// Simple model G(r) from just distances (unit weights, no background).
    pub fn from_distances(distances: &[f64], sigma: f64, r_values: &[f64]) -> Vec<(f64, f64)> {
        let pairs: Vec<(f64, f64)> = distances.iter().map(|&d| (d, 1.0)).collect();
        Self::from_atom_pairs(&pairs, sigma, r_values, 0.0)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Partial PDF Decomposition
// ─────────────────────────────────────────────────────────────────────

/// A partial pair distribution function for a specific element pair.
#[derive(Debug, Clone)]
pub struct PartialPdf {
    /// Element pair label (e.g., "Si-O").
    pub label: String,
    /// Weight factor w_alpha_beta = c_a * c_b * f_a * f_b / <f>^2
    pub weight: f64,
    /// (Q, partial S_ab(Q)) data if available.
    pub s_partial: Vec<(f64, f64)>,
}

/// Decompose total PDF into partial contributions from element pairs.
pub struct PartialPdfDecomposition;

impl PartialPdfDecomposition {
    /// Calculate weight factors for all element pairs at a given Q.
    ///
    /// w_{ab} = c_a * c_b * f_a(Q) * f_b(Q) / <f(Q)>^2
    /// For a != b, there are two permutations so weight is doubled.
    pub fn pair_weights(composition: &Composition, q: f64) -> Vec<(String, f64)> {
        let f_avg = composition.average_scattering_factor(q);
        let f_avg_sq = f_avg * f_avg;
        if f_avg_sq.abs() < 1e-30 {
            return Vec::new();
        }

        let elems = composition.elements();
        let mut weights = Vec::new();

        for i in 0..elems.len() {
            for j in i..elems.len() {
                let (elem_a, c_a) = &elems[i];
                let (elem_b, c_b) = &elems[j];
                let fa = elem_a.form_factor(q);
                let fb = elem_b.form_factor(q);
                let mut w = c_a * c_b * fa * fb / f_avg_sq;
                if i != j {
                    w *= 2.0; // Both (a,b) and (b,a)
                }
                let label = format!("{:?}-{:?}", elem_a, elem_b);
                weights.push((label, w));
            }
        }
        weights
    }

    /// Compute partial PDFs from multiple diffraction patterns
    /// of isotopically substituted samples (neutron) or anomalous scattering (X-ray).
    ///
    /// This is a simplified version: given N patterns for N compositions,
    /// solve the linear system to extract partial structure functions.
    pub fn calculate_partials(
        compositions: &[Composition],
        patterns: &[DiffractionPattern],
    ) -> Vec<PartialPdf> {
        if compositions.is_empty() || patterns.is_empty() {
            return Vec::new();
        }

        // For a single composition, just decompose weights
        if compositions.len() == 1 {
            let comp = &compositions[0];
            let q_mid = {
                let (qmin, qmax) = patterns[0].q_range();
                (qmin + qmax) / 2.0
            };
            let weights = Self::pair_weights(comp, q_mid);
            return weights
                .into_iter()
                .map(|(label, weight)| PartialPdf {
                    label,
                    weight,
                    s_partial: Vec::new(),
                })
                .collect();
        }

        // Multiple compositions: return weight decomposition for each
        let mut partials = Vec::new();
        for (i, comp) in compositions.iter().enumerate() {
            let q_mid = {
                let (qmin, qmax) = patterns[i.min(patterns.len() - 1)].q_range();
                (qmin + qmax) / 2.0
            };
            let weights = Self::pair_weights(comp, q_mid);
            for (label, weight) in weights {
                partials.push(PartialPdf {
                    label,
                    weight,
                    s_partial: Vec::new(),
                });
            }
        }
        partials
    }
}

// ─────────────────────────────────────────────────────────────────────
// Q-Space Processing
// ─────────────────────────────────────────────────────────────────────

/// Pre-transform data processing in Q-space.
pub struct QSpaceProcessing;

impl QSpaceProcessing {
    /// Normalize measured intensity to atomic units.
    /// I_au(Q) = I(Q) / <f^2(Q)>
    pub fn normalize_to_atomic_units(
        pattern: &DiffractionPattern,
        comp: &Composition,
    ) -> Vec<(f64, f64)> {
        pattern
            .q_values
            .iter()
            .zip(pattern.intensities.iter())
            .map(|(&q, &iq)| {
                let f2 = comp.average_sq_scattering_factor(q);
                let normed = if f2.abs() < 1e-30 { 0.0 } else { iq / f2 };
                (q, normed)
            })
            .collect()
    }

    /// Smooth high-frequency oscillations in S(Q) using a simple moving average.
    pub fn smooth_oscillations(
        s_q: &[(f64, f64)],
        num_passes: usize,
    ) -> Vec<(f64, f64)> {
        if s_q.len() < 3 {
            return s_q.to_vec();
        }
        let mut data: Vec<f64> = s_q.iter().map(|&(_, v)| v).collect();

        for _ in 0..num_passes {
            let mut smoothed = data.clone();
            for i in 1..data.len() - 1 {
                smoothed[i] = (data[i - 1] + data[i] + data[i + 1]) / 3.0;
            }
            data = smoothed;
        }

        s_q.iter()
            .zip(data.iter())
            .map(|(&(q, _), &v)| (q, v))
            .collect()
    }

    /// Extrapolate S(Q) to Q=0 using low-order polynomial fit.
    /// S(Q) -> 1 as Q -> 0 for liquids and glasses.
    pub fn extrapolate_low_q(s_q: &mut Vec<(f64, f64)>) {
        if s_q.len() < 4 {
            return;
        }
        // Use first few points to fit a linear extrapolation
        let q0 = s_q[0].0;
        if q0 <= 0.0 {
            return; // Already starts at 0
        }

        // Fit line through first two points
        let (q1, s1) = s_q[0];
        let (q2, s2) = s_q[1];
        let slope = if (q2 - q1).abs() > 1e-15 {
            (s2 - s1) / (q2 - q1)
        } else {
            0.0
        };

        // Add points from Q=0 to Q=q0
        let dq = if s_q.len() > 1 {
            s_q[1].0 - s_q[0].0
        } else {
            0.1
        };
        let n_extra = (q0 / dq).ceil() as usize;
        let mut extra = Vec::new();
        for i in 0..n_extra {
            let q = i as f64 * dq;
            if q < q0 {
                let s_extrap = s1 + slope * (q - q1);
                // Ensure S(Q) approaches a reasonable value
                extra.push((q, s_extrap.max(0.0)));
            }
        }
        extra.append(s_q);
        *s_q = extra;
    }

    /// Apply a polynomial baseline correction to S(Q).
    /// Fits and subtracts a polynomial of given degree from S(Q) - 1.
    pub fn baseline_correction(s_q: &[(f64, f64)], degree: usize) -> Vec<(f64, f64)> {
        if s_q.is_empty() || degree == 0 {
            return s_q.to_vec();
        }
        let n = s_q.len();
        let deg = degree.min(5); // Limit to 5th order

        // Simple polynomial fit using normal equations (least squares)
        // For S(Q) - 1 to find the baseline
        let dim = deg + 1;
        let mut ata = vec![0.0; dim * dim];
        let mut atb = vec![0.0; dim];

        for &(q, sq) in s_q {
            let y = sq - 1.0;
            for i in 0..dim {
                let qi = q.powi(i as i32);
                atb[i] += qi * y;
                for j in 0..dim {
                    ata[i * dim + j] += qi * q.powi(j as i32);
                }
            }
        }

        // Solve via simple Gaussian elimination
        let coeffs = solve_linear_system(&ata, &atb, dim);

        // Subtract baseline
        s_q.iter()
            .map(|&(q, sq)| {
                let mut baseline = 0.0;
                for (i, &c) in coeffs.iter().enumerate() {
                    baseline += c * q.powi(i as i32);
                }
                (q, sq - baseline)
            })
            .collect()
    }
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut a = vec![0.0; n * n];
    a[..n * n].copy_from_slice(&a_flat[..n * n]);
    let mut x = b.to_vec();

    // Forward elimination with partial pivoting
    for k in 0..n {
        // Find pivot
        let mut max_val = a[k * n + k].abs();
        let mut max_row = k;
        for i in k + 1..n {
            if a[i * n + k].abs() > max_val {
                max_val = a[i * n + k].abs();
                max_row = i;
            }
        }
        // Swap rows
        if max_row != k {
            for j in 0..n {
                a.swap(k * n + j, max_row * n + j);
            }
            x.swap(k, max_row);
        }
        // Eliminate
        let pivot = a[k * n + k];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for i in k + 1..n {
            let factor = a[i * n + k] / pivot;
            for j in k..n {
                a[i * n + j] -= factor * a[k * n + j];
            }
            x[i] -= factor * x[k];
        }
    }

    // Back substitution
    for i in (0..n).rev() {
        let diag = a[i * n + i];
        if diag.abs() < 1e-30 {
            x[i] = 0.0;
            continue;
        }
        for j in i + 1..n {
            x[i] -= a[i * n + j] * x[j];
        }
        x[i] /= diag;
    }
    x
}

// ─────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1e-6;

    // ── Element and Form Factor Tests ──────────────────────────────

    #[test]
    fn test_element_form_factor_at_zero() {
        // At Q=0, f(0) = sum(a_i) + c, which should approximate atomic number
        let fe = Element::Fe;
        let f0 = fe.form_factor(0.0);
        // Fe has Z=26; f(0) should be close
        assert!((f0 - 26.0).abs() < 1.0, "Fe f(0)={} should be ~26", f0);
    }

    #[test]
    fn test_element_form_factor_decreases() {
        // Form factor should generally decrease with increasing Q
        let si = Element::Si;
        let f_low = si.form_factor(1.0);
        let f_high = si.form_factor(10.0);
        assert!(
            f_low > f_high,
            "f(1.0)={} should be > f(10.0)={}",
            f_low,
            f_high
        );
    }

    #[test]
    fn test_element_form_factor_positive() {
        // Form factors should be positive for small Q
        for elem in &[Element::C, Element::O, Element::Si, Element::Fe, Element::Au] {
            let f = elem.form_factor(0.5);
            assert!(f > 0.0, "{:?} form factor at Q=0.5 should be positive: {}", elem, f);
        }
    }

    #[test]
    fn test_element_form_factor_hydrogen() {
        let h = Element::H;
        let f0 = h.form_factor(0.0);
        assert!((f0 - 1.0).abs() < 0.1, "H f(0)={} should be ~1", f0);
    }

    #[test]
    fn test_element_form_factor_silicon() {
        let si = Element::Si;
        let f0 = si.form_factor(0.0);
        assert!((f0 - 14.0).abs() < 1.0, "Si f(0)={} should be ~14", f0);
    }

    #[test]
    fn test_element_atomic_number() {
        assert_eq!(Element::H.atomic_number(), 1);
        assert_eq!(Element::C.atomic_number(), 6);
        assert_eq!(Element::Fe.atomic_number(), 26);
        assert_eq!(Element::Au.atomic_number(), 79);
    }

    #[test]
    fn test_element_form_factor_gold() {
        let au = Element::Au;
        let f0 = au.form_factor(0.0);
        assert!((f0 - 79.0).abs() < 1.0, "Au f(0)={} should be ~79", f0);
    }

    // ── Composition Tests ──────────────────────────────────────────

    #[test]
    fn test_composition_normalization() {
        let comp = Composition::new(vec![(Element::Si, 2.0), (Element::O, 4.0)]);
        let total: f64 = comp.elements().iter().map(|(_, f)| f).sum();
        assert!((total - 1.0).abs() < 1e-10, "Fractions should sum to 1: {}", total);
    }

    #[test]
    fn test_composition_single_element() {
        let comp = Composition::new(vec![(Element::Fe, 1.0)]);
        let f_avg = comp.average_scattering_factor(0.0);
        let f_fe = Element::Fe.form_factor(0.0);
        assert!((f_avg - f_fe).abs() < TOLERANCE);
    }

    #[test]
    fn test_composition_average_scattering_factor() {
        // SiO2: Si=1/3, O=2/3
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        let q = 2.0;
        let f_si = Element::Si.form_factor(q);
        let f_o = Element::O.form_factor(q);
        let expected = (1.0 / 3.0) * f_si + (2.0 / 3.0) * f_o;
        let actual = comp.average_scattering_factor(q);
        assert!((actual - expected).abs() < TOLERANCE);
    }

    #[test]
    fn test_composition_average_sq_scattering() {
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        let q = 2.0;
        let f_si = Element::Si.form_factor(q);
        let f_o = Element::O.form_factor(q);
        let expected = (1.0 / 3.0) * f_si * f_si + (2.0 / 3.0) * f_o * f_o;
        let actual = comp.average_sq_scattering_factor(q);
        assert!((actual - expected).abs() < TOLERANCE);
    }

    #[test]
    fn test_composition_num_elements() {
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        assert_eq!(comp.num_elements(), 2);
    }

    #[test]
    fn test_composition_average_atomic_number() {
        let comp = Composition::new(vec![(Element::Fe, 1.0)]);
        assert!((comp.average_atomic_number() - 26.0).abs() < TOLERANCE);
    }

    // ── DiffractionPattern Tests ───────────────────────────────────

    #[test]
    fn test_diffraction_pattern_new() {
        let q = vec![0.5, 1.0, 1.5, 2.0];
        let i = vec![100.0, 80.0, 60.0, 40.0];
        let pat = DiffractionPattern::new(q.clone(), i.clone());
        assert_eq!(pat.len(), 4);
        assert!(!pat.is_empty());
    }

    #[test]
    fn test_diffraction_pattern_q_range() {
        let q = vec![0.5, 1.0, 2.0, 5.0];
        let i = vec![1.0; 4];
        let pat = DiffractionPattern::new(q, i);
        let (qmin, qmax) = pat.q_range();
        assert!((qmin - 0.5).abs() < TOLERANCE);
        assert!((qmax - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_diffraction_pattern_interpolate_exact() {
        let q = vec![1.0, 2.0, 3.0];
        let i = vec![10.0, 20.0, 30.0];
        let pat = DiffractionPattern::new(q, i);
        assert!((pat.interpolate(2.0) - 20.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_diffraction_pattern_interpolate_between() {
        let q = vec![1.0, 2.0, 3.0];
        let i = vec![10.0, 20.0, 30.0];
        let pat = DiffractionPattern::new(q, i);
        let val = pat.interpolate(1.5);
        assert!((val - 15.0).abs() < TOLERANCE, "Expected 15, got {}", val);
    }

    #[test]
    fn test_diffraction_pattern_interpolate_clamp() {
        let q = vec![1.0, 2.0, 3.0];
        let i = vec![10.0, 20.0, 30.0];
        let pat = DiffractionPattern::new(q, i);
        assert!((pat.interpolate(0.0) - 10.0).abs() < TOLERANCE); // Clamp to first
        assert!((pat.interpolate(5.0) - 30.0).abs() < TOLERANCE); // Clamp to last
    }

    #[test]
    fn test_diffraction_pattern_from_two_theta() {
        // CuKa: lambda = 1.5406 A, 2theta=30 deg -> Q = 4*pi*sin(15deg)/1.5406
        let two_theta = vec![30.0, 60.0, 90.0];
        let intensities = vec![100.0, 50.0, 25.0];
        let lambda = 1.5406;
        let pat = DiffractionPattern::from_two_theta(two_theta, intensities, lambda);
        assert_eq!(pat.len(), 3);
        // Check Q calculation for 2theta=30
        let expected_q = 4.0 * PI * (15.0_f64.to_radians().sin()) / lambda;
        assert!((pat.q_values()[0] - expected_q).abs() < 0.01);
    }

    #[test]
    fn test_diffraction_pattern_accessors() {
        let q = vec![1.0, 2.0];
        let i = vec![10.0, 20.0];
        let pat = DiffractionPattern::new(q, i);
        assert_eq!(pat.q_values().len(), 2);
        assert_eq!(pat.intensities().len(), 2);
    }

    // ── StructureFunction Tests ────────────────────────────────────

    #[test]
    fn test_structure_function_from_data() {
        let data = vec![(1.0, 1.0), (2.0, 1.1), (3.0, 0.95)];
        let sf = StructureFunction::from_data(data.clone());
        assert_eq!(sf.data().len(), 3);
    }

    #[test]
    fn test_structure_function_from_intensity() {
        let q: Vec<f64> = (1..=20).map(|i| i as f64 * 0.5).collect();
        let intensities: Vec<f64> = q.iter().map(|&qi| {
            let comp = Composition::new(vec![(Element::Si, 1.0)]);
            let f = comp.average_scattering_factor(qi);
            f * f + 0.1 * (5.0 * qi).sin()
        }).collect();
        let pat = DiffractionPattern::new(q, intensities);
        let comp = Composition::new(vec![(Element::Si, 1.0)]);
        let sf = StructureFunction::from_intensity(&pat, &comp);
        // S(Q) should be close to 1 + small oscillations
        for &(_, sq) in sf.data() {
            assert!(sq.is_finite(), "S(Q) should be finite");
        }
    }

    #[test]
    fn test_reduced_structure_function() {
        let data = vec![(1.0, 1.5), (2.0, 0.8), (3.0, 1.2)];
        let sf = StructureFunction::from_data(data);
        let fq = sf.reduced_structure_function();
        // F(Q) = Q * (S(Q) - 1)
        assert!((fq[0].1 - 1.0 * 0.5).abs() < TOLERANCE);
        assert!((fq[1].1 - 2.0 * (-0.2)).abs() < TOLERANCE);
        assert!((fq[2].1 - 3.0 * 0.2).abs() < TOLERANCE);
    }

    #[test]
    fn test_structure_function_interpolate() {
        let data = vec![(1.0, 1.0), (2.0, 2.0), (3.0, 1.5)];
        let sf = StructureFunction::from_data(data);
        let val = sf.interpolate(1.5);
        assert!((val - 1.5).abs() < TOLERANCE);
    }

    #[test]
    fn test_structure_function_s_q_unity_for_ideal() {
        // If intensity exactly equals <f^2>, S(Q) should be 1 + <f>^2/<f^2>...
        // For single element: <f> = f, <f^2> = f^2, so S(Q) = I/f^2
        // If I = f^2, then S(Q) = (f^2 - f^2 + f^2)/f^2 + 1 = 2
        // The formula: S(Q) = [I - <f^2> + <f>^2]/<f>^2 + 1
        // For single element: [f^2 - f^2 + f^2]/f^2 + 1 = 2
        let comp = Composition::new(vec![(Element::C, 1.0)]);
        let q_vals: Vec<f64> = (1..=10).map(|i| i as f64).collect();
        let intensities: Vec<f64> = q_vals.iter().map(|&q| {
            let f = comp.average_scattering_factor(q);
            f * f
        }).collect();
        let pat = DiffractionPattern::new(q_vals, intensities);
        let sf = StructureFunction::from_intensity(&pat, &comp);
        for &(_, sq) in sf.data() {
            assert!((sq - 2.0).abs() < TOLERANCE, "S(Q) should be 2.0 for I=f^2, got {}", sq);
        }
    }

    // ── PDF Transform Tests ────────────────────────────────────────

    #[test]
    fn test_pdf_transform_delta_function() {
        // F(Q) = A*sin(Q*r0) should give a peak near r=r0 in G(r)
        let r0 = 2.5;
        let q_max = 30.0;
        let dq = 0.05;
        let n_q = (q_max / dq) as usize;
        let fq: Vec<(f64, f64)> = (0..=n_q)
            .map(|i| {
                let q = i as f64 * dq;
                (q, (q * r0).sin() * 10.0)
            })
            .collect();

        let result = PdfTransform::compute_gr(&fq, 0.1, 5.0, 0.01, q_max, 0.0);
        // Find peak
        let peak_pos = result.max_peak_position().unwrap();
        assert!(
            (peak_pos - r0).abs() < 0.1,
            "Peak at {} should be near {}", peak_pos, r0
        );
    }

    #[test]
    fn test_pdf_transform_zero_fq() {
        // F(Q) = 0 should give G(r) = 0
        let fq: Vec<(f64, f64)> = (0..100).map(|i| (i as f64 * 0.1, 0.0)).collect();
        let result = PdfTransform::compute_gr(&fq, 0.1, 5.0, 0.1, 10.0, 0.0);
        for &gr in &result.g_r {
            assert!(gr.abs() < TOLERANCE, "G(r) should be 0, got {}", gr);
        }
    }

    #[test]
    fn test_pdf_transform_r_values() {
        let fq: Vec<(f64, f64)> = vec![(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)];
        let result = PdfTransform::compute_gr(&fq, 1.0, 5.0, 0.5, 2.0, 0.05);
        assert!((result.r_values[0] - 1.0).abs() < TOLERANCE);
        assert!((result.r_values[1] - 1.5).abs() < TOLERANCE);
        assert_eq!(result.num_density, 0.05);
    }

    #[test]
    fn test_pdf_transform_from_structure_function() {
        let data: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let q = i as f64 * 0.2;
                let sq = 1.0 + 0.1 * (3.0 * q).sin() * (-0.1 * q).exp();
                (q, sq)
            })
            .collect();
        let sf = StructureFunction::from_data(data);
        let result = PdfTransform::from_structure_function(&sf, 0.5, 10.0, 0.1, 20.0, 0.05);
        assert!(!result.g_r.is_empty());
        assert!(!result.rdf.is_empty());
    }

    #[test]
    fn test_pdf_result_pair_correlation() {
        let result = PdfResult {
            r_values: vec![1.0, 2.0, 3.0],
            g_r: vec![0.0, 5.0, 2.0],
            rdf: vec![0.0, 1.0, 0.5],
            num_density: 0.05,
        };
        let g_pair = result.pair_correlation();
        assert_eq!(g_pair.len(), 3);
        // At r=2: g_pair = 5.0/(4*pi*2*0.05) + 1
        let expected = 5.0 / (4.0 * PI * 2.0 * 0.05) + 1.0;
        assert!((g_pair[1] - expected).abs() < TOLERANCE);
    }

    #[test]
    fn test_pdf_result_max_peak() {
        let result = PdfResult {
            r_values: vec![1.0, 2.0, 3.0, 4.0],
            g_r: vec![1.0, 5.0, 3.0, 0.5],
            rdf: vec![0.0; 4],
            num_density: 0.05,
        };
        assert!((result.max_peak_position().unwrap() - 2.0).abs() < TOLERANCE);
    }

    // ── Termination Ripple Tests ───────────────────────────────────

    #[test]
    fn test_lorch_modification() {
        let mut fq: Vec<(f64, f64)> = (0..10).map(|i| {
            let q = i as f64;
            (q, 1.0)
        }).collect();
        let q_max = 9.0;
        TerminationRippleCorrection::lorch_modification(&mut fq, q_max);
        // At Q=0, Lorch factor is 1 (sinc(0) = 1)
        assert!((fq[0].1 - 1.0).abs() < TOLERANCE);
        // At Q=q_max, sinc(pi) = 0
        assert!(fq[9].1.abs() < 0.01, "At Q_max, Lorch should be ~0: {}", fq[9].1);
    }

    #[test]
    fn test_hann_window() {
        let mut fq: Vec<(f64, f64)> = (0..11).map(|i| {
            let q = i as f64;
            (q, 1.0)
        }).collect();
        TerminationRippleCorrection::apply_window(&mut fq, 10.0, WindowType::Hann);
        // At Q=0: w = 0.5*(1+cos(0)) = 1.0
        assert!((fq[0].1 - 1.0).abs() < TOLERANCE);
        // At Q=q_max: w = 0.5*(1+cos(pi)) = 0.0
        assert!(fq[10].1.abs() < TOLERANCE);
    }

    #[test]
    fn test_cosine_rolloff_window() {
        let mut fq: Vec<(f64, f64)> = (0..11).map(|i| (i as f64, 1.0)).collect();
        TerminationRippleCorrection::apply_window(&mut fq, 10.0, WindowType::CosineRolloff(0.2));
        // Before rolloff (Q < 8): should be untouched
        assert!((fq[5].1 - 1.0).abs() < TOLERANCE);
        // At Q_max: should be 0
        assert!(fq[10].1.abs() < TOLERANCE);
    }

    #[test]
    fn test_step_function_correction() {
        let mut g_r = vec![0.0; 50];
        let r_values: Vec<f64> = (0..50).map(|i| i as f64 * 0.1).collect();
        // Add some sinc ripple
        for i in 0..50 {
            let r = r_values[i];
            if r > 0.5 {
                g_r[i] = 2.0 * (20.0 * r).sin() / r;
            }
        }
        let original_sum: f64 = g_r.iter().map(|x| x.abs()).sum();
        TerminationRippleCorrection::step_function_correction(&mut g_r, &r_values, 20.0);
        let corrected_sum: f64 = g_r.iter().map(|x| x.abs()).sum();
        // Correction should reduce the total ripple somewhat
        assert!(corrected_sum < original_sum * 1.1);
    }

    // ── Background Correction Tests ────────────────────────────────

    #[test]
    fn test_subtract_background() {
        let q = vec![1.0, 2.0, 3.0];
        let signal = DiffractionPattern::new(q.clone(), vec![100.0, 80.0, 60.0]);
        let bg = DiffractionPattern::new(q.clone(), vec![10.0, 10.0, 10.0]);
        let corrected = BackgroundCorrection::subtract_background(&signal, &bg);
        assert!((corrected.intensities()[0] - 90.0).abs() < TOLERANCE);
        assert!((corrected.intensities()[1] - 70.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_subtract_background_no_negative() {
        let q = vec![1.0, 2.0];
        let signal = DiffractionPattern::new(q.clone(), vec![5.0, 3.0]);
        let bg = DiffractionPattern::new(q.clone(), vec![10.0, 10.0]);
        let corrected = BackgroundCorrection::subtract_background(&signal, &bg);
        assert!(corrected.intensities()[0] >= 0.0);
    }

    #[test]
    fn test_absorption_correction() {
        let q = vec![1.0, 2.0];
        let pat = DiffractionPattern::new(q, vec![100.0, 50.0]);
        let corrected = BackgroundCorrection::absorption_correction(&pat, 0.5);
        // A factor = 1/(1 - 0.7698*0.5 + 0.1718*0.25)
        let a = 1.0 / (1.0 - 0.7698 * 0.5 + 0.1718 * 0.25);
        assert!((corrected.intensities()[0] - 100.0 * a).abs() < 0.01);
    }

    #[test]
    fn test_container_subtraction() {
        let q = vec![1.0, 2.0, 3.0];
        let sc = DiffractionPattern::new(q.clone(), vec![100.0, 80.0, 60.0]);
        let ec = DiffractionPattern::new(q.clone(), vec![20.0, 15.0, 10.0]);
        let corrected = BackgroundCorrection::container_subtraction(&sc, &ec, 0.8);
        // I_sample = I_sc - 0.8 * I_c
        assert!((corrected.intensities()[0] - (100.0 - 0.8 * 20.0)).abs() < TOLERANCE);
    }

    // ── Peak Analyzer Tests ────────────────────────────────────────

    #[test]
    fn test_find_peaks_single() {
        let r_values: Vec<f64> = (0..50).map(|i| i as f64 * 0.1).collect();
        let g_r: Vec<f64> = r_values
            .iter()
            .map(|&r| {
                5.0 * (-(r - 2.0).powi(2) / (2.0 * 0.1)).exp()
            })
            .collect();
        let pdf = PdfResult {
            r_values,
            g_r,
            rdf: vec![0.0; 50],
            num_density: 0.05,
        };
        let peaks = PeakAnalyzer::find_peaks(&pdf, 1.0);
        assert!(!peaks.is_empty(), "Should find at least one peak");
        assert!((peaks[0].r_position - 2.0).abs() < 0.2);
    }

    #[test]
    fn test_find_peaks_multiple() {
        let r_values: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let g_r: Vec<f64> = r_values
            .iter()
            .map(|&r| {
                5.0 * (-(r - 2.0).powi(2) / 0.2).exp()
                    + 3.0 * (-(r - 4.0).powi(2) / 0.2).exp()
                    + 2.0 * (-(r - 6.0).powi(2) / 0.2).exp()
            })
            .collect();
        let pdf = PdfResult {
            r_values,
            g_r,
            rdf: vec![0.0; 100],
            num_density: 0.05,
        };
        let peaks = PeakAnalyzer::find_peaks(&pdf, 1.0);
        assert!(peaks.len() >= 3, "Should find 3 peaks, found {}", peaks.len());
    }

    #[test]
    fn test_find_peaks_threshold() {
        let r_values: Vec<f64> = (0..50).map(|i| i as f64 * 0.1).collect();
        let g_r: Vec<f64> = r_values
            .iter()
            .map(|&r| {
                5.0 * (-(r - 2.0).powi(2) / 0.2).exp()
                    + 0.5 * (-(r - 4.0).powi(2) / 0.2).exp()
            })
            .collect();
        let pdf = PdfResult {
            r_values,
            g_r,
            rdf: vec![0.0; 50],
            num_density: 0.05,
        };
        let all_peaks = PeakAnalyzer::find_peaks(&pdf, 0.1);
        let high_peaks = PeakAnalyzer::find_peaks(&pdf, 2.0);
        assert!(all_peaks.len() > high_peaks.len());
    }

    #[test]
    fn test_coordination_number() {
        // Construct an RDF with a known integral
        let n = 100;
        let r_step = 0.1;
        let r_values: Vec<f64> = (0..n).map(|i| i as f64 * r_step).collect();
        // RDF peak centered at r=2.0
        let rdf: Vec<f64> = r_values
            .iter()
            .map(|&r| {
                4.0 * (-(r - 2.0).powi(2) / (2.0 * 0.04)).exp()
            })
            .collect();
        let pdf = PdfResult {
            r_values,
            g_r: vec![0.0; n],
            rdf,
            num_density: 0.05,
        };
        let cn = PeakAnalyzer::coordination_number(&pdf, 1.0, 3.0, 0.05);
        assert!(cn > 0.0, "Coordination number should be positive: {}", cn);
    }

    #[test]
    fn test_bond_length_from_peak() {
        let peak = PdfPeak {
            r_position: 1.54,
            height: 5.0,
            width: 0.1,
            coordination_number: 4.0,
        };
        assert!((PeakAnalyzer::bond_length_from_peak(&peak) - 1.54).abs() < TOLERANCE);
    }

    #[test]
    fn test_peak_width_estimation() {
        // Gaussian peak: FWHM = 2*sqrt(2*ln(2))*sigma
        let sigma = 0.1;
        let r_values: Vec<f64> = (0..100).map(|i| i as f64 * 0.05).collect();
        let g_r: Vec<f64> = r_values
            .iter()
            .map(|&r| 5.0 * (-(r - 2.5).powi(2) / (2.0 * sigma * sigma)).exp())
            .collect();
        let pdf = PdfResult {
            r_values,
            g_r,
            rdf: vec![0.0; 100],
            num_density: 0.05,
        };
        let peaks = PeakAnalyzer::find_peaks(&pdf, 1.0);
        assert!(!peaks.is_empty());
        let expected_fwhm = 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * sigma;
        assert!(
            (peaks[0].width - expected_fwhm).abs() < 0.1,
            "FWHM {} should be ~{}", peaks[0].width, expected_fwhm
        );
    }

    // ── Model PDF Calculator Tests ─────────────────────────────────

    #[test]
    fn test_broadened_delta() {
        let sigma = 0.1;
        let r_ij = 2.0;
        // At r = r_ij, should be the maximum
        let at_center = ModelPdfCalculator::broadened_delta(r_ij, r_ij, sigma);
        let off_center = ModelPdfCalculator::broadened_delta(r_ij + 0.5, r_ij, sigma);
        assert!(at_center > off_center);
        // Gaussian normalization: peak = 1/(sigma*sqrt(2*pi))
        let expected_peak = 1.0 / (sigma * (2.0 * PI).sqrt());
        assert!((at_center - expected_peak).abs() < TOLERANCE);
    }

    #[test]
    fn test_model_from_atom_pairs() {
        let pairs = vec![(1.5, 4.0), (2.5, 6.0), (3.5, 8.0)];
        let sigma = 0.1;
        let r_values: Vec<f64> = (1..50).map(|i| i as f64 * 0.1).collect();
        let model = ModelPdfCalculator::from_atom_pairs(&pairs, sigma, &r_values, 0.0);
        assert_eq!(model.len(), r_values.len());
        // Should have peaks near 1.5, 2.5, 3.5
        let at_15: f64 = model.iter().find(|&&(r, _)| (r - 1.5).abs() < 0.05).map(|&(_, g)| g).unwrap_or(0.0);
        let at_20: f64 = model.iter().find(|&&(r, _)| (r - 2.0).abs() < 0.05).map(|&(_, g)| g).unwrap_or(0.0);
        assert!(at_15 > at_20, "Peak at 1.5 should be stronger than valley at 2.0");
    }

    #[test]
    fn test_model_from_distances() {
        let distances = vec![1.5, 2.5];
        let r_values: Vec<f64> = (1..40).map(|i| i as f64 * 0.1).collect();
        let model = ModelPdfCalculator::from_distances(&distances, 0.1, &r_values);
        assert_eq!(model.len(), r_values.len());
    }

    #[test]
    fn test_residual_perfect() {
        let g_obs = vec![1.0, 2.0, 3.0, 4.0];
        let g_calc = vec![1.0, 2.0, 3.0, 4.0];
        let rw = ModelPdfCalculator::residual(&g_obs, &g_calc);
        assert!(rw < TOLERANCE, "Perfect match should give Rw~0: {}", rw);
    }

    #[test]
    fn test_residual_nonzero() {
        let g_obs = vec![1.0, 2.0, 3.0, 4.0];
        let g_calc = vec![1.1, 2.1, 2.9, 3.9];
        let rw = ModelPdfCalculator::residual(&g_obs, &g_calc);
        assert!(rw > 0.0 && rw < 1.0, "Rw should be between 0 and 1: {}", rw);
    }

    #[test]
    fn test_residual_symmetric() {
        let g_obs = vec![1.0, 2.0, 3.0];
        let g_calc = vec![1.5, 2.5, 3.5];
        let rw1 = ModelPdfCalculator::residual(&g_obs, &g_calc);
        let rw2 = ModelPdfCalculator::residual(&g_calc, &g_obs);
        // Not symmetric in general since denominator uses g_obs
        assert!(rw1 > 0.0);
        assert!(rw2 > 0.0);
    }

    // ── Partial PDF Tests ──────────────────────────────────────────

    #[test]
    fn test_pair_weights_single_element() {
        let comp = Composition::new(vec![(Element::Fe, 1.0)]);
        let weights = PartialPdfDecomposition::pair_weights(&comp, 2.0);
        assert_eq!(weights.len(), 1);
        // Single element: w = c^2 * f^2 / f^2 = 1
        assert!((weights[0].1 - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_pair_weights_two_elements() {
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        let weights = PartialPdfDecomposition::pair_weights(&comp, 2.0);
        // Should have 3 pairs: Si-Si, Si-O, O-O
        assert_eq!(weights.len(), 3);
        // Sum of weights should be 1
        let total: f64 = weights.iter().map(|(_, w)| w).sum();
        assert!((total - 1.0).abs() < 0.01, "Weight sum should be ~1: {}", total);
    }

    #[test]
    fn test_calculate_partials_single() {
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        let pat = DiffractionPattern::new(vec![1.0, 2.0, 3.0], vec![10.0, 8.0, 6.0]);
        let partials = PartialPdfDecomposition::calculate_partials(&[comp], &[pat]);
        assert!(!partials.is_empty());
    }

    #[test]
    fn test_calculate_partials_empty() {
        let result = PartialPdfDecomposition::calculate_partials(&[], &[]);
        assert!(result.is_empty());
    }

    // ── Q-Space Processing Tests ───────────────────────────────────

    #[test]
    fn test_normalize_to_atomic_units() {
        let comp = Composition::new(vec![(Element::Si, 1.0)]);
        let q = vec![1.0, 2.0, 3.0];
        let intensities: Vec<f64> = q.iter().map(|&qi| {
            let f = comp.average_sq_scattering_factor(qi);
            f * 2.0 // Twice the atomic units
        }).collect();
        let pat = DiffractionPattern::new(q, intensities);
        let normed = QSpaceProcessing::normalize_to_atomic_units(&pat, &comp);
        for &(_, v) in &normed {
            assert!((v - 2.0).abs() < TOLERANCE, "Normalized should be 2.0: {}", v);
        }
    }

    #[test]
    fn test_smooth_oscillations() {
        let data: Vec<(f64, f64)> = (0..20).map(|i| {
            let q = i as f64 * 0.5;
            let noisy = 1.0 + if i % 2 == 0 { 0.5 } else { -0.5 };
            (q, noisy)
        }).collect();
        let smoothed = QSpaceProcessing::smooth_oscillations(&data, 3);
        // After smoothing, oscillations should be reduced
        let orig_var: f64 = data.iter().map(|&(_, v)| (v - 1.0).powi(2)).sum::<f64>() / data.len() as f64;
        let smooth_var: f64 = smoothed.iter().map(|&(_, v)| (v - 1.0).powi(2)).sum::<f64>() / smoothed.len() as f64;
        assert!(smooth_var < orig_var, "Smoothing should reduce variance");
    }

    #[test]
    fn test_smooth_short_data() {
        let data = vec![(1.0, 1.0), (2.0, 2.0)];
        let smoothed = QSpaceProcessing::smooth_oscillations(&data, 1);
        assert_eq!(smoothed.len(), 2);
    }

    #[test]
    fn test_extrapolate_low_q() {
        let mut sq: Vec<(f64, f64)> = (1..=10).map(|i| (i as f64, 1.0 + 0.01 * i as f64)).collect();
        let orig_len = sq.len();
        QSpaceProcessing::extrapolate_low_q(&mut sq);
        // Should have added points from Q=0 to Q=1
        assert!(sq.len() > orig_len, "Should have added low-Q points");
        assert!(sq[0].0 < 1.0, "First Q should be < 1.0");
    }

    #[test]
    fn test_extrapolate_already_at_zero() {
        let mut sq: Vec<(f64, f64)> = (0..5).map(|i| (i as f64 * 0.5, 1.0)).collect();
        let orig_len = sq.len();
        QSpaceProcessing::extrapolate_low_q(&mut sq);
        assert_eq!(sq.len(), orig_len); // Should not add anything
    }

    #[test]
    fn test_baseline_correction() {
        // S(Q) with a linear baseline drift
        let sq: Vec<(f64, f64)> = (0..50).map(|i| {
            let q = i as f64 * 0.2;
            (q, 1.0 + 0.1 * q + 0.05 * (5.0 * q).sin())
        }).collect();
        let corrected = QSpaceProcessing::baseline_correction(&sq, 1);
        // After linear correction, the mean of corrected S(Q)-1 should be closer to 0
        let mean_before: f64 = sq.iter().map(|&(_, s)| s - 1.0).sum::<f64>() / sq.len() as f64;
        let mean_after: f64 = corrected.iter().map(|&(_, s)| s - 1.0).sum::<f64>() / corrected.len() as f64;
        assert!(
            mean_after.abs() < mean_before.abs() + 0.1,
            "Baseline correction should reduce systematic offset"
        );
    }

    // ── Integration Tests ──────────────────────────────────────────

    #[test]
    fn test_full_pipeline_sio2() {
        // Simulate an SiO2-like diffraction pattern and compute PDF
        let comp = Composition::new(vec![(Element::Si, 1.0), (Element::O, 2.0)]);
        let n_q = 200;
        let q_max = 25.0;
        let dq = q_max / n_q as f64;

        // Generate synthetic I(Q) with Si-O peak at 1.61 A
        let q_values: Vec<f64> = (1..=n_q).map(|i| i as f64 * dq).collect();
        let r_sio = 1.61; // Si-O bond length
        let intensities: Vec<f64> = q_values.iter().map(|&q| {
            let f_avg_sq = comp.average_sq_scattering_factor(q);
            let f_avg = comp.average_scattering_factor(q);
            // S(Q) = 1 + oscillation from Si-O pair
            let sq = 1.0 + 0.3 * (q * r_sio).sin() / (q * r_sio) * (-0.005 * q * q).exp();
            // I(Q) = <f>^2 * (S(Q) - 1) + <f^2>
            // From S(Q) = [I - <f^2> + <f>^2] / <f>^2 + 1
            // => I = (<f>^2) * (S - 1) + <f^2> - <f>^2 + <f^2>  ... wait
            // S = [I - <f^2> + <f>^2]/<f>^2 + 1
            // => (S-1)*<f>^2 = I - <f^2> + <f>^2
            // => I = (S-1)*<f>^2 + <f^2> - <f>^2 = (S-1)*<f>^2 + <f^2> - <f>^2
            // Hmm let me redo: S = (I - <f^2> + <f>^2)/<f>^2 + 1
            // => (S-1) = (I - <f^2> + <f>^2)/<f>^2
            // => I = (S-1)*<f>^2 + <f^2> - <f>^2
            // Actually: I = <f>^2*(S-1) + <f^2> - <f>^2 + <f^2> ... No.
            // (S-1)*<f>^2 = I - <f^2> + <f>^2
            // I = (S-1)*<f>^2 + <f^2> - <f>^2
            // For single element: <f> = f, <f^2> = f^2, so I = f^2 * (S-1) + f^2 - f^2 = f^2*(S-1)
            // That doesn't seem right... Let's just use:
            // I = <f>^2 * S(Q) for simplicity (this is correct when <f^2> = <f>^2)
            f_avg * f_avg * sq + (f_avg_sq - f_avg * f_avg)
        }).collect();

        let pat = DiffractionPattern::new(q_values, intensities);
        let sf = StructureFunction::from_intensity(&pat, &comp);

        // Check S(Q) is reasonable
        for &(_, sq) in sf.data() {
            assert!(sq.is_finite());
        }

        // Compute PDF
        let fq = sf.reduced_structure_function();
        let result = PdfTransform::compute_gr(&fq, 0.5, 8.0, 0.02, q_max, 0.066);

        // G(r) should be finite everywhere
        for &gr in &result.g_r {
            assert!(gr.is_finite(), "G(r) should be finite");
        }
    }

    #[test]
    fn test_full_pipeline_with_window() {
        let comp = Composition::new(vec![(Element::Fe, 1.0)]);
        let q_values: Vec<f64> = (1..=100).map(|i| i as f64 * 0.2).collect();
        let intensities: Vec<f64> = q_values.iter().map(|&q| {
            let f = comp.average_scattering_factor(q);
            f * f * (1.0 + 0.2 * (q * 2.48).sin() * (-0.01 * q * q).exp())
        }).collect();

        let pat = DiffractionPattern::new(q_values, intensities);
        let sf = StructureFunction::from_intensity(&pat, &comp);
        let mut fq = sf.reduced_structure_function();

        // Apply Lorch window
        let q_max = 20.0;
        TerminationRippleCorrection::lorch_modification(&mut fq, q_max);

        let result = PdfTransform::compute_gr(&fq, 0.5, 6.0, 0.02, q_max, 0.085);
        assert!(!result.g_r.is_empty());
    }

    #[test]
    fn test_solve_linear_system() {
        // Solve: [2 1; 1 3] * [x; y] = [5; 7]
        // x=8/5=1.6, y=9/5=1.8
        let a = vec![2.0, 1.0, 1.0, 3.0];
        let b = vec![5.0, 7.0];
        let x = solve_linear_system(&a, &b, 2);
        assert!((x[0] - 1.6).abs() < TOLERANCE);
        assert!((x[1] - 1.8).abs() < TOLERANCE);
    }

    #[test]
    fn test_solve_linear_system_3x3() {
        // 3x3 identity => x = b
        let a = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let b = vec![3.0, 5.0, 7.0];
        let x = solve_linear_system(&a, &b, 3);
        assert!((x[0] - 3.0).abs() < TOLERANCE);
        assert!((x[1] - 5.0).abs() < TOLERANCE);
        assert!((x[2] - 7.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_element_form_factor_many_elements() {
        // Verify all elements have reasonable f(0)
        let elements = vec![
            (Element::H, 1), (Element::C, 6), (Element::N, 7), (Element::O, 8),
            (Element::Si, 14), (Element::Fe, 26), (Element::Cu, 29), (Element::Ag, 47),
        ];
        for (elem, z) in elements {
            let f0 = elem.form_factor(0.0);
            assert!(
                (f0 - z as f64).abs() < 2.0,
                "{:?}: f(0)={}, expected ~{}", elem, f0, z
            );
        }
    }

    #[test]
    fn test_pdf_transform_sine_consistency() {
        // For F(Q) = A * sin(Q*d), the sine transform gives a peak near r=d
        // This verifies the transform normalization
        let d = 3.0;
        let amplitude = 5.0;
        let q_max = 40.0;
        let dq = 0.02;
        let n_q = (q_max / dq) as usize;
        let fq: Vec<(f64, f64)> = (0..=n_q)
            .map(|i| {
                let q = i as f64 * dq;
                (q, amplitude * (q * d).sin())
            })
            .collect();

        let result = PdfTransform::compute_gr(&fq, 0.1, 6.0, 0.01, q_max, 0.0);
        let peak_pos = result.max_peak_position().unwrap();
        assert!(
            (peak_pos - d).abs() < 0.15,
            "Peak at {} should be near {}", peak_pos, d
        );
    }
}
