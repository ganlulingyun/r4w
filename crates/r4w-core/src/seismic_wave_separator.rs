//! P/S/surface wave separation from multi-component seismic data.
//!
//! This module implements seismic wave separation using three complementary techniques:
//!
//! 1. **Polarization analysis** -- Eigendecomposition of the 3x3 covariance matrix of
//!    three-component (Z, N, E) seismic data within sliding time windows. The ratio and
//!    orientation of eigenvalues/eigenvectors identify P-waves (linear, near-vertical),
//!    S-waves (linear, near-horizontal), Rayleigh waves (retrograde elliptical in the
//!    vertical-radial plane), and Love waves (transverse-horizontal).
//!
//! 2. **Hodograph analysis** -- Fits polarization ellipses to particle-motion diagrams
//!    (hodographs) in the Z-R and T planes, extracting ellipticity, tilt angle, and
//!    planarity to discriminate between wave types.
//!
//! 3. **Frequency-wavenumber (f-k) filtering** -- Transforms multi-channel array data
//!    into the f-k domain via 2D DFT, applies fan or velocity-band filters to isolate
//!    waves by apparent velocity, and inverse-transforms to recover separated traces.
//!
//! # Theory
//!
//! ## Polarization
//!
//! For a three-component seismogram `[Z(t), N(t), E(t)]`, the covariance matrix is:
//!
//! ```text
//! C = (1/N) * sum_i  x_i * x_i^T
//! ```
//!
//! where `x_i = [Z_i, N_i, E_i]^T`. The eigenvalues `lambda_1 >= lambda_2 >= lambda_3`
//! quantify the polarization:
//!
//! - **Rectilinearity** `RL = 1 - (lambda_2 + lambda_3) / (2 * lambda_1)` is high for
//!   body waves (P and S) and low for surface waves.
//! - **Planarity** `PL = 1 - 2 * lambda_3 / (lambda_1 + lambda_2)` is high for Rayleigh
//!   waves and low for noise.
//! - **Ellipticity** `EL = lambda_2 / lambda_1` distinguishes P (low) from Rayleigh (moderate).
//!
//! ## Hodograph
//!
//! A hodograph plots one component against another over a short time window. An ellipse
//! fit yields semi-major axis `a`, semi-minor axis `b`, and tilt angle `theta`:
//!
//! - P-waves: nearly linear (b/a ~ 0), tilt ~ incidence angle from vertical.
//! - Rayleigh waves: retrograde ellipse in Z-R plane (b/a ~ 0.5-0.7).
//! - Love waves: linear in transverse plane only.
//!
//! ## F-K Filtering
//!
//! Given an array of receivers with spacing `dx`, the 2D DFT over time and space maps
//! to frequency `f` and wavenumber `k`. A wave with apparent velocity `v` lies along
//! `f = v * k`. Fan filters pass or reject sectors in f-k space to separate wave types
//! by velocity.
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_wave_separator::{
//!     PolarizationAnalyzer, SeismicWaveType, PolarizationParams,
//! };
//!
//! let params = PolarizationParams {
//!     window_samples: 50,
//!     hop_samples: 10,
//!     rectilinearity_threshold: 0.7,
//! };
//! let analyzer = PolarizationAnalyzer::new(params);
//!
//! // Synthetic P-wave: predominantly vertical motion
//! let n = 200;
//! let z: Vec<f64> = (0..n).map(|i| (2.0 * std::f64::consts::PI * i as f64 / 20.0).sin()).collect();
//! let north: Vec<f64> = (0..n).map(|i| 0.1 * (2.0 * std::f64::consts::PI * i as f64 / 20.0).sin()).collect();
//! let east: Vec<f64> = (0..n).map(|i| 0.05 * (2.0 * std::f64::consts::PI * i as f64 / 20.0).sin()).collect();
//!
//! let results = analyzer.analyze(&z, &north, &east);
//! // Most windows should identify P-wave polarization
//! assert!(!results.is_empty());
//! ```

use std::f64::consts::PI;

// ─── Wave Types ─────────────────────────────────────────────────────────────

/// Classification of seismic wave types based on polarization and velocity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SeismicWaveType {
    /// Primary (compressional) wave -- fastest body wave, particle motion parallel
    /// to propagation direction. Typical crustal velocity 5-8 km/s.
    P,
    /// Secondary (shear) wave -- slower body wave, particle motion perpendicular
    /// to propagation direction. Typical crustal velocity 3-4.5 km/s.
    S,
    /// Rayleigh wave -- surface wave with retrograde elliptical particle motion
    /// in the vertical-radial plane. Velocity ~0.92 * Vs.
    Rayleigh,
    /// Love wave -- surface wave with transverse horizontal particle motion,
    /// requires a low-velocity surface layer. Velocity between Vs (surface) and Vs (half-space).
    Love,
    /// Noise or unclassifiable signal.
    Noise,
}

// ─── Polarization Analysis ──────────────────────────────────────────────────

/// Parameters for polarization analysis.
#[derive(Debug, Clone)]
pub struct PolarizationParams {
    /// Number of samples per analysis window.
    pub window_samples: usize,
    /// Number of samples to advance between consecutive windows (hop size).
    pub hop_samples: usize,
    /// Minimum rectilinearity to classify as a body wave (P or S). Range [0, 1].
    pub rectilinearity_threshold: f64,
}

impl Default for PolarizationParams {
    fn default() -> Self {
        Self {
            window_samples: 64,
            hop_samples: 16,
            rectilinearity_threshold: 0.6,
        }
    }
}

/// Result of polarization analysis for one time window.
#[derive(Debug, Clone)]
pub struct PolarizationResult {
    /// Center sample index of the analysis window.
    pub center_sample: usize,
    /// Eigenvalues sorted descending: `[lambda_1, lambda_2, lambda_3]`.
    pub eigenvalues: [f64; 3],
    /// Principal eigenvector (direction of maximum polarization) `[z, n, e]`.
    pub principal_direction: [f64; 3],
    /// Rectilinearity: `1 - (lambda_2 + lambda_3) / (2 * lambda_1)`. High for body waves.
    pub rectilinearity: f64,
    /// Planarity: `1 - 2 * lambda_3 / (lambda_1 + lambda_2)`. High for Rayleigh.
    pub planarity: f64,
    /// Ellipticity: `lambda_2 / lambda_1`. Low for P, moderate for Rayleigh.
    pub ellipticity: f64,
    /// Classified wave type for this window.
    pub wave_type: SeismicWaveType,
    /// Incidence angle from vertical (degrees). Small for near-vertical P arrivals.
    pub incidence_angle_deg: f64,
    /// Back azimuth (degrees from north, clockwise). Direction from which the wave arrives.
    pub back_azimuth_deg: f64,
}

/// Polarization analyzer for three-component seismic data.
///
/// Computes the 3x3 covariance matrix within sliding windows and uses
/// eigendecomposition to extract polarization attributes and classify wave types.
#[derive(Debug, Clone)]
pub struct PolarizationAnalyzer {
    params: PolarizationParams,
}

impl PolarizationAnalyzer {
    /// Create a new polarization analyzer with the given parameters.
    pub fn new(params: PolarizationParams) -> Self {
        Self { params }
    }

    /// Analyze three-component seismogram and return polarization results per window.
    ///
    /// # Arguments
    /// - `z` -- vertical component (positive up)
    /// - `north` -- north-south component (positive north)
    /// - `east` -- east-west component (positive east)
    ///
    /// All three slices must have the same length.
    pub fn analyze(&self, z: &[f64], north: &[f64], east: &[f64]) -> Vec<PolarizationResult> {
        let n = z.len().min(north.len()).min(east.len());
        if n < self.params.window_samples {
            return Vec::new();
        }

        let mut results = Vec::new();
        let mut start = 0;
        while start + self.params.window_samples <= n {
            let end = start + self.params.window_samples;
            let result = self.analyze_window(z, north, east, start, end);
            results.push(result);
            start += self.params.hop_samples;
        }
        results
    }

    /// Analyze a single window of three-component data.
    fn analyze_window(
        &self,
        z: &[f64],
        north: &[f64],
        east: &[f64],
        start: usize,
        end: usize,
    ) -> PolarizationResult {
        let center = (start + end) / 2;
        let wlen = end - start;

        // Compute means
        let (mut mz, mut mn, mut me) = (0.0, 0.0, 0.0);
        for i in start..end {
            mz += z[i];
            mn += north[i];
            me += east[i];
        }
        let inv_n = 1.0 / wlen as f64;
        mz *= inv_n;
        mn *= inv_n;
        me *= inv_n;

        // Compute 3x3 covariance matrix (symmetric)
        let mut cov = [[0.0f64; 3]; 3];
        for i in start..end {
            let dz = z[i] - mz;
            let dn = north[i] - mn;
            let de = east[i] - me;
            let d = [dz, dn, de];
            for r in 0..3 {
                for c in r..3 {
                    cov[r][c] += d[r] * d[c];
                }
            }
        }
        for r in 0..3 {
            for c in r..3 {
                cov[r][c] *= inv_n;
                if c != r {
                    cov[c][r] = cov[r][c];
                }
            }
        }

        // Eigendecomposition via Jacobi iteration
        let (eigenvalues, eigenvectors) = symmetric_eigen_3x3(cov);

        // Sort descending by eigenvalue
        let mut idx = [0usize, 1, 2];
        idx.sort_by(|&a, &b| eigenvalues[b].partial_cmp(&eigenvalues[a]).unwrap_or(std::cmp::Ordering::Equal));

        let lam = [
            eigenvalues[idx[0]].max(0.0),
            eigenvalues[idx[1]].max(0.0),
            eigenvalues[idx[2]].max(0.0),
        ];
        let principal = [
            eigenvectors[0][idx[0]],
            eigenvectors[1][idx[0]],
            eigenvectors[2][idx[0]],
        ];

        // Polarization attributes
        let rectilinearity = if lam[0] > 1e-30 {
            1.0 - (lam[1] + lam[2]) / (2.0 * lam[0])
        } else {
            0.0
        };

        let planarity = if (lam[0] + lam[1]) > 1e-30 {
            1.0 - 2.0 * lam[2] / (lam[0] + lam[1])
        } else {
            0.0
        };

        let ellipticity = if lam[0] > 1e-30 {
            lam[1] / lam[0]
        } else {
            0.0
        };

        // Incidence angle from vertical (Z component of principal direction)
        let pz_abs = principal[0].abs();
        let pmag = (principal[0] * principal[0] + principal[1] * principal[1] + principal[2] * principal[2]).sqrt();
        let incidence_angle_deg = if pmag > 1e-30 {
            (pz_abs / pmag).acos().to_degrees()
        } else {
            90.0
        };

        // Back azimuth from horizontal components of principal direction
        let back_azimuth_deg = principal[2].atan2(principal[1]).to_degrees().rem_euclid(360.0);

        // Classify wave type
        let wave_type = classify_from_polarization(
            rectilinearity,
            planarity,
            ellipticity,
            incidence_angle_deg,
            self.params.rectilinearity_threshold,
        );

        PolarizationResult {
            center_sample: center,
            eigenvalues: lam,
            principal_direction: principal,
            rectilinearity,
            planarity,
            ellipticity,
            wave_type,
            incidence_angle_deg,
            back_azimuth_deg,
        }
    }
}

/// Classify wave type from polarization attributes.
fn classify_from_polarization(
    rectilinearity: f64,
    planarity: f64,
    ellipticity: f64,
    incidence_angle_deg: f64,
    rl_threshold: f64,
) -> SeismicWaveType {
    if rectilinearity >= rl_threshold {
        // Body wave: check incidence angle to distinguish P from S
        if incidence_angle_deg < 45.0 {
            SeismicWaveType::P
        } else {
            SeismicWaveType::S
        }
    } else if planarity > 0.6 && ellipticity > 0.15 && ellipticity < 0.85 {
        SeismicWaveType::Rayleigh
    } else if planarity > 0.5 && ellipticity < 0.15 && incidence_angle_deg > 70.0 {
        SeismicWaveType::Love
    } else {
        SeismicWaveType::Noise
    }
}

// ─── Hodograph Analysis ─────────────────────────────────────────────────────

/// Parameters for hodograph (particle motion) analysis.
#[derive(Debug, Clone)]
pub struct HodographParams {
    /// Number of samples per analysis window.
    pub window_samples: usize,
    /// Number of samples to advance between consecutive windows.
    pub hop_samples: usize,
}

impl Default for HodographParams {
    fn default() -> Self {
        Self {
            window_samples: 50,
            hop_samples: 10,
        }
    }
}

/// Ellipse parameters from hodograph analysis.
#[derive(Debug, Clone)]
pub struct EllipseParams {
    /// Semi-major axis length.
    pub semi_major: f64,
    /// Semi-minor axis length.
    pub semi_minor: f64,
    /// Tilt angle of semi-major axis from the first component axis (radians).
    pub tilt_angle_rad: f64,
    /// Ellipticity ratio `semi_minor / semi_major`. 0 = linear, 1 = circular.
    pub ellipticity: f64,
    /// Sense of rotation: +1 for prograde (counter-clockwise), -1 for retrograde.
    pub rotation_sense: f64,
}

/// Result of hodograph analysis for one window.
#[derive(Debug, Clone)]
pub struct HodographResult {
    /// Center sample of the analysis window.
    pub center_sample: usize,
    /// Ellipse fit in the Z-R (vertical vs radial) plane.
    pub zr_ellipse: EllipseParams,
    /// Ellipse fit in the Z-T (vertical vs transverse) plane.
    pub zt_ellipse: EllipseParams,
    /// Classified wave type.
    pub wave_type: SeismicWaveType,
}

/// Hodograph analyzer for particle-motion ellipse fitting.
///
/// Fits ellipses to two-component particle-motion plots to discriminate
/// between P-waves (linear), Rayleigh waves (retrograde elliptical), and
/// Love waves (transverse horizontal).
#[derive(Debug, Clone)]
pub struct HodographAnalyzer {
    params: HodographParams,
}

impl HodographAnalyzer {
    /// Create a new hodograph analyzer with the given parameters.
    pub fn new(params: HodographParams) -> Self {
        Self { params }
    }

    /// Analyze particle motion from three-component data.
    ///
    /// The radial direction is estimated from the principal horizontal component.
    ///
    /// # Arguments
    /// - `z` -- vertical component
    /// - `north` -- north-south component
    /// - `east` -- east-west component
    pub fn analyze(&self, z: &[f64], north: &[f64], east: &[f64]) -> Vec<HodographResult> {
        let n = z.len().min(north.len()).min(east.len());
        if n < self.params.window_samples {
            return Vec::new();
        }

        let mut results = Vec::new();
        let mut start = 0;
        while start + self.params.window_samples <= n {
            let end = start + self.params.window_samples;
            let result = self.analyze_window(z, north, east, start, end);
            results.push(result);
            start += self.params.hop_samples;
        }
        results
    }

    /// Analyze one window of particle motion.
    fn analyze_window(
        &self,
        z: &[f64],
        north: &[f64],
        east: &[f64],
        start: usize,
        end: usize,
    ) -> HodographResult {
        let center = (start + end) / 2;

        // Estimate back azimuth from horizontal covariance to get radial/transverse
        let (baz_rad, _) = estimate_back_azimuth(&north[start..end], &east[start..end]);

        // Rotate to radial (R) and transverse (T)
        let cos_baz = baz_rad.cos();
        let sin_baz = baz_rad.sin();

        let wlen = end - start;
        let mut radial = vec![0.0; wlen];
        let mut transverse = vec![0.0; wlen];
        let z_win: Vec<f64> = z[start..end].to_vec();

        for i in 0..wlen {
            radial[i] = north[start + i] * cos_baz + east[start + i] * sin_baz;
            transverse[i] = -north[start + i] * sin_baz + east[start + i] * cos_baz;
        }

        // Fit ellipses
        let zr_ellipse = fit_ellipse(&z_win, &radial);
        let zt_ellipse = fit_ellipse(&z_win, &transverse);

        // Classify from hodograph
        let wave_type = classify_from_hodograph(&zr_ellipse, &zt_ellipse);

        HodographResult {
            center_sample: center,
            zr_ellipse,
            zt_ellipse,
            wave_type,
        }
    }
}

/// Estimate back azimuth from horizontal component covariance.
/// Returns (azimuth_rad, principal_variance).
fn estimate_back_azimuth(north: &[f64], east: &[f64]) -> (f64, f64) {
    let n = north.len();
    if n == 0 {
        return (0.0, 0.0);
    }
    let inv_n = 1.0 / n as f64;
    let mn: f64 = north.iter().sum::<f64>() * inv_n;
    let me: f64 = east.iter().sum::<f64>() * inv_n;

    let mut cnn = 0.0;
    let mut cne = 0.0;
    let mut cee = 0.0;
    for i in 0..n {
        let dn = north[i] - mn;
        let de = east[i] - me;
        cnn += dn * dn;
        cne += dn * de;
        cee += de * de;
    }
    cnn *= inv_n;
    cne *= inv_n;
    cee *= inv_n;

    // Principal axis angle via atan2(2*cne, cnn - cee) / 2
    let angle = 0.5 * (2.0 * cne).atan2(cnn - cee);
    let variance = 0.5 * (cnn + cee) + 0.5 * ((cnn - cee).powi(2) + 4.0 * cne * cne).sqrt();
    (angle, variance)
}

/// Fit an ellipse to two-component particle motion using covariance eigenvectors.
fn fit_ellipse(comp1: &[f64], comp2: &[f64]) -> EllipseParams {
    let n = comp1.len();
    if n < 2 {
        return EllipseParams {
            semi_major: 0.0,
            semi_minor: 0.0,
            tilt_angle_rad: 0.0,
            ellipticity: 0.0,
            rotation_sense: 0.0,
        };
    }

    let inv_n = 1.0 / n as f64;
    let m1: f64 = comp1.iter().sum::<f64>() * inv_n;
    let m2: f64 = comp2.iter().sum::<f64>() * inv_n;

    let mut c11 = 0.0;
    let mut c12 = 0.0;
    let mut c22 = 0.0;
    for i in 0..n {
        let d1 = comp1[i] - m1;
        let d2 = comp2[i] - m2;
        c11 += d1 * d1;
        c12 += d1 * d2;
        c22 += d2 * d2;
    }
    c11 *= inv_n;
    c12 *= inv_n;
    c22 *= inv_n;

    // Eigenvalues of 2x2 symmetric matrix
    let trace = c11 + c22;
    let det = c11 * c22 - c12 * c12;
    let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
    let lam1 = 0.5 * (trace + disc);
    let lam2 = 0.5 * (trace - disc).max(0.0);

    let semi_major = lam1.sqrt();
    let semi_minor = lam2.sqrt();
    let tilt_angle_rad = 0.5 * (2.0 * c12).atan2(c11 - c22);
    let ellipticity = if semi_major > 1e-30 {
        semi_minor / semi_major
    } else {
        0.0
    };

    // Rotation sense: check cross product of successive particle-motion vectors
    let mut cross_sum = 0.0;
    for i in 0..n - 1 {
        cross_sum += (comp1[i] - m1) * (comp2[i + 1] - m2)
            - (comp2[i] - m2) * (comp1[i + 1] - m1);
    }
    let rotation_sense = if cross_sum > 0.0 {
        1.0
    } else if cross_sum < 0.0 {
        -1.0
    } else {
        0.0
    };

    EllipseParams {
        semi_major,
        semi_minor,
        tilt_angle_rad,
        ellipticity,
        rotation_sense,
    }
}

/// Classify wave type from hodograph ellipse parameters.
fn classify_from_hodograph(zr: &EllipseParams, zt: &EllipseParams) -> SeismicWaveType {
    let zr_energy = zr.semi_major * zr.semi_major + zr.semi_minor * zr.semi_minor;
    let zt_energy = zt.semi_major * zt.semi_major + zt.semi_minor * zt.semi_minor;
    let total = zr_energy + zt_energy;

    if total < 1e-30 {
        return SeismicWaveType::Noise;
    }

    // Love wave: energy predominantly in transverse plane with linear motion
    if zt_energy > 0.7 * total && zt.ellipticity < 0.3 {
        return SeismicWaveType::Love;
    }

    // Rayleigh wave: retrograde elliptical motion in Z-R plane
    if zr_energy > 0.5 * total && zr.ellipticity > 0.2 && zr.rotation_sense < 0.0 {
        return SeismicWaveType::Rayleigh;
    }

    // P wave: linear motion in Z-R plane with dominant Z component
    if zr_energy > 0.5 * total && zr.ellipticity < 0.2 {
        // Check if motion is more vertical (P) or horizontal (S)
        let tilt_from_z = zr.tilt_angle_rad.abs();
        if tilt_from_z < PI / 4.0 {
            return SeismicWaveType::P;
        } else {
            return SeismicWaveType::S;
        }
    }

    // S wave: linear motion with dominant horizontal
    if zr.ellipticity < 0.3 {
        return SeismicWaveType::S;
    }

    SeismicWaveType::Noise
}

// ─── F-K Filtering ──────────────────────────────────────────────────────────

/// Configuration for f-k (frequency-wavenumber) filtering.
#[derive(Debug, Clone)]
pub struct FkFilterConfig {
    /// Number of receivers in the array.
    pub num_receivers: usize,
    /// Receiver spacing in meters.
    pub receiver_spacing_m: f64,
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of time samples per trace. Must be a power of 2 for FFT efficiency.
    pub num_samples: usize,
    /// Minimum apparent velocity to pass (m/s). Set to 0 for low-pass (surface waves).
    pub velocity_min_m_s: f64,
    /// Maximum apparent velocity to pass (m/s). Set to `f64::INFINITY` for high-pass (body waves).
    pub velocity_max_m_s: f64,
    /// Taper width as fraction of the transition band [0, 1]. Smooths the filter edges.
    pub taper_fraction: f64,
}

impl Default for FkFilterConfig {
    fn default() -> Self {
        Self {
            num_receivers: 24,
            receiver_spacing_m: 10.0,
            sample_rate_hz: 500.0,
            num_samples: 256,
            velocity_min_m_s: 0.0,
            velocity_max_m_s: f64::INFINITY,
            taper_fraction: 0.1,
        }
    }
}

/// F-K domain filter for wave separation by apparent velocity.
///
/// Transforms array seismic data into the frequency-wavenumber domain using 2D DFT,
/// applies a velocity-band filter, and inverse transforms to recover separated traces.
#[derive(Debug, Clone)]
pub struct FkFilter {
    config: FkFilterConfig,
}

impl FkFilter {
    /// Create a new f-k filter with the given configuration.
    pub fn new(config: FkFilterConfig) -> Self {
        Self { config }
    }

    /// Apply f-k filtering to multi-channel seismic data.
    ///
    /// # Arguments
    /// - `traces` -- 2D array of seismic data, `traces[receiver][sample]`.
    ///   All traces must have the same number of samples.
    ///
    /// # Returns
    /// Filtered traces with the same dimensions as input.
    pub fn filter(&self, traces: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let nr = traces.len();
        if nr == 0 {
            return Vec::new();
        }
        let nt = traces[0].len();
        if nt == 0 {
            return vec![Vec::new(); nr];
        }

        // Pad to power-of-2 for FFT
        let nf = next_power_of_2(nt);
        let nk = next_power_of_2(nr);

        // 2D DFT: transform along time (columns) then along space (rows)
        let mut spectrum_re = vec![vec![0.0; nf]; nk];
        let mut spectrum_im = vec![vec![0.0; nf]; nk];

        // Forward FFT along time for each receiver
        for r in 0..nr {
            let mut re = vec![0.0; nf];
            let mut im = vec![0.0; nf];
            for s in 0..nt {
                re[s] = traces[r][s];
            }
            fft_radix2(&mut re, &mut im, false);
            spectrum_re[r] = re;
            spectrum_im[r] = im;
        }
        // Zero-pad remaining spatial channels
        // (already zero from initialization)

        // Forward FFT along space for each frequency bin
        for f_bin in 0..nf {
            let mut re = vec![0.0; nk];
            let mut im = vec![0.0; nk];
            for r in 0..nk {
                re[r] = spectrum_re[r][f_bin];
                im[r] = spectrum_im[r][f_bin];
            }
            fft_radix2(&mut re, &mut im, false);
            for r in 0..nk {
                spectrum_re[r][f_bin] = re[r];
                spectrum_im[r][f_bin] = im[r];
            }
        }

        // Apply velocity-band filter in f-k domain
        let df = self.config.sample_rate_hz / nf as f64;
        let dk = 1.0 / (nk as f64 * self.config.receiver_spacing_m);

        for k_bin in 0..nk {
            // Wavenumber: map to [-Nyquist, Nyquist)
            let k_idx = if k_bin <= nk / 2 { k_bin as f64 } else { k_bin as f64 - nk as f64 };
            let k = k_idx * dk;

            for f_bin in 0..nf {
                let f_idx = if f_bin <= nf / 2 { f_bin as f64 } else { f_bin as f64 - nf as f64 };
                let f = f_idx * df;

                // Apparent velocity = f / k  (for k != 0)
                let gain = if k.abs() < 1e-15 {
                    // DC wavenumber: pass if velocity_max is infinite
                    if self.config.velocity_max_m_s.is_infinite() && f.abs() < df {
                        1.0
                    } else if f.abs() < 1e-15 {
                        1.0  // DC-DC always pass
                    } else {
                        // f != 0 but k ~ 0 means infinite apparent velocity
                        if self.config.velocity_max_m_s.is_infinite() {
                            1.0
                        } else {
                            0.0
                        }
                    }
                } else {
                    let v_app = (f / k).abs();
                    velocity_band_gain(
                        v_app,
                        self.config.velocity_min_m_s,
                        self.config.velocity_max_m_s,
                        self.config.taper_fraction,
                    )
                };

                spectrum_re[k_bin][f_bin] *= gain;
                spectrum_im[k_bin][f_bin] *= gain;
            }
        }

        // Inverse 2D DFT: inverse FFT along space, then along time
        for f_bin in 0..nf {
            let mut re = vec![0.0; nk];
            let mut im = vec![0.0; nk];
            for r in 0..nk {
                re[r] = spectrum_re[r][f_bin];
                im[r] = spectrum_im[r][f_bin];
            }
            fft_radix2(&mut re, &mut im, true);
            for r in 0..nk {
                spectrum_re[r][f_bin] = re[r];
                spectrum_im[r][f_bin] = im[r];
            }
        }

        for r in 0..nr {
            fft_radix2(&mut spectrum_re[r], &mut spectrum_im[r], true);
        }

        // Extract original-size traces
        let mut output = Vec::with_capacity(nr);
        for r in 0..nr {
            output.push(spectrum_re[r][..nt].to_vec());
        }
        output
    }

    /// Compute the 2D f-k power spectrum for visualization.
    ///
    /// Returns `(power, freq_axis, wavenumber_axis)` where `power[k_bin][f_bin]`.
    pub fn compute_fk_spectrum(&self, traces: &[Vec<f64>]) -> (Vec<Vec<f64>>, Vec<f64>, Vec<f64>) {
        let nr = traces.len();
        if nr == 0 {
            return (Vec::new(), Vec::new(), Vec::new());
        }
        let nt = traces[0].len();
        if nt == 0 {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        let nf = next_power_of_2(nt);
        let nk = next_power_of_2(nr);

        let mut spec_re = vec![vec![0.0; nf]; nk];
        let mut spec_im = vec![vec![0.0; nf]; nk];

        // 2D forward DFT
        for r in 0..nr {
            let mut re = vec![0.0; nf];
            let mut im = vec![0.0; nf];
            for s in 0..nt {
                re[s] = traces[r][s];
            }
            fft_radix2(&mut re, &mut im, false);
            spec_re[r] = re;
            spec_im[r] = im;
        }
        for f_bin in 0..nf {
            let mut re = vec![0.0; nk];
            let mut im = vec![0.0; nk];
            for r in 0..nk {
                re[r] = spec_re[r][f_bin];
                im[r] = spec_im[r][f_bin];
            }
            fft_radix2(&mut re, &mut im, false);
            for r in 0..nk {
                spec_re[r][f_bin] = re[r];
                spec_im[r][f_bin] = im[r];
            }
        }

        // Power spectrum
        let mut power = vec![vec![0.0; nf]; nk];
        for k in 0..nk {
            for f in 0..nf {
                power[k][f] = spec_re[k][f] * spec_re[k][f] + spec_im[k][f] * spec_im[k][f];
            }
        }

        let df = self.config.sample_rate_hz / nf as f64;
        let dk = 1.0 / (nk as f64 * self.config.receiver_spacing_m);
        let freq_axis: Vec<f64> = (0..nf).map(|i| i as f64 * df).collect();
        let k_axis: Vec<f64> = (0..nk).map(|i| {
            if i <= nk / 2 { i as f64 * dk } else { (i as f64 - nk as f64) * dk }
        }).collect();

        (power, freq_axis, k_axis)
    }
}

/// Velocity band-pass gain with smooth taper.
fn velocity_band_gain(v: f64, v_min: f64, v_max: f64, taper: f64) -> f64 {
    if v_max.is_infinite() {
        // High-pass velocity filter
        if v >= v_min {
            1.0
        } else {
            let margin = taper * v_min;
            if margin < 1e-15 || v < v_min - margin {
                0.0
            } else {
                let t = (v - (v_min - margin)) / margin;
                0.5 * (1.0 - (PI * t).cos())
            }
        }
    } else if v_min <= 0.0 {
        // Low-pass velocity filter
        if v <= v_max {
            1.0
        } else {
            let margin = taper * v_max;
            if margin < 1e-15 || v > v_max + margin {
                0.0
            } else {
                let t = (v_max + margin - v) / margin;
                0.5 * (1.0 - (PI * t).cos())
            }
        }
    } else {
        // Band-pass velocity filter
        let gain_low = if v >= v_min {
            1.0
        } else {
            let margin = taper * v_min;
            if margin < 1e-15 || v < v_min - margin {
                0.0
            } else {
                let t = (v - (v_min - margin)) / margin;
                0.5 * (1.0 - (PI * t).cos())
            }
        };
        let gain_high = if v <= v_max {
            1.0
        } else {
            let margin = taper * v_max;
            if margin < 1e-15 || v > v_max + margin {
                0.0
            } else {
                let t = (v_max + margin - v) / margin;
                0.5 * (1.0 - (PI * t).cos())
            }
        };
        gain_low * gain_high
    }
}

// ─── Wave Separation Result ─────────────────────────────────────────────────

/// Complete result of wave separation on three-component data.
#[derive(Debug, Clone)]
pub struct WaveSeparationResult {
    /// Separated P-wave component `[z, north, east]`.
    pub p_wave: [Vec<f64>; 3],
    /// Separated S-wave component `[z, north, east]`.
    pub s_wave: [Vec<f64>; 3],
    /// Separated Rayleigh wave component `[z, north, east]`.
    pub rayleigh_wave: [Vec<f64>; 3],
    /// Separated Love wave component `[z, north, east]`.
    pub love_wave: [Vec<f64>; 3],
    /// Residual (noise / unclassified) `[z, north, east]`.
    pub residual: [Vec<f64>; 3],
    /// Per-window polarization classification results.
    pub polarization_results: Vec<PolarizationResult>,
}

/// Separate seismic waves using polarization-based windowed projection.
///
/// For each analysis window, the covariance eigenvectors define projection
/// directions. Samples are projected onto the P, S, or surface-wave subspace
/// based on the classification of each window, with overlap-add to reconstruct
/// smooth separated traces.
///
/// # Arguments
/// - `z`, `north`, `east` -- three-component seismogram of equal length.
/// - `params` -- polarization analysis parameters controlling window size and thresholds.
///
/// # Returns
/// A `WaveSeparationResult` containing separated wave components.
pub fn separate_waves(
    z: &[f64],
    north: &[f64],
    east: &[f64],
    params: &PolarizationParams,
) -> WaveSeparationResult {
    let n = z.len().min(north.len()).min(east.len());
    let mut p_wave = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let mut s_wave = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let mut rayleigh_wave = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let mut love_wave = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let mut weight_sum = vec![0.0; n];

    let analyzer = PolarizationAnalyzer::new(params.clone());
    let pol_results = analyzer.analyze(z, north, east);

    for result in &pol_results {
        let half_win = params.window_samples / 2;
        let win_start = if result.center_sample >= half_win {
            result.center_sample - half_win
        } else {
            0
        };
        let win_end = (result.center_sample + half_win).min(n);

        // Hann window for smooth overlap-add
        let wlen = win_end - win_start;
        for i in 0..wlen {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / wlen as f64).cos());
            let idx = win_start + i;
            let sample = [z[idx], north[idx], east[idx]];

            let target = match result.wave_type {
                SeismicWaveType::P => &mut p_wave,
                SeismicWaveType::S => &mut s_wave,
                SeismicWaveType::Rayleigh => &mut rayleigh_wave,
                SeismicWaveType::Love => &mut love_wave,
                SeismicWaveType::Noise => {
                    weight_sum[idx] += w;
                    continue;
                }
            };

            for c in 0..3 {
                target[c][idx] += w * sample[c];
            }
            weight_sum[idx] += w;
        }
    }

    // Normalize by overlap weight
    for idx in 0..n {
        if weight_sum[idx] > 1e-30 {
            let inv_w = 1.0 / weight_sum[idx];
            for c in 0..3 {
                p_wave[c][idx] *= inv_w;
                s_wave[c][idx] *= inv_w;
                rayleigh_wave[c][idx] *= inv_w;
                love_wave[c][idx] *= inv_w;
            }
        }
    }

    // Residual = original - sum of separated
    let mut residual = [vec![0.0; n], vec![0.0; n], vec![0.0; n]];
    let orig = [z, north, east];
    for c in 0..3 {
        for i in 0..n {
            residual[c][i] = orig[c][i]
                - p_wave[c][i]
                - s_wave[c][i]
                - rayleigh_wave[c][i]
                - love_wave[c][i];
        }
    }

    WaveSeparationResult {
        p_wave,
        s_wave,
        rayleigh_wave,
        love_wave,
        residual,
        polarization_results: pol_results,
    }
}

// ─── Helper Functions ───────────────────────────────────────────────────────

/// Compute apparent velocity from inter-receiver time delay.
///
/// `v_app = dx / dt` where `dx` is receiver spacing and `dt` is the
/// time delay between adjacent receivers.
///
/// # Arguments
/// - `receiver_spacing_m` -- distance between adjacent receivers in meters.
/// - `time_delay_s` -- time delay between adjacent receivers in seconds.
///
/// # Returns
/// Apparent velocity in m/s. Returns `f64::INFINITY` if `time_delay_s` is zero.
pub fn apparent_velocity(receiver_spacing_m: f64, time_delay_s: f64) -> f64 {
    if time_delay_s.abs() < 1e-30 {
        f64::INFINITY
    } else {
        (receiver_spacing_m / time_delay_s).abs()
    }
}

/// Compute the ray parameter (horizontal slowness) from incidence angle and velocity.
///
/// `p = sin(theta) / v` where `theta` is the incidence angle from vertical
/// and `v` is the wave velocity. The ray parameter is constant along a ray
/// path in a 1D velocity model (Snell's law).
///
/// # Arguments
/// - `incidence_angle_rad` -- incidence angle from vertical in radians.
/// - `velocity_m_s` -- wave velocity in m/s.
///
/// # Returns
/// Ray parameter in s/m.
pub fn ray_parameter(incidence_angle_rad: f64, velocity_m_s: f64) -> f64 {
    if velocity_m_s.abs() < 1e-30 {
        return 0.0;
    }
    incidence_angle_rad.sin() / velocity_m_s
}

/// Apply Snell's law to compute the refracted angle given ray parameter and velocity.
///
/// `sin(theta_2) = p * v_2` where `p` is the ray parameter and `v_2` is
/// the velocity of the second layer. If `p * v_2 > 1`, total internal
/// reflection occurs and `None` is returned.
///
/// # Arguments
/// - `ray_param` -- ray parameter in s/m (constant along ray).
/// - `velocity_m_s` -- velocity in the new layer (m/s).
///
/// # Returns
/// `Some(angle_rad)` -- refracted angle from vertical, or `None` for total reflection.
pub fn snell_law(ray_param: f64, velocity_m_s: f64) -> Option<f64> {
    let sin_theta = ray_param * velocity_m_s;
    if sin_theta.abs() > 1.0 {
        None // Total internal reflection
    } else {
        Some(sin_theta.asin())
    }
}

/// Compute P-wave and S-wave travel times for a homogeneous half-space.
///
/// `t = sqrt(z^2 + x^2) / v` for each wave type.
///
/// # Arguments
/// - `depth_m` -- source depth in meters.
/// - `offset_m` -- horizontal offset (epicentral distance) in meters.
/// - `vp_m_s` -- P-wave velocity in m/s.
/// - `vs_m_s` -- S-wave velocity in m/s.
///
/// # Returns
/// `(t_p, t_s)` -- P-wave and S-wave travel times in seconds.
pub fn travel_time_curve(
    depth_m: f64,
    offset_m: f64,
    vp_m_s: f64,
    vs_m_s: f64,
) -> (f64, f64) {
    let distance = (depth_m * depth_m + offset_m * offset_m).sqrt();
    let t_p = if vp_m_s > 1e-30 { distance / vp_m_s } else { f64::INFINITY };
    let t_s = if vs_m_s > 1e-30 { distance / vs_m_s } else { f64::INFINITY };
    (t_p, t_s)
}

/// Compute the Vp/Vs ratio from P and S travel time difference.
///
/// Given `dt = t_s - t_p` and distance `d`:
/// `Vp/Vs = t_s / t_p = 1 + dt * vp / d`
///
/// Alternatively, from Wadati diagram: `Vp/Vs = 1 + dt_s_p / t_p`
///
/// # Arguments
/// - `t_p` -- P-wave travel time in seconds.
/// - `t_s` -- S-wave travel time in seconds.
///
/// # Returns
/// Vp/Vs ratio.
pub fn vp_vs_ratio(t_p: f64, t_s: f64) -> f64 {
    if t_p.abs() < 1e-30 {
        return 1.732; // Default Poisson solid
    }
    t_s / t_p
}

/// Estimate epicentral distance from P-S time difference.
///
/// `d = dt * vp * vs / (vp - vs)` for a homogeneous model.
///
/// # Arguments
/// - `dt_s` -- S-P time difference in seconds.
/// - `vp_m_s` -- P-wave velocity in m/s.
/// - `vs_m_s` -- S-wave velocity in m/s.
///
/// # Returns
/// Estimated distance in meters.
pub fn distance_from_ps_delay(dt_s: f64, vp_m_s: f64, vs_m_s: f64) -> f64 {
    let denom = vp_m_s - vs_m_s;
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    dt_s * vp_m_s * vs_m_s / denom
}

/// Compute critical angle for total internal reflection.
///
/// `theta_c = asin(v1 / v2)` for `v1 < v2`.
///
/// # Arguments
/// - `v1_m_s` -- velocity in the incident medium (m/s).
/// - `v2_m_s` -- velocity in the refracting medium (m/s).
///
/// # Returns
/// `Some(angle_rad)` if `v1 < v2`, `None` otherwise.
pub fn critical_angle(v1_m_s: f64, v2_m_s: f64) -> Option<f64> {
    if v2_m_s.abs() < 1e-30 {
        return None;
    }
    let ratio = v1_m_s / v2_m_s;
    if ratio >= 1.0 || ratio <= 0.0 {
        None
    } else {
        Some(ratio.asin())
    }
}

// ─── Internal: Linear Algebra Utilities ─────────────────────────────────────

/// 3x3 symmetric matrix eigendecomposition via Jacobi iteration.
///
/// Returns `(eigenvalues, eigenvectors)` where `eigenvectors[row][col]` and
/// `col` indexes the eigenvector (i.e., the k-th eigenvector is column k).
fn symmetric_eigen_3x3(mat: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = mat;
    // Eigenvector matrix (columns are eigenvectors), starts as identity
    let mut v = [[0.0f64; 3]; 3];
    for i in 0..3 {
        v[i][i] = 1.0;
    }

    let max_iter = 100;
    for _ in 0..max_iter {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val = a[0][1].abs();
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > max_val {
                    max_val = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-15 {
            break;
        }

        // Compute rotation angle
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];

        let theta = if (app - aqq).abs() < 1e-30 {
            PI / 4.0
        } else {
            0.5 * (2.0 * apq / (app - aqq)).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Apply Givens rotation: A' = G^T A G
        let mut new_a = a;

        // Update rows/columns p and q
        for i in 0..3 {
            if i != p && i != q {
                let aip = a[i][p];
                let aiq = a[i][q];
                new_a[i][p] = c * aip + s * aiq;
                new_a[p][i] = new_a[i][p];
                new_a[i][q] = -s * aip + c * aiq;
                new_a[q][i] = new_a[i][q];
            }
        }
        new_a[p][p] = c * c * app + 2.0 * c * s * apq + s * s * aqq;
        new_a[q][q] = s * s * app - 2.0 * c * s * apq + c * c * aqq;
        new_a[p][q] = 0.0;
        new_a[q][p] = 0.0;

        a = new_a;

        // Update eigenvectors: V' = V * G
        for i in 0..3 {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = c * vip + s * viq;
            v[i][q] = -s * vip + c * viq;
        }
    }

    let eigenvalues = [a[0][0], a[1][1], a[2][2]];
    (eigenvalues, v)
}

/// In-place radix-2 Cooley-Tukey FFT.
///
/// `inverse = true` for IFFT (includes 1/N normalization).
/// Input length must be a power of 2.
fn fft_radix2(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert_eq!(n, im.len());
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut j = 0;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let even = start + k;
                let odd = start + k + half;

                let t_re = w_re * re[odd] - w_im * im[odd];
                let t_im = w_re * im[odd] + w_im * re[odd];

                re[odd] = re[even] - t_re;
                im[odd] = im[even] - t_im;
                re[even] += t_re;
                im[even] += t_im;

                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    // Normalize for inverse
    if inverse {
        let inv_n = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= inv_n;
            im[i] *= inv_n;
        }
    }
}

/// Next power of 2 >= n.
fn next_power_of_2(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a sine wave.
    fn sine_wave(n: usize, freq: f64, amp: f64, phase: f64) -> Vec<f64> {
        (0..n).map(|i| amp * (2.0 * PI * freq * i as f64 + phase).sin()).collect()
    }

    // ── SeismicWaveType ─────────────────────────────────────────────────

    #[test]
    fn test_wave_type_enum_variants() {
        let types = [
            SeismicWaveType::P,
            SeismicWaveType::S,
            SeismicWaveType::Rayleigh,
            SeismicWaveType::Love,
            SeismicWaveType::Noise,
        ];
        for t in &types {
            let _ = format!("{:?}", t);
        }
        assert_eq!(SeismicWaveType::P, SeismicWaveType::P);
        assert_ne!(SeismicWaveType::P, SeismicWaveType::S);
    }

    #[test]
    fn test_wave_type_clone_copy() {
        let wt = SeismicWaveType::Rayleigh;
        let wt2 = wt;
        assert_eq!(wt, wt2);
        let wt3 = wt.clone();
        assert_eq!(wt, wt3);
    }

    // ── Polarization Analyzer ───────────────────────────────────────────

    #[test]
    fn test_polarization_p_wave_vertical() {
        // P-wave arriving near-vertically: dominant Z component
        let n = 200;
        let z = sine_wave(n, 0.05, 1.0, 0.0);
        let north = sine_wave(n, 0.05, 0.1, 0.0);
        let east = sine_wave(n, 0.05, 0.05, 0.0);

        let params = PolarizationParams {
            window_samples: 50,
            hop_samples: 25,
            rectilinearity_threshold: 0.6,
        };
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        assert!(!results.is_empty());
        // Most windows should be classified as P
        let p_count = results.iter().filter(|r| r.wave_type == SeismicWaveType::P).count();
        assert!(p_count > results.len() / 2, "Expected mostly P-wave, got {}/{}", p_count, results.len());
    }

    #[test]
    fn test_polarization_s_wave_horizontal() {
        // S-wave: dominant horizontal, little vertical
        let n = 200;
        let z = sine_wave(n, 0.05, 0.05, 0.0);
        let north = sine_wave(n, 0.05, 1.0, 0.0);
        let east = sine_wave(n, 0.05, 0.1, 0.0);

        let params = PolarizationParams {
            window_samples: 50,
            hop_samples: 25,
            rectilinearity_threshold: 0.6,
        };
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        assert!(!results.is_empty());
        let s_count = results.iter().filter(|r| r.wave_type == SeismicWaveType::S).count();
        assert!(s_count > results.len() / 2, "Expected mostly S-wave, got {}/{}", s_count, results.len());
    }

    #[test]
    fn test_polarization_high_rectilinearity_for_linear_motion() {
        let n = 100;
        let z = sine_wave(n, 0.1, 1.0, 0.0);
        let north = sine_wave(n, 0.1, 0.0, 0.0);  // zero
        let east = sine_wave(n, 0.1, 0.0, 0.0);    // zero

        let params = PolarizationParams {
            window_samples: 50,
            hop_samples: 25,
            rectilinearity_threshold: 0.5,
        };
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        for r in &results {
            assert!(r.rectilinearity > 0.8, "Linear motion should have high rectilinearity, got {}", r.rectilinearity);
        }
    }

    #[test]
    fn test_polarization_short_input_returns_empty() {
        let params = PolarizationParams {
            window_samples: 100,
            hop_samples: 10,
            rectilinearity_threshold: 0.6,
        };
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&[1.0; 50], &[0.0; 50], &[0.0; 50]);
        assert!(results.is_empty());
    }

    #[test]
    fn test_polarization_incidence_angle_near_vertical() {
        // Purely vertical motion should give incidence angle near 0
        let n = 100;
        let z = sine_wave(n, 0.1, 1.0, 0.0);
        let north = vec![0.0; n];
        let east = vec![0.0; n];

        let params = PolarizationParams {
            window_samples: 50,
            hop_samples: 25,
            rectilinearity_threshold: 0.5,
        };
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        for r in &results {
            assert!(r.incidence_angle_deg < 15.0,
                "Vertical motion should have small incidence angle, got {}", r.incidence_angle_deg);
        }
    }

    #[test]
    fn test_polarization_default_params() {
        let params = PolarizationParams::default();
        assert_eq!(params.window_samples, 64);
        assert_eq!(params.hop_samples, 16);
        assert!((params.rectilinearity_threshold - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_polarization_eigenvalues_nonnegative() {
        let n = 200;
        let z = sine_wave(n, 0.07, 1.0, 0.0);
        let north = sine_wave(n, 0.11, 0.5, 0.3);
        let east = sine_wave(n, 0.13, 0.3, 0.7);

        let params = PolarizationParams::default();
        let analyzer = PolarizationAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        for r in &results {
            for &lam in &r.eigenvalues {
                assert!(lam >= 0.0, "Eigenvalue should be non-negative, got {}", lam);
            }
            assert!(r.eigenvalues[0] >= r.eigenvalues[1]);
            assert!(r.eigenvalues[1] >= r.eigenvalues[2]);
        }
    }

    // ── Hodograph Analyzer ──────────────────────────────────────────────

    #[test]
    fn test_hodograph_p_wave_linear() {
        // P-wave: linear motion, mostly Z
        let n = 200;
        let z = sine_wave(n, 0.05, 1.0, 0.0);
        let north = sine_wave(n, 0.05, 0.08, 0.0); // in-phase with Z (linear)
        let east = vec![0.0; n];

        let params = HodographParams {
            window_samples: 50,
            hop_samples: 25,
        };
        let analyzer = HodographAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        assert!(!results.is_empty());
        // Z-R ellipse should have low ellipticity (linear)
        for r in &results {
            assert!(r.zr_ellipse.ellipticity < 0.5,
                "P-wave ZR ellipticity should be low, got {}", r.zr_ellipse.ellipticity);
        }
    }

    #[test]
    fn test_hodograph_rayleigh_retrograde() {
        // Rayleigh wave: retrograde elliptical motion in Z-R plane
        // Z and R are 90 degrees out of phase with R leading (retrograde)
        let n = 200;
        let freq = 0.05;
        let z: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64).sin()).collect();
        let north: Vec<f64> = (0..n).map(|i| 0.7 * (2.0 * PI * freq * i as f64 + PI / 2.0).sin()).collect();
        let east = vec![0.0; n];

        let params = HodographParams {
            window_samples: 80,
            hop_samples: 40,
        };
        let analyzer = HodographAnalyzer::new(params);
        let results = analyzer.analyze(&z, &north, &east);

        assert!(!results.is_empty());
        // Should have moderate ellipticity in Z-R
        for r in &results {
            assert!(r.zr_ellipse.ellipticity > 0.1,
                "Rayleigh ZR ellipticity should be non-zero, got {}", r.zr_ellipse.ellipticity);
        }
    }

    #[test]
    fn test_hodograph_default_params() {
        let params = HodographParams::default();
        assert_eq!(params.window_samples, 50);
        assert_eq!(params.hop_samples, 10);
    }

    #[test]
    fn test_hodograph_short_input() {
        let params = HodographParams {
            window_samples: 100,
            hop_samples: 10,
        };
        let analyzer = HodographAnalyzer::new(params);
        let results = analyzer.analyze(&[0.0; 50], &[0.0; 50], &[0.0; 50]);
        assert!(results.is_empty());
    }

    // ── F-K Filter ──────────────────────────────────────────────────────

    #[test]
    fn test_fk_filter_preserves_dc() {
        // All-pass filter should preserve input
        let config = FkFilterConfig {
            num_receivers: 4,
            receiver_spacing_m: 10.0,
            sample_rate_hz: 100.0,
            num_samples: 16,
            velocity_min_m_s: 0.0,
            velocity_max_m_s: f64::INFINITY,
            taper_fraction: 0.0,
        };
        let filter = FkFilter::new(config);

        let traces: Vec<Vec<f64>> = (0..4)
            .map(|_| vec![1.0; 16])
            .collect();
        let filtered = filter.filter(&traces);

        assert_eq!(filtered.len(), 4);
        for trace in &filtered {
            assert_eq!(trace.len(), 16);
            for &v in trace {
                assert!((v - 1.0).abs() < 0.1, "All-pass should preserve DC, got {}", v);
            }
        }
    }

    #[test]
    fn test_fk_filter_output_dimensions() {
        let config = FkFilterConfig {
            num_receivers: 8,
            receiver_spacing_m: 5.0,
            sample_rate_hz: 200.0,
            num_samples: 32,
            velocity_min_m_s: 100.0,
            velocity_max_m_s: 5000.0,
            taper_fraction: 0.1,
        };
        let filter = FkFilter::new(config);

        let traces: Vec<Vec<f64>> = (0..8)
            .map(|r| sine_wave(32, 0.05 + r as f64 * 0.01, 1.0, 0.0))
            .collect();
        let filtered = filter.filter(&traces);

        assert_eq!(filtered.len(), 8);
        for trace in &filtered {
            assert_eq!(trace.len(), 32);
        }
    }

    #[test]
    fn test_fk_filter_empty_input() {
        let filter = FkFilter::new(FkFilterConfig::default());
        let result = filter.filter(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_fk_spectrum_dimensions() {
        let config = FkFilterConfig {
            num_receivers: 4,
            receiver_spacing_m: 10.0,
            sample_rate_hz: 100.0,
            num_samples: 16,
            ..FkFilterConfig::default()
        };
        let filter = FkFilter::new(config);

        let traces: Vec<Vec<f64>> = (0..4)
            .map(|_| sine_wave(16, 0.1, 1.0, 0.0))
            .collect();
        let (power, freq_axis, k_axis) = filter.compute_fk_spectrum(&traces);

        let nf = next_power_of_2(16);
        let nk = next_power_of_2(4);
        assert_eq!(power.len(), nk);
        assert_eq!(power[0].len(), nf);
        assert_eq!(freq_axis.len(), nf);
        assert_eq!(k_axis.len(), nk);
    }

    #[test]
    fn test_fk_default_config() {
        let config = FkFilterConfig::default();
        assert_eq!(config.num_receivers, 24);
        assert!((config.receiver_spacing_m - 10.0).abs() < 1e-10);
        assert!((config.sample_rate_hz - 500.0).abs() < 1e-10);
        assert_eq!(config.num_samples, 256);
    }

    // ── Wave Separation ─────────────────────────────────────────────────

    #[test]
    fn test_wave_separation_basic() {
        let n = 300;
        // Dominant vertical signal (P-wave like)
        let z = sine_wave(n, 0.05, 1.0, 0.0);
        let north = sine_wave(n, 0.05, 0.1, 0.0);
        let east = sine_wave(n, 0.05, 0.05, 0.0);

        let params = PolarizationParams {
            window_samples: 50,
            hop_samples: 25,
            rectilinearity_threshold: 0.6,
        };
        let result = separate_waves(&z, &north, &east, &params);

        assert_eq!(result.p_wave[0].len(), n);
        assert_eq!(result.s_wave[0].len(), n);
        assert_eq!(result.rayleigh_wave[0].len(), n);
        assert_eq!(result.love_wave[0].len(), n);
        assert_eq!(result.residual[0].len(), n);
    }

    #[test]
    fn test_wave_separation_energy_conservation() {
        let n = 200;
        let z = sine_wave(n, 0.05, 1.0, 0.0);
        let north = sine_wave(n, 0.05, 0.5, 0.3);
        let east = sine_wave(n, 0.05, 0.3, 0.7);

        let params = PolarizationParams {
            window_samples: 40,
            hop_samples: 20,
            rectilinearity_threshold: 0.6,
        };
        let result = separate_waves(&z, &north, &east, &params);

        // Sum of separated + residual should approximate original
        for c in 0..3 {
            let orig = match c {
                0 => &z,
                1 => &north,
                _ => &east,
            };
            for i in 0..n {
                let reconstructed = result.p_wave[c][i]
                    + result.s_wave[c][i]
                    + result.rayleigh_wave[c][i]
                    + result.love_wave[c][i]
                    + result.residual[c][i];
                assert!(
                    (reconstructed - orig[i]).abs() < 1e-10,
                    "Energy not conserved at sample {}, component {}: {} vs {}",
                    i, c, reconstructed, orig[i]
                );
            }
        }
    }

    // ── Helper Functions ────────────────────────────────────────────────

    #[test]
    fn test_apparent_velocity() {
        assert!((apparent_velocity(100.0, 0.01) - 10000.0).abs() < 1e-6);
        assert!(apparent_velocity(100.0, 0.0).is_infinite());
        assert!((apparent_velocity(50.0, 0.02) - 2500.0).abs() < 1e-6);
    }

    #[test]
    fn test_ray_parameter() {
        // At normal incidence (0 rad), ray parameter = 0
        assert!(ray_parameter(0.0, 6000.0).abs() < 1e-15);
        // At 30 degrees
        let p = ray_parameter(30.0_f64.to_radians(), 6000.0);
        let expected = (30.0_f64.to_radians()).sin() / 6000.0;
        assert!((p - expected).abs() < 1e-15);
    }

    #[test]
    fn test_snell_law_normal() {
        // Normal incidence: p = 0 -> refracted angle = 0
        let result = snell_law(0.0, 6000.0);
        assert!(result.is_some());
        assert!(result.unwrap().abs() < 1e-15);
    }

    #[test]
    fn test_snell_law_total_reflection() {
        // Large ray parameter with high velocity -> total reflection
        let p = ray_parameter(60.0_f64.to_radians(), 3000.0);
        let result = snell_law(p, 2000.0); // Slower medium
        // sin(theta) = p * v = sin(60) * 2000/3000 = 0.577 -> OK
        assert!(result.is_some());

        // Now try with a ray parameter that would exceed 1
        let p_large = 1.0 / 2000.0; // sin(theta) = 1.0 at v=2000
        let result2 = snell_law(p_large, 3000.0); // sin(theta) = 1.5 -> total reflection
        assert!(result2.is_none());
    }

    #[test]
    fn test_travel_time_curve() {
        let (tp, ts) = travel_time_curve(10000.0, 30000.0, 6000.0, 3500.0);
        let dist = (10000.0_f64.powi(2) + 30000.0_f64.powi(2)).sqrt();
        assert!((tp - dist / 6000.0).abs() < 1e-10);
        assert!((ts - dist / 3500.0).abs() < 1e-10);
        assert!(ts > tp); // S is slower
    }

    #[test]
    fn test_vp_vs_ratio() {
        let ratio = vp_vs_ratio(10.0, 17.32);
        assert!((ratio - 1.732).abs() < 0.01);
        // Zero P time -> default Poisson ratio
        let default_ratio = vp_vs_ratio(0.0, 5.0);
        assert!((default_ratio - 1.732).abs() < 1e-3);
    }

    #[test]
    fn test_distance_from_ps_delay() {
        let vp = 6000.0;
        let vs = 3500.0;
        let dist = 50000.0;
        let tp = dist / vp;
        let ts = dist / vs;
        let dt = ts - tp;
        let estimated = distance_from_ps_delay(dt, vp, vs);
        assert!((estimated - dist).abs() < 1.0, "Distance estimate {} != {}", estimated, dist);
    }

    #[test]
    fn test_critical_angle() {
        // v1=3000, v2=6000 -> critical angle = asin(0.5) = 30 deg
        let ca = critical_angle(3000.0, 6000.0);
        assert!(ca.is_some());
        assert!((ca.unwrap().to_degrees() - 30.0).abs() < 1e-10);

        // v1 >= v2 -> no critical angle
        assert!(critical_angle(6000.0, 3000.0).is_none());
        assert!(critical_angle(3000.0, 3000.0).is_none());
        assert!(critical_angle(3000.0, 0.0).is_none());
    }

    // ── Internal utilities ──────────────────────────────────────────────

    #[test]
    fn test_next_power_of_2() {
        assert_eq!(next_power_of_2(0), 1);
        assert_eq!(next_power_of_2(1), 1);
        assert_eq!(next_power_of_2(2), 2);
        assert_eq!(next_power_of_2(3), 4);
        assert_eq!(next_power_of_2(5), 8);
        assert_eq!(next_power_of_2(16), 16);
        assert_eq!(next_power_of_2(17), 32);
    }

    #[test]
    fn test_fft_roundtrip() {
        let n = 8;
        let original: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).sin()).collect();
        let mut re = original.clone();
        let mut im = vec![0.0; n];

        fft_radix2(&mut re, &mut im, false);
        fft_radix2(&mut re, &mut im, true);

        for (i, &orig) in original.iter().enumerate() {
            assert!((re[i] - orig).abs() < 1e-12, "FFT roundtrip failed at {}: {} vs {}", i, re[i], orig);
        }
        for &v in &im {
            assert!(v.abs() < 1e-12, "Imaginary part should be ~0 after roundtrip: {}", v);
        }
    }

    #[test]
    fn test_symmetric_eigen_diagonal() {
        // Diagonal matrix: eigenvalues are the diagonal entries
        let mat = [[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]];
        let (evals, _) = symmetric_eigen_3x3(mat);
        let mut sorted = evals;
        sorted.sort_by(|a, b| b.partial_cmp(a).unwrap());
        assert!((sorted[0] - 3.0).abs() < 1e-10);
        assert!((sorted[1] - 2.0).abs() < 1e-10);
        assert!((sorted[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_velocity_band_gain() {
        // Within band
        assert!((velocity_band_gain(3000.0, 1000.0, 5000.0, 0.1) - 1.0).abs() < 1e-10);
        // Below band
        assert!((velocity_band_gain(100.0, 1000.0, 5000.0, 0.1)).abs() < 1e-10);
        // Above band
        assert!((velocity_band_gain(10000.0, 1000.0, 5000.0, 0.1)).abs() < 1e-10);
        // Infinite max (high-pass)
        assert!((velocity_band_gain(5000.0, 1000.0, f64::INFINITY, 0.1) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ellipse_fit_linear() {
        // Perfectly correlated (linear) motion
        let n = 100;
        let comp1 = sine_wave(n, 0.1, 1.0, 0.0);
        let comp2 = sine_wave(n, 0.1, 0.5, 0.0); // in-phase, linear
        let ell = fit_ellipse(&comp1, &comp2);
        assert!(ell.ellipticity < 0.15, "Linear motion should have low ellipticity: {}", ell.ellipticity);
    }

    #[test]
    fn test_ellipse_fit_circular() {
        // Circular motion: 90-degree phase shift, equal amplitude
        let n = 200;
        let freq = 0.05;
        let comp1: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64).sin()).collect();
        let comp2: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64 + PI / 2.0).sin()).collect();
        let ell = fit_ellipse(&comp1, &comp2);
        // Should have high ellipticity (near 1 for circular)
        assert!(ell.ellipticity > 0.7, "Circular motion should have high ellipticity: {}", ell.ellipticity);
    }
}
