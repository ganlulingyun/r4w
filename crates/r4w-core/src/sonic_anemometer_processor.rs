//! Ultrasonic anemometer signal processing for 3D wind vector measurement
//! and turbulence characterization.
//!
//! Implements transit-time ultrasonic anemometry: paired transducers measure
//! forward and reverse acoustic travel times along known paths. The difference
//! gives the wind component along each path; the sum gives speed of sound
//! (and thus sonic temperature).
//!
//! Features:
//! - Transit-time wind and sonic temperature measurement
//! - 3D wind vector from multiple acoustic paths
//! - Coordinate rotation (double rotation, planar fit)
//! - Reynolds decomposition and turbulence statistics
//! - Eddy covariance fluxes (momentum, sensible heat, friction velocity, Obukhov length)
//! - Spectral analysis (power spectra, cospectra, Kolmogorov -5/3 law)
//! - Quality control (spike detection, stationarity, integral turbulence tests)

use std::f64::consts::PI;

// Physical constants
const GAMMA_AIR: f64 = 1.4; // ratio of specific heats for dry air
const R_D: f64 = 287.05; // specific gas constant for dry air [J/(kg*K)]
const VON_KARMAN: f64 = 0.4; // von Karman constant
const GRAVITY: f64 = 9.80665; // gravitational acceleration [m/s^2]
const RHO_AIR: f64 = 1.225; // standard air density [kg/m^3]
const CP_AIR: f64 = 1005.0; // specific heat at constant pressure [J/(kg*K)]

/// Configuration for a sonic anemometer.
#[derive(Clone, Debug)]
pub struct SonicConfig {
    /// Acoustic path length in meters (typically 0.1 - 0.2 m).
    pub path_length_m: f64,
    /// Angle of acoustic paths from vertical in degrees.
    pub path_angle_deg: f64,
    /// Number of measurement axes (2 or 3).
    pub num_axes: usize,
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
}

impl SonicConfig {
    /// Create a new sonic anemometer configuration.
    pub fn new(path_length_m: f64, path_angle_deg: f64, num_axes: usize, sample_rate_hz: f64) -> Self {
        assert!(num_axes == 2 || num_axes == 3, "num_axes must be 2 or 3");
        assert!(path_length_m > 0.0, "path_length must be positive");
        assert!(sample_rate_hz > 0.0, "sample_rate must be positive");
        Self {
            path_length_m,
            path_angle_deg,
            num_axes,
            sample_rate_hz,
        }
    }

    /// Typical 3-axis orthogonal sonic anemometer (e.g., Campbell CSAT3).
    pub fn csat3() -> Self {
        Self::new(0.1155, 60.0, 3, 20.0)
    }

    /// Typical 3-axis non-orthogonal sonic anemometer (e.g., Gill R3).
    pub fn gill_r3() -> Self {
        Self::new(0.15, 45.0, 3, 20.0)
    }
}

/// 3D wind vector in meteorological coordinates.
#[derive(Clone, Debug, Default)]
pub struct WindVector {
    /// East-west component (positive = from west) [m/s].
    pub u: f64,
    /// North-south component (positive = from south) [m/s].
    pub v: f64,
    /// Vertical component (positive = upward) [m/s].
    pub w: f64,
}

impl WindVector {
    pub fn new(u: f64, v: f64, w: f64) -> Self {
        Self { u, v, w }
    }

    /// Horizontal wind speed [m/s].
    pub fn horizontal_speed(&self) -> f64 {
        (self.u * self.u + self.v * self.v).sqrt()
    }

    /// Wind direction in degrees (meteorological convention: direction wind comes FROM,
    /// 0 = north, 90 = east, clockwise).
    ///
    /// Uses atan2(u, v) + 180 to convert the wind vector (pointing in the direction
    /// the wind is going) to the direction it is coming from.
    pub fn direction_deg(&self) -> f64 {
        let dir = self.u.atan2(self.v) * 180.0 / PI + 180.0;
        if dir >= 360.0 {
            dir - 360.0
        } else if dir < 0.0 {
            dir + 360.0
        } else {
            dir
        }
    }

    /// 3D wind speed [m/s].
    pub fn speed_3d(&self) -> f64 {
        (self.u * self.u + self.v * self.v + self.w * self.w).sqrt()
    }
}

/// Transit time measurement result for a single acoustic path.
#[derive(Clone, Debug)]
pub struct TransitTimeMeasurement {
    /// Wind component along the acoustic path [m/s].
    pub wind_along_path: f64,
    /// Speed of sound along the path [m/s].
    pub speed_of_sound: f64,
}

/// Compute wind speed and speed of sound from transit times along a path.
///
/// - `path_length`: acoustic path length L [m]
/// - `t_forward`: transit time in forward direction [s]
/// - `t_reverse`: transit time in reverse direction [s]
///
/// Returns wind along path and speed of sound.
pub fn transit_time_measurement(path_length: f64, t_forward: f64, t_reverse: f64) -> TransitTimeMeasurement {
    assert!(t_forward > 0.0 && t_reverse > 0.0, "transit times must be positive");
    let inv_fwd = 1.0 / t_forward;
    let inv_rev = 1.0 / t_reverse;
    let speed_of_sound = path_length * (inv_fwd + inv_rev) / 2.0;
    let wind_along_path = path_length * (inv_fwd - inv_rev) / 2.0;
    TransitTimeMeasurement {
        wind_along_path,
        speed_of_sound,
    }
}

/// Sonic temperature from speed of sound.
///
/// T_sonic = c^2 / (gamma * R_d) in Kelvin.
pub fn sonic_temperature(speed_of_sound: f64) -> f64 {
    speed_of_sound * speed_of_sound / (GAMMA_AIR * R_D)
}

/// Virtual temperature accounting for humidity.
///
/// T_v ≈ T_sonic * (1 + 0.51 * q) where q is specific humidity [kg/kg].
pub fn virtual_temperature(t_sonic: f64, specific_humidity: f64) -> f64 {
    t_sonic * (1.0 + 0.51 * specific_humidity)
}

/// Transform path-wind measurements from 3 orthogonal axes to (u, v, w).
///
/// For orthogonal 3-axis sonics, the path winds directly decompose into components.
/// `path_winds` = [v_path1, v_path2, v_path3] along each axis.
/// `azimuth_angles_deg` = azimuth angle of each path from north [deg].
/// `elevation_angles_deg` = elevation angle of each path from horizontal [deg].
pub fn orthogonal_3axis_to_uvw(
    path_winds: &[f64; 3],
    azimuth_angles_deg: &[f64; 3],
    elevation_angles_deg: &[f64; 3],
) -> WindVector {
    // Build 3x3 geometry matrix: each row is the unit vector along a path
    // path_wind_i = u * sin(az_i)*cos(el_i) + v * cos(az_i)*cos(el_i) + w * sin(el_i)
    // Solve via Cramer's rule (3x3 matrix inversion)
    let mut a = [[0.0f64; 3]; 3];
    for i in 0..3 {
        let az = azimuth_angles_deg[i] * PI / 180.0;
        let el = elevation_angles_deg[i] * PI / 180.0;
        a[i][0] = az.sin() * el.cos();
        a[i][1] = az.cos() * el.cos();
        a[i][2] = el.sin();
    }

    let uvw = solve_3x3(&a, path_winds);
    WindVector::new(uvw[0], uvw[1], uvw[2])
}

/// Solve a 3x3 linear system Ax = b using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> [f64; 3] {
    let det = det_3x3(a);
    assert!(det.abs() > 1e-15, "singular geometry matrix");

    let mut result = [0.0; 3];
    for col in 0..3 {
        let mut a_mod = *a;
        for row in 0..3 {
            a_mod[row][col] = b[row];
        }
        result[col] = det_3x3(&a_mod) / det;
    }
    result
}

/// Determinant of a 3x3 matrix.
fn det_3x3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Non-orthogonal path transformation using a rotation matrix.
///
/// Given a 3x3 transformation matrix T, converts path winds to (u, v, w):
///   [u, v, w]^T = T * [v_path1, v_path2, v_path3]^T
pub fn non_orthogonal_transform(
    path_winds: &[f64; 3],
    transform_matrix: &[[f64; 3]; 3],
) -> WindVector {
    let u = transform_matrix[0][0] * path_winds[0]
        + transform_matrix[0][1] * path_winds[1]
        + transform_matrix[0][2] * path_winds[2];
    let v = transform_matrix[1][0] * path_winds[0]
        + transform_matrix[1][1] * path_winds[1]
        + transform_matrix[1][2] * path_winds[2];
    let w = transform_matrix[2][0] * path_winds[0]
        + transform_matrix[2][1] * path_winds[1]
        + transform_matrix[2][2] * path_winds[2];
    WindVector::new(u, v, w)
}

/// Rotation angles from double rotation.
#[derive(Clone, Debug)]
pub struct RotationAngles {
    /// Yaw angle (rotation about z-axis to align u with mean wind) [rad].
    pub yaw: f64,
    /// Pitch angle (rotation about y-axis to zero mean w) [rad].
    pub pitch: f64,
}

/// Double rotation: align u with mean wind direction, zero mean v and w.
///
/// First rotation (yaw): rotate about z to align horizontal wind with u axis.
/// Second rotation (pitch): rotate about y to zero w.
///
/// Returns rotated wind vectors and rotation angles.
pub fn double_rotation(winds: &[WindVector]) -> (Vec<WindVector>, RotationAngles) {
    let n = winds.len();
    assert!(n > 0, "need at least one wind record");

    // Compute means
    let mut u_mean = 0.0;
    let mut v_mean = 0.0;
    let mut w_mean = 0.0;
    for w in winds {
        u_mean += w.u;
        v_mean += w.v;
        w_mean += w.w;
    }
    u_mean /= n as f64;
    v_mean /= n as f64;
    w_mean /= n as f64;

    // First rotation: yaw about z-axis to zero mean v
    let yaw = v_mean.atan2(u_mean);
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    // After first rotation, mean u' = sqrt(u_mean^2 + v_mean^2), mean v' = 0
    let u_mean_rot1 = (u_mean * u_mean + v_mean * v_mean).sqrt();

    // Second rotation: pitch about y'-axis to zero mean w
    let pitch = w_mean.atan2(u_mean_rot1);
    let cos_pitch = pitch.cos();
    let sin_pitch = pitch.sin();

    let mut rotated = Vec::with_capacity(n);
    for wind in winds {
        // First rotation (yaw)
        let u1 = wind.u * cos_yaw + wind.v * sin_yaw;
        let v1 = -wind.u * sin_yaw + wind.v * cos_yaw;
        // Second rotation (pitch)
        let u2 = u1 * cos_pitch + wind.w * sin_pitch;
        let w2 = -u1 * sin_pitch + wind.w * cos_pitch;
        rotated.push(WindVector::new(u2, v1, w2));
    }

    (rotated, RotationAngles { yaw, pitch })
}

/// Planar fit coefficients.
#[derive(Clone, Debug)]
pub struct PlanarFitCoefficients {
    /// Offset [m/s].
    pub b0: f64,
    /// Slope in u direction.
    pub b1: f64,
    /// Slope in v direction.
    pub b2: f64,
}

/// Planar fit: regression plane through mean w = b0 + b1*u + b2*v.
///
/// Removes mean vertical velocity bias by fitting a plane to (u, v, w) data.
/// Returns corrected wind vectors with w' = w - (b0 + b1*u + b2*v).
pub fn planar_fit(winds: &[WindVector]) -> (Vec<WindVector>, PlanarFitCoefficients) {
    let n = winds.len() as f64;
    assert!(n >= 3.0, "need at least 3 records for planar fit");

    // Build normal equations for w = b0 + b1*u + b2*v
    let mut su = 0.0;
    let mut sv = 0.0;
    let mut sw = 0.0;
    let mut suu = 0.0;
    let mut svv = 0.0;
    let mut suv = 0.0;
    let mut suw = 0.0;
    let mut svw = 0.0;

    for wind in winds {
        su += wind.u;
        sv += wind.v;
        sw += wind.w;
        suu += wind.u * wind.u;
        svv += wind.v * wind.v;
        suv += wind.u * wind.v;
        suw += wind.u * wind.w;
        svw += wind.v * wind.w;
    }

    // Normal equations: A * [b0, b1, b2]^T = rhs
    let a = [
        [n, su, sv],
        [su, suu, suv],
        [sv, suv, svv],
    ];
    let rhs = [sw, suw, svw];
    let coeffs = solve_3x3(&a, &rhs);

    let pf = PlanarFitCoefficients {
        b0: coeffs[0],
        b1: coeffs[1],
        b2: coeffs[2],
    };

    let corrected: Vec<WindVector> = winds
        .iter()
        .map(|w| {
            let w_corrected = w.w - (pf.b0 + pf.b1 * w.u + pf.b2 * w.v);
            WindVector::new(w.u, w.v, w_corrected)
        })
        .collect();

    (corrected, pf)
}

/// Reynolds decomposition: separate mean and fluctuation.
///
/// Returns (means, fluctuations) where fluctuation[i] = data[i] - mean.
pub fn reynolds_decomposition(data: &[f64]) -> (f64, Vec<f64>) {
    let n = data.len();
    assert!(n > 0);
    let mean = data.iter().sum::<f64>() / n as f64;
    let fluct: Vec<f64> = data.iter().map(|&x| x - mean).collect();
    (mean, fluct)
}

/// Turbulence statistics from wind vector time series.
#[derive(Clone, Debug)]
pub struct TurbulenceStats {
    /// Mean wind components [m/s].
    pub u_mean: f64,
    pub v_mean: f64,
    pub w_mean: f64,
    /// Variance of fluctuations [m^2/s^2].
    pub var_u: f64,
    pub var_v: f64,
    pub var_w: f64,
    /// Standard deviations [m/s].
    pub sigma_u: f64,
    pub sigma_v: f64,
    pub sigma_w: f64,
    /// Turbulence intensity (sigma_u / horizontal mean wind).
    pub turbulence_intensity: f64,
    /// Turbulent kinetic energy [m^2/s^2].
    pub tke: f64,
}

/// Compute turbulence statistics from wind vector time series.
pub fn turbulence_statistics(winds: &[WindVector]) -> TurbulenceStats {
    let n = winds.len() as f64;
    assert!(n > 1.0, "need at least 2 records");

    let mut u_mean = 0.0;
    let mut v_mean = 0.0;
    let mut w_mean = 0.0;
    for w in winds {
        u_mean += w.u;
        v_mean += w.v;
        w_mean += w.w;
    }
    u_mean /= n;
    v_mean /= n;
    w_mean /= n;

    let mut var_u = 0.0;
    let mut var_v = 0.0;
    let mut var_w = 0.0;
    for w in winds {
        let du = w.u - u_mean;
        let dv = w.v - v_mean;
        let dw = w.w - w_mean;
        var_u += du * du;
        var_v += dv * dv;
        var_w += dw * dw;
    }
    var_u /= n;
    var_v /= n;
    var_w /= n;

    let sigma_u = var_u.sqrt();
    let sigma_v = var_v.sqrt();
    let sigma_w = var_w.sqrt();

    let horiz_mean = (u_mean * u_mean + v_mean * v_mean).sqrt();
    let ti = if horiz_mean > 1e-10 {
        sigma_u / horiz_mean
    } else {
        0.0
    };

    let tke = 0.5 * (var_u + var_v + var_w);

    TurbulenceStats {
        u_mean,
        v_mean,
        w_mean,
        var_u,
        var_v,
        var_w,
        sigma_u,
        sigma_v,
        sigma_w,
        turbulence_intensity: ti,
        tke,
    }
}

/// Eddy covariance flux results.
#[derive(Clone, Debug)]
pub struct EddyCovarianceFluxes {
    /// Momentum flux (Reynolds stress) tau_uw = -rho * <u'w'> [Pa].
    pub momentum_flux_uw: f64,
    /// Momentum flux tau_vw = -rho * <v'w'> [Pa].
    pub momentum_flux_vw: f64,
    /// Sensible heat flux H = rho*cp * <w'T'> [W/m^2].
    pub sensible_heat_flux: f64,
    /// Friction velocity u* [m/s].
    pub friction_velocity: f64,
    /// Obukhov length L [m].
    pub obukhov_length: f64,
    /// Stability parameter z/L (z = measurement height).
    pub stability_parameter: f64,
    /// Covariance <u'w'> [m^2/s^2].
    pub cov_uw: f64,
    /// Covariance <v'w'> [m^2/s^2].
    pub cov_vw: f64,
    /// Covariance <w'T'> [m*K/s].
    pub cov_wt: f64,
}

/// Compute eddy covariance fluxes from wind and temperature fluctuations.
///
/// - `winds`: wind vector time series (should be coordinate-rotated)
/// - `temperatures`: sonic temperature time series [K]
/// - `measurement_height`: height above ground [m]
pub fn eddy_covariance_fluxes(
    winds: &[WindVector],
    temperatures: &[f64],
    measurement_height: f64,
) -> EddyCovarianceFluxes {
    let n = winds.len();
    assert_eq!(n, temperatures.len(), "winds and temperatures must have same length");
    assert!(n > 1, "need at least 2 records");

    let nf = n as f64;

    // Compute means
    let mut u_mean = 0.0;
    let mut v_mean = 0.0;
    let mut w_mean = 0.0;
    let mut t_mean = 0.0;
    for (w, &t) in winds.iter().zip(temperatures.iter()) {
        u_mean += w.u;
        v_mean += w.v;
        w_mean += w.w;
        t_mean += t;
    }
    u_mean /= nf;
    v_mean /= nf;
    w_mean /= nf;
    t_mean /= nf;

    // Compute covariances
    let mut cov_uw = 0.0;
    let mut cov_vw = 0.0;
    let mut cov_wt = 0.0;
    for (w, &t) in winds.iter().zip(temperatures.iter()) {
        let u_prime = w.u - u_mean;
        let v_prime = w.v - v_mean;
        let w_prime = w.w - w_mean;
        let t_prime = t - t_mean;
        cov_uw += u_prime * w_prime;
        cov_vw += v_prime * w_prime;
        cov_wt += w_prime * t_prime;
    }
    cov_uw /= nf;
    cov_vw /= nf;
    cov_wt /= nf;

    // Friction velocity: u* = (cov_uw^2 + cov_vw^2)^(1/4)
    let u_star = (cov_uw * cov_uw + cov_vw * cov_vw).sqrt().sqrt();

    // Momentum fluxes
    let momentum_flux_uw = -RHO_AIR * cov_uw;
    let momentum_flux_vw = -RHO_AIR * cov_vw;

    // Sensible heat flux
    let sensible_heat_flux = RHO_AIR * CP_AIR * cov_wt;

    // Obukhov length: L = -u*^3 * T_mean / (kappa * g * <w'T'>)
    let obukhov_length = if cov_wt.abs() > 1e-15 {
        -(u_star.powi(3) * t_mean) / (VON_KARMAN * GRAVITY * cov_wt)
    } else {
        f64::INFINITY
    };

    // Stability parameter
    let stability = if obukhov_length.is_finite() && obukhov_length.abs() > 1e-10 {
        measurement_height / obukhov_length
    } else {
        0.0
    };

    EddyCovarianceFluxes {
        momentum_flux_uw,
        momentum_flux_vw,
        sensible_heat_flux,
        friction_velocity: u_star,
        obukhov_length,
        stability_parameter: stability,
        cov_uw,
        cov_vw,
        cov_wt,
    }
}

/// Power spectral density via DFT (simple periodogram).
///
/// Returns (frequencies [Hz], power spectral density [units^2/Hz]).
pub fn power_spectrum(data: &[f64], sample_rate: f64) -> (Vec<f64>, Vec<f64>) {
    let n = data.len();
    assert!(n > 0);

    // Apply Hann window
    let windowed: Vec<f64> = data
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            x * w
        })
        .collect();

    // DFT (real input, only positive frequencies)
    let n_fft = n / 2 + 1;
    let mut psd = Vec::with_capacity(n_fft);
    let mut freqs = Vec::with_capacity(n_fft);

    // Window power for normalization
    let window_power: f64 = (0..n)
        .map(|i| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            w * w
        })
        .sum::<f64>();

    for k in 0..n_fft {
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &x) in windowed.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        let mag_sq = re * re + im * im;
        // One-sided PSD: multiply by 2 for k > 0 and k < N/2
        let scale = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
        psd.push(scale * mag_sq / (sample_rate * window_power));
        freqs.push(k as f64 * sample_rate / n as f64);
    }

    (freqs, psd)
}

/// Cross-spectrum (cospectrum) between two signals.
///
/// Returns (frequencies [Hz], cospectral density [units_a * units_b / Hz]).
pub fn cospectrum(a: &[f64], b: &[f64], sample_rate: f64) -> (Vec<f64>, Vec<f64>) {
    let n = a.len();
    assert_eq!(n, b.len());
    assert!(n > 0);

    // Apply Hann window
    let win_a: Vec<f64> = a
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            x * w
        })
        .collect();
    let win_b: Vec<f64> = b
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            x * w
        })
        .collect();

    let window_power: f64 = (0..n)
        .map(|i| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            w * w
        })
        .sum::<f64>();

    let n_fft = n / 2 + 1;
    let mut cospec = Vec::with_capacity(n_fft);
    let mut freqs = Vec::with_capacity(n_fft);

    for k in 0..n_fft {
        let mut re_a = 0.0;
        let mut im_a = 0.0;
        let mut re_b = 0.0;
        let mut im_b = 0.0;
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            re_a += win_a[j] * cos_a;
            im_a += win_a[j] * sin_a;
            re_b += win_b[j] * cos_a;
            im_b += win_b[j] * sin_a;
        }
        // Co-spectrum = Re(A* . B) = Re_A*Re_B + Im_A*Im_B
        let co = re_a * re_b + im_a * im_b;
        let scale = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
        cospec.push(scale * co / (sample_rate * window_power));
        freqs.push(k as f64 * sample_rate / n as f64);
    }

    (freqs, cospec)
}

/// Check if a power spectrum follows the Kolmogorov -5/3 law in a given
/// frequency range.
///
/// Fits log(PSD) = a - (5/3) * log(f) and returns the R-squared goodness of fit.
pub fn kolmogorov_fit(freqs: &[f64], psd: &[f64], f_min: f64, f_max: f64) -> f64 {
    assert_eq!(freqs.len(), psd.len());

    // Select inertial subrange
    let mut log_f = Vec::new();
    let mut log_psd = Vec::new();
    for (&f, &p) in freqs.iter().zip(psd.iter()) {
        if f >= f_min && f <= f_max && f > 0.0 && p > 0.0 {
            log_f.push(f.ln());
            log_psd.push(p.ln());
        }
    }

    if log_f.len() < 3 {
        return 0.0;
    }

    let n = log_f.len() as f64;
    let mean_log_psd: f64 = log_psd.iter().sum::<f64>() / n;

    // Predicted: log(PSD) = a + (-5/3) * log(f)
    // Find best-fit offset a given slope = -5/3
    let slope = -5.0 / 3.0;
    let mean_log_f: f64 = log_f.iter().sum::<f64>() / n;
    let a = mean_log_psd - slope * mean_log_f;

    // R-squared
    let mut ss_res = 0.0;
    let mut ss_tot = 0.0;
    for (lf, lp) in log_f.iter().zip(log_psd.iter()) {
        let predicted = a + slope * lf;
        ss_res += (lp - predicted) * (lp - predicted);
        ss_tot += (lp - mean_log_psd) * (lp - mean_log_psd);
    }

    if ss_tot < 1e-30 {
        return 1.0;
    }

    1.0 - ss_res / ss_tot
}

/// Autocorrelation function of a signal.
///
/// Returns autocorrelation coefficients for lags 0..max_lag, normalized so R(0) = 1.
pub fn autocorrelation(data: &[f64], max_lag: usize) -> Vec<f64> {
    let n = data.len();
    assert!(n > 1);
    let mean: f64 = data.iter().sum::<f64>() / n as f64;
    let var: f64 = data.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / n as f64;

    if var < 1e-30 {
        return vec![1.0; max_lag.min(n)];
    }

    let lags = max_lag.min(n - 1);
    let mut acf = Vec::with_capacity(lags + 1);
    for lag in 0..=lags {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += (data[i] - mean) * (data[i + lag] - mean);
        }
        acf.push(sum / (n as f64 * var));
    }
    acf
}

/// Integral length scale from autocorrelation.
///
/// Integrates autocorrelation from lag 0 to first zero crossing,
/// multiplied by mean wind / sample_rate to convert to meters.
pub fn integral_length_scale(acf: &[f64], mean_wind: f64, sample_rate: f64) -> f64 {
    let mut integral = 0.0;
    for &r in acf.iter().skip(1) {
        if r <= 0.0 {
            break;
        }
        integral += r;
    }
    // Convert lag sum to time (dt = 1/sample_rate), then to length via Taylor hypothesis
    let integral_time = integral / sample_rate;
    integral_time * mean_wind.abs()
}

/// Quality control: spike detection using median absolute deviation.
///
/// Returns indices of detected spikes (values exceeding `threshold` * MAD from median).
pub fn detect_spikes(data: &[f64], threshold: f64) -> Vec<usize> {
    let n = data.len();
    if n < 3 {
        return Vec::new();
    }

    // Compute median
    let mut sorted = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = if n % 2 == 0 {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    } else {
        sorted[n / 2]
    };

    // Compute MAD (median absolute deviation)
    let mut deviations: Vec<f64> = data.iter().map(|&x| (x - median).abs()).collect();
    deviations.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mad = if n % 2 == 0 {
        (deviations[n / 2 - 1] + deviations[n / 2]) / 2.0
    } else {
        deviations[n / 2]
    };

    // Scale MAD to estimate sigma: sigma ≈ 1.4826 * MAD
    let sigma_est = 1.4826 * mad;

    if sigma_est < 1e-15 {
        return Vec::new();
    }

    let mut spikes = Vec::new();
    for (i, &x) in data.iter().enumerate() {
        if (x - median).abs() > threshold * sigma_est {
            spikes.push(i);
        }
    }
    spikes
}

/// Remove spikes by replacing with linear interpolation from neighbors.
pub fn despike(data: &mut [f64], spike_indices: &[usize]) {
    let n = data.len();
    for &idx in spike_indices {
        if idx == 0 {
            // Replace with next non-spike value
            if n > 1 {
                data[0] = data[1];
            }
        } else if idx >= n - 1 {
            if n > 1 {
                data[n - 1] = data[n - 2];
            }
        } else {
            // Linear interpolation
            data[idx] = (data[idx - 1] + data[idx + 1]) / 2.0;
        }
    }
}

/// Stationarity test: compare sub-period means to the overall mean.
///
/// Splits the record into `num_segments` segments, computes the RMS difference
/// of segment means from the overall mean, normalized by the overall standard deviation.
/// Values < 0.5 typically indicate stationarity.
pub fn stationarity_test(data: &[f64], num_segments: usize) -> f64 {
    let n = data.len();
    assert!(num_segments > 0 && n >= num_segments);

    let overall_mean = data.iter().sum::<f64>() / n as f64;
    let overall_var: f64 = data.iter().map(|&x| (x - overall_mean).powi(2)).sum::<f64>() / n as f64;
    let overall_sigma = overall_var.sqrt();

    if overall_sigma < 1e-15 {
        return 0.0;
    }

    let seg_len = n / num_segments;
    let mut rms_diff = 0.0;

    for s in 0..num_segments {
        let start = s * seg_len;
        let end = if s == num_segments - 1 { n } else { start + seg_len };
        let seg_mean: f64 = data[start..end].iter().sum::<f64>() / (end - start) as f64;
        let diff = seg_mean - overall_mean;
        rms_diff += diff * diff;
    }

    rms_diff = (rms_diff / num_segments as f64).sqrt();
    rms_diff / overall_sigma
}

/// Integral turbulence test: compare measured sigma_w/u* to predicted value
/// from Monin-Obukhov stability.
///
/// For near-neutral conditions, sigma_w/u* ≈ 1.25.
/// Returns the ratio of measured / predicted.
pub fn integral_turbulence_test(sigma_w: f64, u_star: f64, stability: f64) -> f64 {
    if u_star < 1e-10 {
        return f64::NAN;
    }

    let measured_ratio = sigma_w / u_star;

    // Predicted ratio from Monin-Obukhov similarity
    // For unstable (z/L < 0): sigma_w/u* = 1.25 * (1 - 3*z/L)^(1/3)
    // For near-neutral: sigma_w/u* ≈ 1.25
    // For stable (z/L > 0): sigma_w/u* = 1.25 * (1 + 0.2*z/L) (capped)
    let predicted_ratio = if stability < -0.01 {
        1.25 * (1.0 - 3.0 * stability).powf(1.0 / 3.0)
    } else if stability > 0.01 {
        1.25 * (1.0 + 0.2 * stability).min(2.5)
    } else {
        1.25
    };

    measured_ratio / predicted_ratio
}

/// Wind direction sector exclusion check.
///
/// Returns true if the wind direction falls within the excluded sector
/// (e.g., tower wake). Sector is defined by center direction and half-width.
pub fn is_in_excluded_sector(direction_deg: f64, sector_center_deg: f64, sector_halfwidth_deg: f64) -> bool {
    let mut diff = (direction_deg - sector_center_deg) % 360.0;
    if diff > 180.0 {
        diff -= 360.0;
    }
    if diff < -180.0 {
        diff += 360.0;
    }
    diff.abs() <= sector_halfwidth_deg
}

/// Main sonic anemometer processor.
///
/// Processes raw transit-time data into calibrated wind vectors,
/// performs quality control, coordinate rotation, and computes
/// turbulence statistics and fluxes.
pub struct SonicAnemometerProcessor {
    config: SonicConfig,
    /// Accumulated wind vectors for the current averaging period.
    wind_buffer: Vec<WindVector>,
    /// Accumulated sonic temperatures for the current averaging period.
    temp_buffer: Vec<f64>,
    /// Measurement height above ground [m].
    measurement_height: f64,
    /// Excluded wind direction sectors: (center_deg, halfwidth_deg).
    excluded_sectors: Vec<(f64, f64)>,
}

impl SonicAnemometerProcessor {
    /// Create a new processor with given configuration.
    pub fn new(config: SonicConfig, measurement_height: f64) -> Self {
        Self {
            config,
            wind_buffer: Vec::new(),
            temp_buffer: Vec::new(),
            measurement_height,
            excluded_sectors: Vec::new(),
        }
    }

    /// Add an excluded wind direction sector (e.g., tower wake).
    pub fn add_excluded_sector(&mut self, center_deg: f64, halfwidth_deg: f64) {
        self.excluded_sectors.push((center_deg, halfwidth_deg));
    }

    /// Process a single set of transit-time measurements.
    ///
    /// For a 3-axis sonic: 3 pairs of (t_forward, t_reverse).
    /// Returns the wind vector and sonic temperature.
    pub fn process_transit_times(
        &mut self,
        transit_times: &[(f64, f64)],
        azimuth_angles_deg: &[f64],
        elevation_angles_deg: &[f64],
    ) -> (WindVector, f64) {
        assert_eq!(transit_times.len(), self.config.num_axes);
        assert!(transit_times.len() <= 3);

        let mut path_winds = [0.0f64; 3];
        let mut speed_of_sound_sum = 0.0;

        for (i, &(t_fwd, t_rev)) in transit_times.iter().enumerate() {
            let meas = transit_time_measurement(self.config.path_length_m, t_fwd, t_rev);
            path_winds[i] = meas.wind_along_path;
            speed_of_sound_sum += meas.speed_of_sound;
        }

        let avg_sos = speed_of_sound_sum / transit_times.len() as f64;
        let t_sonic = sonic_temperature(avg_sos);

        let wind = if self.config.num_axes == 3 {
            let mut az = [0.0f64; 3];
            let mut el = [0.0f64; 3];
            for i in 0..3 {
                az[i] = azimuth_angles_deg[i];
                el[i] = elevation_angles_deg[i];
            }
            orthogonal_3axis_to_uvw(&path_winds, &az, &el)
        } else {
            // 2-axis: horizontal only
            let az0 = azimuth_angles_deg[0] * PI / 180.0;
            let az1 = azimuth_angles_deg[1] * PI / 180.0;
            // Simple 2-axis decomposition (no vertical)
            let det = az0.sin() * az1.cos() - az1.sin() * az0.cos();
            if det.abs() > 1e-15 {
                let u = (path_winds[0] * az1.cos() - path_winds[1] * az0.cos()) / det;
                let v = (path_winds[1] * az0.sin() - path_winds[0] * az1.sin()) / det;
                WindVector::new(u, v, 0.0)
            } else {
                WindVector::default()
            }
        };

        self.wind_buffer.push(wind.clone());
        self.temp_buffer.push(t_sonic);

        (wind, t_sonic)
    }

    /// Get the current buffer of wind vectors.
    pub fn wind_buffer(&self) -> &[WindVector] {
        &self.wind_buffer
    }

    /// Get the current buffer of sonic temperatures.
    pub fn temp_buffer(&self) -> &[f64] {
        &self.temp_buffer
    }

    /// Clear the internal buffers.
    pub fn clear_buffers(&mut self) {
        self.wind_buffer.clear();
        self.temp_buffer.clear();
    }

    /// Compute full analysis on accumulated data.
    ///
    /// Performs quality control, coordinate rotation, and computes
    /// turbulence statistics and eddy covariance fluxes.
    pub fn analyze(&self) -> Option<SonicAnalysisResult> {
        if self.wind_buffer.len() < 10 {
            return None;
        }

        // Extract component arrays
        let u_raw: Vec<f64> = self.wind_buffer.iter().map(|w| w.u).collect();
        let v_raw: Vec<f64> = self.wind_buffer.iter().map(|w| w.v).collect();
        let w_raw: Vec<f64> = self.wind_buffer.iter().map(|w| w.w).collect();

        // Spike detection
        let u_spikes = detect_spikes(&u_raw, 3.5);
        let v_spikes = detect_spikes(&v_raw, 3.5);
        let w_spikes = detect_spikes(&w_raw, 3.5);
        let total_spikes = u_spikes.len() + v_spikes.len() + w_spikes.len();

        // Double rotation
        let (rotated, rotation_angles) = double_rotation(&self.wind_buffer);

        // Turbulence statistics on rotated data
        let turb_stats = turbulence_statistics(&rotated);

        // Eddy covariance fluxes
        let fluxes = eddy_covariance_fluxes(&rotated, &self.temp_buffer, self.measurement_height);

        // Stationarity test on u-component
        let u_rotated: Vec<f64> = rotated.iter().map(|w| w.u).collect();
        let stationarity = if u_rotated.len() >= 6 {
            stationarity_test(&u_rotated, 6)
        } else {
            0.0
        };

        // Integral turbulence test
        let it_test = integral_turbulence_test(
            turb_stats.sigma_w,
            fluxes.friction_velocity,
            fluxes.stability_parameter,
        );

        // Direction check
        let mean_wind = WindVector::new(turb_stats.u_mean, turb_stats.v_mean, turb_stats.w_mean);
        let dir = mean_wind.direction_deg();
        let in_excluded = self
            .excluded_sectors
            .iter()
            .any(|&(center, half)| is_in_excluded_sector(dir, center, half));

        Some(SonicAnalysisResult {
            turbulence_stats: turb_stats,
            fluxes,
            rotation_angles,
            total_spikes,
            stationarity_ratio: stationarity,
            integral_turbulence_ratio: it_test,
            in_excluded_sector: in_excluded,
            num_samples: self.wind_buffer.len(),
        })
    }
}

/// Complete analysis result from sonic anemometer processing.
#[derive(Clone, Debug)]
pub struct SonicAnalysisResult {
    pub turbulence_stats: TurbulenceStats,
    pub fluxes: EddyCovarianceFluxes,
    pub rotation_angles: RotationAngles,
    pub total_spikes: usize,
    pub stationarity_ratio: f64,
    pub integral_turbulence_ratio: f64,
    pub in_excluded_sector: bool,
    pub num_samples: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol || (a.is_infinite() && b.is_infinite() && a.signum() == b.signum())
    }

    // --- Transit time and temperature tests ---

    #[test]
    fn test_transit_time_no_wind() {
        // Equal transit times → zero wind, nonzero speed of sound
        let path_len = 0.15;
        let c = 343.0; // m/s
        let t = path_len / c;
        let m = transit_time_measurement(path_len, t, t);
        assert!(m.wind_along_path.abs() < TOL);
        assert!(approx_eq(m.speed_of_sound, c, TOL_LOOSE));
    }

    #[test]
    fn test_transit_time_with_wind() {
        let path_len = 0.15;
        let c = 343.0;
        let v_wind = 5.0;
        // t_forward = L / (c + v), t_reverse = L / (c - v)
        let t_fwd = path_len / (c + v_wind);
        let t_rev = path_len / (c - v_wind);
        let m = transit_time_measurement(path_len, t_fwd, t_rev);
        assert!(approx_eq(m.wind_along_path, v_wind, TOL_LOOSE));
        assert!(approx_eq(m.speed_of_sound, c, TOL_LOOSE));
    }

    #[test]
    fn test_sonic_temperature() {
        // At c = 343 m/s, T ≈ 293 K ≈ 20°C
        let c = 343.0;
        let t = sonic_temperature(c);
        assert!(t > 290.0 && t < 296.0, "T_sonic = {}", t);
    }

    #[test]
    fn test_virtual_temperature_dry() {
        let t_sonic = 293.0;
        let t_v = virtual_temperature(t_sonic, 0.0);
        assert!(approx_eq(t_v, t_sonic, TOL));
    }

    #[test]
    fn test_virtual_temperature_humid() {
        let t_sonic = 293.0;
        let q = 0.01; // 10 g/kg
        let t_v = virtual_temperature(t_sonic, q);
        // T_v should be slightly higher than T_sonic
        assert!(t_v > t_sonic);
        assert!(approx_eq(t_v, 293.0 * 1.0051, TOL_LOOSE));
    }

    // --- Wind vector tests ---

    #[test]
    fn test_wind_vector_horizontal_speed() {
        let w = WindVector::new(3.0, 4.0, 0.0);
        assert!(approx_eq(w.horizontal_speed(), 5.0, TOL));
    }

    #[test]
    fn test_wind_vector_3d_speed() {
        let w = WindVector::new(1.0, 2.0, 2.0);
        assert!(approx_eq(w.speed_3d(), 3.0, TOL));
    }

    #[test]
    fn test_wind_direction_north() {
        // Wind from north: u=0, v negative (southward flow means FROM north)
        let w = WindVector::new(0.0, -1.0, 0.0);
        let dir = w.direction_deg();
        assert!(approx_eq(dir, 0.0, 1.0) || approx_eq(dir, 360.0, 1.0),
            "Expected ~0 or ~360, got {}", dir);
    }

    #[test]
    fn test_wind_direction_south() {
        // Wind from south: u=0, v positive
        let w = WindVector::new(0.0, 1.0, 0.0);
        let dir = w.direction_deg();
        assert!(approx_eq(dir, 180.0, 1.0), "Expected ~180, got {}", dir);
    }

    #[test]
    fn test_wind_direction_west() {
        // Wind from west: u positive, v=0
        let w = WindVector::new(1.0, 0.0, 0.0);
        let dir = w.direction_deg();
        assert!(approx_eq(dir, 270.0, 1.0), "Expected ~270, got {}", dir);
    }

    #[test]
    fn test_wind_direction_east() {
        // Wind from east: u negative, v=0
        let w = WindVector::new(-1.0, 0.0, 0.0);
        let dir = w.direction_deg();
        assert!(approx_eq(dir, 90.0, 1.0), "Expected ~90, got {}", dir);
    }

    // --- 3D coordinate transform tests ---

    #[test]
    fn test_orthogonal_3axis_identity() {
        // Paths along x, y, z axes → direct decomposition
        let path_winds = [3.0, 4.0, 1.0];
        let azimuth = [90.0, 0.0, 0.0]; // axis 1: east, axis 2: north
        let elevation = [0.0, 0.0, 90.0]; // axis 3: vertical
        let w = orthogonal_3axis_to_uvw(&path_winds, &azimuth, &elevation);
        assert!(approx_eq(w.u, 3.0, TOL_LOOSE));
        assert!(approx_eq(w.v, 4.0, TOL_LOOSE));
        assert!(approx_eq(w.w, 1.0, TOL_LOOSE));
    }

    #[test]
    fn test_non_orthogonal_identity_matrix() {
        let path_winds = [2.0, 3.0, 1.0];
        let identity = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let w = non_orthogonal_transform(&path_winds, &identity);
        assert!(approx_eq(w.u, 2.0, TOL));
        assert!(approx_eq(w.v, 3.0, TOL));
        assert!(approx_eq(w.w, 1.0, TOL));
    }

    #[test]
    fn test_non_orthogonal_scaling() {
        let path_winds = [1.0, 1.0, 1.0];
        let mat = [
            [2.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [0.0, 0.0, 0.5],
        ];
        let w = non_orthogonal_transform(&path_winds, &mat);
        assert!(approx_eq(w.u, 2.0, TOL));
        assert!(approx_eq(w.v, 3.0, TOL));
        assert!(approx_eq(w.w, 0.5, TOL));
    }

    // --- Coordinate rotation tests ---

    #[test]
    fn test_double_rotation_zeros_mean_vw() {
        let winds: Vec<WindVector> = (0..100)
            .map(|i| {
                let t = i as f64 / 100.0;
                WindVector::new(
                    5.0 + 0.3 * (2.0 * PI * t).sin(),
                    1.0 + 0.2 * (3.0 * PI * t).cos(),
                    0.5 + 0.1 * (4.0 * PI * t).sin(),
                )
            })
            .collect();

        let (rotated, _angles) = double_rotation(&winds);

        let n = rotated.len() as f64;
        let v_mean: f64 = rotated.iter().map(|w| w.v).sum::<f64>() / n;
        let w_mean: f64 = rotated.iter().map(|w| w.w).sum::<f64>() / n;

        assert!(v_mean.abs() < 0.01, "mean v should be ~0, got {}", v_mean);
        assert!(w_mean.abs() < 0.01, "mean w should be ~0, got {}", w_mean);
    }

    #[test]
    fn test_planar_fit_removes_w_bias() {
        let winds: Vec<WindVector> = (0..200)
            .map(|i| {
                let t = i as f64 / 200.0;
                // w has systematic tilt: w = 0.1 * u + 0.05 * v + 0.2
                let u = 5.0 + 0.5 * (2.0 * PI * t).sin();
                let v = 2.0 + 0.3 * (3.0 * PI * t).cos();
                let w = 0.1 * u + 0.05 * v + 0.2 + 0.1 * (5.0 * PI * t).sin();
                WindVector::new(u, v, w)
            })
            .collect();

        let (corrected, coeffs) = planar_fit(&winds);

        // Check coefficients recovered
        assert!(approx_eq(coeffs.b0, 0.2, 0.05), "b0 = {}", coeffs.b0);
        assert!(approx_eq(coeffs.b1, 0.1, 0.02), "b1 = {}", coeffs.b1);
        assert!(approx_eq(coeffs.b2, 0.05, 0.02), "b2 = {}", coeffs.b2);

        // Mean corrected w should be near zero
        let mean_w: f64 = corrected.iter().map(|w| w.w).sum::<f64>() / corrected.len() as f64;
        assert!(mean_w.abs() < 0.1, "mean corrected w = {}", mean_w);
    }

    // --- Reynolds decomposition and turbulence tests ---

    #[test]
    fn test_reynolds_decomposition() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let (mean, fluct) = reynolds_decomposition(&data);
        assert!(approx_eq(mean, 3.0, TOL));
        assert!(approx_eq(fluct[0], -2.0, TOL));
        assert!(approx_eq(fluct[4], 2.0, TOL));
        // Sum of fluctuations should be zero
        let sum: f64 = fluct.iter().sum();
        assert!(sum.abs() < TOL);
    }

    #[test]
    fn test_turbulence_statistics_uniform() {
        // No turbulence: constant wind
        let winds: Vec<WindVector> = (0..50).map(|_| WindVector::new(5.0, 0.0, 0.0)).collect();
        let stats = turbulence_statistics(&winds);
        assert!(approx_eq(stats.u_mean, 5.0, TOL));
        assert!(stats.var_u < TOL);
        assert!(stats.tke < TOL);
        assert!(approx_eq(stats.turbulence_intensity, 0.0, TOL));
    }

    #[test]
    fn test_turbulence_statistics_known() {
        let winds: Vec<WindVector> = (0..1000)
            .map(|i| {
                let t = i as f64 / 1000.0;
                WindVector::new(
                    5.0 + (2.0 * PI * t).sin(),
                    0.0,
                    0.0,
                )
            })
            .collect();
        let stats = turbulence_statistics(&winds);
        assert!(approx_eq(stats.u_mean, 5.0, 0.01));
        // Variance of sin(2*pi*t) over [0,1) ≈ 0.5
        assert!(approx_eq(stats.var_u, 0.5, 0.02));
        assert!(stats.tke > 0.0);
    }

    #[test]
    fn test_tke_formula() {
        // TKE = 0.5 * (var_u + var_v + var_w)
        let winds = vec![
            WindVector::new(1.0, 1.0, 1.0),
            WindVector::new(-1.0, -1.0, -1.0),
        ];
        let stats = turbulence_statistics(&winds);
        // mean = 0 for all, var = 1 for all
        assert!(approx_eq(stats.tke, 1.5, TOL));
    }

    // --- Eddy covariance tests ---

    #[test]
    fn test_eddy_covariance_no_flux() {
        // Uncorrelated w and T → zero heat flux
        let winds: Vec<WindVector> = (0..100)
            .map(|i| WindVector::new(5.0, 0.0, if i % 2 == 0 { 0.1 } else { -0.1 }))
            .collect();
        let temps: Vec<f64> = (0..100)
            .map(|i| 293.0 + if (i / 3) % 2 == 0 { 0.5 } else { -0.5 })
            .collect();
        let fluxes = eddy_covariance_fluxes(&winds, &temps, 3.0);
        // Heat flux should be small (not exactly zero due to discrete correlation)
        assert!(fluxes.sensible_heat_flux.abs() < 100.0);
    }

    #[test]
    fn test_friction_velocity_definition() {
        // Construct data with known cov_uw
        let n = 1000;
        let mut winds = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / n as f64;
            let u = 5.0 + (2.0 * PI * t).sin();
            let w = 0.3 * (2.0 * PI * t).sin(); // correlated with u
            winds.push(WindVector::new(u, 0.0, w));
        }
        let temps: Vec<f64> = vec![293.0; n];
        let fluxes = eddy_covariance_fluxes(&winds, &temps, 3.0);

        // cov_uw should be positive (u and w positively correlated)
        assert!(fluxes.cov_uw > 0.0);
        // u* = (cov_uw^2 + cov_vw^2)^(1/4)
        let expected_ustar = (fluxes.cov_uw.powi(2) + fluxes.cov_vw.powi(2)).sqrt().sqrt();
        assert!(approx_eq(fluxes.friction_velocity, expected_ustar, TOL));
    }

    #[test]
    fn test_obukhov_length_neutral() {
        // Zero heat flux → infinite Obukhov length
        let winds: Vec<WindVector> = (0..100).map(|_| WindVector::new(5.0, 0.0, 0.0)).collect();
        let temps: Vec<f64> = vec![293.0; 100];
        let fluxes = eddy_covariance_fluxes(&winds, &temps, 3.0);
        assert!(fluxes.obukhov_length.is_infinite() || fluxes.obukhov_length.abs() > 1e10);
    }

    // --- Spectral analysis tests ---

    #[test]
    fn test_power_spectrum_single_tone() {
        let n = 256;
        let fs = 20.0;
        let f0 = 3.0;
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).sin())
            .collect();
        let (freqs, psd) = power_spectrum(&data, fs);

        // Find peak frequency
        let (peak_idx, _) = psd
            .iter()
            .enumerate()
            .skip(1) // skip DC
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_freq = freqs[peak_idx];

        // Peak should be near 3 Hz
        assert!((peak_freq - f0).abs() < fs / n as f64 * 2.0,
            "peak at {} Hz, expected {} Hz", peak_freq, f0);
    }

    #[test]
    fn test_cospectrum_correlated() {
        let n = 128;
        let fs = 20.0;
        let f0 = 2.0;
        let a: Vec<f64> = (0..n).map(|i| (2.0 * PI * f0 * i as f64 / fs).sin()).collect();
        let b: Vec<f64> = (0..n).map(|i| (2.0 * PI * f0 * i as f64 / fs).sin()).collect();
        let (_freqs, cospec) = cospectrum(&a, &b, fs);

        // Sum of cospectrum should be positive for positively correlated signals
        let total: f64 = cospec.iter().sum();
        assert!(total > 0.0, "total cospectrum = {}", total);
    }

    #[test]
    fn test_autocorrelation_periodic() {
        let n = 200;
        let data: Vec<f64> = (0..n).map(|i| (2.0 * PI * i as f64 / 20.0).sin()).collect();
        let acf = autocorrelation(&data, 40);
        // ACF at lag 0 should be 1.0
        assert!(approx_eq(acf[0], 1.0, TOL));
        // ACF at lag = period (20 samples) should be close to 1
        assert!(acf[20] > 0.8, "ACF(20) = {}", acf[20]);
    }

    #[test]
    fn test_integral_length_scale() {
        // Construct ACF that decays exponentially
        let acf: Vec<f64> = (0..50).map(|i| (-i as f64 / 10.0).exp()).collect();
        let mean_wind = 5.0;
        let sample_rate = 20.0;
        let ils = integral_length_scale(&acf, mean_wind, sample_rate);
        // Integral of exp(-t/10) from 1 to ∞ ≈ sum ≈ 10 (in samples)
        // Time = 10/20 = 0.5s, length = 0.5 * 5.0 = 2.5m approximately
        assert!(ils > 0.0, "ILS = {}", ils);
    }

    #[test]
    fn test_kolmogorov_fit_synthetic() {
        // Create a synthetic -5/3 spectrum
        let n = 50;
        let freqs: Vec<f64> = (1..=n).map(|i| i as f64 * 0.1).collect();
        let psd: Vec<f64> = freqs.iter().map(|&f| 10.0 * f.powf(-5.0 / 3.0)).collect();
        let r2 = kolmogorov_fit(&freqs, &psd, 0.1, 5.0);
        assert!(r2 > 0.99, "R^2 = {}", r2);
    }

    // --- Quality control tests ---

    #[test]
    fn test_detect_spikes_none() {
        let data: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let spikes = detect_spikes(&data, 3.5);
        assert!(spikes.is_empty(), "found {} spikes in clean data", spikes.len());
    }

    #[test]
    fn test_detect_spikes_present() {
        let mut data: Vec<f64> = vec![0.0; 100];
        for i in 0..100 {
            data[i] = (i as f64 * 0.1).sin();
        }
        // Insert obvious spikes
        data[25] = 100.0;
        data[75] = -100.0;
        let spikes = detect_spikes(&data, 3.5);
        assert!(spikes.contains(&25), "missed spike at index 25");
        assert!(spikes.contains(&75), "missed spike at index 75");
    }

    #[test]
    fn test_despike() {
        let mut data = vec![1.0, 2.0, 100.0, 4.0, 5.0];
        despike(&mut data, &[2]);
        assert!(approx_eq(data[2], 3.0, TOL)); // (2+4)/2
    }

    #[test]
    fn test_stationarity_stationary() {
        // Stationary: constant mean with small rapid oscillation
        // Use a full-period sine so segment means are close to the overall mean
        let data: Vec<f64> = (0..600)
            .map(|i| 5.0 + 0.5 * (2.0 * PI * 6.0 * i as f64 / 600.0).sin())
            .collect();
        let ratio = stationarity_test(&data, 6);
        assert!(ratio < 0.5, "stationarity ratio = {}", ratio);
    }

    #[test]
    fn test_stationarity_nonstationary() {
        // Non-stationary: step change in mean
        let mut data = vec![0.0; 600];
        for i in 0..300 {
            data[i] = 2.0 + 0.1 * (i as f64 * 0.1).sin();
        }
        for i in 300..600 {
            data[i] = 8.0 + 0.1 * (i as f64 * 0.1).sin();
        }
        let ratio = stationarity_test(&data, 6);
        assert!(ratio > 0.3, "stationarity ratio = {} (expected large)", ratio);
    }

    #[test]
    fn test_integral_turbulence_neutral() {
        // At neutral stability, sigma_w/u* ≈ 1.25
        let ratio = integral_turbulence_test(1.25, 1.0, 0.0);
        assert!(approx_eq(ratio, 1.0, 0.01), "IT ratio = {}", ratio);
    }

    #[test]
    fn test_integral_turbulence_unstable() {
        let ratio = integral_turbulence_test(2.0, 1.0, -1.0);
        // Should be reasonable ratio
        assert!(ratio > 0.0 && ratio < 5.0, "IT ratio = {}", ratio);
    }

    // --- Sector exclusion tests ---

    #[test]
    fn test_sector_exclusion_inside() {
        assert!(is_in_excluded_sector(180.0, 180.0, 15.0));
        assert!(is_in_excluded_sector(190.0, 180.0, 15.0));
        assert!(is_in_excluded_sector(170.0, 180.0, 15.0));
    }

    #[test]
    fn test_sector_exclusion_outside() {
        assert!(!is_in_excluded_sector(150.0, 180.0, 15.0));
        assert!(!is_in_excluded_sector(210.0, 180.0, 15.0));
    }

    #[test]
    fn test_sector_exclusion_wraparound() {
        // Sector centered at 350 deg with halfwidth 20 should include 5 deg
        assert!(is_in_excluded_sector(5.0, 350.0, 20.0));
        assert!(is_in_excluded_sector(355.0, 350.0, 20.0));
    }

    // --- Processor integration tests ---

    #[test]
    fn test_sonic_config_presets() {
        let csat3 = SonicConfig::csat3();
        assert_eq!(csat3.num_axes, 3);
        assert!(csat3.path_length_m > 0.0);

        let gill = SonicConfig::gill_r3();
        assert_eq!(gill.num_axes, 3);
        assert!(gill.path_length_m > 0.0);
    }

    #[test]
    fn test_processor_transit_times() {
        let config = SonicConfig::new(0.15, 0.0, 3, 20.0);
        let mut proc = SonicAnemometerProcessor::new(config, 3.0);

        let c = 343.0;
        let winds = [3.0, 2.0, 0.5]; // path winds
        let azimuths = [90.0, 0.0, 0.0]; // east, north, north
        let elevations = [0.0, 0.0, 90.0]; // horizontal, horizontal, vertical

        // Compute transit times for each path
        let mut transit_times = Vec::new();
        for &w in &winds {
            let t_fwd = 0.15 / (c + w);
            let t_rev = 0.15 / (c - w);
            transit_times.push((t_fwd, t_rev));
        }

        let (wind, temp) = proc.process_transit_times(
            &transit_times,
            &azimuths,
            &elevations,
        );

        assert!(approx_eq(wind.u, 3.0, 0.1), "u = {}", wind.u);
        assert!(approx_eq(wind.v, 2.0, 0.1), "v = {}", wind.v);
        assert!(approx_eq(wind.w, 0.5, 0.1), "w = {}", wind.w);
        assert!(temp > 290.0 && temp < 296.0, "T = {}", temp);
    }

    #[test]
    fn test_processor_analyze() {
        let config = SonicConfig::new(0.15, 0.0, 3, 20.0);
        let mut proc = SonicAnemometerProcessor::new(config, 3.0);
        proc.add_excluded_sector(180.0, 15.0);

        let c = 343.0;
        let azimuths = [90.0, 0.0, 0.0];
        let elevations = [0.0, 0.0, 90.0];

        // Add 100 samples with some variability
        for i in 0..100 {
            let t = i as f64 / 100.0;
            let u_wind = 5.0 + 0.5 * (2.0 * PI * t).sin();
            let v_wind = 1.0 + 0.3 * (3.0 * PI * t).cos();
            let w_wind = 0.1 * (4.0 * PI * t).sin();

            let winds = [u_wind, v_wind, w_wind];
            let mut tt = Vec::new();
            for &w in &winds {
                let t_fwd = 0.15 / (c + w);
                let t_rev = 0.15 / (c - w);
                tt.push((t_fwd, t_rev));
            }
            proc.process_transit_times(&tt, &azimuths, &elevations);
        }

        let result = proc.analyze();
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.num_samples, 100);
        assert!(r.turbulence_stats.u_mean > 4.0);
        assert!(r.turbulence_stats.tke > 0.0);
    }

    #[test]
    fn test_processor_clear_buffers() {
        let config = SonicConfig::new(0.15, 0.0, 3, 20.0);
        let mut proc = SonicAnemometerProcessor::new(config, 3.0);

        // Add some data
        let c = 343.0;
        let tt = vec![
            (0.15 / (c + 1.0), 0.15 / (c - 1.0)),
            (0.15 / (c + 0.5), 0.15 / (c - 0.5)),
            (0.15 / (c + 0.1), 0.15 / (c - 0.1)),
        ];
        proc.process_transit_times(&tt, &[90.0, 0.0, 0.0], &[0.0, 0.0, 90.0]);

        assert_eq!(proc.wind_buffer().len(), 1);
        proc.clear_buffers();
        assert_eq!(proc.wind_buffer().len(), 0);
        assert_eq!(proc.temp_buffer().len(), 0);
    }

    #[test]
    fn test_det_3x3() {
        let m = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        assert!(approx_eq(det_3x3(&m), 1.0, TOL));
    }

    #[test]
    fn test_solve_3x3_simple() {
        // Identity * x = b → x = b
        let a = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let b = [3.0, 4.0, 5.0];
        let x = solve_3x3(&a, &b);
        assert!(approx_eq(x[0], 3.0, TOL));
        assert!(approx_eq(x[1], 4.0, TOL));
        assert!(approx_eq(x[2], 5.0, TOL));
    }
}
