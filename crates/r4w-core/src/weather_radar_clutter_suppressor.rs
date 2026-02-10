//! Weather radar ground clutter suppression module.
//!
//! This module removes ground clutter from weather radar returns using filtering
//! and polarimetric techniques. It provides:
//!
//! - **Ground clutter filtering** — IIR high-pass and polynomial regression filters
//!   that remove zero-velocity clutter from pulse trains.
//! - **Clutter-to-signal ratio estimation** — Per-range-gate CSR computed from the
//!   DC spectral component relative to total power.
//! - **Dual-polarization clutter identification** — Uses ZDR, rhoHV, and PhiDP
//!   thresholds to classify range gates as ground clutter.
//! - **Velocity dealiasing** — Corrects aliased Doppler velocities by folding into
//!   the correct Nyquist interval.
//! - **Reflectivity calibration** — Converts linear received power to dBZ using the
//!   standard radar equation.
//! - **Quality index generation** — Produces a 0.0–1.0 data quality score from
//!   polarimetric moments.
//!
//! All routines use only the Rust standard library (no external crates).
//!
//! # Example
//!
//! ```
//! use r4w_core::weather_radar_clutter_suppressor::{
//!     RadarConfig, ClutterSuppressor, RadarMoments,
//! };
//!
//! let config = RadarConfig {
//!     wavelength_m: 0.1,
//!     prf_hz: 1000.0,
//!     num_pulses: 64,
//!     nyquist_velocity_mps: 25.0,
//! };
//!
//! let suppressor = ClutterSuppressor::new(config);
//!
//! // Create a trivial pulse train (2 pulses, 4 range gates each)
//! let pulses = vec![
//!     vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0), (0.1, 0.2)],
//!     vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0), (0.1, 0.2)],
//! ];
//!
//! // IIR clutter filter with alpha = 0.9
//! let filtered = suppressor.iir_clutter_filter(&pulses, 0.9);
//! assert_eq!(filtered.len(), pulses.len());
//! assert_eq!(filtered[0].len(), pulses[0].len());
//!
//! // Compute moments for range gate 1
//! let moments = suppressor.compute_moments(&pulses, 1);
//! assert!(moments.reflectivity_dbz.is_finite());
//!
//! // Quality index is between 0 and 1
//! let qi = suppressor.quality_index(&moments);
//! assert!((0.0..=1.0).contains(&qi));
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public data types
// ---------------------------------------------------------------------------

/// Configuration parameters for the weather radar.
#[derive(Debug, Clone)]
pub struct RadarConfig {
    /// Radar wavelength in metres (e.g. 0.1 for S-band, 0.032 for X-band).
    pub wavelength_m: f64,
    /// Pulse repetition frequency in Hz.
    pub prf_hz: f64,
    /// Number of pulses per dwell / CPI.
    pub num_pulses: usize,
    /// Nyquist (maximum unambiguous) velocity in m/s.
    pub nyquist_velocity_mps: f64,
}

/// Radar signal moments estimated from a pulse train at a single range gate.
#[derive(Debug, Clone)]
pub struct RadarMoments {
    /// Equivalent reflectivity factor in dBZ.
    pub reflectivity_dbz: f64,
    /// Mean radial velocity in m/s.
    pub velocity_mps: f64,
    /// Spectrum width (velocity dispersion) in m/s.
    pub spectrum_width_mps: f64,
    /// Differential reflectivity in dB (dual-pol).
    pub zdr_db: f64,
    /// Co-polar correlation coefficient (dual-pol, 0–1).
    pub rho_hv: f64,
    /// Differential phase in degrees (dual-pol).
    pub phi_dp_deg: f64,
}

/// Ground clutter suppressor for weather radar data.
///
/// Wraps a [`RadarConfig`] and exposes all clutter-filtering, moment
/// estimation, and quality-assessment routines.
#[derive(Debug, Clone)]
pub struct ClutterSuppressor {
    /// The radar configuration used by this suppressor.
    pub config: RadarConfig,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl ClutterSuppressor {
    /// Create a new `ClutterSuppressor` with the given radar configuration.
    pub fn new(config: RadarConfig) -> Self {
        Self { config }
    }

    /// IIR high-pass clutter filter applied along the pulse (slow-time)
    /// dimension for every range gate.
    ///
    /// The filter is a first-order IIR notch at DC:
    ///
    /// ```text
    /// y[n] = x[n] - x[n-1] + alpha * y[n-1]
    /// ```
    ///
    /// where `alpha` (0 < alpha < 1) controls the notch width.  Values close
    /// to 1.0 give a very narrow notch (removes only strict zero-Doppler
    /// clutter) while smaller values remove a broader band around DC.
    ///
    /// # Arguments
    ///
    /// * `pulses` — Slice of pulse vectors. Each inner `Vec<(f64, f64)>` is one
    ///   PRI with `(I, Q)` samples per range gate.
    /// * `alpha` — IIR feedback coefficient in (0, 1).
    ///
    /// # Returns
    ///
    /// A new pulse train of the same shape with clutter removed.
    pub fn iir_clutter_filter(
        &self,
        pulses: &[Vec<(f64, f64)>],
        alpha: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        if pulses.is_empty() {
            return Vec::new();
        }
        let num_gates = pulses[0].len();
        let num_pulses = pulses.len();

        let mut output: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); num_gates]; num_pulses];

        for gate in 0..num_gates {
            // First pulse: y[0] = x[0] (no previous sample)
            output[0][gate] = pulses[0][gate];
            for n in 1..num_pulses {
                let (xi, xq) = pulses[n][gate];
                let (xi_prev, xq_prev) = pulses[n - 1][gate];
                let (yi_prev, yq_prev) = output[n - 1][gate];

                let yi = xi - xi_prev + alpha * yi_prev;
                let yq = xq - xq_prev + alpha * yq_prev;
                output[n][gate] = (yi, yq);
            }
        }
        output
    }

    /// Polynomial regression clutter filter.
    ///
    /// Fits a polynomial of the given `order` to the slow-time IQ data at each
    /// range gate and subtracts the fit, thereby removing low-order (clutter)
    /// components.
    ///
    /// Uses a least-squares Vandermonde approach.  The polynomial basis is
    /// evaluated at normalised time indices in \[-1, 1\].
    ///
    /// # Arguments
    ///
    /// * `pulses` — Pulse train (same layout as [`iir_clutter_filter`]).
    /// * `order` — Polynomial order (0 = remove mean, 1 = remove linear trend, …).
    ///
    /// # Returns
    ///
    /// Filtered pulse train with the polynomial fit subtracted.
    pub fn regression_clutter_filter(
        &self,
        pulses: &[Vec<(f64, f64)>],
        order: usize,
    ) -> Vec<Vec<(f64, f64)>> {
        if pulses.is_empty() {
            return Vec::new();
        }
        let num_pulses = pulses.len();
        let num_gates = pulses[0].len();
        let k = order + 1; // number of basis functions

        // Normalised time indices in [-1, 1]
        let t_vals: Vec<f64> = (0..num_pulses)
            .map(|i| {
                if num_pulses <= 1 {
                    0.0
                } else {
                    2.0 * (i as f64) / ((num_pulses - 1) as f64) - 1.0
                }
            })
            .collect();

        // Build Vandermonde matrix V (num_pulses x k)
        let mut v = vec![vec![0.0f64; k]; num_pulses];
        for (i, ti) in t_vals.iter().enumerate() {
            let mut power = 1.0f64;
            for j in 0..k {
                v[i][j] = power;
                power *= ti;
            }
        }

        // Compute V^T V (k x k)
        let mut vtv = vec![vec![0.0f64; k]; k];
        for r in 0..k {
            for c in 0..k {
                let mut s = 0.0;
                for i in 0..num_pulses {
                    s += v[i][r] * v[i][c];
                }
                vtv[r][c] = s;
            }
        }

        // Invert V^T V via Gauss-Jordan
        let vtv_inv = invert_matrix(&vtv);

        // Pre-compute vtv_inv * V^T  => (k x num_pulses) matrix M
        // Then P * x = V * M * x
        let mut m_mat = vec![vec![0.0f64; num_pulses]; k];
        for r in 0..k {
            for j in 0..num_pulses {
                let mut s = 0.0;
                for c in 0..k {
                    s += vtv_inv[r][c] * v[j][c];
                }
                m_mat[r][j] = s;
            }
        }

        let mut output: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); num_gates]; num_pulses];

        for gate in 0..num_gates {
            // Extract slow-time I and Q for this gate
            let xi: Vec<f64> = pulses.iter().map(|p| p[gate].0).collect();
            let xq: Vec<f64> = pulses.iter().map(|p| p[gate].1).collect();

            // Compute coefficients: c = M * x
            let mut ci = vec![0.0f64; k];
            let mut cq = vec![0.0f64; k];
            for r in 0..k {
                for j in 0..num_pulses {
                    ci[r] += m_mat[r][j] * xi[j];
                    cq[r] += m_mat[r][j] * xq[j];
                }
            }

            // Reconstruct fit and subtract
            for i in 0..num_pulses {
                let mut fi = 0.0;
                let mut fq = 0.0;
                for j in 0..k {
                    fi += ci[j] * v[i][j];
                    fq += cq[j] * v[i][j];
                }
                output[i][gate] = (xi[i] - fi, xq[i] - fq);
            }
        }

        output
    }

    /// Estimate the clutter-to-signal ratio (CSR) per range gate.
    ///
    /// CSR is computed as the ratio of the DC (zero-Doppler) spectral power
    /// to the total spectral power minus the DC component. The result is
    /// returned in linear scale for each range gate.
    ///
    /// # Arguments
    ///
    /// * `pulses` — Pulse train.
    ///
    /// # Returns
    ///
    /// A vector of CSR values (one per range gate).
    pub fn estimate_csr(&self, pulses: &[Vec<(f64, f64)>]) -> Vec<f64> {
        if pulses.is_empty() {
            return Vec::new();
        }
        let num_pulses = pulses.len();
        let num_gates = pulses[0].len();
        let mut csr = vec![0.0f64; num_gates];

        for gate in 0..num_gates {
            // DC component = mean of slow-time samples
            let mut sum_i = 0.0;
            let mut sum_q = 0.0;
            let mut total_power = 0.0;
            for p in pulses.iter() {
                let (i, q) = p[gate];
                sum_i += i;
                sum_q += q;
                total_power += i * i + q * q;
            }
            let dc_i = sum_i / num_pulses as f64;
            let dc_q = sum_q / num_pulses as f64;
            let dc_power = (dc_i * dc_i + dc_q * dc_q) * num_pulses as f64;
            let signal_power = total_power - dc_power;

            if signal_power > 1e-30 {
                csr[gate] = dc_power / signal_power;
            } else {
                csr[gate] = f64::INFINITY;
            }
        }
        csr
    }

    /// Identify whether a range gate contains ground clutter using
    /// dual-polarization thresholds.
    ///
    /// Ground clutter typically exhibits:
    /// - High differential reflectivity (|ZDR| > 3 dB)
    /// - Low co-polar correlation coefficient (rhoHV < 0.90)
    ///
    /// Returns `true` if the gate is classified as ground clutter.
    pub fn identify_clutter_polarimetric(&self, moments: &RadarMoments) -> bool {
        let high_zdr = moments.zdr_db.abs() > 3.0;
        let low_rho_hv = moments.rho_hv < 0.90;

        // Clutter if *either* polarimetric indicator is triggered
        high_zdr || low_rho_hv
    }

    /// Dealias a measured radial velocity.
    ///
    /// If the measured velocity differs from the reference by more than one
    /// Nyquist interval, it is folded back by the appropriate multiple of
    /// `2 * nyquist_mps`.
    ///
    /// # Arguments
    ///
    /// * `velocity_mps` — Aliased measured velocity.
    /// * `nyquist_mps` — Nyquist velocity (maximum unambiguous velocity).
    /// * `reference_mps` — Reference velocity from a neighbouring gate or
    ///   previous scan.
    ///
    /// # Returns
    ///
    /// Dealiased velocity.
    pub fn dealias_velocity(
        &self,
        velocity_mps: f64,
        nyquist_mps: f64,
        reference_mps: f64,
    ) -> f64 {
        dealias_velocity(velocity_mps, nyquist_mps, reference_mps)
    }

    /// Compute equivalent reflectivity factor in dBZ from linear power.
    ///
    /// Uses the standard radar equation form:
    ///
    /// ```text
    /// Z_dBZ = 10 * log10(power_linear) + 20 * log10(range_m) + radar_constant_db
    /// ```
    pub fn compute_reflectivity_dbz(
        &self,
        power_linear: f64,
        range_m: f64,
        radar_constant_db: f64,
    ) -> f64 {
        compute_reflectivity_dbz(power_linear, range_m, radar_constant_db)
    }

    /// Estimate radar moments from a pulse train at a given range gate using
    /// the pulse-pair (autocovariance) method.
    ///
    /// Returns a [`RadarMoments`] struct.  The dual-pol fields (`zdr_db`,
    /// `rho_hv`, `phi_dp_deg`) are set to nominal meteorological defaults
    /// because single-channel IQ data cannot provide polarimetric information.
    pub fn compute_moments(
        &self,
        pulses: &[Vec<(f64, f64)>],
        range_gate: usize,
    ) -> RadarMoments {
        compute_moments_inner(pulses, range_gate, &self.config)
    }

    /// Compute a data-quality index in \[0, 1\] from radar moments.
    ///
    /// The index combines several heuristics:
    /// - rhoHV close to 1 is good (meteorological targets have rhoHV > 0.95).
    /// - Narrow spectrum width is better than very broad.
    /// - Moderate ZDR (close to 0) is typical for rain.
    /// - PhiDP stability (small absolute values indicate less propagation noise).
    ///
    /// The individual sub-scores are averaged with equal weight and clamped to
    /// \[0, 1\].
    pub fn quality_index(&self, moments: &RadarMoments) -> f64 {
        quality_index(moments)
    }
}

// ---------------------------------------------------------------------------
// Free functions (also publicly exported for convenience)
// ---------------------------------------------------------------------------

/// IIR high-pass clutter filter (free function variant).
///
/// See [`ClutterSuppressor::iir_clutter_filter`] for details.
pub fn iir_clutter_filter(
    pulses: &[Vec<(f64, f64)>],
    alpha: f64,
) -> Vec<Vec<(f64, f64)>> {
    let dummy = ClutterSuppressor::new(RadarConfig {
        wavelength_m: 0.1,
        prf_hz: 1000.0,
        num_pulses: 0,
        nyquist_velocity_mps: 25.0,
    });
    dummy.iir_clutter_filter(pulses, alpha)
}

/// Polynomial regression clutter filter (free function variant).
///
/// See [`ClutterSuppressor::regression_clutter_filter`] for details.
pub fn regression_clutter_filter(
    pulses: &[Vec<(f64, f64)>],
    order: usize,
) -> Vec<Vec<(f64, f64)>> {
    let dummy = ClutterSuppressor::new(RadarConfig {
        wavelength_m: 0.1,
        prf_hz: 1000.0,
        num_pulses: 0,
        nyquist_velocity_mps: 25.0,
    });
    dummy.regression_clutter_filter(pulses, order)
}

/// Clutter-to-signal ratio estimation (free function variant).
///
/// See [`ClutterSuppressor::estimate_csr`] for details.
pub fn estimate_csr(pulses: &[Vec<(f64, f64)>]) -> Vec<f64> {
    let dummy = ClutterSuppressor::new(RadarConfig {
        wavelength_m: 0.1,
        prf_hz: 1000.0,
        num_pulses: 0,
        nyquist_velocity_mps: 25.0,
    });
    dummy.estimate_csr(pulses)
}

/// Dual-polarization clutter identification (free function variant).
///
/// See [`ClutterSuppressor::identify_clutter_polarimetric`] for details.
pub fn identify_clutter_polarimetric(moments: &RadarMoments) -> bool {
    let dummy = ClutterSuppressor::new(RadarConfig {
        wavelength_m: 0.1,
        prf_hz: 1000.0,
        num_pulses: 0,
        nyquist_velocity_mps: 25.0,
    });
    dummy.identify_clutter_polarimetric(moments)
}

/// Dealias a measured radial velocity (free function).
///
/// Folds `velocity_mps` by multiples of `2 * nyquist_mps` until it is within
/// one Nyquist interval of `reference_mps`.
pub fn dealias_velocity(velocity_mps: f64, nyquist_mps: f64, reference_mps: f64) -> f64 {
    let interval = 2.0 * nyquist_mps;
    if interval <= 0.0 {
        return velocity_mps;
    }
    let mut v = velocity_mps;
    while (v - reference_mps) > nyquist_mps {
        v -= interval;
    }
    while (v - reference_mps) < -nyquist_mps {
        v += interval;
    }
    v
}

/// Compute equivalent reflectivity in dBZ (free function).
///
/// ```text
/// Z_dBZ = 10 * log10(power_linear) + 20 * log10(range_m) + radar_constant_db
/// ```
pub fn compute_reflectivity_dbz(
    power_linear: f64,
    range_m: f64,
    radar_constant_db: f64,
) -> f64 {
    10.0 * power_linear.log10() + 20.0 * range_m.log10() + radar_constant_db
}

/// Pulse-pair moment estimator (free function).
///
/// See [`ClutterSuppressor::compute_moments`] for the method description.
pub fn compute_moments(
    pulses: &[Vec<(f64, f64)>],
    range_gate: usize,
) -> RadarMoments {
    let config = RadarConfig {
        wavelength_m: 0.1,
        prf_hz: 1000.0,
        num_pulses: pulses.len(),
        nyquist_velocity_mps: 25.0,
    };
    compute_moments_inner(pulses, range_gate, &config)
}

/// Data quality index in \[0, 1\] (free function).
///
/// See [`ClutterSuppressor::quality_index`] for details.
pub fn quality_index(moments: &RadarMoments) -> f64 {
    // Sub-score 1: rhoHV (ideal ~ 1.0 for weather)
    let rho_score = moments.rho_hv.clamp(0.0, 1.0);

    // Sub-score 2: spectrum width — narrower is better, normalised against
    // a reference maximum of 15 m/s.
    let sw_ref = 15.0f64;
    let sw_score = (1.0 - (moments.spectrum_width_mps.abs() / sw_ref)).clamp(0.0, 1.0);

    // Sub-score 3: ZDR closeness to 0 dB (rain ~ 0-3 dB)
    let zdr_score = (1.0 - (moments.zdr_db.abs() / 6.0)).clamp(0.0, 1.0);

    // Sub-score 4: PhiDP stability — smaller absolute value -> more stable
    let phi_score = (1.0 - (moments.phi_dp_deg.abs() / 180.0)).clamp(0.0, 1.0);

    let avg = (rho_score + sw_score + zdr_score + phi_score) / 4.0;
    avg.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Internal pulse-pair moment computation.
fn compute_moments_inner(
    pulses: &[Vec<(f64, f64)>],
    range_gate: usize,
    config: &RadarConfig,
) -> RadarMoments {
    let n = pulses.len();
    if n == 0 {
        return RadarMoments {
            reflectivity_dbz: f64::NEG_INFINITY,
            velocity_mps: 0.0,
            spectrum_width_mps: 0.0,
            zdr_db: 0.0,
            rho_hv: 1.0,
            phi_dp_deg: 0.0,
        };
    }

    // R(0) — zero-lag autocorrelation (average power)
    let mut r0 = 0.0f64;
    for p in pulses.iter() {
        let (i, q) = p[range_gate];
        r0 += i * i + q * q;
    }
    r0 /= n as f64;

    // R(1) — lag-1 autocorrelation (complex)
    let mut r1_i = 0.0f64;
    let mut r1_q = 0.0f64;
    if n > 1 {
        for k in 0..(n - 1) {
            let (i0, q0) = pulses[k][range_gate];
            let (i1, q1) = pulses[k + 1][range_gate];
            // R(1) += s[k+1] * conj(s[k])
            r1_i += i1 * i0 + q1 * q0;
            r1_q += q1 * i0 - i1 * q0;
        }
        r1_i /= (n - 1) as f64;
        r1_q /= (n - 1) as f64;
    }

    // Mean velocity from arg(R(1))
    let phase = r1_q.atan2(r1_i);
    let prt = 1.0 / config.prf_hz;
    let velocity = -config.wavelength_m * phase / (4.0 * PI * prt);

    // Spectrum width from |R(1)|/R(0)
    let r1_mag = (r1_i * r1_i + r1_q * r1_q).sqrt();
    let ratio = if r0 > 1e-30 { r1_mag / r0 } else { 1.0 };
    let ratio_clamped = ratio.clamp(1e-6, 1.0);
    let spectrum_width = config.wavelength_m / (4.0 * PI * prt)
        * (-2.0 * ratio_clamped.ln()).sqrt();

    // Reflectivity: simple dBZ = 10*log10(R0)  (uncalibrated)
    let reflectivity_dbz = if r0 > 1e-30 {
        10.0 * r0.log10()
    } else {
        f64::NEG_INFINITY
    };

    RadarMoments {
        reflectivity_dbz,
        velocity_mps: velocity,
        spectrum_width_mps: spectrum_width,
        // Dual-pol defaults for single-channel data
        zdr_db: 0.0,
        rho_hv: 1.0,
        phi_dp_deg: 0.0,
    }
}

/// Gauss-Jordan inversion of a small square matrix.
///
/// Returns the inverse matrix.  Panics if the matrix is singular.
fn invert_matrix(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    // Augmented matrix [A | I]
    let mut aug = vec![vec![0.0f64; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }

    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        assert!(
            pivot.abs() > 1e-15,
            "Singular matrix in invert_matrix"
        );

        for j in 0..(2 * n) {
            aug[col][j] /= pivot;
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Extract inverse
    let mut inv = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    inv
}

// ===========================================================================
// Tests
// ===========================================================================
#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> RadarConfig {
        RadarConfig {
            wavelength_m: 0.1,
            prf_hz: 1000.0,
            num_pulses: 64,
            nyquist_velocity_mps: 25.0,
        }
    }

    fn make_suppressor() -> ClutterSuppressor {
        ClutterSuppressor::new(default_config())
    }

    /// Helper: generate a constant-clutter pulse train (all identical pulses).
    fn constant_clutter(num_pulses: usize, num_gates: usize) -> Vec<Vec<(f64, f64)>> {
        vec![vec![(1.0, 0.0); num_gates]; num_pulses]
    }

    /// Helper: generate a Doppler-shifted pulse train for one gate at the given
    /// normalised Doppler frequency.
    fn doppler_signal(
        num_pulses: usize,
        num_gates: usize,
        doppler_gate: usize,
        freq_norm: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let mut pulses = vec![vec![(0.0, 0.0); num_gates]; num_pulses];
        for n in 0..num_pulses {
            let phase = 2.0 * PI * freq_norm * n as f64;
            pulses[n][doppler_gate] = (phase.cos(), phase.sin());
        }
        pulses
    }

    // -----------------------------------------------------------------------
    // 1. IIR clutter filter — removes DC
    // -----------------------------------------------------------------------
    #[test]
    fn test_iir_clutter_filter_removes_dc() {
        let s = make_suppressor();
        let pulses = constant_clutter(32, 4);
        let filtered = s.iir_clutter_filter(&pulses, 0.95);

        // After the filter settles, DC should be suppressed
        for gate in 0..4 {
            let (i, q) = filtered[31][gate];
            assert!(
                (i * i + q * q) < 0.1,
                "DC should be suppressed at gate {gate}: ({i}, {q})"
            );
        }
    }

    // -----------------------------------------------------------------------
    // 2. IIR clutter filter — preserves Doppler
    // -----------------------------------------------------------------------
    #[test]
    fn test_iir_clutter_filter_preserves_doppler() {
        let s = make_suppressor();
        let pulses = doppler_signal(64, 4, 2, 0.25);
        let filtered = s.iir_clutter_filter(&pulses, 0.95);

        // The Doppler signal at gate 2 should still have significant power.
        let mut power = 0.0;
        for n in 32..64 {
            let (i, q) = filtered[n][2];
            power += i * i + q * q;
        }
        assert!(power > 10.0, "Doppler signal should be preserved: {power}");
    }

    // -----------------------------------------------------------------------
    // 3. IIR filter — empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_iir_clutter_filter_empty() {
        let s = make_suppressor();
        let result = s.iir_clutter_filter(&[], 0.9);
        assert!(result.is_empty());
    }

    // -----------------------------------------------------------------------
    // 4. Regression filter — order 0 removes mean
    // -----------------------------------------------------------------------
    #[test]
    fn test_regression_filter_order_0_removes_mean() {
        let s = make_suppressor();
        let pulses = constant_clutter(16, 2);
        let filtered = s.regression_clutter_filter(&pulses, 0);

        for n in 0..16 {
            for g in 0..2 {
                let (i, q) = filtered[n][g];
                assert!(
                    i.abs() < 1e-10 && q.abs() < 1e-10,
                    "Order-0 should remove constant: ({i}, {q})"
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // 5. Regression filter — preserves higher-order signal
    // -----------------------------------------------------------------------
    #[test]
    fn test_regression_filter_preserves_doppler() {
        let s = make_suppressor();
        let pulses = doppler_signal(32, 2, 0, 0.3);
        let filtered = s.regression_clutter_filter(&pulses, 0);

        // Doppler signal should have residual power after mean removal
        let mut power = 0.0;
        for n in 0..32 {
            let (i, q) = filtered[n][0];
            power += i * i + q * q;
        }
        assert!(power > 5.0, "Doppler should survive order-0 regression: {power}");
    }

    // -----------------------------------------------------------------------
    // 6. Regression filter — empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_regression_filter_empty() {
        let s = make_suppressor();
        let result = s.regression_clutter_filter(&[], 1);
        assert!(result.is_empty());
    }

    // -----------------------------------------------------------------------
    // 7. CSR estimation — pure clutter has high CSR
    // -----------------------------------------------------------------------
    #[test]
    fn test_estimate_csr_pure_clutter() {
        let s = make_suppressor();
        let pulses = constant_clutter(32, 4);
        let csr = s.estimate_csr(&pulses);
        assert_eq!(csr.len(), 4);
        for &c in &csr {
            assert!(c.is_infinite() || c > 100.0, "Pure clutter CSR should be very large: {c}");
        }
    }

    // -----------------------------------------------------------------------
    // 8. CSR estimation — pure Doppler has low CSR
    // -----------------------------------------------------------------------
    #[test]
    fn test_estimate_csr_pure_doppler() {
        let s = make_suppressor();
        let pulses = doppler_signal(64, 2, 0, 0.25);
        let csr = s.estimate_csr(&pulses);
        assert!(
            csr[0] < 0.5,
            "Pure Doppler CSR should be small: {}",
            csr[0]
        );
    }

    // -----------------------------------------------------------------------
    // 9. CSR — empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_estimate_csr_empty() {
        let s = make_suppressor();
        let result = s.estimate_csr(&[]);
        assert!(result.is_empty());
    }

    // -----------------------------------------------------------------------
    // 10. Polarimetric clutter ID — clutter-like moments
    // -----------------------------------------------------------------------
    #[test]
    fn test_identify_clutter_polarimetric_clutter() {
        let s = make_suppressor();
        let moments = RadarMoments {
            reflectivity_dbz: 30.0,
            velocity_mps: 0.5,
            spectrum_width_mps: 1.0,
            zdr_db: 5.0,
            rho_hv: 0.7,
            phi_dp_deg: 10.0,
        };
        assert!(s.identify_clutter_polarimetric(&moments));
    }

    // -----------------------------------------------------------------------
    // 11. Polarimetric clutter ID — weather-like moments
    // -----------------------------------------------------------------------
    #[test]
    fn test_identify_clutter_polarimetric_weather() {
        let s = make_suppressor();
        let moments = RadarMoments {
            reflectivity_dbz: 40.0,
            velocity_mps: 8.0,
            spectrum_width_mps: 3.0,
            zdr_db: 1.5,
            rho_hv: 0.98,
            phi_dp_deg: 45.0,
        };
        assert!(!s.identify_clutter_polarimetric(&moments));
    }

    // -----------------------------------------------------------------------
    // 12. Polarimetric ID — boundary: high ZDR alone triggers clutter
    // -----------------------------------------------------------------------
    #[test]
    fn test_identify_clutter_high_zdr_only() {
        let s = make_suppressor();
        let moments = RadarMoments {
            reflectivity_dbz: 20.0,
            velocity_mps: 5.0,
            spectrum_width_mps: 2.0,
            zdr_db: 4.0,
            rho_hv: 0.99,
            phi_dp_deg: 5.0,
        };
        assert!(s.identify_clutter_polarimetric(&moments));
    }

    // -----------------------------------------------------------------------
    // 13. Polarimetric ID — boundary: low rhoHV alone triggers clutter
    // -----------------------------------------------------------------------
    #[test]
    fn test_identify_clutter_low_rho_only() {
        let s = make_suppressor();
        let moments = RadarMoments {
            reflectivity_dbz: 25.0,
            velocity_mps: 3.0,
            spectrum_width_mps: 1.0,
            zdr_db: 0.5,
            rho_hv: 0.80,
            phi_dp_deg: 15.0,
        };
        assert!(s.identify_clutter_polarimetric(&moments));
    }

    // -----------------------------------------------------------------------
    // 14. Velocity dealiasing — no alias needed
    // -----------------------------------------------------------------------
    #[test]
    fn test_dealias_velocity_no_alias() {
        let v = dealias_velocity(5.0, 25.0, 4.0);
        assert!((v - 5.0).abs() < 1e-10, "No aliasing needed: {v}");
    }

    // -----------------------------------------------------------------------
    // 15. Velocity dealiasing — positive fold
    // -----------------------------------------------------------------------
    #[test]
    fn test_dealias_velocity_positive_fold() {
        let v = dealias_velocity(40.0, 25.0, -10.0);
        assert!(
            (v - (-10.0)).abs() < 1e-10,
            "Expected -10, got {v}"
        );
    }

    // -----------------------------------------------------------------------
    // 16. Velocity dealiasing — negative fold
    // -----------------------------------------------------------------------
    #[test]
    fn test_dealias_velocity_negative_fold() {
        let v = dealias_velocity(-20.0, 25.0, 20.0);
        assert!(
            (v - 30.0).abs() < 1e-10,
            "Expected 30, got {v}"
        );
    }

    // -----------------------------------------------------------------------
    // 17. Reflectivity dBZ computation
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_reflectivity_dbz() {
        let dbz = compute_reflectivity_dbz(1e-10, 100_000.0, 50.0);
        assert!(
            (dbz - 50.0).abs() < 1e-6,
            "Expected 50 dBZ, got {dbz}"
        );
    }

    // -----------------------------------------------------------------------
    // 18. Moment estimation — zero-Doppler signal
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_moments_zero_doppler() {
        let s = make_suppressor();
        let pulses = constant_clutter(64, 4);
        let m = s.compute_moments(&pulses, 0);

        assert!(
            m.velocity_mps.abs() < 0.1,
            "Constant signal velocity should be ~0: {}",
            m.velocity_mps
        );
        assert!(
            (m.reflectivity_dbz - 0.0).abs() < 0.1,
            "Unit power -> 0 dBZ: {}",
            m.reflectivity_dbz
        );
    }

    // -----------------------------------------------------------------------
    // 19. Moment estimation — known Doppler
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_moments_known_doppler() {
        let config = RadarConfig {
            wavelength_m: 0.1,
            prf_hz: 1000.0,
            num_pulses: 64,
            nyquist_velocity_mps: 25.0,
        };
        let s = ClutterSuppressor::new(config);

        let target_v = 10.0;
        let prt = 1.0 / 1000.0;
        let phase_per_prt = -4.0 * PI * target_v * prt / 0.1;

        let num_gates = 2;
        let num_pulses = 64;
        let mut pulses = vec![vec![(0.0, 0.0); num_gates]; num_pulses];
        for n in 0..num_pulses {
            let phi = phase_per_prt * n as f64;
            pulses[n][0] = (phi.cos(), phi.sin());
        }

        let m = s.compute_moments(&pulses, 0);
        assert!(
            (m.velocity_mps - target_v).abs() < 0.5,
            "Expected ~{target_v} m/s, got {}",
            m.velocity_mps
        );
    }

    // -----------------------------------------------------------------------
    // 20. Quality index — perfect weather
    // -----------------------------------------------------------------------
    #[test]
    fn test_quality_index_perfect() {
        let m = RadarMoments {
            reflectivity_dbz: 35.0,
            velocity_mps: 5.0,
            spectrum_width_mps: 0.0,
            zdr_db: 0.0,
            rho_hv: 1.0,
            phi_dp_deg: 0.0,
        };
        let qi = quality_index(&m);
        assert!(
            (qi - 1.0).abs() < 1e-10,
            "Perfect moments -> QI=1.0: {qi}"
        );
    }

    // -----------------------------------------------------------------------
    // 21. Quality index — poor data
    // -----------------------------------------------------------------------
    #[test]
    fn test_quality_index_poor() {
        let m = RadarMoments {
            reflectivity_dbz: 10.0,
            velocity_mps: 0.0,
            spectrum_width_mps: 20.0,
            zdr_db: 10.0,
            rho_hv: 0.3,
            phi_dp_deg: 170.0,
        };
        let qi = quality_index(&m);
        assert!(
            qi < 0.3,
            "Poor moments should give low QI: {qi}"
        );
    }

    // -----------------------------------------------------------------------
    // 22. Quality index — range [0, 1]
    // -----------------------------------------------------------------------
    #[test]
    fn test_quality_index_bounds() {
        let test_cases = vec![
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (50.0, 30.0, 30.0, 20.0, 1.0, 180.0),
            (-10.0, -20.0, 0.5, -8.0, 0.5, -90.0),
        ];
        for (dbz, v, sw, zdr, rho, phi) in test_cases {
            let m = RadarMoments {
                reflectivity_dbz: dbz,
                velocity_mps: v,
                spectrum_width_mps: sw,
                zdr_db: zdr,
                rho_hv: rho,
                phi_dp_deg: phi,
            };
            let qi = quality_index(&m);
            assert!(
                (0.0..=1.0).contains(&qi),
                "QI out of bounds: {qi} for moments {:?}",
                m
            );
        }
    }

    // -----------------------------------------------------------------------
    // 23. Free-function variants produce same results
    // -----------------------------------------------------------------------
    #[test]
    fn test_free_function_iir_matches_method() {
        let s = make_suppressor();
        let pulses = doppler_signal(16, 3, 1, 0.15);
        let via_method = s.iir_clutter_filter(&pulses, 0.9);
        let via_free = iir_clutter_filter(&pulses, 0.9);
        assert_eq!(via_method.len(), via_free.len());
        for n in 0..via_method.len() {
            for g in 0..via_method[n].len() {
                assert!(
                    (via_method[n][g].0 - via_free[n][g].0).abs() < 1e-12,
                    "Mismatch at pulse {n} gate {g}"
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // 24. Regression filter order 1 removes linear trend
    // -----------------------------------------------------------------------
    #[test]
    fn test_regression_filter_order_1() {
        let s = make_suppressor();
        let num_pulses = 16;
        let num_gates = 1;
        let pulses: Vec<Vec<(f64, f64)>> = (0..num_pulses)
            .map(|n| vec![(n as f64, 0.0); num_gates])
            .collect();
        let filtered = s.regression_clutter_filter(&pulses, 1);
        for n in 0..num_pulses {
            let (i, q) = filtered[n][0];
            assert!(
                i.abs() < 1e-8 && q.abs() < 1e-8,
                "Order-1 should remove linear: ({i}, {q}) at pulse {n}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // 25. Dealias velocity — zero Nyquist edge case
    // -----------------------------------------------------------------------
    #[test]
    fn test_dealias_velocity_zero_nyquist() {
        let v = dealias_velocity(10.0, 0.0, 5.0);
        assert!((v - 10.0).abs() < 1e-10, "Zero Nyquist should return input: {v}");
    }

    // -----------------------------------------------------------------------
    // 26. Moments — empty pulse train
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_moments_empty() {
        let m = compute_moments(&[], 0);
        assert_eq!(m.reflectivity_dbz, f64::NEG_INFINITY);
        assert_eq!(m.velocity_mps, 0.0);
    }

    // -----------------------------------------------------------------------
    // 27. RadarConfig clone and debug
    // -----------------------------------------------------------------------
    #[test]
    fn test_radar_config_clone_debug() {
        let c = default_config();
        let c2 = c.clone();
        assert_eq!(c.wavelength_m, c2.wavelength_m);
        let dbg = format!("{:?}", c);
        assert!(dbg.contains("RadarConfig"));
    }

    // -----------------------------------------------------------------------
    // 28. ClutterSuppressor debug
    // -----------------------------------------------------------------------
    #[test]
    fn test_clutter_suppressor_debug() {
        let s = make_suppressor();
        let dbg = format!("{:?}", s);
        assert!(dbg.contains("ClutterSuppressor"));
    }
}
