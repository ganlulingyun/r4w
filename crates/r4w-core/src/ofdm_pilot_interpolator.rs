//! OFDM pilot extraction and channel estimation interpolation across data subcarriers.
//!
//! This module provides configurable pilot patterns (comb, block, scattered),
//! least-squares channel estimation at pilot positions, and multiple interpolation
//! strategies (linear, cubic spline, 2D time-frequency) for estimating the channel
//! response at data subcarrier positions. It also supports pilot-aided phase tracking
//! for residual CFO correction and channel estimation MSE computation.
//!
//! # Example
//!
//! ```
//! use r4w_core::ofdm_pilot_interpolator::{OfdmPilotInterpolator, PilotPattern};
//!
//! // 64-subcarrier OFDM with comb pilots every 4th subcarrier
//! let interp = OfdmPilotInterpolator::new(64, PilotPattern::Comb { spacing: 4 });
//!
//! // Known pilot values (all ones) and a received OFDM symbol
//! let pilot_tx: Vec<(f64, f64)> = vec![(1.0, 0.0); 64];
//! let received: Vec<(f64, f64)> = (0..64)
//!     .map(|k| {
//!         let phase = 0.3 * k as f64;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Estimate channel at all subcarriers via linear interpolation
//! let h_est = interp.estimate_channel_linear(&received, &pilot_tx);
//! assert_eq!(h_est.len(), 64);
//! ```



// ---------------------------------------------------------------------------
// Complex helper functions using (f64, f64) tuples
// ---------------------------------------------------------------------------

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

#[inline]
fn c_abs2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    c_abs2(a).sqrt()
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// Pilot pattern types
// ---------------------------------------------------------------------------

/// Describes how pilots are arranged in the OFDM time-frequency grid.
#[derive(Debug, Clone, PartialEq)]
pub enum PilotPattern {
    /// Comb pattern: pilots placed at every `spacing`-th subcarrier in every symbol.
    /// Subcarrier indices: 0, spacing, 2*spacing, ...
    Comb { spacing: usize },

    /// Block pattern: all subcarriers carry pilots every `symbol_period`-th symbol.
    Block { symbol_period: usize },

    /// Scattered pattern: pilots shift by `freq_offset` subcarriers each symbol,
    /// with `freq_spacing` subcarriers between pilots in a given symbol and
    /// `time_spacing` symbols between repetitions of the same pilot subcarrier.
    Scattered {
        freq_spacing: usize,
        time_spacing: usize,
        freq_offset: usize,
    },
}

// ---------------------------------------------------------------------------
// Main struct
// ---------------------------------------------------------------------------

/// OFDM pilot-based channel estimator and interpolator.
///
/// Extracts pilot subcarriers from received OFDM symbols, performs least-squares
/// channel estimation at those positions, and interpolates to obtain channel
/// estimates at all data subcarrier positions.
#[derive(Debug, Clone)]
pub struct OfdmPilotInterpolator {
    /// Total number of subcarriers (FFT size).
    num_subcarriers: usize,
    /// Pilot placement pattern.
    pattern: PilotPattern,
    /// Linear power boost factor applied to pilot subcarriers (>= 1.0).
    pilot_boost: f64,
}

impl OfdmPilotInterpolator {
    /// Create a new interpolator.
    ///
    /// # Panics
    /// Panics if `num_subcarriers` is 0 or if the pilot pattern parameters are 0.
    pub fn new(num_subcarriers: usize, pattern: PilotPattern) -> Self {
        assert!(num_subcarriers > 0, "num_subcarriers must be > 0");
        match &pattern {
            PilotPattern::Comb { spacing } => {
                assert!(*spacing > 0, "comb spacing must be > 0");
            }
            PilotPattern::Block { symbol_period } => {
                assert!(*symbol_period > 0, "block symbol_period must be > 0");
            }
            PilotPattern::Scattered {
                freq_spacing,
                time_spacing,
                ..
            } => {
                assert!(*freq_spacing > 0, "freq_spacing must be > 0");
                assert!(*time_spacing > 0, "time_spacing must be > 0");
            }
        }
        Self {
            num_subcarriers,
            pattern,
            pilot_boost: 1.0,
        }
    }

    /// Set the pilot boost factor (linear amplitude multiplier, >= 1.0).
    pub fn set_pilot_boost(&mut self, boost: f64) {
        assert!(boost >= 1.0, "pilot boost must be >= 1.0");
        self.pilot_boost = boost;
    }

    /// Return the pilot boost factor.
    pub fn pilot_boost(&self) -> f64 {
        self.pilot_boost
    }

    /// Return the number of subcarriers.
    pub fn num_subcarriers(&self) -> usize {
        self.num_subcarriers
    }

    /// Return a reference to the pilot pattern.
    pub fn pattern(&self) -> &PilotPattern {
        &self.pattern
    }

    // -------------------------------------------------------------------
    // Pilot index queries
    // -------------------------------------------------------------------

    /// Return the sorted subcarrier indices that carry pilots for the given
    /// OFDM symbol index.
    pub fn pilot_indices(&self, symbol_index: usize) -> Vec<usize> {
        match &self.pattern {
            PilotPattern::Comb { spacing } => {
                (0..self.num_subcarriers).step_by(*spacing).collect()
            }
            PilotPattern::Block { symbol_period } => {
                if symbol_index % *symbol_period == 0 {
                    (0..self.num_subcarriers).collect()
                } else {
                    Vec::new()
                }
            }
            PilotPattern::Scattered {
                freq_spacing,
                time_spacing,
                freq_offset,
            } => {
                let offset = (symbol_index % *time_spacing) * *freq_offset;
                let start = offset % *freq_spacing;
                let mut indices = Vec::new();
                let mut k = start;
                while k < self.num_subcarriers {
                    indices.push(k);
                    k += *freq_spacing;
                }
                indices.sort_unstable();
                indices
            }
        }
    }

    /// Return the sorted subcarrier indices that carry data (non-pilot) for
    /// the given OFDM symbol index.
    pub fn data_indices(&self, symbol_index: usize) -> Vec<usize> {
        let pilots: std::collections::HashSet<usize> =
            self.pilot_indices(symbol_index).into_iter().collect();
        (0..self.num_subcarriers)
            .filter(|k| !pilots.contains(k))
            .collect()
    }

    // -------------------------------------------------------------------
    // Least-squares channel estimation at pilot positions
    // -------------------------------------------------------------------

    /// Least-squares channel estimate at pilot subcarrier positions.
    ///
    /// Returns `Vec<(usize, (f64, f64))>` -- pairs of (subcarrier_index, H_est).
    ///
    /// `received` is the full received symbol (length `num_subcarriers`).
    /// `pilot_tx` contains the known transmitted pilot values (same length).
    /// Only the entries at pilot positions are used from `pilot_tx`.
    ///
    /// If `pilot_boost` > 1 the transmitted pilots are assumed to have been
    /// boosted, so the estimate divides out the boost.
    pub fn ls_estimate(
        &self,
        received: &[(f64, f64)],
        pilot_tx: &[(f64, f64)],
        symbol_index: usize,
    ) -> Vec<(usize, (f64, f64))> {
        assert_eq!(received.len(), self.num_subcarriers);
        assert!(pilot_tx.len() >= self.num_subcarriers);
        let indices = self.pilot_indices(symbol_index);
        indices
            .iter()
            .map(|&k| {
                let tx = c_scale(pilot_tx[k], self.pilot_boost);
                let h = c_div(received[k], tx);
                (k, h)
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // Linear interpolation in frequency domain
    // -------------------------------------------------------------------

    /// Estimate the channel at all subcarriers using least-squares at pilot
    /// positions followed by linear interpolation in the frequency domain.
    ///
    /// Assumes `symbol_index = 0` (comb pattern typical usage).
    pub fn estimate_channel_linear(
        &self,
        received: &[(f64, f64)],
        pilot_tx: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        self.estimate_channel_linear_sym(received, pilot_tx, 0)
    }

    /// Same as [`estimate_channel_linear`] but for a specific symbol index.
    pub fn estimate_channel_linear_sym(
        &self,
        received: &[(f64, f64)],
        pilot_tx: &[(f64, f64)],
        symbol_index: usize,
    ) -> Vec<(f64, f64)> {
        let pilots = self.ls_estimate(received, pilot_tx, symbol_index);
        if pilots.is_empty() {
            return vec![(0.0, 0.0); self.num_subcarriers];
        }
        Self::linear_interp_complex(&pilots, self.num_subcarriers)
    }

    /// Linearly interpolate complex values given at sparse positions to fill
    /// `n` output positions (0..n-1).
    fn linear_interp_complex(points: &[(usize, (f64, f64))], n: usize) -> Vec<(f64, f64)> {
        let mut out = vec![(0.0, 0.0); n];
        if points.is_empty() {
            return out;
        }
        if points.len() == 1 {
            let val = points[0].1;
            for v in out.iter_mut() {
                *v = val;
            }
            return out;
        }
        for k in 0..n {
            if k <= points[0].0 {
                out[k] = points[0].1;
            } else if k >= points[points.len() - 1].0 {
                out[k] = points[points.len() - 1].1;
            } else {
                let mut lo = 0;
                let mut hi = points.len() - 1;
                while hi - lo > 1 {
                    let mid = (lo + hi) / 2;
                    if points[mid].0 <= k {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let x0 = points[lo].0 as f64;
                let x1 = points[hi].0 as f64;
                let t = (k as f64 - x0) / (x1 - x0);
                let v0 = points[lo].1;
                let v1 = points[hi].1;
                out[k] = (v0.0 + t * (v1.0 - v0.0), v0.1 + t * (v1.1 - v0.1));
            }
        }
        out
    }

    // -------------------------------------------------------------------
    // Cubic spline interpolation in frequency domain
    // -------------------------------------------------------------------

    /// Estimate the channel using cubic spline interpolation at pilot positions.
    pub fn estimate_channel_cubic(
        &self,
        received: &[(f64, f64)],
        pilot_tx: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        self.estimate_channel_cubic_sym(received, pilot_tx, 0)
    }

    /// Cubic spline estimation for a specific symbol index.
    pub fn estimate_channel_cubic_sym(
        &self,
        received: &[(f64, f64)],
        pilot_tx: &[(f64, f64)],
        symbol_index: usize,
    ) -> Vec<(f64, f64)> {
        let pilots = self.ls_estimate(received, pilot_tx, symbol_index);
        if pilots.is_empty() {
            return vec![(0.0, 0.0); self.num_subcarriers];
        }
        if pilots.len() < 3 {
            return Self::linear_interp_complex(&pilots, self.num_subcarriers);
        }
        Self::cubic_spline_interp_complex(&pilots, self.num_subcarriers)
    }

    /// Natural cubic spline interpolation of complex values.
    fn cubic_spline_interp_complex(
        points: &[(usize, (f64, f64))],
        n: usize,
    ) -> Vec<(f64, f64)> {
        let xs: Vec<f64> = points.iter().map(|(k, _)| *k as f64).collect();
        let yr: Vec<f64> = points.iter().map(|(_, v)| v.0).collect();
        let yi: Vec<f64> = points.iter().map(|(_, v)| v.1).collect();

        let sr = Self::natural_cubic_spline_coeffs(&xs, &yr);
        let si = Self::natural_cubic_spline_coeffs(&xs, &yi);

        let mut out = vec![(0.0, 0.0); n];
        for k in 0..n {
            let x = k as f64;
            let re = Self::eval_cubic_spline(&xs, &yr, &sr, x);
            let im = Self::eval_cubic_spline(&xs, &yi, &si, x);
            out[k] = (re, im);
        }
        out
    }

    /// Compute second derivatives for natural cubic spline (S'' at knots).
    fn natural_cubic_spline_coeffs(xs: &[f64], ys: &[f64]) -> Vec<f64> {
        let n = xs.len();
        if n < 2 {
            return vec![0.0; n];
        }
        let mut h = vec![0.0; n - 1];
        for i in 0..n - 1 {
            h[i] = xs[i + 1] - xs[i];
        }
        let m = n - 2;
        if m == 0 {
            return vec![0.0; n];
        }
        let mut a_vec = vec![0.0; m];
        let mut b_vec = vec![0.0; m];
        let mut c_vec = vec![0.0; m];
        let mut d_vec = vec![0.0; m];

        for i in 0..m {
            let ii = i + 1;
            if i > 0 {
                a_vec[i] = h[ii - 1];
            }
            b_vec[i] = 2.0 * (h[ii - 1] + h[ii]);
            if i + 1 < m {
                c_vec[i] = h[ii];
            }
            d_vec[i] = 6.0
                * ((ys[ii + 1] - ys[ii]) / h[ii] - (ys[ii] - ys[ii - 1]) / h[ii - 1]);
        }

        // Thomas algorithm
        let mut cp = vec![0.0; m];
        let mut dp = vec![0.0; m];
        cp[0] = c_vec[0] / b_vec[0];
        dp[0] = d_vec[0] / b_vec[0];
        for i in 1..m {
            let w = b_vec[i] - a_vec[i] * cp[i - 1];
            if i < m - 1 {
                cp[i] = c_vec[i] / w;
            }
            dp[i] = (d_vec[i] - a_vec[i] * dp[i - 1]) / w;
        }
        let mut sol = vec![0.0; m];
        sol[m - 1] = dp[m - 1];
        for i in (0..m - 1).rev() {
            sol[i] = dp[i] - cp[i] * sol[i + 1];
        }

        let mut s = vec![0.0; n];
        for i in 0..m {
            s[i + 1] = sol[i];
        }
        s
    }

    /// Evaluate a natural cubic spline at point `x`.
    fn eval_cubic_spline(xs: &[f64], ys: &[f64], s: &[f64], x: f64) -> f64 {
        let n = xs.len();
        if x <= xs[0] {
            return ys[0];
        }
        if x >= xs[n - 1] {
            return ys[n - 1];
        }
        let mut lo = 0;
        let mut hi = n - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if xs[mid] <= x {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let h = xs[hi] - xs[lo];
        let a = (xs[hi] - x) / h;
        let b = (x - xs[lo]) / h;
        a * ys[lo]
            + b * ys[hi]
            + (a * (a * a - 1.0) * s[lo] + b * (b * b - 1.0) * s[hi]) * h * h / 6.0
    }

    // -------------------------------------------------------------------
    // 2D interpolation (time x frequency) for scattered pilots
    // -------------------------------------------------------------------

    /// Perform 2D channel estimation across multiple OFDM symbols.
    ///
    /// `symbols` is a slice of received OFDM symbols (each of length `num_subcarriers`).
    /// `pilot_tx` is the known pilot reference for each symbol (same outer length).
    ///
    /// Returns a 2D grid `[symbol][subcarrier]` of channel estimates using
    /// bilinear interpolation across the time-frequency pilot grid.
    pub fn estimate_channel_2d(
        &self,
        symbols: &[Vec<(f64, f64)>],
        pilot_tx: &[Vec<(f64, f64)>],
    ) -> Vec<Vec<(f64, f64)>> {
        let num_symbols = symbols.len();
        assert_eq!(pilot_tx.len(), num_symbols);

        // Gather all pilot observations: (sym_idx, sc_idx, H_est).
        let mut observations: Vec<(usize, usize, (f64, f64))> = Vec::new();
        for (sym, (rx, tx)) in symbols.iter().zip(pilot_tx.iter()).enumerate() {
            let est = self.ls_estimate(rx, tx, sym);
            for (k, h) in est {
                observations.push((sym, k, h));
            }
        }

        // Step 1: per-symbol frequency interpolation from that symbol's pilots.
        let mut per_symbol: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_symbols);
        for sym in 0..num_symbols {
            let pilots: Vec<(usize, (f64, f64))> = observations
                .iter()
                .filter(|(s, _, _)| *s == sym)
                .map(|(_, k, h)| (*k, *h))
                .collect();
            if pilots.is_empty() {
                per_symbol.push(vec![(0.0, 0.0); self.num_subcarriers]);
            } else {
                per_symbol.push(Self::linear_interp_complex(&pilots, self.num_subcarriers));
            }
        }

        // Step 2: for each subcarrier, linearly interpolate along the time axis.
        let symbols_with_pilots: Vec<usize> = (0..num_symbols)
            .filter(|s| !self.pilot_indices(*s).is_empty())
            .collect();

        let mut grid = vec![vec![(0.0, 0.0); self.num_subcarriers]; num_symbols];

        for k in 0..self.num_subcarriers {
            if symbols_with_pilots.is_empty() {
                continue;
            }
            let time_points: Vec<(usize, (f64, f64))> = symbols_with_pilots
                .iter()
                .map(|&s| (s, per_symbol[s][k]))
                .collect();
            let time_interp = Self::linear_interp_complex(&time_points, num_symbols);
            for s in 0..num_symbols {
                grid[s][k] = time_interp[s];
            }
        }

        grid
    }

    // -------------------------------------------------------------------
    // Pilot-aided phase tracking (residual CFO correction)
    // -------------------------------------------------------------------

    /// Estimate the residual phase offset of each OFDM symbol by averaging
    /// the phase of (received_pilot / known_pilot) across all pilot subcarriers.
    ///
    /// Returns one phase (radians) per symbol.
    pub fn estimate_residual_phase(
        &self,
        symbols: &[Vec<(f64, f64)>],
        pilot_tx: &[Vec<(f64, f64)>],
    ) -> Vec<f64> {
        symbols
            .iter()
            .zip(pilot_tx.iter())
            .enumerate()
            .map(|(sym, (rx, tx))| {
                let indices = self.pilot_indices(sym);
                if indices.is_empty() {
                    return 0.0;
                }
                let mut sum_re = 0.0;
                let mut sum_im = 0.0;
                for &k in &indices {
                    let p = c_div(rx[k], c_scale(tx[k], self.pilot_boost));
                    sum_re += p.0;
                    sum_im += p.1;
                }
                sum_im.atan2(sum_re)
            })
            .collect()
    }

    /// Correct residual phase on each symbol in-place. Multiplies every
    /// subcarrier by `exp(-j * phase)`.
    pub fn correct_residual_phase(symbols: &mut [Vec<(f64, f64)>], phases: &[f64]) {
        assert_eq!(symbols.len(), phases.len());
        for (sym, &phi) in symbols.iter_mut().zip(phases.iter()) {
            let corr = ((-phi).cos(), (-phi).sin());
            for s in sym.iter_mut() {
                *s = c_mul(*s, corr);
            }
        }
    }

    // -------------------------------------------------------------------
    // Channel estimation MSE
    // -------------------------------------------------------------------

    /// Compute the mean squared error between an estimated channel response
    /// and a known true channel response.
    pub fn channel_estimation_mse(h_est: &[(f64, f64)], h_true: &[(f64, f64)]) -> f64 {
        assert_eq!(h_est.len(), h_true.len());
        let n = h_est.len() as f64;
        h_est
            .iter()
            .zip(h_true.iter())
            .map(|(&e, &t)| c_abs2(c_sub(e, t)))
            .sum::<f64>()
            / n
    }

    // -------------------------------------------------------------------
    // Pilot boosting helper
    // -------------------------------------------------------------------

    /// Apply pilot boosting to a transmit symbol in-place. Multiplies pilot
    /// subcarriers by `pilot_boost`.
    pub fn apply_pilot_boost(&self, symbol: &mut [(f64, f64)], symbol_index: usize) {
        let indices = self.pilot_indices(symbol_index);
        for k in indices {
            symbol[k] = c_scale(symbol[k], self.pilot_boost);
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: build a channel where H(k) = exp(j * slope * k).
    fn make_phase_channel(n: usize, slope: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let phi = slope * k as f64;
                (phi.cos(), phi.sin())
            })
            .collect()
    }

    /// Helper: multiply element-wise.
    fn apply_channel(tx: &[(f64, f64)], h: &[(f64, f64)]) -> Vec<(f64, f64)> {
        tx.iter()
            .zip(h.iter())
            .map(|(&a, &b)| c_mul(a, b))
            .collect()
    }

    // --- Construction and basic queries ---

    #[test]
    fn test_new_comb() {
        let ip = OfdmPilotInterpolator::new(64, PilotPattern::Comb { spacing: 4 });
        assert_eq!(ip.num_subcarriers(), 64);
        let idx = ip.pilot_indices(0);
        assert_eq!(
            idx,
            vec![0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60]
        );
    }

    #[test]
    fn test_new_block() {
        let ip = OfdmPilotInterpolator::new(16, PilotPattern::Block { symbol_period: 3 });
        assert_eq!(ip.pilot_indices(0).len(), 16);
        assert!(ip.pilot_indices(1).is_empty());
        assert!(ip.pilot_indices(2).is_empty());
        assert_eq!(ip.pilot_indices(3).len(), 16);
    }

    #[test]
    fn test_scattered_pattern() {
        let ip = OfdmPilotInterpolator::new(
            16,
            PilotPattern::Scattered {
                freq_spacing: 4,
                time_spacing: 4,
                freq_offset: 1,
            },
        );
        let idx0 = ip.pilot_indices(0);
        let idx1 = ip.pilot_indices(1);
        assert_eq!(idx0, vec![0, 4, 8, 12]);
        assert_eq!(idx1, vec![1, 5, 9, 13]);
    }

    #[test]
    fn test_data_indices_complement() {
        let ip = OfdmPilotInterpolator::new(16, PilotPattern::Comb { spacing: 4 });
        let pilots = ip.pilot_indices(0);
        let data = ip.data_indices(0);
        assert_eq!(pilots.len() + data.len(), 16);
        for d in &data {
            assert!(!pilots.contains(d));
        }
    }

    #[test]
    #[should_panic]
    fn test_zero_subcarriers_panics() {
        OfdmPilotInterpolator::new(0, PilotPattern::Comb { spacing: 4 });
    }

    #[test]
    #[should_panic]
    fn test_zero_spacing_panics() {
        OfdmPilotInterpolator::new(64, PilotPattern::Comb { spacing: 0 });
    }

    // --- Least-squares estimation ---

    #[test]
    fn test_ls_estimate_identity_channel() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = tx.clone();
        let est = ip.ls_estimate(&rx, &tx, 0);
        for (_, h) in &est {
            assert!((h.0 - 1.0).abs() < 1e-12);
            assert!(h.1.abs() < 1e-12);
        }
    }

    #[test]
    fn test_ls_estimate_known_channel() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let h_true = make_phase_channel(n, 0.1);
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let est = ip.ls_estimate(&rx, &tx, 0);
        for &(k, h) in &est {
            let err = c_abs(c_sub(h, h_true[k]));
            assert!(err < 1e-12, "LS error at k={}: {}", k, err);
        }
    }

    // --- Linear interpolation ---

    #[test]
    fn test_linear_interp_identity() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let h_true: Vec<(f64, f64)> = (0..n).map(|k| (1.0 + k as f64 * 0.01, 0.0)).collect();
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let h_est = ip.estimate_channel_linear(&rx, &tx);
        assert_eq!(h_est.len(), n);
        for &k in &ip.pilot_indices(0) {
            let err = c_abs(c_sub(h_est[k], h_true[k]));
            assert!(err < 1e-12, "Linear exact at pilot k={}: err={}", k, err);
        }
    }

    #[test]
    fn test_linear_interp_linear_channel() {
        // A linearly varying (real) channel should be perfectly recovered
        // between the first and last pilot positions.
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let h_true: Vec<(f64, f64)> = (0..n).map(|k| (1.0 + 0.02 * k as f64, 0.0)).collect();
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let h_est = ip.estimate_channel_linear(&rx, &tx);
        let pilots = ip.pilot_indices(0);
        let first = pilots[0];
        let last = *pilots.last().unwrap();
        // Within interpolation range: exact for a linear channel.
        for k in first..=last {
            let err = (h_est[k].0 - h_true[k].0).abs();
            assert!(err < 1e-12, "Linear channel exact at k={}: err={}", k, err);
        }
        // Beyond last pilot: flat extrapolation equals last pilot value.
        for k in (last + 1)..n {
            assert!(
                (h_est[k].0 - h_true[last].0).abs() < 1e-12,
                "Flat extrapolation at k={}",
                k,
            );
        }
    }

    // --- Cubic spline interpolation ---

    #[test]
    fn test_cubic_matches_linear_channel() {
        let n = 32;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let h_true: Vec<(f64, f64)> = (0..n).map(|k| (2.0 + 0.05 * k as f64, 0.0)).collect();
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let h_lin = ip.estimate_channel_linear(&rx, &tx);
        let h_cub = ip.estimate_channel_cubic(&rx, &tx);
        for k in 0..n {
            let diff = (h_lin[k].0 - h_cub[k].0).abs();
            assert!(diff < 1e-6, "Cubic vs linear at k={}: diff={}", k, diff);
        }
    }

    #[test]
    fn test_cubic_smoother_than_linear() {
        let n = 64;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 8 });
        let h_true: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let x = k as f64 / n as f64;
                ((2.0 * PI * x).cos(), (2.0 * PI * x).sin() * 0.3)
            })
            .collect();
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let mse_lin = OfdmPilotInterpolator::channel_estimation_mse(
            &ip.estimate_channel_linear(&rx, &tx),
            &h_true,
        );
        let mse_cub = OfdmPilotInterpolator::channel_estimation_mse(
            &ip.estimate_channel_cubic(&rx, &tx),
            &h_true,
        );
        assert!(
            mse_cub <= mse_lin + 1e-6,
            "Cubic MSE {} should be <= linear MSE {}",
            mse_cub,
            mse_lin,
        );
    }

    // --- 2D interpolation ---

    #[test]
    fn test_2d_block_constant_channel() {
        let n = 8;
        let num_syms = 6;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Block { symbol_period: 3 });
        let h_true = vec![(0.8, 0.2); n];
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx = apply_channel(&tx, &h_true);
        let symbols: Vec<Vec<(f64, f64)>> = vec![rx.clone(); num_syms];
        let pilot_txs: Vec<Vec<(f64, f64)>> = vec![tx.clone(); num_syms];
        let grid = ip.estimate_channel_2d(&symbols, &pilot_txs);
        assert_eq!(grid.len(), num_syms);
        for s in 0..num_syms {
            for k in 0..n {
                let err = c_abs(c_sub(grid[s][k], h_true[k]));
                assert!(err < 1e-10, "2D err at sym={} k={}: {}", s, k, err);
            }
        }
    }

    #[test]
    fn test_2d_scattered_returns_correct_size() {
        let n = 16;
        let num_syms = 8;
        let ip = OfdmPilotInterpolator::new(
            n,
            PilotPattern::Scattered {
                freq_spacing: 4,
                time_spacing: 4,
                freq_offset: 1,
            },
        );
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx: Vec<(f64, f64)> = vec![(0.5, 0.1); n];
        let symbols = vec![rx.clone(); num_syms];
        let pilot_txs = vec![tx.clone(); num_syms];
        let grid = ip.estimate_channel_2d(&symbols, &pilot_txs);
        assert_eq!(grid.len(), num_syms);
        for row in &grid {
            assert_eq!(row.len(), n);
        }
    }

    // --- Phase tracking ---

    #[test]
    fn test_residual_phase_zero_for_aligned() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let phases = ip.estimate_residual_phase(&[tx.clone()], &[tx.clone()]);
        assert!((phases[0]).abs() < 1e-12);
    }

    #[test]
    fn test_residual_phase_detects_offset() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let phi = 0.5_f64;
        let rot = (phi.cos(), phi.sin());
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx: Vec<(f64, f64)> = tx.iter().map(|&v| c_mul(v, rot)).collect();
        let phases = ip.estimate_residual_phase(&[rx], &[tx]);
        assert!(
            (phases[0] - phi).abs() < 1e-12,
            "Detected phase {} vs expected {}",
            phases[0],
            phi,
        );
    }

    #[test]
    fn test_correct_residual_phase() {
        let n = 16;
        let ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        let phi = 0.3_f64;
        let rot = (phi.cos(), phi.sin());
        let tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let rx: Vec<(f64, f64)> = tx.iter().map(|&v| c_mul(v, rot)).collect();
        let mut symbols = vec![rx];
        let phases = ip.estimate_residual_phase(&symbols, &[tx.clone()]);
        OfdmPilotInterpolator::correct_residual_phase(&mut symbols, &phases);
        for &s in &symbols[0] {
            assert!((s.0 - 1.0).abs() < 1e-12);
            assert!(s.1.abs() < 1e-12);
        }
    }

    // --- MSE metric ---

    #[test]
    fn test_mse_zero_for_identical() {
        let h = vec![(1.0, 0.5), (0.3, -0.2), (0.0, 1.0)];
        let mse = OfdmPilotInterpolator::channel_estimation_mse(&h, &h);
        assert!(mse < 1e-15);
    }

    #[test]
    fn test_mse_positive_for_different() {
        let h1 = vec![(1.0, 0.0); 4];
        let h2 = vec![(0.0, 0.0); 4];
        let mse = OfdmPilotInterpolator::channel_estimation_mse(&h1, &h2);
        assert!((mse - 1.0).abs() < 1e-12);
    }

    // --- Pilot boosting ---

    #[test]
    fn test_pilot_boost_applied() {
        let n = 16;
        let mut ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        ip.set_pilot_boost(2.0);
        let mut sym: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        ip.apply_pilot_boost(&mut sym, 0);
        for k in 0..n {
            if k % 4 == 0 {
                assert!((sym[k].0 - 2.0).abs() < 1e-12);
            } else {
                assert!((sym[k].0 - 1.0).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_ls_with_boost_recovers_channel() {
        let n = 16;
        let mut ip = OfdmPilotInterpolator::new(n, PilotPattern::Comb { spacing: 4 });
        ip.set_pilot_boost(3.0);
        let h_true: Vec<(f64, f64)> = vec![(0.7, 0.4); n];
        let mut tx: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        ip.apply_pilot_boost(&mut tx, 0);
        let rx = apply_channel(&tx, &h_true);
        let est = ip.ls_estimate(&rx, &vec![(1.0, 0.0); n], 0);
        for &(k, h) in &est {
            let err = c_abs(c_sub(h, h_true[k]));
            assert!(err < 1e-12, "Boosted LS error at k={}: {}", k, err);
        }
    }

    #[test]
    #[should_panic]
    fn test_pilot_boost_below_one_panics() {
        let mut ip = OfdmPilotInterpolator::new(16, PilotPattern::Comb { spacing: 4 });
        ip.set_pilot_boost(0.5);
    }
}
