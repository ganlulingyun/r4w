//! Real-time state estimation and synchronization for digital twin systems.
//!
//! This module provides state observers (Luenberger, extended state, disturbance),
//! anomaly detection (chi-squared, CUSUM), latency compensation, parameter drift
//! tracking, and health scoring for digital twins that mirror physical RF/signal
//! processing equipment.
//!
//! # Example
//!
//! ```
//! use r4w_core::digital_twin_state_observer::{
//!     TwinConfig, TwinState, StateObserver, compute_residual, residual_norm,
//! };
//!
//! let config = TwinConfig {
//!     state_dim: 2,
//!     output_dim: 1,
//!     sample_rate_hz: 1000.0,
//!     sync_interval_s: 0.01,
//! };
//!
//! let mut observer = StateObserver::new(config);
//! let measurement = vec![1.0];
//! let a_mat = vec![vec![1.0, 0.1], vec![0.0, 1.0]];
//! let c_mat = vec![vec![1.0, 0.0]];
//! let l_gain = vec![vec![0.5], vec![0.3]];
//!
//! let residual = observer.step_luenberger(&measurement, &a_mat, &c_mat, &l_gain);
//! assert_eq!(residual.len(), 1);
//!
//! let r = compute_residual(&[1.0, 2.0], &[1.1, 1.9]);
//! assert!((residual_norm(&r) - 0.1414).abs() < 0.01);
//! ```

/// Configuration for a digital twin observer.
#[derive(Debug, Clone)]
pub struct TwinConfig {
    /// Dimension of the state vector.
    pub state_dim: usize,
    /// Dimension of the output/measurement vector.
    pub output_dim: usize,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Synchronization interval in seconds.
    pub sync_interval_s: f64,
}

/// Snapshot of the twin's estimated state.
#[derive(Debug, Clone)]
pub struct TwinState {
    /// State estimate vector.
    pub state: Vec<f64>,
    /// Error covariance matrix (state_dim x state_dim).
    pub covariance: Vec<Vec<f64>>,
    /// Timestamp in seconds since epoch.
    pub timestamp_s: f64,
    /// Health score in the range [0.0, 1.0].
    pub health_score: f64,
}

/// Main state observer for a digital twin.
///
/// Maintains internal state and provides methods for Luenberger, extended-state,
/// and disturbance observer updates along with anomaly detection facilities.
#[derive(Debug, Clone)]
pub struct StateObserver {
    config: TwinConfig,
    state: Vec<f64>,
    covariance: Vec<Vec<f64>>,
    timestamp_s: f64,
    health: f64,
    residual_history: Vec<Vec<f64>>,
}

impl StateObserver {
    /// Create a new `StateObserver` with zero-initialised state.
    pub fn new(config: TwinConfig) -> Self {
        let n = config.state_dim;
        let cov = (0..n)
            .map(|i| {
                let mut row = vec![0.0; n];
                row[i] = 1.0;
                row
            })
            .collect();
        Self {
            state: vec![0.0; config.state_dim],
            covariance: cov,
            timestamp_s: 0.0,
            health: 1.0,
            residual_history: Vec::new(),
            config,
        }
    }

    /// Return the current state snapshot.
    pub fn snapshot(&self) -> TwinState {
        TwinState {
            state: self.state.clone(),
            covariance: self.covariance.clone(),
            timestamp_s: self.timestamp_s,
            health_score: self.health,
        }
    }

    /// Perform a Luenberger observer update, returning the measurement residual.
    pub fn step_luenberger(
        &mut self,
        measurement: &[f64],
        a_mat: &[Vec<f64>],
        c_mat: &[Vec<f64>],
        l_gain: &[Vec<f64>],
    ) -> Vec<f64> {
        let residual = luenberger_observe(&mut self.state, measurement, a_mat, c_mat, l_gain);
        self.residual_history.push(residual.clone());
        self.timestamp_s += 1.0 / self.config.sample_rate_hz;
        residual
    }

    /// Perform an extended state observer update, returning the residual.
    pub fn step_extended(
        &mut self,
        measurement: &[f64],
        f: &dyn Fn(&[f64]) -> Vec<f64>,
        h: &dyn Fn(&[f64]) -> Vec<f64>,
        gain: &[Vec<f64>],
    ) -> Vec<f64> {
        let residual = extended_state_observe(&mut self.state, measurement, f, h, gain);
        self.residual_history.push(residual.clone());
        self.timestamp_s += 1.0 / self.config.sample_rate_hz;
        residual
    }

    /// Perform a disturbance observer update, returning the estimated disturbance.
    pub fn step_disturbance(
        &mut self,
        input: &[f64],
        output: &[f64],
        gain: f64,
    ) -> Vec<f64> {
        let dist = disturbance_observe(&mut self.state, input, output, gain);
        self.timestamp_s += 1.0 / self.config.sample_rate_hz;
        dist
    }

    /// Update the health score based on accumulated residuals.
    pub fn update_health(&mut self, nominal_variance: &[f64]) {
        if !self.residual_history.is_empty() {
            self.health = health_score(&self.residual_history, nominal_variance);
        }
    }

    /// Return a reference to the accumulated residual history.
    pub fn residual_history(&self) -> &[Vec<f64>] {
        &self.residual_history
    }

    /// Clear the residual history buffer.
    pub fn clear_history(&mut self) {
        self.residual_history.clear();
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Luenberger observer single-step update.
///
/// Computes `x_pred = A * x`, `y_pred = C * x_pred`, `residual = measurement - y_pred`,
/// then corrects: `x = x_pred + L * residual`.
///
/// Returns the measurement residual (innovation) vector.
pub fn luenberger_observe(
    state: &mut Vec<f64>,
    measurement: &[f64],
    a_mat: &[Vec<f64>],
    c_mat: &[Vec<f64>],
    l_gain: &[Vec<f64>],
) -> Vec<f64> {
    // Predict: x_pred = A * x
    let x_pred = mat_vec_mul(a_mat, state);

    // Output prediction: y_pred = C * x_pred
    let y_pred = mat_vec_mul(c_mat, &x_pred);

    // Residual: e = measurement - y_pred
    let residual = compute_residual(measurement, &y_pred);

    // Correct: x = x_pred + L * residual
    let correction = mat_vec_mul(l_gain, &residual);
    for (xi, ci) in state.iter_mut().zip(x_pred.iter()) {
        *xi = *ci;
    }
    for (xi, ci) in state.iter_mut().zip(correction.iter()) {
        *xi += *ci;
    }

    residual
}

/// Extended state observer update using nonlinear state and output models.
///
/// `f` is the nonlinear state transition function, `h` is the nonlinear
/// output function. Returns the output residual.
pub fn extended_state_observe(
    state: &mut Vec<f64>,
    measurement: &[f64],
    f: &dyn Fn(&[f64]) -> Vec<f64>,
    h: &dyn Fn(&[f64]) -> Vec<f64>,
    gain: &[Vec<f64>],
) -> Vec<f64> {
    // Predict
    let x_pred = f(state);

    // Output prediction
    let y_pred = h(&x_pred);

    // Residual
    let residual = compute_residual(measurement, &y_pred);

    // Correct
    let correction = mat_vec_mul(gain, &residual);
    for (i, xi) in state.iter_mut().enumerate() {
        *xi = x_pred[i] + correction[i];
    }

    residual
}

/// Disturbance observer (DOB) update.
///
/// Estimates disturbance by comparing the model output with the actual output.
/// `state` holds the internal DOB filter state. Returns the estimated disturbance
/// vector (same length as `output`).
pub fn disturbance_observe(
    state: &mut Vec<f64>,
    input: &[f64],
    output: &[f64],
    gain: f64,
) -> Vec<f64> {
    // Ensure state is large enough
    if state.len() < output.len() {
        state.resize(output.len(), 0.0);
    }

    let mut disturbance = vec![0.0; output.len()];
    for i in 0..output.len() {
        let input_val = if i < input.len() { input[i] } else { 0.0 };
        // Simple first-order DOB: d_hat += gain * (y - x - u)
        let error = output[i] - state[i] - input_val;
        state[i] += gain * error;
        disturbance[i] = state[i];
    }
    disturbance
}

/// Compute the residual (innovation) vector: `predicted - measured` element-wise.
pub fn compute_residual(predicted: &[f64], measured: &[f64]) -> Vec<f64> {
    predicted
        .iter()
        .zip(measured.iter())
        .map(|(p, m)| p - m)
        .collect()
}

/// Compute the L2 (Euclidean) norm of a residual vector.
pub fn residual_norm(residual: &[f64]) -> f64 {
    residual.iter().map(|r| r * r).sum::<f64>().sqrt()
}

/// Chi-squared anomaly test.
///
/// Computes `r^T * C^{-1} * r` for the residual `r` and covariance `C`.
/// Returns `true` if the statistic exceeds `threshold`, indicating an anomaly.
///
/// For simplicity, only the diagonal of the covariance is used (assumed
/// diagonal or diagonally-dominant).
pub fn chi_squared_test(
    residual: &[f64],
    covariance: &[Vec<f64>],
    threshold: f64,
) -> bool {
    let mut chi2 = 0.0;
    for (i, &r) in residual.iter().enumerate() {
        let var = covariance[i][i];
        if var > 0.0 {
            chi2 += (r * r) / var;
        }
    }
    chi2 > threshold
}

/// CUSUM (cumulative sum) change-point detector.
///
/// Scans the `residuals` sequence and returns `Some(index)` of the first
/// sample where the cumulative sum exceeds `threshold`. `drift` is the
/// allowable slack per sample.
///
/// Returns `None` if no change is detected.
pub fn cusum_detect(residuals: &[f64], threshold: f64, drift: f64) -> Option<usize> {
    let mut s_pos = 0.0_f64;
    let mut s_neg = 0.0_f64;
    for (i, &r) in residuals.iter().enumerate() {
        s_pos = (s_pos + r - drift).max(0.0);
        s_neg = (s_neg - r - drift).max(0.0);
        if s_pos > threshold || s_neg > threshold {
            return Some(i);
        }
    }
    None
}

/// Forward-predict a state through `latency_steps` applications of the
/// state transition matrix `a_mat`.
///
/// This compensates for measurement or communication latency by projecting
/// the state forward in time.
pub fn latency_compensate(
    state: &[f64],
    a_mat: &[Vec<f64>],
    latency_steps: usize,
) -> Vec<f64> {
    let mut current = state.to_vec();
    for _ in 0..latency_steps {
        current = mat_vec_mul(a_mat, &current);
    }
    current
}

/// Estimate per-parameter drift rate from a history of state snapshots.
///
/// Uses a simple finite-difference on the first and last entries.
/// Returns a vector of drift rates (units per snapshot interval).
pub fn parameter_drift_rate(history: &[Vec<f64>]) -> Vec<f64> {
    if history.len() < 2 {
        return if let Some(first) = history.first() {
            vec![0.0; first.len()]
        } else {
            Vec::new()
        };
    }
    let first = &history[0];
    let last = &history[history.len() - 1];
    let n = (history.len() - 1) as f64;
    first
        .iter()
        .zip(last.iter())
        .map(|(a, b)| (b - a) / n)
        .collect()
}

/// Compute a health score in [0.0, 1.0] from residual history.
///
/// Each residual component is compared against its nominal variance.
/// A score of 1.0 means all residuals are within nominal bounds; 0.0 means
/// severe degradation.
pub fn health_score(residuals: &[Vec<f64>], nominal_variance: &[f64]) -> f64 {
    if residuals.is_empty() || nominal_variance.is_empty() {
        return 1.0;
    }

    let dim = nominal_variance.len();
    let n = residuals.len() as f64;

    // Compute mean squared residual per dimension
    let mut msr = vec![0.0; dim];
    for r in residuals {
        for (j, &val) in r.iter().enumerate() {
            if j < dim {
                msr[j] += val * val;
            }
        }
    }
    for v in &mut msr {
        *v /= n;
    }

    // Score: average of exp(-excess_ratio) per dimension
    let mut score_sum = 0.0;
    let mut count = 0;
    for (j, &var) in nominal_variance.iter().enumerate() {
        if var > 0.0 && j < msr.len() {
            let ratio = msr[j] / var;
            // ratio <= 1 => score 1.0, ratio >> 1 => score -> 0
            score_sum += (-((ratio - 1.0).max(0.0))).exp();
            count += 1;
        }
    }

    if count == 0 {
        1.0
    } else {
        score_sum / count as f64
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Multiply a matrix (given as `&[Vec<f64>]` row-major) by a column vector.
fn mat_vec_mul(mat: &[Vec<f64>], vec: &[f64]) -> Vec<f64> {
    mat.iter()
        .map(|row| row.iter().zip(vec.iter()).map(|(a, b)| a * b).sum())
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn identity_2x2() -> Vec<Vec<f64>> {
        vec![vec![1.0, 0.0], vec![0.0, 1.0]]
    }

    fn simple_a() -> Vec<Vec<f64>> {
        vec![vec![1.0, 0.1], vec![0.0, 1.0]]
    }

    #[test]
    fn test_twin_config_construction() {
        let cfg = TwinConfig {
            state_dim: 3,
            output_dim: 2,
            sample_rate_hz: 1000.0,
            sync_interval_s: 0.01,
        };
        assert_eq!(cfg.state_dim, 3);
        assert_eq!(cfg.output_dim, 2);
        assert!((cfg.sample_rate_hz - 1000.0).abs() < f64::EPSILON);
        assert!((cfg.sync_interval_s - 0.01).abs() < f64::EPSILON);
    }

    #[test]
    fn test_twin_state_snapshot() {
        let cfg = TwinConfig {
            state_dim: 2,
            output_dim: 1,
            sample_rate_hz: 100.0,
            sync_interval_s: 0.1,
        };
        let observer = StateObserver::new(cfg);
        let snap = observer.snapshot();
        assert_eq!(snap.state.len(), 2);
        assert_eq!(snap.covariance.len(), 2);
        assert!((snap.timestamp_s - 0.0).abs() < f64::EPSILON);
        assert!((snap.health_score - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_compute_residual_basic() {
        let r = compute_residual(&[1.0, 2.0, 3.0], &[1.0, 2.0, 3.0]);
        assert!(r.iter().all(|&v| v.abs() < f64::EPSILON));
    }

    #[test]
    fn test_compute_residual_nonzero() {
        let r = compute_residual(&[2.0, 4.0], &[1.0, 3.0]);
        assert!((r[0] - 1.0).abs() < f64::EPSILON);
        assert!((r[1] - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_residual_norm_zero() {
        assert!(residual_norm(&[0.0, 0.0]).abs() < f64::EPSILON);
    }

    #[test]
    fn test_residual_norm_unit() {
        let n = residual_norm(&[3.0, 4.0]);
        assert!((n - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_luenberger_identity_system() {
        // A = I, C = I, L = 0.5*I, state starts at zero
        let mut state = vec![0.0, 0.0];
        let measurement = vec![1.0, 2.0];
        let a = identity_2x2();
        let c = identity_2x2();
        let l = vec![vec![0.5, 0.0], vec![0.0, 0.5]];

        let residual = luenberger_observe(&mut state, &measurement, &a, &c, &l);
        // y_pred = C * A * x = I * I * [0,0] = [0,0]
        // residual = measurement - y_pred = [1,2] - [0,0] = [1,2]
        assert!((residual[0] - 1.0).abs() < 1e-12);
        assert!((residual[1] - 2.0).abs() < 1e-12);

        // State should be corrected: x = A*x + L*residual = [0,0] + 0.5*[1,2] = [0.5,1.0]
        assert!((state[0] - 0.5).abs() < 1e-12);
        assert!((state[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_luenberger_convergence() {
        // After many steps with constant measurement, state should converge
        let mut state = vec![0.0, 0.0];
        let measurement = vec![5.0, 3.0];
        let a = identity_2x2();
        let c = identity_2x2();
        let l = vec![vec![0.5, 0.0], vec![0.0, 0.5]];

        for _ in 0..100 {
            luenberger_observe(&mut state, &measurement, &a, &c, &l);
        }
        // With A=I, C=I, L=0.5*I:
        // x_{k+1} = A*x_k + L*(meas - C*A*x_k) = x_k + 0.5*(meas - x_k) = 0.5*x_k + 0.5*meas
        // Fixed point: x = meas
        assert!((state[0] - 5.0).abs() < 1e-6);
        assert!((state[1] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_extended_state_observe_linear() {
        // Use linear functions to verify extended observer matches Luenberger behavior
        let mut state = vec![0.0, 0.0];
        let measurement = vec![1.0, 0.5];
        let gain = identity_2x2();

        let f = |x: &[f64]| x.to_vec(); // identity transition
        let h = |x: &[f64]| x.to_vec(); // identity output

        let residual = extended_state_observe(&mut state, &measurement, &f, &h, &gain);
        // residual = measurement - h(f(x)) = [1,0.5] - [0,0] = [1, 0.5]
        assert!((residual[0] - 1.0).abs() < 1e-12);
        assert!((residual[1] - 0.5).abs() < 1e-12);
        // state = f(x) + gain * residual = [0,0] + I*[1,0.5] = [1, 0.5]
        assert!((state[0] - 1.0).abs() < 1e-12);
        assert!((state[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_extended_state_observe_nonlinear() {
        let mut state = vec![1.0];
        let measurement = vec![2.0];
        let gain = vec![vec![0.5]];

        let f = |x: &[f64]| vec![x[0] * x[0]]; // square
        let h = |x: &[f64]| vec![x[0]]; // identity output

        let residual = extended_state_observe(&mut state, &measurement, &f, &h, &gain);
        // f(1) = 1, h(1) = 1, residual = 2 - 1 = 1
        // state = 1 + 0.5*1 = 1.5
        assert!((residual[0] - 1.0).abs() < 1e-12);
        assert!((state[0] - 1.5).abs() < 1e-12);
    }

    #[test]
    fn test_disturbance_observe_no_disturbance() {
        let mut state = vec![0.0];
        let input = vec![1.0];
        let output = vec![1.0]; // output matches input, no disturbance
        let dist = disturbance_observe(&mut state, &input, &output, 0.5);
        // error = output - state - input = 1 - 0 - 1 = 0
        // state += 0.5 * 0 = 0
        assert!((dist[0]).abs() < 1e-12);
    }

    #[test]
    fn test_disturbance_observe_with_disturbance() {
        let mut state = vec![0.0];
        let input = vec![1.0];
        let output = vec![2.0]; // disturbance of 1.0
        let dist = disturbance_observe(&mut state, &input, &output, 1.0);
        // error = 2 - 0 - 1 = 1, state += 1*1 = 1
        assert!((dist[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_disturbance_observe_state_resize() {
        let mut state = vec![];
        let input = vec![1.0, 2.0];
        let output = vec![1.5, 2.5];
        let dist = disturbance_observe(&mut state, &input, &output, 0.5);
        assert_eq!(dist.len(), 2);
        assert_eq!(state.len(), 2);
    }

    #[test]
    fn test_chi_squared_no_anomaly() {
        let residual = vec![0.1, 0.1];
        let cov = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        // chi2 = 0.01 + 0.01 = 0.02, threshold 5.99
        assert!(!chi_squared_test(&residual, &cov, 5.99));
    }

    #[test]
    fn test_chi_squared_anomaly() {
        let residual = vec![3.0, 3.0];
        let cov = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        // chi2 = 9 + 9 = 18 > 5.99
        assert!(chi_squared_test(&residual, &cov, 5.99));
    }

    #[test]
    fn test_cusum_no_change() {
        let residuals: Vec<f64> = vec![0.1, -0.1, 0.05, -0.05, 0.02];
        assert!(cusum_detect(&residuals, 5.0, 0.5).is_none());
    }

    #[test]
    fn test_cusum_positive_shift() {
        // Large positive shift should trigger detection
        let mut residuals = vec![0.0; 5];
        residuals.extend(vec![5.0; 10]); // step change
        let idx = cusum_detect(&residuals, 10.0, 0.5);
        assert!(idx.is_some());
        assert!(idx.unwrap() >= 5);
    }

    #[test]
    fn test_cusum_negative_shift() {
        let mut residuals = vec![0.0; 5];
        residuals.extend(vec![-5.0; 10]);
        let idx = cusum_detect(&residuals, 10.0, 0.5);
        assert!(idx.is_some());
    }

    #[test]
    fn test_latency_compensate_zero_steps() {
        let state = vec![1.0, 2.0];
        let a = identity_2x2();
        let result = latency_compensate(&state, &a, 0);
        assert_eq!(result, state);
    }

    #[test]
    fn test_latency_compensate_forward_predict() {
        let state = vec![1.0, 0.5]; // position=1, velocity=0.5
        let a = simple_a(); // [[1, 0.1], [0, 1]]
        let result = latency_compensate(&state, &a, 3);
        // After 1 step: [1 + 0.1*0.5, 0.5] = [1.05, 0.5]
        // After 2 steps: [1.05 + 0.05, 0.5] = [1.10, 0.5]
        // After 3 steps: [1.10 + 0.05, 0.5] = [1.15, 0.5]
        assert!((result[0] - 1.15).abs() < 1e-12);
        assert!((result[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_parameter_drift_rate_empty() {
        let result = parameter_drift_rate(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_parameter_drift_rate_single() {
        let result = parameter_drift_rate(&[vec![1.0, 2.0]]);
        assert_eq!(result, vec![0.0, 0.0]);
    }

    #[test]
    fn test_parameter_drift_rate_linear() {
        let history = vec![
            vec![0.0, 10.0],
            vec![1.0, 8.0],
            vec![2.0, 6.0],
            vec![3.0, 4.0],
        ];
        let rate = parameter_drift_rate(&history);
        assert!((rate[0] - 1.0).abs() < 1e-12);
        assert!((rate[1] - (-2.0)).abs() < 1e-12);
    }

    #[test]
    fn test_health_score_perfect() {
        // Residuals well within nominal variance
        let residuals = vec![vec![0.01, 0.01]; 10];
        let nominal = vec![1.0, 1.0];
        let score = health_score(&residuals, &nominal);
        // MSR = 0.0001 per dim, ratio = 0.0001 << 1, so score ~ 1.0
        assert!(score > 0.99);
    }

    #[test]
    fn test_health_score_degraded() {
        // Large residuals compared to nominal variance
        let residuals = vec![vec![10.0, 10.0]; 50];
        let nominal = vec![1.0, 1.0];
        let score = health_score(&residuals, &nominal);
        // MSR = 100 per dim, ratio = 100, exp(-(100-1)) ~ 0
        assert!(score < 0.01);
    }

    #[test]
    fn test_health_score_empty_residuals() {
        let score = health_score(&[], &[1.0]);
        assert!((score - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_observer_step_advances_timestamp() {
        let cfg = TwinConfig {
            state_dim: 2,
            output_dim: 2,
            sample_rate_hz: 100.0,
            sync_interval_s: 0.1,
        };
        let mut obs = StateObserver::new(cfg);
        let a = identity_2x2();
        let c = identity_2x2();
        let l = vec![vec![0.1, 0.0], vec![0.0, 0.1]];

        obs.step_luenberger(&[1.0, 1.0], &a, &c, &l);
        assert!((obs.snapshot().timestamp_s - 0.01).abs() < 1e-12);

        obs.step_luenberger(&[1.0, 1.0], &a, &c, &l);
        assert!((obs.snapshot().timestamp_s - 0.02).abs() < 1e-12);
    }

    #[test]
    fn test_observer_update_health() {
        let cfg = TwinConfig {
            state_dim: 1,
            output_dim: 1,
            sample_rate_hz: 100.0,
            sync_interval_s: 0.1,
        };
        let mut obs = StateObserver::new(cfg);

        // Simulate some small residuals
        let a = vec![vec![1.0]];
        let c = vec![vec![1.0]];
        let l = vec![vec![0.5]];
        for _ in 0..10 {
            obs.step_luenberger(&[0.01], &a, &c, &l);
        }

        obs.update_health(&[1.0]);
        assert!(obs.snapshot().health_score > 0.9);
    }

    #[test]
    fn test_observer_clear_history() {
        let cfg = TwinConfig {
            state_dim: 1,
            output_dim: 1,
            sample_rate_hz: 100.0,
            sync_interval_s: 0.1,
        };
        let mut obs = StateObserver::new(cfg);
        let a = vec![vec![1.0]];
        let c = vec![vec![1.0]];
        let l = vec![vec![0.5]];

        obs.step_luenberger(&[1.0], &a, &c, &l);
        assert_eq!(obs.residual_history().len(), 1);

        obs.clear_history();
        assert!(obs.residual_history().is_empty());
    }

    #[test]
    fn test_mat_vec_mul() {
        let m = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let v = vec![5.0, 6.0];
        let result = mat_vec_mul(&m, &v);
        assert!((result[0] - 17.0).abs() < 1e-12);
        assert!((result[1] - 39.0).abs() < 1e-12);
    }
}
