//! Unscented Kalman Filter (UKF) — Nonlinear state estimation
//!
//! Uses the unscented transform to propagate mean and covariance through nonlinear
//! functions, avoiding the linearization errors of the Extended Kalman Filter (EKF).
//! Applications: carrier/timing tracking at low SNR, GNSS navigation, nonlinear
//! channel estimation, adaptive equalization.
//!
//! ## Algorithm
//!
//! 1. Generate 2n+1 sigma points around the current state estimate
//! 2. Propagate sigma points through the nonlinear state transition f()
//! 3. Compute predicted mean and covariance from transformed sigma points
//! 4. Propagate sigma points through measurement model h()
//! 5. Compute innovation covariance and cross-covariance
//! 6. Compute Kalman gain and update state
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::unscented_kalman_filter::{UnscentedKalmanFilter, UkfModel};
//!
//! struct ConstantVelocity;
//! impl UkfModel for ConstantVelocity {
//!     fn state_dim(&self) -> usize { 2 }
//!     fn measurement_dim(&self) -> usize { 1 }
//!     fn state_transition(&self, state: &[f64], dt: f64) -> Vec<f64> {
//!         vec![state[0] + state[1] * dt, state[1]]
//!     }
//!     fn measurement_model(&self, state: &[f64]) -> Vec<f64> {
//!         vec![state[0]]
//!     }
//!     fn process_noise(&self, _dt: f64) -> Vec<Vec<f64>> {
//!         vec![vec![0.1, 0.0], vec![0.0, 0.1]]
//!     }
//!     fn measurement_noise(&self) -> Vec<Vec<f64>> {
//!         vec![vec![1.0]]
//!     }
//! }
//!
//! let model = ConstantVelocity;
//! let mut ukf = UnscentedKalmanFilter::new(&model);
//! ukf.predict(&model, 1.0);
//! ukf.update(&[1.0], &model);
//! assert_eq!(ukf.state().len(), 2);
//! ```

/// Trait for UKF system models.
pub trait UkfModel {
    /// State vector dimension.
    fn state_dim(&self) -> usize;
    /// Measurement vector dimension.
    fn measurement_dim(&self) -> usize;
    /// Nonlinear state transition: x_{k+1} = f(x_k, dt).
    fn state_transition(&self, state: &[f64], dt: f64) -> Vec<f64>;
    /// Nonlinear measurement model: z = h(x).
    fn measurement_model(&self, state: &[f64]) -> Vec<f64>;
    /// Process noise covariance matrix Q (n × n).
    fn process_noise(&self, dt: f64) -> Vec<Vec<f64>>;
    /// Measurement noise covariance matrix R (m × m).
    fn measurement_noise(&self) -> Vec<Vec<f64>>;
}

/// Unscented Kalman Filter.
#[derive(Debug, Clone)]
pub struct UnscentedKalmanFilter {
    /// State dimension.
    n: usize,
    /// Measurement dimension.
    m: usize,
    /// State estimate.
    x: Vec<f64>,
    /// State covariance.
    p: Vec<Vec<f64>>,
    /// Spread parameter (default 1e-3).
    alpha: f64,
    /// Prior knowledge parameter (default 2.0 for Gaussian).
    beta: f64,
    /// Secondary scaling parameter (default 0.0).
    kappa: f64,
    /// Derived: alpha^2 * (n + kappa) - n.
    lambda: f64,
}

impl UnscentedKalmanFilter {
    /// Create a new UKF from a model.
    pub fn new(model: &dyn UkfModel) -> Self {
        let n = model.state_dim();
        let m = model.measurement_dim();
        let alpha = 1e-3;
        let beta = 2.0;
        let kappa = 0.0;
        let lambda = alpha * alpha * (n as f64 + kappa) - n as f64;

        Self {
            n,
            m,
            x: vec![0.0; n],
            p: identity(n),
            alpha,
            beta,
            kappa,
            lambda,
        }
    }

    /// Create with custom tuning parameters.
    pub fn with_params(model: &dyn UkfModel, alpha: f64, beta: f64, kappa: f64) -> Self {
        let n = model.state_dim();
        let m = model.measurement_dim();
        let lambda = alpha * alpha * (n as f64 + kappa) - n as f64;

        Self {
            n,
            m,
            x: vec![0.0; n],
            p: identity(n),
            alpha,
            beta,
            kappa,
            lambda,
        }
    }

    /// Set the state estimate and covariance.
    pub fn set_state(&mut self, state: Vec<f64>, covariance: Vec<Vec<f64>>) {
        assert_eq!(state.len(), self.n);
        assert_eq!(covariance.len(), self.n);
        self.x = state;
        self.p = covariance;
    }

    /// Get current state estimate.
    pub fn state(&self) -> &[f64] {
        &self.x
    }

    /// Get current state covariance.
    pub fn covariance(&self) -> &[Vec<f64>] {
        &self.p
    }

    /// Prediction step: propagate state through nonlinear transition.
    pub fn predict(&mut self, model: &dyn UkfModel, dt: f64) {
        let n = self.n;

        // Generate sigma points
        let sigma_pts = self.generate_sigma_points();

        // Propagate through state transition
        let mut sigma_f: Vec<Vec<f64>> = Vec::with_capacity(2 * n + 1);
        for sp in &sigma_pts {
            sigma_f.push(model.state_transition(sp, dt));
        }

        // Compute weights
        let (wm, wc) = self.compute_weights();

        // Predicted mean
        let mut x_pred = vec![0.0; n];
        for i in 0..sigma_f.len() {
            for j in 0..n {
                x_pred[j] += wm[i] * sigma_f[i][j];
            }
        }

        // Predicted covariance
        let mut p_pred = model.process_noise(dt);
        for i in 0..sigma_f.len() {
            let dx: Vec<f64> = (0..n).map(|j| sigma_f[i][j] - x_pred[j]).collect();
            for r in 0..n {
                for c in 0..n {
                    p_pred[r][c] += wc[i] * dx[r] * dx[c];
                }
            }
        }

        self.x = x_pred;
        self.p = p_pred;
    }

    /// Update step: incorporate a measurement.
    pub fn update(&mut self, measurement: &[f64], model: &dyn UkfModel) {
        assert_eq!(measurement.len(), self.m);
        let n = self.n;
        let m = self.m;

        // Generate sigma points from predicted state
        let sigma_pts = self.generate_sigma_points();

        // Propagate through measurement model
        let mut sigma_z: Vec<Vec<f64>> = Vec::with_capacity(2 * n + 1);
        for sp in &sigma_pts {
            sigma_z.push(model.measurement_model(sp));
        }

        let (wm, wc) = self.compute_weights();

        // Predicted measurement mean
        let mut z_pred = vec![0.0; m];
        for i in 0..sigma_z.len() {
            for j in 0..m {
                z_pred[j] += wm[i] * sigma_z[i][j];
            }
        }

        // Innovation covariance S = P_zz + R
        let mut s = model.measurement_noise();
        for i in 0..sigma_z.len() {
            let dz: Vec<f64> = (0..m).map(|j| sigma_z[i][j] - z_pred[j]).collect();
            for r in 0..m {
                for c in 0..m {
                    s[r][c] += wc[i] * dz[r] * dz[c];
                }
            }
        }

        // Cross-covariance P_xz
        let mut p_xz = vec![vec![0.0; m]; n];
        for i in 0..sigma_pts.len() {
            let dx: Vec<f64> = (0..n).map(|j| sigma_pts[i][j] - self.x[j]).collect();
            let dz: Vec<f64> = (0..m).map(|j| sigma_z[i][j] - z_pred[j]).collect();
            for r in 0..n {
                for c in 0..m {
                    p_xz[r][c] += wc[i] * dx[r] * dz[c];
                }
            }
        }

        // Kalman gain K = P_xz * S^(-1)
        let s_inv = invert_matrix(&s);
        let k = mat_mul_rect(&p_xz, &s_inv);

        // Innovation
        let innovation: Vec<f64> = (0..m).map(|j| measurement[j] - z_pred[j]).collect();

        // State update: x = x + K * innovation
        for i in 0..n {
            for j in 0..m {
                self.x[i] += k[i][j] * innovation[j];
            }
        }

        // Covariance update: P = P - K * S * K^T
        let k_s = mat_mul_rect(&k, &s);
        let k_t = transpose(&k);
        let k_s_kt = mat_mul_rect(&k_s, &k_t);
        for i in 0..n {
            for j in 0..n {
                self.p[i][j] -= k_s_kt[i][j];
            }
        }
    }

    /// Generate 2n+1 sigma points.
    fn generate_sigma_points(&self) -> Vec<Vec<f64>> {
        let n = self.n;
        let scale = (n as f64 + self.lambda).max(0.0);

        // Cholesky decomposition of (n + lambda) * P
        let mut scaled_p = self.p.clone();
        for i in 0..n {
            for j in 0..n {
                scaled_p[i][j] *= scale;
            }
        }
        let l = cholesky(&scaled_p);

        let mut sigma = Vec::with_capacity(2 * n + 1);

        // sigma_0 = x
        sigma.push(self.x.clone());

        // sigma_i = x + L_i (column i of L)
        for i in 0..n {
            let mut sp = self.x.clone();
            for j in 0..n {
                sp[j] += l[j][i];
            }
            sigma.push(sp);
        }

        // sigma_{n+i} = x - L_i
        for i in 0..n {
            let mut sp = self.x.clone();
            for j in 0..n {
                sp[j] -= l[j][i];
            }
            sigma.push(sp);
        }

        sigma
    }

    /// Compute sigma point weights (mean weights, covariance weights).
    fn compute_weights(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.n;
        let num_sigma = 2 * n + 1;
        let n_plus_lambda = n as f64 + self.lambda;

        let mut wm = vec![0.0; num_sigma];
        let mut wc = vec![0.0; num_sigma];

        wm[0] = self.lambda / n_plus_lambda;
        wc[0] = self.lambda / n_plus_lambda + (1.0 - self.alpha * self.alpha + self.beta);

        let w = 1.0 / (2.0 * n_plus_lambda);
        for i in 1..num_sigma {
            wm[i] = w;
            wc[i] = w;
        }

        (wm, wc)
    }

    /// Get the normalized estimation error squared (NEES) for consistency checking.
    pub fn nees(&self, true_state: &[f64]) -> f64 {
        let n = self.n;
        let err: Vec<f64> = (0..n).map(|i| true_state[i] - self.x[i]).collect();
        let p_inv = invert_matrix(&self.p);
        let mut nees = 0.0;
        for i in 0..n {
            for j in 0..n {
                nees += err[i] * p_inv[i][j] * err[j];
            }
        }
        nees
    }
}

// --- Linear algebra helpers ---

fn identity(n: usize) -> Vec<Vec<f64>> {
    let mut m = vec![vec![0.0; n]; n];
    for i in 0..n {
        m[i][i] = 1.0;
    }
    m
}

fn cholesky(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let mut l = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i][k] * l[j][k];
            }
            if i == j {
                let val = a[i][i] - sum;
                l[i][j] = if val > 0.0 { val.sqrt() } else { 1e-10 };
            } else {
                l[i][j] = if l[j][j].abs() > 1e-15 {
                    (a[i][j] - sum) / l[j][j]
                } else {
                    0.0
                };
            }
        }
    }
    l
}

fn invert_matrix(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let mut aug = vec![vec![0.0; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][i + n] = 1.0;
    }

    for col in 0..n {
        let mut max_val = 0.0_f64;
        let mut max_row = col;
        for row in col..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            continue;
        }

        let pivot_inv = 1.0 / pivot;
        for j in 0..2 * n {
            aug[col][j] *= pivot_inv;
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..2 * n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    let mut result = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            result[i][j] = aug[i][j + n];
        }
    }
    result
}

fn mat_mul_rect(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rows = a.len();
    let cols = b[0].len();
    let inner = a[0].len();
    let mut c = vec![vec![0.0; cols]; rows];
    for i in 0..rows {
        for j in 0..cols {
            for k in 0..inner {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

fn transpose(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rows = a.len();
    let cols = a[0].len();
    let mut t = vec![vec![0.0; rows]; cols];
    for i in 0..rows {
        for j in 0..cols {
            t[j][i] = a[i][j];
        }
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple 1D constant-velocity model.
    struct ConstVelocity {
        q: f64,
        r: f64,
    }

    impl UkfModel for ConstVelocity {
        fn state_dim(&self) -> usize { 2 }
        fn measurement_dim(&self) -> usize { 1 }
        fn state_transition(&self, state: &[f64], dt: f64) -> Vec<f64> {
            vec![state[0] + state[1] * dt, state[1]]
        }
        fn measurement_model(&self, state: &[f64]) -> Vec<f64> {
            vec![state[0]]
        }
        fn process_noise(&self, dt: f64) -> Vec<Vec<f64>> {
            let q = self.q;
            vec![
                vec![q * dt * dt * dt / 3.0, q * dt * dt / 2.0],
                vec![q * dt * dt / 2.0, q * dt],
            ]
        }
        fn measurement_noise(&self) -> Vec<Vec<f64>> {
            vec![vec![self.r]]
        }
    }

    /// Nonlinear range-bearing model.
    struct RangeBearing;

    impl UkfModel for RangeBearing {
        fn state_dim(&self) -> usize { 4 } // x, y, vx, vy
        fn measurement_dim(&self) -> usize { 2 } // range, bearing
        fn state_transition(&self, state: &[f64], dt: f64) -> Vec<f64> {
            vec![
                state[0] + state[2] * dt,
                state[1] + state[3] * dt,
                state[2],
                state[3],
            ]
        }
        fn measurement_model(&self, state: &[f64]) -> Vec<f64> {
            let range = (state[0] * state[0] + state[1] * state[1]).sqrt();
            let bearing = state[1].atan2(state[0]);
            vec![range, bearing]
        }
        fn process_noise(&self, _dt: f64) -> Vec<Vec<f64>> {
            let q = 0.1;
            vec![
                vec![q, 0.0, 0.0, 0.0],
                vec![0.0, q, 0.0, 0.0],
                vec![0.0, 0.0, q, 0.0],
                vec![0.0, 0.0, 0.0, q],
            ]
        }
        fn measurement_noise(&self) -> Vec<Vec<f64>> {
            vec![
                vec![1.0, 0.0],
                vec![0.0, 0.01],
            ]
        }
    }

    #[test]
    fn test_1d_position_tracking() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let mut ukf = UnscentedKalmanFilter::new(&model);
        ukf.set_state(vec![0.0, 1.0], vec![vec![1.0, 0.0], vec![0.0, 1.0]]);

        // Simulate constant velocity: position = t, velocity = 1
        for t in 1..=20 {
            ukf.predict(&model, 1.0);
            let z = t as f64 + 0.1; // measurement with small bias
            ukf.update(&[z], &model);
        }

        let state = ukf.state();
        assert!((state[0] - 20.0).abs() < 2.0,
            "Position: expected ~20, got {:.2}", state[0]);
        assert!((state[1] - 1.0).abs() < 1.0,
            "Velocity: expected ~1, got {:.2}", state[1]);
    }

    #[test]
    fn test_nonlinear_range_bearing() {
        let model = RangeBearing;
        let mut ukf = UnscentedKalmanFilter::new(&model);
        ukf.set_state(
            vec![10.0, 0.0, 1.0, 0.5],
            vec![
                vec![2.0, 0.0, 0.0, 0.0],
                vec![0.0, 2.0, 0.0, 0.0],
                vec![0.0, 0.0, 1.0, 0.0],
                vec![0.0, 0.0, 0.0, 1.0],
            ],
        );

        // True trajectory: x = 10 + t, y = 0.5t
        for t in 1..=10 {
            ukf.predict(&model, 1.0);
            let true_x = 10.0 + t as f64;
            let true_y = 0.5 * t as f64;
            let range = (true_x * true_x + true_y * true_y).sqrt();
            let bearing = true_y.atan2(true_x);
            ukf.update(&[range, bearing], &model);
        }

        let state = ukf.state();
        assert!((state[0] - 20.0).abs() < 3.0, "X: expected ~20, got {:.2}", state[0]);
        assert!((state[1] - 5.0).abs() < 3.0, "Y: expected ~5, got {:.2}", state[1]);
    }

    #[test]
    fn test_divergence_recovery() {
        let model = ConstVelocity { q: 1.0, r: 0.5 };
        let mut ukf = UnscentedKalmanFilter::new(&model);
        // Start with large initial error
        ukf.set_state(vec![100.0, -10.0], vec![vec![1000.0, 0.0], vec![0.0, 100.0]]);

        // True state: position = t, velocity = 1
        for t in 1..=50 {
            ukf.predict(&model, 1.0);
            ukf.update(&[t as f64], &model);
        }

        let state = ukf.state();
        assert!((state[0] - 50.0).abs() < 5.0,
            "Should converge to ~50, got {:.2}", state[0]);
    }

    #[test]
    fn test_sigma_point_count() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let ukf = UnscentedKalmanFilter::new(&model);
        let sigma = ukf.generate_sigma_points();
        assert_eq!(sigma.len(), 2 * 2 + 1); // 2n + 1 for n=2
    }

    #[test]
    fn test_weights_sum_to_one() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let ukf = UnscentedKalmanFilter::new(&model);
        let (wm, _) = ukf.compute_weights();
        let sum: f64 = wm.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10, "Mean weights should sum to 1, got {:.10}", sum);
    }

    #[test]
    fn test_covariance_convergence() {
        let model = ConstVelocity { q: 0.01, r: 1.0 };
        let mut ukf = UnscentedKalmanFilter::new(&model);
        ukf.set_state(vec![0.0, 1.0], vec![vec![10.0, 0.0], vec![0.0, 10.0]]);

        let initial_trace = ukf.covariance()[0][0] + ukf.covariance()[1][1];

        for t in 1..=100 {
            ukf.predict(&model, 1.0);
            ukf.update(&[t as f64], &model);
        }

        let final_trace = ukf.covariance()[0][0] + ukf.covariance()[1][1];
        assert!(final_trace < initial_trace,
            "Covariance should decrease: initial={:.2}, final={:.2}", initial_trace, final_trace);
    }

    #[test]
    fn test_custom_params() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let ukf = UnscentedKalmanFilter::with_params(&model, 0.5, 2.0, 1.0);
        assert_eq!(ukf.state().len(), 2);
        let sigma = ukf.generate_sigma_points();
        assert_eq!(sigma.len(), 5); // 2*2 + 1
    }

    #[test]
    fn test_nees_metric() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let mut ukf = UnscentedKalmanFilter::new(&model);
        ukf.set_state(vec![0.0, 1.0], vec![vec![1.0, 0.0], vec![0.0, 1.0]]);

        for t in 1..=20 {
            ukf.predict(&model, 1.0);
            ukf.update(&[t as f64], &model);
        }

        let nees = ukf.nees(&[20.0, 1.0]);
        // NEES should be small for well-tracked state
        assert!(nees < 50.0, "NEES too large: {:.2}", nees);
    }

    #[test]
    fn test_predict_without_update() {
        let model = ConstVelocity { q: 0.1, r: 1.0 };
        let mut ukf = UnscentedKalmanFilter::new(&model);
        ukf.set_state(vec![0.0, 1.0], vec![vec![1.0, 0.0], vec![0.0, 1.0]]);

        // Only predict, no updates — covariance should grow
        for _ in 0..10 {
            ukf.predict(&model, 1.0);
        }

        let state = ukf.state();
        assert!((state[0] - 10.0).abs() < 0.1, "Position should be ~10, got {:.2}", state[0]);
        assert!(ukf.covariance()[0][0] > 1.0, "Covariance should grow without updates");
    }
}
