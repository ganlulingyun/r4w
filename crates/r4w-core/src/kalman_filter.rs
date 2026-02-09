//! Kalman Filter — Linear state estimation
//!
//! General-purpose linear Kalman filter for tracking continuous-valued
//! state vectors from noisy observations. Applications: carrier tracking
//! (PLL/FLL smoothing), GNSS position/velocity, clock drift estimation,
//! and channel parameter tracking. Supports configurable state and
//! measurement dimensions.
//! GNU Radio equivalent: custom OOT / `gnss-sdr` tracking loops.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::kalman_filter::KalmanFilter;
//!
//! // 1D position tracking: state = [position], measurement = [position]
//! let mut kf = KalmanFilter::new_1d(0.0, 1.0, 0.5);
//! for &meas in &[1.1, 0.9, 1.05, 0.95, 1.0] {
//!     kf.update_1d(meas);
//!     kf.predict_1d();
//! }
//! assert!((kf.state_1d() - 1.0).abs() < 0.5);
//! ```

/// General Kalman filter with NxN matrices (small fixed-size).
///
/// For SDR applications, state dimensions are typically 1-4:
/// - 1D: frequency tracking
/// - 2D: phase + frequency
/// - 3D: position + velocity + acceleration (1-axis)
/// - 4D: position + velocity (2-axis)
#[derive(Debug, Clone)]
pub struct KalmanFilter {
    /// State dimension.
    n: usize,
    /// Measurement dimension.
    m: usize,
    /// State vector (n x 1), stored as flat Vec.
    x: Vec<f64>,
    /// State covariance (n x n), stored row-major.
    p: Vec<f64>,
    /// State transition matrix (n x n).
    f: Vec<f64>,
    /// Observation matrix (m x n).
    h: Vec<f64>,
    /// Process noise covariance (n x n).
    q: Vec<f64>,
    /// Measurement noise covariance (m x m).
    r: Vec<f64>,
}

impl KalmanFilter {
    /// Create a new Kalman filter.
    ///
    /// - `state_dim`: dimension of state vector
    /// - `meas_dim`: dimension of measurement vector
    pub fn new(state_dim: usize, meas_dim: usize) -> Self {
        let n = state_dim.max(1);
        let m = meas_dim.max(1);
        Self {
            n,
            m,
            x: vec![0.0; n],
            p: identity(n),
            f: identity(n),
            h: {
                let mut h = vec![0.0; m * n];
                for i in 0..m.min(n) {
                    h[i * n + i] = 1.0;
                }
                h
            },
            q: scaled_identity(n, 0.01),
            r: scaled_identity(m, 1.0),
        }
    }

    /// Create a 1D filter for simple scalar tracking.
    ///
    /// - `initial_state`: initial value
    /// - `process_noise`: Q value
    /// - `measurement_noise`: R value
    pub fn new_1d(initial_state: f64, process_noise: f64, measurement_noise: f64) -> Self {
        let mut kf = Self::new(1, 1);
        kf.x[0] = initial_state;
        kf.q[0] = process_noise;
        kf.r[0] = measurement_noise;
        kf
    }

    /// Create a 2D phase-frequency tracker.
    ///
    /// State: [phase, frequency], Measurement: [phase].
    pub fn phase_frequency_tracker(dt: f64, phase_noise: f64, freq_noise: f64, meas_noise: f64) -> Self {
        let mut kf = Self::new(2, 1);
        // F = [[1, dt], [0, 1]]
        kf.f = vec![1.0, dt, 0.0, 1.0];
        // H = [1, 0]
        kf.h = vec![1.0, 0.0];
        // Q: process noise
        kf.q = vec![phase_noise, 0.0, 0.0, freq_noise];
        // R: measurement noise
        kf.r = vec![meas_noise];
        kf
    }

    /// Create a position-velocity tracker (1-axis).
    pub fn position_velocity_tracker(dt: f64, accel_noise: f64, meas_noise: f64) -> Self {
        let mut kf = Self::new(2, 1);
        kf.f = vec![1.0, dt, 0.0, 1.0];
        kf.h = vec![1.0, 0.0];
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;
        kf.q = vec![
            dt4 / 4.0 * accel_noise, dt3 / 2.0 * accel_noise,
            dt3 / 2.0 * accel_noise, dt2 * accel_noise,
        ];
        kf.r = vec![meas_noise];
        kf
    }

    /// Prediction step: x = F*x, P = F*P*F' + Q.
    pub fn predict(&mut self) {
        // x = F * x
        let new_x = mat_vec_mul(&self.f, &self.x, self.n, self.n);
        self.x = new_x;

        // P = F * P * F' + Q
        let fp = mat_mul(&self.f, &self.p, self.n, self.n, self.n);
        let ft = transpose(&self.f, self.n, self.n);
        let fpft = mat_mul(&fp, &ft, self.n, self.n, self.n);
        self.p = mat_add(&fpft, &self.q, self.n, self.n);
    }

    /// Update step with measurement vector.
    ///
    /// Returns the innovation (measurement residual).
    pub fn update(&mut self, measurement: &[f64]) -> Vec<f64> {
        let z = &measurement[..self.m];

        // Innovation: y = z - H * x
        let hx = mat_vec_mul(&self.h, &self.x, self.m, self.n);
        let y: Vec<f64> = z.iter().zip(hx.iter()).map(|(&a, &b)| a - b).collect();

        // Innovation covariance: S = H * P * H' + R
        let hp = mat_mul(&self.h, &self.p, self.m, self.n, self.n);
        let ht = transpose(&self.h, self.m, self.n);
        let hpht = mat_mul(&hp, &ht, self.m, self.n, self.m);
        let s = mat_add(&hpht, &self.r, self.m, self.m);

        // Kalman gain: K = P * H' * S^(-1)
        let pht = mat_mul(&self.p, &ht, self.n, self.n, self.m);
        let s_inv = mat_inv(&s, self.m);
        let k = mat_mul(&pht, &s_inv, self.n, self.m, self.m);

        // State update: x = x + K * y
        let ky = mat_vec_mul(&k, &y, self.n, self.m);
        for i in 0..self.n {
            self.x[i] += ky[i];
        }

        // Covariance update: P = (I - K*H) * P
        let kh = mat_mul(&k, &self.h, self.n, self.m, self.n);
        let i_kh = mat_sub(&identity(self.n), &kh, self.n, self.n);
        self.p = mat_mul(&i_kh, &self.p, self.n, self.n, self.n);

        y
    }

    /// Combined predict and update.
    pub fn predict_and_update(&mut self, measurement: &[f64]) -> Vec<f64> {
        self.predict();
        self.update(measurement)
    }

    // 1D convenience methods

    /// Predict step for 1D filter.
    pub fn predict_1d(&mut self) {
        self.predict();
    }

    /// Update step for 1D filter.
    pub fn update_1d(&mut self, measurement: f64) -> f64 {
        let y = self.update(&[measurement]);
        y[0]
    }

    /// Get 1D state value.
    pub fn state_1d(&self) -> f64 {
        self.x[0]
    }

    // General accessors

    /// Get state vector.
    pub fn state(&self) -> &[f64] {
        &self.x
    }

    /// Set state vector.
    pub fn set_state(&mut self, state: &[f64]) {
        let len = state.len().min(self.n);
        self.x[..len].copy_from_slice(&state[..len]);
    }

    /// Get state covariance diagonal (uncertainties).
    pub fn covariance_diagonal(&self) -> Vec<f64> {
        (0..self.n).map(|i| self.p[i * self.n + i]).collect()
    }

    /// Get state dimension.
    pub fn state_dim(&self) -> usize {
        self.n
    }

    /// Get measurement dimension.
    pub fn meas_dim(&self) -> usize {
        self.m
    }

    /// Set process noise covariance.
    pub fn set_process_noise(&mut self, q: &[f64]) {
        let len = q.len().min(self.n * self.n);
        self.q[..len].copy_from_slice(&q[..len]);
    }

    /// Set measurement noise covariance.
    pub fn set_measurement_noise(&mut self, r: &[f64]) {
        let len = r.len().min(self.m * self.m);
        self.r[..len].copy_from_slice(&r[..len]);
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.x.fill(0.0);
        self.p = identity(self.n);
    }
}

// Simple matrix operations for small dimensions

fn identity(n: usize) -> Vec<f64> {
    let mut m = vec![0.0; n * n];
    for i in 0..n {
        m[i * n + i] = 1.0;
    }
    m
}

fn scaled_identity(n: usize, scale: f64) -> Vec<f64> {
    let mut m = vec![0.0; n * n];
    for i in 0..n {
        m[i * n + i] = scale;
    }
    m
}

fn mat_vec_mul(a: &[f64], x: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let mut result = vec![0.0; rows];
    for i in 0..rows {
        for j in 0..cols {
            result[i] += a[i * cols + j] * x[j];
        }
    }
    result
}

fn mat_mul(a: &[f64], b: &[f64], m: usize, k: usize, n: usize) -> Vec<f64> {
    let mut result = vec![0.0; m * n];
    for i in 0..m {
        for j in 0..n {
            for l in 0..k {
                result[i * n + j] += a[i * k + l] * b[l * n + j];
            }
        }
    }
    result
}

fn transpose(a: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let mut result = vec![0.0; rows * cols];
    for i in 0..rows {
        for j in 0..cols {
            result[j * rows + i] = a[i * cols + j];
        }
    }
    result
}

fn mat_add(a: &[f64], b: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    a.iter()
        .zip(b.iter())
        .take(rows * cols)
        .map(|(&x, &y)| x + y)
        .collect()
}

fn mat_sub(a: &[f64], b: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    a.iter()
        .zip(b.iter())
        .take(rows * cols)
        .map(|(&x, &y)| x - y)
        .collect()
}

/// Invert a small matrix (1x1, 2x2, or general via Gauss-Jordan).
fn mat_inv(a: &[f64], n: usize) -> Vec<f64> {
    if n == 1 {
        return vec![1.0 / a[0].max(1e-30)];
    }
    if n == 2 {
        let det = a[0] * a[3] - a[1] * a[2];
        let inv_det = 1.0 / det.max(1e-30);
        return vec![a[3] * inv_det, -a[1] * inv_det, -a[2] * inv_det, a[0] * inv_det];
    }
    // Gauss-Jordan for larger matrices
    let mut aug = vec![0.0; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = a[i * n + j];
        }
        aug[i * 2 * n + n + i] = 1.0;
    }
    for i in 0..n {
        let pivot = aug[i * 2 * n + i];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for j in 0..2 * n {
            aug[i * 2 * n + j] /= pivot;
        }
        for k in 0..n {
            if k != i {
                let factor = aug[k * 2 * n + i];
                for j in 0..2 * n {
                    aug[k * 2 * n + j] -= factor * aug[i * 2 * n + j];
                }
            }
        }
    }
    let mut result = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = aug[i * 2 * n + n + j];
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_1d_constant() {
        let mut kf = KalmanFilter::new_1d(0.0, 0.01, 1.0);
        // Measure a constant value
        for _ in 0..50 {
            kf.update_1d(5.0);
            kf.predict_1d();
        }
        assert!((kf.state_1d() - 5.0).abs() < 0.5, "Should converge to 5.0");
    }

    #[test]
    fn test_1d_noisy() {
        let mut kf = KalmanFilter::new_1d(0.0, 0.01, 1.0);
        let measurements = [10.1, 9.8, 10.2, 10.0, 9.9, 10.1, 10.0, 9.95];
        for &m in &measurements {
            kf.update_1d(m);
            kf.predict_1d();
        }
        assert!(
            (kf.state_1d() - 10.0).abs() < 1.0,
            "Should track ~10.0, got {}",
            kf.state_1d()
        );
    }

    #[test]
    fn test_phase_frequency_tracker() {
        let mut kf = KalmanFilter::phase_frequency_tracker(0.001, 0.01, 0.001, 0.1);
        assert_eq!(kf.state_dim(), 2);
        assert_eq!(kf.meas_dim(), 1);

        // Simulate phase growing linearly (constant frequency)
        for i in 0..100 {
            let phase = 0.1 * i as f64; // 100 rad/s
            kf.predict_and_update(&[phase]);
        }
        let state = kf.state();
        assert!(state[1] > 0.0, "Should detect positive frequency");
    }

    #[test]
    fn test_position_velocity() {
        let dt = 0.1;
        let mut kf = KalmanFilter::position_velocity_tracker(dt, 0.1, 0.5);
        // Object moving at velocity 2.0
        for i in 0..50 {
            let pos = 2.0 * i as f64 * dt;
            kf.predict_and_update(&[pos]);
        }
        let state = kf.state();
        assert!(
            (state[1] - 2.0).abs() < 1.0,
            "Should estimate velocity ~2.0, got {}",
            state[1]
        );
    }

    #[test]
    fn test_innovation() {
        let mut kf = KalmanFilter::new_1d(0.0, 0.01, 1.0);
        let innovation = kf.update_1d(5.0);
        assert!((innovation - 5.0).abs() < 1e-6); // First innovation = measurement - 0
    }

    #[test]
    fn test_covariance_decreases() {
        let mut kf = KalmanFilter::new_1d(0.0, 0.01, 1.0);
        let cov_before = kf.covariance_diagonal()[0];
        for _ in 0..10 {
            kf.update_1d(1.0);
            kf.predict_1d();
        }
        let cov_after = kf.covariance_diagonal()[0];
        assert!(cov_after < cov_before, "Covariance should decrease with measurements");
    }

    #[test]
    fn test_set_state() {
        let mut kf = KalmanFilter::new(2, 1);
        kf.set_state(&[3.0, 4.0]);
        assert_eq!(kf.state(), &[3.0, 4.0]);
    }

    #[test]
    fn test_reset() {
        let mut kf = KalmanFilter::new_1d(5.0, 0.01, 1.0);
        kf.reset();
        assert_eq!(kf.state_1d(), 0.0);
    }

    #[test]
    fn test_mat_inv_2x2() {
        let a = vec![2.0, 1.0, 1.0, 3.0];
        let inv = mat_inv(&a, 2);
        // A * A^-1 should be identity
        let product = mat_mul(&a, &inv, 2, 2, 2);
        assert!((product[0] - 1.0).abs() < 1e-10);
        assert!((product[3] - 1.0).abs() < 1e-10);
        assert!(product[1].abs() < 1e-10);
    }

    #[test]
    fn test_dimensions() {
        let kf = KalmanFilter::new(3, 2);
        assert_eq!(kf.state_dim(), 3);
        assert_eq!(kf.meas_dim(), 2);
        assert_eq!(kf.state().len(), 3);
    }
}
