//! Lattice Filter
//!
//! Lattice and lattice-ladder filter structures using partial correlation
//! (PARCOR) / reflection coefficients. Includes Levinson-Durbin recursion
//! for computing optimal linear prediction coefficients from autocorrelation,
//! and Burg's method for direct estimation from data.
//!
//! Lattice filters are numerically superior to direct-form implementations
//! for high-order AR/ARMA models. They guarantee stability when all
//! |k_i| < 1 and allow order updates without recomputation.
//!
//! GNU Radio equivalent: `gr::filter::adaptive_fir_ccc` (partial),
//! but lattice structure is a distinct architecture.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::lattice_filter::{LatticePredictor, levinson_durbin};
//!
//! // Compute reflection coefficients from autocorrelation
//! let autocorr = vec![1.0, 0.8, 0.5, 0.2];
//! let (parcor, pred_coeffs, error) = levinson_durbin(&autocorr);
//! assert_eq!(parcor.len(), 3);
//!
//! // Use lattice predictor
//! let mut predictor = LatticePredictor::new(&parcor);
//! let output = predictor.predict(&[1.0, 0.5, -0.3, 0.7]);
//! assert_eq!(output.len(), 4);
//! ```

/// Result of Levinson-Durbin recursion.
///
/// Returns (reflection_coefficients, prediction_coefficients, prediction_error_power).
pub fn levinson_durbin(autocorrelation: &[f64]) -> (Vec<f64>, Vec<f64>, f64) {
    let m = autocorrelation.len() - 1; // Order
    if m == 0 || autocorrelation[0] <= 0.0 {
        return (vec![], vec![], autocorrelation.get(0).copied().unwrap_or(0.0));
    }

    let r = autocorrelation;
    let mut a = vec![0.0; m]; // Prediction coefficients
    let mut parcor = Vec::with_capacity(m); // Reflection coefficients
    let mut error = r[0];

    for i in 0..m {
        // Compute reflection coefficient k[i]
        let mut sum = r[i + 1];
        for j in 0..i {
            sum += a[j] * r[i - j];
        }
        let k = -sum / error;
        parcor.push(k);

        // Update prediction coefficients (Levinson recursion)
        let mut a_new = vec![0.0; m];
        a_new[i] = k;
        for j in 0..i {
            a_new[j] = a[j] + k * a[i - 1 - j];
        }
        a = a_new;

        // Update prediction error
        error *= 1.0 - k * k;
        if error <= 0.0 {
            break;
        }
    }

    // Trim prediction coefficients to order
    a.truncate(m);
    (parcor, a, error)
}

/// Burg's method: estimate reflection coefficients directly from data.
///
/// More robust than autocorrelation method for short data segments.
/// Returns (reflection_coefficients, prediction_error_power).
pub fn burg_method(signal: &[f64], order: usize) -> (Vec<f64>, f64) {
    let n = signal.len();
    if n <= order || order == 0 {
        return (vec![], signal.iter().map(|x| x * x).sum::<f64>() / n.max(1) as f64);
    }

    let mut parcor = Vec::with_capacity(order);
    let mut forward: Vec<f64> = signal.to_vec();
    let mut backward: Vec<f64> = signal.to_vec();
    let mut error: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;

    for m in 0..order {
        let len = n - m - 1;
        // Compute reflection coefficient
        let mut num = 0.0;
        let mut den = 0.0;
        for j in 0..len {
            num += forward[j + 1] * backward[j];
            den += forward[j + 1] * forward[j + 1] + backward[j] * backward[j];
        }

        let k = if den.abs() > 1e-30 {
            -2.0 * num / den
        } else {
            0.0
        };
        parcor.push(k);

        // Update forward and backward prediction errors
        let mut new_forward = vec![0.0; len];
        let mut new_backward = vec![0.0; len];
        for j in 0..len {
            new_forward[j] = forward[j + 1] + k * backward[j];
            new_backward[j] = backward[j] + k * forward[j + 1];
        }
        forward = new_forward;
        backward = new_backward;

        error *= 1.0 - k * k;
    }

    (parcor, error)
}

/// Convert reflection coefficients to direct-form prediction coefficients.
///
/// Uses step-up recursion (inverse of Levinson-Durbin decomposition).
pub fn parcor_to_direct(parcor: &[f64]) -> Vec<f64> {
    let m = parcor.len();
    if m == 0 {
        return vec![];
    }

    let mut a = vec![0.0; m];
    a[0] = parcor[0];

    for i in 1..m {
        let mut a_new = vec![0.0; m];
        a_new[i] = parcor[i];
        for j in 0..i {
            a_new[j] = a[j] + parcor[i] * a[i - 1 - j];
        }
        a = a_new;
    }

    a
}

/// Convert direct-form prediction coefficients to reflection coefficients.
///
/// Uses step-down recursion. Returns `None` if the filter is unstable
/// (any |k_i| >= 1).
pub fn direct_to_parcor(coeffs: &[f64]) -> Option<Vec<f64>> {
    let m = coeffs.len();
    if m == 0 {
        return Some(vec![]);
    }

    let mut a = coeffs.to_vec();
    let mut parcor = vec![0.0; m];

    for i in (0..m).rev() {
        let k = a[i];
        if k.abs() >= 1.0 {
            return None; // Unstable
        }
        parcor[i] = k;

        if i > 0 {
            let denom = 1.0 - k * k;
            let mut a_prev = vec![0.0; i];
            for j in 0..i {
                a_prev[j] = (a[j] - k * a[i - 1 - j]) / denom;
            }
            a = a_prev;
        }
    }

    Some(parcor)
}

/// Lattice predictor filter (FIR lattice / all-zero).
///
/// Implements forward and backward linear prediction using reflection
/// coefficients. Output is the prediction error (residual).
#[derive(Debug, Clone)]
pub struct LatticePredictor {
    /// Reflection coefficients
    parcor: Vec<f64>,
    /// Forward delay states
    forward: Vec<f64>,
    /// Backward delay states
    backward: Vec<f64>,
}

impl LatticePredictor {
    /// Create a new lattice predictor from reflection coefficients.
    pub fn new(parcor: &[f64]) -> Self {
        let order = parcor.len();
        Self {
            parcor: parcor.to_vec(),
            forward: vec![0.0; order + 1],
            backward: vec![0.0; order + 1],
        }
    }

    /// Process a single sample, returning the prediction error.
    pub fn process_sample(&mut self, input: f64) -> f64 {
        let m = self.parcor.len();
        self.forward[0] = input;
        self.backward[0] = input;

        for i in 0..m {
            let f_prev = self.forward[i];
            let b_prev = self.backward[i];
            self.forward[i + 1] = f_prev + self.parcor[i] * b_prev;
            self.backward[i + 1] = b_prev + self.parcor[i] * f_prev;
        }

        // Shift backward states for next sample
        let result = self.forward[m];
        for i in (1..=m).rev() {
            self.backward[i] = self.backward[i - 1];
        }

        result
    }

    /// Process a block of samples.
    pub fn predict(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_sample(s)).collect()
    }

    /// Get the filter order.
    pub fn order(&self) -> usize {
        self.parcor.len()
    }

    /// Check if the lattice filter is stable (all |k_i| < 1).
    pub fn is_stable(&self) -> bool {
        self.parcor.iter().all(|&k| k.abs() < 1.0)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.forward.fill(0.0);
        self.backward.fill(0.0);
    }
}

/// Lattice-ladder filter (IIR lattice / pole-zero / ARMA).
///
/// Combines a lattice all-pole section with a ladder (FIR) section
/// for implementing ARMA models.
#[derive(Debug, Clone)]
pub struct LatticeLadder {
    /// Reflection coefficients (poles)
    parcor: Vec<f64>,
    /// Ladder coefficients (zeros / MA part)
    ladder: Vec<f64>,
    /// Internal state
    state: Vec<f64>,
}

impl LatticeLadder {
    /// Create a new lattice-ladder filter.
    ///
    /// `parcor`: reflection coefficients (AR part)
    /// `ladder`: ladder tap coefficients (MA part), length = order + 1
    pub fn new(parcor: &[f64], ladder: &[f64]) -> Self {
        let order = parcor.len();
        Self {
            parcor: parcor.to_vec(),
            ladder: ladder.to_vec(),
            state: vec![0.0; order + 1],
        }
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, input: f64) -> f64 {
        let m = self.parcor.len();

        // Forward lattice pass (all-pole)
        let mut f = vec![0.0; m + 1];
        f[m] = input;

        // Step down through lattice (inverse)
        for i in (0..m).rev() {
            f[i] = f[i + 1] - self.parcor[i] * self.state[i];
        }

        // Compute output via ladder taps
        let mut output = 0.0;
        for i in 0..self.ladder.len().min(m + 1) {
            output += self.ladder[i] * f[i];
        }

        // Update state (backward prediction errors)
        for i in (1..=m).rev() {
            self.state[i] = self.state[i - 1] + self.parcor[i - 1] * f[i - 1];
        }
        self.state[0] = f[0];

        output
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_sample(s)).collect()
    }

    /// Get the filter order.
    pub fn order(&self) -> usize {
        self.parcor.len()
    }

    /// Check stability.
    pub fn is_stable(&self) -> bool {
        self.parcor.iter().all(|&k| k.abs() < 1.0)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.state.fill(0.0);
    }
}

/// Compute the power spectral density from reflection coefficients.
///
/// Returns PSD at `nfft` equally-spaced frequencies in [0, 0.5] (normalized).
pub fn lattice_psd(parcor: &[f64], error_power: f64, nfft: usize) -> Vec<f64> {
    let coeffs = parcor_to_direct(parcor);
    let mut psd = Vec::with_capacity(nfft);

    for k in 0..nfft {
        let f = k as f64 / (2.0 * nfft as f64);
        let omega = 2.0 * std::f64::consts::PI * f;

        // A(e^jω) = 1 + Σ a_i · e^{-jωi}
        let mut a_re = 1.0;
        let mut a_im = 0.0;
        for (i, &c) in coeffs.iter().enumerate() {
            let angle = -omega * (i + 1) as f64;
            a_re += c * angle.cos();
            a_im += c * angle.sin();
        }
        let mag_sq = a_re * a_re + a_im * a_im;
        psd.push(error_power / mag_sq.max(1e-30));
    }

    psd
}

/// Compute the Line Spectral Frequencies (LSF) from prediction coefficients.
///
/// LSFs are an alternative parameterization used in speech coding (e.g., LPC-10, CELP).
/// They are always in ascending order in [0, π] when the filter is stable.
pub fn lpc_to_lsf(coeffs: &[f64]) -> Vec<f64> {
    let m = coeffs.len();
    if m == 0 {
        return vec![];
    }

    // Form symmetric and antisymmetric polynomials
    // P(z) = A(z) + z^{-(m+1)} A(z^{-1})
    // Q(z) = A(z) - z^{-(m+1)} A(z^{-1})
    let mut p = vec![0.0; m + 2];
    let mut q = vec![0.0; m + 2];

    p[0] = 1.0;
    q[0] = 1.0;
    for i in 0..m {
        p[i + 1] = coeffs[i] + coeffs[m - 1 - i];
        q[i + 1] = coeffs[i] - coeffs[m - 1 - i];
    }
    p[m + 1] = 1.0;
    q[m + 1] = -1.0;

    // Find roots by evaluating on unit circle and finding zero crossings
    let n_eval = 1024;
    let mut lsf = Vec::new();

    for poly in [&p, &q] {
        let mut prev_val = eval_poly_on_unit_circle(poly, 0.0);
        for k in 1..=n_eval {
            let omega = std::f64::consts::PI * k as f64 / n_eval as f64;
            let val = eval_poly_on_unit_circle(poly, omega);
            if prev_val * val < 0.0 {
                // Zero crossing - refine with bisection
                let mut lo = std::f64::consts::PI * (k - 1) as f64 / n_eval as f64;
                let mut hi = omega;
                for _ in 0..20 {
                    let mid = (lo + hi) / 2.0;
                    let mid_val = eval_poly_on_unit_circle(poly, mid);
                    if prev_val * mid_val < 0.0 {
                        hi = mid;
                    } else {
                        lo = mid;
                        prev_val = mid_val;
                    }
                }
                lsf.push((lo + hi) / 2.0);
            }
            prev_val = val;
        }
    }

    lsf.sort_by(|a, b| a.partial_cmp(b).unwrap());
    lsf.truncate(m);
    lsf
}

fn eval_poly_on_unit_circle(coeffs: &[f64], omega: f64) -> f64 {
    let mut re = 0.0;
    for (i, &c) in coeffs.iter().enumerate() {
        re += c * (omega * i as f64).cos();
    }
    re
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_levinson_durbin_first_order() {
        // R = [1.0, 0.9] => k = -(-0.9/1.0) = 0.9... wait
        // k[0] = -r[1]/r[0] = -0.9
        let (parcor, coeffs, error) = levinson_durbin(&[1.0, 0.9]);
        assert_eq!(parcor.len(), 1);
        assert!((parcor[0] - (-0.9)).abs() < 1e-10);
        assert!((coeffs[0] - (-0.9)).abs() < 1e-10);
        assert!((error - (1.0 - 0.81)).abs() < 1e-10);
    }

    #[test]
    fn test_levinson_durbin_second_order() {
        let r = vec![1.0, 0.8, 0.5];
        let (parcor, coeffs, error) = levinson_durbin(&r);
        assert_eq!(parcor.len(), 2);
        assert_eq!(coeffs.len(), 2);
        assert!(error > 0.0);
        // Verify prediction error decreases with order
        let (_, _, error1) = levinson_durbin(&r[..2]);
        assert!(error < error1 + 1e-10);
    }

    #[test]
    fn test_burg_method() {
        // AR(1) process: x[n] = 0.9*x[n-1] + e[n]
        let mut signal = vec![0.0; 500];
        let mut state = 0.0_f64;
        // Deterministic pseudo-noise
        let mut seed = 42u64;
        for s in signal.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let noise = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
            state = 0.9 * state + noise;
            *s = state;
        }
        let (parcor, _error) = burg_method(&signal, 1);
        assert_eq!(parcor.len(), 1);
        // k[0] should be near -0.9 (negative of AR coefficient)
        assert!(
            (parcor[0] - (-0.9)).abs() < 0.15,
            "k[0]={}, expected -0.9",
            parcor[0]
        );
    }

    #[test]
    fn test_parcor_roundtrip() {
        let parcor = vec![-0.8, 0.5, -0.3];
        let direct = parcor_to_direct(&parcor);
        let recovered = direct_to_parcor(&direct).unwrap();
        for (a, b) in parcor.iter().zip(recovered.iter()) {
            assert!((a - b).abs() < 1e-10, "{a} != {b}");
        }
    }

    #[test]
    fn test_unstable_direct_to_parcor() {
        // Coefficients that lead to |k| >= 1
        let result = direct_to_parcor(&[2.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_lattice_predictor() {
        let parcor = vec![-0.5, 0.3];
        let mut pred = LatticePredictor::new(&parcor);
        assert_eq!(pred.order(), 2);
        assert!(pred.is_stable());
        let output = pred.predict(&[1.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(output.len(), 5);
        // Output should be finite and non-zero for impulse input
        assert!(output[0].is_finite());
        assert!(output[0].abs() > 0.0);
    }

    #[test]
    fn test_lattice_predictor_dc() {
        // With zero PARCOR, predictor is pass-through
        let parcor = vec![0.0, 0.0];
        let mut pred = LatticePredictor::new(&parcor);
        let output = pred.predict(&[1.0, 1.0, 1.0, 1.0]);
        for &o in &output {
            assert!((o - 1.0).abs() < 1e-10, "o={o}");
        }
    }

    #[test]
    fn test_lattice_ladder() {
        let parcor = vec![-0.5];
        let ladder = vec![1.0, 0.5]; // MA part
        let mut filter = LatticeLadder::new(&parcor, &ladder);
        assert_eq!(filter.order(), 1);
        let output = filter.process(&[1.0, 0.0, 0.0, 0.0]);
        assert_eq!(output.len(), 4);
    }

    #[test]
    fn test_lattice_psd() {
        let (parcor, _, error) = levinson_durbin(&[1.0, 0.9, 0.7]);
        let psd = lattice_psd(&parcor, error, 128);
        assert_eq!(psd.len(), 128);
        // PSD should be positive
        assert!(psd.iter().all(|&p| p > 0.0));
    }

    #[test]
    fn test_lpc_to_lsf() {
        let (_, coeffs, _) = levinson_durbin(&[1.0, 0.8, 0.5]);
        let lsf = lpc_to_lsf(&coeffs);
        // LSFs should be in ascending order
        for i in 1..lsf.len() {
            assert!(lsf[i] > lsf[i - 1], "LSFs not ascending: {:?}", lsf);
        }
    }

    #[test]
    fn test_reset() {
        let mut pred = LatticePredictor::new(&[-0.5, 0.3]);
        pred.predict(&[1.0, 2.0, 3.0]);
        pred.reset();
        // After reset, should behave like new
        let mut fresh = LatticePredictor::new(&[-0.5, 0.3]);
        let out1 = pred.predict(&[1.0]);
        let out2 = fresh.predict(&[1.0]);
        assert!((out1[0] - out2[0]).abs() < 1e-10);
    }

    #[test]
    fn test_burg_empty() {
        let (parcor, _) = burg_method(&[], 2);
        assert!(parcor.is_empty());
    }
}
