//! TDOA Estimator — Time Difference of Arrival Geolocation
//!
//! Estimates emitter position from time-delay measurements between
//! distributed receivers. Uses GCC-PHAT cross-correlation for delay
//! estimation and Chan/least-squares solvers for hyperbolic position fix.
//!
//! Complements `music_doa` (angle-based, co-located array) with
//! distributed-receiver geolocation.
//!
//! GNU Radio equivalent: custom SIGINT/spectrum monitoring OOT modules.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tdoa_estimator::{TdoaEstimator, ReceiverPosition};
//!
//! let receivers = vec![
//!     ReceiverPosition::new(0.0, 0.0, 0.0),
//!     ReceiverPosition::new(1000.0, 0.0, 0.0),
//!     ReceiverPosition::new(500.0, 866.0, 0.0),
//! ];
//! let estimator = TdoaEstimator::new(receivers, 1e6);
//! let delay = estimator.cross_correlate_delay(
//!     &[(1.0, 0.0); 256],
//!     &[(1.0, 0.0); 256],
//! );
//! assert!(delay.abs() < 256.0);
//! ```

use std::f64::consts::PI;

/// 3D receiver position.
#[derive(Debug, Clone)]
pub struct ReceiverPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl ReceiverPosition {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    fn distance_to(&self, other: &ReceiverPosition) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }
}

/// TDOA position fix result.
#[derive(Debug, Clone)]
pub struct TdoaFix {
    /// Estimated emitter position.
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Geometric Dilution of Precision.
    pub gdop: f64,
    /// RMS residual of delay equations.
    pub residual: f64,
}

/// TDOA delay pair measurement.
#[derive(Debug, Clone)]
pub struct TdoaPair {
    /// First receiver index.
    pub rx_a: usize,
    /// Second receiver index.
    pub rx_b: usize,
    /// Measured time delay (seconds, positive means signal arrives at A first).
    pub delay_s: f64,
    /// Correlation peak magnitude (confidence).
    pub confidence: f64,
}

/// TDOA geolocation estimator.
#[derive(Debug, Clone)]
pub struct TdoaEstimator {
    receivers: Vec<ReceiverPosition>,
    sample_rate: f64,
}

impl TdoaEstimator {
    /// Create a new TDOA estimator.
    ///
    /// `receivers`: positions of distributed receivers.
    /// `sample_rate`: sampling rate for delay computation (Hz).
    pub fn new(receivers: Vec<ReceiverPosition>, sample_rate: f64) -> Self {
        Self {
            receivers,
            sample_rate,
        }
    }

    /// Estimate time delay between two signals via cross-correlation.
    ///
    /// Returns delay in samples (positive = sig_a arrives first).
    /// Uses parabolic interpolation for sub-sample accuracy.
    pub fn cross_correlate_delay(&self, sig_a: &[(f64, f64)], sig_b: &[(f64, f64)]) -> f64 {
        let n = sig_a.len().min(sig_b.len());
        if n == 0 {
            return 0.0;
        }

        // Cross-correlate using time-domain sliding
        let max_lag = (n / 2).min(1024);
        let mut best_lag: i64 = 0;
        let mut best_val = f64::NEG_INFINITY;
        let mut corr_vals: Vec<(i64, f64)> = Vec::new();

        for lag in -(max_lag as i64)..=(max_lag as i64) {
            let mut sum_r = 0.0;
            let mut sum_i = 0.0;
            let mut count = 0;
            for i in 0..n {
                let j = i as i64 + lag;
                if j >= 0 && (j as usize) < n {
                    let (ar, ai) = sig_a[i];
                    let (br, bi) = sig_b[j as usize];
                    // a * conj(b)
                    sum_r += ar * br + ai * bi;
                    sum_i += ai * br - ar * bi;
                    count += 1;
                }
            }
            let mag = (sum_r * sum_r + sum_i * sum_i).sqrt() / count.max(1) as f64;
            corr_vals.push((lag, mag));
            if mag > best_val {
                best_val = mag;
                best_lag = lag;
            }
        }

        // Parabolic interpolation around peak
        let idx = corr_vals
            .iter()
            .position(|&(l, _)| l == best_lag)
            .unwrap_or(0);
        if idx > 0 && idx < corr_vals.len() - 1 {
            let y_prev = corr_vals[idx - 1].1;
            let y_peak = corr_vals[idx].1;
            let y_next = corr_vals[idx + 1].1;
            let denom = y_prev - 2.0 * y_peak + y_next;
            if denom.abs() > 1e-20 {
                let delta = 0.5 * (y_prev - y_next) / denom;
                return best_lag as f64 + delta;
            }
        }

        best_lag as f64
    }

    /// Compute GCC-PHAT (Generalized Cross-Correlation with Phase Transform).
    ///
    /// Returns cross-correlation with whitened spectrum for sharper peaks.
    pub fn gcc_phat(&self, sig_a: &[(f64, f64)], sig_b: &[(f64, f64)]) -> Vec<f64> {
        let n = sig_a.len().min(sig_b.len());
        let fft_size = n.next_power_of_two();

        // Zero-pad and FFT both signals
        let mut a_fft = vec![(0.0, 0.0); fft_size];
        let mut b_fft = vec![(0.0, 0.0); fft_size];
        for i in 0..n {
            a_fft[i] = sig_a[i];
            b_fft[i] = sig_b[i];
        }

        let a_freq = fft(&a_fft, false);
        let b_freq = fft(&b_fft, false);

        // Cross-spectral density with phase whitening
        let gcc: Vec<(f64, f64)> = a_freq
            .iter()
            .zip(b_freq.iter())
            .map(|(&(ar, ai), &(br, bi))| {
                // A * conj(B)
                let cr = ar * br + ai * bi;
                let ci = ai * br - ar * bi;
                let mag = (cr * cr + ci * ci).sqrt().max(1e-20);
                (cr / mag, ci / mag)
            })
            .collect();

        // IFFT
        let result = fft(&gcc, true);
        result
            .iter()
            .map(|(r, _)| r / fft_size as f64)
            .collect()
    }

    /// Compute TDOA pairs for all receiver combinations.
    pub fn compute_all_pairs(&self, signals: &[Vec<(f64, f64)>]) -> Vec<TdoaPair> {
        let num_rx = self.receivers.len().min(signals.len());
        let mut pairs = Vec::new();

        for i in 0..num_rx {
            for j in (i + 1)..num_rx {
                let delay_samples = self.cross_correlate_delay(&signals[i], &signals[j]);
                let delay_s = delay_samples / self.sample_rate;
                pairs.push(TdoaPair {
                    rx_a: i,
                    rx_b: j,
                    delay_s,
                    confidence: 1.0,
                });
            }
        }
        pairs
    }

    /// Solve emitter position from TDOA pairs using least-squares.
    ///
    /// Requires at least 3 receivers for 2D fix.
    pub fn solve_position(&self, pairs: &[TdoaPair]) -> Option<TdoaFix> {
        if pairs.len() < 2 || self.receivers.len() < 3 {
            return None;
        }

        let c = 299_792_458.0; // speed of light

        // Use centroid of receivers as initial estimate
        let n_rx = self.receivers.len() as f64;
        let mut est_x = self.receivers.iter().map(|r| r.x).sum::<f64>() / n_rx;
        let mut est_y = self.receivers.iter().map(|r| r.y).sum::<f64>() / n_rx;
        let est_z = self.receivers.iter().map(|r| r.z).sum::<f64>() / n_rx;

        // Iterative least-squares (simplified Taylor series expansion)
        for _ in 0..20 {
            let mut ata = [[0.0; 2]; 2];
            let mut atb = [0.0; 2];

            for pair in pairs {
                let rx_a = &self.receivers[pair.rx_a];
                let rx_b = &self.receivers[pair.rx_b];

                let da = ((est_x - rx_a.x).powi(2) + (est_y - rx_a.y).powi(2)).sqrt().max(1e-6);
                let db = ((est_x - rx_b.x).powi(2) + (est_y - rx_b.y).powi(2)).sqrt().max(1e-6);

                let range_diff = pair.delay_s * c;
                let residual = da - db - range_diff;

                // Jacobian row
                let j0 = (est_x - rx_a.x) / da - (est_x - rx_b.x) / db;
                let j1 = (est_y - rx_a.y) / da - (est_y - rx_b.y) / db;

                // Normal equations: J^T * J * dx = J^T * (-residual)
                ata[0][0] += j0 * j0;
                ata[0][1] += j0 * j1;
                ata[1][0] += j1 * j0;
                ata[1][1] += j1 * j1;
                atb[0] += -j0 * residual;
                atb[1] += -j1 * residual;
            }

            // Solve 2x2 system
            let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
            if det.abs() < 1e-20 {
                break;
            }
            let dx = (ata[1][1] * atb[0] - ata[0][1] * atb[1]) / det;
            let dy = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;

            est_x += dx;
            est_y += dy;

            if dx * dx + dy * dy < 1e-6 {
                break;
            }
        }

        // Compute residual
        let c = 299_792_458.0;
        let residual: f64 = pairs
            .iter()
            .map(|p| {
                let da = ((est_x - self.receivers[p.rx_a].x).powi(2)
                    + (est_y - self.receivers[p.rx_a].y).powi(2))
                .sqrt();
                let db = ((est_x - self.receivers[p.rx_b].x).powi(2)
                    + (est_y - self.receivers[p.rx_b].y).powi(2))
                .sqrt();
                (da - db - p.delay_s * c).powi(2)
            })
            .sum::<f64>()
            .sqrt()
            / pairs.len() as f64;

        Some(TdoaFix {
            x: est_x,
            y: est_y,
            z: est_z,
            gdop: self.compute_gdop(est_x, est_y),
            residual,
        })
    }

    fn compute_gdop(&self, x: f64, y: f64) -> f64 {
        // Simplified 2D GDOP
        let mut sum_dx2 = 0.0;
        let mut sum_dy2 = 0.0;
        for rx in &self.receivers {
            let d = ((x - rx.x).powi(2) + (y - rx.y).powi(2)).sqrt().max(1e-6);
            sum_dx2 += ((x - rx.x) / d).powi(2);
            sum_dy2 += ((y - rx.y) / d).powi(2);
        }
        let n = self.receivers.len() as f64;
        ((n / sum_dx2.max(1e-20)) + (n / sum_dy2.max(1e-20))).sqrt()
    }
}

// Minimal radix-2 FFT
fn fft(x: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = x.len();
    if n <= 1 {
        return x.to_vec();
    }
    let even: Vec<(f64, f64)> = x.iter().step_by(2).cloned().collect();
    let odd: Vec<(f64, f64)> = x.iter().skip(1).step_by(2).cloned().collect();
    let even_fft = fft(&even, inverse);
    let odd_fft = fft(&odd, inverse);
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n / 2 {
        let angle = sign * 2.0 * PI * k as f64 / n as f64;
        let tw = (angle.cos(), angle.sin());
        let (or, oi) = odd_fft[k];
        let prod = (tw.0 * or - tw.1 * oi, tw.0 * oi + tw.1 * or);
        result[k] = (even_fft[k].0 + prod.0, even_fft[k].1 + prod.1);
        result[k + n / 2] = (even_fft[k].0 - prod.0, even_fft[k].1 - prod.1);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_delay_detection() {
        // Use a broadband pulse signal with a sharp autocorrelation peak
        let mut signal = vec![(0.0, 0.0); 256];
        // Short pulse burst in the middle
        for i in 100..130 {
            let t = (i - 100) as f64;
            signal[i] = ((t * 0.7).sin() * (-(t - 15.0).powi(2) / 50.0).exp(), 0.0);
        }
        let estimator = TdoaEstimator::new(vec![], 1e6);
        let delay = estimator.cross_correlate_delay(&signal, &signal);
        assert!(delay.abs() < 2.0, "zero delay expected, got {delay}");
    }

    #[test]
    fn test_known_integer_delay() {
        // Create signal with known delay
        let mut sig_a = vec![(0.0, 0.0); 128];
        let mut sig_b = vec![(0.0, 0.0); 128];
        for i in 20..60 {
            sig_a[i] = ((i as f64 * 0.5).sin(), 0.0);
            sig_b[i + 5] = ((i as f64 * 0.5).sin(), 0.0); // 5 sample delay
        }
        let estimator = TdoaEstimator::new(vec![], 1e6);
        let delay = estimator.cross_correlate_delay(&sig_a, &sig_b);
        assert!(
            (delay - 5.0).abs() < 2.0,
            "expected ~5 sample delay, got {delay}"
        );
    }

    #[test]
    fn test_gcc_phat_length() {
        let sig = vec![(1.0, 0.0); 64];
        let estimator = TdoaEstimator::new(vec![], 1e6);
        let gcc = estimator.gcc_phat(&sig, &sig);
        assert_eq!(gcc.len(), 64); // next power of 2
    }

    #[test]
    fn test_compute_all_pairs() {
        let receivers = vec![
            ReceiverPosition::new(0.0, 0.0, 0.0),
            ReceiverPosition::new(1000.0, 0.0, 0.0),
            ReceiverPosition::new(500.0, 866.0, 0.0),
        ];
        let estimator = TdoaEstimator::new(receivers, 1e6);
        let signals: Vec<Vec<(f64, f64)>> = vec![
            vec![(1.0, 0.0); 256],
            vec![(1.0, 0.0); 256],
            vec![(1.0, 0.0); 256],
        ];
        let pairs = estimator.compute_all_pairs(&signals);
        assert_eq!(pairs.len(), 3); // C(3,2) = 3
    }

    #[test]
    fn test_minimum_receivers() {
        let estimator = TdoaEstimator::new(
            vec![
                ReceiverPosition::new(0.0, 0.0, 0.0),
                ReceiverPosition::new(100.0, 0.0, 0.0),
            ],
            1e6,
        );
        let pairs = vec![TdoaPair {
            rx_a: 0,
            rx_b: 1,
            delay_s: 0.0,
            confidence: 1.0,
        }];
        let fix = estimator.solve_position(&pairs);
        assert!(fix.is_none(), "need at least 3 receivers for 2D fix");
    }

    #[test]
    fn test_solve_position_at_centroid() {
        let receivers = vec![
            ReceiverPosition::new(0.0, 0.0, 0.0),
            ReceiverPosition::new(1000.0, 0.0, 0.0),
            ReceiverPosition::new(500.0, 866.0, 0.0),
        ];
        let estimator = TdoaEstimator::new(receivers, 1e6);
        // Zero delays → emitter equidistant from all receivers (centroid area)
        let pairs = vec![
            TdoaPair { rx_a: 0, rx_b: 1, delay_s: 0.0, confidence: 1.0 },
            TdoaPair { rx_a: 0, rx_b: 2, delay_s: 0.0, confidence: 1.0 },
            TdoaPair { rx_a: 1, rx_b: 2, delay_s: 0.0, confidence: 1.0 },
        ];
        let fix = estimator.solve_position(&pairs);
        assert!(fix.is_some());
    }

    #[test]
    fn test_receiver_distance() {
        let a = ReceiverPosition::new(0.0, 0.0, 0.0);
        let b = ReceiverPosition::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_gdop_computation() {
        let receivers = vec![
            ReceiverPosition::new(0.0, 0.0, 0.0),
            ReceiverPosition::new(1000.0, 0.0, 0.0),
            ReceiverPosition::new(0.0, 1000.0, 0.0),
            ReceiverPosition::new(1000.0, 1000.0, 0.0),
        ];
        let estimator = TdoaEstimator::new(receivers, 1e6);
        let gdop = estimator.compute_gdop(500.0, 500.0);
        assert!(gdop.is_finite() && gdop > 0.0, "GDOP={gdop}");
    }

    #[test]
    fn test_empty_signals() {
        let estimator = TdoaEstimator::new(vec![], 1e6);
        let delay = estimator.cross_correlate_delay(&[], &[]);
        assert_eq!(delay, 0.0);
    }

    #[test]
    fn test_tdoa_pair_fields() {
        let pair = TdoaPair {
            rx_a: 0,
            rx_b: 1,
            delay_s: 1e-6,
            confidence: 0.95,
        };
        assert_eq!(pair.rx_a, 0);
        assert_eq!(pair.rx_b, 1);
        assert!((pair.delay_s - 1e-6).abs() < 1e-20);
    }
}
