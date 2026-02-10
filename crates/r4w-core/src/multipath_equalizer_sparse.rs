//! Sparse tap adaptive equalizer for channels with long delay spreads.
//!
//! Designed for HF, underwater acoustic, and urban multipath channels where
//! the channel impulse response is long but sparse — only a few taps carry
//! significant energy. By adapting only active taps, computational cost is
//! O(K) instead of O(L), where K << L.
//!
//! # Example
//!
//! ```
//! use r4w_core::multipath_equalizer_sparse::{SparseEqualizer, SparseConfig, AdaptationMode};
//!
//! // Configure a sparse equalizer
//! let config = SparseConfig {
//!     max_taps: 64,
//!     activation_threshold: 0.1,
//!     deactivation_threshold: 0.01,
//!     step_size: 0.01,
//!     max_delay_samples: 256,
//!     adaptation_mode: AdaptationMode::Nlms,
//!     rls_forgetting_factor: 0.99,
//!     prune_interval: 100,
//! };
//!
//! let mut eq = SparseEqualizer::new(config);
//!
//! // Suppose we have a simple two-tap channel: h = [1.0, 0, 0, 0.5]
//! // We create a short training sequence and the received signal.
//! let pilot_tx: Vec<(f64, f64)> = vec![
//!     (1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0),
//!     (1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0),
//! ];
//!
//! // Received = convolution of pilot_tx with channel [1.0, 0, 0, 0.5]
//! let pilot_rx: Vec<(f64, f64)> = vec![
//!     (1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.5, 0.0),
//!     (1.0, 0.0), (0.0, 0.0), (1.5, 0.0), (0.5, 0.0),
//! ];
//!
//! // Estimate channel taps from pilots
//! let taps = eq.estimate_channel(&pilot_tx, &pilot_rx);
//! assert!(taps.len() > 0);
//!
//! // Equalize using training data
//! let output = eq.equalize(&pilot_rx, &pilot_tx);
//! assert_eq!(output.len(), pilot_rx.len());
//! ```

/// Complex multiplication: (a+bi)(c+di) = (ac-bd) + (ad+bc)i
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex addition
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Squared magnitude |z|^2
#[inline]
fn mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex value by a real scalar
#[inline]
fn cscale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Adaptation algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdaptationMode {
    /// Least Mean Squares — fixed step size.
    Lms,
    /// Normalized LMS — step size normalized by input power.
    Nlms,
    /// Recursive Least Squares — exponentially weighted LS.
    Rls,
}

/// Configuration for the sparse equalizer.
#[derive(Debug, Clone)]
pub struct SparseConfig {
    /// Maximum number of active taps allowed.
    pub max_taps: usize,
    /// Normalized correlation power (0..1) above which a tap is activated.
    pub activation_threshold: f64,
    /// Normalized correlation power below which a tap is pruned.
    pub deactivation_threshold: f64,
    /// LMS / NLMS step size (mu). For RLS this is ignored.
    pub step_size: f64,
    /// Maximum delay span in samples that the equalizer can represent.
    pub max_delay_samples: usize,
    /// Which adaptation algorithm to use.
    pub adaptation_mode: AdaptationMode,
    /// RLS forgetting factor lambda (typically 0.95–0.999). Only used in RLS mode.
    pub rls_forgetting_factor: f64,
    /// How often (in samples processed) to run the pruning pass.
    pub prune_interval: usize,
}

impl Default for SparseConfig {
    fn default() -> Self {
        Self {
            max_taps: 32,
            activation_threshold: 0.1,
            deactivation_threshold: 0.01,
            step_size: 0.01,
            max_delay_samples: 256,
            adaptation_mode: AdaptationMode::Nlms,
            rls_forgetting_factor: 0.99,
            prune_interval: 100,
        }
    }
}

/// Information about a single equalizer tap.
#[derive(Debug, Clone)]
pub struct TapInfo {
    /// Sample delay index (0 = direct path).
    pub delay_index: usize,
    /// Complex tap coefficient.
    pub coefficient: (f64, f64),
    /// Instantaneous tap power in dB (10 * log10(|coeff|^2)).
    pub power_db: f64,
    /// Whether this tap is currently active in the filter.
    pub is_active: bool,
}

/// Sparse-tap adaptive equalizer.
///
/// Maintains a set of taps at specific delay indices rather than a dense
/// delay line, making it efficient for long but sparse channels.
pub struct SparseEqualizer {
    config: SparseConfig,
    /// Active and inactive taps, keyed by delay index.
    taps: Vec<TapInfo>,
    /// Running count of samples processed (for prune scheduling).
    sample_count: usize,
    /// Per-tap inverse correlation matrix diagonal (for RLS). Indexed by
    /// position in `taps` vec.
    rls_p: Vec<f64>,
}

impl SparseEqualizer {
    /// Create a new sparse equalizer with the given configuration.
    pub fn new(config: SparseConfig) -> Self {
        Self {
            taps: Vec::new(),
            rls_p: Vec::new(),
            sample_count: 0,
            config,
        }
    }

    /// Equalize `input` using a known `desired` reference (training / decision-directed).
    ///
    /// Returns the equalizer output, one sample per input sample.
    pub fn equalize(
        &mut self,
        input: &[(f64, f64)],
        desired: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = input.len().min(desired.len());
        let mut output = Vec::with_capacity(n);

        for i in 0..n {
            let y = self.filter_sample(input, i);
            output.push(y);

            let error = csub(desired[i], y);
            self.adapt(input, i, error);

            self.sample_count += 1;
            if self.config.prune_interval > 0
                && self.sample_count % self.config.prune_interval == 0
            {
                self.prune_taps();
            }
        }
        output
    }

    /// Blind equalization using the Constant Modulus Algorithm (CMA).
    ///
    /// Assumes the transmitted signal has constant envelope (|x| = 1).
    /// Returns the equalizer output.
    pub fn equalize_blind(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = input.len();
        let mut output = Vec::with_capacity(n);

        // If no taps yet, seed a single zero-delay tap so we have something.
        if self.taps.is_empty() {
            self.add_tap(0, (1.0, 0.0));
        }

        for i in 0..n {
            let y = self.filter_sample(input, i);
            output.push(y);

            // CMA error: e = y * (1 - |y|^2)
            let r2 = 1.0; // target radius squared
            let y_mag2 = mag2(y);
            let cma_error = cscale(y, r2 - y_mag2);

            self.adapt(input, i, cma_error);

            self.sample_count += 1;
            if self.config.prune_interval > 0
                && self.sample_count % self.config.prune_interval == 0
            {
                self.prune_taps();
            }
        }
        output
    }

    /// Estimate the channel impulse response from pilot symbols.
    ///
    /// Uses cross-correlation between `pilot_tx` and `pilot_rx` to identify
    /// delay indices with significant energy, then activates taps at those
    /// positions.
    pub fn estimate_channel(
        &mut self,
        pilot_tx: &[(f64, f64)],
        pilot_rx: &[(f64, f64)],
    ) -> Vec<TapInfo> {
        let n = pilot_tx.len().min(pilot_rx.len());
        if n == 0 {
            return Vec::new();
        }

        let max_lag = self.config.max_delay_samples.min(n);

        // Cross-correlate: R(k) = sum_i rx[i] * conj(tx[i - k])
        let mut corr: Vec<(f64, f64)> = Vec::with_capacity(max_lag);
        for k in 0..max_lag {
            let mut acc = (0.0, 0.0);
            for i in k..n {
                acc = cadd(acc, cmul(pilot_rx[i], conj(pilot_tx[i - k])));
            }
            corr.push(acc);
        }

        // Find peak magnitude for normalization
        let peak_power = corr.iter().map(|c| mag2(*c)).fold(0.0_f64, f64::max);
        if peak_power < 1e-30 {
            return Vec::new();
        }

        // Activate taps where normalized power exceeds threshold
        self.taps.clear();
        self.rls_p.clear();

        let mut activated: Vec<TapInfo> = Vec::new();
        for (k, &c) in corr.iter().enumerate() {
            let norm_power = mag2(c) / peak_power;
            if norm_power >= self.config.activation_threshold
                && activated.len() < self.config.max_taps
            {
                let pdb = if mag2(c) > 0.0 {
                    10.0 * mag2(c).log10()
                } else {
                    f64::NEG_INFINITY
                };
                let info = TapInfo {
                    delay_index: k,
                    coefficient: c,
                    power_db: pdb,
                    is_active: true,
                };
                activated.push(info);
            }
        }

        // Normalize tap coefficients so that the strongest path has unit gain
        if let Some(max_mag2) = activated.iter().map(|t| mag2(t.coefficient)).reduce(f64::max) {
            if max_mag2 > 1e-30 {
                let scale = 1.0 / max_mag2.sqrt();
                for tap in &mut activated {
                    tap.coefficient = cscale(tap.coefficient, scale);
                    let m2 = mag2(tap.coefficient);
                    tap.power_db = if m2 > 0.0 {
                        10.0 * m2.log10()
                    } else {
                        f64::NEG_INFINITY
                    };
                }
            }
        }

        self.taps = activated.clone();
        self.rls_p = vec![1.0 / self.config.step_size.max(1e-10); self.taps.len()];

        activated
    }

    /// Return references to all currently active taps.
    pub fn active_taps(&self) -> Vec<&TapInfo> {
        self.taps.iter().filter(|t| t.is_active).collect()
    }

    /// Number of currently active taps.
    pub fn num_active_taps(&self) -> usize {
        self.taps.iter().filter(|t| t.is_active).count()
    }

    /// Deactivate taps whose power falls below the deactivation threshold.
    pub fn prune_taps(&mut self) {
        if self.taps.is_empty() {
            return;
        }

        // Find maximum power among active taps for relative comparison
        let max_power = self
            .taps
            .iter()
            .filter(|t| t.is_active)
            .map(|t| mag2(t.coefficient))
            .fold(0.0_f64, f64::max);

        if max_power < 1e-30 {
            return;
        }

        for tap in &mut self.taps {
            if tap.is_active {
                let relative_power = mag2(tap.coefficient) / max_power;
                if relative_power < self.config.deactivation_threshold {
                    tap.is_active = false;
                }
            }
        }
    }

    /// Fraction of taps that are inactive: `1 - (active / total)`.
    ///
    /// Returns 0.0 if there are no taps.
    pub fn tap_sparsity(&self) -> f64 {
        if self.taps.is_empty() {
            return 0.0;
        }
        let active = self.num_active_taps() as f64;
        let total = self.taps.len() as f64;
        1.0 - active / total
    }

    // ── internal helpers ──────────────────────────────────────────────

    /// Compute the filter output at sample index `i` using active taps.
    fn filter_sample(&self, input: &[(f64, f64)], i: usize) -> (f64, f64) {
        let mut y = (0.0, 0.0);
        for tap in &self.taps {
            if !tap.is_active {
                continue;
            }
            if i >= tap.delay_index {
                let idx = i - tap.delay_index;
                if idx < input.len() {
                    y = cadd(y, cmul(tap.coefficient, input[idx]));
                }
            }
        }
        y
    }

    /// Adapt active tap coefficients based on the error signal.
    fn adapt(&mut self, input: &[(f64, f64)], i: usize, error: (f64, f64)) {
        match self.config.adaptation_mode {
            AdaptationMode::Lms => self.adapt_lms(input, i, error),
            AdaptationMode::Nlms => self.adapt_nlms(input, i, error),
            AdaptationMode::Rls => self.adapt_rls(input, i, error),
        }
    }

    fn adapt_lms(&mut self, input: &[(f64, f64)], i: usize, error: (f64, f64)) {
        let mu = self.config.step_size;
        for tap in &mut self.taps {
            if !tap.is_active {
                continue;
            }
            if i >= tap.delay_index {
                let idx = i - tap.delay_index;
                if idx < input.len() {
                    // w += mu * error * conj(x[i - delay])
                    let update = cscale(cmul(error, conj(input[idx])), mu);
                    tap.coefficient = cadd(tap.coefficient, update);
                    tap.power_db = power_db(tap.coefficient);
                }
            }
        }
    }

    fn adapt_nlms(&mut self, input: &[(f64, f64)], i: usize, error: (f64, f64)) {
        let mu = self.config.step_size;
        let eps = 1e-10;

        // Compute input power across active taps
        let mut input_power = 0.0;
        for tap in &self.taps {
            if !tap.is_active {
                continue;
            }
            if i >= tap.delay_index {
                let idx = i - tap.delay_index;
                if idx < input.len() {
                    input_power += mag2(input[idx]);
                }
            }
        }

        let norm_mu = mu / (input_power + eps);

        for tap in &mut self.taps {
            if !tap.is_active {
                continue;
            }
            if i >= tap.delay_index {
                let idx = i - tap.delay_index;
                if idx < input.len() {
                    let update = cscale(cmul(error, conj(input[idx])), norm_mu);
                    tap.coefficient = cadd(tap.coefficient, update);
                    tap.power_db = power_db(tap.coefficient);
                }
            }
        }
    }

    fn adapt_rls(&mut self, input: &[(f64, f64)], i: usize, error: (f64, f64)) {
        let lambda = self.config.rls_forgetting_factor;
        let inv_lambda = 1.0 / lambda;
        let eps = 1e-10;

        // Ensure rls_p has the right size
        while self.rls_p.len() < self.taps.len() {
            self.rls_p.push(1.0 / self.config.step_size.max(1e-10));
        }

        for (j, tap) in self.taps.iter_mut().enumerate() {
            if !tap.is_active {
                continue;
            }
            if i >= tap.delay_index {
                let idx = i - tap.delay_index;
                if idx < input.len() {
                    let x = input[idx];
                    let p = self.rls_p[j];

                    // Simplified scalar RLS (diagonal approximation):
                    // k = p * x / (lambda + p * |x|^2)
                    let denom = lambda + p * mag2(x);
                    let gain = p / (denom + eps);

                    // w += gain * error * conj(x)
                    let update = cscale(cmul(error, conj(x)), gain);
                    tap.coefficient = cadd(tap.coefficient, update);
                    tap.power_db = power_db(tap.coefficient);

                    // P = (1/lambda) * (P - gain * |x|^2 * P)
                    self.rls_p[j] = inv_lambda * (p - gain * mag2(x) * p);
                    if self.rls_p[j] < eps {
                        self.rls_p[j] = eps;
                    }
                }
            }
        }
    }

    /// Add a new tap at the given delay index.
    fn add_tap(&mut self, delay_index: usize, coefficient: (f64, f64)) {
        let info = TapInfo {
            delay_index,
            coefficient,
            power_db: power_db(coefficient),
            is_active: true,
        };
        self.taps.push(info);
        self.rls_p
            .push(1.0 / self.config.step_size.max(1e-10));
    }
}

/// Compute power in dB from a complex coefficient.
fn power_db(c: (f64, f64)) -> f64 {
    let m2 = mag2(c);
    if m2 > 0.0 {
        10.0 * m2.log10()
    } else {
        f64::NEG_INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SparseConfig {
        SparseConfig {
            max_taps: 16,
            activation_threshold: 0.05,
            deactivation_threshold: 0.005,
            step_size: 0.05,
            max_delay_samples: 64,
            adaptation_mode: AdaptationMode::Nlms,
            rls_forgetting_factor: 0.99,
            prune_interval: 0, // disabled for most tests
        }
    }

    /// Convolve signal with a sparse channel impulse response.
    fn convolve(signal: &[(f64, f64)], channel: &[(usize, (f64, f64))]) -> Vec<(f64, f64)> {
        let max_delay = channel.iter().map(|(d, _)| *d).max().unwrap_or(0);
        let out_len = signal.len() + max_delay;
        let mut out = vec![(0.0, 0.0); out_len];
        for (delay, coeff) in channel {
            for (i, &s) in signal.iter().enumerate() {
                out[i + delay] = cadd(out[i + delay], cmul(*coeff, s));
            }
        }
        out.truncate(signal.len());
        out
    }

    #[test]
    fn test_new_creates_empty_equalizer() {
        let eq = SparseEqualizer::new(default_config());
        assert_eq!(eq.num_active_taps(), 0);
        assert!(eq.active_taps().is_empty());
        assert_eq!(eq.tap_sparsity(), 0.0);
    }

    #[test]
    fn test_default_config() {
        let cfg = SparseConfig::default();
        assert_eq!(cfg.max_taps, 32);
        assert_eq!(cfg.max_delay_samples, 256);
        assert!(cfg.step_size > 0.0);
        assert!(cfg.activation_threshold > cfg.deactivation_threshold);
    }

    #[test]
    fn test_adaptation_mode_eq() {
        assert_eq!(AdaptationMode::Lms, AdaptationMode::Lms);
        assert_ne!(AdaptationMode::Lms, AdaptationMode::Nlms);
        assert_ne!(AdaptationMode::Nlms, AdaptationMode::Rls);
    }

    #[test]
    fn test_estimate_channel_identity() {
        let mut eq = SparseEqualizer::new(default_config());
        // Identity channel: pilot_rx == pilot_tx
        let pilot: Vec<(f64, f64)> = (0..32)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();
        let taps = eq.estimate_channel(&pilot, &pilot);

        // Should find at least one tap at delay 0
        assert!(!taps.is_empty());
        assert!(taps[0].delay_index == 0);
        assert!(taps[0].is_active);
    }

    #[test]
    fn test_estimate_channel_single_delay() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            activation_threshold: 0.05,
            max_delay_samples: 32,
            ..default_config()
        });

        let pilot_tx: Vec<(f64, f64)> = (0..64)
            .map(|i| ((i as f64 * 0.3).sin(), (i as f64 * 0.3).cos()))
            .collect();

        // Channel with single tap at delay 3
        let channel = vec![(3, (0.8, 0.2))];
        let pilot_rx = convolve(&pilot_tx, &channel);

        let taps = eq.estimate_channel(&pilot_tx, &pilot_rx);
        // Should detect a tap at delay 3
        let delays: Vec<usize> = taps.iter().map(|t| t.delay_index).collect();
        assert!(delays.contains(&3), "Expected delay 3 in {:?}", delays);
    }

    #[test]
    fn test_estimate_channel_two_taps() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            activation_threshold: 0.05,
            max_delay_samples: 32,
            ..default_config()
        });

        let pilot_tx: Vec<(f64, f64)> = (0..128)
            .map(|i| if i % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) })
            .collect();

        let channel = vec![(0, (1.0, 0.0)), (5, (0.5, 0.0))];
        let pilot_rx = convolve(&pilot_tx, &channel);

        let taps = eq.estimate_channel(&pilot_tx, &pilot_rx);
        let delays: Vec<usize> = taps.iter().map(|t| t.delay_index).collect();
        assert!(delays.contains(&0), "Expected delay 0 in {:?}", delays);
        assert!(delays.contains(&5), "Expected delay 5 in {:?}", delays);
    }

    #[test]
    fn test_equalize_passthrough() {
        let mut eq = SparseEqualizer::new(default_config());

        // Manually seed a single identity tap at delay 0
        eq.add_tap(0, (1.0, 0.0));

        let input: Vec<(f64, f64)> = (0..32)
            .map(|i| ((i as f64 * 0.7).cos(), 0.0))
            .collect();
        let output = eq.equalize(&input, &input);
        assert_eq!(output.len(), input.len());

        // With a single tap at coefficient 1.0 and desired == input,
        // error is zero so no adaptation occurs and output == input.
        for i in 0..output.len() {
            let err = mag2(csub(output[i], input[i]));
            assert!(
                err < 1e-20,
                "Error too large at sample {}: {}",
                i,
                err.sqrt()
            );
        }
    }

    #[test]
    fn test_equalize_output_length() {
        let mut eq = SparseEqualizer::new(default_config());
        let pilot: Vec<(f64, f64)> = vec![(1.0, 0.0); 16];
        eq.estimate_channel(&pilot, &pilot);

        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 50];
        let desired: Vec<(f64, f64)> = vec![(1.0, 0.0); 50];
        let output = eq.equalize(&input, &desired);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_equalize_blind_output_length() {
        let mut eq = SparseEqualizer::new(default_config());
        let input: Vec<(f64, f64)> = (0..100)
            .map(|i| ((i as f64 * 0.2).cos(), (i as f64 * 0.2).sin()))
            .collect();
        let output = eq.equalize_blind(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_equalize_blind_seeds_tap() {
        let mut eq = SparseEqualizer::new(default_config());
        assert_eq!(eq.num_active_taps(), 0);

        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 10];
        let _output = eq.equalize_blind(&input);

        // Should have seeded at least one tap
        assert!(eq.num_active_taps() >= 1);
    }

    #[test]
    fn test_prune_taps_removes_weak() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            deactivation_threshold: 0.1,
            ..default_config()
        });

        // Manually add taps: one strong, one very weak
        eq.add_tap(0, (1.0, 0.0));
        eq.add_tap(5, (0.01, 0.0)); // relative power = 0.0001, below 0.1

        assert_eq!(eq.num_active_taps(), 2);
        eq.prune_taps();
        assert_eq!(eq.num_active_taps(), 1);

        let active = eq.active_taps();
        assert_eq!(active[0].delay_index, 0);
    }

    #[test]
    fn test_prune_taps_keeps_strong() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            deactivation_threshold: 0.01,
            ..default_config()
        });

        eq.add_tap(0, (1.0, 0.0));
        eq.add_tap(3, (0.5, 0.0)); // relative power = 0.25, well above 0.01

        eq.prune_taps();
        assert_eq!(eq.num_active_taps(), 2);
    }

    #[test]
    fn test_tap_sparsity() {
        let mut eq = SparseEqualizer::new(default_config());

        // No taps: sparsity is 0
        assert_eq!(eq.tap_sparsity(), 0.0);

        eq.add_tap(0, (1.0, 0.0));
        eq.add_tap(5, (0.5, 0.0));
        // Both active: sparsity = 0
        assert_eq!(eq.tap_sparsity(), 0.0);

        // Deactivate one
        eq.taps[1].is_active = false;
        // sparsity = 1 - 1/2 = 0.5
        assert!((eq.tap_sparsity() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_lms_adaptation() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            adaptation_mode: AdaptationMode::Lms,
            step_size: 0.01,
            ..default_config()
        });

        // Simple channel: gain of 2.0 on direct path
        let channel = vec![(0, (2.0, 0.0))];
        let training_tx: Vec<(f64, f64)> = (0..200)
            .map(|i| ((i as f64 * 0.3).sin(), (i as f64 * 0.3).cos()))
            .collect();
        let training_rx = convolve(&training_tx, &channel);

        eq.estimate_channel(&training_tx, &training_rx);
        let output = eq.equalize(&training_rx, &training_tx);

        // Check convergence in last 20 samples
        let mut last_mse = 0.0;
        for i in (output.len() - 20)..output.len() {
            last_mse += mag2(csub(output[i], training_tx[i]));
        }
        last_mse /= 20.0;
        assert!(last_mse < 1.0, "LMS did not converge, MSE = {}", last_mse);
    }

    #[test]
    fn test_rls_adaptation() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            adaptation_mode: AdaptationMode::Rls,
            step_size: 0.1,
            rls_forgetting_factor: 0.98,
            ..default_config()
        });

        let channel = vec![(0, (1.0, 0.0)), (2, (0.3, 0.1))];
        let training_tx: Vec<(f64, f64)> = (0..300)
            .map(|i| ((i as f64 * 0.4).sin(), (i as f64 * 0.4).cos()))
            .collect();
        let training_rx = convolve(&training_tx, &channel);

        eq.estimate_channel(&training_tx, &training_rx);
        let output = eq.equalize(&training_rx, &training_tx);

        // Check last 20 samples
        let mut last_mse = 0.0;
        let tail = 20;
        for i in (output.len() - tail)..output.len() {
            last_mse += mag2(csub(output[i], training_tx[i]));
        }
        last_mse /= tail as f64;
        assert!(last_mse < 2.0, "RLS did not converge, MSE = {}", last_mse);
    }

    #[test]
    fn test_periodic_pruning() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            prune_interval: 10,
            deactivation_threshold: 0.1,
            ..default_config()
        });

        // Add a strong tap and a weak tap
        eq.add_tap(0, (1.0, 0.0));
        eq.add_tap(5, (0.001, 0.0));

        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 20];
        let desired: Vec<(f64, f64)> = vec![(1.0, 0.0); 20];

        let _output = eq.equalize(&input, &desired);
        // After processing 20 samples with prune_interval=10, pruning should have run
        // The weak tap should have been deactivated
        let weak_tap = eq.taps.iter().find(|t| t.delay_index == 5).unwrap();
        assert!(!weak_tap.is_active, "Weak tap should have been pruned");
    }

    #[test]
    fn test_estimate_channel_empty_input() {
        let mut eq = SparseEqualizer::new(default_config());
        let taps = eq.estimate_channel(&[], &[]);
        assert!(taps.is_empty());
    }

    #[test]
    fn test_tap_info_power_db() {
        let info = TapInfo {
            delay_index: 0,
            coefficient: (1.0, 0.0),
            power_db: power_db((1.0, 0.0)),
            is_active: true,
        };
        // |1+0i|^2 = 1 => 10*log10(1) = 0 dB
        assert!((info.power_db - 0.0).abs() < 1e-10);

        let info2 = TapInfo {
            delay_index: 1,
            coefficient: (0.1, 0.0),
            power_db: power_db((0.1, 0.0)),
            is_active: true,
        };
        // |0.1|^2 = 0.01 => 10*log10(0.01) = -20 dB
        assert!((info2.power_db - (-20.0)).abs() < 1e-6);
    }

    #[test]
    fn test_complex_helpers() {
        // cmul
        let a = (3.0, 4.0);
        let b = (1.0, 2.0);
        let p = cmul(a, b);
        // (3+4i)(1+2i) = 3+6i+4i+8i^2 = (3-8)+(6+4)i = (-5, 10)
        assert!((p.0 - (-5.0)).abs() < 1e-10);
        assert!((p.1 - 10.0).abs() < 1e-10);

        // conj
        let c = conj((3.0, 4.0));
        assert_eq!(c, (3.0, -4.0));

        // mag2
        assert!((mag2((3.0, 4.0)) - 25.0).abs() < 1e-10);

        // cadd / csub
        assert_eq!(cadd((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(csub((3.0, 4.0), (1.0, 2.0)), (2.0, 2.0));
    }

    #[test]
    fn test_max_taps_limit() {
        let mut eq = SparseEqualizer::new(SparseConfig {
            max_taps: 3,
            activation_threshold: 0.01,
            max_delay_samples: 64,
            ..default_config()
        });

        // Create a channel with many taps
        let pilot_tx: Vec<(f64, f64)> = (0..128)
            .map(|i| ((i as f64 * 0.2).sin(), (i as f64 * 0.2).cos()))
            .collect();

        let channel = vec![
            (0, (1.0, 0.0)),
            (3, (0.7, 0.0)),
            (7, (0.5, 0.0)),
            (12, (0.4, 0.0)),
            (20, (0.3, 0.0)),
        ];
        let pilot_rx = convolve(&pilot_tx, &channel);

        let taps = eq.estimate_channel(&pilot_tx, &pilot_rx);
        // Should not exceed max_taps=3
        assert!(taps.len() <= 3, "Got {} taps, expected <= 3", taps.len());
    }
}
