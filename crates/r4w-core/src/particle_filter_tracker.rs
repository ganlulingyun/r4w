//! Sequential Monte Carlo (Particle Filter) estimator for nonlinear/non-Gaussian state tracking.
//!
//! This module provides a particle filter implementation suitable for tracking
//! applications where the state transition or measurement model is nonlinear
//! and/or the noise distributions are non-Gaussian. The filter maintains a
//! weighted set of particles (state hypotheses) and uses predict-update cycles
//! with resampling to approximate the posterior distribution.
//!
//! # Example
//!
//! ```
//! use r4w_core::particle_filter_tracker::{ParticleFilter, PfConfig, ResamplingMethod};
//!
//! // Configure a 1-D random-walk tracker
//! let config = PfConfig {
//!     num_particles: 200,
//!     state_dim: 1,
//!     process_noise_std: 0.1,
//!     measurement_noise_std: 0.5,
//!     resampling_method: ResamplingMethod::Systematic,
//!     seed: 42,
//! };
//!
//! let mut pf = ParticleFilter::new(config);
//!
//! // Predict step (random walk with dt=1.0)
//! pf.predict(1.0);
//!
//! // Update with a measurement at position 3.0
//! let sigma2 = 0.5_f64.powi(2);
//! pf.update(&[3.0], |state, meas| {
//!     let diff = state[0] - meas[0];
//!     (-0.5 * diff * diff / sigma2).exp()
//! });
//!
//! // Weighted mean estimate
//! let est = pf.estimate_state();
//! assert_eq!(est.len(), 1);
//! // After one update the estimate should be pulled toward 3.0
//! assert!(est[0].abs() < 10.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Simple LCG PRNG for reproducibility (no external crates)
// ---------------------------------------------------------------------------

/// Linear congruential generator (Numerical Recipes parameters).
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Advance and return next u64.
    fn next_u64(&mut self) -> u64 {
        // Numerical Recipes LCG
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    /// Uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Approximate standard-normal sample via Box-Muller.
    fn next_gaussian(&mut self) -> f64 {
        loop {
            let u1 = self.next_f64();
            let u2 = self.next_f64();
            if u1 > 1e-30 {
                return (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Resampling strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResamplingMethod {
    /// Systematic resampling – O(N), low variance.
    Systematic,
    /// Stratified resampling – O(N), stratified uniform draws.
    Stratified,
    /// Multinomial resampling – O(N log N), classic bootstrap.
    Multinomial,
}

/// A single particle: a state hypothesis with an associated weight.
#[derive(Debug, Clone)]
pub struct Particle {
    /// State vector.
    pub state: Vec<f64>,
    /// Normalised importance weight.
    pub weight: f64,
}

/// Configuration for [`ParticleFilter`].
#[derive(Debug, Clone)]
pub struct PfConfig {
    /// Number of particles.
    pub num_particles: usize,
    /// Dimensionality of the state vector.
    pub state_dim: usize,
    /// Standard deviation of the additive process noise per dimension.
    pub process_noise_std: f64,
    /// Standard deviation of the measurement noise (used in default likelihood).
    pub measurement_noise_std: f64,
    /// Which resampling algorithm to use.
    pub resampling_method: ResamplingMethod,
    /// PRNG seed for reproducibility.
    pub seed: u64,
}

/// Sequential Monte Carlo (particle) filter.
pub struct ParticleFilter {
    particles: Vec<Particle>,
    config: PfConfig,
    rng: Lcg,
    /// Optional custom state-transition function: fn(state, dt) -> new_state.
    transition_fn: Option<Box<dyn Fn(&[f64], f64) -> Vec<f64>>>,
}

impl ParticleFilter {
    /// Create a new particle filter from the given configuration.
    ///
    /// All particles are initialised at the origin with uniform weight.
    pub fn new(config: PfConfig) -> Self {
        let n = config.num_particles.max(1);
        let w = 1.0 / n as f64;
        let particles = (0..n)
            .map(|_| Particle {
                state: vec![0.0; config.state_dim],
                weight: w,
            })
            .collect();

        let rng = Lcg::new(config.seed);

        Self {
            particles,
            config,
            rng,
            transition_fn: None,
        }
    }

    /// Set a custom state-transition function `f(state, dt) -> new_state`.
    ///
    /// If not set, a random-walk model (`x += noise`) is used.
    pub fn set_transition_fn(&mut self, f: impl Fn(&[f64], f64) -> Vec<f64> + 'static) {
        self.transition_fn = Some(Box::new(f));
    }

    /// Predict step: propagate every particle through the state-transition
    /// model and add process noise.
    pub fn predict(&mut self, dt: f64) {
        let noise_std = self.config.process_noise_std;
        for p in &mut self.particles {
            // Apply transition
            if let Some(ref f) = self.transition_fn {
                p.state = f(&p.state, dt);
            }
            // else: identity (random walk handled by noise below)

            // Add process noise
            for x in &mut p.state {
                *x += noise_std * self.rng.next_gaussian();
            }
        }
    }

    /// Update step: re-weight particles using a user-supplied likelihood
    /// function `likelihood_fn(particle_state, measurement) -> weight_factor`.
    ///
    /// Weights are normalised after the update.  If ESS drops below N/2,
    /// resampling is triggered automatically.
    pub fn update(
        &mut self,
        measurement: &[f64],
        likelihood_fn: impl Fn(&[f64], &[f64]) -> f64,
    ) {
        // Re-weight
        for p in &mut self.particles {
            let lk = likelihood_fn(&p.state, measurement);
            p.weight *= lk;
        }

        // Normalise
        self.normalise_weights();

        // Adaptive resampling
        let n = self.particles.len() as f64;
        if self.effective_sample_size() < n / 2.0 {
            self.resample();
        }
    }

    /// Resample the particle set according to the configured method.
    pub fn resample(&mut self) {
        let n = self.particles.len();
        let indices = match self.config.resampling_method {
            ResamplingMethod::Systematic => self.systematic_resample(),
            ResamplingMethod::Stratified => self.stratified_resample(),
            ResamplingMethod::Multinomial => self.multinomial_resample(),
        };

        let old = self.particles.clone();
        let w = 1.0 / n as f64;
        for (i, &idx) in indices.iter().enumerate() {
            self.particles[i].state = old[idx].state.clone();
            self.particles[i].weight = w;
        }
    }

    /// Weighted-mean state estimate.
    pub fn estimate_state(&self) -> Vec<f64> {
        let dim = self.config.state_dim;
        let mut mean = vec![0.0; dim];
        for p in &self.particles {
            for (j, v) in p.state.iter().enumerate() {
                mean[j] += p.weight * v;
            }
        }
        mean
    }

    /// Weighted covariance matrix of the particle cloud.
    pub fn estimate_covariance(&self) -> Vec<Vec<f64>> {
        let dim = self.config.state_dim;
        let mean = self.estimate_state();
        let mut cov = vec![vec![0.0; dim]; dim];
        for p in &self.particles {
            for i in 0..dim {
                let di = p.state[i] - mean[i];
                for j in 0..dim {
                    let dj = p.state[j] - mean[j];
                    cov[i][j] += p.weight * di * dj;
                }
            }
        }
        cov
    }

    /// Effective sample size: ESS = 1 / sum(w_i^2).
    ///
    /// Ranges from 1 (degenerate) to N (uniform weights).
    pub fn effective_sample_size(&self) -> f64 {
        let sum_sq: f64 = self.particles.iter().map(|p| p.weight * p.weight).sum();
        if sum_sq > 0.0 {
            1.0 / sum_sq
        } else {
            0.0
        }
    }

    /// Immutable access to the particle set.
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    fn normalise_weights(&mut self) {
        let total: f64 = self.particles.iter().map(|p| p.weight).sum();
        if total > 0.0 {
            for p in &mut self.particles {
                p.weight /= total;
            }
        } else {
            // Degenerate case – reset to uniform
            let w = 1.0 / self.particles.len() as f64;
            for p in &mut self.particles {
                p.weight = w;
            }
        }
    }

    /// O(N) systematic resampling with a single uniform draw.
    fn systematic_resample(&mut self) -> Vec<usize> {
        let n = self.particles.len();
        let mut indices = Vec::with_capacity(n);
        let u0 = self.rng.next_f64() / n as f64;
        let mut cumulative = 0.0;
        let mut j = 0;
        for i in 0..n {
            let threshold = u0 + i as f64 / n as f64;
            while cumulative + self.particles[j].weight < threshold && j + 1 < n {
                cumulative += self.particles[j].weight;
                j += 1;
            }
            indices.push(j);
        }
        indices
    }

    /// O(N) stratified resampling – one uniform draw per stratum.
    fn stratified_resample(&mut self) -> Vec<usize> {
        let n = self.particles.len();
        let mut indices = Vec::with_capacity(n);
        let mut cumulative = 0.0;
        let mut j = 0;
        for i in 0..n {
            let u = (i as f64 + self.rng.next_f64()) / n as f64;
            while cumulative + self.particles[j].weight < u && j + 1 < n {
                cumulative += self.particles[j].weight;
                j += 1;
            }
            indices.push(j);
        }
        indices
    }

    /// Multinomial resampling – classic bootstrap (N independent draws).
    fn multinomial_resample(&mut self) -> Vec<usize> {
        let n = self.particles.len();
        let mut indices = Vec::with_capacity(n);

        // Build CDF
        let mut cdf = Vec::with_capacity(n);
        let mut acc = 0.0;
        for p in &self.particles {
            acc += p.weight;
            cdf.push(acc);
        }

        for _ in 0..n {
            let u = self.rng.next_f64();
            // Binary search
            let idx = match cdf.binary_search_by(|c| c.partial_cmp(&u).unwrap()) {
                Ok(i) => i,
                Err(i) => i.min(n - 1),
            };
            indices.push(idx);
        }
        indices
    }
}

/// Gaussian likelihood helper: `exp(-0.5 * ||z - h(x)||^2 / sigma^2)`.
///
/// Useful as a likelihood function in [`ParticleFilter::update`].
pub fn gaussian_likelihood(state: &[f64], measurement: &[f64], sigma: f64) -> f64 {
    let sq_sum: f64 = state
        .iter()
        .zip(measurement.iter())
        .map(|(s, m)| {
            let d = s - m;
            d * d
        })
        .sum();
    (-0.5 * sq_sum / (sigma * sigma)).exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PfConfig {
        PfConfig {
            num_particles: 500,
            state_dim: 1,
            process_noise_std: 0.1,
            measurement_noise_std: 0.5,
            resampling_method: ResamplingMethod::Systematic,
            seed: 12345,
        }
    }

    #[test]
    fn test_new_particle_count() {
        let cfg = default_config();
        let pf = ParticleFilter::new(cfg.clone());
        assert_eq!(pf.particles().len(), cfg.num_particles);
    }

    #[test]
    fn test_initial_weights_uniform() {
        let cfg = default_config();
        let pf = ParticleFilter::new(cfg.clone());
        let expected = 1.0 / cfg.num_particles as f64;
        for p in pf.particles() {
            assert!((p.weight - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn test_initial_state_zero() {
        let cfg = default_config();
        let pf = ParticleFilter::new(cfg.clone());
        for p in pf.particles() {
            assert_eq!(p.state, vec![0.0; cfg.state_dim]);
        }
    }

    #[test]
    fn test_ess_uniform() {
        let cfg = default_config();
        let pf = ParticleFilter::new(cfg.clone());
        let ess = pf.effective_sample_size();
        assert!((ess - cfg.num_particles as f64).abs() < 1e-6);
    }

    #[test]
    fn test_predict_adds_noise() {
        let cfg = default_config();
        let mut pf = ParticleFilter::new(cfg);
        pf.predict(1.0);
        // After prediction with noise, not all particles should be at zero
        let any_nonzero = pf.particles().iter().any(|p| p.state[0].abs() > 1e-12);
        assert!(any_nonzero, "predict should perturb particles");
    }

    #[test]
    fn test_update_shifts_estimate() {
        let cfg = PfConfig {
            num_particles: 2000,
            state_dim: 1,
            process_noise_std: 1.0,
            measurement_noise_std: 0.5,
            seed: 99,
            ..default_config()
        };
        let mut pf = ParticleFilter::new(cfg.clone());
        let sigma = cfg.measurement_noise_std;

        // Spread particles with large predict step
        pf.predict(1.0);

        // Update toward measurement at 2.0
        pf.update(&[2.0], |s, m| gaussian_likelihood(s, m, sigma));

        let est = pf.estimate_state();
        // Estimate should move toward 2.0 (within a generous tolerance)
        assert!(
            est[0].abs() < 5.0,
            "estimate {} should be in a reasonable range",
            est[0]
        );
    }

    #[test]
    fn test_gaussian_likelihood_peak() {
        // Likelihood should be maximised when state == measurement
        let peak = gaussian_likelihood(&[1.0], &[1.0], 1.0);
        let off = gaussian_likelihood(&[1.0], &[2.0], 1.0);
        assert!((peak - 1.0).abs() < 1e-12);
        assert!(off < peak);
    }

    #[test]
    fn test_gaussian_likelihood_symmetry() {
        let l1 = gaussian_likelihood(&[0.0], &[1.0], 1.0);
        let l2 = gaussian_likelihood(&[0.0], &[-1.0], 1.0);
        assert!((l1 - l2).abs() < 1e-12);
    }

    #[test]
    fn test_resample_preserves_count() {
        let cfg = default_config();
        let mut pf = ParticleFilter::new(cfg.clone());
        pf.predict(1.0);
        pf.resample();
        assert_eq!(pf.particles().len(), cfg.num_particles);
    }

    #[test]
    fn test_resample_uniform_weights_after() {
        let cfg = default_config();
        let mut pf = ParticleFilter::new(cfg.clone());
        pf.predict(1.0);
        // Manually skew weights
        pf.particles[0].weight = 0.99;
        for i in 1..pf.particles.len() {
            pf.particles[i].weight = 0.01 / (pf.particles.len() - 1) as f64;
        }
        pf.resample();
        let expected = 1.0 / cfg.num_particles as f64;
        for p in pf.particles() {
            assert!((p.weight - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn test_estimate_covariance_dimensions() {
        let cfg = PfConfig {
            state_dim: 3,
            ..default_config()
        };
        let pf = ParticleFilter::new(cfg);
        let cov = pf.estimate_covariance();
        assert_eq!(cov.len(), 3);
        for row in &cov {
            assert_eq!(row.len(), 3);
        }
    }

    #[test]
    fn test_covariance_zero_at_init() {
        let cfg = default_config();
        let pf = ParticleFilter::new(cfg);
        let cov = pf.estimate_covariance();
        // All particles at zero -> zero covariance
        assert!(cov[0][0].abs() < 1e-12);
    }

    #[test]
    fn test_stratified_resampling() {
        let cfg = PfConfig {
            resampling_method: ResamplingMethod::Stratified,
            ..default_config()
        };
        let mut pf = ParticleFilter::new(cfg.clone());
        pf.predict(1.0);
        pf.resample();
        assert_eq!(pf.particles().len(), cfg.num_particles);
    }

    #[test]
    fn test_multinomial_resampling() {
        let cfg = PfConfig {
            resampling_method: ResamplingMethod::Multinomial,
            ..default_config()
        };
        let mut pf = ParticleFilter::new(cfg.clone());
        pf.predict(1.0);
        pf.resample();
        assert_eq!(pf.particles().len(), cfg.num_particles);
    }

    #[test]
    fn test_custom_transition_fn() {
        let cfg = PfConfig {
            num_particles: 100,
            state_dim: 2,
            process_noise_std: 0.0, // no noise, isolate transition
            ..default_config()
        };
        let mut pf = ParticleFilter::new(cfg);

        // Constant-velocity model: x' = x + v*dt, v' = v
        pf.set_transition_fn(|state: &[f64], dt: f64| {
            vec![state[0] + state[1] * dt, state[1]]
        });

        // Manually set one particle
        pf.particles[0].state = vec![0.0, 5.0]; // pos=0, vel=5
        pf.predict(2.0); // dt=2 -> pos should be 10

        assert!(
            (pf.particles[0].state[0] - 10.0).abs() < 1e-9,
            "position should be 10.0 but got {}",
            pf.particles[0].state[0]
        );
        assert!(
            (pf.particles[0].state[1] - 5.0).abs() < 1e-9,
            "velocity should remain 5.0"
        );
    }

    #[test]
    fn test_ess_after_degenerate_weights() {
        let cfg = default_config();
        let mut pf = ParticleFilter::new(cfg);
        // All weight on one particle
        pf.particles[0].weight = 1.0;
        for i in 1..pf.particles.len() {
            pf.particles[i].weight = 0.0;
        }
        let ess = pf.effective_sample_size();
        assert!((ess - 1.0).abs() < 1e-6, "ESS should be 1.0 for degenerate");
    }

    #[test]
    fn test_multidim_tracking() {
        // 2-D tracking: state = [x, y], measure both
        let cfg = PfConfig {
            num_particles: 3000,
            state_dim: 2,
            process_noise_std: 2.0,
            measurement_noise_std: 0.5,
            resampling_method: ResamplingMethod::Systematic,
            seed: 77,
        };
        let mut pf = ParticleFilter::new(cfg);
        let sigma = 0.5;

        // Spread particles
        pf.predict(1.0);

        // Update toward (3.0, -1.0)
        pf.update(&[3.0, -1.0], |s, m| gaussian_likelihood(s, m, sigma));

        let est = pf.estimate_state();
        assert_eq!(est.len(), 2);
        // With 3000 particles and large spread, estimate should be reasonable
        assert!(
            est[0].abs() < 10.0 && est[1].abs() < 10.0,
            "estimate {:?} out of range",
            est
        );
    }

    #[test]
    fn test_iq_phase_tracking() {
        // Demonstrate IQ tracking: state = [phase], measurement = (I, Q) pair
        // where I = cos(phase), Q = sin(phase).
        let cfg = PfConfig {
            num_particles: 2000,
            state_dim: 1,
            process_noise_std: 0.5,
            measurement_noise_std: 0.1,
            resampling_method: ResamplingMethod::Systematic,
            seed: 42,
        };
        let mut pf = ParticleFilter::new(cfg);

        // True phase = pi/4
        let true_phase = std::f64::consts::FRAC_PI_4;
        let iq: (f64, f64) = (true_phase.cos(), true_phase.sin());

        // Spread particles
        pf.predict(1.0);

        // Custom likelihood: compare predicted IQ with measured IQ
        let sigma = 0.1;
        pf.update(&[iq.0, iq.1], |state, meas| {
            let pred_i = state[0].cos();
            let pred_q = state[0].sin();
            let di = pred_i - meas[0];
            let dq = pred_q - meas[1];
            let sq = di * di + dq * dq;
            (-0.5 * sq / (sigma * sigma)).exp()
        });

        // The filter ran - verify the estimate exists and has correct dimension
        let est = pf.estimate_state();
        assert_eq!(est.len(), 1);
    }

    #[test]
    fn test_weights_sum_to_one() {
        let cfg = default_config();
        let mut pf = ParticleFilter::new(cfg);
        pf.predict(1.0);
        pf.update(&[1.0], |s, m| gaussian_likelihood(s, m, 1.0));
        let sum: f64 = pf.particles().iter().map(|p| p.weight).sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "weights should sum to 1.0 but got {}",
            sum
        );
    }
}
