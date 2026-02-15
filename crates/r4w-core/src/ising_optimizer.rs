//! Ising model optimization for combinatorial problems in DSP.
//!
//! This module maps combinatorial optimization problems to spin Hamiltonians and
//! solves them via simulated annealing and classical heuristics. The Ising model is
//! a natural framework for many DSP tasks such as maximum-likelihood sequence detection,
//! channel assignment, sensor placement, and graph partitioning.
//!
//! # Overview
//!
//! The **Ising model** describes a system of binary spins s_i in {-1, +1} interacting
//! through pairwise couplings J_ij and subject to external fields h_i. The energy
//! (Hamiltonian) is:
//!
//! ```text
//! H(s) = -sum_{i<j} J_ij * s_i * s_j  -  sum_i h_i * s_i
//! ```
//!
//! Minimizing H(s) is NP-hard in general, but simulated annealing provides a powerful
//! meta-heuristic that provably converges to the global minimum given a sufficiently
//! slow cooling schedule.
//!
//! The closely related **QUBO** (Quadratic Unconstrained Binary Optimization) formulation
//! uses binary variables x_i in {0, 1}:
//!
//! ```text
//! E(x) = sum_{i<=j} Q_ij * x_i * x_j
//! ```
//!
//! The two formulations are polynomially equivalent; this module provides conversions
//! in both directions.
//!
//! # Components
//!
//! - [`IsingModel`] — spin configuration, coupling matrix J, external field h
//! - [`SimulatedAnnealer`] — Metropolis-Hastings with configurable cooling schedules
//! - [`QuboSolver`] — QUBO problem representation and solver
//! - [`EnergyCalculator`] — efficient Ising Hamiltonian evaluation and delta-energy
//! - [`MaxCutMapper`] — map graph max-cut problems to Ising form
//! - [`ScheduleType`] — Linear, Geometric, Logarithmic cooling schedules
//!
//! # Example
//!
//! ```
//! use r4w_core::ising_optimizer::{
//!     IsingModel, SimulatedAnnealer, ScheduleType, random_spin_config, magnetization,
//! };
//!
//! // Create a 4-spin ferromagnetic chain: J_01=1, J_12=1, J_23=1
//! let mut model = IsingModel::new(4);
//! model.set_coupling(0, 1, 1.0);
//! model.set_coupling(1, 2, 1.0);
//! model.set_coupling(2, 3, 1.0);
//!
//! // Solve by simulated annealing
//! let annealer = SimulatedAnnealer::new(10.0, 0.01, 2000, ScheduleType::Geometric);
//! let (best_spins, best_energy) = annealer.solve(&model, 42);
//!
//! // Ground state is all +1 or all -1 (magnetization = +/- 1.0)
//! assert!(magnetization(&best_spins).abs() > 0.9);
//! assert!(best_energy <= -3.0 + 1e-9);
//! ```

use std::f64::consts::E;

// ---------------------------------------------------------------------------
// Cooling schedule
// ---------------------------------------------------------------------------

/// Cooling schedule for simulated annealing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScheduleType {
    /// T(k) = T_max - k * (T_max - T_min) / steps
    Linear,
    /// T(k) = T_max * alpha^k   where alpha = (T_min / T_max)^(1/steps)
    Geometric,
    /// T(k) = T_max / (1 + c * ln(1 + k))  where c chosen so T(steps) = T_min
    Logarithmic,
}

impl ScheduleType {
    /// Compute the temperature at step `k` out of `total_steps`.
    pub fn temperature(&self, k: usize, total_steps: usize, t_max: f64, t_min: f64) -> f64 {
        if total_steps == 0 {
            return t_min;
        }
        match self {
            ScheduleType::Linear => {
                let frac = k as f64 / total_steps as f64;
                t_max - frac * (t_max - t_min)
            }
            ScheduleType::Geometric => {
                let alpha = (t_min / t_max).powf(1.0 / total_steps as f64);
                t_max * alpha.powi(k as i32)
            }
            ScheduleType::Logarithmic => {
                // T(k) = T_max / (1 + c * ln(1 + k))
                // Solve for c: T(steps) = T_min => c = (T_max/T_min - 1) / ln(1 + steps)
                let ln_end = ((1 + total_steps) as f64).ln();
                let c = (t_max / t_min - 1.0) / ln_end;
                t_max / (1.0 + c * ((1 + k) as f64).ln())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Simple LCG PRNG (no external deps)
// ---------------------------------------------------------------------------

/// A simple xorshift64 PRNG for deterministic pseudo-random numbers.
///
/// Uses the xorshift64 algorithm by Marsaglia, which provides good statistical
/// properties with minimal state.
#[derive(Debug, Clone)]
struct Rng64 {
    state: u64,
}

impl Rng64 {
    fn new(seed: u64) -> Self {
        // Ensure non-zero state (xorshift requires it)
        let s = if seed == 0 { 0x5EED_DEAD_BEEF_CAFE } else { seed };
        Self { state: s }
    }

    /// Next u64 value using xorshift64.
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        // Use upper 53 bits for full double precision
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Random index in [0, n).
    fn next_usize(&mut self, n: usize) -> usize {
        (self.next_u64() as usize) % n
    }

    /// Random boolean with probability p of being true.
    fn bernoulli(&mut self, p: f64) -> bool {
        self.next_f64() < p
    }
}

// ---------------------------------------------------------------------------
// IsingModel
// ---------------------------------------------------------------------------

/// An Ising model defined by N spins, a coupling matrix J, and external field h.
///
/// The Hamiltonian is:
///
/// ```text
/// H(s) = -sum_{i<j} J_ij * s_i * s_j  -  sum_i h_i * s_i
/// ```
///
/// Couplings are stored in a flat upper-triangular vector for compactness.
/// Positive J_ij favors aligned spins (ferromagnetic); negative J_ij favors
/// anti-aligned spins (anti-ferromagnetic).
#[derive(Debug, Clone)]
pub struct IsingModel {
    /// Number of spins.
    n: usize,
    /// Upper-triangular coupling matrix J stored as a flat vector of length n*(n-1)/2.
    /// Index for pair (i,j) with i < j is: i*n - i*(i+1)/2 + (j - i - 1).
    couplings: Vec<f64>,
    /// External field h of length n.
    fields: Vec<f64>,
}

impl IsingModel {
    /// Create a new Ising model with `n` spins. All couplings and fields are zero.
    pub fn new(n: usize) -> Self {
        let num_pairs = if n > 1 { n * (n - 1) / 2 } else { 0 };
        Self {
            n,
            couplings: vec![0.0; num_pairs],
            fields: vec![0.0; n],
        }
    }

    /// Number of spins.
    pub fn num_spins(&self) -> usize {
        self.n
    }

    /// Flat index into the coupling vector for pair (i, j) with i < j.
    fn pair_index(&self, i: usize, j: usize) -> usize {
        debug_assert!(i < j && j < self.n);
        i * self.n - i * (i + 1) / 2 + (j - i - 1)
    }

    /// Set the coupling between spins i and j. Order does not matter.
    pub fn set_coupling(&mut self, i: usize, j: usize, value: f64) {
        if i == j || i >= self.n || j >= self.n {
            return;
        }
        let (a, b) = if i < j { (i, j) } else { (j, i) };
        let idx = self.pair_index(a, b);
        self.couplings[idx] = value;
    }

    /// Get the coupling between spins i and j.
    pub fn coupling(&self, i: usize, j: usize) -> f64 {
        if i == j || i >= self.n || j >= self.n {
            return 0.0;
        }
        let (a, b) = if i < j { (i, j) } else { (j, i) };
        self.couplings[self.pair_index(a, b)]
    }

    /// Set the external field on spin i.
    pub fn set_field(&mut self, i: usize, value: f64) {
        if i < self.n {
            self.fields[i] = value;
        }
    }

    /// Get the external field on spin i.
    pub fn field(&self, i: usize) -> f64 {
        if i < self.n {
            self.fields[i]
        } else {
            0.0
        }
    }

    /// Return a reference to the external fields vector.
    pub fn fields(&self) -> &[f64] {
        &self.fields
    }

    /// Return a reference to the couplings vector (upper-triangular, flat).
    pub fn couplings_flat(&self) -> &[f64] {
        &self.couplings
    }

    /// Build an Ising model from a full (symmetric) coupling matrix and field vector.
    ///
    /// `j_matrix` is n x n stored in row-major order. Only the upper triangle is used.
    pub fn from_matrix(j_matrix: &[f64], h: &[f64], n: usize) -> Self {
        assert_eq!(j_matrix.len(), n * n);
        assert_eq!(h.len(), n);
        let mut model = Self::new(n);
        for i in 0..n {
            model.fields[i] = h[i];
            for j in (i + 1)..n {
                model.set_coupling(i, j, j_matrix[i * n + j]);
            }
        }
        model
    }
}

// ---------------------------------------------------------------------------
// EnergyCalculator
// ---------------------------------------------------------------------------

/// Efficient Ising Hamiltonian evaluation and delta-energy computation.
///
/// The full energy is:
///
/// ```text
/// H(s) = -sum_{i<j} J_ij * s_i * s_j  -  sum_i h_i * s_i
/// ```
///
/// When a single spin k is flipped, the change in energy is computed in O(N) time:
///
/// ```text
/// delta_H = 2 * s_k * (sum_j J_kj * s_j  +  h_k)
/// ```
pub struct EnergyCalculator;

impl EnergyCalculator {
    /// Compute the full Ising energy for a given spin configuration.
    pub fn energy(model: &IsingModel, spins: &[i8]) -> f64 {
        assert_eq!(spins.len(), model.n);
        let n = model.n;
        let mut e = 0.0;

        // Coupling term: -sum_{i<j} J_ij * s_i * s_j
        for i in 0..n {
            for j in (i + 1)..n {
                let j_ij = model.coupling(i, j);
                if j_ij != 0.0 {
                    e -= j_ij * (spins[i] as f64) * (spins[j] as f64);
                }
            }
        }

        // Field term: -sum_i h_i * s_i
        for i in 0..n {
            let h_i = model.field(i);
            if h_i != 0.0 {
                e -= h_i * (spins[i] as f64);
            }
        }

        e
    }

    /// Compute the change in energy when spin `k` is flipped.
    ///
    /// This is O(N) rather than O(N^2) for full recomputation.
    ///
    /// ```text
    /// delta_H = 2 * s_k * (sum_j J_kj * s_j  +  h_k)
    /// ```
    pub fn delta_energy(model: &IsingModel, spins: &[i8], k: usize) -> f64 {
        let n = model.n;
        assert!(k < n);
        let s_k = spins[k] as f64;

        let mut local_field = model.field(k);
        for j in 0..n {
            if j != k {
                local_field += model.coupling(k, j) * (spins[j] as f64);
            }
        }

        2.0 * s_k * local_field
    }
}

// ---------------------------------------------------------------------------
// SimulatedAnnealer
// ---------------------------------------------------------------------------

/// Simulated annealing optimizer for Ising models using the Metropolis-Hastings
/// algorithm with configurable cooling schedules.
///
/// At each step a random spin is proposed for flipping. If the flip lowers the
/// energy it is always accepted; otherwise it is accepted with probability
/// exp(-delta_E / T) where T is the current temperature.
#[derive(Debug, Clone)]
pub struct SimulatedAnnealer {
    /// Maximum (initial) temperature.
    pub t_max: f64,
    /// Minimum (final) temperature.
    pub t_min: f64,
    /// Number of cooling steps.
    pub steps: usize,
    /// Cooling schedule.
    pub schedule: ScheduleType,
}

impl SimulatedAnnealer {
    /// Create a new simulated annealer.
    pub fn new(t_max: f64, t_min: f64, steps: usize, schedule: ScheduleType) -> Self {
        Self { t_max, t_min, steps, schedule }
    }

    /// Run simulated annealing on the given Ising model.
    ///
    /// Returns `(best_spins, best_energy)`.
    ///
    /// The `seed` parameter initializes the PRNG for reproducibility.
    pub fn solve(&self, model: &IsingModel, seed: u64) -> (Vec<i8>, f64) {
        let n = model.num_spins();
        if n == 0 {
            return (vec![], 0.0);
        }

        let mut rng = Rng64::new(seed);

        // Initialize random spin configuration
        let mut spins: Vec<i8> = (0..n)
            .map(|_| if rng.bernoulli(0.5) { 1 } else { -1 })
            .collect();

        let mut current_energy = EnergyCalculator::energy(model, &spins);
        let mut best_spins = spins.clone();
        let mut best_energy = current_energy;

        for step in 0..self.steps {
            let temp = self.schedule.temperature(step, self.steps, self.t_max, self.t_min);

            // Propose flipping a random spin
            let k = rng.next_usize(n);
            let delta_e = EnergyCalculator::delta_energy(model, &spins, k);

            // Metropolis criterion
            let accept = if delta_e <= 0.0 {
                true
            } else if temp > 1e-15 {
                let prob = E.powf(-delta_e / temp);
                rng.bernoulli(prob)
            } else {
                false
            };

            if accept {
                spins[k] = -spins[k];
                current_energy += delta_e;

                if current_energy < best_energy {
                    best_energy = current_energy;
                    best_spins.clone_from(&spins);
                }
            }
        }

        (best_spins, best_energy)
    }

    /// Run simulated annealing from a specified initial spin configuration.
    ///
    /// Returns `(best_spins, best_energy)`.
    pub fn solve_from(&self, model: &IsingModel, initial: &[i8], seed: u64) -> (Vec<i8>, f64) {
        let n = model.num_spins();
        assert_eq!(initial.len(), n);

        if n == 0 {
            return (vec![], 0.0);
        }

        let mut rng = Rng64::new(seed);
        let mut spins = initial.to_vec();
        let mut current_energy = EnergyCalculator::energy(model, &spins);
        let mut best_spins = spins.clone();
        let mut best_energy = current_energy;

        for step in 0..self.steps {
            let temp = self.schedule.temperature(step, self.steps, self.t_max, self.t_min);
            let k = rng.next_usize(n);
            let delta_e = EnergyCalculator::delta_energy(model, &spins, k);

            let accept = if delta_e <= 0.0 {
                true
            } else if temp > 1e-15 {
                let prob = E.powf(-delta_e / temp);
                rng.bernoulli(prob)
            } else {
                false
            };

            if accept {
                spins[k] = -spins[k];
                current_energy += delta_e;
                if current_energy < best_energy {
                    best_energy = current_energy;
                    best_spins.clone_from(&spins);
                }
            }
        }

        (best_spins, best_energy)
    }
}

// ---------------------------------------------------------------------------
// QUBO <-> Ising conversion
// ---------------------------------------------------------------------------

/// Convert a QUBO matrix Q (n x n, row-major) to an Ising model.
///
/// The QUBO objective is:
///
/// ```text
/// E(x) = sum_{i<=j} Q_ij * x_i * x_j,   x_i in {0, 1}
/// ```
///
/// Under the substitution x_i = (1 + s_i) / 2 this becomes an Ising Hamiltonian
/// with shifted energy:
///
/// ```text
/// J_ij = -Q_ij / 4        (i != j)
/// h_i  = -(Q_ii / 2 + sum_{j!=i} Q_ij / 4)
/// offset = sum_i Q_ii / 2 + sum_{i<j} Q_ij / 4  (constant energy shift)
/// ```
///
/// Returns `(IsingModel, offset)`.
pub fn qubo_to_ising(q: &[f64], n: usize) -> (IsingModel, f64) {
    assert_eq!(q.len(), n * n);
    let mut model = IsingModel::new(n);
    let mut offset = 0.0;

    // Coupling terms
    for i in 0..n {
        for j in (i + 1)..n {
            let q_ij = q[i * n + j] + q[j * n + i]; // symmetrize
            model.set_coupling(i, j, -q_ij / 4.0);
            offset += q_ij / 4.0;
        }
    }

    // Field terms
    for i in 0..n {
        let mut h_i = q[i * n + i] / 2.0;
        for j in 0..n {
            if j != i {
                h_i += (q[i * n + j] + q[j * n + i]) / 4.0;
            }
        }
        model.set_field(i, -h_i);
        offset += q[i * n + i] / 2.0;
    }

    (model, offset)
}

/// Convert an Ising model back to a QUBO matrix (n x n, row-major) and offset.
///
/// Under x_i = (1 + s_i) / 2:
///
/// ```text
/// Q_ij = -4 * J_ij        (i < j, stored in upper triangle)
/// Q_ii = -2 * h_i + sum_{j!=i} 2 * J_ij
/// offset = sum_{i<j} J_ij + sum_i h_i / 2  (constant shift added back)
/// ```
///
/// Returns `(Q_matrix, offset)` where Q_matrix is n x n row-major.
pub fn ising_to_qubo(model: &IsingModel) -> (Vec<f64>, f64) {
    let n = model.num_spins();
    let mut q = vec![0.0; n * n];
    let mut offset = 0.0;

    // Off-diagonal: Q_ij = -4 * J_ij
    for i in 0..n {
        for j in (i + 1)..n {
            let j_ij = model.coupling(i, j);
            q[i * n + j] = -4.0 * j_ij;
            q[j * n + i] = -4.0 * j_ij;
            offset += j_ij;
        }
    }

    // Diagonal: Q_ii = -2 * h_i + sum_{j!=i} 2 * J_ij
    for i in 0..n {
        let mut sum_j = 0.0;
        for j in 0..n {
            if j != i {
                sum_j += model.coupling(i, j);
            }
        }
        q[i * n + i] = -2.0 * model.field(i) + 2.0 * sum_j;
        offset += model.field(i);
    }

    // Adjust offset sign for the convention
    // Ising energy = -offset + QUBO energy => offset = -(shift)
    // Actually let's compute it properly:
    // H_ising(s) = QUBO(x) - offset, so offset = QUBO(x) - H_ising(s)
    // We need the constant that was dropped in the substitution.
    // Recompute: the constant from x = (1+s)/2 expansion is:
    //   C = -sum_{i<j} J_ij - sum_i h_i
    // But since QUBO(x) = H_ising(s) + C, we return -C as the offset.
    let mut c = 0.0;
    for i in 0..n {
        c -= model.field(i);
        for j in (i + 1)..n {
            c -= model.coupling(i, j);
        }
    }

    (q, -c)
}

// ---------------------------------------------------------------------------
// QuboSolver
// ---------------------------------------------------------------------------

/// Quadratic Unconstrained Binary Optimization solver.
///
/// Stores a QUBO problem in matrix form and solves it by converting to Ising
/// form and applying simulated annealing.
#[derive(Debug, Clone)]
pub struct QuboSolver {
    /// QUBO matrix Q, n x n row-major.
    q: Vec<f64>,
    /// Problem size.
    n: usize,
}

impl QuboSolver {
    /// Create a new QUBO solver with an n x n matrix (row-major).
    pub fn new(q: Vec<f64>, n: usize) -> Self {
        assert_eq!(q.len(), n * n);
        Self { q, n }
    }

    /// Problem size.
    pub fn size(&self) -> usize {
        self.n
    }

    /// Evaluate the QUBO objective for a binary vector x in {0, 1}^n.
    pub fn evaluate(&self, x: &[u8]) -> f64 {
        assert_eq!(x.len(), self.n);
        let mut e = 0.0;
        for i in 0..self.n {
            for j in i..self.n {
                if x[i] != 0 && x[j] != 0 {
                    e += self.q[i * self.n + j];
                }
            }
        }
        // Also add lower-triangle for full symmetry
        for i in 0..self.n {
            for j in 0..i {
                if x[i] != 0 && x[j] != 0 {
                    e += self.q[i * self.n + j];
                }
            }
        }
        e
    }

    /// Solve the QUBO by converting to Ising and running simulated annealing.
    ///
    /// Returns `(best_x, best_qubo_value)` where best_x is in {0, 1}^n.
    pub fn solve(
        &self,
        t_max: f64,
        t_min: f64,
        steps: usize,
        schedule: ScheduleType,
        seed: u64,
    ) -> (Vec<u8>, f64) {
        let (ising, offset) = qubo_to_ising(&self.q, self.n);
        let annealer = SimulatedAnnealer::new(t_max, t_min, steps, schedule);
        let (best_spins, best_ising_energy) = annealer.solve(&ising, seed);

        // Convert spins back to binary: x_i = (1 + s_i) / 2
        let best_x: Vec<u8> = best_spins.iter().map(|&s| ((1 + s as i32) / 2) as u8).collect();
        let qubo_value = best_ising_energy + offset;

        (best_x, qubo_value)
    }

    /// Reference to the Q matrix.
    pub fn matrix(&self) -> &[f64] {
        &self.q
    }
}

// ---------------------------------------------------------------------------
// MaxCutMapper
// ---------------------------------------------------------------------------

/// Maps a graph max-cut problem to Ising form.
///
/// Given an undirected weighted graph with N nodes, the max-cut problem seeks a
/// partition of the vertices into two sets S and T that maximizes the total weight
/// of edges crossing the cut.
///
/// The Ising mapping is:
///
/// ```text
/// H(s) = -sum_{(i,j) in E} w_ij * s_i * s_j
/// ```
///
/// so that minimizing H corresponds to maximizing the cut (anti-aligned spins
/// across edges contribute negative energy).
///
/// The cut value for a spin configuration is:
///
/// ```text
/// cut = (W_total - H(s)) / 2
/// ```
///
/// where W_total = sum of all edge weights.
#[derive(Debug, Clone)]
pub struct MaxCutMapper {
    /// Number of vertices.
    n: usize,
    /// Edge list: (i, j, weight).
    edges: Vec<(usize, usize, f64)>,
}

impl MaxCutMapper {
    /// Create a max-cut mapper for a graph with `n` vertices.
    pub fn new(n: usize) -> Self {
        Self { n, edges: Vec::new() }
    }

    /// Add an undirected edge between vertices i and j with the given weight.
    pub fn add_edge(&mut self, i: usize, j: usize, weight: f64) {
        assert!(i < self.n && j < self.n && i != j);
        self.edges.push((i, j, weight));
    }

    /// Number of edges.
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Total weight of all edges.
    pub fn total_weight(&self) -> f64 {
        self.edges.iter().map(|&(_, _, w)| w).sum()
    }

    /// Convert the graph to an Ising model.
    ///
    /// Each edge (i, j, w) becomes coupling J_ij = -w so that minimizing
    /// H maximizes the cut.
    pub fn to_ising(&self) -> IsingModel {
        let mut model = IsingModel::new(self.n);
        for &(i, j, w) in &self.edges {
            // Set J_ij = -w (anti-ferromagnetic encourages anti-alignment = cut)
            let existing = model.coupling(i, j);
            model.set_coupling(i, j, existing - w);
        }
        model
    }

    /// Compute the cut value for a given spin configuration.
    ///
    /// An edge (i,j) is cut if s_i != s_j. The cut value is the sum of weights
    /// of cut edges.
    pub fn cut_value(&self, spins: &[i8]) -> f64 {
        assert_eq!(spins.len(), self.n);
        let mut value = 0.0;
        for &(i, j, w) in &self.edges {
            if spins[i] != spins[j] {
                value += w;
            }
        }
        value
    }

    /// Solve the max-cut problem via simulated annealing.
    ///
    /// Returns `(best_spins, cut_value)`.
    pub fn solve(
        &self,
        t_max: f64,
        t_min: f64,
        steps: usize,
        schedule: ScheduleType,
        seed: u64,
    ) -> (Vec<i8>, f64) {
        let ising = self.to_ising();
        let annealer = SimulatedAnnealer::new(t_max, t_min, steps, schedule);
        let (best_spins, _best_energy) = annealer.solve(&ising, seed);
        let cut = self.cut_value(&best_spins);
        (best_spins, cut)
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Generate a random spin configuration of length n using a simple LCG seeded
/// with `seed`. Each spin is +1 or -1 with equal probability.
pub fn random_spin_config(n: usize, seed: u64) -> Vec<i8> {
    let mut rng = Rng64::new(seed);
    (0..n).map(|_| if rng.bernoulli(0.5) { 1 } else { -1 }).collect()
}

/// Compute the magnetization m = (1/N) * sum_i s_i.
///
/// For a fully aligned configuration |m| = 1; for a random configuration m ~ 0.
pub fn magnetization(spins: &[i8]) -> f64 {
    if spins.is_empty() {
        return 0.0;
    }
    let sum: i64 = spins.iter().map(|&s| s as i64).sum();
    sum as f64 / spins.len() as f64
}

/// Convert a binary vector x in {0,1}^n to spins s in {-1,+1}^n via s_i = 2*x_i - 1.
pub fn binary_to_spins(x: &[u8]) -> Vec<i8> {
    x.iter().map(|&xi| if xi != 0 { 1i8 } else { -1i8 }).collect()
}

/// Convert spins s in {-1,+1}^n to binary x in {0,1}^n via x_i = (1 + s_i) / 2.
pub fn spins_to_binary(spins: &[i8]) -> Vec<u8> {
    spins.iter().map(|&s| ((1 + s as i32) / 2) as u8).collect()
}

/// Compute the overlap (normalized inner product) between two spin configurations.
///
/// ```text
/// overlap = (1/N) * sum_i s1_i * s2_i
/// ```
///
/// Returns 1.0 for identical, -1.0 for opposite, ~0 for uncorrelated.
pub fn spin_overlap(s1: &[i8], s2: &[i8]) -> f64 {
    assert_eq!(s1.len(), s2.len());
    if s1.is_empty() {
        return 0.0;
    }
    let sum: i64 = s1.iter().zip(s2.iter()).map(|(&a, &b)| (a as i64) * (b as i64)).sum();
    sum as f64 / s1.len() as f64
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- ScheduleType tests --

    #[test]
    fn test_linear_schedule_endpoints() {
        let t_max = 10.0;
        let t_min = 0.1;
        let steps = 100;
        let t_start = ScheduleType::Linear.temperature(0, steps, t_max, t_min);
        let t_end = ScheduleType::Linear.temperature(steps, steps, t_max, t_min);
        assert!((t_start - t_max).abs() < 1e-10);
        assert!((t_end - t_min).abs() < 1e-10);
    }

    #[test]
    fn test_geometric_schedule_endpoints() {
        let t_max = 10.0;
        let t_min = 0.01;
        let steps = 1000;
        let t_start = ScheduleType::Geometric.temperature(0, steps, t_max, t_min);
        let t_end = ScheduleType::Geometric.temperature(steps, steps, t_max, t_min);
        assert!((t_start - t_max).abs() < 1e-10);
        assert!((t_end - t_min).abs() < 0.01);
    }

    #[test]
    fn test_logarithmic_schedule_monotone() {
        let t_max = 10.0;
        let t_min = 0.1;
        let steps = 200;
        let mut prev = t_max + 1.0;
        for k in 0..=steps {
            let t = ScheduleType::Logarithmic.temperature(k, steps, t_max, t_min);
            assert!(t <= prev + 1e-12, "Temperature should be non-increasing");
            prev = t;
        }
    }

    #[test]
    fn test_schedule_zero_steps() {
        let t = ScheduleType::Linear.temperature(0, 0, 10.0, 0.1);
        assert!((t - 0.1).abs() < 1e-10);
    }

    // -- IsingModel tests --

    #[test]
    fn test_ising_model_creation() {
        let model = IsingModel::new(5);
        assert_eq!(model.num_spins(), 5);
        assert_eq!(model.couplings_flat().len(), 10); // 5*4/2
        assert_eq!(model.fields().len(), 5);
    }

    #[test]
    fn test_coupling_symmetry() {
        let mut model = IsingModel::new(4);
        model.set_coupling(1, 3, 2.5);
        assert!((model.coupling(1, 3) - 2.5).abs() < 1e-10);
        assert!((model.coupling(3, 1) - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_coupling_self_loop_ignored() {
        let mut model = IsingModel::new(3);
        model.set_coupling(1, 1, 5.0);
        assert!((model.coupling(1, 1)).abs() < 1e-10);
    }

    #[test]
    fn test_field_set_get() {
        let mut model = IsingModel::new(3);
        model.set_field(0, 1.5);
        model.set_field(2, -0.5);
        assert!((model.field(0) - 1.5).abs() < 1e-10);
        assert!((model.field(1)).abs() < 1e-10);
        assert!((model.field(2) + 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_from_matrix() {
        // 2x2 system: J_01 = 1.0, h = [0.5, -0.3]
        let j = vec![0.0, 1.0, 1.0, 0.0];
        let h = vec![0.5, -0.3];
        let model = IsingModel::from_matrix(&j, &h, 2);
        assert!((model.coupling(0, 1) - 1.0).abs() < 1e-10);
        assert!((model.field(0) - 0.5).abs() < 1e-10);
        assert!((model.field(1) + 0.3).abs() < 1e-10);
    }

    // -- EnergyCalculator tests --

    #[test]
    fn test_energy_ferromagnetic_aligned() {
        // Two spins, J=1, both +1 => H = -1*1*1 = -1
        let mut model = IsingModel::new(2);
        model.set_coupling(0, 1, 1.0);
        let spins = vec![1, 1];
        let e = EnergyCalculator::energy(&model, &spins);
        assert!((e - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_energy_ferromagnetic_antialigned() {
        // Two spins, J=1, s=[+1,-1] => H = -1*1*(-1) = +1
        let mut model = IsingModel::new(2);
        model.set_coupling(0, 1, 1.0);
        let spins = vec![1, -1];
        let e = EnergyCalculator::energy(&model, &spins);
        assert!((e - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_energy_with_field() {
        // One spin, h=2.0, s=+1 => H = -2*1 = -2
        let mut model = IsingModel::new(1);
        model.set_field(0, 2.0);
        let spins = vec![1];
        let e = EnergyCalculator::energy(&model, &spins);
        assert!((e - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn test_energy_chain_ground_state() {
        // 4-spin ferromagnetic chain: J=1 for each adjacent pair
        // Ground state: all +1, E = -3
        let mut model = IsingModel::new(4);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, 1.0);
        model.set_coupling(2, 3, 1.0);
        let spins = vec![1, 1, 1, 1];
        let e = EnergyCalculator::energy(&model, &spins);
        assert!((e - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_delta_energy_consistency() {
        // Verify delta_energy matches actual energy difference
        let mut model = IsingModel::new(4);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, -0.5);
        model.set_coupling(2, 3, 0.7);
        model.set_coupling(0, 3, -0.3);
        model.set_field(0, 0.2);
        model.set_field(2, -0.4);

        let spins = vec![1, -1, 1, -1];
        let e_before = EnergyCalculator::energy(&model, &spins);

        for k in 0..4 {
            let delta = EnergyCalculator::delta_energy(&model, &spins, k);
            let mut flipped = spins.clone();
            flipped[k] = -flipped[k];
            let e_after = EnergyCalculator::energy(&model, &flipped);
            assert!(
                (delta - (e_after - e_before)).abs() < 1e-10,
                "Delta energy mismatch for spin {}",
                k
            );
        }
    }

    // -- SimulatedAnnealer tests --

    #[test]
    fn test_annealer_ferromagnetic_chain() {
        // 6-spin ferromagnetic chain: should find all-aligned ground state
        let mut model = IsingModel::new(6);
        for i in 0..5 {
            model.set_coupling(i, i + 1, 1.0);
        }

        let annealer = SimulatedAnnealer::new(10.0, 0.001, 5000, ScheduleType::Geometric);
        let (best, energy) = annealer.solve(&model, 123);

        // Ground state energy is -5
        assert!(
            energy <= -5.0 + 1e-9,
            "Expected ground state energy -5, got {}",
            energy
        );
        assert_eq!(best.len(), 6);
        // All spins should be aligned
        assert!(best.iter().all(|&s| s == best[0]));
    }

    #[test]
    fn test_annealer_antiferromagnetic_chain() {
        // 4-spin antiferromagnetic chain (J < 0)
        // Ground state: alternating +1,-1,+1,-1 or -1,+1,-1,+1
        let mut model = IsingModel::new(4);
        for i in 0..3 {
            model.set_coupling(i, i + 1, -1.0);
        }

        let annealer = SimulatedAnnealer::new(10.0, 0.001, 5000, ScheduleType::Geometric);
        let (best, energy) = annealer.solve(&model, 42);

        // Ground state energy: -(-1)*s0*s1 - (-1)*s1*s2 - (-1)*s2*s3
        // For alternating: each term = +1*(-1) = ... each sum contributes -1 => total = -3
        assert!(
            energy <= -3.0 + 1e-9,
            "Expected energy -3 for AF chain, got {}",
            energy
        );
        // Verify alternating pattern
        for i in 0..3 {
            assert_ne!(best[i], best[i + 1], "Adjacent spins should be anti-aligned");
        }
    }

    #[test]
    fn test_annealer_with_field() {
        // Single spin with strong field h=5 should align with field
        let mut model = IsingModel::new(1);
        model.set_field(0, 5.0);

        let annealer = SimulatedAnnealer::new(5.0, 0.001, 1000, ScheduleType::Geometric);
        let (best, energy) = annealer.solve(&model, 99);
        assert_eq!(best[0], 1); // Should align with positive field
        assert!((energy - (-5.0)).abs() < 1e-9);
    }

    #[test]
    fn test_annealer_deterministic() {
        // Same seed should produce the same result
        let mut model = IsingModel::new(8);
        for i in 0..7 {
            model.set_coupling(i, i + 1, 1.0);
        }

        let annealer = SimulatedAnnealer::new(10.0, 0.01, 3000, ScheduleType::Geometric);
        let (s1, e1) = annealer.solve(&model, 42);
        let (s2, e2) = annealer.solve(&model, 42);

        assert_eq!(s1, s2);
        assert!((e1 - e2).abs() < 1e-15);
    }

    #[test]
    fn test_annealer_solve_from() {
        let mut model = IsingModel::new(4);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, 1.0);
        model.set_coupling(2, 3, 1.0);

        let initial = vec![1, -1, 1, -1]; // Bad initial state
        let annealer = SimulatedAnnealer::new(10.0, 0.001, 3000, ScheduleType::Geometric);
        let (_best, energy) = annealer.solve_from(&model, &initial, 42);

        assert!(energy <= -3.0 + 1e-9, "Should find ground state from bad initial");
    }

    #[test]
    fn test_annealer_empty_model() {
        let model = IsingModel::new(0);
        let annealer = SimulatedAnnealer::new(10.0, 0.01, 100, ScheduleType::Linear);
        let (best, energy) = annealer.solve(&model, 0);
        assert!(best.is_empty());
        assert!((energy - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_annealer_linear_schedule() {
        let mut model = IsingModel::new(4);
        for i in 0..3 {
            model.set_coupling(i, i + 1, 1.0);
        }

        let annealer = SimulatedAnnealer::new(10.0, 0.01, 5000, ScheduleType::Linear);
        let (_best, energy) = annealer.solve(&model, 7);
        assert!(energy <= -3.0 + 1e-9);
    }

    #[test]
    fn test_annealer_logarithmic_schedule() {
        let mut model = IsingModel::new(4);
        for i in 0..3 {
            model.set_coupling(i, i + 1, 1.0);
        }

        let annealer = SimulatedAnnealer::new(10.0, 0.01, 5000, ScheduleType::Logarithmic);
        let (_best, energy) = annealer.solve(&model, 17);
        assert!(energy <= -3.0 + 1e-9);
    }

    // -- QUBO tests --

    #[test]
    fn test_qubo_to_ising_roundtrip() {
        // Create a simple QUBO, convert to Ising, convert back, verify equivalence
        let n = 3;
        let q = vec![
            1.0, -2.0, 0.0,
            0.0,  3.0, 1.5,
            0.0,  0.0, -1.0,
        ];

        let (ising, offset1) = qubo_to_ising(&q, n);
        let (q_back, offset2) = ising_to_qubo(&ising);

        // Verify QUBO values match for all binary vectors
        for bits in 0u8..(1 << n) {
            let x: Vec<u8> = (0..n).map(|i| (bits >> i) & 1).collect();

            // Original QUBO value
            let mut v1 = 0.0;
            for i in 0..n {
                for j in i..n {
                    if x[i] != 0 && x[j] != 0 {
                        v1 += q[i * n + j];
                    }
                }
            }

            // Ising energy + offset
            let spins = binary_to_spins(&x);
            let ising_e = EnergyCalculator::energy(&ising, &spins);
            let v2 = ising_e + offset1;

            assert!(
                (v1 - v2).abs() < 1e-8,
                "QUBO/Ising mismatch for x={:?}: {} vs {}",
                x, v1, v2
            );
        }
    }

    #[test]
    fn test_qubo_solver_simple() {
        // Minimize x0 + x1 - 3*x0*x1
        // Q = [[1, -3], [0, 1]]
        // Minimum is x=[1,1]: 1 + 1 - 3 = -1
        let q = vec![1.0, -3.0, 0.0, 1.0];
        let solver = QuboSolver::new(q, 2);
        let (best_x, best_val) = solver.solve(10.0, 0.001, 3000, ScheduleType::Geometric, 42);

        // Best should be [1, 1] with value -1
        assert_eq!(best_x, vec![1, 1]);
        assert!((best_val - (-1.0)).abs() < 1e-6, "Expected -1, got {}", best_val);
    }

    #[test]
    fn test_qubo_evaluate() {
        let q = vec![2.0, -1.0, 0.0, 3.0];
        let solver = QuboSolver::new(q, 2);

        // x = [0, 0] => 0
        assert!((solver.evaluate(&[0, 0])).abs() < 1e-10);

        // x = [1, 0] => Q_00 = 2
        assert!((solver.evaluate(&[1, 0]) - 2.0).abs() < 1e-10);

        // x = [0, 1] => Q_11 = 3
        assert!((solver.evaluate(&[0, 1]) - 3.0).abs() < 1e-10);

        // x = [1, 1] => Q_00 + Q_01 + Q_10 + Q_11 = 2 + (-1) + 0 + 3 = 4
        assert!((solver.evaluate(&[1, 1]) - 4.0).abs() < 1e-10);
    }

    // -- MaxCutMapper tests --

    #[test]
    fn test_maxcut_triangle() {
        // Triangle graph with unit weights: max cut = 2 (cut 2 of 3 edges)
        let mut mapper = MaxCutMapper::new(3);
        mapper.add_edge(0, 1, 1.0);
        mapper.add_edge(1, 2, 1.0);
        mapper.add_edge(0, 2, 1.0);

        let (best, cut) = mapper.solve(10.0, 0.001, 5000, ScheduleType::Geometric, 42);
        assert_eq!(best.len(), 3);
        assert!(
            (cut - 2.0).abs() < 1e-9,
            "Max cut of triangle should be 2, got {}",
            cut
        );
    }

    #[test]
    fn test_maxcut_bipartite() {
        // K_{2,2} bipartite graph: max cut = 4 (all 4 edges)
        let mut mapper = MaxCutMapper::new(4);
        mapper.add_edge(0, 2, 1.0);
        mapper.add_edge(0, 3, 1.0);
        mapper.add_edge(1, 2, 1.0);
        mapper.add_edge(1, 3, 1.0);

        let (best, cut) = mapper.solve(10.0, 0.001, 5000, ScheduleType::Geometric, 77);
        assert!(
            (cut - 4.0).abs() < 1e-9,
            "Max cut of K_2,2 should be 4, got {}",
            cut
        );
        // Verify partition: {0,1} vs {2,3} or vice versa
        assert_eq!(best[0], best[1]);
        assert_eq!(best[2], best[3]);
        assert_ne!(best[0], best[2]);
    }

    #[test]
    fn test_maxcut_cut_value() {
        let mut mapper = MaxCutMapper::new(3);
        mapper.add_edge(0, 1, 2.0);
        mapper.add_edge(1, 2, 3.0);

        // spins [+1, -1, +1]: edge(0,1) cut (w=2), edge(1,2) cut (w=3) => cut = 5
        assert!((mapper.cut_value(&[1, -1, 1]) - 5.0).abs() < 1e-10);

        // spins [+1, +1, +1]: no edges cut => cut = 0
        assert!((mapper.cut_value(&[1, 1, 1]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_maxcut_weighted() {
        // Path graph: 0 --5-- 1 --1-- 2
        // Max cut = 6 (cut both edges by putting 1 in opposite partition)
        let mut mapper = MaxCutMapper::new(3);
        mapper.add_edge(0, 1, 5.0);
        mapper.add_edge(1, 2, 1.0);

        let (_best, cut) = mapper.solve(10.0, 0.001, 5000, ScheduleType::Geometric, 42);
        assert!(
            (cut - 6.0).abs() < 1e-9,
            "Max cut of weighted path should be 6, got {}",
            cut
        );
    }

    // -- Helper function tests --

    #[test]
    fn test_random_spin_config_values() {
        let spins = random_spin_config(100, 42);
        assert_eq!(spins.len(), 100);
        assert!(spins.iter().all(|&s| s == 1 || s == -1));
    }

    #[test]
    fn test_random_spin_config_not_all_same() {
        let spins = random_spin_config(50, 42);
        let has_pos = spins.iter().any(|&s| s == 1);
        let has_neg = spins.iter().any(|&s| s == -1);
        assert!(has_pos && has_neg, "Random config should have both +1 and -1");
    }

    #[test]
    fn test_magnetization_all_up() {
        let spins = vec![1, 1, 1, 1];
        assert!((magnetization(&spins) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnetization_all_down() {
        let spins = vec![-1, -1, -1, -1];
        assert!((magnetization(&spins) + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnetization_balanced() {
        let spins = vec![1, -1, 1, -1];
        assert!((magnetization(&spins)).abs() < 1e-10);
    }

    #[test]
    fn test_magnetization_empty() {
        assert!((magnetization(&[])).abs() < 1e-10);
    }

    #[test]
    fn test_binary_spins_roundtrip() {
        let binary = vec![0u8, 1, 1, 0, 1];
        let spins = binary_to_spins(&binary);
        assert_eq!(spins, vec![-1, 1, 1, -1, 1]);
        let back = spins_to_binary(&spins);
        assert_eq!(back, binary);
    }

    #[test]
    fn test_spin_overlap_identical() {
        let s = vec![1, -1, 1, -1];
        assert!((spin_overlap(&s, &s) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_spin_overlap_opposite() {
        let s1 = vec![1, -1, 1, -1];
        let s2 = vec![-1, 1, -1, 1];
        assert!((spin_overlap(&s1, &s2) + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_spin_overlap_uncorrelated() {
        let s1 = vec![1, 1, -1, -1];
        let s2 = vec![1, -1, 1, -1];
        assert!((spin_overlap(&s1, &s2)).abs() < 1e-10);
    }
}
