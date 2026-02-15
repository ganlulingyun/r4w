//! # Quantum Annealing Optimizer
//!
//! Quantum annealing and simulated quantum annealing optimization for
//! combinatorial problems. Implements classical simulated annealing (SA)
//! and simulated quantum annealing (SQA) via path-integral Monte Carlo
//! with Trotter slices.
//!
//! ## Supported Problem Encodings
//! - **Ising model**: spins s_i in {-1, +1} with coupling J and local fields h
//! - **QUBO**: Quadratic Unconstrained Binary Optimization, x_i in {0, 1}
//! - **MaxCut**: graph partitioning to maximize cut edges
//! - **Number partitioning**: minimize |sum_A - sum_B| over two subsets
//! - **Graph 2-coloring**: penalty for monochromatic edges
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_annealing_optimizer::{
//!     IsingModel, SimulatedAnnealing, AnnealingConfig, ScheduleType,
//! };
//!
//! // Create a simple 3-spin antiferromagnetic chain
//! let mut model = IsingModel::new(3);
//! model.set_coupling(0, 1, -1.0);
//! model.set_coupling(1, 2, -1.0);
//!
//! let config = AnnealingConfig {
//!     initial_temp: 10.0,
//!     final_temp: 0.01,
//!     num_sweeps: 1000,
//!     schedule: ScheduleType::Exponential,
//!     seed: 42,
//! };
//!
//! let result = SimulatedAnnealing::run(&model, &config);
//! assert!(result.energy <= 1.0); // optimal energy is -1 for frustrated chain
//! ```

// ─── Simple LCG PRNG ───────────────────────────────────────────────────────

/// Linear congruential generator for reproducible pseudo-random numbers.
/// Uses the Numerical Recipes LCG parameters.
#[derive(Debug, Clone)]
pub struct Lcg {
    state: u64,
}

impl Lcg {
    /// Create a new LCG with the given seed.
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Generate next u64.
    pub fn next_u64(&mut self) -> u64 {
        // Numerical Recipes LCG
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    /// Generate a uniform f64 in [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Generate a uniform usize in [0, n).
    pub fn next_usize(&mut self, n: usize) -> usize {
        (self.next_f64() * n as f64) as usize % n
    }

    /// Generate a random spin: -1 or +1.
    pub fn next_spin(&mut self) -> i8 {
        if self.next_u64() & 1 == 0 { 1 } else { -1 }
    }
}

// ─── Ising Model ────────────────────────────────────────────────────────────

/// Ising model: E = -sum_{i<j} J_ij * s_i * s_j - sum_i h_i * s_i
///
/// Spins s_i take values in {-1, +1}.
#[derive(Debug, Clone)]
pub struct IsingModel {
    /// Number of spins.
    pub num_spins: usize,
    /// Coupling matrix J (symmetric, stored as full NxN for fast access).
    /// J[i * num_spins + j] = coupling between spin i and spin j.
    pub j_matrix: Vec<f64>,
    /// Local fields h.
    pub h_fields: Vec<f64>,
}

impl IsingModel {
    /// Create a new Ising model with N spins, zero couplings and fields.
    pub fn new(num_spins: usize) -> Self {
        Self {
            num_spins,
            j_matrix: vec![0.0; num_spins * num_spins],
            h_fields: vec![0.0; num_spins],
        }
    }

    /// Set coupling J_ij (symmetric: also sets J_ji).
    pub fn set_coupling(&mut self, i: usize, j: usize, value: f64) {
        assert!(i < self.num_spins && j < self.num_spins);
        self.j_matrix[i * self.num_spins + j] = value;
        self.j_matrix[j * self.num_spins + i] = value;
    }

    /// Get coupling J_ij.
    pub fn get_coupling(&self, i: usize, j: usize) -> f64 {
        self.j_matrix[i * self.num_spins + j]
    }

    /// Set local field h_i.
    pub fn set_field(&mut self, i: usize, value: f64) {
        assert!(i < self.num_spins);
        self.h_fields[i] = value;
    }

    /// Compute energy for a spin configuration.
    /// E = -sum_{i<j} J_ij * s_i * s_j - sum_i h_i * s_i
    pub fn energy(&self, spins: &[i8]) -> f64 {
        assert_eq!(spins.len(), self.num_spins);
        let n = self.num_spins;
        let mut e = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                e -= self.j_matrix[i * n + j] * (spins[i] as f64) * (spins[j] as f64);
            }
            e -= self.h_fields[i] * (spins[i] as f64);
        }
        e
    }

    /// Compute energy change when flipping spin at index `k`.
    /// delta_E = E_new - E_old = 2 * s_k * (sum_j J_kj * s_j + h_k)
    pub fn delta_energy_flip(&self, spins: &[i8], k: usize) -> f64 {
        let n = self.num_spins;
        let sk = spins[k] as f64;
        let mut local_field = self.h_fields[k];
        for j in 0..n {
            if j != k {
                local_field += self.j_matrix[k * n + j] * (spins[j] as f64);
            }
        }
        2.0 * sk * local_field
    }

    /// Generate a random Ising instance with couplings in [-1, 1] and fields in [-0.5, 0.5].
    pub fn random(num_spins: usize, seed: u64) -> Self {
        let mut rng = Lcg::new(seed);
        let mut model = Self::new(num_spins);
        for i in 0..num_spins {
            for j in (i + 1)..num_spins {
                let val = rng.next_f64() * 2.0 - 1.0;
                model.set_coupling(i, j, val);
            }
            model.h_fields[i] = rng.next_f64() - 0.5;
        }
        model
    }

    /// Generate a random spin configuration.
    pub fn random_spins(&self, rng: &mut Lcg) -> Vec<i8> {
        (0..self.num_spins).map(|_| rng.next_spin()).collect()
    }

    /// Convert from QUBO model. The relationship is s_i = 2*x_i - 1.
    ///
    /// QUBO: E = sum_{i<=j} Q_ij * x_i * x_j
    /// Ising: E = -sum_{i<j} J_ij * s_i * s_j - sum_i h_i * s_i + const
    ///
    /// Substituting x_i = (s_i + 1)/2:
    ///   J_ij = -Q_ij / 4 (for i != j)
    ///   h_i  = -(Q_ii / 2 + sum_{j!=i} Q_ij / 4)
    pub fn from_qubo(qubo: &QuboModel) -> Self {
        let n = qubo.num_variables;
        let mut model = Self::new(n);
        for i in 0..n {
            let mut hi = qubo.get(i, i) / 2.0;
            for j in 0..n {
                if j != i {
                    let qij = if i < j { qubo.get(i, j) } else { qubo.get(j, i) };
                    hi += qij / 4.0;
                }
            }
            model.h_fields[i] = -hi;

            for j in (i + 1)..n {
                model.set_coupling(i, j, -qubo.get(i, j) / 4.0);
            }
        }
        model
    }
}

// ─── QUBO Model ─────────────────────────────────────────────────────────────

/// Quadratic Unconstrained Binary Optimization model.
/// E = sum_{i<=j} Q_ij * x_i * x_j, where x_i in {0, 1}.
///
/// The Q matrix is stored as upper triangular (diagonal terms represent linear
/// coefficients since x_i^2 = x_i for binary variables).
#[derive(Debug, Clone)]
pub struct QuboModel {
    /// Number of binary variables.
    pub num_variables: usize,
    /// Upper triangular Q matrix, stored in row-major as full NxN.
    /// Only entries with i <= j are meaningful.
    pub q_matrix: Vec<f64>,
}

impl QuboModel {
    /// Create a new QUBO model with N variables, all-zero Q.
    pub fn new(num_variables: usize) -> Self {
        Self {
            num_variables,
            q_matrix: vec![0.0; num_variables * num_variables],
        }
    }

    /// Set Q_ij. For off-diagonal, only i < j is stored.
    pub fn set(&mut self, i: usize, j: usize, value: f64) {
        assert!(i < self.num_variables && j < self.num_variables);
        if i <= j {
            self.q_matrix[i * self.num_variables + j] = value;
        } else {
            self.q_matrix[j * self.num_variables + i] = value;
        }
    }

    /// Get Q_ij. Returns the upper-triangular value.
    pub fn get(&self, i: usize, j: usize) -> f64 {
        if i <= j {
            self.q_matrix[i * self.num_variables + j]
        } else {
            self.q_matrix[j * self.num_variables + i]
        }
    }

    /// Compute QUBO energy for a binary assignment.
    pub fn energy(&self, x: &[u8]) -> f64 {
        assert_eq!(x.len(), self.num_variables);
        let n = self.num_variables;
        let mut e = 0.0;
        for i in 0..n {
            for j in i..n {
                e += self.q_matrix[i * n + j] * (x[i] as f64) * (x[j] as f64);
            }
        }
        e
    }

    /// Convert to Ising model.
    pub fn to_ising(&self) -> IsingModel {
        IsingModel::from_qubo(self)
    }

    /// Convert Ising spins to QUBO binary variables: x_i = (s_i + 1) / 2.
    pub fn spins_to_binary(spins: &[i8]) -> Vec<u8> {
        spins.iter().map(|&s| if s == 1 { 1 } else { 0 }).collect()
    }

    /// Convert QUBO binary to Ising spins: s_i = 2*x_i - 1.
    pub fn binary_to_spins(x: &[u8]) -> Vec<i8> {
        x.iter().map(|&xi| if xi == 1 { 1 } else { -1 }).collect()
    }
}

// ─── Annealing Schedule ─────────────────────────────────────────────────────

/// Type of annealing schedule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScheduleType {
    /// Linear: T(t) = T_init - (T_init - T_final) * t / total
    Linear,
    /// Exponential: T(t) = T_init * (T_final / T_init)^(t / total)
    Exponential,
    /// Logarithmic: T(t) = T_init / (1 + alpha * ln(1 + t))
    /// where alpha is chosen so T(total) = T_final.
    Logarithmic,
}

/// Annealing schedule trait.
pub trait AnnealingSchedule {
    /// Compute the schedule value at the given step.
    fn value_at(&self, step: usize, total_steps: usize) -> f64;
}

/// Linear annealing schedule.
#[derive(Debug, Clone)]
pub struct LinearSchedule {
    pub initial: f64,
    pub final_val: f64,
}

impl AnnealingSchedule for LinearSchedule {
    fn value_at(&self, step: usize, total_steps: usize) -> f64 {
        if total_steps == 0 {
            return self.final_val;
        }
        let frac = step as f64 / total_steps as f64;
        self.initial + (self.final_val - self.initial) * frac
    }
}

/// Exponential annealing schedule.
#[derive(Debug, Clone)]
pub struct ExponentialSchedule {
    pub initial: f64,
    pub final_val: f64,
}

impl AnnealingSchedule for ExponentialSchedule {
    fn value_at(&self, step: usize, total_steps: usize) -> f64 {
        if total_steps == 0 {
            return self.final_val;
        }
        let frac = step as f64 / total_steps as f64;
        self.initial * (self.final_val / self.initial).powf(frac)
    }
}

/// Logarithmic annealing schedule.
#[derive(Debug, Clone)]
pub struct LogarithmicSchedule {
    pub initial: f64,
    pub final_val: f64,
}

impl AnnealingSchedule for LogarithmicSchedule {
    fn value_at(&self, step: usize, total_steps: usize) -> f64 {
        if total_steps == 0 {
            return self.final_val;
        }
        // T(t) = T_init / (1 + alpha * ln(1 + t))
        // At t = total: T_final = T_init / (1 + alpha * ln(1 + total))
        // => alpha = (T_init / T_final - 1) / ln(1 + total)
        let ln_total = ((1 + total_steps) as f64).ln();
        let alpha = (self.initial / self.final_val - 1.0) / ln_total;
        let ln_step = ((1 + step) as f64).ln();
        self.initial / (1.0 + alpha * ln_step)
    }
}

/// Create a boxed schedule from type and parameters.
fn make_schedule(stype: ScheduleType, initial: f64, final_val: f64) -> Box<dyn AnnealingSchedule> {
    match stype {
        ScheduleType::Linear => Box::new(LinearSchedule { initial, final_val }),
        ScheduleType::Exponential => Box::new(ExponentialSchedule { initial, final_val }),
        ScheduleType::Logarithmic => Box::new(LogarithmicSchedule { initial, final_val }),
    }
}

// ─── Annealing Configuration ────────────────────────────────────────────────

/// Configuration for simulated annealing.
#[derive(Debug, Clone)]
pub struct AnnealingConfig {
    /// Initial temperature.
    pub initial_temp: f64,
    /// Final temperature.
    pub final_temp: f64,
    /// Number of sweeps (each sweep flips N spins).
    pub num_sweeps: usize,
    /// Schedule type.
    pub schedule: ScheduleType,
    /// Random seed.
    pub seed: u64,
}

/// Result of an annealing run.
#[derive(Debug, Clone)]
pub struct AnnealingResult {
    /// Best spin configuration found.
    pub best_spins: Vec<i8>,
    /// Energy of the best configuration.
    pub energy: f64,
    /// Number of sweeps performed.
    pub sweeps_performed: usize,
    /// Energy history (sampled at each sweep).
    pub energy_history: Vec<f64>,
}

// ─── Simulated Annealing ────────────────────────────────────────────────────

/// Classical simulated annealing solver.
pub struct SimulatedAnnealing;

impl SimulatedAnnealing {
    /// Run simulated annealing on an Ising model.
    pub fn run(model: &IsingModel, config: &AnnealingConfig) -> AnnealingResult {
        let n = model.num_spins;
        let mut rng = Lcg::new(config.seed);
        let schedule = make_schedule(config.schedule, config.initial_temp, config.final_temp);

        // Initialize random spin configuration
        let mut spins = model.random_spins(&mut rng);
        let mut current_energy = model.energy(&spins);

        let mut best_spins = spins.clone();
        let mut best_energy = current_energy;
        let mut energy_history = Vec::with_capacity(config.num_sweeps);

        for sweep in 0..config.num_sweeps {
            let temp = schedule.value_at(sweep, config.num_sweeps.saturating_sub(1));

            // One sweep: attempt to flip each spin once
            for _ in 0..n {
                let k = rng.next_usize(n);
                let delta_e = model.delta_energy_flip(&spins, k);

                // Metropolis acceptance
                let accept = if delta_e <= 0.0 {
                    true
                } else if temp > 1e-15 {
                    rng.next_f64() < (-delta_e / temp).exp()
                } else {
                    false
                };

                if accept {
                    spins[k] = -spins[k];
                    current_energy += delta_e;
                }
            }

            energy_history.push(current_energy);

            if current_energy < best_energy {
                best_energy = current_energy;
                best_spins = spins.clone();
            }
        }

        AnnealingResult {
            best_spins,
            energy: best_energy,
            sweeps_performed: config.num_sweeps,
            energy_history,
        }
    }

    /// Run multiple restarts and return the best result.
    pub fn run_restarts(
        model: &IsingModel,
        config: &AnnealingConfig,
        num_restarts: usize,
    ) -> AnnealingResult {
        let mut best_result: Option<AnnealingResult> = None;

        for restart in 0..num_restarts {
            let mut cfg = config.clone();
            cfg.seed = config.seed.wrapping_add(restart as u64 * 12345);
            let result = Self::run(model, &cfg);

            if best_result.is_none() || result.energy < best_result.as_ref().unwrap().energy {
                best_result = Some(result);
            }
        }

        best_result.unwrap()
    }
}

// ─── Simulated Quantum Annealing ────────────────────────────────────────────

/// Configuration for simulated quantum annealing.
#[derive(Debug, Clone)]
pub struct SqaConfig {
    /// Number of Trotter slices (replicas).
    pub num_trotter_slices: usize,
    /// Initial transverse field strength Gamma.
    pub gamma_initial: f64,
    /// Final transverse field strength Gamma.
    pub gamma_final: f64,
    /// Physical temperature T.
    pub temperature: f64,
    /// Number of sweeps.
    pub num_sweeps: usize,
    /// Gamma schedule type.
    pub gamma_schedule: ScheduleType,
    /// Random seed.
    pub seed: u64,
}

/// Simulated Quantum Annealing via path-integral Monte Carlo.
///
/// Uses P Trotter slices (replicas) coupled along the imaginary-time
/// direction. The transverse field Gamma decreases over time, driving
/// the system from a quantum superposition to a classical ground state.
pub struct SimulatedQuantumAnnealing;

impl SimulatedQuantumAnnealing {
    /// Run SQA on an Ising model.
    pub fn run(model: &IsingModel, config: &SqaConfig) -> AnnealingResult {
        let n = model.num_spins;
        let p = config.num_trotter_slices;
        let temp = config.temperature;
        let mut rng = Lcg::new(config.seed);
        let gamma_schedule = make_schedule(
            config.gamma_schedule,
            config.gamma_initial,
            config.gamma_final,
        );

        // Initialize P replicas with random spins
        // replicas[k][i] = spin i in replica k
        let mut replicas: Vec<Vec<i8>> = (0..p)
            .map(|_| model.random_spins(&mut rng))
            .collect();

        let mut best_spins = replicas[0].clone();
        let mut best_energy = model.energy(&best_spins);
        let mut energy_history = Vec::with_capacity(config.num_sweeps);

        // Effective temperature for each replica: T_eff = P * T
        let t_eff = p as f64 * temp;

        for sweep in 0..config.num_sweeps {
            let gamma = gamma_schedule.value_at(sweep, config.num_sweeps.saturating_sub(1));

            // Inter-replica coupling: J_perp = -(T/2) * ln(tanh(Gamma / (P * T)))
            let j_perp = {
                let arg = gamma / (p as f64 * temp);
                if arg > 20.0 {
                    // tanh(arg) ≈ 1, ln(1) = 0, so J_perp ≈ 0 (but really gamma dominates)
                    // For large arg, tanh(x) ≈ 1 - 2*exp(-2x)
                    // ln(tanh(x)) ≈ -2*exp(-2x)
                    // J_perp ≈ (T/2) * 2 * exp(-2*arg) = T * exp(-2*arg)
                    temp * (-2.0 * arg).exp()
                } else if arg < 1e-15 {
                    0.0
                } else {
                    -(temp / 2.0) * arg.tanh().ln()
                }
            };

            // Sweep through all replicas
            for k in 0..p {
                for _ in 0..n {
                    let i = rng.next_usize(n);
                    let s_ik = replicas[k][i] as f64;

                    // Classical energy change from flipping spin i in replica k
                    // Scaled by 1/P for the Trotter decomposition
                    let delta_class = model.delta_energy_flip(&replicas[k], i) / p as f64;

                    // Inter-replica coupling energy change
                    let k_prev = if k == 0 { p - 1 } else { k - 1 };
                    let k_next = if k == p - 1 { 0 } else { k + 1 };
                    let s_prev = replicas[k_prev][i] as f64;
                    let s_next = replicas[k_next][i] as f64;
                    // Before flip: -J_perp * s_ik * (s_prev + s_next)
                    // After flip:  -J_perp * (-s_ik) * (s_prev + s_next)
                    // delta = 2 * J_perp * s_ik * (s_prev + s_next)
                    let delta_quantum = 2.0 * j_perp * s_ik * (s_prev + s_next);

                    let delta_total = delta_class + delta_quantum;

                    // Metropolis acceptance at physical temperature T
                    let accept = if delta_total <= 0.0 {
                        true
                    } else if temp > 1e-15 {
                        rng.next_f64() < (-delta_total / temp).exp()
                    } else {
                        false
                    };

                    if accept {
                        replicas[k][i] = -replicas[k][i];
                    }
                }
            }

            // Find best replica
            let mut sweep_best_energy = f64::INFINITY;
            for k in 0..p {
                let e = model.energy(&replicas[k]);
                if e < sweep_best_energy {
                    sweep_best_energy = e;
                }
                if e < best_energy {
                    best_energy = e;
                    best_spins = replicas[k].clone();
                }
            }
            energy_history.push(sweep_best_energy);
        }

        AnnealingResult {
            best_spins,
            energy: best_energy,
            sweeps_performed: config.num_sweeps,
            energy_history,
        }
    }
}

// ─── Problem Encodings ──────────────────────────────────────────────────────

/// MaxCut problem: partition graph vertices into two sets to maximize cut edges.
///
/// Given a graph with adjacency weights w_ij, find partition S, S_bar that
/// maximizes sum_{(i,j) in cut} w_ij.
///
/// Ising encoding: maximize sum_{i<j} w_ij * (1 - s_i * s_j) / 2
/// Equivalently, minimize sum w_ij * s_i * s_j.
/// Since Ising E = -sum J_ij s_i s_j, we need J_ij = -w_ij so that
/// minimizing E is equivalent to minimizing sum w_ij s_i s_j.
pub fn encode_maxcut(num_vertices: usize, edges: &[(usize, usize, f64)]) -> IsingModel {
    let mut model = IsingModel::new(num_vertices);
    for &(i, j, weight) in edges {
        model.set_coupling(i, j, -weight);
    }
    model
}

/// Evaluate the MaxCut value for a given spin configuration.
/// Cut value = sum of weights of edges crossing the partition.
pub fn maxcut_value(edges: &[(usize, usize, f64)], spins: &[i8]) -> f64 {
    let mut cut = 0.0;
    for &(i, j, weight) in edges {
        if spins[i] != spins[j] {
            cut += weight;
        }
    }
    cut
}

/// Number partitioning: partition a set of numbers into two subsets
/// minimizing |sum_A - sum_B|.
///
/// Ising encoding: minimize (sum_i n_i * s_i)^2 = sum_{i,j} n_i * n_j * s_i * s_j
/// So J_ij = -n_i * n_j for i != j (we want to minimize, and energy is negated).
pub fn encode_number_partition(numbers: &[f64]) -> IsingModel {
    let n = numbers.len();
    let mut model = IsingModel::new(n);
    for i in 0..n {
        for j in (i + 1)..n {
            // We want to minimize (sum n_i s_i)^2.
            // Ising energy = -sum J_ij s_i s_j.
            // We want to minimize sum n_i n_j s_i s_j = -sum (-n_i n_j) s_i s_j.
            // So set J_ij = -n_i * n_j, then E = sum n_i n_j s_i s_j (up to constant).
            model.set_coupling(i, j, -numbers[i] * numbers[j]);
        }
    }
    model
}

/// Evaluate number partition imbalance |sum_A - sum_B| for a spin configuration.
pub fn partition_imbalance(numbers: &[f64], spins: &[i8]) -> f64 {
    let diff: f64 = numbers
        .iter()
        .zip(spins.iter())
        .map(|(&n, &s)| n * s as f64)
        .sum();
    diff.abs()
}

/// Graph 2-coloring: assign two colors to vertices, penalizing same-color
/// adjacent vertices.
///
/// Ising encoding: minimize number of monochromatic edges.
/// For each edge (i,j): penalty when s_i == s_j, i.e., (1 + s_i * s_j) / 2.
/// Minimize sum_{(i,j)} (1 + s_i * s_j) / 2 ≡ minimize sum s_i * s_j.
/// Ising: minimize -sum (-1) * s_i * s_j, so J_ij = -1 for each edge.
pub fn encode_graph_coloring_2(num_vertices: usize, edges: &[(usize, usize)]) -> IsingModel {
    let mut model = IsingModel::new(num_vertices);
    for &(i, j) in edges {
        model.set_coupling(i, j, -1.0);
    }
    model
}

/// Count monochromatic edges (violations) for a 2-coloring.
pub fn coloring_violations(edges: &[(usize, usize)], spins: &[i8]) -> usize {
    edges.iter().filter(|&&(i, j)| spins[i] == spins[j]).count()
}

// ─── Solution Analysis ──────────────────────────────────────────────────────

/// Check if a 2-coloring solution is valid (no monochromatic edges).
pub fn is_valid_2_coloring(edges: &[(usize, usize)], spins: &[i8]) -> bool {
    coloring_violations(edges, spins) == 0
}

/// Check if a number partition has zero imbalance.
pub fn is_perfect_partition(numbers: &[f64], spins: &[i8]) -> bool {
    partition_imbalance(numbers, spins) < 1e-10
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // --- LCG tests ---

    #[test]
    fn test_lcg_reproducibility() {
        let mut rng1 = Lcg::new(42);
        let mut rng2 = Lcg::new(42);
        for _ in 0..100 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_lcg_range() {
        let mut rng = Lcg::new(123);
        for _ in 0..1000 {
            let v = rng.next_f64();
            assert!(v >= 0.0 && v < 1.0);
        }
    }

    #[test]
    fn test_lcg_next_usize() {
        let mut rng = Lcg::new(7);
        for _ in 0..1000 {
            let v = rng.next_usize(10);
            assert!(v < 10);
        }
    }

    #[test]
    fn test_lcg_next_spin() {
        let mut rng = Lcg::new(99);
        let mut count_up = 0;
        let mut count_down = 0;
        for _ in 0..1000 {
            match rng.next_spin() {
                1 => count_up += 1,
                -1 => count_down += 1,
                _ => panic!("Invalid spin value"),
            }
        }
        // Both should appear (not all one value)
        assert!(count_up > 100);
        assert!(count_down > 100);
    }

    // --- Ising Model tests ---

    #[test]
    fn test_ising_energy_ferromagnetic() {
        // Two spins with J=1: E = -J * s1 * s2
        let mut model = IsingModel::new(2);
        model.set_coupling(0, 1, 1.0);

        // Aligned: E = -1
        assert!((model.energy(&[1, 1]) - (-1.0)).abs() < 1e-10);
        assert!((model.energy(&[-1, -1]) - (-1.0)).abs() < 1e-10);

        // Anti-aligned: E = 1
        assert!((model.energy(&[1, -1]) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ising_energy_with_field() {
        // Single spin with h=2: E = -h * s = -2s
        let mut model = IsingModel::new(1);
        model.set_field(0, 2.0);

        assert!((model.energy(&[1]) - (-2.0)).abs() < 1e-10);
        assert!((model.energy(&[-1]) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_ising_energy_three_spin_chain() {
        // Chain: J01 = 1, J12 = 1, all ferromagnetic
        let mut model = IsingModel::new(3);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, 1.0);

        // All aligned: E = -1 - 1 = -2
        assert!((model.energy(&[1, 1, 1]) - (-2.0)).abs() < 1e-10);
        // Frustrated: E = -1 + 1 = 0 or +1 -1 = 0
        assert!((model.energy(&[1, 1, -1]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_ising_delta_energy() {
        let mut model = IsingModel::new(3);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, -0.5);
        model.set_field(0, 0.3);

        let spins = vec![1, -1, 1];
        let e_before = model.energy(&spins);

        for k in 0..3 {
            let delta = model.delta_energy_flip(&spins, k);
            let mut flipped = spins.clone();
            flipped[k] = -flipped[k];
            let e_after = model.energy(&flipped);
            assert!(
                (delta - (e_after - e_before)).abs() < 1e-10,
                "Delta energy mismatch for spin {}",
                k
            );
        }
    }

    #[test]
    fn test_ising_symmetric_coupling() {
        let mut model = IsingModel::new(3);
        model.set_coupling(0, 2, 0.7);
        assert!((model.get_coupling(0, 2) - 0.7).abs() < 1e-10);
        assert!((model.get_coupling(2, 0) - 0.7).abs() < 1e-10);
    }

    #[test]
    fn test_ising_random_instance() {
        let model = IsingModel::random(5, 42);
        assert_eq!(model.num_spins, 5);
        // Check symmetry
        for i in 0..5 {
            for j in 0..5 {
                assert!((model.get_coupling(i, j) - model.get_coupling(j, i)).abs() < 1e-10);
            }
        }
    }

    // --- QUBO tests ---

    #[test]
    fn test_qubo_energy() {
        let mut qubo = QuboModel::new(2);
        qubo.set(0, 0, -1.0); // -x0
        qubo.set(1, 1, -1.0); // -x1
        qubo.set(0, 1, 2.0);  // +2*x0*x1

        // x=[0,0]: E = 0
        assert!((qubo.energy(&[0, 0]) - 0.0).abs() < 1e-10);
        // x=[1,0]: E = -1
        assert!((qubo.energy(&[1, 0]) - (-1.0)).abs() < 1e-10);
        // x=[0,1]: E = -1
        assert!((qubo.energy(&[0, 1]) - (-1.0)).abs() < 1e-10);
        // x=[1,1]: E = -1 - 1 + 2 = 0
        assert!((qubo.energy(&[1, 1]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_qubo_symmetric_access() {
        let mut qubo = QuboModel::new(3);
        qubo.set(2, 1, 3.0);
        assert!((qubo.get(1, 2) - 3.0).abs() < 1e-10);
        assert!((qubo.get(2, 1) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_qubo_to_ising_roundtrip() {
        // Create QUBO, convert to Ising, verify energies match up to constant
        let mut qubo = QuboModel::new(3);
        qubo.set(0, 0, -2.0);
        qubo.set(1, 1, -3.0);
        qubo.set(2, 2, -1.0);
        qubo.set(0, 1, 1.0);
        qubo.set(0, 2, 0.5);
        qubo.set(1, 2, -0.5);

        let ising = qubo.to_ising();

        // Check that energy differences are consistent
        // QUBO E(x) and Ising E(s) should differ by a constant for all configs
        let configs: Vec<Vec<u8>> = vec![
            vec![0, 0, 0],
            vec![1, 0, 0],
            vec![0, 1, 0],
            vec![0, 0, 1],
            vec![1, 1, 0],
            vec![1, 0, 1],
            vec![0, 1, 1],
            vec![1, 1, 1],
        ];

        let mut offsets = Vec::new();
        for x in &configs {
            let spins = QuboModel::binary_to_spins(x);
            let qubo_e = qubo.energy(x);
            let ising_e = ising.energy(&spins);
            offsets.push(qubo_e - ising_e);
        }

        // All offsets should be the same constant
        for i in 1..offsets.len() {
            assert!(
                (offsets[i] - offsets[0]).abs() < 1e-10,
                "QUBO-Ising offset mismatch: {} vs {}",
                offsets[i],
                offsets[0]
            );
        }
    }

    #[test]
    fn test_binary_spin_conversion() {
        let x = vec![0, 1, 0, 1, 1];
        let s = QuboModel::binary_to_spins(&x);
        assert_eq!(s, vec![-1, 1, -1, 1, 1]);

        let x_back = QuboModel::spins_to_binary(&s);
        assert_eq!(x_back, x);
    }

    // --- Annealing Schedule tests ---

    #[test]
    fn test_linear_schedule_endpoints() {
        let sched = LinearSchedule {
            initial: 10.0,
            final_val: 1.0,
        };
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);
        assert!((sched.value_at(100, 100) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_schedule_midpoint() {
        let sched = LinearSchedule {
            initial: 10.0,
            final_val: 0.0,
        };
        assert!((sched.value_at(50, 100) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_exponential_schedule_endpoints() {
        let sched = ExponentialSchedule {
            initial: 10.0,
            final_val: 0.01,
        };
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);
        assert!((sched.value_at(100, 100) - 0.01).abs() < 1e-6);
    }

    #[test]
    fn test_exponential_schedule_monotone() {
        let sched = ExponentialSchedule {
            initial: 10.0,
            final_val: 0.1,
        };
        let mut prev = sched.value_at(0, 100);
        for step in 1..=100 {
            let val = sched.value_at(step, 100);
            assert!(val <= prev + 1e-10, "Non-monotone at step {}", step);
            prev = val;
        }
    }

    #[test]
    fn test_logarithmic_schedule_endpoints() {
        let sched = LogarithmicSchedule {
            initial: 10.0,
            final_val: 1.0,
        };
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);
        assert!((sched.value_at(100, 100) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_logarithmic_schedule_monotone() {
        let sched = LogarithmicSchedule {
            initial: 10.0,
            final_val: 0.5,
        };
        let mut prev = sched.value_at(0, 200);
        for step in 1..=200 {
            let val = sched.value_at(step, 200);
            assert!(val <= prev + 1e-10, "Non-monotone at step {}", step);
            prev = val;
        }
    }

    // --- Simulated Annealing tests ---

    #[test]
    fn test_sa_two_spin_ferromagnetic() {
        // Ground state should be both spins aligned
        let mut model = IsingModel::new(2);
        model.set_coupling(0, 1, 1.0);

        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        assert!((result.energy - (-1.0)).abs() < 1e-10);
        assert_eq!(result.best_spins[0], result.best_spins[1]);
    }

    #[test]
    fn test_sa_single_spin_with_field() {
        // h = 5.0, ground state is s = +1, E = -5
        let mut model = IsingModel::new(1);
        model.set_field(0, 5.0);

        let config = AnnealingConfig {
            initial_temp: 10.0,
            final_temp: 0.001,
            num_sweeps: 200,
            schedule: ScheduleType::Linear,
            seed: 7,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        assert!((result.energy - (-5.0)).abs() < 1e-10);
        assert_eq!(result.best_spins[0], 1);
    }

    #[test]
    fn test_sa_energy_decreasing_trend() {
        let model = IsingModel::random(8, 123);
        let config = AnnealingConfig {
            initial_temp: 10.0,
            final_temp: 0.01,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };
        let result = SimulatedAnnealing::run(&model, &config);

        // Compare average of first 50 vs last 50 sweeps
        let first_avg: f64 =
            result.energy_history[..50].iter().sum::<f64>() / 50.0;
        let last_avg: f64 = result.energy_history[result.energy_history.len() - 50..]
            .iter()
            .sum::<f64>()
            / 50.0;
        assert!(
            last_avg <= first_avg + 1.0,
            "Energy should generally decrease: first_avg={}, last_avg={}",
            first_avg,
            last_avg
        );
    }

    #[test]
    fn test_sa_restarts_improve() {
        let model = IsingModel::random(10, 77);
        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.01,
            num_sweeps: 200,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let single = SimulatedAnnealing::run(&model, &config);
        let multi = SimulatedAnnealing::run_restarts(&model, &config, 10);

        // Multi-restart should be at least as good
        assert!(multi.energy <= single.energy + 1e-10);
    }

    #[test]
    fn test_sa_different_schedules() {
        let model = IsingModel::random(6, 55);

        for schedule in [ScheduleType::Linear, ScheduleType::Exponential, ScheduleType::Logarithmic] {
            let config = AnnealingConfig {
                initial_temp: 5.0,
                final_temp: 0.01,
                num_sweeps: 300,
                schedule,
                seed: 42,
            };
            let result = SimulatedAnnealing::run(&model, &config);
            // Just verify it runs and produces valid output
            assert_eq!(result.best_spins.len(), 6);
            assert_eq!(result.sweeps_performed, 300);
            assert_eq!(result.energy_history.len(), 300);
        }
    }

    #[test]
    fn test_sa_reproducible() {
        let model = IsingModel::random(5, 33);
        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.01,
            num_sweeps: 100,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let r1 = SimulatedAnnealing::run(&model, &config);
        let r2 = SimulatedAnnealing::run(&model, &config);
        assert_eq!(r1.best_spins, r2.best_spins);
        assert!((r1.energy - r2.energy).abs() < 1e-10);
    }

    // --- SQA tests ---

    #[test]
    fn test_sqa_two_spin_ferromagnetic() {
        let mut model = IsingModel::new(2);
        model.set_coupling(0, 1, 1.0);

        let config = SqaConfig {
            num_trotter_slices: 4,
            gamma_initial: 5.0,
            gamma_final: 0.01,
            temperature: 0.5,
            num_sweeps: 500,
            gamma_schedule: ScheduleType::Linear,
            seed: 42,
        };

        let result = SimulatedQuantumAnnealing::run(&model, &config);
        assert!((result.energy - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_sqa_single_spin_field() {
        let mut model = IsingModel::new(1);
        model.set_field(0, 3.0);

        let config = SqaConfig {
            num_trotter_slices: 8,
            gamma_initial: 3.0,
            gamma_final: 0.001,
            temperature: 0.5,
            num_sweeps: 300,
            gamma_schedule: ScheduleType::Exponential,
            seed: 99,
        };

        let result = SimulatedQuantumAnnealing::run(&model, &config);
        assert!((result.energy - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_sqa_reproducible() {
        let model = IsingModel::random(4, 11);
        let config = SqaConfig {
            num_trotter_slices: 4,
            gamma_initial: 3.0,
            gamma_final: 0.01,
            temperature: 0.5,
            num_sweeps: 200,
            gamma_schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let r1 = SimulatedQuantumAnnealing::run(&model, &config);
        let r2 = SimulatedQuantumAnnealing::run(&model, &config);
        assert_eq!(r1.best_spins, r2.best_spins);
        assert!((r1.energy - r2.energy).abs() < 1e-10);
    }

    #[test]
    fn test_sqa_energy_history_length() {
        let model = IsingModel::random(3, 7);
        let config = SqaConfig {
            num_trotter_slices: 4,
            gamma_initial: 2.0,
            gamma_final: 0.01,
            temperature: 0.5,
            num_sweeps: 100,
            gamma_schedule: ScheduleType::Linear,
            seed: 42,
        };
        let result = SimulatedQuantumAnnealing::run(&model, &config);
        assert_eq!(result.energy_history.len(), 100);
    }

    // --- MaxCut tests ---

    #[test]
    fn test_maxcut_triangle() {
        // Triangle graph: all edges weight 1. MaxCut = 2 (can't cut all 3).
        let edges = vec![(0, 1, 1.0), (1, 2, 1.0), (0, 2, 1.0)];
        let model = encode_maxcut(3, &edges);

        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        let cut = maxcut_value(&edges, &result.best_spins);
        assert!((cut - 2.0).abs() < 1e-10, "MaxCut of triangle should be 2, got {}", cut);
    }

    #[test]
    fn test_maxcut_square() {
        // Square graph (4-cycle): MaxCut = 4 (bipartite)
        let edges = vec![(0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)];
        let model = encode_maxcut(4, &edges);

        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        let cut = maxcut_value(&edges, &result.best_spins);
        assert!((cut - 4.0).abs() < 1e-10, "MaxCut of 4-cycle should be 4, got {}", cut);
    }

    #[test]
    fn test_maxcut_value_function() {
        let edges = vec![(0, 1, 1.0), (1, 2, 2.0)];
        // Spins: [1, -1, 1] => cuts both edges => value = 1 + 2 = 3
        assert!((maxcut_value(&edges, &[1, -1, 1]) - 3.0).abs() < 1e-10);
        // Spins: [1, 1, -1] => cuts only edge (1,2) => value = 2
        assert!((maxcut_value(&edges, &[1, 1, -1]) - 2.0).abs() < 1e-10);
    }

    // --- Number Partitioning tests ---

    #[test]
    fn test_partition_simple() {
        // Numbers: [1, 2, 3] => best partition: {3} vs {1,2}, imbalance = 0
        let numbers = vec![1.0, 2.0, 3.0];
        let model = encode_number_partition(&numbers);

        let config = AnnealingConfig {
            initial_temp: 10.0,
            final_temp: 0.001,
            num_sweeps: 1000,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run_restarts(&model, &config, 5);
        let imbalance = partition_imbalance(&numbers, &result.best_spins);
        assert!(
            imbalance < 1e-10,
            "Should find perfect partition, got imbalance {}",
            imbalance
        );
    }

    #[test]
    fn test_partition_imbalance_function() {
        let numbers = vec![3.0, 1.0, 2.0];
        // Spins: [1, -1, -1] => sum = 3 - 1 - 2 = 0
        assert!((partition_imbalance(&numbers, &[1, -1, -1]) - 0.0).abs() < 1e-10);
        // Spins: [1, 1, -1] => sum = 3 + 1 - 2 = 2
        assert!((partition_imbalance(&numbers, &[1, 1, -1]) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_is_perfect_partition() {
        let numbers = vec![1.0, 2.0, 3.0];
        assert!(is_perfect_partition(&numbers, &[1, 1, -1])); // 1+2 = 3
        assert!(!is_perfect_partition(&numbers, &[1, 1, 1])); // imbalance = 6
    }

    // --- Graph Coloring tests ---

    #[test]
    fn test_graph_coloring_bipartite() {
        // Square (bipartite): should be 2-colorable
        let edges = vec![(0, 1), (1, 2), (2, 3), (3, 0)];
        let model = encode_graph_coloring_2(4, &edges);

        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        assert!(
            is_valid_2_coloring(&edges, &result.best_spins),
            "Square graph should be 2-colorable"
        );
    }

    #[test]
    fn test_graph_coloring_triangle() {
        // Triangle: NOT 2-colorable (chromatic number = 3)
        let edges = vec![(0, 1), (1, 2), (0, 2)];
        let model = encode_graph_coloring_2(3, &edges);

        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&model, &config);
        let violations = coloring_violations(&edges, &result.best_spins);
        // Triangle can't be 2-colored, minimum violations = 1
        assert!(violations >= 1, "Triangle needs at least 1 violation");
    }

    #[test]
    fn test_coloring_violations_function() {
        let edges = vec![(0, 1), (1, 2)];
        // [1, -1, 1]: no violations
        assert_eq!(coloring_violations(&edges, &[1, -1, 1]), 0);
        // [1, 1, 1]: both edges violated
        assert_eq!(coloring_violations(&edges, &[1, 1, 1]), 2);
        // [1, 1, -1]: edge (0,1) violated
        assert_eq!(coloring_violations(&edges, &[1, 1, -1]), 1);
    }

    // --- QUBO with SA ---

    #[test]
    fn test_qubo_via_ising() {
        // QUBO: minimize -x0 - x1 + 2*x0*x1
        // Optimal: x=[1,0] or x=[0,1], E = -1
        let mut qubo = QuboModel::new(2);
        qubo.set(0, 0, -1.0);
        qubo.set(1, 1, -1.0);
        qubo.set(0, 1, 2.0);

        let ising = qubo.to_ising();
        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.001,
            num_sweeps: 500,
            schedule: ScheduleType::Exponential,
            seed: 42,
        };

        let result = SimulatedAnnealing::run(&ising, &config);
        let x = QuboModel::spins_to_binary(&result.best_spins);
        let qubo_e = qubo.energy(&x);
        assert!(
            (qubo_e - (-1.0)).abs() < 1e-10,
            "QUBO optimal should be -1, got {}",
            qubo_e
        );
    }

    // --- SQA vs SA comparison ---

    #[test]
    fn test_sqa_finds_ground_state_small() {
        // Small enough to verify: 4-spin with known ground state
        let mut model = IsingModel::new(4);
        model.set_coupling(0, 1, 1.0);
        model.set_coupling(1, 2, 1.0);
        model.set_coupling(2, 3, 1.0);
        model.set_coupling(3, 0, 1.0);
        // Fully ferromagnetic 4-cycle: ground state energy = -4

        let config = SqaConfig {
            num_trotter_slices: 8,
            gamma_initial: 5.0,
            gamma_final: 0.01,
            temperature: 0.3,
            num_sweeps: 500,
            gamma_schedule: ScheduleType::Linear,
            seed: 42,
        };

        let result = SimulatedQuantumAnnealing::run(&model, &config);
        assert!(
            (result.energy - (-4.0)).abs() < 1e-10,
            "SQA should find ground state -4, got {}",
            result.energy
        );
    }

    // --- Edge cases ---

    #[test]
    fn test_single_spin_system() {
        let model = IsingModel::new(1);
        let spins = vec![1];
        assert!((model.energy(&spins) - 0.0).abs() < 1e-10);
        assert!((model.delta_energy_flip(&spins, 0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_zero_sweeps() {
        let model = IsingModel::random(3, 42);
        let config = AnnealingConfig {
            initial_temp: 5.0,
            final_temp: 0.01,
            num_sweeps: 0,
            schedule: ScheduleType::Linear,
            seed: 42,
        };
        let result = SimulatedAnnealing::run(&model, &config);
        assert_eq!(result.sweeps_performed, 0);
        assert!(result.energy_history.is_empty());
    }

    #[test]
    fn test_make_schedule_factory() {
        let sched = make_schedule(ScheduleType::Linear, 10.0, 1.0);
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);

        let sched = make_schedule(ScheduleType::Exponential, 10.0, 0.1);
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);

        let sched = make_schedule(ScheduleType::Logarithmic, 10.0, 1.0);
        assert!((sched.value_at(0, 100) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_qubo_new_zeros() {
        let qubo = QuboModel::new(4);
        for i in 0..4 {
            for j in 0..4 {
                assert!((qubo.get(i, j) - 0.0).abs() < 1e-10);
            }
        }
        assert!((qubo.energy(&[1, 1, 1, 1]) - 0.0).abs() < 1e-10);
    }
}
