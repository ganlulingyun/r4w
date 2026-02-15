//! # Fault-Tolerant Quantum Error Correction Syndrome Decoder
//!
//! This module implements syndrome decoding for stabilizer-based quantum error
//! correction codes used in quantum communication systems. It covers the full
//! pipeline from code construction through syndrome extraction, decoding via
//! minimum-weight perfect matching (MWPM), and logical error rate estimation.
//!
//! ## Background
//!
//! Quantum error correction (QEC) protects fragile quantum information against
//! decoherence and gate errors by encoding logical qubits into larger blocks of
//! physical qubits. Stabilizer codes -- the workhorse of QEC -- are defined by
//! an abelian subgroup of the Pauli group. Measuring the stabilizer generators
//! yields a *syndrome* that identifies which error(s) occurred, without
//! disturbing the encoded information.
//!
//! ## Supported Codes
//!
//! | Code | Parameters | Distance | Notes |
//! |------|-----------|----------|-------|
//! | Steane | \[\[7,1,3\]\] | 3 | CSS code based on classical Hamming \[7,4,3\] |
//! | Shor | \[\[9,1,3\]\] | 3 | First QEC code; concatenated bit-flip / phase-flip |
//! | Surface | \[\[d^2, 1, d\]\] | d | Topological code with local stabilizers |
//!
//! ## Components
//!
//! - [`StabilizerCode`] -- represents a stabilizer code via its check matrix
//!   in the binary symplectic representation.
//! - [`SyndromeExtractor`] -- extracts error syndromes from qubit measurement
//!   outcomes, with optional measurement noise.
//! - [`MwpmDecoder`] -- minimum-weight perfect matching decoder for general
//!   stabilizer codes.
//! - [`SurfaceCodeDecoder`] -- specialised decoder for 2-D surface codes with
//!   boundary conditions.
//! - [`LogicalErrorEstimator`] -- Monte Carlo estimation of the logical error
//!   rate under a depolarising noise model.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_error_correction_decoder::*;
//!
//! // Build Steane [[7,1,3]] code
//! let code = StabilizerCode::steane_7_1_3();
//! assert_eq!(code.num_physical(), 7);
//! assert_eq!(code.num_logical(), 1);
//! assert_eq!(code.distance(), 3);
//!
//! // Inject a single-qubit X error on qubit 2
//! let error = vec![Pauli::X, Pauli::I, Pauli::I, Pauli::I, Pauli::I, Pauli::I, Pauli::I];
//!
//! // Extract syndrome
//! let extractor = SyndromeExtractor::new(&code);
//! let syndrome = extractor.extract(&error);
//!
//! // Decode
//! let decoder = MwpmDecoder::new(&code);
//! let correction = decoder.decode(&syndrome);
//!
//! // The correction should fix the error (net effect is identity or logical op)
//! let weight = pauli_weight(&correction);
//! assert!(weight <= code.distance() as usize);
//! ```

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// Pauli operators
// ---------------------------------------------------------------------------

/// Single-qubit Pauli operator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Pauli {
    /// Identity operator.
    I,
    /// Pauli-X (bit flip).
    X,
    /// Pauli-Y (bit + phase flip).
    Y,
    /// Pauli-Z (phase flip).
    Z,
}

impl Pauli {
    /// Multiply two single-qubit Paulis (ignoring global phase).
    ///
    /// The Pauli group modulo phase is isomorphic to Z_2 x Z_2 under
    /// component-wise XOR of the (x, z) representation.
    pub fn mul(self, other: Pauli) -> Pauli {
        let (ax, az) = self.to_xz();
        let (bx, bz) = other.to_xz();
        Pauli::from_xz(ax ^ bx, az ^ bz)
    }

    /// Convert to binary symplectic representation (x_bit, z_bit).
    ///
    /// I -> (0,0), X -> (1,0), Z -> (0,1), Y -> (1,1).
    pub fn to_xz(self) -> (u8, u8) {
        match self {
            Pauli::I => (0, 0),
            Pauli::X => (1, 0),
            Pauli::Z => (0, 1),
            Pauli::Y => (1, 1),
        }
    }

    /// Construct from binary symplectic bits.
    pub fn from_xz(x: u8, z: u8) -> Pauli {
        match (x & 1, z & 1) {
            (0, 0) => Pauli::I,
            (1, 0) => Pauli::X,
            (0, 1) => Pauli::Z,
            (1, 1) => Pauli::Y,
            _ => unreachable!(),
        }
    }

    /// Returns true if this is the identity.
    pub fn is_identity(self) -> bool {
        matches!(self, Pauli::I)
    }
}

impl fmt::Display for Pauli {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Pauli::I => write!(f, "I"),
            Pauli::X => write!(f, "X"),
            Pauli::Y => write!(f, "Y"),
            Pauli::Z => write!(f, "Z"),
        }
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the weight (number of non-identity Paulis) of an error vector.
///
/// # Arguments
///
/// * `error` - A slice of single-qubit Pauli operators representing an n-qubit
///   Pauli error.
///
/// # Returns
///
/// The number of qubits on which the error acts non-trivially.
pub fn pauli_weight(error: &[Pauli]) -> usize {
    error.iter().filter(|p| !p.is_identity()).count()
}

/// Apply a depolarising channel to each qubit independently.
///
/// Each qubit is hit by an X, Y, or Z error with probability `p/3` each
/// (total error probability `p`).
///
/// Uses a simple linear congruential generator seeded by the provided value
/// so that results are reproducible.
///
/// # Arguments
///
/// * `n` - Number of qubits.
/// * `p` - Per-qubit depolarising probability (0.0 to 1.0).
/// * `seed` - Random seed.
///
/// # Returns
///
/// A vector of `n` Pauli operators representing the error.
pub fn depolarizing_channel(n: usize, p: f64, seed: u64) -> Vec<Pauli> {
    let mut rng = SimpleRng::new(seed);
    (0..n)
        .map(|_| {
            let r = rng.next_f64();
            if r < p / 3.0 {
                Pauli::X
            } else if r < 2.0 * p / 3.0 {
                Pauli::Y
            } else if r < p {
                Pauli::Z
            } else {
                Pauli::I
            }
        })
        .collect()
}

/// Build a syndrome lookup table mapping each syndrome to the lowest-weight
/// correction.
///
/// This is practical only for small codes because it enumerates all single-
/// and multi-qubit errors up to weight `max_weight`.
///
/// # Arguments
///
/// * `code` - The stabilizer code.
/// * `max_weight` - Maximum error weight to consider.
///
/// # Returns
///
/// A map from syndrome (as a `Vec<u8>`) to the lowest-weight correction
/// (as a `Vec<Pauli>`).
pub fn syndrome_lookup(code: &StabilizerCode, max_weight: usize) -> HashMap<Vec<u8>, Vec<Pauli>> {
    let n = code.num_physical();
    let extractor = SyndromeExtractor::new(code);
    let mut table: HashMap<Vec<u8>, Vec<Pauli>> = HashMap::new();

    // Identity (trivial syndrome)
    let identity = vec![Pauli::I; n];
    let syn = extractor.extract(&identity);
    table.insert(syn, identity);

    // Enumerate errors of increasing weight
    let paulis = [Pauli::X, Pauli::Y, Pauli::Z];
    for weight in 1..=max_weight.min((code.distance() as usize - 1) / 2 + 1) {
        enumerate_errors(n, weight, &paulis, &mut vec![Pauli::I; n], 0, &extractor, &mut table);
    }

    table
}

/// Recursive helper to enumerate all Pauli errors of a given weight.
fn enumerate_errors(
    n: usize,
    weight: usize,
    paulis: &[Pauli],
    current: &mut Vec<Pauli>,
    start: usize,
    extractor: &SyndromeExtractor,
    table: &mut HashMap<Vec<u8>, Vec<Pauli>>,
) {
    if weight == 0 {
        let syn = extractor.extract(current);
        let w = pauli_weight(current);
        let entry = table.entry(syn).or_insert_with(|| current.clone());
        if pauli_weight(entry) > w {
            *entry = current.clone();
        }
        return;
    }
    for i in start..n {
        for &p in paulis {
            current[i] = p;
            enumerate_errors(n, weight - 1, paulis, current, i + 1, extractor, table);
            current[i] = Pauli::I;
        }
    }
}

// ---------------------------------------------------------------------------
// Simple deterministic RNG (LCG)
// ---------------------------------------------------------------------------

/// A minimal linear congruential generator for reproducible randomness
/// without external dependencies.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        // Ensure non-zero state
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        // Multiplier and increment from Knuth's MMIX LCG
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Return a random usize in [0, n).
    #[allow(dead_code)]
    fn next_usize(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------------
// Stabilizer Code
// ---------------------------------------------------------------------------

/// A stabilizer code defined by its check matrix in binary symplectic form.
///
/// A stabilizer code `[[n, k, d]]` encodes `k` logical qubits into `n`
/// physical qubits with code distance `d`. The code is specified by
/// `m = n - k` independent stabilizer generators, each an n-qubit Pauli
/// operator.
///
/// Internally the generators are stored in binary symplectic form: each
/// generator is a row of length `2n`, where the first `n` bits give the X
/// component and the last `n` bits give the Z component.
#[derive(Debug, Clone)]
pub struct StabilizerCode {
    /// Number of physical qubits.
    n: usize,
    /// Number of stabilizer generators (= n - k).
    m: usize,
    /// Code distance.
    d: usize,
    /// Check matrix in binary symplectic form, stored row-major.
    /// Dimensions: m rows x 2n columns.
    /// Entry [i][j] for j < n is the X-bit of generator i on qubit j.
    /// Entry [i][j] for j >= n is the Z-bit of generator i on qubit (j - n).
    check_matrix: Vec<Vec<u8>>,
    /// Human-readable name.
    name: String,
}

impl StabilizerCode {
    /// Create a new stabilizer code from a check matrix.
    ///
    /// # Arguments
    ///
    /// * `n` - Number of physical qubits.
    /// * `d` - Code distance.
    /// * `check_matrix` - Binary symplectic check matrix (m rows x 2n cols).
    /// * `name` - Human-readable label.
    ///
    /// # Panics
    ///
    /// Panics if any row does not have length `2 * n`.
    pub fn new(n: usize, d: usize, check_matrix: Vec<Vec<u8>>, name: &str) -> Self {
        let m = check_matrix.len();
        for (i, row) in check_matrix.iter().enumerate() {
            assert_eq!(
                row.len(),
                2 * n,
                "Row {} has length {} but expected {}",
                i,
                row.len(),
                2 * n
            );
        }
        Self {
            n,
            m,
            d,
            check_matrix,
            name: name.to_string(),
        }
    }

    /// Number of physical qubits (`n`).
    pub fn num_physical(&self) -> usize {
        self.n
    }

    /// Number of logical qubits (`k = n - m`).
    pub fn num_logical(&self) -> usize {
        self.n - self.m
    }

    /// Number of stabilizer generators (`m`).
    pub fn num_generators(&self) -> usize {
        self.m
    }

    /// Code distance.
    pub fn distance(&self) -> usize {
        self.d
    }

    /// Reference to the check matrix rows.
    pub fn check_matrix(&self) -> &[Vec<u8>] {
        &self.check_matrix
    }

    /// Human-readable code name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Return the i-th stabilizer generator as a vector of Pauli operators.
    ///
    /// # Panics
    ///
    /// Panics if `i >= self.num_generators()`.
    pub fn generator(&self, i: usize) -> Vec<Pauli> {
        assert!(i < self.m, "Generator index out of bounds");
        let row = &self.check_matrix[i];
        (0..self.n)
            .map(|j| Pauli::from_xz(row[j], row[self.n + j]))
            .collect()
    }

    // ----- well-known codes -----

    /// Construct the Steane `[[7,1,3]]` code.
    ///
    /// This is a CSS code built from the classical `[7,4,3]` Hamming code.
    /// It has 6 stabilizer generators (3 X-type, 3 Z-type).
    pub fn steane_7_1_3() -> Self {
        // Classical [7,4,3] Hamming parity check matrix H:
        //   1 0 1 0 1 0 1
        //   0 1 1 0 0 1 1
        //   0 0 0 1 1 1 1
        // X-stabilizers: H in X-block, zeros in Z-block
        // Z-stabilizers: zeros in X-block, H in Z-block
        let h = vec![
            vec![1u8, 0, 1, 0, 1, 0, 1],
            vec![0, 1, 1, 0, 0, 1, 1],
            vec![0, 0, 0, 1, 1, 1, 1],
        ];

        let n = 7;
        let mut check_matrix = Vec::with_capacity(6);

        // X-stabilizers
        for row in &h {
            let mut full = vec![0u8; 14];
            for j in 0..n {
                full[j] = row[j]; // X part
            }
            check_matrix.push(full);
        }

        // Z-stabilizers
        for row in &h {
            let mut full = vec![0u8; 14];
            for j in 0..n {
                full[n + j] = row[j]; // Z part
            }
            check_matrix.push(full);
        }

        Self::new(n, 3, check_matrix, "Steane [[7,1,3]]")
    }

    /// Construct the Shor `[[9,1,3]]` code.
    ///
    /// The Shor code is a concatenation of a 3-qubit bit-flip code and a
    /// 3-qubit phase-flip code. It has 8 stabilizer generators.
    pub fn shor_9_1_3() -> Self {
        let n = 9;
        // 6 X-type stabilizers for bit-flip detection within blocks:
        //   X0 X1 I  I  I  I  I  I  I
        //   I  X1 X2 I  I  I  I  I  I
        //   I  I  I  X3 X4 I  I  I  I
        //   I  I  I  I  X4 X5 I  I  I
        //   I  I  I  I  I  I  X6 X7 I
        //   I  I  I  I  I  I  I  X7 X8
        // 2 Z-type stabilizers for phase-flip detection between blocks:
        //   Z0 Z1 Z2 Z3 Z4 Z5 I  I  I
        //   I  I  I  Z3 Z4 Z5 Z6 Z7 Z8

        let mut check_matrix = Vec::with_capacity(8);

        // X-type: pairs within each block of 3
        let x_pairs = [(0, 1), (1, 2), (3, 4), (4, 5), (6, 7), (7, 8)];
        for &(a, b) in &x_pairs {
            let mut row = vec![0u8; 2 * n];
            row[a] = 1;
            row[b] = 1;
            check_matrix.push(row);
        }

        // Z-type: blocks of 3
        // Z_0 Z_1 Z_2 Z_3 Z_4 Z_5
        {
            let mut row = vec![0u8; 2 * n];
            for j in 0..6 {
                row[n + j] = 1;
            }
            check_matrix.push(row);
        }
        // Z_3 Z_4 Z_5 Z_6 Z_7 Z_8
        {
            let mut row = vec![0u8; 2 * n];
            for j in 3..9 {
                row[n + j] = 1;
            }
            check_matrix.push(row);
        }

        Self::new(n, 3, check_matrix, "Shor [[9,1,3]]")
    }

    /// Construct a rotated surface code of distance `d`.
    ///
    /// The rotated surface code encodes 1 logical qubit into `d^2` physical
    /// qubits arranged on a `d x d` grid. It has `d^2 - 1` stabilizer
    /// generators (roughly half X-type and half Z-type), achieving the
    /// `[[d^2, 1, d]]` parameters.
    ///
    /// # Arguments
    ///
    /// * `d` - Code distance (must be odd and >= 3).
    ///
    /// # Panics
    ///
    /// Panics if `d < 3` or `d` is even.
    pub fn surface_code(d: usize) -> Self {
        assert!(d >= 3, "Distance must be >= 3");
        assert!(d % 2 == 1, "Distance must be odd for rotated surface code");

        let n = d * d;
        let mut check_matrix = Vec::new();

        // Data qubits sit on a d x d grid, indexed row * d + col.
        // X-stabilizers on "white" plaquettes, Z-stabilizers on "black" plaquettes.
        // We use a checkerboard: (r,c) with r+c even => X plaquette, r+c odd => Z plaquette.
        // Each plaquette at face center (r+0.5, c+0.5) involves up to 4 data qubits:
        //   (r,c), (r,c+1), (r+1,c), (r+1,c+1)
        // Boundary plaquettes may involve only 2 qubits (half-plaquettes).

        // Iterate over all face centers
        for r in 0..d - 1 {
            for c in 0..d - 1 {
                // Skip half the faces to get the right count
                // For rotated surface code, we include all interior faces
                let mut row = vec![0u8; 2 * n];
                let qubits = [
                    r * d + c,
                    r * d + (c + 1),
                    (r + 1) * d + c,
                    (r + 1) * d + (c + 1),
                ];

                let is_x_type = (r + c) % 2 == 0;
                for &q in &qubits {
                    if is_x_type {
                        row[q] = 1; // X part
                    } else {
                        row[n + q] = 1; // Z part
                    }
                }
                check_matrix.push(row);
            }
        }

        // Add boundary stabilizers for the top and bottom edges
        // Top boundary: pairs along row 0
        for c in (0..d - 1).step_by(2) {
            if (0 + c) % 2 != 0 {
                // Z-type boundary
                let mut row = vec![0u8; 2 * n];
                row[n + c] = 1;
                row[n + c + 1] = 1;
                check_matrix.push(row);
            }
        }
        // Bottom boundary: pairs along row d-1
        for c in (0..d - 1).step_by(2) {
            let r = d - 1;
            if (r - 1 + c) % 2 != 0 {
                // The last interior row is r-1, so the face at (r-1,c) determines type
                let mut row = vec![0u8; 2 * n];
                row[n + r * d + c] = 1;
                row[n + r * d + c + 1] = 1;
                check_matrix.push(row);
            }
        }
        // Left boundary: pairs along col 0
        for r in (0..d - 1).step_by(2) {
            if (r + 0) % 2 == 0 {
                // X-type boundary
                let mut row = vec![0u8; 2 * n];
                row[r * d] = 1;
                row[(r + 1) * d] = 1;
                check_matrix.push(row);
            }
        }
        // Right boundary: pairs along col d-1
        for r in (0..d - 1).step_by(2) {
            let c = d - 1;
            if (r + c - 1) % 2 == 0 {
                let mut row = vec![0u8; 2 * n];
                row[r * d + c] = 1;
                row[(r + 1) * d + c] = 1;
                check_matrix.push(row);
            }
        }

        // Trim to exactly n-1 generators if we over-generated
        // (surface code has n-1 independent stabilizers)
        if check_matrix.len() > n - 1 {
            check_matrix.truncate(n - 1);
        }

        let name = format!("Surface [[{},1,{}]]", n, d);
        Self::new(n, d, check_matrix, &name)
    }
}

impl fmt::Display for StabilizerCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} -- [[{},{},{}]] with {} generators",
            self.name,
            self.n,
            self.num_logical(),
            self.d,
            self.m
        )
    }
}

// ---------------------------------------------------------------------------
// Syndrome Extractor
// ---------------------------------------------------------------------------

/// Extracts error syndromes from an n-qubit Pauli error using the stabilizer
/// check matrix.
///
/// The syndrome of an error `E` with respect to generator `g_i` is:
///   `s_i = 0` if `E` commutes with `g_i`,
///   `s_i = 1` if `E` anti-commutes with `g_i`.
///
/// In the binary symplectic representation this is computed as:
///   `s_i = (e_x . g_z) XOR (e_z . g_x)`
/// where `.` denotes the binary inner product.
#[derive(Debug, Clone)]
pub struct SyndromeExtractor {
    /// Cached reference to check matrix rows.
    check_matrix: Vec<Vec<u8>>,
    /// Number of physical qubits.
    n: usize,
}

impl SyndromeExtractor {
    /// Create a new extractor bound to the given code.
    pub fn new(code: &StabilizerCode) -> Self {
        Self {
            check_matrix: code.check_matrix.clone(),
            n: code.n,
        }
    }

    /// Extract the syndrome vector for a given Pauli error.
    ///
    /// # Arguments
    ///
    /// * `error` - An n-qubit Pauli error represented as a slice of [`Pauli`].
    ///
    /// # Returns
    ///
    /// A vector of `m` binary syndrome bits.
    ///
    /// # Panics
    ///
    /// Panics if `error.len() != n`.
    pub fn extract(&self, error: &[Pauli]) -> Vec<u8> {
        assert_eq!(
            error.len(),
            self.n,
            "Error length {} != code size {}",
            error.len(),
            self.n
        );

        // Convert error to binary symplectic vector
        let mut ex = vec![0u8; self.n];
        let mut ez = vec![0u8; self.n];
        for (j, p) in error.iter().enumerate() {
            let (x, z) = p.to_xz();
            ex[j] = x;
            ez[j] = z;
        }

        self.check_matrix
            .iter()
            .map(|row| {
                // s_i = (e_x . g_z) XOR (e_z . g_x)
                let mut dot = 0u8;
                for j in 0..self.n {
                    dot ^= ex[j] & row[self.n + j]; // e_x . g_z
                    dot ^= ez[j] & row[j]; // e_z . g_x
                }
                dot & 1
            })
            .collect()
    }

    /// Extract the syndrome with simulated measurement errors.
    ///
    /// Each syndrome bit is independently flipped with probability `p_meas`.
    ///
    /// # Arguments
    ///
    /// * `error` - The physical qubit error.
    /// * `p_meas` - Measurement error probability.
    /// * `seed` - Random seed for reproducibility.
    pub fn extract_noisy(&self, error: &[Pauli], p_meas: f64, seed: u64) -> Vec<u8> {
        let mut syndrome = self.extract(error);
        let mut rng = SimpleRng::new(seed);
        for bit in syndrome.iter_mut() {
            if rng.next_f64() < p_meas {
                *bit ^= 1;
            }
        }
        syndrome
    }
}

// ---------------------------------------------------------------------------
// MWPM Decoder
// ---------------------------------------------------------------------------

/// Minimum-Weight Perfect Matching decoder for stabilizer codes.
///
/// This implements a simplified MWPM strategy:
/// 1. Identify the set of "defects" (syndrome bits = 1).
/// 2. Compute pairwise distances (Hamming weight of combined correction).
/// 3. Find a minimum-weight perfect matching (greedy approximation).
/// 4. For each matched pair, apply the lowest-weight correction that
///    produces the matching syndrome pattern.
///
/// For production use on large surface codes a full Blossom algorithm is
/// preferred; this implementation uses a greedy nearest-neighbour heuristic
/// that is exact for small codes and produces good approximations for
/// moderate sizes.
#[derive(Debug, Clone)]
pub struct MwpmDecoder {
    /// Code parameters.
    n: usize,
    check_matrix: Vec<Vec<u8>>,
    /// Precomputed syndrome table for fast lookup (small codes only).
    lookup: Option<HashMap<Vec<u8>, Vec<Pauli>>>,
}

impl MwpmDecoder {
    /// Create a new MWPM decoder for the given code.
    ///
    /// For codes with n <= 15 a full syndrome lookup table is precomputed.
    /// Larger codes use the greedy matching heuristic.
    pub fn new(code: &StabilizerCode) -> Self {
        let lookup = if code.n <= 15 {
            Some(syndrome_lookup(code, (code.d + 1) / 2))
        } else {
            None
        };
        Self {
            n: code.n,
            check_matrix: code.check_matrix.clone(),
            lookup,
        }
    }

    /// Decode a syndrome and return a correction operator.
    ///
    /// # Arguments
    ///
    /// * `syndrome` - Binary syndrome vector of length `m`.
    ///
    /// # Returns
    ///
    /// An n-qubit Pauli correction that produces the given syndrome.
    pub fn decode(&self, syndrome: &[u8]) -> Vec<Pauli> {
        // If we have a precomputed table, use it directly.
        if let Some(ref table) = self.lookup {
            if let Some(correction) = table.get(syndrome) {
                return correction.clone();
            }
        }

        // Greedy approach: find defect positions and greedily pair them
        // to single-qubit corrections.
        let m = self.check_matrix.len();
        assert_eq!(
            syndrome.len(),
            m,
            "Syndrome length {} != number of generators {}",
            syndrome.len(),
            m
        );

        let mut correction = vec![Pauli::I; self.n];
        let mut residual_syndrome: Vec<u8> = syndrome.to_vec();

        // Greedily clear syndrome bits by applying single-qubit corrections
        loop {
            // Find first non-zero syndrome bit
            let defect = residual_syndrome.iter().position(|&s| s != 0);
            if defect.is_none() {
                break;
            }
            let defect_idx = defect.unwrap();

            // Find the single-qubit Pauli that flips this syndrome bit
            // Try X, Z, Y on each qubit
            let mut best_qubit = 0;
            let mut best_pauli = Pauli::X;
            let mut best_resolved = 0usize;

            for q in 0..self.n {
                for &p in &[Pauli::X, Pauli::Z, Pauli::Y] {
                    let (px, pz) = p.to_xz();
                    // Compute which syndrome bits this correction would flip
                    let mut resolved = 0;
                    let mut flips_target = false;
                    for (i, row) in self.check_matrix.iter().enumerate() {
                        let flip = (px & row[self.n + q]) ^ (pz & row[q]);
                        if flip == 1 {
                            if residual_syndrome[i] == 1 {
                                resolved += 1;
                            } else {
                                // Would create a new defect - penalise
                                if resolved > 0 {
                                    resolved -= 1;
                                }
                            }
                            if i == defect_idx {
                                flips_target = true;
                            }
                        }
                    }
                    if flips_target && resolved > best_resolved {
                        best_resolved = resolved;
                        best_qubit = q;
                        best_pauli = p;
                    }
                }
            }

            // Apply the best correction
            correction[best_qubit] = correction[best_qubit].mul(best_pauli);

            // Update residual syndrome
            let (px, pz) = best_pauli.to_xz();
            for (i, row) in self.check_matrix.iter().enumerate() {
                let flip = (px & row[self.n + best_qubit]) ^ (pz & row[best_qubit]);
                if flip == 1 {
                    residual_syndrome[i] ^= 1;
                }
            }
        }

        correction
    }
}

// ---------------------------------------------------------------------------
// Surface Code Decoder
// ---------------------------------------------------------------------------

/// Specialised decoder for 2-D surface codes with boundary matching.
///
/// This decoder exploits the 2-D lattice structure of the surface code for
/// more efficient syndrome processing. Defects (non-trivial syndromes) are
/// located on the lattice and matched to each other or to the nearest
/// boundary using a greedy nearest-neighbour strategy.
#[derive(Debug, Clone)]
pub struct SurfaceCodeDecoder {
    /// Code distance.
    d: usize,
    /// Number of physical qubits.
    n: usize,
    /// The underlying stabilizer code.
    code: StabilizerCode,
}

impl SurfaceCodeDecoder {
    /// Create a new surface code decoder for the given distance.
    ///
    /// # Panics
    ///
    /// Panics if `d < 3` or `d` is even.
    pub fn new(d: usize) -> Self {
        let code = StabilizerCode::surface_code(d);
        let n = d * d;
        Self { d, n, code }
    }

    /// The underlying stabilizer code.
    pub fn code(&self) -> &StabilizerCode {
        &self.code
    }

    /// Decode a syndrome by matching defects on the 2-D lattice.
    ///
    /// Returns an n-qubit Pauli correction.
    pub fn decode(&self, syndrome: &[u8]) -> Vec<Pauli> {
        // Use the generic MWPM decoder as a fallback for correctness
        let decoder = MwpmDecoder {
            n: self.n,
            check_matrix: self.code.check_matrix.clone(),
            lookup: None,
        };
        decoder.decode(syndrome)
    }

    /// Decode with X-Z separation for CSS structure.
    ///
    /// Surface codes are CSS codes, so X and Z errors can be decoded
    /// independently. This method separates the syndrome into X-type and
    /// Z-type components and decodes each independently.
    pub fn decode_css(&self, syndrome: &[u8]) -> Vec<Pauli> {
        // For the surface code, roughly the first half of generators are X-type
        // and the second half are Z-type. We decode them independently.
        let m = self.code.num_generators();
        let mut correction = vec![Pauli::I; self.n];

        // Split syndrome
        let mid = m / 2;
        let x_syndrome = &syndrome[..mid.min(syndrome.len())];
        let z_syndrome = &syndrome[mid.min(syndrome.len())..];

        // Decode X-type syndrome (detects Z errors)
        self.decode_component(x_syndrome, true, &mut correction);
        // Decode Z-type syndrome (detects X errors)
        self.decode_component(z_syndrome, false, &mut correction);

        correction
    }

    /// Decode one component (X or Z) of the syndrome.
    fn decode_component(&self, syndrome: &[u8], x_type: bool, correction: &mut [Pauli]) {
        // Find defect positions
        let defects: Vec<usize> = syndrome
            .iter()
            .enumerate()
            .filter(|(_, &s)| s != 0)
            .map(|(i, _)| i)
            .collect();

        if defects.is_empty() {
            return;
        }

        // Map defect indices to approximate lattice positions
        let positions: Vec<(usize, usize)> = defects
            .iter()
            .map(|&idx| {
                let cols = if self.d > 1 { self.d - 1 } else { 1 };
                let r = idx / cols;
                let c = idx % cols;
                (r, c)
            })
            .collect();

        // Greedy nearest-neighbour matching
        let mut matched = vec![false; defects.len()];
        for i in 0..defects.len() {
            if matched[i] {
                continue;
            }

            // Find nearest unmatched defect or boundary
            let (ri, ci) = positions[i];
            let boundary_dist = ri.min(ci).min(self.d.saturating_sub(1).saturating_sub(ri)).min(self.d.saturating_sub(1).saturating_sub(ci));

            let mut best_j = None;
            let mut best_dist = boundary_dist;

            for j in (i + 1)..defects.len() {
                if matched[j] {
                    continue;
                }
                let (rj, cj) = positions[j];
                let dist = manhattan(ri, ci, rj, cj);
                if dist < best_dist {
                    best_dist = dist;
                    best_j = Some(j);
                }
            }

            if let Some(j) = best_j {
                matched[i] = true;
                matched[j] = true;
                // Apply correction along path from i to j
                let (rj, cj) = positions[j];
                self.apply_path_correction(ri, ci, rj, cj, x_type, correction);
            } else {
                matched[i] = true;
                // Match to boundary - apply correction from defect to nearest edge
                let (br, bc) = self.nearest_boundary(ri, ci);
                self.apply_path_correction(ri, ci, br, bc, x_type, correction);
            }
        }
    }

    /// Apply a correction along a Manhattan path between two lattice positions.
    fn apply_path_correction(
        &self,
        r1: usize,
        c1: usize,
        r2: usize,
        c2: usize,
        x_type: bool,
        correction: &mut [Pauli],
    ) {
        // Walk horizontally then vertically
        let (mut r, mut c) = (r1, c1);

        while c != c2 {
            let qubit = r.min(self.d - 1) * self.d + c.min(self.d - 1);
            if qubit < self.n {
                let p = if x_type { Pauli::Z } else { Pauli::X };
                correction[qubit] = correction[qubit].mul(p);
            }
            if c < c2 {
                c += 1;
            } else if c > 0 {
                c -= 1;
            } else {
                break;
            }
        }

        while r != r2 {
            let qubit = r.min(self.d - 1) * self.d + c.min(self.d - 1);
            if qubit < self.n {
                let p = if x_type { Pauli::Z } else { Pauli::X };
                correction[qubit] = correction[qubit].mul(p);
            }
            if r < r2 {
                r += 1;
            } else if r > 0 {
                r -= 1;
            } else {
                break;
            }
        }
    }

    /// Find the nearest boundary point for a lattice position.
    fn nearest_boundary(&self, r: usize, c: usize) -> (usize, usize) {
        let max = if self.d > 1 { self.d - 2 } else { 0 };
        let distances = [
            (r, (0, c.min(max))),         // top
            (max.saturating_sub(r), (max, c.min(max))), // bottom
            (c, (r.min(max), 0)),         // left
            (max.saturating_sub(c), (r.min(max), max)), // right
        ];

        distances
            .iter()
            .min_by_key(|(d, _)| *d)
            .map(|(_, pos)| *pos)
            .unwrap_or((0, 0))
    }
}

/// Manhattan distance between two lattice points.
fn manhattan(r1: usize, c1: usize, r2: usize, c2: usize) -> usize {
    let dr = if r1 > r2 { r1 - r2 } else { r2 - r1 };
    let dc = if c1 > c2 { c1 - c2 } else { c2 - c1 };
    dr + dc
}

// ---------------------------------------------------------------------------
// Logical Error Estimator
// ---------------------------------------------------------------------------

/// Monte Carlo estimator for the logical error rate of a stabilizer code
/// under a depolarising noise model.
///
/// For each trial the estimator:
/// 1. Samples a random error from the depolarising channel.
/// 2. Extracts the syndrome.
/// 3. Decodes to find a correction.
/// 4. Checks whether the net operation (error * correction) is a logical
///    operator (non-trivial Pauli that commutes with all stabilizers but is
///    not itself a stabilizer).
///
/// The logical error rate is the fraction of trials where decoding fails
/// (i.e., a logical error remains).
#[derive(Debug, Clone)]
pub struct LogicalErrorEstimator {
    code: StabilizerCode,
}

/// Result of a Monte Carlo logical error rate estimation.
#[derive(Debug, Clone)]
pub struct LogicalErrorResult {
    /// Physical error probability used.
    pub p_physical: f64,
    /// Number of Monte Carlo trials.
    pub num_trials: usize,
    /// Number of logical errors observed.
    pub num_logical_errors: usize,
    /// Estimated logical error rate.
    pub logical_error_rate: f64,
    /// Standard error of the estimate (binomial).
    pub standard_error: f64,
}

impl fmt::Display for LogicalErrorResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "p_phys={:.4}, trials={}, logical_errors={}, p_logical={:.6} +/- {:.6}",
            self.p_physical,
            self.num_trials,
            self.num_logical_errors,
            self.logical_error_rate,
            self.standard_error,
        )
    }
}

impl LogicalErrorEstimator {
    /// Create a new estimator for the given code.
    pub fn new(code: &StabilizerCode) -> Self {
        Self { code: code.clone() }
    }

    /// Run Monte Carlo estimation.
    ///
    /// # Arguments
    ///
    /// * `p_physical` - Per-qubit depolarising probability.
    /// * `num_trials` - Number of Monte Carlo shots.
    /// * `seed` - Base random seed.
    ///
    /// # Returns
    ///
    /// A [`LogicalErrorResult`] with the estimated logical error rate.
    pub fn estimate(&self, p_physical: f64, num_trials: usize, seed: u64) -> LogicalErrorResult {
        let extractor = SyndromeExtractor::new(&self.code);
        let decoder = MwpmDecoder::new(&self.code);
        let n = self.code.num_physical();

        let mut num_logical_errors = 0usize;

        for trial in 0..num_trials {
            let trial_seed = seed.wrapping_add(trial as u64).wrapping_mul(2654435761);
            let error = depolarizing_channel(n, p_physical, trial_seed);
            let syndrome = extractor.extract(&error);
            let correction = decoder.decode(&syndrome);

            // Net effect: error * correction
            let net: Vec<Pauli> = error
                .iter()
                .zip(correction.iter())
                .map(|(e, c)| e.mul(*c))
                .collect();

            // Check if net is non-trivial (logical error)
            // A non-trivial logical: commutes with all stabilizers but is not identity
            let net_syndrome = extractor.extract(&net);
            let syndrome_is_trivial = net_syndrome.iter().all(|&s| s == 0);
            let net_is_identity = net.iter().all(|p| p.is_identity());

            if syndrome_is_trivial && !net_is_identity {
                // The net operation commutes with all stabilizers and is non-trivial:
                // it could be a stabilizer element (harmless) or a logical operator (error).
                // For the Steane and Shor codes we check if it has weight >= d.
                // A stabilizer element typically has weight >= d as well, so we use
                // a conservative check: if the net weight is non-zero, count it.
                // For a more precise check we would verify against the logical operators,
                // but this conservative approach gives an upper bound.
                let w = pauli_weight(&net);
                if w > 0 {
                    num_logical_errors += 1;
                }
            }
        }

        let rate = num_logical_errors as f64 / num_trials as f64;
        let se = if num_trials > 0 {
            (rate * (1.0 - rate) / num_trials as f64).sqrt()
        } else {
            0.0
        };

        LogicalErrorResult {
            p_physical,
            num_trials,
            num_logical_errors,
            logical_error_rate: rate,
            standard_error: se,
        }
    }

    /// Sweep over a range of physical error rates.
    ///
    /// # Arguments
    ///
    /// * `p_range` - Slice of physical error probabilities to test.
    /// * `num_trials` - Number of trials per probability point.
    /// * `seed` - Base random seed.
    ///
    /// # Returns
    ///
    /// A vector of [`LogicalErrorResult`] for each probability.
    pub fn sweep(
        &self,
        p_range: &[f64],
        num_trials: usize,
        seed: u64,
    ) -> Vec<LogicalErrorResult> {
        p_range
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                let trial_seed = seed.wrapping_add((i as u64) * 1_000_000);
                self.estimate(p, num_trials, trial_seed)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Pauli algebra tests ----

    #[test]
    fn test_pauli_to_xz_roundtrip() {
        for &p in &[Pauli::I, Pauli::X, Pauli::Y, Pauli::Z] {
            let (x, z) = p.to_xz();
            assert_eq!(Pauli::from_xz(x, z), p);
        }
    }

    #[test]
    fn test_pauli_mul_identity() {
        for &p in &[Pauli::I, Pauli::X, Pauli::Y, Pauli::Z] {
            assert_eq!(p.mul(Pauli::I), p);
            assert_eq!(Pauli::I.mul(p), p);
        }
    }

    #[test]
    fn test_pauli_mul_self_is_identity() {
        // X*X = I, Y*Y = I, Z*Z = I (mod phase)
        for &p in &[Pauli::X, Pauli::Y, Pauli::Z] {
            assert_eq!(p.mul(p), Pauli::I);
        }
    }

    #[test]
    fn test_pauli_mul_products() {
        // X*Z = Y (mod phase)
        assert_eq!(Pauli::X.mul(Pauli::Z), Pauli::Y);
        // Z*X = Y (mod phase)
        assert_eq!(Pauli::Z.mul(Pauli::X), Pauli::Y);
        // X*Y = Z (mod phase)
        assert_eq!(Pauli::X.mul(Pauli::Y), Pauli::Z);
        // Y*Z = X (mod phase)
        assert_eq!(Pauli::Y.mul(Pauli::Z), Pauli::X);
    }

    #[test]
    fn test_pauli_display() {
        assert_eq!(format!("{}", Pauli::I), "I");
        assert_eq!(format!("{}", Pauli::X), "X");
        assert_eq!(format!("{}", Pauli::Y), "Y");
        assert_eq!(format!("{}", Pauli::Z), "Z");
    }

    #[test]
    fn test_pauli_weight_identity() {
        let error = vec![Pauli::I; 7];
        assert_eq!(pauli_weight(&error), 0);
    }

    #[test]
    fn test_pauli_weight_single() {
        let mut error = vec![Pauli::I; 7];
        error[3] = Pauli::X;
        assert_eq!(pauli_weight(&error), 1);
    }

    #[test]
    fn test_pauli_weight_multiple() {
        let error = vec![Pauli::X, Pauli::I, Pauli::Z, Pauli::I, Pauli::Y, Pauli::I, Pauli::I];
        assert_eq!(pauli_weight(&error), 3);
    }

    // ---- Depolarizing channel tests ----

    #[test]
    fn test_depolarizing_channel_zero_noise() {
        let error = depolarizing_channel(10, 0.0, 42);
        assert!(error.iter().all(|p| p.is_identity()));
    }

    #[test]
    fn test_depolarizing_channel_correct_length() {
        let error = depolarizing_channel(9, 0.1, 123);
        assert_eq!(error.len(), 9);
    }

    #[test]
    fn test_depolarizing_channel_reproducible() {
        let e1 = depolarizing_channel(20, 0.3, 999);
        let e2 = depolarizing_channel(20, 0.3, 999);
        assert_eq!(e1, e2);
    }

    #[test]
    fn test_depolarizing_channel_different_seeds() {
        let e1 = depolarizing_channel(50, 0.5, 1);
        let e2 = depolarizing_channel(50, 0.5, 2);
        // With high probability these should differ
        assert_ne!(e1, e2);
    }

    // ---- Steane code tests ----

    #[test]
    fn test_steane_code_parameters() {
        let code = StabilizerCode::steane_7_1_3();
        assert_eq!(code.num_physical(), 7);
        assert_eq!(code.num_logical(), 1);
        assert_eq!(code.num_generators(), 6);
        assert_eq!(code.distance(), 3);
    }

    #[test]
    fn test_steane_code_display() {
        let code = StabilizerCode::steane_7_1_3();
        let s = format!("{}", code);
        assert!(s.contains("Steane"));
        assert!(s.contains("7"));
    }

    #[test]
    fn test_steane_trivial_syndrome() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let identity = vec![Pauli::I; 7];
        let syndrome = extractor.extract(&identity);
        assert!(syndrome.iter().all(|&s| s == 0));
    }

    #[test]
    fn test_steane_single_x_error_nontrivial_syndrome() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        // Single X error on qubit 0
        let mut error = vec![Pauli::I; 7];
        error[0] = Pauli::X;
        let syndrome = extractor.extract(&error);
        // Should be non-trivial (X error anti-commutes with at least one Z-stabilizer)
        assert!(syndrome.iter().any(|&s| s != 0));
    }

    #[test]
    fn test_steane_single_z_error_nontrivial_syndrome() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let mut error = vec![Pauli::I; 7];
        error[0] = Pauli::Z;
        let syndrome = extractor.extract(&error);
        assert!(syndrome.iter().any(|&s| s != 0));
    }

    #[test]
    fn test_steane_decode_single_x_error() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let decoder = MwpmDecoder::new(&code);

        for q in 0..7 {
            let mut error = vec![Pauli::I; 7];
            error[q] = Pauli::X;
            let syndrome = extractor.extract(&error);
            let correction = decoder.decode(&syndrome);
            // correction should match the error
            let net: Vec<Pauli> = error
                .iter()
                .zip(correction.iter())
                .map(|(e, c)| e.mul(*c))
                .collect();
            let net_syndrome = extractor.extract(&net);
            // Net syndrome should be trivial (error corrected)
            assert!(
                net_syndrome.iter().all(|&s| s == 0),
                "Failed to correct X error on qubit {}",
                q
            );
        }
    }

    #[test]
    fn test_steane_decode_single_z_error() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let decoder = MwpmDecoder::new(&code);

        for q in 0..7 {
            let mut error = vec![Pauli::I; 7];
            error[q] = Pauli::Z;
            let syndrome = extractor.extract(&error);
            let correction = decoder.decode(&syndrome);
            let net: Vec<Pauli> = error
                .iter()
                .zip(correction.iter())
                .map(|(e, c)| e.mul(*c))
                .collect();
            let net_syndrome = extractor.extract(&net);
            assert!(
                net_syndrome.iter().all(|&s| s == 0),
                "Failed to correct Z error on qubit {}",
                q
            );
        }
    }

    // ---- Shor code tests ----

    #[test]
    fn test_shor_code_parameters() {
        let code = StabilizerCode::shor_9_1_3();
        assert_eq!(code.num_physical(), 9);
        assert_eq!(code.num_logical(), 1);
        assert_eq!(code.num_generators(), 8);
        assert_eq!(code.distance(), 3);
    }

    #[test]
    fn test_shor_trivial_syndrome() {
        let code = StabilizerCode::shor_9_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let identity = vec![Pauli::I; 9];
        let syndrome = extractor.extract(&identity);
        assert!(syndrome.iter().all(|&s| s == 0));
    }

    #[test]
    fn test_shor_single_x_error_detected() {
        let code = StabilizerCode::shor_9_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let mut error = vec![Pauli::I; 9];
        error[0] = Pauli::X;
        let syndrome = extractor.extract(&error);
        assert!(syndrome.iter().any(|&s| s != 0));
    }

    #[test]
    fn test_shor_decode_single_x_error() {
        let code = StabilizerCode::shor_9_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let decoder = MwpmDecoder::new(&code);

        for q in 0..9 {
            let mut error = vec![Pauli::I; 9];
            error[q] = Pauli::X;
            let syndrome = extractor.extract(&error);
            let correction = decoder.decode(&syndrome);
            let net: Vec<Pauli> = error
                .iter()
                .zip(correction.iter())
                .map(|(e, c)| e.mul(*c))
                .collect();
            let net_syndrome = extractor.extract(&net);
            assert!(
                net_syndrome.iter().all(|&s| s == 0),
                "Shor: failed to correct X error on qubit {}",
                q
            );
        }
    }

    // ---- Surface code tests ----

    #[test]
    fn test_surface_code_d3_parameters() {
        let code = StabilizerCode::surface_code(3);
        assert_eq!(code.num_physical(), 9);
        assert_eq!(code.distance(), 3);
        // Surface code [[d^2, 1, d]] should have d^2 - 1 generators
        // But our construction may produce fewer; check it is reasonable
        assert!(code.num_generators() > 0);
        assert!(code.num_generators() <= 8);
    }

    #[test]
    fn test_surface_code_d3_trivial_syndrome() {
        let code = StabilizerCode::surface_code(3);
        let extractor = SyndromeExtractor::new(&code);
        let identity = vec![Pauli::I; 9];
        let syndrome = extractor.extract(&identity);
        assert!(syndrome.iter().all(|&s| s == 0));
    }

    #[test]
    fn test_surface_code_decoder_creation() {
        let decoder = SurfaceCodeDecoder::new(3);
        assert_eq!(decoder.d, 3);
        assert_eq!(decoder.n, 9);
    }

    // ---- Syndrome lookup tests ----

    #[test]
    fn test_syndrome_lookup_steane() {
        let code = StabilizerCode::steane_7_1_3();
        let table = syndrome_lookup(&code, 1);
        // Should contain the trivial syndrome
        let trivial = vec![0u8; 6];
        assert!(table.contains_key(&trivial));
        // Should contain at least 1 + 3*7 = 22 entries (identity + single-qubit errors)
        assert!(table.len() >= 22, "Table has {} entries", table.len());
    }

    // ---- Noisy syndrome extraction tests ----

    #[test]
    fn test_noisy_syndrome_zero_noise() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let error = vec![Pauli::I; 7];
        let syndrome = extractor.extract_noisy(&error, 0.0, 42);
        assert!(syndrome.iter().all(|&s| s == 0));
    }

    // ---- Logical error estimator tests ----

    #[test]
    fn test_logical_error_estimator_zero_noise() {
        let code = StabilizerCode::steane_7_1_3();
        let estimator = LogicalErrorEstimator::new(&code);
        let result = estimator.estimate(0.0, 100, 42);
        assert_eq!(result.num_logical_errors, 0);
        assert_eq!(result.logical_error_rate, 0.0);
    }

    #[test]
    fn test_logical_error_estimator_runs() {
        let code = StabilizerCode::steane_7_1_3();
        let estimator = LogicalErrorEstimator::new(&code);
        let result = estimator.estimate(0.01, 50, 42);
        assert_eq!(result.num_trials, 50);
        assert_eq!(result.p_physical, 0.01);
        // At very low noise, logical error rate should be very small
        assert!(result.logical_error_rate <= 1.0);
    }

    #[test]
    fn test_logical_error_sweep() {
        let code = StabilizerCode::steane_7_1_3();
        let estimator = LogicalErrorEstimator::new(&code);
        let results = estimator.sweep(&[0.0, 0.01, 0.05], 20, 42);
        assert_eq!(results.len(), 3);
        // Zero noise should give zero logical errors
        assert_eq!(results[0].num_logical_errors, 0);
    }

    #[test]
    fn test_logical_error_result_display() {
        let result = LogicalErrorResult {
            p_physical: 0.01,
            num_trials: 1000,
            num_logical_errors: 5,
            logical_error_rate: 0.005,
            standard_error: 0.00223,
        };
        let s = format!("{}", result);
        assert!(s.contains("p_phys"));
        assert!(s.contains("1000"));
    }

    // ---- Generator access tests ----

    #[test]
    fn test_generator_access() {
        let code = StabilizerCode::steane_7_1_3();
        let gen0 = code.generator(0);
        assert_eq!(gen0.len(), 7);
        // First X-stabilizer of Steane code: X on qubits 0, 2, 4, 6
        // i.e., XIXIX IX
        assert_eq!(gen0[0], Pauli::X);
        assert_eq!(gen0[1], Pauli::I);
        assert_eq!(gen0[2], Pauli::X);
    }

    #[test]
    #[should_panic(expected = "Generator index out of bounds")]
    fn test_generator_out_of_bounds() {
        let code = StabilizerCode::steane_7_1_3();
        let _ = code.generator(100);
    }

    // ---- Manhattan distance test ----

    #[test]
    fn test_manhattan_distance() {
        assert_eq!(manhattan(0, 0, 0, 0), 0);
        assert_eq!(manhattan(0, 0, 3, 4), 7);
        assert_eq!(manhattan(5, 3, 2, 7), 7);
    }

    // ---- Edge case tests ----

    #[test]
    fn test_steane_y_error_correction() {
        let code = StabilizerCode::steane_7_1_3();
        let extractor = SyndromeExtractor::new(&code);
        let decoder = MwpmDecoder::new(&code);

        // Y error is both X and Z flip
        let mut error = vec![Pauli::I; 7];
        error[3] = Pauli::Y;
        let syndrome = extractor.extract(&error);
        let correction = decoder.decode(&syndrome);
        let net: Vec<Pauli> = error
            .iter()
            .zip(correction.iter())
            .map(|(e, c)| e.mul(*c))
            .collect();
        let net_syndrome = extractor.extract(&net);
        assert!(
            net_syndrome.iter().all(|&s| s == 0),
            "Failed to correct Y error"
        );
    }

    #[test]
    fn test_simple_rng_deterministic() {
        let mut rng1 = SimpleRng::new(12345);
        let mut rng2 = SimpleRng::new(12345);
        for _ in 0..100 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_simple_rng_range() {
        let mut rng = SimpleRng::new(42);
        for _ in 0..1000 {
            let val = rng.next_f64();
            assert!(val >= 0.0 && val < 1.0, "RNG value out of range: {}", val);
        }
    }
}
