//! Reconfigurable Intelligent Surface (RIS) Phase Controller
//!
//! Computes optimal phase shifts for RIS elements to steer and enhance
//! reflected signals between a transmitter and receiver. Supports continuous
//! and quantized (discrete) phase configurations, far-field channel models,
//! beam pattern evaluation, codebook search, and theoretical SNR gain analysis.
//!
//! # Example
//!
//! ```
//! use r4w_core::ris_phase_controller::{RisConfig, RisPhaseController};
//!
//! // Create a 4x4 RIS at 28 GHz with 2-bit phase quantization
//! let config = RisConfig {
//!     num_elements_x: 4,
//!     num_elements_y: 4,
//!     element_spacing_m: 0.005,
//!     frequency_hz: 28.0e9,
//!     phase_bits: 2,
//! };
//! let controller = RisPhaseController::new(config);
//!
//! // TX at (10, 0, 0), RX at (-5, 3, 0), RIS at origin
//! let tx_pos = [10.0, 0.0, 0.0];
//! let rx_pos = [-5.0, 3.0, 0.0];
//!
//! // Compute optimal continuous phases then quantize
//! let phases = controller.compute_phases(&tx_pos, &rx_pos);
//! let quantized = controller.quantize_phases(&phases);
//!
//! // Theoretical SNR gain for 16 elements with perfect CSI
//! let gain = controller.snr_gain_db();
//! assert!(gain > 20.0); // ~24 dB for N=16
//! ```

use std::f64::consts::PI;

/// Complex number as (re, im) tuple.
type Complex = (f64, f64);

// ── helpers ────────────────────────────────────────────────────────────

fn cx_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn cx_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[allow(dead_code)]
fn cx_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

fn cx_mag(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn cx_angle(a: Complex) -> f64 {
    a.1.atan2(a.0)
}

fn cx_from_polar(mag: f64, phase: f64) -> Complex {
    (mag * phase.cos(), mag * phase.sin())
}

// ── configuration ──────────────────────────────────────────────────────

/// Configuration for a planar RIS array.
#[derive(Debug, Clone)]
pub struct RisConfig {
    /// Number of elements along X axis.
    pub num_elements_x: usize,
    /// Number of elements along Y axis.
    pub num_elements_y: usize,
    /// Element spacing in metres (typically λ/2).
    pub element_spacing_m: f64,
    /// Carrier frequency in Hz.
    pub frequency_hz: f64,
    /// Phase quantisation bits (0 = continuous, 1 = 2 levels, 2 = 4 levels, …).
    pub phase_bits: u8,
}

impl RisConfig {
    /// Total number of RIS elements.
    pub fn num_elements(&self) -> usize {
        self.num_elements_x * self.num_elements_y
    }

    /// Wavelength in metres.
    pub fn wavelength(&self) -> f64 {
        2.998e8 / self.frequency_hz
    }

    /// Wave-number (2π / λ).
    pub fn wavenumber(&self) -> f64 {
        2.0 * PI / self.wavelength()
    }

    /// Number of discrete phase levels (2^phase_bits). Returns 0 for continuous.
    pub fn num_phase_levels(&self) -> usize {
        if self.phase_bits == 0 {
            0
        } else {
            1 << self.phase_bits
        }
    }
}

// ── state ──────────────────────────────────────────────────────────────

/// Current phase configuration of the RIS.
#[derive(Debug, Clone)]
pub struct RisState {
    /// Phase shift per element in radians, row-major order.
    pub phases: Vec<f64>,
}

// ── controller ─────────────────────────────────────────────────────────

/// Phase controller for a planar RIS.
#[derive(Debug, Clone)]
pub struct RisPhaseController {
    /// Array configuration.
    pub config: RisConfig,
    /// Current element phases.
    pub state: RisState,
}

impl RisPhaseController {
    /// Create a new controller with all phases initialised to zero.
    pub fn new(config: RisConfig) -> Self {
        let n = config.num_elements();
        Self {
            config,
            state: RisState {
                phases: vec![0.0; n],
            },
        }
    }

    // ── geometry helpers ────────────────────────────────────────────

    /// (x, y, z) position of element (ix, iy) in the RIS local frame.
    /// The array centre is at the origin.
    fn element_position(&self, ix: usize, iy: usize) -> [f64; 3] {
        let d = self.config.element_spacing_m;
        let cx = (self.config.num_elements_x as f64 - 1.0) / 2.0;
        let cy = (self.config.num_elements_y as f64 - 1.0) / 2.0;
        [
            (ix as f64 - cx) * d,
            (iy as f64 - cy) * d,
            0.0,
        ]
    }

    /// Euclidean distance between two 3-D points.
    fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    // ── channel model ──────────────────────────────────────────────

    /// Far-field complex channel coefficient from `src` to element `(ix,iy)`.
    ///
    /// Uses free-space path-loss (1/r) amplitude and phase = −k·r, where
    /// `k` is the wave-number and `r` is the distance.
    fn channel_coeff(&self, src: &[f64; 3], ix: usize, iy: usize) -> Complex {
        let elem = self.element_position(ix, iy);
        let r = Self::distance(src, &elem);
        let k = self.config.wavenumber();
        let amplitude = 1.0 / r;
        cx_from_polar(amplitude, -k * r)
    }

    /// Compute the full channel model for all elements.
    ///
    /// Returns `(h_tx, h_rx)` where each is a `Vec<Complex>` of length
    /// `num_elements`, representing the TX→element and element→RX
    /// far-field channel coefficients respectively.
    pub fn channel_model(
        &self,
        tx_pos: &[f64; 3],
        rx_pos: &[f64; 3],
    ) -> (Vec<Complex>, Vec<Complex>) {
        let n = self.config.num_elements();
        let mut h_tx = Vec::with_capacity(n);
        let mut h_rx = Vec::with_capacity(n);
        for iy in 0..self.config.num_elements_y {
            for ix in 0..self.config.num_elements_x {
                h_tx.push(self.channel_coeff(tx_pos, ix, iy));
                h_rx.push(self.channel_coeff(rx_pos, ix, iy));
            }
        }
        (h_tx, h_rx)
    }

    // ── phase computation ──────────────────────────────────────────

    /// Compute optimal (continuous) phase shifts for coherent combining.
    ///
    /// For each element *n*, the optimal phase is
    ///
    /// ```text
    /// phi_n = -angle(h_tx_n) - angle(h_rx_n)
    /// ```
    ///
    /// which aligns all element contributions in-phase at the receiver.
    pub fn compute_phases(&self, tx_pos: &[f64; 3], rx_pos: &[f64; 3]) -> Vec<f64> {
        let (h_tx, h_rx) = self.channel_model(tx_pos, rx_pos);
        h_tx.iter()
            .zip(h_rx.iter())
            .map(|(ht, hr)| {
                let phi = -cx_angle(*ht) - cx_angle(*hr);
                // Wrap to [-pi, pi)
                wrap_phase(phi)
            })
            .collect()
    }

    /// Quantise a vector of continuous phases to discrete levels determined
    /// by `config.phase_bits`. If `phase_bits == 0` the phases are returned
    /// unchanged.
    pub fn quantize_phases(&self, phases: &[f64]) -> Vec<f64> {
        let levels = self.config.num_phase_levels();
        if levels == 0 {
            return phases.to_vec();
        }
        let step = 2.0 * PI / levels as f64;
        phases
            .iter()
            .map(|&p| {
                let wrapped = wrap_phase(p);
                // Shift to [0, 2pi) for quantisation
                let pos = if wrapped < 0.0 { wrapped + 2.0 * PI } else { wrapped };
                let idx = ((pos / step).round() as usize) % levels;
                let q = idx as f64 * step;
                wrap_phase(q)
            })
            .collect()
    }

    /// Apply a phase configuration to the controller state.
    pub fn apply_phases(&mut self, phases: Vec<f64>) {
        assert_eq!(phases.len(), self.config.num_elements());
        self.state.phases = phases;
    }

    // ── random baseline ────────────────────────────────────────────

    /// Generate uniformly random phases in [-pi, pi) using a simple LCG.
    ///
    /// The `seed` provides reproducibility.
    pub fn random_phases(&self, seed: u64) -> Vec<f64> {
        let n = self.config.num_elements();
        let mut phases = Vec::with_capacity(n);
        // Simple LCG (Numerical Recipes constants)
        let mut s = seed;
        for _ in 0..n {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u = (s >> 11) as f64 / (1u64 << 53) as f64; // [0, 1)
            phases.push(u * 2.0 * PI - PI);
        }
        phases
    }

    // ── beam pattern ───────────────────────────────────────────────

    /// Compute the reflected beam pattern in a plane at fixed phi_az = 0.
    ///
    /// Returns a vector of `(theta_deg, magnitude)` pairs for theta in
    /// `[-90, +90]` degrees with `num_points` steps. The pattern uses the
    /// current `state.phases`.
    pub fn beam_pattern(&self, num_points: usize) -> Vec<(f64, f64)> {
        let k = self.config.wavenumber();
        let d = self.config.element_spacing_m;
        let mut pattern = Vec::with_capacity(num_points);

        let cx_center = (self.config.num_elements_x as f64 - 1.0) / 2.0;

        for i in 0..num_points {
            let theta = -PI / 2.0 + PI * i as f64 / (num_points - 1) as f64;
            let sin_theta = theta.sin();

            let mut sum: Complex = (0.0, 0.0);

            for iy in 0..self.config.num_elements_y {
                for ix in 0..self.config.num_elements_x {
                    let idx = iy * self.config.num_elements_x + ix;
                    let phi_n = self.state.phases[idx];

                    // For a 1-D cut at phi=0, only x contributes to the
                    // far-field phase term k * x * sin(theta).
                    let x_offset = (ix as f64 - cx_center) * d;
                    let spatial_phase = k * x_offset * sin_theta;
                    let total_phase = phi_n + spatial_phase;
                    sum = cx_add(sum, cx_from_polar(1.0, total_phase));
                }
            }

            let mag = cx_mag(sum);
            pattern.push((theta.to_degrees(), mag));
        }
        pattern
    }

    // ── SNR gain ───────────────────────────────────────────────────

    /// Theoretical maximum SNR gain in dB from using the RIS.
    ///
    /// With perfect CSI and continuous phases, the coherent combining
    /// gain scales as N^2 where N is the total number of elements.
    pub fn snr_gain_db(&self) -> f64 {
        let n = self.config.num_elements() as f64;
        20.0 * n.log10()
    }

    /// Effective SNR gain in dB for the given phase configuration,
    /// relative to a single-element reflection.
    ///
    /// Evaluates the composite channel with the supplied phases and
    /// compares the received power to the single-element case.
    pub fn effective_snr_gain_db(
        &self,
        tx_pos: &[f64; 3],
        rx_pos: &[f64; 3],
        phases: &[f64],
    ) -> f64 {
        let (h_tx, h_rx) = self.channel_model(tx_pos, rx_pos);

        // Composite channel: sum of h_tx_n * exp(j*phi_n) * h_rx_n
        let mut composite: Complex = (0.0, 0.0);
        for (i, (&ht, &hr)) in h_tx.iter().zip(h_rx.iter()).enumerate() {
            let phase_shift = cx_from_polar(1.0, phases[i]);
            let reflected = cx_mul(cx_mul(ht, phase_shift), hr);
            composite = cx_add(composite, reflected);
        }
        let power_ris = cx_mag(composite).powi(2);

        // Single element reference (element 0, phase 0)
        let single = cx_mul(h_tx[0], h_rx[0]);
        let power_single = cx_mag(single).powi(2);

        10.0 * (power_ris / power_single).log10()
    }

    // ── codebook search ────────────────────────────────────────────

    /// Generate a DFT-based codebook of beam-steering vectors.
    ///
    /// Returns `num_entries` phase configurations, each a `Vec<f64>`.
    pub fn generate_codebook(&self, num_entries: usize) -> Vec<Vec<f64>> {
        let n = self.config.num_elements();
        let mut codebook = Vec::with_capacity(num_entries);

        for idx in 0..num_entries {
            let mut phases = Vec::with_capacity(n);
            for elem in 0..n {
                let phi = 2.0 * PI * (idx as f64) * (elem as f64) / (num_entries as f64);
                phases.push(wrap_phase(phi));
            }
            codebook.push(phases);
        }
        codebook
    }

    /// Search through a codebook of phase configurations and return the
    /// one that maximises received power for the given TX/RX positions.
    ///
    /// Returns `(best_index, best_phases, best_gain_db)`.
    pub fn codebook_search(
        &self,
        tx_pos: &[f64; 3],
        rx_pos: &[f64; 3],
        codebook: &[Vec<f64>],
    ) -> (usize, Vec<f64>, f64) {
        let mut best_idx = 0;
        let mut best_gain = f64::NEG_INFINITY;

        for (i, phases) in codebook.iter().enumerate() {
            let gain = self.effective_snr_gain_db(tx_pos, rx_pos, phases);
            if gain > best_gain {
                best_gain = gain;
                best_idx = i;
            }
        }

        (best_idx, codebook[best_idx].clone(), best_gain)
    }
}

// ── utilities ──────────────────────────────────────────────────────────

/// Wrap angle to [-pi, pi).
fn wrap_phase(mut phi: f64) -> f64 {
    phi %= 2.0 * PI;
    if phi >= PI {
        phi -= 2.0 * PI;
    } else if phi < -PI {
        phi += 2.0 * PI;
    }
    phi
}

// ── tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> RisConfig {
        RisConfig {
            num_elements_x: 4,
            num_elements_y: 4,
            element_spacing_m: 0.005,
            frequency_hz: 28.0e9,
            phase_bits: 2,
        }
    }

    #[test]
    fn test_config_num_elements() {
        let cfg = default_config();
        assert_eq!(cfg.num_elements(), 16);
    }

    #[test]
    fn test_config_wavelength() {
        let cfg = default_config();
        let lambda = cfg.wavelength();
        // 28 GHz -> lambda ~ 0.01071 m
        assert!((lambda - 0.01071).abs() < 0.001);
    }

    #[test]
    fn test_config_phase_levels() {
        let mut cfg = default_config();
        assert_eq!(cfg.num_phase_levels(), 4); // 2 bits -> 4 levels
        cfg.phase_bits = 1;
        assert_eq!(cfg.num_phase_levels(), 2);
        cfg.phase_bits = 0;
        assert_eq!(cfg.num_phase_levels(), 0); // continuous
    }

    #[test]
    fn test_element_positions_symmetric() {
        let ctrl = RisPhaseController::new(default_config());
        // For a 4x4 array the centre elements should be symmetric about origin
        let p00 = ctrl.element_position(0, 0);
        let p33 = ctrl.element_position(3, 3);
        assert!((p00[0] + p33[0]).abs() < 1e-12);
        assert!((p00[1] + p33[1]).abs() < 1e-12);
    }

    #[test]
    fn test_channel_model_reciprocity_magnitude() {
        let ctrl = RisPhaseController::new(default_config());
        let tx = [10.0, 0.0, 0.0];
        let rx = [10.0, 0.0, 0.0]; // same position
        let (h_tx, h_rx) = ctrl.channel_model(&tx, &rx);
        // Magnitudes should be identical
        for (ht, hr) in h_tx.iter().zip(h_rx.iter()) {
            assert!((cx_mag(*ht) - cx_mag(*hr)).abs() < 1e-12);
        }
    }

    #[test]
    fn test_compute_phases_length() {
        let ctrl = RisPhaseController::new(default_config());
        let phases = ctrl.compute_phases(&[10.0, 0.0, 0.0], &[-5.0, 3.0, 0.0]);
        assert_eq!(phases.len(), 16);
        // All phases should be in [-pi, pi)
        for &p in &phases {
            assert!(p >= -PI && p < PI, "phase {} out of range", p);
        }
    }

    #[test]
    fn test_quantize_phases_discrete_levels() {
        let ctrl = RisPhaseController::new(default_config());
        // With 2 bits -> 4 levels: 0, pi/2, pi, -pi/2
        let input = vec![0.1, 1.5, 3.0, -1.4];
        let q = ctrl.quantize_phases(&input);
        let step = PI / 2.0;
        for &p in &q {
            // Each quantised phase should be a multiple of pi/2
            let normalised = (p / step).round();
            assert!(
                (p - normalised * step).abs() < 1e-10,
                "quantised phase {} is not a multiple of pi/2",
                p,
            );
        }
    }

    #[test]
    fn test_quantize_continuous_passthrough() {
        let mut cfg = default_config();
        cfg.phase_bits = 0; // continuous
        let ctrl = RisPhaseController::new(cfg);
        let input = vec![0.123, -2.5, 1.0, 0.0];
        let q = ctrl.quantize_phases(&input);
        assert_eq!(input, q);
    }

    #[test]
    fn test_snr_gain_db_n_squared() {
        let ctrl = RisPhaseController::new(default_config());
        let gain = ctrl.snr_gain_db();
        // N=16 -> 20*log10(16) ~ 24.08 dB
        assert!((gain - 24.08).abs() < 0.1, "gain = {}", gain);
    }

    #[test]
    fn test_optimal_beats_random() {
        let ctrl = RisPhaseController::new(default_config());
        let tx = [10.0, 0.0, 0.0];
        let rx = [-5.0, 3.0, 0.0];

        let optimal = ctrl.compute_phases(&tx, &rx);
        let random = ctrl.random_phases(42);

        let g_opt = ctrl.effective_snr_gain_db(&tx, &rx, &optimal);
        let g_rand = ctrl.effective_snr_gain_db(&tx, &rx, &random);

        assert!(
            g_opt > g_rand,
            "optimal gain ({:.2} dB) should exceed random ({:.2} dB)",
            g_opt,
            g_rand,
        );
    }

    #[test]
    fn test_beam_pattern_peak() {
        let mut ctrl = RisPhaseController::new(default_config());
        // All phases zero -> peak at broadside (theta = 0)
        ctrl.apply_phases(vec![0.0; 16]);
        let pattern = ctrl.beam_pattern(181);
        // Find peak
        let (peak_theta, _peak_mag) = pattern
            .iter()
            .cloned()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert!(
            peak_theta.abs() < 2.0,
            "peak should be near 0 deg, got {} deg",
            peak_theta,
        );
    }

    #[test]
    fn test_codebook_search_finds_best() {
        let ctrl = RisPhaseController::new(default_config());
        let tx = [10.0, 0.0, 0.0];
        let rx = [-5.0, 3.0, 0.0];
        let codebook = ctrl.generate_codebook(64);
        let (best_idx, _best_phases, best_gain) = ctrl.codebook_search(&tx, &rx, &codebook);

        // Verify best is actually best
        for (i, entry) in codebook.iter().enumerate() {
            let g = ctrl.effective_snr_gain_db(&tx, &rx, entry);
            assert!(
                g <= best_gain + 1e-9,
                "entry {} ({:.2} dB) beats reported best {} ({:.2} dB)",
                i,
                g,
                best_idx,
                best_gain,
            );
        }
    }

    #[test]
    fn test_random_phases_reproducible() {
        let ctrl = RisPhaseController::new(default_config());
        let a = ctrl.random_phases(99);
        let b = ctrl.random_phases(99);
        assert_eq!(a, b);
    }

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0) - 0.0).abs() < 1e-12);
        assert!((wrap_phase(2.0 * PI) - 0.0).abs() < 1e-12);
        assert!((wrap_phase(-PI) - (-PI)).abs() < 1e-12);
        assert!((wrap_phase(PI + 0.01) - (-PI + 0.01)).abs() < 1e-10);
    }
}
