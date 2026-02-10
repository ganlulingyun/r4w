//! Network Analyzer — VNA-Style Frequency Response & S-Parameter Extraction
//!
//! Virtual network analyzer for characterizing channels, filters, and RF networks.
//! Performs swept-frequency measurement to extract frequency response H(f), then
//! derives S-parameters (S11 return loss, S21 forward transmission), VSWR,
//! impedance, insertion loss, and group delay.
//!
//! Uses `(f64, f64)` tuples for complex values (real, imaginary) — no external
//! crate dependencies.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::network_analyzer::{NetworkAnalyzer, SweepConfig, SweepType};
//!
//! // Configure a linear sweep from 1 MHz to 100 MHz with 101 points
//! let config = SweepConfig {
//!     freq_start: 1e6,
//!     freq_stop: 100e6,
//!     num_points: 101,
//!     sweep_type: SweepType::Linear,
//!     reference_impedance: 50.0,
//! };
//! let analyzer = NetworkAnalyzer::new(config);
//!
//! // Generate swept-sine stimulus
//! let sample_rate = 250e6;
//! let stimulus = analyzer.generate_stimulus(sample_rate, 1024);
//! assert_eq!(stimulus.len(), 1024);
//!
//! // For a trivial pass-through (response == stimulus), H(f) ≈ 1+0j
//! let response = stimulus.clone();
//! let h = analyzer.measure_frequency_response(&stimulus, &response, sample_rate);
//! assert_eq!(h.len(), 101);
//!
//! // Magnitude of first bin should be close to 1.0
//! let mag = (h[0].0 * h[0].0 + h[0].1 * h[0].1).sqrt();
//! assert!((mag - 1.0).abs() < 0.15, "mag = {mag}");
//! ```

use std::f64::consts::PI;

// ── Complex-number helpers (re, im) tuples ──────────────────────────────────

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom == 0.0 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mag(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_phase(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// DFT of real-valued signal at a single normalized frequency `f_norm` (cycles/sample).
fn dft_at_freq(signal: &[f64], f_norm: f64) -> (f64, f64) {
    let n = signal.len() as f64;
    let mut re = 0.0;
    let mut im = 0.0;
    for (k, &s) in signal.iter().enumerate() {
        let angle = -2.0 * PI * f_norm * k as f64;
        re += s * angle.cos();
        im += s * angle.sin();
    }
    (re / n, im / n)
}

/// DFT of complex-valued signal at a single normalized frequency.
fn dft_complex_at_freq(signal: &[(f64, f64)], f_norm: f64) -> (f64, f64) {
    let n = signal.len() as f64;
    let mut re = 0.0;
    let mut im = 0.0;
    for (k, &(sr, si)) in signal.iter().enumerate() {
        let angle = -2.0 * PI * f_norm * k as f64;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        re += sr * cos_a - si * sin_a;
        im += sr * sin_a + si * cos_a;
    }
    (re / n, im / n)
}

// ── Public types ────────────────────────────────────────────────────────────

/// Sweep type for frequency sweep generation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepType {
    /// Linearly spaced frequency points.
    Linear,
    /// Logarithmically spaced frequency points.
    Log,
}

/// Configuration for a network analyzer frequency sweep.
#[derive(Debug, Clone)]
pub struct SweepConfig {
    /// Start frequency in Hz.
    pub freq_start: f64,
    /// Stop frequency in Hz.
    pub freq_stop: f64,
    /// Number of measurement points across the sweep.
    pub num_points: usize,
    /// Linear or logarithmic frequency spacing.
    pub sweep_type: SweepType,
    /// Reference impedance in ohms (typically 50 Ω).
    pub reference_impedance: f64,
}

impl Default for SweepConfig {
    fn default() -> Self {
        Self {
            freq_start: 1e6,
            freq_stop: 100e6,
            num_points: 201,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        }
    }
}

impl SweepConfig {
    /// Return the vector of frequency points for this sweep configuration.
    pub fn frequency_points(&self) -> Vec<f64> {
        if self.num_points <= 1 {
            return vec![self.freq_start];
        }
        match self.sweep_type {
            SweepType::Linear => {
                let step = (self.freq_stop - self.freq_start) / (self.num_points - 1) as f64;
                (0..self.num_points)
                    .map(|i| self.freq_start + step * i as f64)
                    .collect()
            }
            SweepType::Log => {
                let log_start = self.freq_start.ln();
                let log_stop = self.freq_stop.ln();
                let step = (log_stop - log_start) / (self.num_points - 1) as f64;
                (0..self.num_points)
                    .map(|i| (log_start + step * i as f64).exp())
                    .collect()
            }
        }
    }
}

/// S-parameter measurement results.
#[derive(Debug, Clone)]
pub struct SParameters {
    /// Frequency vector in Hz for each measurement point.
    pub frequencies: Vec<f64>,
    /// S11 (input reflection coefficient) at each frequency — complex (re, im).
    pub s11: Vec<(f64, f64)>,
    /// S21 (forward transmission coefficient) at each frequency — complex (re, im).
    pub s21: Vec<(f64, f64)>,
}

/// VNA-style network analyzer.
///
/// Generates stimulus signals and extracts frequency-domain characteristics
/// from stimulus/response pairs.
#[derive(Debug, Clone)]
pub struct NetworkAnalyzer {
    /// Sweep configuration.
    pub config: SweepConfig,
}

impl NetworkAnalyzer {
    /// Create a new network analyzer with the given sweep configuration.
    pub fn new(config: SweepConfig) -> Self {
        Self { config }
    }

    /// Generate a multi-tone stimulus signal (sum of sinusoids at each sweep freq).
    ///
    /// Returns a vector of `num_samples` real-valued samples at the given `sample_rate`.
    pub fn generate_stimulus(&self, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        let freqs = self.config.frequency_points();
        let amplitude = 1.0 / freqs.len() as f64;
        let mut signal = vec![0.0; num_samples];
        for &f in &freqs {
            for (n, s) in signal.iter_mut().enumerate() {
                *s += amplitude * (2.0 * PI * f * n as f64 / sample_rate).cos();
            }
        }
        signal
    }

    /// Measure frequency response H(f) = Y(f)/X(f) at each sweep frequency.
    ///
    /// `stimulus` and `response` are real-valued time-domain signal vectors sampled
    /// at `sample_rate` Hz. Returns a complex H(f) vector with one entry per sweep
    /// frequency point.
    pub fn measure_frequency_response(
        &self,
        stimulus: &[f64],
        response: &[f64],
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let freqs = self.config.frequency_points();
        freqs
            .iter()
            .map(|&f| {
                let f_norm = f / sample_rate;
                let x_f = dft_at_freq(stimulus, f_norm);
                let y_f = dft_at_freq(response, f_norm);
                c_div(y_f, x_f)
            })
            .collect()
    }

    /// Measure frequency response from complex-valued stimulus/response pairs.
    pub fn measure_frequency_response_complex(
        &self,
        stimulus: &[(f64, f64)],
        response: &[(f64, f64)],
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let freqs = self.config.frequency_points();
        freqs
            .iter()
            .map(|&f| {
                let f_norm = f / sample_rate;
                let x_f = dft_complex_at_freq(stimulus, f_norm);
                let y_f = dft_complex_at_freq(response, f_norm);
                c_div(y_f, x_f)
            })
            .collect()
    }

    /// Compute S-parameters from incident, reflected, and transmitted signals.
    ///
    /// - `incident`: stimulus signal injected at port 1
    /// - `reflected`: signal measured coming back from port 1
    /// - `transmitted`: signal measured at port 2
    ///
    /// S11 = Reflected/Incident, S21 = Transmitted/Incident.
    pub fn compute_s_parameters(
        &self,
        incident: &[f64],
        reflected: &[f64],
        transmitted: &[f64],
        sample_rate: f64,
    ) -> SParameters {
        let freqs = self.config.frequency_points();
        let mut s11 = Vec::with_capacity(freqs.len());
        let mut s21 = Vec::with_capacity(freqs.len());

        for &f in &freqs {
            let f_norm = f / sample_rate;
            let inc = dft_at_freq(incident, f_norm);
            let refl = dft_at_freq(reflected, f_norm);
            let trans = dft_at_freq(transmitted, f_norm);
            s11.push(c_div(refl, inc));
            s21.push(c_div(trans, inc));
        }

        SParameters {
            frequencies: freqs,
            s11,
            s21,
        }
    }

    /// Compute VSWR from S11: VSWR = (1 + |S11|) / (1 - |S11|).
    ///
    /// Returns one VSWR value per frequency point. Values are clamped to a
    /// maximum of 999.0 when |S11| ≈ 1 (total reflection).
    pub fn vswr(s11: &[(f64, f64)]) -> Vec<f64> {
        s11.iter()
            .map(|&s| {
                let mag = c_mag(s).min(0.9999);
                (1.0 + mag) / (1.0 - mag)
            })
            .collect()
    }

    /// Return loss in dB: RL = -20 log10(|S11|).
    pub fn return_loss_db(s11: &[(f64, f64)]) -> Vec<f64> {
        s11.iter()
            .map(|&s| {
                let mag = c_mag(s).max(1e-15);
                -20.0 * mag.log10()
            })
            .collect()
    }

    /// Insertion loss in dB: IL = -20 log10(|S21|).
    pub fn insertion_loss_db(s21: &[(f64, f64)]) -> Vec<f64> {
        s21.iter()
            .map(|&s| {
                let mag = c_mag(s).max(1e-15);
                -20.0 * mag.log10()
            })
            .collect()
    }

    /// Compute impedance from S11: Z = Z0 * (1 + S11) / (1 - S11).
    pub fn impedance_from_s11(s11: &[(f64, f64)], z0: f64) -> Vec<(f64, f64)> {
        s11.iter()
            .map(|&s| {
                let num = c_add((1.0, 0.0), s);
                let den = c_sub((1.0, 0.0), s);
                let ratio = c_div(num, den);
                (z0 * ratio.0, z0 * ratio.1)
            })
            .collect()
    }

    /// Group delay from S21 phase: τ_g = -dφ/dω (seconds).
    ///
    /// Uses central finite differences on the unwrapped phase of S21.
    /// The frequency vector must be provided to compute Δω.
    pub fn group_delay(s21: &[(f64, f64)], frequencies: &[f64]) -> Vec<f64> {
        let n = s21.len();
        if n < 2 {
            return vec![0.0; n];
        }

        // Unwrap phase
        let mut phase: Vec<f64> = s21.iter().map(|&s| c_phase(s)).collect();
        for i in 1..n {
            let mut diff = phase[i] - phase[i - 1];
            while diff > PI {
                diff -= 2.0 * PI;
            }
            while diff < -PI {
                diff += 2.0 * PI;
            }
            phase[i] = phase[i - 1] + diff;
        }

        // Central differences for group delay: -dφ/dω
        let mut gd = vec![0.0; n];
        // Forward difference at first point
        let dw0 = 2.0 * PI * (frequencies[1] - frequencies[0]);
        if dw0.abs() > 1e-30 {
            gd[0] = -(phase[1] - phase[0]) / dw0;
        }
        // Central differences for interior points
        for i in 1..n - 1 {
            let dw = 2.0 * PI * (frequencies[i + 1] - frequencies[i - 1]);
            if dw.abs() > 1e-30 {
                gd[i] = -(phase[i + 1] - phase[i - 1]) / dw;
            }
        }
        // Backward difference at last point
        let dwn = 2.0 * PI * (frequencies[n - 1] - frequencies[n - 2]);
        if dwn.abs() > 1e-30 {
            gd[n - 1] = -(phase[n - 1] - phase[n - 2]) / dwn;
        }

        gd
    }

    /// Magnitude response in dB: 20 log10(|H(f)|).
    pub fn magnitude_db(h: &[(f64, f64)]) -> Vec<f64> {
        h.iter()
            .map(|&v| {
                let mag = c_mag(v).max(1e-15);
                20.0 * mag.log10()
            })
            .collect()
    }

    /// Phase response in degrees.
    pub fn phase_degrees(h: &[(f64, f64)]) -> Vec<f64> {
        h.iter().map(|&v| c_phase(v) * 180.0 / PI).collect()
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_sweep_config_linear_frequencies() {
        let cfg = SweepConfig {
            freq_start: 1e6,
            freq_stop: 10e6,
            num_points: 10,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        };
        let freqs = cfg.frequency_points();
        assert_eq!(freqs.len(), 10);
        assert!(approx_eq(freqs[0], 1e6, 1.0));
        assert!(approx_eq(freqs[9], 10e6, 1.0));
        // Check uniform spacing
        let step = freqs[1] - freqs[0];
        assert!(approx_eq(step, 1e6, 1.0));
    }

    #[test]
    fn test_sweep_config_log_frequencies() {
        let cfg = SweepConfig {
            freq_start: 1e3,
            freq_stop: 1e6,
            num_points: 4,
            sweep_type: SweepType::Log,
            reference_impedance: 50.0,
        };
        let freqs = cfg.frequency_points();
        assert_eq!(freqs.len(), 4);
        assert!(approx_eq(freqs[0], 1e3, 1.0));
        assert!(approx_eq(freqs[3], 1e6, 1.0));
        // Log spacing: each step should be a factor of 10
        let ratio_01 = freqs[1] / freqs[0];
        let ratio_12 = freqs[2] / freqs[1];
        assert!(approx_eq(ratio_01, ratio_12, 0.01));
    }

    #[test]
    fn test_sweep_config_single_point() {
        let cfg = SweepConfig {
            freq_start: 5e6,
            freq_stop: 100e6,
            num_points: 1,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        };
        let freqs = cfg.frequency_points();
        assert_eq!(freqs.len(), 1);
        assert!(approx_eq(freqs[0], 5e6, 1.0));
    }

    #[test]
    fn test_generate_stimulus_length() {
        let analyzer = NetworkAnalyzer::new(SweepConfig {
            freq_start: 1e6,
            freq_stop: 10e6,
            num_points: 5,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        });
        let stim = analyzer.generate_stimulus(100e6, 512);
        assert_eq!(stim.len(), 512);
    }

    #[test]
    fn test_passthrough_frequency_response() {
        // When response == stimulus, H(f) should be ~1.0 at each freq
        let config = SweepConfig {
            freq_start: 1e6,
            freq_stop: 5e6,
            num_points: 5,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        };
        let analyzer = NetworkAnalyzer::new(config);
        let sample_rate = 20e6;
        let stim = analyzer.generate_stimulus(sample_rate, 2048);
        let resp = stim.clone();
        let h = analyzer.measure_frequency_response(&stim, &resp, sample_rate);
        assert_eq!(h.len(), 5);
        for (i, &val) in h.iter().enumerate() {
            let mag = c_mag(val);
            assert!(
                approx_eq(mag, 1.0, 0.15),
                "H(f) magnitude at point {i} = {mag}, expected ~1.0"
            );
        }
    }

    #[test]
    fn test_half_gain_frequency_response() {
        // If response = 0.5 * stimulus, then |H(f)| ≈ 0.5
        let config = SweepConfig {
            freq_start: 2e6,
            freq_stop: 8e6,
            num_points: 4,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        };
        let analyzer = NetworkAnalyzer::new(config);
        let sample_rate = 25e6;
        let stim = analyzer.generate_stimulus(sample_rate, 2048);
        let resp: Vec<f64> = stim.iter().map(|&x| 0.5 * x).collect();
        let h = analyzer.measure_frequency_response(&stim, &resp, sample_rate);
        for (i, &val) in h.iter().enumerate() {
            let mag = c_mag(val);
            assert!(
                approx_eq(mag, 0.5, 0.1),
                "H(f) magnitude at point {i} = {mag}, expected ~0.5"
            );
        }
    }

    #[test]
    fn test_compute_s_parameters_passthrough() {
        // Perfect match: no reflection (S11=0), full transmission (S21=1)
        let config = SweepConfig {
            freq_start: 1e6,
            freq_stop: 5e6,
            num_points: 3,
            sweep_type: SweepType::Linear,
            reference_impedance: 50.0,
        };
        let analyzer = NetworkAnalyzer::new(config);
        let sr = 20e6;
        let inc = analyzer.generate_stimulus(sr, 2048);
        let reflected: Vec<f64> = vec![0.0; inc.len()]; // no reflection
        let transmitted = inc.clone(); // perfect transmission
        let sp = analyzer.compute_s_parameters(&inc, &reflected, &transmitted, sr);
        assert_eq!(sp.s11.len(), 3);
        assert_eq!(sp.s21.len(), 3);
        for (i, &s11) in sp.s11.iter().enumerate() {
            let mag = c_mag(s11);
            assert!(
                mag < 0.15,
                "S11 magnitude at point {i} = {mag}, expected ~0"
            );
        }
        for (i, &s21) in sp.s21.iter().enumerate() {
            let mag = c_mag(s21);
            assert!(
                approx_eq(mag, 1.0, 0.15),
                "S21 magnitude at point {i} = {mag}, expected ~1"
            );
        }
    }

    #[test]
    fn test_vswr_perfect_match() {
        // S11 = 0 → VSWR = 1.0
        let s11 = vec![(0.0, 0.0); 5];
        let vswr = NetworkAnalyzer::vswr(&s11);
        for &v in &vswr {
            assert!(approx_eq(v, 1.0, 0.01));
        }
    }

    #[test]
    fn test_vswr_high_reflection() {
        // |S11| = 0.5 → VSWR = 3.0
        let s11 = vec![(0.5, 0.0); 3];
        let vswr = NetworkAnalyzer::vswr(&s11);
        for &v in &vswr {
            assert!(approx_eq(v, 3.0, 0.01));
        }
    }

    #[test]
    fn test_return_loss_db() {
        // |S11| = 0.1 → RL = 20 dB
        let s11 = vec![(0.1, 0.0)];
        let rl = NetworkAnalyzer::return_loss_db(&s11);
        assert!(approx_eq(rl[0], 20.0, 0.01));
    }

    #[test]
    fn test_insertion_loss_db() {
        // |S21| = 1.0 → IL = 0 dB  (lossless)
        // |S21| = 0.5 → IL = ~6.02 dB
        let s21 = vec![(1.0, 0.0), (0.5, 0.0)];
        let il = NetworkAnalyzer::insertion_loss_db(&s21);
        assert!(approx_eq(il[0], 0.0, 0.01));
        assert!(approx_eq(il[1], 6.0206, 0.01));
    }

    #[test]
    fn test_impedance_from_s11_matched() {
        // S11 = 0 → Z = Z0 = 50 Ω
        let s11 = vec![(0.0, 0.0)];
        let z = NetworkAnalyzer::impedance_from_s11(&s11, 50.0);
        assert!(approx_eq(z[0].0, 50.0, 0.01));
        assert!(approx_eq(z[0].1, 0.0, 0.01));
    }

    #[test]
    fn test_impedance_from_s11_open() {
        // S11 ≈ 1 → Z → ∞ (open circuit)
        let s11 = vec![(0.999, 0.0)];
        let z = NetworkAnalyzer::impedance_from_s11(&s11, 50.0);
        assert!(z[0].0 > 1000.0, "Open circuit impedance should be very high");
    }

    #[test]
    fn test_impedance_from_s11_short() {
        // S11 ≈ -1 → Z → 0 (short circuit)
        let s11 = vec![(-0.999, 0.0)];
        let z = NetworkAnalyzer::impedance_from_s11(&s11, 50.0);
        assert!(z[0].0.abs() < 1.0, "Short circuit impedance should be near 0, got {}", z[0].0);
    }

    #[test]
    fn test_group_delay_constant_phase() {
        // Constant phase → zero group delay
        let s21: Vec<(f64, f64)> = (0..10).map(|_| (1.0, 0.0)).collect();
        let freqs: Vec<f64> = (0..10).map(|i| 1e6 + i as f64 * 1e6).collect();
        let gd = NetworkAnalyzer::group_delay(&s21, &freqs);
        for &g in &gd {
            assert!(g.abs() < 1e-10, "Expected zero group delay, got {g}");
        }
    }

    #[test]
    fn test_group_delay_linear_phase() {
        // Linear phase with known slope → constant group delay
        // φ(f) = -2π * τ * f  →  group delay = τ
        let tau = 1e-8; // 10 ns group delay
        let freqs: Vec<f64> = (0..20).map(|i| 10e6 + i as f64 * 0.5e6).collect();
        let s21: Vec<(f64, f64)> = freqs
            .iter()
            .map(|&f| {
                let phase = -2.0 * PI * tau * f;
                (phase.cos(), phase.sin())
            })
            .collect();
        let gd = NetworkAnalyzer::group_delay(&s21, &freqs);
        // Interior points should all be ~τ
        for (i, &g) in gd.iter().enumerate().skip(1).take(gd.len() - 2) {
            assert!(
                approx_eq(g, tau, 1e-12),
                "Group delay at point {i} = {g}, expected {tau}"
            );
        }
    }

    #[test]
    fn test_magnitude_db_unity() {
        let h = vec![(1.0, 0.0), (0.0, 1.0)]; // both have magnitude 1
        let db = NetworkAnalyzer::magnitude_db(&h);
        assert!(approx_eq(db[0], 0.0, 0.01));
        assert!(approx_eq(db[1], 0.0, 0.01));
    }

    #[test]
    fn test_phase_degrees() {
        let h = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let ph = NetworkAnalyzer::phase_degrees(&h);
        assert!(approx_eq(ph[0], 0.0, 0.01));
        assert!(approx_eq(ph[1], 90.0, 0.01));
        assert!(approx_eq(ph[2], 180.0, 0.01));
    }

    #[test]
    fn test_default_sweep_config() {
        let cfg = SweepConfig::default();
        assert!(approx_eq(cfg.freq_start, 1e6, 1.0));
        assert!(approx_eq(cfg.freq_stop, 100e6, 1.0));
        assert_eq!(cfg.num_points, 201);
        assert_eq!(cfg.sweep_type, SweepType::Linear);
        assert!(approx_eq(cfg.reference_impedance, 50.0, 0.01));
    }

    #[test]
    fn test_complex_helpers() {
        // Verify internal complex arithmetic
        assert_eq!(c_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0));
        let d = c_div((1.0, 0.0), (0.0, 1.0));
        assert!(approx_eq(d.0, 0.0, 1e-12));
        assert!(approx_eq(d.1, -1.0, 1e-12));
        assert_eq!(c_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(c_sub((5.0, 3.0), (2.0, 1.0)), (3.0, 2.0));
        assert!(approx_eq(c_mag((3.0, 4.0)), 5.0, 1e-12));
        assert!(approx_eq(c_phase((1.0, 0.0)), 0.0, 1e-12));
        assert_eq!(c_conj((1.0, 2.0)), (1.0, -2.0));
    }
}
