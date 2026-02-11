//! # Blade Tip Timing (BTT) Analysis
//!
//! Implements Blade Tip Timing analysis for gas turbine / jet engine health
//! monitoring.  Fixed probes around the engine casing record the arrival time
//! of each blade tip.  Deviations from the expected arrival (computed from RPM
//! and blade geometry) reveal blade vibration amplitude, frequency content,
//! crack growth, and foreign-object damage (FOD).
//!
//! ## Overview
//!
//! ```text
//!   Probe 1 ──┐
//!   Probe 2 ──┤  arrival_times[probe][blade]
//!   Probe 3 ──┤        │
//!   Probe 4 ──┘        ▼
//!              ┌────────────────┐
//!              │ BttProcessor   │
//!              │  • deflection  │
//!              │  • sinusoid fit│
//!              │  • TWA         │
//!              │  • alerting    │
//!              └───────┬────────┘
//!                      ▼
//!              BladeHealthReport
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::turbine_blade_tip_timing::{BttConfig, BttProcessor};
//!
//! let config = BttConfig {
//!     num_blades: 60,
//!     num_probes: 4,
//!     rpm: 10000.0,
//!     probe_angles_deg: vec![0.0, 90.0, 180.0, 270.0],
//!     sample_rate_hz: 1_000_000.0,
//! };
//! let mut processor = BttProcessor::new(config);
//! ```

use std::f64::consts::PI;

// ───────────────────────────────────────────────────────────────────────────
// Configuration
// ───────────────────────────────────────────────────────────────────────────

/// Configuration for a Blade Tip Timing measurement system.
#[derive(Debug, Clone)]
pub struct BttConfig {
    /// Number of blades on the rotor.
    pub num_blades: usize,
    /// Number of circumferential probes.
    pub num_probes: usize,
    /// Rotor speed in revolutions per minute.
    pub rpm: f64,
    /// Angular positions of each probe in degrees (0 = reference).
    pub probe_angles_deg: Vec<f64>,
    /// Acquisition sample rate in Hz (determines timing resolution).
    pub sample_rate_hz: f64,
}

// ───────────────────────────────────────────────────────────────────────────
// Alerts
// ───────────────────────────────────────────────────────────────────────────

/// Classification of a blade health alert.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertType {
    /// Blade deflection exceeds configured threshold.
    ExcessiveVibration,
    /// Blade natural frequency has shifted (possible crack growth).
    FrequencyShift,
    /// No arrival pulse detected for a blade.
    MissingBlade,
    /// Deflection pattern consistent with fatigue crack.
    CrackSuspected,
    /// Sudden broadband deflection increase (foreign-object damage).
    FodDamage,
}

/// A single health alert for one blade.
#[derive(Debug, Clone)]
pub struct BladeAlert {
    /// Zero-based blade index.
    pub blade_index: usize,
    /// Type of alert.
    pub alert_type: AlertType,
    /// Severity in the range 0.0 (informational) to 1.0 (critical).
    pub severity: f64,
}

// ───────────────────────────────────────────────────────────────────────────
// Health report
// ───────────────────────────────────────────────────────────────────────────

/// Per-revolution health report produced by [`BttProcessor`].
#[derive(Debug, Clone)]
pub struct BladeHealthReport {
    /// Peak-to-peak deflection for each blade in millimetres.
    pub blade_deflections_mm: Vec<f64>,
    /// Dominant vibration frequency for each blade in Hz.
    pub blade_frequencies_hz: Vec<f64>,
    /// Active alerts.
    pub alerts: Vec<BladeAlert>,
    /// Cumulative revolution count since processor creation.
    pub revolution_count: usize,
}

// ───────────────────────────────────────────────────────────────────────────
// Traveling-wave analysis
// ───────────────────────────────────────────────────────────────────────────

/// Direction of a traveling wave around the rotor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaveDirection {
    /// Co-rotating with the rotor.
    Forward,
    /// Counter-rotating.
    Backward,
}

/// A single traveling-wave component extracted from circumferential
/// deflection data.
#[derive(Debug, Clone)]
pub struct TravelingWave {
    /// Engine order (frequency normalised to rotation speed).
    pub engine_order: f64,
    /// Amplitude in millimetres.
    pub amplitude_mm: f64,
    /// Nodal diameter count.
    pub nodal_diameter: usize,
    /// Propagation direction.
    pub direction: WaveDirection,
}

// ───────────────────────────────────────────────────────────────────────────
// Core math helpers (no external crate deps)
// ───────────────────────────────────────────────────────────────────────────

/// Convert degrees to radians.
#[inline]
fn deg2rad(d: f64) -> f64 {
    d * PI / 180.0
}

/// Convert radians to degrees.
#[inline]
#[allow(dead_code)]
fn rad2deg(r: f64) -> f64 {
    r * 180.0 / PI
}

/// Compute the expected time-of-arrival of `blade_index` at a probe
/// located at `probe_angle_deg`, given the rotor speed `rpm`.
///
/// The reference is blade 0 crossing the 0-degree probe at t = 0.
pub fn compute_expected_arrival(blade_index: usize, probe_angle_deg: f64, rpm: f64) -> f64 {
    assert!(rpm > 0.0, "RPM must be positive");
    let rev_period = 60.0 / rpm;
    // Without knowing num_blades, this function computes the arrival time
    // for the probe angle only.  Use `expected_arrival_full` when the total
    // blade count is available.  The blade_index is accepted for API
    // compatibility but ignored here.
    let _ = blade_index;
    (probe_angle_deg / 360.0) * rev_period
}

/// Compute the expected arrival time of blade `blade_index` (0-based) at a
/// probe located at `probe_angle_deg`, given `rpm` and `num_blades` evenly
/// spaced blades.
///
/// Returns the time in seconds from t=0 (blade 0 crossing 0-degree probe).
pub fn expected_arrival_full(
    blade_index: usize,
    probe_angle_deg: f64,
    rpm: f64,
    num_blades: usize,
) -> f64 {
    assert!(rpm > 0.0, "RPM must be positive");
    assert!(num_blades > 0, "Must have at least one blade");
    let rev_period = 60.0 / rpm;
    let blade_pitch_deg = 360.0 / num_blades as f64;
    let total_angle = probe_angle_deg + blade_index as f64 * blade_pitch_deg;
    // Wrap into [0, 360)
    let wrapped = total_angle.rem_euclid(360.0);
    (wrapped / 360.0) * rev_period
}

/// Convert a time-of-arrival deviation into a blade-tip deflection in mm.
///
/// * `expected_toa` - expected arrival time (s)
/// * `measured_toa` - measured arrival time (s)
/// * `tip_radius_m` - blade tip radius in metres
/// * `rpm`          - rotor speed (rev/min)
///
/// The blade tip linear velocity is `v = 2π * tip_radius * (rpm/60)`.
/// A timing deviation Δt maps to a circumferential deflection `v * Δt`.
pub fn deflection_from_toa(
    expected_toa: f64,
    measured_toa: f64,
    tip_radius_m: f64,
    rpm: f64,
) -> f64 {
    let omega = 2.0 * PI * rpm / 60.0; // rad/s
    let tip_speed = omega * tip_radius_m; // m/s
    let dt = measured_toa - expected_toa;
    tip_speed * dt * 1000.0 // convert m to mm
}

/// Least-squares sinusoidal fit of the form `A * sin(n * theta + phi) + C`.
///
/// Given probe angles (radians) and corresponding deflections, returns
/// `(amplitude, engine_order, phase)`.
///
/// Engine order is scanned over integer values 1..N/2 and the best fit is
/// returned.  The fit for each candidate engine order `n` uses the
/// pseudoinverse of the design matrix `[sin(n*θ), cos(n*θ), 1]`.
pub fn fit_sinusoid_lsq(
    probe_angles: &[f64],
    deflections: &[f64],
) -> (f64, f64, f64) {
    assert_eq!(
        probe_angles.len(),
        deflections.len(),
        "angles and deflections must have the same length"
    );
    let m = probe_angles.len();
    assert!(m >= 3, "need at least 3 data points for sinusoidal fit");

    // Scan integer engine orders from 1 to max_order.
    let max_order = (m / 2).max(1).max(20);

    let mut best_residual = f64::MAX;
    let mut best_a = 0.0;
    let mut best_n = 1.0_f64;
    let mut best_phi = 0.0;

    for n in 1..=max_order {
        let nf = n as f64;
        // Build normal equations for model y = a1*sin(n*θ) + a2*cos(n*θ) + a3
        // A^T A x = A^T y   (3x3 system)
        let mut ata = [[0.0_f64; 3]; 3];
        let mut aty = [0.0_f64; 3];

        for i in 0..m {
            let s = (nf * probe_angles[i]).sin();
            let c = (nf * probe_angles[i]).cos();
            let row = [s, c, 1.0];
            let y = deflections[i];
            for r in 0..3 {
                for col in 0..3 {
                    ata[r][col] += row[r] * row[col];
                }
                aty[r] += row[r] * y;
            }
        }

        // Solve 3x3 system via Cramer's rule
        if let Some(x) = solve_3x3(&ata, &aty) {
            let a1 = x[0];
            let a2 = x[1];
            let _a3 = x[2];

            // Compute residual
            let mut res = 0.0;
            for i in 0..m {
                let s = (nf * probe_angles[i]).sin();
                let c = (nf * probe_angles[i]).cos();
                let pred = a1 * s + a2 * c + _a3;
                let e = deflections[i] - pred;
                res += e * e;
            }

            if res < best_residual {
                best_residual = res;
                best_a = (a1 * a1 + a2 * a2).sqrt();
                best_n = nf;
                best_phi = a2.atan2(a1);
                // A*sin(n*θ + φ) = A*(sin(n*θ)cos(φ) + cos(n*θ)sin(φ))
                // so a1 = A*cos(φ), a2 = A*sin(φ) → φ = atan2(a2, a1)
            }
        }
    }

    (best_a, best_n, best_phi)
}

/// Solve a 3x3 linear system `A x = b` using Cramer's rule.
/// Returns `None` if the determinant is near zero.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let det = det3(a);
    if det.abs() < 1e-30 {
        return None;
    }
    let inv = 1.0 / det;

    // Replace column k with b, compute determinant
    let mut x = [0.0; 3];
    for k in 0..3 {
        let mut ak = *a;
        for r in 0..3 {
            ak[r][k] = b[r];
        }
        x[k] = det3(&ak) * inv;
    }
    Some(x)
}

/// Determinant of a 3x3 matrix.
fn det3(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

/// Perform traveling-wave analysis on blade deflections.
///
/// `blade_deflections[probe][blade]` holds the deflection (mm) of each blade
/// as seen by each probe.  Returns the dominant traveling-wave components.
///
/// The method computes a spatial DFT around the blade ring for each probe,
/// then averages across probes to identify nodal diameters and their
/// amplitudes.
pub fn traveling_wave_analysis(
    blade_deflections: &[Vec<f64>],
    rpm: f64,
) -> Vec<TravelingWave> {
    if blade_deflections.is_empty() {
        return Vec::new();
    }
    let num_blades = blade_deflections[0].len();
    if num_blades == 0 {
        return Vec::new();
    }
    let num_probes = blade_deflections.len();
    let max_nd = num_blades / 2;
    let rev_freq = rpm / 60.0;

    // For each nodal diameter, compute the complex spatial DFT coefficient
    // averaged over all probes.
    let mut results = Vec::new();

    for nd in 1..=max_nd {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;

        for probe in 0..num_probes {
            let defl = &blade_deflections[probe];
            let mut re = 0.0;
            let mut im = 0.0;
            for (k, &d) in defl.iter().enumerate() {
                let angle = 2.0 * PI * nd as f64 * k as f64 / num_blades as f64;
                re += d * angle.cos();
                im += d * angle.sin();
            }
            re /= num_blades as f64;
            im /= num_blades as f64;
            sum_re += re;
            sum_im += im;
        }
        sum_re /= num_probes as f64;
        sum_im /= num_probes as f64;

        let amplitude = 2.0 * (sum_re * sum_re + sum_im * sum_im).sqrt();
        let phase = sum_im.atan2(sum_re);

        // Direction: positive phase progression → forward wave
        let direction = if phase >= 0.0 {
            WaveDirection::Forward
        } else {
            WaveDirection::Backward
        };

        // Engine order = nodal_diameter (for a rotor-fixed frame, the
        // observed frequency is ND * rev_freq ± excitation).
        let engine_order = nd as f64;

        if amplitude > 1e-9 {
            results.push(TravelingWave {
                engine_order: engine_order * rev_freq / rev_freq, // = nd
                amplitude_mm: amplitude,
                nodal_diameter: nd,
                direction,
            });
        }
    }

    // Sort by descending amplitude
    results.sort_by(|a, b| b.amplitude_mm.partial_cmp(&a.amplitude_mm).unwrap());
    results
}

// ───────────────────────────────────────────────────────────────────────────
// Synthetic data generation
// ───────────────────────────────────────────────────────────────────────────

/// Generate synthetic BTT arrival-time data with a known sinusoidal
/// vibration pattern.
///
/// Returns `arrival_times[probe][blade]` in seconds.
///
/// The vibration is modeled as a circumferential deflection of amplitude
/// `vibration_amplitude_mm` at the given `engine_order`.  Each blade's
/// measured arrival time is shifted from its expected value by the
/// deflection converted back to a time offset.
pub fn generate_synthetic_btt_data(
    config: &BttConfig,
    vibration_amplitude_mm: f64,
    engine_order: f64,
) -> Vec<Vec<f64>> {
    let tip_radius_m = 0.3; // 300 mm typical compressor blade
    let omega = 2.0 * PI * config.rpm / 60.0;
    let tip_speed = omega * tip_radius_m; // m/s

    let mut arrivals = Vec::with_capacity(config.num_probes);
    for probe_idx in 0..config.num_probes {
        let probe_angle = deg2rad(config.probe_angles_deg[probe_idx]);
        let mut blade_arrivals = Vec::with_capacity(config.num_blades);

        for blade_idx in 0..config.num_blades {
            let expected = expected_arrival_full(
                blade_idx,
                config.probe_angles_deg[probe_idx],
                config.rpm,
                config.num_blades,
            );

            // Vibration deflection for this blade at this probe angle
            let blade_angle = 2.0 * PI * blade_idx as f64 / config.num_blades as f64;
            let deflection_mm =
                vibration_amplitude_mm * (engine_order * (probe_angle + blade_angle)).sin();

            // Convert deflection back to time offset
            let dt = (deflection_mm / 1000.0) / tip_speed;

            blade_arrivals.push(expected + dt);
        }
        arrivals.push(blade_arrivals);
    }
    arrivals
}

// ───────────────────────────────────────────────────────────────────────────
// BttProcessor
// ───────────────────────────────────────────────────────────────────────────

/// Blade Tip Timing processor.
///
/// Accepts per-revolution arrival-time matrices and produces health reports.
#[derive(Debug, Clone)]
pub struct BttProcessor {
    config: BttConfig,
    revolution_count: usize,
    /// Running history of per-blade peak deflections (up to `history_depth`).
    deflection_history: Vec<Vec<f64>>,
    /// Maximum number of revolutions to retain.
    history_depth: usize,
    /// Deflection threshold (mm) for excessive-vibration alert.
    vibration_threshold_mm: f64,
    /// Tip radius in metres (used for deflection calculation).
    tip_radius_m: f64,
    /// Frequency shift threshold (fraction) for crack-suspected alert.
    freq_shift_threshold: f64,
    /// Previous dominant frequencies (Hz) per blade; used for shift detection.
    prev_frequencies: Vec<f64>,
}

impl BttProcessor {
    /// Create a new BTT processor with default thresholds.
    pub fn new(config: BttConfig) -> Self {
        let num_blades = config.num_blades;
        Self {
            config,
            revolution_count: 0,
            deflection_history: Vec::new(),
            history_depth: 64,
            vibration_threshold_mm: 1.0,
            tip_radius_m: 0.3,
            freq_shift_threshold: 0.05, // 5 % shift
            prev_frequencies: vec![0.0; num_blades],
        }
    }

    /// Set the vibration alert threshold (mm).
    pub fn set_vibration_threshold(&mut self, threshold_mm: f64) {
        self.vibration_threshold_mm = threshold_mm;
    }

    /// Set the tip radius (m) used for deflection calculation.
    pub fn set_tip_radius(&mut self, radius_m: f64) {
        self.tip_radius_m = radius_m;
    }

    /// Set the frequency-shift threshold (fraction, e.g. 0.05 = 5 %).
    pub fn set_freq_shift_threshold(&mut self, frac: f64) {
        self.freq_shift_threshold = frac;
    }

    /// Process one revolution of arrival-time data.
    ///
    /// `arrival_times[probe][blade]` contains measured arrival times in
    /// seconds for each probe and blade.
    pub fn process_revolution(&mut self, arrival_times: &[Vec<f64>]) -> BladeHealthReport {
        self.revolution_count += 1;

        let num_probes = self.config.num_probes;
        let num_blades = self.config.num_blades;

        assert_eq!(
            arrival_times.len(),
            num_probes,
            "arrival_times must have one entry per probe"
        );

        // ── Step 1: compute per-blade deflections at each probe ──────────
        // deflections[probe][blade] in mm
        let mut deflections: Vec<Vec<f64>> = Vec::with_capacity(num_probes);
        for probe_idx in 0..num_probes {
            assert_eq!(
                arrival_times[probe_idx].len(),
                num_blades,
                "each probe must have num_blades arrival times"
            );
            let mut probe_defl = Vec::with_capacity(num_blades);
            for blade_idx in 0..num_blades {
                let expected = expected_arrival_full(
                    blade_idx,
                    self.config.probe_angles_deg[probe_idx],
                    self.config.rpm,
                    num_blades,
                );
                let measured = arrival_times[probe_idx][blade_idx];
                let d = deflection_from_toa(expected, measured, self.tip_radius_m, self.config.rpm);
                probe_defl.push(d);
            }
            deflections.push(probe_defl);
        }

        // ── Step 2: per-blade peak deflection (max across probes) ────────
        let mut peak_deflections = vec![0.0_f64; num_blades];
        for blade_idx in 0..num_blades {
            let mut max_abs = 0.0_f64;
            for probe_idx in 0..num_probes {
                let abs_d = deflections[probe_idx][blade_idx].abs();
                if abs_d > max_abs {
                    max_abs = abs_d;
                }
            }
            peak_deflections[blade_idx] = max_abs;
        }

        // ── Step 3: fit sinusoid per blade to estimate frequency ─────────
        let probe_angles_rad: Vec<f64> = self
            .config
            .probe_angles_deg
            .iter()
            .map(|&d| deg2rad(d))
            .collect();

        let mut blade_frequencies = vec![0.0_f64; num_blades];
        let rev_freq = self.config.rpm / 60.0;

        if num_probes >= 3 {
            for blade_idx in 0..num_blades {
                let blade_defl: Vec<f64> =
                    (0..num_probes).map(|p| deflections[p][blade_idx]).collect();
                let (_, engine_order, _) = fit_sinusoid_lsq(&probe_angles_rad, &blade_defl);
                blade_frequencies[blade_idx] = engine_order * rev_freq;
            }
        }

        // ── Step 4: generate alerts ──────────────────────────────────────
        let mut alerts = Vec::new();

        for blade_idx in 0..num_blades {
            let defl = peak_deflections[blade_idx];

            // Excessive vibration
            if defl > self.vibration_threshold_mm {
                let severity = (defl / self.vibration_threshold_mm).min(1.0);
                alerts.push(BladeAlert {
                    blade_index: blade_idx,
                    alert_type: AlertType::ExcessiveVibration,
                    severity,
                });
            }

            // Missing blade (arrival time is NaN or very large deflection)
            if defl.is_nan() || defl > 50.0 {
                alerts.push(BladeAlert {
                    blade_index: blade_idx,
                    alert_type: AlertType::MissingBlade,
                    severity: 1.0,
                });
            }

            // Frequency shift (requires previous revolution data)
            if self.revolution_count > 1 && blade_frequencies[blade_idx] > 0.0 {
                let prev_f = self.prev_frequencies[blade_idx];
                if prev_f > 0.0 {
                    let shift = (blade_frequencies[blade_idx] - prev_f).abs() / prev_f;
                    if shift > self.freq_shift_threshold {
                        let severity = (shift / self.freq_shift_threshold).min(1.0);
                        alerts.push(BladeAlert {
                            blade_index: blade_idx,
                            alert_type: AlertType::FrequencyShift,
                            severity,
                        });
                    }
                }
            }

            // FOD damage: sudden jump in deflection compared to history mean
            if self.deflection_history.len() >= 4 {
                let hist_mean: f64 = self
                    .deflection_history
                    .iter()
                    .map(|rev| rev[blade_idx])
                    .sum::<f64>()
                    / self.deflection_history.len() as f64;
                if hist_mean > 0.01 && defl > 3.0 * hist_mean {
                    let severity = ((defl / hist_mean) / 5.0).min(1.0);
                    alerts.push(BladeAlert {
                        blade_index: blade_idx,
                        alert_type: AlertType::FodDamage,
                        severity,
                    });
                }
            }

            // Crack suspected: monotonically increasing deflection over recent history
            if self.deflection_history.len() >= 4 {
                let recent: Vec<f64> = self
                    .deflection_history
                    .iter()
                    .rev()
                    .take(4)
                    .map(|rev| rev[blade_idx])
                    .collect();
                // Check if each successive revolution is larger
                let monotonic = recent.windows(2).all(|w| w[0] >= w[1]);
                let growth = if recent.last().unwrap().abs() > 1e-12 {
                    recent[0] / recent.last().unwrap()
                } else {
                    0.0
                };
                if monotonic && growth > 1.5 {
                    let severity = ((growth - 1.0) / 2.0).min(1.0);
                    alerts.push(BladeAlert {
                        blade_index: blade_idx,
                        alert_type: AlertType::CrackSuspected,
                        severity,
                    });
                }
            }
        }

        // ── Step 5: update history ───────────────────────────────────────
        self.deflection_history.push(peak_deflections.clone());
        if self.deflection_history.len() > self.history_depth {
            self.deflection_history.remove(0);
        }
        self.prev_frequencies = blade_frequencies.clone();

        BladeHealthReport {
            blade_deflections_mm: peak_deflections,
            blade_frequencies_hz: blade_frequencies,
            alerts,
            revolution_count: self.revolution_count,
        }
    }

    /// Return the cumulative revolution count.
    pub fn revolution_count(&self) -> usize {
        self.revolution_count
    }

    /// Return a reference to the stored configuration.
    pub fn config(&self) -> &BttConfig {
        &self.config
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Tests
// ───────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> BttConfig {
        BttConfig {
            num_blades: 60,
            num_probes: 4,
            rpm: 10000.0,
            probe_angles_deg: vec![0.0, 90.0, 180.0, 270.0],
            sample_rate_hz: 1_000_000.0,
        }
    }

    // ── expected_arrival_full ────────────────────────────────────────────

    #[test]
    fn test_expected_arrival_blade_zero_probe_zero() {
        // Blade 0 at probe 0 deg should arrive at t=0
        let t = expected_arrival_full(0, 0.0, 10000.0, 60);
        assert!((t - 0.0).abs() < 1e-12, "blade 0 at 0 deg should be t=0, got {t}");
    }

    #[test]
    fn test_expected_arrival_probe_180() {
        // Any blade at 180 deg should arrive half a revolution later
        let rev_period = 60.0 / 10000.0; // 0.006 s
        let t = expected_arrival_full(0, 180.0, 10000.0, 60);
        assert!(
            (t - rev_period / 2.0).abs() < 1e-12,
            "blade 0 at 180 deg: expected {}, got {t}",
            rev_period / 2.0
        );
    }

    #[test]
    fn test_expected_arrival_blade_offset() {
        // Blade 1 at probe 0: should arrive after one blade pitch
        let rev_period = 60.0 / 10000.0;
        let blade_pitch_time = rev_period / 60.0;
        let t = expected_arrival_full(1, 0.0, 10000.0, 60);
        assert!(
            (t - blade_pitch_time).abs() < 1e-12,
            "blade 1 at 0 deg: expected {blade_pitch_time}, got {t}"
        );
    }

    #[test]
    fn test_expected_arrival_wraps_around() {
        // Blade 59 at 0 deg, 60 blades → last blade pitch
        let rev_period = 60.0 / 10000.0;
        let expected = 59.0 / 60.0 * rev_period;
        let t = expected_arrival_full(59, 0.0, 10000.0, 60);
        assert!(
            (t - expected).abs() < 1e-12,
            "blade 59: expected {expected}, got {t}"
        );
    }

    #[test]
    fn test_expected_arrival_combined() {
        // Blade 10 at probe 90 deg
        let rev_period = 60.0 / 10000.0;
        let total_angle = 90.0 + 10.0 * (360.0 / 60.0); // 90 + 60 = 150 deg
        let expected = (total_angle / 360.0) * rev_period;
        let t = expected_arrival_full(10, 90.0, 10000.0, 60);
        assert!(
            (t - expected).abs() < 1e-12,
            "blade 10 at 90 deg: expected {expected}, got {t}"
        );
    }

    // ── compute_expected_arrival (simple version) ────────────────────────

    #[test]
    fn test_compute_expected_arrival_zero() {
        let t = compute_expected_arrival(0, 0.0, 10000.0);
        assert!((t - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_compute_expected_arrival_quarter() {
        let rev = 60.0 / 10000.0;
        let t = compute_expected_arrival(0, 90.0, 10000.0);
        assert!((t - rev / 4.0).abs() < 1e-12);
    }

    // ── deflection_from_toa ──────────────────────────────────────────────

    #[test]
    fn test_deflection_zero_when_on_time() {
        let d = deflection_from_toa(0.001, 0.001, 0.3, 10000.0);
        assert!(d.abs() < 1e-12, "zero timing error should give zero deflection");
    }

    #[test]
    fn test_deflection_positive_late() {
        // Late arrival → positive deflection (blade lagging)
        let d = deflection_from_toa(0.001, 0.001001, 0.3, 10000.0);
        assert!(d > 0.0, "late arrival should give positive deflection");
    }

    #[test]
    fn test_deflection_negative_early() {
        let d = deflection_from_toa(0.001, 0.000999, 0.3, 10000.0);
        assert!(d < 0.0, "early arrival should give negative deflection");
    }

    #[test]
    fn test_deflection_magnitude() {
        // tip_speed = 2π * 10000/60 * 0.3 ≈ 314.159 m/s
        // dt = 1e-6 s → deflection ≈ 314.159e-6 m = 0.3142 mm
        let d = deflection_from_toa(0.001, 0.001001, 0.3, 10000.0);
        let expected = 2.0 * PI * (10000.0 / 60.0) * 0.3 * 1e-6 * 1000.0;
        assert!(
            (d - expected).abs() < 1e-6,
            "deflection: expected {expected} mm, got {d} mm"
        );
    }

    #[test]
    fn test_deflection_scales_with_rpm() {
        let d1 = deflection_from_toa(0.0, 1e-6, 0.3, 10000.0);
        let d2 = deflection_from_toa(0.0, 1e-6, 0.3, 20000.0);
        assert!(
            (d2 / d1 - 2.0).abs() < 1e-6,
            "doubling RPM should double deflection"
        );
    }

    #[test]
    fn test_deflection_scales_with_radius() {
        let d1 = deflection_from_toa(0.0, 1e-6, 0.3, 10000.0);
        let d2 = deflection_from_toa(0.0, 1e-6, 0.6, 10000.0);
        assert!(
            (d2 / d1 - 2.0).abs() < 1e-6,
            "doubling radius should double deflection"
        );
    }

    // ── fit_sinusoid_lsq ─────────────────────────────────────────────────

    #[test]
    fn test_sinusoid_fit_pure_sine() {
        // Generate pure A*sin(3*theta) at 8 points
        let n_points = 8;
        let amplitude = 2.5;
        let eo = 3.0;
        let angles: Vec<f64> = (0..n_points)
            .map(|i| 2.0 * PI * i as f64 / n_points as f64)
            .collect();
        let deflections: Vec<f64> = angles.iter().map(|&a| amplitude * (eo * a).sin()).collect();

        let (a, n, _phi) = fit_sinusoid_lsq(&angles, &deflections);
        assert!(
            (a - amplitude).abs() < 0.1,
            "amplitude: expected {amplitude}, got {a}"
        );
        assert!(
            (n - eo).abs() < 0.5,
            "engine order: expected {eo}, got {n}"
        );
    }

    #[test]
    fn test_sinusoid_fit_with_offset() {
        // A*sin(2*theta) + C
        let angles: Vec<f64> = (0..8)
            .map(|i| 2.0 * PI * i as f64 / 8.0)
            .collect();
        let deflections: Vec<f64> = angles.iter().map(|&a| 1.5 * (2.0 * a).sin() + 3.0).collect();
        let (a, n, _) = fit_sinusoid_lsq(&angles, &deflections);
        assert!((a - 1.5).abs() < 0.1, "amplitude {a}");
        assert!((n - 2.0).abs() < 0.5, "engine order {n}");
    }

    #[test]
    fn test_sinusoid_fit_engine_order_1() {
        let angles: Vec<f64> = (0..6)
            .map(|i| 2.0 * PI * i as f64 / 6.0)
            .collect();
        let deflections: Vec<f64> = angles.iter().map(|&a| 4.0 * (a + 0.5).sin()).collect();
        let (a, n, _) = fit_sinusoid_lsq(&angles, &deflections);
        assert!((a - 4.0).abs() < 0.2, "amplitude {a}");
        assert!((n - 1.0).abs() < 0.5, "engine order {n}");
    }

    #[test]
    fn test_sinusoid_fit_minimum_points() {
        // Exactly 3 points
        let angles = vec![0.0, PI / 2.0, PI];
        let deflections = vec![0.0, 1.0, 0.0];
        let (a, _n, _phi) = fit_sinusoid_lsq(&angles, &deflections);
        assert!(a >= 0.0, "amplitude should be non-negative");
    }

    // ── traveling_wave_analysis ──────────────────────────────────────────

    #[test]
    fn test_traveling_wave_single_nd() {
        // Create deflection pattern with nodal diameter 3
        let num_blades = 24;
        let num_probes = 4;
        let nd = 3;
        let amp = 1.0;

        let mut defl = Vec::new();
        for _probe in 0..num_probes {
            let blade_defl: Vec<f64> = (0..num_blades)
                .map(|k| amp * (2.0 * PI * nd as f64 * k as f64 / num_blades as f64).cos())
                .collect();
            defl.push(blade_defl);
        }

        let waves = traveling_wave_analysis(&defl, 10000.0);
        assert!(!waves.is_empty(), "should detect at least one wave");
        // The dominant wave should be at nd=3
        assert_eq!(
            waves[0].nodal_diameter, nd,
            "dominant ND: expected {nd}, got {}",
            waves[0].nodal_diameter
        );
    }

    #[test]
    fn test_traveling_wave_amplitude() {
        let num_blades = 16;
        let nd = 2;
        let amp = 3.5;
        let defl = vec![
            (0..num_blades)
                .map(|k| amp * (2.0 * PI * nd as f64 * k as f64 / num_blades as f64).cos())
                .collect::<Vec<f64>>(),
        ];
        let waves = traveling_wave_analysis(&defl, 6000.0);
        assert!(!waves.is_empty());
        // Amplitude should be close to input
        assert!(
            (waves[0].amplitude_mm - amp).abs() < 0.5,
            "amplitude: expected ~{amp}, got {}",
            waves[0].amplitude_mm
        );
    }

    #[test]
    fn test_traveling_wave_empty() {
        let waves = traveling_wave_analysis(&[], 10000.0);
        assert!(waves.is_empty());
    }

    #[test]
    fn test_traveling_wave_zero_deflections() {
        let defl = vec![vec![0.0; 20]; 3];
        let waves = traveling_wave_analysis(&defl, 10000.0);
        assert!(waves.is_empty(), "zero deflections should produce no waves");
    }

    // ── synthetic data generation ────────────────────────────────────────

    #[test]
    fn test_synthetic_data_dimensions() {
        let config = default_config();
        let data = generate_synthetic_btt_data(&config, 0.5, 3.0);
        assert_eq!(data.len(), config.num_probes);
        for probe_data in &data {
            assert_eq!(probe_data.len(), config.num_blades);
        }
    }

    #[test]
    fn test_synthetic_data_zero_vibration() {
        let config = default_config();
        let data = generate_synthetic_btt_data(&config, 0.0, 1.0);
        // With zero vibration, measured == expected
        for probe_idx in 0..config.num_probes {
            for blade_idx in 0..config.num_blades {
                let expected = expected_arrival_full(
                    blade_idx,
                    config.probe_angles_deg[probe_idx],
                    config.rpm,
                    config.num_blades,
                );
                assert!(
                    (data[probe_idx][blade_idx] - expected).abs() < 1e-15,
                    "zero vibration: probe {probe_idx} blade {blade_idx}"
                );
            }
        }
    }

    #[test]
    fn test_synthetic_data_positive_times() {
        let config = default_config();
        let data = generate_synthetic_btt_data(&config, 1.0, 5.0);
        for probe_data in &data {
            for &t in probe_data {
                assert!(t.is_finite(), "arrival time must be finite");
            }
        }
    }

    // ── round-trip: synthetic → processor → deflection recovery ─────────

    #[test]
    fn test_roundtrip_deflection_recovery() {
        let config = default_config();
        let vib_amp = 0.5; // mm
        let data = generate_synthetic_btt_data(&config, vib_amp, 3.0);

        let mut proc = BttProcessor::new(config.clone());
        let report = proc.process_revolution(&data);

        // Peak deflection should be in the ballpark of the vibration amplitude
        let max_defl = report
            .blade_deflections_mm
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        assert!(
            max_defl > 0.01,
            "should detect non-zero deflection, got {max_defl}"
        );
        // Not too far from expected (within order of magnitude)
        assert!(
            max_defl < vib_amp * 10.0,
            "deflection {max_defl} is unreasonably large"
        );
    }

    // ── BttProcessor ─────────────────────────────────────────────────────

    #[test]
    fn test_processor_revolution_count() {
        let config = default_config();
        let mut proc = BttProcessor::new(config.clone());
        assert_eq!(proc.revolution_count(), 0);

        let data = generate_synthetic_btt_data(&config, 0.0, 1.0);
        proc.process_revolution(&data);
        assert_eq!(proc.revolution_count(), 1);

        proc.process_revolution(&data);
        assert_eq!(proc.revolution_count(), 2);
    }

    #[test]
    fn test_processor_no_alerts_for_quiet_rotor() {
        let config = default_config();
        let mut proc = BttProcessor::new(config.clone());
        proc.set_vibration_threshold(2.0);

        let data = generate_synthetic_btt_data(&config, 0.0, 1.0);
        let report = proc.process_revolution(&data);
        assert!(
            report.alerts.is_empty(),
            "quiet rotor should have no alerts, got {}",
            report.alerts.len()
        );
    }

    #[test]
    fn test_processor_excessive_vibration_alert() {
        let config = default_config();
        let mut proc = BttProcessor::new(config.clone());
        proc.set_vibration_threshold(0.01); // very low threshold

        let data = generate_synthetic_btt_data(&config, 0.5, 3.0);
        let report = proc.process_revolution(&data);

        let vib_alerts: Vec<_> = report
            .alerts
            .iter()
            .filter(|a| a.alert_type == AlertType::ExcessiveVibration)
            .collect();
        assert!(
            !vib_alerts.is_empty(),
            "should generate vibration alerts with low threshold"
        );
    }

    #[test]
    fn test_processor_report_dimensions() {
        let config = default_config();
        let mut proc = BttProcessor::new(config.clone());
        let data = generate_synthetic_btt_data(&config, 0.1, 2.0);
        let report = proc.process_revolution(&data);

        assert_eq!(report.blade_deflections_mm.len(), config.num_blades);
        assert_eq!(report.blade_frequencies_hz.len(), config.num_blades);
        assert_eq!(report.revolution_count, 1);
    }

    #[test]
    fn test_processor_config_accessor() {
        let config = default_config();
        let proc = BttProcessor::new(config.clone());
        assert_eq!(proc.config().num_blades, 60);
        assert_eq!(proc.config().num_probes, 4);
    }

    #[test]
    fn test_processor_fod_alert_after_sudden_increase() {
        // Run several quiet revolutions, then a big one
        let config = BttConfig {
            num_blades: 12,
            num_probes: 4,
            rpm: 8000.0,
            probe_angles_deg: vec![0.0, 90.0, 180.0, 270.0],
            sample_rate_hz: 1_000_000.0,
        };
        let mut proc = BttProcessor::new(config.clone());
        proc.set_vibration_threshold(100.0); // disable vibration alerts

        // 5 quiet revolutions
        let quiet = generate_synthetic_btt_data(&config, 0.05, 2.0);
        for _ in 0..5 {
            proc.process_revolution(&quiet);
        }

        // 1 loud revolution
        let loud = generate_synthetic_btt_data(&config, 2.0, 2.0);
        let report = proc.process_revolution(&loud);

        let fod_alerts: Vec<_> = report
            .alerts
            .iter()
            .filter(|a| a.alert_type == AlertType::FodDamage)
            .collect();
        assert!(
            !fod_alerts.is_empty(),
            "sudden deflection increase should trigger FOD alert"
        );
    }

    // ── edge cases ──────────────────────────────────────────────────────

    #[test]
    fn test_single_blade() {
        let config = BttConfig {
            num_blades: 1,
            num_probes: 4,
            rpm: 5000.0,
            probe_angles_deg: vec![0.0, 90.0, 180.0, 270.0],
            sample_rate_hz: 1_000_000.0,
        };
        let data = generate_synthetic_btt_data(&config, 0.1, 1.0);
        assert_eq!(data.len(), 4);
        assert_eq!(data[0].len(), 1);

        let mut proc = BttProcessor::new(config);
        let report = proc.process_revolution(&data);
        assert_eq!(report.blade_deflections_mm.len(), 1);
    }

    #[test]
    fn test_single_probe() {
        let config = BttConfig {
            num_blades: 10,
            num_probes: 1,
            rpm: 10000.0,
            probe_angles_deg: vec![45.0],
            sample_rate_hz: 1_000_000.0,
        };
        let data = generate_synthetic_btt_data(&config, 0.2, 2.0);
        assert_eq!(data.len(), 1);
        assert_eq!(data[0].len(), 10);
        // With only 1 probe, sinusoid fit won't run (needs >=3)
        let mut proc = BttProcessor::new(config);
        let report = proc.process_revolution(&data);
        assert_eq!(report.blade_frequencies_hz.len(), 10);
        // All frequencies should be 0 (not enough probes)
        assert!(report.blade_frequencies_hz.iter().all(|&f| f == 0.0));
    }

    #[test]
    fn test_high_rpm() {
        let config = BttConfig {
            num_blades: 80,
            num_probes: 4,
            rpm: 50000.0,
            probe_angles_deg: vec![0.0, 90.0, 180.0, 270.0],
            sample_rate_hz: 10_000_000.0,
        };
        let data = generate_synthetic_btt_data(&config, 0.3, 5.0);
        let mut proc = BttProcessor::new(config);
        let report = proc.process_revolution(&data);
        assert_eq!(report.blade_deflections_mm.len(), 80);
    }

    #[test]
    fn test_low_rpm() {
        let config = BttConfig {
            num_blades: 20,
            num_probes: 3,
            rpm: 100.0,
            probe_angles_deg: vec![0.0, 120.0, 240.0],
            sample_rate_hz: 100_000.0,
        };
        let data = generate_synthetic_btt_data(&config, 0.1, 1.0);
        let mut proc = BttProcessor::new(config);
        let report = proc.process_revolution(&data);
        assert_eq!(report.blade_deflections_mm.len(), 20);
    }

    // ── det3 helper ─────────────────────────────────────────────────────

    #[test]
    fn test_det3_identity() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        assert!((det3(&id) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_det3_singular() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert!(det3(&m).abs() < 1e-10, "this matrix is singular");
    }

    #[test]
    fn test_solve_3x3_simple() {
        // x + 0 + 0 = 5, 0 + y + 0 = 3, 0 + 0 + z = 7
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [5.0, 3.0, 7.0];
        let x = solve_3x3(&a, &b).unwrap();
        assert!((x[0] - 5.0).abs() < 1e-12);
        assert!((x[1] - 3.0).abs() < 1e-12);
        assert!((x[2] - 7.0).abs() < 1e-12);
    }

    #[test]
    fn test_solve_3x3_singular_returns_none() {
        let a = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let b = [1.0, 2.0, 3.0];
        assert!(solve_3x3(&a, &b).is_none());
    }

    // ── deg2rad / rad2deg ───────────────────────────────────────────────

    #[test]
    fn test_deg2rad() {
        assert!((deg2rad(180.0) - PI).abs() < 1e-12);
        assert!((deg2rad(90.0) - PI / 2.0).abs() < 1e-12);
        assert!((deg2rad(0.0)).abs() < 1e-12);
    }

    #[test]
    fn test_rad2deg() {
        assert!((rad2deg(PI) - 180.0).abs() < 1e-12);
        assert!((rad2deg(PI / 2.0) - 90.0).abs() < 1e-12);
    }

    // ── Alert type equality ─────────────────────────────────────────────

    #[test]
    fn test_alert_type_equality() {
        assert_eq!(AlertType::ExcessiveVibration, AlertType::ExcessiveVibration);
        assert_ne!(AlertType::ExcessiveVibration, AlertType::FodDamage);
        assert_ne!(AlertType::CrackSuspected, AlertType::MissingBlade);
    }

    // ── WaveDirection ───────────────────────────────────────────────────

    #[test]
    fn test_wave_direction_equality() {
        assert_eq!(WaveDirection::Forward, WaveDirection::Forward);
        assert_ne!(WaveDirection::Forward, WaveDirection::Backward);
    }

    // ── Multiple revolutions ────────────────────────────────────────────

    #[test]
    fn test_multiple_revolutions_accumulate() {
        let config = default_config();
        let mut proc = BttProcessor::new(config.clone());
        let data = generate_synthetic_btt_data(&config, 0.1, 2.0);

        for i in 1..=10 {
            let report = proc.process_revolution(&data);
            assert_eq!(report.revolution_count, i);
        }
        assert_eq!(proc.revolution_count(), 10);
    }
}
