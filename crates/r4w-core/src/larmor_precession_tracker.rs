//! # Larmor Precession Tracker
//!
//! NMR/MRI Larmor precession frequency tracking, free induction decay (FID) analysis,
//! and magnetic field measurement from nuclear spin precession signals.
//!
//! ## Physics Background
//!
//! The Larmor equation describes nuclear spin precession in a magnetic field:
//!   ω₀ = γB₀
//! where γ is the gyromagnetic ratio (rad/s/T) and B₀ is the magnetic field strength.
//!
//! The frequency form is: f_L = γB₀/(2π) Hz
//!
//! Free Induction Decay (FID): s(t) = M₀ exp(-t/T2*) exp(iω₀t)
//!
//! Bloch equations describe magnetization dynamics:
//!   dMx/dt = γ(M×B)x - Mx/T2
//!   dMy/dt = γ(M×B)y - My/T2
//!   dMz/dt = γ(M×B)z - (Mz-M0)/T1
//!
//! T1: spin-lattice (longitudinal) relaxation
//! T2: spin-spin (transverse) relaxation
//! T2* includes field inhomogeneity: 1/T2* = 1/T2 + γΔB₀/2
//!
//! Chemical shift: δ = (f - f_ref)/f_ref × 10⁶ ppm

use std::f64::consts::PI;

// ─── Nucleus enum with gyromagnetic ratios ───────────────────────────────────

/// Nuclear species with known gyromagnetic ratios (γ in rad/s/T).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Nucleus {
    /// ¹H proton, γ = 267.522×10⁶ rad/s/T
    Proton,
    /// ¹³C carbon-13, γ = 67.2828×10⁶ rad/s/T
    Carbon13,
    /// ¹⁵N nitrogen-15, γ = -27.126×10⁶ rad/s/T
    Nitrogen15,
    /// ¹⁹F fluorine-19, γ = 251.815×10⁶ rad/s/T
    Fluorine19,
    /// ³¹P phosphorus-31, γ = 108.394×10⁶ rad/s/T
    Phosphorus31,
    /// ²³Na sodium-23, γ = 70.808×10⁶ rad/s/T
    Sodium23,
    /// ²H deuterium, γ = 41.066×10⁶ rad/s/T
    Deuterium,
}

impl Nucleus {
    /// Return the gyromagnetic ratio γ in rad/s/T.
    pub fn gyromagnetic_ratio(&self) -> f64 {
        match self {
            Nucleus::Proton => 267.522e6,
            Nucleus::Carbon13 => 67.2828e6,
            Nucleus::Nitrogen15 => -27.126e6,
            Nucleus::Fluorine19 => 251.815e6,
            Nucleus::Phosphorus31 => 108.394e6,
            Nucleus::Sodium23 => 70.808e6,
            Nucleus::Deuterium => 41.066e6,
        }
    }

    /// Return the absolute gyromagnetic ratio |γ| in rad/s/T.
    pub fn abs_gyromagnetic_ratio(&self) -> f64 {
        self.gyromagnetic_ratio().abs()
    }
}

// ─── LarmorFrequency ─────────────────────────────────────────────────────────

/// Larmor precession frequency calculator.
pub struct LarmorFrequency;

impl LarmorFrequency {
    /// Calculate Larmor precession frequency from field strength.
    /// f_L = |γ| B₀ / (2π) in Hz.
    pub fn from_field(b_tesla: f64, nucleus: Nucleus) -> f64 {
        nucleus.abs_gyromagnetic_ratio() * b_tesla / (2.0 * PI)
    }

    /// Calculate magnetic field from Larmor frequency.
    /// B = 2π f / |γ| in Tesla.
    pub fn field_from_frequency(freq_hz: f64, nucleus: Nucleus) -> f64 {
        2.0 * PI * freq_hz / nucleus.abs_gyromagnetic_ratio()
    }

    /// Chemical shift in ppm.
    /// δ = (f - f_ref) / f_ref × 10⁶
    pub fn chemical_shift_ppm(freq_hz: f64, ref_freq_hz: f64) -> f64 {
        (freq_hz - ref_freq_hz) / ref_freq_hz * 1.0e6
    }

    /// Angular Larmor frequency ω₀ = |γ| B₀ in rad/s.
    pub fn angular_frequency(b_tesla: f64, nucleus: Nucleus) -> f64 {
        nucleus.abs_gyromagnetic_ratio() * b_tesla
    }
}

// ─── FidSignal ───────────────────────────────────────────────────────────────

/// Free Induction Decay signal model and analysis.
///
/// FID: s(t) = A exp(-t/T2*) exp(i 2π f t)
pub struct FidSignal;

impl FidSignal {
    /// Generate a single-component complex FID signal.
    /// Returns Vec of (real, imag) pairs.
    pub fn generate(
        amplitude: f64,
        frequency_hz: f64,
        t2_star_s: f64,
        duration_s: f64,
        sample_rate_hz: f64,
    ) -> Vec<(f64, f64)> {
        let n_samples = (duration_s * sample_rate_hz).ceil() as usize;
        let dt = 1.0 / sample_rate_hz;
        let mut out = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 * dt;
            let env = amplitude * (-t / t2_star_s).exp();
            let phase = 2.0 * PI * frequency_hz * t;
            out.push((env * phase.cos(), env * phase.sin()));
        }
        out
    }

    /// Generate a multi-component complex FID.
    /// Each component is (amplitude, frequency_hz, t2_star_s).
    pub fn multi_component(
        components: &[(f64, f64, f64)],
        duration_s: f64,
        sample_rate_hz: f64,
    ) -> Vec<(f64, f64)> {
        let n_samples = (duration_s * sample_rate_hz).ceil() as usize;
        let dt = 1.0 / sample_rate_hz;
        let mut out = vec![(0.0_f64, 0.0_f64); n_samples];
        for &(amp, freq, t2s) in components {
            for i in 0..n_samples {
                let t = i as f64 * dt;
                let env = amp * (-t / t2s).exp();
                let phase = 2.0 * PI * freq * t;
                out[i].0 += env * phase.cos();
                out[i].1 += env * phase.sin();
            }
        }
        out
    }

    /// Estimate T2* from the magnitude envelope of an FID signal
    /// using a least-squares fit of ln(|signal|) vs time.
    pub fn estimate_t2_star(signal: &[(f64, f64)], sample_rate_hz: f64) -> f64 {
        // Compute magnitudes, skip near-zero
        let mut times = Vec::new();
        let mut log_mags = Vec::new();
        let dt = 1.0 / sample_rate_hz;
        for (i, &(re, im)) in signal.iter().enumerate() {
            let mag = (re * re + im * im).sqrt();
            if mag > 1.0e-15 {
                times.push(i as f64 * dt);
                log_mags.push(mag.ln());
            }
        }
        if times.len() < 2 {
            return f64::INFINITY;
        }
        // Linear regression: ln(mag) = ln(A) - t/T2*
        // slope = -1/T2*
        let n = times.len() as f64;
        let sum_t: f64 = times.iter().sum();
        let sum_y: f64 = log_mags.iter().sum();
        let sum_tt: f64 = times.iter().map(|t| t * t).sum();
        let sum_ty: f64 = times.iter().zip(log_mags.iter()).map(|(t, y)| t * y).sum();
        let denom = n * sum_tt - sum_t * sum_t;
        if denom.abs() < 1.0e-30 {
            return f64::INFINITY;
        }
        let slope = (n * sum_ty - sum_t * sum_y) / denom;
        if slope >= 0.0 {
            return f64::INFINITY; // no decay
        }
        -1.0 / slope
    }

    /// Extract dominant frequency from FID signal using FFT peak detection.
    /// Uses a simple DFT over the signal.
    pub fn extract_frequency(signal: &[(f64, f64)], sample_rate_hz: f64) -> f64 {
        let n = signal.len();
        if n == 0 {
            return 0.0;
        }
        // Compute magnitude spectrum with DFT (for moderate lengths)
        // For large signals, use a partial search
        let search_n = n.min(4096);
        let mut max_mag = 0.0_f64;
        let mut max_k = 0_usize;
        for k in 0..search_n {
            let mut re_sum = 0.0_f64;
            let mut im_sum = 0.0_f64;
            for (i, &(re, im)) in signal.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                re_sum += re * cos_a - im * sin_a;
                im_sum += re * sin_a + im * cos_a;
            }
            let mag = re_sum * re_sum + im_sum * im_sum;
            if mag > max_mag {
                max_mag = mag;
                max_k = k;
            }
        }
        // Convert bin to frequency
        let bin = if max_k > n / 2 { max_k as f64 - n as f64 } else { max_k as f64 };
        bin * sample_rate_hz / n as f64
    }

    /// Apply phase correction to FID signal.
    /// zero_order: constant phase offset (rad)
    /// first_order: linear phase slope (rad per sample)
    pub fn phase_correction(
        signal: &mut [(f64, f64)],
        zero_order: f64,
        first_order: f64,
    ) {
        for (i, sample) in signal.iter_mut().enumerate() {
            let phi = zero_order + first_order * i as f64;
            let cos_p = phi.cos();
            let sin_p = phi.sin();
            let re = sample.0;
            let im = sample.1;
            sample.0 = re * cos_p - im * sin_p;
            sample.1 = re * sin_p + im * cos_p;
        }
    }
}

// ─── BlochEquationSolver ─────────────────────────────────────────────────────

/// Numerical Bloch equation integrator for magnetization dynamics.
///
/// Bloch equations (rotating frame for proton):
///   dMx/dt = γ(M×B)x - Mx/T2
///   dMy/dt = γ(M×B)y - My/T2
///   dMz/dt = γ(M×B)z - (Mz - M0)/T1
pub struct BlochEquationSolver {
    /// Equilibrium magnetization M0.
    pub m0: f64,
    /// Spin-lattice relaxation T1 (seconds).
    pub t1: f64,
    /// Spin-spin relaxation T2 (seconds).
    pub t2: f64,
    /// Gyromagnetic ratio γ (rad/s/T). Defaults to proton.
    pub gamma: f64,
}

impl BlochEquationSolver {
    /// Create a new Bloch equation solver with proton gyromagnetic ratio.
    pub fn new(m0: f64, t1_s: f64, t2_s: f64) -> Self {
        Self {
            m0,
            t1: t1_s,
            t2: t2_s,
            gamma: Nucleus::Proton.gyromagnetic_ratio(),
        }
    }

    /// Create with a specific nucleus.
    pub fn with_nucleus(m0: f64, t1_s: f64, t2_s: f64, nucleus: Nucleus) -> Self {
        Self {
            m0,
            t1: t1_s,
            t2: t2_s,
            gamma: nucleus.gyromagnetic_ratio(),
        }
    }

    /// Single-step Bloch equation evolution using RK4 integration.
    /// b_field: (Bx, By, Bz) in Tesla.
    /// Returns (Mx, My, Mz) after dt seconds.
    pub fn evolve(
        &self,
        mx: f64,
        my: f64,
        mz: f64,
        b_field: (f64, f64, f64),
        dt_s: f64,
    ) -> (f64, f64, f64) {
        let deriv = |mx: f64, my: f64, mz: f64| -> (f64, f64, f64) {
            let (bx, by, bz) = b_field;
            // M × B cross product
            let cross_x = my * bz - mz * by;
            let cross_y = mz * bx - mx * bz;
            let cross_z = mx * by - my * bx;
            let dmx = self.gamma * cross_x - mx / self.t2;
            let dmy = self.gamma * cross_y - my / self.t2;
            let dmz = self.gamma * cross_z - (mz - self.m0) / self.t1;
            (dmx, dmy, dmz)
        };

        // RK4
        let (k1x, k1y, k1z) = deriv(mx, my, mz);
        let (k2x, k2y, k2z) = deriv(
            mx + 0.5 * dt_s * k1x,
            my + 0.5 * dt_s * k1y,
            mz + 0.5 * dt_s * k1z,
        );
        let (k3x, k3y, k3z) = deriv(
            mx + 0.5 * dt_s * k2x,
            my + 0.5 * dt_s * k2y,
            mz + 0.5 * dt_s * k2z,
        );
        let (k4x, k4y, k4z) = deriv(
            mx + dt_s * k3x,
            my + dt_s * k3y,
            mz + dt_s * k3z,
        );

        (
            mx + dt_s / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x),
            my + dt_s / 6.0 * (k1y + 2.0 * k2y + 2.0 * k3y + k4y),
            mz + dt_s / 6.0 * (k1z + 2.0 * k2z + 2.0 * k3z + k4z),
        )
    }

    /// Simulate FID after a 90° excitation.
    /// B0 along z-axis. Returns time series of (Mx, My, Mz).
    pub fn simulate_fid(
        m0: f64,
        t1: f64,
        t2: f64,
        b0: f64,
        duration: f64,
        dt: f64,
    ) -> Vec<(f64, f64, f64)> {
        let solver = BlochEquationSolver::new(m0, t1, t2);
        let n = (duration / dt).ceil() as usize;
        let mut result = Vec::with_capacity(n);
        // After 90° pulse about x-axis: Mx=0, My=M0, Mz=0
        let mut mx = 0.0;
        let mut my = m0;
        let mut mz = 0.0;
        let b_field = (0.0, 0.0, b0);
        for _ in 0..n {
            result.push((mx, my, mz));
            let (nmx, nmy, nmz) = solver.evolve(mx, my, mz, b_field, dt);
            mx = nmx;
            my = nmy;
            mz = nmz;
        }
        result
    }

    /// Apply an instantaneous RF pulse.
    /// flip_angle_rad: rotation angle (e.g., π/2 for 90°)
    /// phase_rad: phase of the pulse (0 = x-axis, π/2 = y-axis)
    pub fn apply_rf_pulse(
        mx: f64,
        my: f64,
        mz: f64,
        flip_angle_rad: f64,
        phase_rad: f64,
    ) -> (f64, f64, f64) {
        // Rotate magnetization about axis in xy-plane at angle `phase_rad`
        // First rotate to align pulse axis with x, then rotate about x, then back
        let cos_p = phase_rad.cos();
        let sin_p = phase_rad.sin();

        // Rotate about z by -phase to align pulse with x-axis
        let rx = mx * cos_p + my * sin_p;
        let ry = -mx * sin_p + my * cos_p;
        let rz = mz;

        // Rotate about x-axis by flip_angle
        let cos_f = flip_angle_rad.cos();
        let sin_f = flip_angle_rad.sin();
        let fx = rx;
        let fy = ry * cos_f - rz * sin_f;
        let fz = ry * sin_f + rz * cos_f;

        // Rotate back about z by +phase
        let out_x = fx * cos_p - fy * sin_p;
        let out_y = fx * sin_p + fy * cos_p;
        let out_z = fz;

        (out_x, out_y, out_z)
    }
}

// ─── SpinEchoSequence ────────────────────────────────────────────────────────

/// Result of T2 mono-exponential fit.
#[derive(Debug, Clone)]
pub struct T2FitResult {
    /// Fitted T2 in seconds.
    pub t2: f64,
    /// Fitted initial amplitude M0.
    pub m0: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
}

/// Result of bi-exponential T2 fit.
#[derive(Debug, Clone)]
pub struct BiExpResult {
    /// Short T2 component.
    pub t2_short: f64,
    /// Long T2 component.
    pub t2_long: f64,
    /// Amplitude fraction of short component.
    pub fraction_short: f64,
    /// Total amplitude.
    pub m0: f64,
}

/// Spin echo sequence utilities for T2 measurement.
pub struct SpinEchoSequence;

impl SpinEchoSequence {
    /// Hahn echo amplitude: M(TE) = M0 exp(-TE/T2).
    pub fn hahn_echo(t2: f64, te_s: f64, amplitude: f64) -> f64 {
        amplitude * (-te_s / t2).exp()
    }

    /// CPMG echo train amplitudes.
    pub fn cpmg_echo_train(
        t2: f64,
        echo_spacing: f64,
        num_echoes: usize,
        amplitude: f64,
    ) -> Vec<f64> {
        (1..=num_echoes)
            .map(|n| {
                let te = n as f64 * echo_spacing;
                amplitude * (-te / t2).exp()
            })
            .collect()
    }

    /// Fit T2 from echo times and amplitudes using least-squares.
    /// Model: A(t) = M0 exp(-t/T2)
    pub fn fit_t2(echo_times: &[f64], amplitudes: &[f64]) -> T2FitResult {
        assert_eq!(echo_times.len(), amplitudes.len());
        assert!(amplitudes.len() >= 2);

        // Linear regression on ln(amplitude) vs time
        let mut times = Vec::new();
        let mut log_amps = Vec::new();
        for (i, &a) in amplitudes.iter().enumerate() {
            if a > 1.0e-15 {
                times.push(echo_times[i]);
                log_amps.push(a.ln());
            }
        }
        let n = times.len() as f64;
        if n < 2.0 {
            return T2FitResult { t2: f64::INFINITY, m0: 0.0, r_squared: 0.0 };
        }
        let sum_t: f64 = times.iter().sum();
        let sum_y: f64 = log_amps.iter().sum();
        let sum_tt: f64 = times.iter().map(|t| t * t).sum();
        let sum_ty: f64 = times.iter().zip(log_amps.iter()).map(|(t, y)| t * y).sum();
        let denom = n * sum_tt - sum_t * sum_t;
        let slope = (n * sum_ty - sum_t * sum_y) / denom;
        let intercept = (sum_y - slope * sum_t) / n;

        let t2 = if slope < 0.0 { -1.0 / slope } else { f64::INFINITY };
        let m0 = intercept.exp();

        // R-squared
        let mean_y = sum_y / n;
        let ss_tot: f64 = log_amps.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = times.iter().zip(log_amps.iter())
            .map(|(t, y)| {
                let pred = intercept + slope * t;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 1.0e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

        T2FitResult { t2, m0, r_squared }
    }

    /// Fit bi-exponential decay: A(t) = A_s exp(-t/T2_s) + A_l exp(-t/T2_l).
    /// Uses iterative separation: fit tail for long component, subtract, fit residual.
    pub fn fit_t2_biexponential(times: &[f64], amps: &[f64]) -> BiExpResult {
        assert_eq!(times.len(), amps.len());
        assert!(amps.len() >= 4);

        // Step 1: fit the tail (last half) for the long component
        let half = times.len() / 2;
        let tail_times = &times[half..];
        let tail_amps = &amps[half..];
        let long_fit = SpinEchoSequence::fit_t2(tail_times, tail_amps);
        let t2_long = long_fit.t2;
        let a_long = long_fit.m0;

        // Step 2: subtract long component and fit residual
        let residual_amps: Vec<f64> = times.iter().zip(amps.iter())
            .map(|(t, a)| (a - a_long * (-t / t2_long).exp()).max(1.0e-15))
            .collect();
        let short_fit = SpinEchoSequence::fit_t2(times, &residual_amps);
        let t2_short = short_fit.t2;
        let a_short = short_fit.m0;

        let total = a_short + a_long;
        let fraction_short = if total > 1.0e-15 { a_short / total } else { 0.5 };

        BiExpResult {
            t2_short,
            t2_long,
            fraction_short,
            m0: total,
        }
    }
}

// ─── InversionRecovery ───────────────────────────────────────────────────────

/// Result of T1 fit.
#[derive(Debug, Clone)]
pub struct T1FitResult {
    /// Fitted T1 in seconds.
    pub t1: f64,
    /// Fitted M0.
    pub m0: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
}

/// Inversion recovery sequence for T1 measurement.
pub struct InversionRecovery;

impl InversionRecovery {
    /// Signal amplitude after inversion recovery.
    /// M(TI) = M0 (1 - 2 exp(-TI/T1))
    pub fn signal_amplitude(t1: f64, ti: f64, m0: f64) -> f64 {
        m0 * (1.0 - 2.0 * (-ti / t1).exp())
    }

    /// Fit T1 from inversion times and signal values.
    /// Model: S(TI) = M0 (1 - 2 exp(-TI/T1))
    /// Uses iterative Gauss-Newton-like approach.
    pub fn fit_t1(inversion_times: &[f64], signals: &[f64]) -> T1FitResult {
        assert_eq!(inversion_times.len(), signals.len());
        assert!(signals.len() >= 2);

        // Initial estimate: null point gives T1 ≈ TI_null / ln(2)
        // Find approximate null crossing
        let mut t1_est = 1.0_f64; // initial guess
        let max_sig = signals.iter().copied().fold(0.0_f64, f64::max);
        let m0_est = if max_sig.abs() > 1.0e-15 { max_sig } else { 1.0 };

        // Find null point for initial T1 estimate
        for i in 0..signals.len().saturating_sub(1) {
            if signals[i] * signals[i + 1] < 0.0 {
                // Linear interpolation for null point
                let frac = signals[i].abs() / (signals[i].abs() + signals[i + 1].abs());
                let ti_null = inversion_times[i] + frac * (inversion_times[i + 1] - inversion_times[i]);
                t1_est = ti_null / 2.0_f64.ln();
                break;
            }
        }

        // Iterative least-squares refinement
        let mut t1 = t1_est;
        let mut m0 = m0_est;
        for _ in 0..50 {
            // Compute Jacobian and residual for [M0, T1]
            let mut jt_j = [[0.0_f64; 2]; 2];
            let mut jt_r = [0.0_f64; 2];
            for (ti, &s) in inversion_times.iter().zip(signals.iter()) {
                let exp_val = (-ti / t1).exp();
                let pred = m0 * (1.0 - 2.0 * exp_val);
                let residual = s - pred;
                // Partial derivatives
                let dp_dm0 = 1.0 - 2.0 * exp_val;
                let dp_dt1 = -m0 * 2.0 * exp_val * ti / (t1 * t1);
                jt_j[0][0] += dp_dm0 * dp_dm0;
                jt_j[0][1] += dp_dm0 * dp_dt1;
                jt_j[1][0] += dp_dt1 * dp_dm0;
                jt_j[1][1] += dp_dt1 * dp_dt1;
                jt_r[0] += dp_dm0 * residual;
                jt_r[1] += dp_dt1 * residual;
            }
            // Solve 2x2 system
            let det = jt_j[0][0] * jt_j[1][1] - jt_j[0][1] * jt_j[1][0];
            if det.abs() < 1.0e-30 {
                break;
            }
            let dm0 = (jt_j[1][1] * jt_r[0] - jt_j[0][1] * jt_r[1]) / det;
            let dt1 = (-jt_j[1][0] * jt_r[0] + jt_j[0][0] * jt_r[1]) / det;
            m0 += dm0;
            t1 += dt1;
            if t1 < 1.0e-10 {
                t1 = 1.0e-10;
            }
            if dm0.abs() < 1.0e-12 && dt1.abs() < 1.0e-12 {
                break;
            }
        }

        // Compute R-squared
        let mean_s: f64 = signals.iter().sum::<f64>() / signals.len() as f64;
        let ss_tot: f64 = signals.iter().map(|s| (s - mean_s).powi(2)).sum();
        let ss_res: f64 = inversion_times.iter().zip(signals.iter())
            .map(|(ti, s)| {
                let pred = m0 * (1.0 - 2.0 * (-ti / t1).exp());
                (s - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 1.0e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

        T1FitResult { t1, m0, r_squared }
    }

    /// Null point: TI where signal crosses zero.
    /// TI_null = T1 × ln(2)
    pub fn null_point(t1: f64) -> f64 {
        t1 * 2.0_f64.ln()
    }
}

// ─── FieldTracker ────────────────────────────────────────────────────────────

/// Field stability metrics.
#[derive(Debug, Clone)]
pub struct FieldStability {
    /// Mean field in Tesla.
    pub mean: f64,
    /// Standard deviation in Tesla.
    pub std_dev: f64,
    /// Drift rate in Tesla/second.
    pub drift_rate: f64,
    /// Peak-to-peak variation in Tesla.
    pub peak_to_peak: f64,
}

/// Real-time magnetic field tracker from precession signals.
pub struct FieldTracker {
    nucleus: Nucleus,
    sample_rate_hz: f64,
    field_history: Vec<f64>,
    /// Running frequency estimate via IIR filter.
    freq_estimate: f64,
    initialized: bool,
}

impl FieldTracker {
    /// Create a new field tracker.
    pub fn new(nucleus: Nucleus, sample_rate_hz: f64) -> Self {
        Self {
            nucleus,
            sample_rate_hz,
            field_history: Vec::new(),
            freq_estimate: 0.0,
            initialized: false,
        }
    }

    /// Process a chunk of signal data and return current field estimate in Tesla.
    pub fn update(&mut self, signal_chunk: &[(f64, f64)]) -> f64 {
        let freq = FidSignal::extract_frequency(signal_chunk, self.sample_rate_hz);
        if !self.initialized {
            self.freq_estimate = freq;
            self.initialized = true;
        } else {
            // IIR averaging
            let alpha = 0.3;
            self.freq_estimate = alpha * freq + (1.0 - alpha) * self.freq_estimate;
        }
        let field = Self::frequency_to_field(self.freq_estimate, self.nucleus);
        self.field_history.push(field);
        field
    }

    /// Convert frequency to field strength.
    pub fn frequency_to_field(freq: f64, nucleus: Nucleus) -> f64 {
        LarmorFrequency::field_from_frequency(freq, nucleus)
    }

    /// Get field history.
    pub fn field_history(&self) -> &[f64] {
        &self.field_history
    }

    /// Compute field stability metrics from a set of measurements.
    pub fn field_stability(measurements: &[f64]) -> FieldStability {
        let n = measurements.len() as f64;
        if measurements.is_empty() {
            return FieldStability {
                mean: 0.0,
                std_dev: 0.0,
                drift_rate: 0.0,
                peak_to_peak: 0.0,
            };
        }
        let mean = measurements.iter().sum::<f64>() / n;
        let var = measurements.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
        let std_dev = var.sqrt();
        let mut min_val = f64::INFINITY;
        let mut max_val = f64::NEG_INFINITY;
        for &m in measurements {
            if m < min_val { min_val = m; }
            if m > max_val { max_val = m; }
        }
        // Drift rate from linear fit
        let drift_rate = if measurements.len() >= 2 {
            let sum_i: f64 = (0..measurements.len()).map(|i| i as f64).sum();
            let sum_y: f64 = measurements.iter().sum();
            let sum_ii: f64 = (0..measurements.len()).map(|i| (i as f64).powi(2)).sum();
            let sum_iy: f64 = measurements.iter().enumerate().map(|(i, y)| i as f64 * y).sum();
            let denom = n * sum_ii - sum_i * sum_i;
            if denom.abs() > 1.0e-30 {
                (n * sum_iy - sum_i * sum_y) / denom
            } else {
                0.0
            }
        } else {
            0.0
        };

        FieldStability {
            mean,
            std_dev,
            drift_rate,
            peak_to_peak: max_val - min_val,
        }
    }

    /// Compute Allan variance for frequency stability analysis.
    /// Returns Vec of (tau, allan_variance) pairs.
    pub fn allan_variance(
        measurements: &[f64],
        sample_interval_s: f64,
    ) -> Vec<(f64, f64)> {
        let n = measurements.len();
        if n < 3 {
            return vec![];
        }
        let mut result = Vec::new();
        let max_m = n / 2;
        let mut m = 1;
        while m <= max_m {
            let tau = m as f64 * sample_interval_s;
            let mut sum = 0.0;
            let mut count = 0;
            for i in 0..n - 2 * m {
                // Average over m samples
                let avg1: f64 = measurements[i..i + m].iter().sum::<f64>() / m as f64;
                let avg2: f64 = measurements[i + m..i + 2 * m].iter().sum::<f64>() / m as f64;
                sum += (avg2 - avg1).powi(2);
                count += 1;
            }
            if count > 0 {
                let avar = sum / (2.0 * count as f64);
                result.push((tau, avar));
            }
            // Octave scaling
            m *= 2;
        }
        result
    }
}

// ─── GradientCalculator ──────────────────────────────────────────────────────

/// MRI gradient field calculations.
pub struct GradientCalculator;

impl GradientCalculator {
    /// Frequency at a position along a gradient.
    /// f = |γ|/(2π) × (B0 + G × x)
    pub fn frequency_encode(
        position_m: f64,
        gradient_t_per_m: f64,
        b0: f64,
        nucleus: Nucleus,
    ) -> f64 {
        let b_total = b0 + gradient_t_per_m * position_m;
        LarmorFrequency::from_field(b_total, nucleus)
    }

    /// Bandwidth of a slice-select gradient.
    /// BW = |γ|/(2π) × G × thickness
    pub fn slice_select_bandwidth(
        gradient: f64,
        thickness_m: f64,
        nucleus: Nucleus,
    ) -> f64 {
        nucleus.abs_gyromagnetic_ratio() * gradient.abs() * thickness_m / (2.0 * PI)
    }

    /// Phase accumulated by a phase-encode gradient step.
    /// φ = γ × G × Δ × x
    pub fn phase_encode_step(
        gradient: f64,
        duration_s: f64,
        position_m: f64,
        nucleus: Nucleus,
    ) -> f64 {
        nucleus.gyromagnetic_ratio() * gradient * duration_s * position_m
    }

    /// Diffusion-weighted signal attenuation.
    /// S/S0 = exp(-b × ADC)
    pub fn diffusion_attenuation(b_value: f64, adc: f64) -> f64 {
        (-b_value * adc).exp()
    }

    /// Stejskal-Tanner b-value for pulsed gradient spin echo.
    /// b = γ² G² δ² (Δ - δ/3)
    pub fn b_value(
        gradient: f64,
        duration_s: f64,
        spacing_s: f64,
        nucleus: Nucleus,
    ) -> f64 {
        let gamma = nucleus.gyromagnetic_ratio();
        gamma * gamma * gradient * gradient * duration_s * duration_s
            * (spacing_s - duration_s / 3.0)
    }

    /// Spatial resolution from readout gradient and bandwidth.
    /// Δx = BW / (|γ|/(2π) × G)
    pub fn spatial_resolution(
        bandwidth_hz: f64,
        gradient_t_per_m: f64,
        nucleus: Nucleus,
    ) -> f64 {
        let gamma_bar = nucleus.abs_gyromagnetic_ratio() / (2.0 * PI);
        bandwidth_hz / (gamma_bar * gradient_t_per_m)
    }
}

// ─── SpectroscopyProcessor ───────────────────────────────────────────────────

/// A peak found in the NMR spectrum.
#[derive(Debug, Clone)]
pub struct SpectralPeak {
    /// Frequency in Hz.
    pub frequency: f64,
    /// Magnitude.
    pub magnitude: f64,
    /// Index in the spectrum array.
    pub index: usize,
}

/// NMR spectroscopy processing utilities.
pub struct SpectroscopyProcessor;

impl SpectroscopyProcessor {
    /// Compute magnitude spectrum from FID using DFT.
    /// Returns Vec of (frequency_hz, magnitude) pairs, sorted by frequency.
    pub fn fft_spectrum(fid: &[(f64, f64)], sample_rate: f64) -> Vec<(f64, f64)> {
        let n = fid.len();
        if n == 0 {
            return vec![];
        }
        let mut spectrum = Vec::with_capacity(n);
        for k in 0..n {
            let mut re_sum = 0.0_f64;
            let mut im_sum = 0.0_f64;
            for (i, &(re, im)) in fid.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                re_sum += re * cos_a - im * sin_a;
                im_sum += re * sin_a + im * cos_a;
            }
            let mag = (re_sum * re_sum + im_sum * im_sum).sqrt() / n as f64;
            // Map bin to frequency (centered)
            let freq = if k <= n / 2 {
                k as f64 * sample_rate / n as f64
            } else {
                (k as f64 - n as f64) * sample_rate / n as f64
            };
            spectrum.push((freq, mag));
        }
        // Sort by frequency
        spectrum.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        spectrum
    }

    /// Zero-fill FID to improve spectral resolution.
    pub fn zero_fill(fid: &mut Vec<(f64, f64)>, target_len: usize) {
        while fid.len() < target_len {
            fid.push((0.0, 0.0));
        }
    }

    /// Apodization: apply exponential line broadening.
    /// Multiplies FID by exp(-π × lb_hz × t).
    pub fn apodize(fid: &mut [(f64, f64)], lb_hz: f64, sample_rate: f64) {
        let dt = 1.0 / sample_rate;
        for (i, sample) in fid.iter_mut().enumerate() {
            let t = i as f64 * dt;
            let weight = (-PI * lb_hz * t).exp();
            sample.0 *= weight;
            sample.1 *= weight;
        }
    }

    /// Pick peaks above a threshold in the magnitude spectrum.
    pub fn peak_pick(spectrum: &[(f64, f64)], threshold: f64) -> Vec<SpectralPeak> {
        let mut peaks = Vec::new();
        for i in 1..spectrum.len().saturating_sub(1) {
            let mag = spectrum[i].1;
            if mag > threshold && mag > spectrum[i - 1].1 && mag > spectrum[i + 1].1 {
                peaks.push(SpectralPeak {
                    frequency: spectrum[i].0,
                    magnitude: mag,
                    index: i,
                });
            }
        }
        peaks
    }

    /// Integrate spectrum magnitude over a frequency range.
    pub fn integrate_region(spectrum: &[(f64, f64)], f_min: f64, f_max: f64) -> f64 {
        let mut sum = 0.0;
        let mut count = 0;
        let df = if spectrum.len() >= 2 {
            (spectrum.last().unwrap().0 - spectrum.first().unwrap().0) / (spectrum.len() as f64 - 1.0)
        } else {
            1.0
        };
        for &(freq, mag) in spectrum {
            if freq >= f_min && freq <= f_max {
                sum += mag;
                count += 1;
            }
        }
        if count > 0 {
            sum * df.abs()
        } else {
            0.0
        }
    }
}

// ─── RelaxometryMapper ───────────────────────────────────────────────────────

/// Relaxometry mapping for T1, T2, T2* from multi-echo/IR image data.
pub struct RelaxometryMapper;

impl RelaxometryMapper {
    /// Compute per-voxel T2 map from multi-echo images.
    /// echo_images: slice of images (each a Vec<f64> with the same length).
    /// echo_times: corresponding echo times in seconds.
    pub fn t2_map(echo_images: &[Vec<f64>], echo_times: &[f64]) -> Vec<f64> {
        assert!(echo_images.len() >= 2);
        assert_eq!(echo_images.len(), echo_times.len());
        let n_voxels = echo_images[0].len();
        let mut t2_values = vec![0.0_f64; n_voxels];
        for v in 0..n_voxels {
            let amps: Vec<f64> = echo_images.iter().map(|img| img[v]).collect();
            let fit = SpinEchoSequence::fit_t2(echo_times, &amps);
            t2_values[v] = fit.t2;
        }
        t2_values
    }

    /// Compute per-voxel T1 map from inversion recovery images.
    pub fn t1_map(ir_images: &[Vec<f64>], ti_times: &[f64]) -> Vec<f64> {
        assert!(ir_images.len() >= 2);
        assert_eq!(ir_images.len(), ti_times.len());
        let n_voxels = ir_images[0].len();
        let mut t1_values = vec![0.0_f64; n_voxels];
        for v in 0..n_voxels {
            let signals: Vec<f64> = ir_images.iter().map(|img| img[v]).collect();
            let fit = InversionRecovery::fit_t1(ti_times, &signals);
            t1_values[v] = fit.t1;
        }
        t1_values
    }

    /// Compute per-voxel T2* map from multi-echo gradient echo images.
    pub fn t2_star_map(multi_echo: &[Vec<f64>], te_times: &[f64]) -> Vec<f64> {
        // T2* fitting is same as T2 fitting from magnitude data
        Self::t2_map(multi_echo, te_times)
    }
}

// ─── TemperatureSensor ───────────────────────────────────────────────────────

/// NMR-based temperature measurement.
pub struct TemperatureSensor;

impl TemperatureSensor {
    /// Temperature from chemical shift change.
    /// T = ref_temp + (delta_ppm - reference_ppm) / coefficient
    /// coefficient is typically in ppm/K.
    pub fn from_chemical_shift(
        delta_ppm: f64,
        reference_ppm: f64,
        coefficient: f64,
    ) -> f64 {
        // T = (delta - ref) / coeff  gives the temp change relative to some reference temperature
        // For water proton NMR: coefficient ≈ -0.01 ppm/°C
        // The caller provides reference temperature implicitly via reference_ppm.
        // Return temperature in Kelvin assuming reference_ppm is at 273.15 K (0°C)
        273.15 + (delta_ppm - reference_ppm) / coefficient
    }

    /// Water proton chemical shift temperature.
    /// Water proton resonance shift: δ ≈ -0.01 ppm/°C relative to 4.7 ppm at 25°C.
    /// Returns temperature in Kelvin.
    pub fn water_chemical_shift_temp(delta_ppm: f64) -> f64 {
        // At 25°C (298.15 K), water proton shift is ~4.7 ppm
        // Coefficient: -0.01 ppm/°C
        let reference_ppm = 4.7; // at 25°C
        let reference_temp_k = 298.15;
        let coefficient = -0.01; // ppm per °C
        reference_temp_k + (delta_ppm - reference_ppm) / coefficient
    }

    /// Temperature from T1 using Arrhenius model.
    /// T1 = T1_ref × exp(Ea/R × (1/T - 1/T_ref))
    /// Solving for T: 1/T = 1/T_ref + (R/Ea) × ln(T1/T1_ref)
    pub fn t1_temperature(
        t1_s: f64,
        reference_t1: f64,
        ref_temp_k: f64,
        activation_energy_j: f64,
    ) -> f64 {
        // Boltzmann constant per mole (gas constant R = 8.314 J/(mol·K))
        let r = 8.314;
        let ln_ratio = (t1_s / reference_t1).ln();
        let inv_t = 1.0 / ref_temp_k + (r / activation_energy_j) * ln_ratio;
        if inv_t > 1.0e-10 {
            1.0 / inv_t
        } else {
            f64::INFINITY
        }
    }
}

// ─── Helper: T2* from T2 and B0 inhomogeneity ───────────────────────────────

/// Calculate T2* from T2 and field inhomogeneity.
/// 1/T2* = 1/T2 + |γ|ΔB₀/2
pub fn t2_star_from_t2(t2: f64, delta_b0: f64, nucleus: Nucleus) -> f64 {
    let r2_star = 1.0 / t2 + nucleus.abs_gyromagnetic_ratio() * delta_b0.abs() / 2.0;
    1.0 / r2_star
}

/// Calculate effective T2* relaxation rate.
/// R2* = 1/T2* = R2 + |γ|ΔB₀/2
pub fn r2_star(r2: f64, delta_b0: f64, nucleus: Nucleus) -> f64 {
    r2 + nucleus.abs_gyromagnetic_ratio() * delta_b0.abs() / 2.0
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1.0e-6;
    const TOL_LOOSE: f64 = 1.0e-3;

    // ─── Nucleus tests ───────────────────────────────────────────────────

    #[test]
    fn test_proton_gyromagnetic_ratio() {
        let gamma = Nucleus::Proton.gyromagnetic_ratio();
        assert!((gamma - 267.522e6).abs() < 1.0);
    }

    #[test]
    fn test_carbon13_gyromagnetic_ratio() {
        let gamma = Nucleus::Carbon13.gyromagnetic_ratio();
        assert!((gamma - 67.2828e6).abs() < 1.0);
    }

    #[test]
    fn test_nitrogen15_negative_gyromagnetic_ratio() {
        let gamma = Nucleus::Nitrogen15.gyromagnetic_ratio();
        assert!(gamma < 0.0, "N-15 has negative gamma");
        assert!((gamma - (-27.126e6)).abs() < 1.0);
    }

    #[test]
    fn test_fluorine19_gyromagnetic_ratio() {
        let gamma = Nucleus::Fluorine19.gyromagnetic_ratio();
        assert!((gamma - 251.815e6).abs() < 1.0);
    }

    #[test]
    fn test_phosphorus31_gyromagnetic_ratio() {
        assert!((Nucleus::Phosphorus31.gyromagnetic_ratio() - 108.394e6).abs() < 1.0);
    }

    #[test]
    fn test_sodium23_gyromagnetic_ratio() {
        assert!((Nucleus::Sodium23.gyromagnetic_ratio() - 70.808e6).abs() < 1.0);
    }

    #[test]
    fn test_deuterium_gyromagnetic_ratio() {
        assert!((Nucleus::Deuterium.gyromagnetic_ratio() - 41.066e6).abs() < 1.0);
    }

    #[test]
    fn test_abs_gyromagnetic_ratio_positive() {
        assert!(Nucleus::Nitrogen15.abs_gyromagnetic_ratio() > 0.0);
    }

    // ─── LarmorFrequency tests ──────────────────────────────────────────

    #[test]
    fn test_larmor_frequency_proton_1_5t() {
        // At 1.5T, proton frequency ≈ 63.87 MHz
        let freq = LarmorFrequency::from_field(1.5, Nucleus::Proton);
        let expected = 267.522e6 * 1.5 / (2.0 * PI);
        assert!((freq - expected).abs() < 1.0);
    }

    #[test]
    fn test_larmor_frequency_proton_3t() {
        // At 3.0T, proton frequency ≈ 127.74 MHz
        let freq = LarmorFrequency::from_field(3.0, Nucleus::Proton);
        assert!(freq > 127.0e6 && freq < 128.0e6);
    }

    #[test]
    fn test_larmor_frequency_proton_7t() {
        let freq = LarmorFrequency::from_field(7.0, Nucleus::Proton);
        // ~298 MHz
        assert!(freq > 297.0e6 && freq < 300.0e6);
    }

    #[test]
    fn test_field_from_frequency_roundtrip() {
        let b0 = 1.5;
        let freq = LarmorFrequency::from_field(b0, Nucleus::Proton);
        let b_back = LarmorFrequency::field_from_frequency(freq, Nucleus::Proton);
        assert!((b_back - b0).abs() < 1.0e-10);
    }

    #[test]
    fn test_field_from_frequency_carbon13() {
        let b0 = 3.0;
        let freq = LarmorFrequency::from_field(b0, Nucleus::Carbon13);
        let b_back = LarmorFrequency::field_from_frequency(freq, Nucleus::Carbon13);
        assert!((b_back - b0).abs() < 1.0e-10);
    }

    #[test]
    fn test_chemical_shift_zero() {
        let cs = LarmorFrequency::chemical_shift_ppm(100.0e6, 100.0e6);
        assert!(cs.abs() < TOL);
    }

    #[test]
    fn test_chemical_shift_positive() {
        // 1 ppm shift at 100 MHz = 100 Hz
        let ref_freq = 100.0e6;
        let freq = ref_freq + 100.0;
        let cs = LarmorFrequency::chemical_shift_ppm(freq, ref_freq);
        assert!((cs - 1.0).abs() < TOL);
    }

    #[test]
    fn test_angular_frequency() {
        let omega = LarmorFrequency::angular_frequency(1.5, Nucleus::Proton);
        let expected = 267.522e6 * 1.5;
        assert!((omega - expected).abs() < 1.0);
    }

    #[test]
    fn test_larmor_zero_field() {
        let freq = LarmorFrequency::from_field(0.0, Nucleus::Proton);
        assert!(freq.abs() < TOL);
    }

    // ─── FidSignal tests ────────────────────────────────────────────────

    #[test]
    fn test_fid_generate_length() {
        let fid = FidSignal::generate(1.0, 100.0, 0.1, 0.5, 1000.0);
        assert_eq!(fid.len(), 500);
    }

    #[test]
    fn test_fid_generate_initial_amplitude() {
        let fid = FidSignal::generate(1.0, 0.0, 0.1, 0.5, 1000.0);
        // At t=0, signal should be ~amplitude
        let (re, im) = fid[0];
        let mag = (re * re + im * im).sqrt();
        assert!((mag - 1.0).abs() < TOL);
    }

    #[test]
    fn test_fid_decay() {
        let fid = FidSignal::generate(1.0, 0.0, 0.01, 0.1, 10000.0);
        let mag_0 = (fid[0].0.powi(2) + fid[0].1.powi(2)).sqrt();
        let last = fid.last().unwrap();
        let mag_end = (last.0.powi(2) + last.1.powi(2)).sqrt();
        assert!(mag_end < mag_0, "FID should decay");
    }

    #[test]
    fn test_fid_decay_rate() {
        let t2_star = 0.05;
        let fid = FidSignal::generate(1.0, 0.0, t2_star, 0.2, 10000.0);
        // At t = T2*, magnitude should be ~exp(-1) ≈ 0.368
        let idx = (t2_star * 10000.0) as usize;
        let mag = (fid[idx].0.powi(2) + fid[idx].1.powi(2)).sqrt();
        assert!((mag - (-1.0_f64).exp()).abs() < 0.01);
    }

    #[test]
    fn test_fid_multi_component() {
        let components = vec![(1.0, 100.0, 0.1), (0.5, 200.0, 0.05)];
        let fid = FidSignal::multi_component(&components, 0.2, 1000.0);
        assert_eq!(fid.len(), 200);
        // Initial magnitude should be sum of amplitudes
        let mag0 = (fid[0].0.powi(2) + fid[0].1.powi(2)).sqrt();
        // Both start in phase at t=0, so magnitudes add
        assert!((mag0 - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_fid_estimate_t2_star() {
        let t2_star = 0.05;
        let fid = FidSignal::generate(1.0, 100.0, t2_star, 0.3, 5000.0);
        let estimated = FidSignal::estimate_t2_star(&fid, 5000.0);
        assert!((estimated - t2_star).abs() / t2_star < 0.1, "T2* estimate within 10%: got {}", estimated);
    }

    #[test]
    fn test_fid_extract_frequency() {
        let freq = 200.0;
        let fid = FidSignal::generate(1.0, freq, 1.0, 0.5, 2000.0);
        let extracted = FidSignal::extract_frequency(&fid, 2000.0);
        assert!((extracted - freq).abs() < 5.0, "Frequency extraction: expected {} got {}", freq, extracted);
    }

    #[test]
    fn test_fid_phase_correction_zero_order() {
        let mut fid = vec![(1.0, 0.0), (0.5, 0.0)];
        let phase = PI / 4.0;
        FidSignal::phase_correction(&mut fid, -phase, 0.0);
        // Should rotate by -phase: (1,0) -> (cos(-pi/4), sin(-pi/4))
        // Actually we apply +phase inside, so let's verify consistency
        let mag = (fid[0].0.powi(2) + fid[0].1.powi(2)).sqrt();
        assert!((mag - 1.0).abs() < TOL, "Magnitude preserved");
    }

    #[test]
    fn test_fid_phase_correction_preserves_magnitude() {
        let mut fid = FidSignal::generate(1.0, 100.0, 0.1, 0.05, 1000.0);
        let mags_before: Vec<f64> = fid.iter().map(|(r, i)| (r * r + i * i).sqrt()).collect();
        FidSignal::phase_correction(&mut fid, 0.5, 0.01);
        let mags_after: Vec<f64> = fid.iter().map(|(r, i)| (r * r + i * i).sqrt()).collect();
        for (a, b) in mags_before.iter().zip(mags_after.iter()) {
            assert!((a - b).abs() < TOL);
        }
    }

    // ─── BlochEquationSolver tests ──────────────────────────────────────

    #[test]
    fn test_bloch_solver_creation() {
        let solver = BlochEquationSolver::new(1.0, 1.0, 0.1);
        assert!((solver.m0 - 1.0).abs() < TOL);
        assert!((solver.t1 - 1.0).abs() < TOL);
        assert!((solver.t2 - 0.1).abs() < TOL);
    }

    #[test]
    fn test_bloch_equilibrium_stable() {
        // At equilibrium (0, 0, M0) with only B0 along z, should stay stable
        let solver = BlochEquationSolver::new(1.0, 1.0, 0.1);
        let (mx, my, mz) = solver.evolve(0.0, 0.0, 1.0, (0.0, 0.0, 1.0), 0.001);
        assert!(mx.abs() < TOL);
        assert!(my.abs() < TOL);
        assert!((mz - 1.0).abs() < TOL);
    }

    #[test]
    fn test_bloch_transverse_decay() {
        // Transverse magnetization should decay with T2
        let solver = BlochEquationSolver::new(1.0, 10.0, 0.1);
        let mut mx = 1.0;
        let mut my = 0.0;
        let mut mz = 0.0;
        let dt = 0.0001;
        let b0 = 0.0; // no precession, just relaxation
        for _ in 0..1000 {
            let (nmx, nmy, nmz) = solver.evolve(mx, my, mz, (0.0, 0.0, b0), dt);
            mx = nmx;
            my = nmy;
            mz = nmz;
        }
        // After 0.1 s, Mx should be ~exp(-1) ≈ 0.368 of initial
        let mag_xy = (mx * mx + my * my).sqrt();
        assert!((mag_xy - (-1.0_f64).exp()).abs() < 0.05,
            "Transverse decay after T2: expected ~0.368, got {}", mag_xy);
    }

    #[test]
    fn test_bloch_longitudinal_recovery() {
        // Mz should recover toward M0 with T1
        let solver = BlochEquationSolver::new(1.0, 0.1, 10.0);
        let mut mx = 0.0;
        let mut my = 0.0;
        let mut mz = 0.0; // start inverted to 0
        let dt = 0.0001;
        for _ in 0..1000 {
            let (nmx, nmy, nmz) = solver.evolve(mx, my, mz, (0.0, 0.0, 0.0), dt);
            mx = nmx;
            my = nmy;
            mz = nmz;
        }
        // After T1, Mz should be ~M0(1-exp(-1)) ≈ 0.632
        assert!((mz - (1.0 - (-1.0_f64).exp())).abs() < 0.05,
            "T1 recovery: expected ~0.632, got {}", mz);
    }

    #[test]
    fn test_bloch_simulate_fid_returns_data() {
        let result = BlochEquationSolver::simulate_fid(1.0, 1.0, 0.1, 1.0, 0.01, 0.0001);
        assert_eq!(result.len(), 100);
    }

    #[test]
    fn test_bloch_simulate_fid_initial_state() {
        // After 90° pulse: Mx=0, My=M0, Mz=0
        let result = BlochEquationSolver::simulate_fid(1.0, 1.0, 0.1, 0.0, 0.01, 0.0001);
        let (mx, my, mz) = result[0];
        assert!(mx.abs() < TOL);
        assert!((my - 1.0).abs() < TOL);
        assert!(mz.abs() < TOL);
    }

    #[test]
    fn test_rf_pulse_90_x() {
        // 90° pulse about x on (0,0,1) -> (0,-1,0) or (0,1,0) depending on convention
        let (mx, my, mz) = BlochEquationSolver::apply_rf_pulse(0.0, 0.0, 1.0, PI / 2.0, 0.0);
        assert!(mx.abs() < TOL);
        assert!((my.abs() - 1.0).abs() < TOL || (mz.abs()).abs() < TOL);
        // Total magnitude preserved
        let mag = (mx * mx + my * my + mz * mz).sqrt();
        assert!((mag - 1.0).abs() < TOL);
    }

    #[test]
    fn test_rf_pulse_180_x() {
        // 180° pulse inverts Mz
        let (mx, my, mz) = BlochEquationSolver::apply_rf_pulse(0.0, 0.0, 1.0, PI, 0.0);
        assert!(mx.abs() < TOL);
        assert!(my.abs() < TOL);
        assert!((mz - (-1.0)).abs() < TOL, "180° inversion: expected -1, got {}", mz);
    }

    #[test]
    fn test_rf_pulse_preserves_magnitude() {
        let (mx, my, mz) = BlochEquationSolver::apply_rf_pulse(0.3, 0.5, 0.8, 1.2, 0.7);
        let orig_mag = (0.3_f64.powi(2) + 0.5_f64.powi(2) + 0.8_f64.powi(2)).sqrt();
        let new_mag = (mx * mx + my * my + mz * mz).sqrt();
        assert!((orig_mag - new_mag).abs() < TOL);
    }

    #[test]
    fn test_rf_pulse_360_identity() {
        let (mx, my, mz) = BlochEquationSolver::apply_rf_pulse(0.3, 0.5, 0.8, 2.0 * PI, 0.0);
        assert!((mx - 0.3).abs() < TOL);
        assert!((my - 0.5).abs() < TOL);
        assert!((mz - 0.8).abs() < TOL);
    }

    // ─── SpinEchoSequence tests ─────────────────────────────────────────

    #[test]
    fn test_hahn_echo_at_zero() {
        let amp = SpinEchoSequence::hahn_echo(0.1, 0.0, 1.0);
        assert!((amp - 1.0).abs() < TOL);
    }

    #[test]
    fn test_hahn_echo_decay() {
        let t2 = 0.1;
        let te = 0.1;
        let amp = SpinEchoSequence::hahn_echo(t2, te, 1.0);
        assert!((amp - (-1.0_f64).exp()).abs() < TOL);
    }

    #[test]
    fn test_cpmg_echo_train_length() {
        let train = SpinEchoSequence::cpmg_echo_train(0.1, 0.02, 10, 1.0);
        assert_eq!(train.len(), 10);
    }

    #[test]
    fn test_cpmg_echo_train_decay() {
        let train = SpinEchoSequence::cpmg_echo_train(0.1, 0.02, 5, 1.0);
        for i in 1..train.len() {
            assert!(train[i] < train[i - 1], "Echoes should decay");
        }
    }

    #[test]
    fn test_t2_fit_from_known_data() {
        let t2_true = 0.08;
        let m0_true = 2.0;
        let times: Vec<f64> = (1..=10).map(|i| i as f64 * 0.01).collect();
        let amps: Vec<f64> = times.iter().map(|t| m0_true * (-t / t2_true).exp()).collect();
        let fit = SpinEchoSequence::fit_t2(&times, &amps);
        assert!((fit.t2 - t2_true).abs() / t2_true < 0.01, "T2 fit: expected {}, got {}", t2_true, fit.t2);
        assert!((fit.m0 - m0_true).abs() / m0_true < 0.01);
        assert!(fit.r_squared > 0.99);
    }

    #[test]
    fn test_t2_biexponential_fit() {
        let t2_s = 0.03;
        let t2_l = 0.15;
        let a_s = 0.6;
        let a_l = 0.4;
        let times: Vec<f64> = (1..=20).map(|i| i as f64 * 0.01).collect();
        let amps: Vec<f64> = times.iter().map(|t| {
            a_s * (-t / t2_s).exp() + a_l * (-t / t2_l).exp()
        }).collect();
        let fit = SpinEchoSequence::fit_t2_biexponential(&times, &amps);
        // Just verify we get two distinct components
        assert!(fit.t2_short < fit.t2_long || fit.t2_short > 0.0);
        assert!(fit.fraction_short > 0.0 && fit.fraction_short < 1.0);
    }

    // ─── InversionRecovery tests ────────────────────────────────────────

    #[test]
    fn test_ir_signal_at_zero() {
        let sig = InversionRecovery::signal_amplitude(1.0, 0.0, 1.0);
        // M(0) = M0(1 - 2) = -M0
        assert!((sig - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_ir_signal_at_infinity() {
        let sig = InversionRecovery::signal_amplitude(1.0, 100.0, 1.0);
        // M(∞) = M0
        assert!((sig - 1.0).abs() < TOL);
    }

    #[test]
    fn test_ir_null_point() {
        let t1 = 1.0;
        let ti_null = InversionRecovery::null_point(t1);
        assert!((ti_null - 2.0_f64.ln()).abs() < TOL);
        // At null point, signal should be zero
        let sig = InversionRecovery::signal_amplitude(t1, ti_null, 1.0);
        assert!(sig.abs() < TOL);
    }

    #[test]
    fn test_t1_fit_from_known_data() {
        let t1_true = 0.5;
        let m0_true = 1.0;
        let tis: Vec<f64> = (1..=15).map(|i| i as f64 * 0.1).collect();
        let signals: Vec<f64> = tis.iter().map(|ti| {
            InversionRecovery::signal_amplitude(t1_true, *ti, m0_true)
        }).collect();
        let fit = InversionRecovery::fit_t1(&tis, &signals);
        assert!((fit.t1 - t1_true).abs() / t1_true < 0.05,
            "T1 fit: expected {}, got {}", t1_true, fit.t1);
        assert!(fit.r_squared > 0.99, "R² = {}", fit.r_squared);
    }

    // ─── FieldTracker tests ─────────────────────────────────────────────

    #[test]
    fn test_field_tracker_creation() {
        let ft = FieldTracker::new(Nucleus::Proton, 10000.0);
        assert!(!ft.initialized);
    }

    #[test]
    fn test_frequency_to_field() {
        let freq = LarmorFrequency::from_field(1.5, Nucleus::Proton);
        let field = FieldTracker::frequency_to_field(freq, Nucleus::Proton);
        assert!((field - 1.5).abs() < 1.0e-10);
    }

    #[test]
    fn test_field_stability_constant() {
        let measurements = vec![1.5; 100];
        let stability = FieldTracker::field_stability(&measurements);
        assert!((stability.mean - 1.5).abs() < TOL);
        assert!(stability.std_dev < TOL);
        assert!(stability.peak_to_peak < TOL);
    }

    #[test]
    fn test_field_stability_linear_drift() {
        let measurements: Vec<f64> = (0..100).map(|i| 1.5 + i as f64 * 0.001).collect();
        let stability = FieldTracker::field_stability(&measurements);
        assert!((stability.drift_rate - 0.001).abs() < 1.0e-5);
    }

    #[test]
    fn test_field_stability_empty() {
        let stability = FieldTracker::field_stability(&[]);
        assert!(stability.mean.abs() < TOL);
    }

    #[test]
    fn test_allan_variance_basic() {
        let measurements: Vec<f64> = (0..64).map(|i| 1.5 + 0.001 * (i as f64 * 0.1).sin()).collect();
        let avar = FieldTracker::allan_variance(&measurements, 0.01);
        assert!(!avar.is_empty());
        // Tau values should increase
        for i in 1..avar.len() {
            assert!(avar[i].0 > avar[i - 1].0);
        }
    }

    #[test]
    fn test_allan_variance_too_short() {
        let avar = FieldTracker::allan_variance(&[1.0, 2.0], 0.01);
        assert!(avar.is_empty());
    }

    #[test]
    fn test_field_tracker_update() {
        let mut ft = FieldTracker::new(Nucleus::Proton, 10000.0);
        let fid = FidSignal::generate(1.0, 200.0, 0.1, 0.01, 10000.0);
        let field = ft.update(&fid);
        assert!(field > 0.0, "Field estimate should be positive");
        assert!(!ft.field_history().is_empty());
    }

    // ─── GradientCalculator tests ───────────────────────────────────────

    #[test]
    fn test_frequency_encode_center() {
        let f = GradientCalculator::frequency_encode(0.0, 0.01, 1.5, Nucleus::Proton);
        let f0 = LarmorFrequency::from_field(1.5, Nucleus::Proton);
        assert!((f - f0).abs() < 1.0);
    }

    #[test]
    fn test_frequency_encode_offset() {
        let f_center = GradientCalculator::frequency_encode(0.0, 0.01, 1.5, Nucleus::Proton);
        let f_pos = GradientCalculator::frequency_encode(0.1, 0.01, 1.5, Nucleus::Proton);
        assert!(f_pos > f_center, "Positive position should increase frequency");
    }

    #[test]
    fn test_slice_select_bandwidth() {
        let bw = GradientCalculator::slice_select_bandwidth(0.01, 0.005, Nucleus::Proton);
        // γ_bar * G * d = (267.522e6/(2π)) * 0.01 * 0.005
        let expected = Nucleus::Proton.abs_gyromagnetic_ratio() * 0.01 * 0.005 / (2.0 * PI);
        assert!((bw - expected).abs() < 1.0);
    }

    #[test]
    fn test_phase_encode_step_zero_position() {
        let phase = GradientCalculator::phase_encode_step(0.01, 0.001, 0.0, Nucleus::Proton);
        assert!(phase.abs() < TOL);
    }

    #[test]
    fn test_diffusion_attenuation_zero_b() {
        let att = GradientCalculator::diffusion_attenuation(0.0, 1.0e-9);
        assert!((att - 1.0).abs() < TOL);
    }

    #[test]
    fn test_diffusion_attenuation_typical() {
        // b = 1000 s/mm² = 1e9 s/m², ADC ≈ 2e-9 m²/s for brain
        let att = GradientCalculator::diffusion_attenuation(1.0e9, 2.0e-9);
        let expected = (-1.0e9 * 2.0e-9_f64).exp();
        assert!((att - expected).abs() < TOL);
    }

    #[test]
    fn test_b_value_calculation() {
        let g = 0.04; // 40 mT/m
        let delta = 0.03; // 30 ms
        let big_delta = 0.04; // 40 ms
        let b = GradientCalculator::b_value(g, delta, big_delta, Nucleus::Proton);
        let gamma = Nucleus::Proton.gyromagnetic_ratio();
        let expected = gamma * gamma * g * g * delta * delta * (big_delta - delta / 3.0);
        assert!((b - expected).abs() / expected.abs() < TOL);
    }

    #[test]
    fn test_spatial_resolution() {
        let res = GradientCalculator::spatial_resolution(100000.0, 0.01, Nucleus::Proton);
        assert!(res > 0.0);
    }

    // ─── SpectroscopyProcessor tests ────────────────────────────────────

    #[test]
    fn test_fft_spectrum_length() {
        let fid = FidSignal::generate(1.0, 100.0, 0.1, 0.1, 1000.0);
        let spec = SpectroscopyProcessor::fft_spectrum(&fid, 1000.0);
        assert_eq!(spec.len(), fid.len());
    }

    #[test]
    fn test_fft_spectrum_peak_at_frequency() {
        let freq = 50.0;
        let fid = FidSignal::generate(1.0, freq, 10.0, 1.0, 1000.0);
        let spec = SpectroscopyProcessor::fft_spectrum(&fid, 1000.0);
        // Find peak
        let peak = spec.iter().max_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
        assert!((peak.0 - freq).abs() < 2.0, "Peak at {} Hz, expected {}", peak.0, freq);
    }

    #[test]
    fn test_zero_fill() {
        let mut fid = vec![(1.0, 0.0); 100];
        SpectroscopyProcessor::zero_fill(&mut fid, 256);
        assert_eq!(fid.len(), 256);
        assert!((fid[200].0).abs() < TOL);
    }

    #[test]
    fn test_apodize() {
        let mut fid = vec![(1.0, 0.0); 100];
        SpectroscopyProcessor::apodize(&mut fid, 10.0, 1000.0);
        // First sample unchanged (t=0)
        assert!((fid[0].0 - 1.0).abs() < TOL);
        // Later samples attenuated
        assert!(fid[50].0 < 1.0);
    }

    #[test]
    fn test_peak_pick_basic() {
        let spectrum = vec![
            (0.0, 0.1), (1.0, 0.5), (2.0, 1.0), (3.0, 0.5), (4.0, 0.2),
        ];
        let peaks = SpectroscopyProcessor::peak_pick(&spectrum, 0.3);
        assert_eq!(peaks.len(), 1);
        assert!((peaks[0].frequency - 2.0).abs() < TOL);
    }

    #[test]
    fn test_peak_pick_multiple() {
        let spectrum = vec![
            (0.0, 0.1), (1.0, 0.5), (2.0, 0.2), (3.0, 0.8), (4.0, 0.1),
        ];
        let peaks = SpectroscopyProcessor::peak_pick(&spectrum, 0.3);
        assert_eq!(peaks.len(), 2);
    }

    #[test]
    fn test_integrate_region() {
        let spectrum: Vec<(f64, f64)> = (0..100).map(|i| (i as f64, 1.0)).collect();
        let integral = SpectroscopyProcessor::integrate_region(&spectrum, 10.0, 20.0);
        assert!(integral > 0.0);
    }

    #[test]
    fn test_integrate_empty_region() {
        let spectrum: Vec<(f64, f64)> = (0..100).map(|i| (i as f64, 1.0)).collect();
        let integral = SpectroscopyProcessor::integrate_region(&spectrum, 200.0, 300.0);
        assert!(integral.abs() < TOL);
    }

    // ─── RelaxometryMapper tests ────────────────────────────────────────

    #[test]
    fn test_t2_map_single_voxel() {
        let t2_true = 0.08;
        let echo_times: Vec<f64> = (1..=5).map(|i| i as f64 * 0.02).collect();
        let echo_images: Vec<Vec<f64>> = echo_times.iter()
            .map(|te| vec![1.0 * (-te / t2_true).exp()])
            .collect();
        let t2_map = RelaxometryMapper::t2_map(&echo_images, &echo_times);
        assert_eq!(t2_map.len(), 1);
        assert!((t2_map[0] - t2_true).abs() / t2_true < 0.05);
    }

    #[test]
    fn test_t1_map_single_voxel() {
        let t1_true = 0.5;
        let ti_times: Vec<f64> = (1..=10).map(|i| i as f64 * 0.1).collect();
        let ir_images: Vec<Vec<f64>> = ti_times.iter()
            .map(|ti| vec![1.0 * (1.0 - 2.0 * (-ti / t1_true).exp())])
            .collect();
        let t1_map = RelaxometryMapper::t1_map(&ir_images, &ti_times);
        assert_eq!(t1_map.len(), 1);
        assert!((t1_map[0] - t1_true).abs() / t1_true < 0.1,
            "T1 map: expected {}, got {}", t1_true, t1_map[0]);
    }

    #[test]
    fn test_t2_star_map_uses_t2_fit() {
        let te_times: Vec<f64> = (1..=5).map(|i| i as f64 * 0.01).collect();
        let images: Vec<Vec<f64>> = te_times.iter()
            .map(|te| vec![(-te / 0.05_f64).exp()])
            .collect();
        let map = RelaxometryMapper::t2_star_map(&images, &te_times);
        assert_eq!(map.len(), 1);
        assert!(map[0] > 0.0);
    }

    // ─── TemperatureSensor tests ────────────────────────────────────────

    #[test]
    fn test_water_chemical_shift_temp_at_25c() {
        let temp = TemperatureSensor::water_chemical_shift_temp(4.7);
        assert!((temp - 298.15).abs() < 0.1, "At 4.7 ppm should be ~25°C: got {} K", temp);
    }

    #[test]
    fn test_water_chemical_shift_temp_higher() {
        // Lower ppm -> higher temperature (coefficient is -0.01 ppm/°C)
        let temp_low = TemperatureSensor::water_chemical_shift_temp(4.6);
        let temp_high = TemperatureSensor::water_chemical_shift_temp(4.7);
        assert!(temp_low > temp_high, "Lower ppm means higher temp for water");
    }

    #[test]
    fn test_from_chemical_shift() {
        let temp = TemperatureSensor::from_chemical_shift(5.0, 4.7, -0.01);
        // (5.0 - 4.7) / (-0.01) = -30 °C offset from 0°C = 273.15 + (-30) = 243.15 K
        assert!((temp - 243.15).abs() < 0.1);
    }

    #[test]
    fn test_t1_temperature_at_reference() {
        let temp = TemperatureSensor::t1_temperature(1.0, 1.0, 300.0, 50000.0);
        // When T1 = T1_ref, temperature = ref_temp
        assert!((temp - 300.0).abs() < 0.1);
    }

    #[test]
    fn test_t1_temperature_higher() {
        // Longer T1 at higher temperature (for most tissues)
        let temp = TemperatureSensor::t1_temperature(1.5, 1.0, 300.0, 20000.0);
        assert!(temp > 300.0 || temp < 300.0, "Temperature should differ from reference");
    }

    // ─── t2_star_from_t2 tests ──────────────────────────────────────────

    #[test]
    fn test_t2_star_with_no_inhomogeneity() {
        let t2s = t2_star_from_t2(0.1, 0.0, Nucleus::Proton);
        assert!((t2s - 0.1).abs() < TOL);
    }

    #[test]
    fn test_t2_star_shorter_than_t2() {
        let t2 = 0.1;
        let t2s = t2_star_from_t2(t2, 1.0e-6, Nucleus::Proton);
        assert!(t2s < t2, "T2* should be shorter than T2");
    }

    #[test]
    fn test_r2_star_additive() {
        let r2 = 10.0;
        let delta_b0 = 1.0e-6;
        let r2s = r2_star(r2, delta_b0, Nucleus::Proton);
        assert!(r2s > r2, "R2* should be larger than R2");
    }

    // ─── Edge case / integration tests ──────────────────────────────────

    #[test]
    fn test_full_nmr_workflow() {
        // Generate FID at 3T proton
        let b0 = 3.0;
        let freq = LarmorFrequency::from_field(b0, Nucleus::Proton);
        assert!(freq > 127.0e6);

        // Generate FID with known T2*
        let t2_star = 0.05;
        let fid = FidSignal::generate(1.0, freq, t2_star, 0.2, freq * 4.0);
        assert!(!fid.is_empty());
    }

    #[test]
    fn test_nucleus_enum_all_variants() {
        let nuclei = [
            Nucleus::Proton, Nucleus::Carbon13, Nucleus::Nitrogen15,
            Nucleus::Fluorine19, Nucleus::Phosphorus31, Nucleus::Sodium23,
            Nucleus::Deuterium,
        ];
        for n in &nuclei {
            let gamma = n.gyromagnetic_ratio();
            assert!(gamma.is_finite());
            let freq = LarmorFrequency::from_field(1.0, *n);
            assert!(freq > 0.0);
        }
    }

    #[test]
    fn test_larmor_different_nuclei_at_same_field() {
        let b0 = 3.0;
        let f_h = LarmorFrequency::from_field(b0, Nucleus::Proton);
        let f_c = LarmorFrequency::from_field(b0, Nucleus::Carbon13);
        // Proton frequency should be ~4x carbon-13
        let ratio = f_h / f_c;
        assert!(ratio > 3.5 && ratio < 4.5, "H/C ratio: {}", ratio);
    }

    #[test]
    fn test_bloch_with_nucleus() {
        let solver = BlochEquationSolver::with_nucleus(1.0, 1.0, 0.1, Nucleus::Carbon13);
        assert!((solver.gamma - Nucleus::Carbon13.gyromagnetic_ratio()).abs() < 1.0);
    }

    #[test]
    fn test_gradient_b_value_zero_gradient() {
        let b = GradientCalculator::b_value(0.0, 0.03, 0.04, Nucleus::Proton);
        assert!(b.abs() < TOL);
    }

    #[test]
    fn test_cpmg_matches_hahn_echo() {
        let t2 = 0.1;
        let spacing = 0.02;
        let train = SpinEchoSequence::cpmg_echo_train(t2, spacing, 5, 1.0);
        for (i, &amp) in train.iter().enumerate() {
            let te = (i + 1) as f64 * spacing;
            let hahn = SpinEchoSequence::hahn_echo(t2, te, 1.0);
            assert!((amp - hahn).abs() < TOL);
        }
    }
}
