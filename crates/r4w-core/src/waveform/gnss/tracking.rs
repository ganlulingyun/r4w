//! GNSS Signal Tracking (DLL + PLL/FLL)
//!
//! After acquisition provides coarse estimates of code phase and carrier
//! frequency, the tracking loops refine these estimates and maintain lock.
//!
//! ## DLL (Delay Lock Loop) - Code Tracking
//!
//! Uses Early/Prompt/Late correlators to track the code phase:
//!
//! ```text
//!       Early      Prompt      Late
//!        │           │           │
//!        ▼           ▼           ▼
//!    ┌───────┐  ┌───────┐  ┌───────┐
//!    │ E-0.5 │  │   P   │  │ E+0.5 │  (0.5 chip spacing)
//!    └───┬───┘  └───┬───┘  └───┬───┘
//!        │          │          │
//!        └──── DLL Discriminator ────→ Code NCO
//!              (|E| - |L|) / (|E| + |L|)
//! ```
//!
//! ## PLL (Phase Lock Loop) - Carrier Tracking
//!
//! Costas discriminator for data-insensitive carrier phase tracking:
//!
//! ```text
//! Costas: atan(Q_P / I_P) → Loop Filter → Carrier NCO
//! ```

use std::f64::consts::PI;

use super::types::TrackingState;

/// DLL/PLL tracking channel for a single GNSS satellite
#[derive(Debug, Clone)]
pub struct TrackingChannel {
    /// PRN being tracked
    prn: u8,
    /// Code length in chips
    code_length: usize,
    /// Sample rate
    sample_rate: f64,
    /// Chipping rate
    chipping_rate: f64,

    // Code tracking (DLL)
    /// Current code phase in chips
    code_phase: f64,
    /// Code frequency (chips/s) = chipping_rate + code_doppler
    code_freq: f64,
    /// DLL loop bandwidth in Hz
    dll_bandwidth: f64,
    /// Early-Late spacing in chips
    el_spacing: f64,
    /// DLL loop filter state
    dll_filter: LoopFilter2nd,

    // Carrier tracking (PLL/FLL)
    /// Current carrier phase in radians
    carrier_phase: f64,
    /// Current carrier frequency offset in Hz
    carrier_freq: f64,
    /// PLL loop bandwidth in Hz
    pll_bandwidth: f64,
    /// FLL loop bandwidth in Hz (for initial pull-in)
    fll_bandwidth: f64,
    /// PLL loop filter state
    pll_filter: LoopFilter3rd,
    /// Whether FLL-assisted PLL is active
    fll_assist: bool,

    // Correlator outputs (accumulated over 1ms)
    /// Early correlator (I, Q)
    early_iq: (f64, f64),
    /// Prompt correlator (I, Q)
    prompt_iq: (f64, f64),
    /// Late correlator (I, Q)
    late_iq: (f64, f64),

    // Lock detection
    /// C/N0 estimator buffer
    cn0_buffer: Vec<f64>,
    /// Carrier lock indicator
    carrier_lock: bool,
    /// Code lock indicator
    code_lock: bool,
    /// Bit synchronization achieved
    bit_sync: bool,
    /// Millisecond counter
    ms_count: u64,

    // Nav data extraction
    /// Prompt I accumulation for nav bit detection
    nav_bit_accumulator: f64,
    /// Samples in current nav bit
    nav_bit_count: u32,
    /// Detected navigation bits
    nav_bits: Vec<i8>,
    /// Previous prompt I sign (for bit edge detection)
    prev_prompt_sign: i8,
}

impl TrackingChannel {
    /// Create a new tracking channel
    ///
    /// # Arguments
    /// * `prn` - PRN number
    /// * `code_length` - Code length in chips (e.g., 1023 for GPS L1 C/A)
    /// * `sample_rate` - Sample rate in Hz
    /// * `chipping_rate` - Chipping rate in chips/s (e.g., 1.023e6 for GPS)
    /// * `initial_code_phase` - Code phase from acquisition (chips)
    /// * `initial_doppler` - Doppler from acquisition (Hz)
    pub fn new(
        prn: u8,
        code_length: usize,
        sample_rate: f64,
        chipping_rate: f64,
        initial_code_phase: f64,
        initial_doppler: f64,
    ) -> Self {
        // Carrier-aided code Doppler
        let code_doppler = initial_doppler * chipping_rate / 1_575_420_000.0;

        Self {
            prn,
            code_length,
            sample_rate,
            chipping_rate,
            code_phase: initial_code_phase,
            code_freq: chipping_rate + code_doppler,
            dll_bandwidth: 1.0, // 1 Hz DLL bandwidth
            el_spacing: 0.5,    // 0.5 chip E-L spacing
            dll_filter: LoopFilter2nd::new(1.0, 0.001), // 1 Hz BW, 1ms update
            carrier_phase: 0.0,
            carrier_freq: initial_doppler,
            pll_bandwidth: 15.0, // 15 Hz PLL bandwidth
            fll_bandwidth: 50.0, // 50 Hz FLL bandwidth for pull-in
            pll_filter: LoopFilter3rd::new(15.0, 0.001),
            fll_assist: true,
            early_iq: (0.0, 0.0),
            prompt_iq: (0.0, 0.0),
            late_iq: (0.0, 0.0),
            cn0_buffer: Vec::new(),
            carrier_lock: false,
            code_lock: false,
            bit_sync: false,
            ms_count: 0,
            nav_bit_accumulator: 0.0,
            nav_bit_count: 0,
            nav_bits: Vec::new(),
            prev_prompt_sign: 0,
        }
    }

    /// Set DLL bandwidth
    pub fn with_dll_bandwidth(mut self, bw_hz: f64) -> Self {
        self.dll_bandwidth = bw_hz;
        self.dll_filter = LoopFilter2nd::new(bw_hz, 0.001);
        self
    }

    /// Set PLL bandwidth
    pub fn with_pll_bandwidth(mut self, bw_hz: f64) -> Self {
        self.pll_bandwidth = bw_hz;
        self.pll_filter = LoopFilter3rd::new(bw_hz, 0.001);
        self
    }

    /// Process one code period (1 ms for GPS L1 C/A) of samples
    ///
    /// # Arguments
    /// * `samples` - Input I/Q samples for one code period
    /// * `code` - Local code replica (+1/-1 values, one code period)
    ///
    /// # Returns
    /// Updated tracking state
    pub fn process(&mut self, samples: &[crate::types::IQSample], code: &[i8]) -> TrackingState {
        let samples_per_chip = self.sample_rate / self.code_freq;

        // Reset correlators
        self.early_iq = (0.0, 0.0);
        self.prompt_iq = (0.0, 0.0);
        self.late_iq = (0.0, 0.0);

        // Correlate with Early, Prompt, Late replicas
        for (i, &sample) in samples.iter().enumerate() {
            let t = i as f64 / self.sample_rate;

            // Strip carrier
            let carrier = (-2.0 * PI * (self.carrier_freq * t + self.carrier_phase)).sin_cos();
            let stripped = rustfft::num_complex::Complex64::new(
                sample.re * carrier.1 - sample.im * carrier.0,
                sample.re * carrier.0 + sample.im * carrier.1,
            );

            // Current code phase for this sample
            let chip = self.code_phase + i as f64 / samples_per_chip;

            // Early, Prompt, Late code phase indices
            let early_idx = ((chip - self.el_spacing / 2.0).rem_euclid(self.code_length as f64)) as usize;
            let prompt_idx = (chip.rem_euclid(self.code_length as f64)) as usize;
            let late_idx = ((chip + self.el_spacing / 2.0).rem_euclid(self.code_length as f64)) as usize;

            let early_code = code[early_idx % self.code_length] as f64;
            let prompt_code = code[prompt_idx % self.code_length] as f64;
            let late_code = code[late_idx % self.code_length] as f64;

            // Accumulate correlations
            self.early_iq.0 += stripped.re * early_code;
            self.early_iq.1 += stripped.im * early_code;
            self.prompt_iq.0 += stripped.re * prompt_code;
            self.prompt_iq.1 += stripped.im * prompt_code;
            self.late_iq.0 += stripped.re * late_code;
            self.late_iq.1 += stripped.im * late_code;
        }

        // DLL discriminator: normalized early-minus-late power
        let early_power = (self.early_iq.0.powi(2) + self.early_iq.1.powi(2)).sqrt();
        let late_power = (self.late_iq.0.powi(2) + self.late_iq.1.powi(2)).sqrt();
        let dll_discriminator = if early_power + late_power > 0.0 {
            (early_power - late_power) / (early_power + late_power)
        } else {
            0.0
        };

        // PLL discriminator: Costas (data-insensitive)
        let pll_discriminator = if self.prompt_iq.0.abs() > 1e-10 {
            (self.prompt_iq.1).atan2(self.prompt_iq.0) / (2.0 * PI)
        } else {
            0.0
        };

        // FLL discriminator (frequency error from cross-product)
        // Simplified: use phase rate of change
        let fll_discriminator = pll_discriminator; // Use PLL output as FLL approximation

        // Update loop filters
        let code_correction = self.dll_filter.update(dll_discriminator);
        let carrier_correction = if self.fll_assist && self.ms_count < 100 {
            // FLL-assisted PLL for initial pull-in
            self.pll_filter.update(pll_discriminator) * 0.5
                + fll_discriminator * self.fll_bandwidth * 0.5
        } else {
            self.pll_filter.update(pll_discriminator)
        };

        // Update NCOs
        self.code_phase += code_correction * self.el_spacing;
        self.code_phase = self.code_phase.rem_euclid(self.code_length as f64);

        self.carrier_freq += carrier_correction;
        self.carrier_phase += self.carrier_freq / self.sample_rate;
        self.carrier_phase = self.carrier_phase.rem_euclid(1.0);

        // Carrier-aided code tracking
        let code_doppler = self.carrier_freq * self.chipping_rate / 1_575_420_000.0;
        self.code_freq = self.chipping_rate + code_doppler;

        // C/N0 estimation (Narrow-band to Wide-band power ratio)
        let prompt_power = self.prompt_iq.0.powi(2) + self.prompt_iq.1.powi(2);
        self.cn0_buffer.push(prompt_power);
        if self.cn0_buffer.len() > 20 {
            self.cn0_buffer.remove(0);
        }

        let cn0_dbhz = self.estimate_cn0();

        // Lock detection
        self.carrier_lock = cn0_dbhz > 25.0 && self.ms_count > 10;
        self.code_lock = cn0_dbhz > 20.0;

        // Nav bit extraction
        self.nav_bit_accumulator += self.prompt_iq.0;
        self.nav_bit_count += 1;

        // Check for bit edge (sign change in prompt I every 20ms for GPS)
        let current_sign = if self.prompt_iq.0 >= 0.0 { 1 } else { -1 };
        if self.prev_prompt_sign != 0 && current_sign != self.prev_prompt_sign {
            if !self.bit_sync && self.ms_count > 20 {
                self.bit_sync = true;
            }
        }
        self.prev_prompt_sign = current_sign;

        // Accumulate 20ms worth for nav bit (GPS L1 C/A)
        if self.nav_bit_count >= 20 {
            let bit = if self.nav_bit_accumulator >= 0.0 { 1 } else { -1 };
            self.nav_bits.push(bit);
            self.nav_bit_accumulator = 0.0;
            self.nav_bit_count = 0;
        }

        self.ms_count += 1;

        // Disable FLL assist after convergence
        if self.ms_count > 200 {
            self.fll_assist = false;
        }

        TrackingState {
            prn: self.prn,
            code_phase: self.code_phase,
            carrier_freq_hz: self.carrier_freq,
            carrier_phase_rad: self.carrier_phase * 2.0 * PI,
            prompt_i: self.prompt_iq.0,
            prompt_q: self.prompt_iq.1,
            cn0_dbhz,
            carrier_lock: self.carrier_lock,
            code_lock: self.code_lock,
            bit_sync: self.bit_sync,
            ms_count: self.ms_count,
        }
    }

    /// Estimate C/N0 in dB-Hz using the moment method
    fn estimate_cn0(&self) -> f64 {
        if self.cn0_buffer.len() < 2 {
            return 0.0;
        }

        let n = self.cn0_buffer.len() as f64;
        let mean = self.cn0_buffer.iter().sum::<f64>() / n;
        let variance = self.cn0_buffer.iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>() / (n - 1.0);

        if variance <= 0.0 || mean <= 0.0 {
            return 0.0;
        }

        // Beaulieu's C/N0 estimator
        let snr = mean.powi(2) / variance;
        let t_coh = 0.001; // 1ms coherent integration

        let cn0_linear = (1.0 / t_coh) * (snr - 1.0).max(0.01);
        10.0 * cn0_linear.log10()
    }

    /// Get extracted navigation bits
    pub fn nav_bits(&self) -> &[i8] {
        &self.nav_bits
    }

    /// Get the current tracking state
    pub fn state(&self) -> TrackingState {
        TrackingState {
            prn: self.prn,
            code_phase: self.code_phase,
            carrier_freq_hz: self.carrier_freq,
            carrier_phase_rad: self.carrier_phase * 2.0 * PI,
            prompt_i: self.prompt_iq.0,
            prompt_q: self.prompt_iq.1,
            cn0_dbhz: self.estimate_cn0(),
            carrier_lock: self.carrier_lock,
            code_lock: self.code_lock,
            bit_sync: self.bit_sync,
            ms_count: self.ms_count,
        }
    }
}

/// 2nd-order loop filter for DLL
#[derive(Debug, Clone)]
pub struct LoopFilter2nd {
    /// Proportional gain
    k1: f64,
    /// Integral gain
    k2: f64,
    /// Integrator state
    integrator: f64,
}

impl LoopFilter2nd {
    /// Create a 2nd-order loop filter
    ///
    /// # Arguments
    /// * `bandwidth` - Loop bandwidth in Hz
    /// * `update_period` - Filter update period in seconds
    pub fn new(bandwidth: f64, update_period: f64) -> Self {
        let omega_n = bandwidth * 8.0 / 3.0; // Natural frequency
        let zeta = 1.0 / 2.0_f64.sqrt(); // Damping ratio (Butterworth)

        Self {
            k1: 2.0 * zeta * omega_n * update_period,
            k2: omega_n.powi(2) * update_period.powi(2),
            integrator: 0.0,
        }
    }

    /// Update the filter with a new discriminator sample
    pub fn update(&mut self, discriminator: f64) -> f64 {
        self.integrator += self.k2 * discriminator;
        self.k1 * discriminator + self.integrator
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }
}

/// 3rd-order loop filter for PLL
#[derive(Debug, Clone)]
pub struct LoopFilter3rd {
    /// Gains
    k1: f64,
    k2: f64,
    k3: f64,
    /// Integrator states
    integrator1: f64,
    integrator2: f64,
}

impl LoopFilter3rd {
    /// Create a 3rd-order loop filter
    pub fn new(bandwidth: f64, update_period: f64) -> Self {
        let omega_n = bandwidth * 2.4; // Natural frequency
        let a3 = 1.1;
        let b3 = 2.4;

        Self {
            k1: b3 * omega_n * update_period,
            k2: a3 * omega_n.powi(2) * update_period.powi(2),
            k3: omega_n.powi(3) * update_period.powi(3),
            integrator1: 0.0,
            integrator2: 0.0,
        }
    }

    /// Update the filter
    pub fn update(&mut self, discriminator: f64) -> f64 {
        self.integrator1 += self.k2 * discriminator;
        self.integrator2 += self.k3 * discriminator;
        self.k1 * discriminator + self.integrator1 + self.integrator2
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.integrator1 = 0.0;
        self.integrator2 = 0.0;
    }
}

/// DLL S-curve for educational visualization
/// Shows the discriminator output as a function of code phase error
pub fn dll_s_curve(el_spacing: f64, num_points: usize) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let error = -1.5 + 3.0 * i as f64 / (num_points - 1) as f64;
        // Simplified triangular autocorrelation model
        let early_corr = (1.0 - (error - el_spacing / 2.0).abs()).max(0.0);
        let late_corr = (1.0 - (error + el_spacing / 2.0).abs()).max(0.0);
        let discriminator = if early_corr + late_corr > 0.0 {
            (early_corr - late_corr) / (early_corr + late_corr)
        } else {
            0.0
        };
        points.push((error, discriminator));
    }
    points
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loop_filter_2nd_converges() {
        let mut filter = LoopFilter2nd::new(1.0, 0.001);
        // Apply constant error, filter should accumulate and produce nonzero output
        let mut output = 0.0;
        for _ in 0..1000 {
            output = filter.update(0.1);
        }
        assert!(output > 0.0, "Filter should produce positive output for positive input");
        // The integrator should grow over time
        let first_output = {
            let mut f = LoopFilter2nd::new(1.0, 0.001);
            f.update(0.1)
        };
        assert!(output > first_output, "Filter output should grow over time due to integrator");
    }

    #[test]
    fn test_loop_filter_3rd_converges() {
        let mut filter = LoopFilter3rd::new(15.0, 0.001);
        let mut output = 0.0;
        for _ in 0..1000 {
            output = filter.update(0.1);
        }
        assert!(output > 0.0, "Filter should produce positive output for positive input");
    }

    #[test]
    fn test_dll_s_curve_shape() {
        let curve = dll_s_curve(0.5, 201);

        // S-curve should be zero at zero error
        let mid = curve.len() / 2;
        assert!((curve[mid].0).abs() < 0.02, "Mid-point should be near zero error");
        assert!((curve[mid].1).abs() < 0.1, "S-curve should be ~0 at zero error, got {}", curve[mid].1);

        // S-curve should be anti-symmetric (odd function)
        // The exact sign depends on the convention: (E-L)/(E+L)
        // For negative error: Early correlator is closer to peak → E > L → positive
        // For positive error: Late correlator is closer to peak → L > E → negative
        // But with our definition, error = actual - estimated:
        //   negative error means code is early, so prompt is before the true peak
        //   Early arm is even earlier (further from peak), Late arm is closer
        //   So |Late| > |Early| → discriminator < 0
        let neg_err_point = curve.iter().find(|(e, _)| *e > -0.4 && *e < -0.1);
        let pos_err_point = curve.iter().find(|(e, _)| *e > 0.1 && *e < 0.4);

        if let (Some((_, d_neg)), Some((_, d_pos))) = (neg_err_point, pos_err_point) {
            // The discriminator should have opposite signs for opposite errors
            assert!(d_neg * d_pos < 0.0,
                "S-curve should be anti-symmetric: neg_err disc={}, pos_err disc={}", d_neg, d_pos);
        }
    }
}
