//! Capacitive Micromachined Ultrasonic Transducer (CMUT) signal processing for
//! ultrasound imaging and sensing.
//!
//! This module provides CMUT cell membrane mechanics (resonance frequency, collapse
//! voltage, static deflection), capacitance modelling with fringing field correction,
//! transmit/receive sensitivity, array beamforming (delay-and-sum with apodization),
//! pulse-echo processing (TGC, envelope detection, log compression, A-line, B-mode),
//! acoustic parameter calculations, resolution estimation, harmonic imaging (pulse
//! inversion, second harmonic extraction), and element characterisation (impedance,
//! bandwidth, crosstalk).
//!
//! # Example
//!
//! ```
//! use r4w_core::capacitive_micromachined_ultrasonic::{
//!     CmutCell, CmutArray, Apodization,
//!     acoustic_impedance, reflection_coefficient, axial_resolution,
//! };
//!
//! // Single CMUT cell
//! let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
//! let f0 = cell.resonance_frequency_hz();
//! assert!(f0 > 1.0e6); // MHz-range resonance
//!
//! // 64-element linear array
//! let array = CmutArray::new_uniform(64, 150.0, cell);
//! let delays = array.focus_delays(0.0, 30_000.0, 1540.0);
//! assert_eq!(delays.len(), 64);
//!
//! // Acoustic parameters
//! let z = acoustic_impedance(1000.0, 1540.0);
//! let r = reflection_coefficient(z, acoustic_impedance(1700.0, 3500.0));
//! assert!(r.abs() < 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Permittivity of free space (F/m).
const EPSILON_0: f64 = 8.854187817e-12;

/// Speed of sound in soft tissue (m/s).
pub const SOUND_SPEED_TISSUE: f64 = 1540.0;

/// Speed of sound in water at 20 deg C (m/s).
pub const SOUND_SPEED_WATER_20C: f64 = 1480.0;

/// Typical silicon nitride Young's modulus (Pa).
const SI3N4_YOUNGS_MODULUS: f64 = 250.0e9;

/// Typical silicon nitride Poisson's ratio.
const SI3N4_POISSON: f64 = 0.23;

/// Typical silicon nitride density (kg/m^3).
const SI3N4_DENSITY: f64 = 3100.0;

// ---------------------------------------------------------------------------
// CmutCell
// ---------------------------------------------------------------------------

/// A single CMUT cell with membrane and electrode geometry.
#[derive(Debug, Clone)]
pub struct CmutCell {
    /// Membrane radius (micrometres).
    pub radius_um: f64,
    /// Vacuum gap height (micrometres).
    pub gap_um: f64,
    /// Membrane thickness (micrometres).
    pub membrane_thickness_um: f64,
    /// Electrode area (square micrometres). If zero, uses full membrane area.
    pub electrode_area_um2: f64,
    /// DC bias voltage (V).
    pub dc_bias_v: f64,
    /// Young's modulus (Pa).
    pub youngs_modulus: f64,
    /// Poisson's ratio.
    pub poisson_ratio: f64,
    /// Membrane material density (kg/m^3).
    pub density: f64,
}

impl CmutCell {
    /// Create a new CMUT cell with silicon-nitride defaults.
    ///
    /// * `radius_um` - membrane radius in micrometres
    /// * `gap_um` - vacuum gap in micrometres
    /// * `membrane_thickness_um` - membrane thickness in micrometres
    /// * `dc_bias_v` - DC bias voltage
    pub fn new(radius_um: f64, gap_um: f64, membrane_thickness_um: f64, dc_bias_v: f64) -> Self {
        let a = radius_um * 1.0e-6;
        let area = PI * a * a * 1.0e12; // back to um^2
        Self {
            radius_um,
            gap_um,
            membrane_thickness_um,
            electrode_area_um2: area,
            dc_bias_v,
            youngs_modulus: SI3N4_YOUNGS_MODULUS,
            poisson_ratio: SI3N4_POISSON,
            density: SI3N4_DENSITY,
        }
    }

    /// Create a cell with custom material properties.
    pub fn with_material(
        radius_um: f64,
        gap_um: f64,
        membrane_thickness_um: f64,
        dc_bias_v: f64,
        youngs_modulus: f64,
        poisson_ratio: f64,
        density: f64,
    ) -> Self {
        let a = radius_um * 1.0e-6;
        let area = PI * a * a * 1.0e12;
        Self {
            radius_um,
            gap_um,
            membrane_thickness_um,
            electrode_area_um2: area,
            dc_bias_v,
            youngs_modulus,
            poisson_ratio,
            density,
        }
    }

    // -- helpers (SI units) --

    fn radius_m(&self) -> f64 {
        self.radius_um * 1.0e-6
    }

    fn gap_m(&self) -> f64 {
        self.gap_um * 1.0e-6
    }

    fn thickness_m(&self) -> f64 {
        self.membrane_thickness_um * 1.0e-6
    }

    fn electrode_area_m2(&self) -> f64 {
        self.electrode_area_um2 * 1.0e-12
    }

    /// Flexural rigidity D = E*h^3 / (12*(1-nu^2)).
    pub fn flexural_rigidity(&self) -> f64 {
        let h = self.thickness_m();
        let nu = self.poisson_ratio;
        self.youngs_modulus * h.powi(3) / (12.0 * (1.0 - nu * nu))
    }

    /// Effective spring constant for a clamped circular membrane.
    /// k_eff = 192 * pi * D / a^2
    pub fn effective_spring_constant(&self) -> f64 {
        192.0 * PI * self.flexural_rigidity() / self.radius_m().powi(2)
    }

    /// Resonance frequency of the membrane in vacuum (Hz).
    ///
    /// f0 = (10.21 / (2*pi*a^2)) * sqrt(D / (rho*h))
    pub fn resonance_frequency_hz(&self) -> f64 {
        let a = self.radius_m();
        let d = self.flexural_rigidity();
        let rho = self.density;
        let h = self.thickness_m();
        (10.21 / (2.0 * PI * a * a)) * (d / (rho * h)).sqrt()
    }

    /// Collapse (pull-in) voltage.
    ///
    /// V_collapse = sqrt(8 * k_eff * g0^3 / (27 * epsilon_0 * A))
    pub fn collapse_voltage(&self) -> f64 {
        let k = self.effective_spring_constant();
        let g0 = self.gap_m();
        let a = self.electrode_area_m2();
        (8.0 * k * g0.powi(3) / (27.0 * EPSILON_0 * a)).sqrt()
    }

    /// Static deflection from DC bias (metres), solved iteratively.
    ///
    /// Solves x = epsilon_0 * A * V^2 / (2 * k * (g0 - x)^2).
    /// Returns None if the bias exceeds collapse voltage.
    pub fn static_deflection_m(&self) -> Option<f64> {
        let k = self.effective_spring_constant();
        let g0 = self.gap_m();
        let a = self.electrode_area_m2();
        let v = self.dc_bias_v;
        let coeff = EPSILON_0 * a * v * v / (2.0 * k);

        let mut x = 0.0_f64;
        for _ in 0..200 {
            let gap = g0 - x;
            if gap <= 1.0e-15 {
                return None; // collapsed
            }
            let x_new = coeff / (gap * gap);
            if x_new >= g0 {
                return None; // collapse
            }
            if (x_new - x).abs() < 1.0e-15 {
                return Some(x_new);
            }
            // damped update for stability
            x = 0.5 * x + 0.5 * x_new;
        }
        // check convergence
        if x < g0 * 0.99 {
            Some(x)
        } else {
            None
        }
    }

    /// Parallel-plate capacitance at a given deflection (F).
    ///
    /// C = epsilon_0 * A / (g0 - x)
    pub fn capacitance_at_deflection(&self, deflection_m: f64) -> f64 {
        let gap = self.gap_m() - deflection_m;
        if gap <= 0.0 {
            return f64::INFINITY;
        }
        EPSILON_0 * self.electrode_area_m2() / gap
    }

    /// Capacitance at the current DC bias static deflection.
    pub fn capacitance(&self) -> f64 {
        match self.static_deflection_m() {
            Some(x) => self.capacitance_at_deflection(x),
            None => f64::INFINITY,
        }
    }

    /// Fringing field correction factor for a circular plate capacitor.
    ///
    /// Kirchhoff approximation: C_corrected = C * (1 + (g/a)*ln(2*pi*a/g)/pi)
    pub fn fringing_field_factor(&self) -> f64 {
        let g = self.gap_m();
        let a = self.radius_m();
        if a <= 0.0 || g <= 0.0 {
            return 1.0;
        }
        1.0 + (g / a) * (2.0 * PI * a / g).ln() / PI
    }

    /// Capacitance with fringing field correction.
    pub fn capacitance_corrected(&self) -> f64 {
        self.capacitance() * self.fringing_field_factor()
    }

    /// Transmit sensitivity: approximate pressure output per volt (Pa/V).
    ///
    /// S_tx ~ epsilon_0 * A * V_dc / (g0^2)  (simplified electrostatic model)
    pub fn transmit_sensitivity(&self) -> f64 {
        let a = self.electrode_area_m2();
        let g0 = self.gap_m();
        EPSILON_0 * a * self.dc_bias_v / (g0 * g0)
    }

    /// Receive sensitivity: approximate voltage output per Pascal (V/Pa).
    ///
    /// S_rx ~ V_dc * epsilon_0 * A / (k * g0^2)
    pub fn receive_sensitivity(&self) -> f64 {
        let a = self.electrode_area_m2();
        let g0 = self.gap_m();
        let k = self.effective_spring_constant();
        self.dc_bias_v * EPSILON_0 * a / (k * g0 * g0)
    }
}

// ---------------------------------------------------------------------------
// CmutArray
// ---------------------------------------------------------------------------

/// Linear array of CMUT elements.
#[derive(Debug, Clone)]
pub struct CmutArray {
    /// Number of elements.
    pub num_elements: usize,
    /// Centre-to-centre pitch (micrometres).
    pub pitch_um: f64,
    /// Per-element configurations.
    pub element_configs: Vec<CmutCell>,
}

impl CmutArray {
    /// Create a uniform array where every element shares the same configuration.
    pub fn new_uniform(num_elements: usize, pitch_um: f64, cell: CmutCell) -> Self {
        Self {
            num_elements,
            pitch_um,
            element_configs: vec![cell; num_elements],
        }
    }

    /// Create an array with individually specified elements.
    pub fn new(pitch_um: f64, elements: Vec<CmutCell>) -> Self {
        let n = elements.len();
        Self {
            num_elements: n,
            pitch_um,
            element_configs: elements,
        }
    }

    /// X-positions of array elements in micrometres, centred around zero.
    pub fn element_positions_um(&self) -> Vec<f64> {
        let n = self.num_elements;
        let total = (n as f64 - 1.0) * self.pitch_um;
        (0..n)
            .map(|i| i as f64 * self.pitch_um - total / 2.0)
            .collect()
    }

    /// Compute focusing delays (seconds) for a focal point at (x_um, z_um) in
    /// micrometres. `speed_mps` is sound speed in m/s.
    pub fn focus_delays(&self, x_focus_um: f64, z_focus_um: f64, speed_mps: f64) -> Vec<f64> {
        let positions = self.element_positions_um();
        // Convert to metres
        let xf = x_focus_um * 1.0e-6;
        let zf = z_focus_um * 1.0e-6;

        let distances: Vec<f64> = positions
            .iter()
            .map(|&xi| {
                let xi_m = xi * 1.0e-6;
                ((xf - xi_m).powi(2) + zf * zf).sqrt()
            })
            .collect();

        let max_dist = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        distances.iter().map(|d| (max_dist - d) / speed_mps).collect()
    }

    /// Delay-and-sum beamforming on received RF data.
    ///
    /// * `rf_data` - 2D data: rf_data[element][sample]
    /// * `delays_s` - per-element delay in seconds
    /// * `sample_rate_hz` - sampling rate
    /// * `apodization` - window across the array
    pub fn delay_and_sum(
        &self,
        rf_data: &[Vec<f64>],
        delays_s: &[f64],
        sample_rate_hz: f64,
        apodization: Apodization,
    ) -> Vec<f64> {
        if rf_data.is_empty() {
            return vec![];
        }
        let n_elem = rf_data.len().min(delays_s.len());
        let n_samples = rf_data[0].len();
        let weights = apodization_weights(n_elem, apodization);
        let mut output = vec![0.0_f64; n_samples];

        for i in 0..n_elem {
            let delay_samples = delays_s[i] * sample_rate_hz;
            let w = weights[i];
            for s in 0..n_samples {
                let src = s as f64 - delay_samples;
                if src >= 0.0 && (src as usize) < n_samples.saturating_sub(1) {
                    let idx = src as usize;
                    let frac = src - idx as f64;
                    // linear interpolation
                    let val = rf_data[i][idx] * (1.0 - frac)
                        + rf_data[i][(idx + 1).min(n_samples - 1)] * frac;
                    output[s] += w * val;
                }
            }
        }
        output
    }

    /// Dynamic focusing: compute delay-and-sum output where the focus depth
    /// tracks each sample. Returns one beamformed A-line.
    pub fn dynamic_focus(
        &self,
        rf_data: &[Vec<f64>],
        x_focus_um: f64,
        sample_rate_hz: f64,
        speed_mps: f64,
        apodization: Apodization,
    ) -> Vec<f64> {
        if rf_data.is_empty() {
            return vec![];
        }
        let n_elem = rf_data.len().min(self.num_elements);
        let n_samples = rf_data[0].len();
        let weights = apodization_weights(n_elem, apodization);
        let positions = self.element_positions_um();
        let xf = x_focus_um * 1.0e-6;

        let mut output = vec![0.0_f64; n_samples];
        for s in 0..n_samples {
            // depth corresponding to this sample (round-trip time)
            let z_m = (s as f64 / sample_rate_hz) * speed_mps / 2.0;
            if z_m <= 0.0 {
                continue;
            }
            let mut sum = 0.0_f64;
            for i in 0..n_elem {
                let xi_m = positions[i] * 1.0e-6;
                let dist = ((xf - xi_m).powi(2) + z_m * z_m).sqrt();
                // delay = (dist - z_m) / c  (relative to on-axis)
                let delay_s = (dist - z_m) / speed_mps;
                let src = s as f64 - delay_s * sample_rate_hz;
                if src >= 0.0 && (src as usize) < n_samples.saturating_sub(1) {
                    let idx = src as usize;
                    let frac = src - idx as f64;
                    let val = rf_data[i][idx] * (1.0 - frac)
                        + rf_data[i][(idx + 1).min(n_samples - 1)] * frac;
                    sum += weights[i] * val;
                }
            }
            output[s] = sum;
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Apodization
// ---------------------------------------------------------------------------

/// Apodization window types for array beamforming.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Apodization {
    /// Uniform (rectangular) weighting.
    Rectangular,
    /// Hann window.
    Hann,
    /// Hamming window.
    Hamming,
}

/// Generate apodization weights for `n` elements.
pub fn apodization_weights(n: usize, window: Apodization) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![1.0];
    }
    let nm1 = (n - 1) as f64;
    match window {
        Apodization::Rectangular => vec![1.0; n],
        Apodization::Hann => (0..n)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / nm1).cos()))
            .collect(),
        Apodization::Hamming => (0..n)
            .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / nm1).cos())
            .collect(),
    }
}

// ---------------------------------------------------------------------------
// Pulse-echo processing
// ---------------------------------------------------------------------------

/// Generate a Gaussian-modulated sinusoidal transmit pulse.
///
/// * `center_freq_hz` - centre frequency
/// * `bandwidth_frac` - fractional bandwidth (e.g., 0.6 for 60%)
/// * `sample_rate_hz` - sample rate
/// * `num_cycles` - approximate number of cycles in the pulse
pub fn generate_tx_pulse(
    center_freq_hz: f64,
    bandwidth_frac: f64,
    sample_rate_hz: f64,
    num_cycles: f64,
) -> Vec<f64> {
    let duration = num_cycles / center_freq_hz;
    let n = (duration * sample_rate_hz).ceil() as usize;
    if n == 0 {
        return vec![];
    }
    let t_mid = duration / 2.0;
    // Gaussian sigma from bandwidth: BW ~ 1/(pi*sigma)
    let sigma = 1.0 / (PI * center_freq_hz * bandwidth_frac);

    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate_hz;
            let dt = t - t_mid;
            let envelope = (-dt * dt / (2.0 * sigma * sigma)).exp();
            envelope * (2.0 * PI * center_freq_hz * t).sin()
        })
        .collect()
}

/// Time-Gain Compensation: gain increasing with depth to compensate
/// tissue attenuation.
///
/// * `n_samples` - number of samples
/// * `sample_rate_hz` - sampling rate
/// * `speed_mps` - speed of sound
/// * `attenuation_db_per_cm_mhz` - attenuation coefficient
/// * `freq_mhz` - centre frequency in MHz
pub fn tgc_curve(
    n_samples: usize,
    sample_rate_hz: f64,
    speed_mps: f64,
    attenuation_db_per_cm_mhz: f64,
    freq_mhz: f64,
) -> Vec<f64> {
    (0..n_samples)
        .map(|i| {
            // round-trip depth in cm
            let depth_cm = (i as f64 / sample_rate_hz) * speed_mps / 2.0 * 100.0;
            let loss_db = attenuation_db_per_cm_mhz * freq_mhz * depth_cm;
            10.0_f64.powf(loss_db / 20.0)
        })
        .collect()
}

/// Apply TGC to an RF signal.
pub fn apply_tgc(signal: &[f64], tgc: &[f64]) -> Vec<f64> {
    signal
        .iter()
        .zip(tgc.iter())
        .map(|(&s, &g)| s * g)
        .collect()
}

/// Envelope detection via Hilbert transform magnitude.
///
/// Uses a frequency-domain approach: zero negative frequencies, IFFT, take magnitude.
pub fn envelope_detect(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // DFT
    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];
    dft_inplace(&mut re, &mut im, false);

    // Zero negative frequencies (analytic signal)
    // DC and Nyquist stay as-is, positive freqs doubled, negative zeroed
    if n >= 2 {
        let half = if n % 2 == 0 { n / 2 } else { (n + 1) / 2 };
        for i in 1..half {
            re[i] *= 2.0;
            im[i] *= 2.0;
        }
        for i in (half + 1)..n {
            re[i] = 0.0;
            im[i] = 0.0;
        }
    }

    // Inverse DFT
    dft_inplace(&mut re, &mut im, true);

    // Magnitude
    re.iter()
        .zip(im.iter())
        .map(|(&r, &i)| (r * r + i * i).sqrt())
        .collect()
}

/// Log compression for B-mode display.
///
/// Returns values in \[0, 1\] with `dynamic_range_db` of compression.
pub fn log_compress(envelope: &[f64], dynamic_range_db: f64) -> Vec<f64> {
    if envelope.is_empty() {
        return vec![];
    }
    let max_val = envelope.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if max_val <= 0.0 {
        return vec![0.0; envelope.len()];
    }
    envelope
        .iter()
        .map(|&e| {
            if e <= 0.0 {
                0.0
            } else {
                let db = 20.0 * (e / max_val).log10();
                ((db + dynamic_range_db) / dynamic_range_db).clamp(0.0, 1.0)
            }
        })
        .collect()
}

/// A-line: single-element pulse-echo processing pipeline.
///
/// Returns envelope-detected, TGC-compensated, log-compressed signal.
pub fn process_a_line(
    rf_signal: &[f64],
    sample_rate_hz: f64,
    speed_mps: f64,
    attenuation_db_per_cm_mhz: f64,
    freq_mhz: f64,
    dynamic_range_db: f64,
) -> Vec<f64> {
    let tgc = tgc_curve(
        rf_signal.len(),
        sample_rate_hz,
        speed_mps,
        attenuation_db_per_cm_mhz,
        freq_mhz,
    );
    let compensated = apply_tgc(rf_signal, &tgc);
    let env = envelope_detect(&compensated);
    log_compress(&env, dynamic_range_db)
}

/// B-mode image from multiple A-lines.
///
/// * `rf_lines` - rf_lines\[line\]\[sample\]
///
/// Returns a flat vector of pixel values, row-major, width = num_lines, height = num_samples.
pub fn bmode_image(
    rf_lines: &[Vec<f64>],
    sample_rate_hz: f64,
    speed_mps: f64,
    attenuation_db_per_cm_mhz: f64,
    freq_mhz: f64,
    dynamic_range_db: f64,
) -> (Vec<f64>, usize, usize) {
    if rf_lines.is_empty() {
        return (vec![], 0, 0);
    }
    let width = rf_lines.len();
    let height = rf_lines[0].len();
    let mut pixels = vec![0.0_f64; width * height];

    for (col, line) in rf_lines.iter().enumerate() {
        let processed = process_a_line(
            line,
            sample_rate_hz,
            speed_mps,
            attenuation_db_per_cm_mhz,
            freq_mhz,
            dynamic_range_db,
        );
        for (row, &val) in processed.iter().enumerate() {
            if row < height {
                pixels[row * width + col] = val;
            }
        }
    }
    (pixels, width, height)
}

// ---------------------------------------------------------------------------
// Acoustic parameters
// ---------------------------------------------------------------------------

/// Acoustic impedance Z = rho * c (Rayl = kg/(m^2*s)).
pub fn acoustic_impedance(density_kgm3: f64, speed_mps: f64) -> f64 {
    density_kgm3 * speed_mps
}

/// Reflection coefficient at a planar boundary.
///
/// R = (Z2 - Z1) / (Z2 + Z1)
pub fn reflection_coefficient(z1: f64, z2: f64) -> f64 {
    (z2 - z1) / (z2 + z1)
}

/// Transmission coefficient at a planar boundary.
///
/// T = 2*Z2 / (Z1 + Z2)
pub fn transmission_coefficient(z1: f64, z2: f64) -> f64 {
    2.0 * z2 / (z1 + z2)
}

/// Attenuation in dB for a given depth, frequency, and tissue parameters.
///
/// alpha_total = alpha_0 * f^n * depth_cm  (dB)
pub fn attenuation_db(
    depth_cm: f64,
    freq_mhz: f64,
    alpha_0: f64,
    n: f64,
) -> f64 {
    alpha_0 * freq_mhz.powf(n) * depth_cm
}

// ---------------------------------------------------------------------------
// Resolution
// ---------------------------------------------------------------------------

/// Axial resolution: delta_z = c / (2 * BW) where BW is bandwidth in Hz.
pub fn axial_resolution(speed_mps: f64, bandwidth_hz: f64) -> f64 {
    speed_mps / (2.0 * bandwidth_hz)
}

/// Lateral resolution (focused): delta_x = lambda * F_number.
///
/// F_number = focal_distance / aperture_size.
pub fn lateral_resolution_focused(wavelength_m: f64, focal_distance_m: f64, aperture_m: f64) -> f64 {
    wavelength_m * focal_distance_m / aperture_m
}

/// Lateral resolution (unfocused): delta_x ~ lambda * z / D approximately.
pub fn lateral_resolution_unfocused(wavelength_m: f64, depth_m: f64, aperture_m: f64) -> f64 {
    wavelength_m * depth_m / aperture_m
}

/// Wavelength from frequency and speed of sound.
pub fn wavelength(speed_mps: f64, freq_hz: f64) -> f64 {
    speed_mps / freq_hz
}

// ---------------------------------------------------------------------------
// Harmonic imaging
// ---------------------------------------------------------------------------

/// Extract second harmonic from a signal by bandpass filtering around 2*f0.
///
/// Uses a simple Gaussian-shaped frequency-domain filter.
pub fn extract_second_harmonic(
    signal: &[f64],
    center_freq_hz: f64,
    bandwidth_hz: f64,
    sample_rate_hz: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];
    dft_inplace(&mut re, &mut im, false);

    let f2 = 2.0 * center_freq_hz;
    let sigma_f = bandwidth_hz / 2.355; // FWHM to sigma

    for k in 0..n {
        let freq = if k <= n / 2 {
            k as f64 * sample_rate_hz / n as f64
        } else {
            (k as f64 - n as f64) * sample_rate_hz / n as f64
        };
        let df = (freq.abs() - f2).abs();
        let weight = (-df * df / (2.0 * sigma_f * sigma_f)).exp();
        re[k] *= weight;
        im[k] *= weight;
    }

    dft_inplace(&mut re, &mut im, true);
    re
}

/// Pulse inversion: sum two transmissions with inverted polarity.
///
/// The fundamental cancels, leaving the second harmonic.
pub fn pulse_inversion(signal_pos: &[f64], signal_neg: &[f64]) -> Vec<f64> {
    let n = signal_pos.len().min(signal_neg.len());
    (0..n)
        .map(|i| signal_pos[i] + signal_neg[i])
        .collect()
}

// ---------------------------------------------------------------------------
// Element characterisation
// ---------------------------------------------------------------------------

/// Compute impedance magnitude and phase vs frequency for a CMUT element.
///
/// Simplified model: parallel R-L-C equivalent near resonance.
///
/// Returns (frequencies_hz, magnitude_ohms, phase_rad).
pub fn element_impedance_spectrum(
    cell: &CmutCell,
    freq_start_hz: f64,
    freq_end_hz: f64,
    num_points: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let f0 = cell.resonance_frequency_hz();
    let c0 = cell.capacitance();
    // Model: series R-L-C with motional branch
    // Motional parameters (simplified Butterworth-Van Dyke model)
    let q_factor = 50.0; // typical CMUT Q
    let omega0 = 2.0 * PI * f0;
    let lm = 1.0 / (c0 * omega0 * omega0); // motional inductance
    let rm = omega0 * lm / q_factor; // motional resistance
    let cm = c0; // motional capacitance

    let mut freqs = Vec::with_capacity(num_points);
    let mut mags = Vec::with_capacity(num_points);
    let mut phases = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let f = freq_start_hz + (freq_end_hz - freq_start_hz) * i as f64 / (num_points.max(1) - 1).max(1) as f64;
        let omega = 2.0 * PI * f;

        // Motional branch impedance: Z_m = Rm + j*omega*Lm + 1/(j*omega*Cm)
        let z_m_re = rm;
        let z_m_im = omega * lm - 1.0 / (omega * cm);

        // Parallel with static capacitance C0: Z_c0 = 1/(j*omega*C0)
        let z_c0_re = 0.0;
        let z_c0_im = -1.0 / (omega * c0);

        // Parallel combination: 1/Z_total = 1/Z_m + 1/Z_c0
        let (inv_re_m, inv_im_m) = complex_inv(z_m_re, z_m_im);
        let (inv_re_c, inv_im_c) = complex_inv(z_c0_re, z_c0_im);
        let sum_re = inv_re_m + inv_re_c;
        let sum_im = inv_im_m + inv_im_c;
        let (z_re, z_im) = complex_inv(sum_re, sum_im);

        let mag = (z_re * z_re + z_im * z_im).sqrt();
        let phase = z_im.atan2(z_re);

        freqs.push(f);
        mags.push(mag);
        phases.push(phase);
    }
    (freqs, mags, phases)
}

/// Fractional bandwidth at -6 dB from the peak spectrum magnitude.
///
/// Returns fractional bandwidth = (f_high - f_low) / f_center.
pub fn fractional_bandwidth_6db(spectrum_mag: &[f64], freq_axis_hz: &[f64]) -> f64 {
    if spectrum_mag.is_empty() || freq_axis_hz.is_empty() {
        return 0.0;
    }
    let n = spectrum_mag.len().min(freq_axis_hz.len());

    let peak_val = spectrum_mag[..n]
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    if peak_val <= 0.0 {
        return 0.0;
    }

    let threshold = peak_val * 0.5; // -6 dB in amplitude

    let mut f_low = freq_axis_hz[0];
    let mut f_high = freq_axis_hz[n - 1];

    // Find lower edge
    for i in 0..n {
        if spectrum_mag[i] >= threshold {
            f_low = freq_axis_hz[i];
            break;
        }
    }

    // Find upper edge
    for i in (0..n).rev() {
        if spectrum_mag[i] >= threshold {
            f_high = freq_axis_hz[i];
            break;
        }
    }

    let f_center = (f_low + f_high) / 2.0;
    if f_center <= 0.0 {
        return 0.0;
    }
    (f_high - f_low) / f_center
}

/// Center frequency from spectrum: frequency at peak magnitude.
pub fn center_frequency(spectrum_mag: &[f64], freq_axis_hz: &[f64]) -> f64 {
    let n = spectrum_mag.len().min(freq_axis_hz.len());
    if n == 0 {
        return 0.0;
    }
    let mut max_val = f64::NEG_INFINITY;
    let mut max_idx = 0;
    for i in 0..n {
        if spectrum_mag[i] > max_val {
            max_val = spectrum_mag[i];
            max_idx = i;
        }
    }
    freq_axis_hz[max_idx]
}

/// Estimate crosstalk between adjacent elements (dB).
///
/// Simplified model based on element spacing and frequency.
/// Crosstalk ~ -20*log10(pitch/lambda) - 10 dB (empirical approximation).
pub fn crosstalk_db(pitch_um: f64, freq_hz: f64, speed_mps: f64) -> f64 {
    let lambda = speed_mps / freq_hz;
    let pitch_m = pitch_um * 1.0e-6;
    let ratio = pitch_m / lambda;
    if ratio <= 0.0 {
        return 0.0;
    }
    -20.0 * ratio.log10() - 10.0
}

// ---------------------------------------------------------------------------
// Internal DFT (no external FFT dependency)
// ---------------------------------------------------------------------------

fn dft_inplace(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    if n <= 1 {
        return;
    }

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Cooley-Tukey radix-2 (works best for power-of-2, but handles any n via DFT fallback)
    // For non-power-of-2, we use the naive O(N^2) approach
    if n & (n - 1) != 0 {
        // Naive DFT for non-power-of-2
        let sign = if inverse { 1.0 } else { -1.0 };
        let mut out_re = vec![0.0_f64; n];
        let mut out_im = vec![0.0_f64; n];

        // undo bit-reversal first (swap back)
        let mut tmp_re = re.to_vec();
        let mut tmp_im = im.to_vec();
        // just recopy from original since we already permuted
        // Actually we need to undo. Let's just do naive DFT on permuted data—
        // easier to just revert. Re-do bit reversal to undo:
        j = 0;
        for i in 1..n {
            let mut bit = n >> 1;
            while j & bit != 0 {
                j ^= bit;
                bit >>= 1;
            }
            j ^= bit;
            if i < j {
                tmp_re.swap(i, j);
                tmp_im.swap(i, j);
            }
        }

        for k in 0..n {
            let mut sr = 0.0_f64;
            let mut si = 0.0_f64;
            for t in 0..n {
                let angle = sign * 2.0 * PI * (k * t) as f64 / n as f64;
                let c = angle.cos();
                let s = angle.sin();
                sr += tmp_re[t] * c - tmp_im[t] * s;
                si += tmp_re[t] * s + tmp_im[t] * c;
            }
            out_re[k] = sr;
            out_im[k] = si;
        }
        if inverse {
            for k in 0..n {
                out_re[k] /= n as f64;
                out_im[k] /= n as f64;
            }
        }
        re.copy_from_slice(&out_re);
        im.copy_from_slice(&out_im);
        return;
    }

    // Radix-2 FFT butterfly
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = sign * 2.0 * PI / len as f64;
        let wn_re = angle_step.cos();
        let wn_im = angle_step.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0_f64;
            let mut w_im = 0.0_f64;
            for k in 0..half {
                let i = start + k;
                let j2 = start + k + half;
                let tr = w_re * re[j2] - w_im * im[j2];
                let ti = w_re * im[j2] + w_im * re[j2];
                re[j2] = re[i] - tr;
                im[j2] = im[i] - ti;
                re[i] += tr;
                im[i] += ti;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for k in 0..n {
            re[k] *= inv_n;
            im[k] *= inv_n;
        }
    }
}

/// Complex reciprocal: 1/(a + jb).
fn complex_inv(a: f64, b: f64) -> (f64, f64) {
    let denom = a * a + b * b;
    if denom < 1e-30 {
        return (1e15, 0.0); // large impedance for near-zero
    }
    (a / denom, -b / denom)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- CmutCell basics --

    #[test]
    fn test_cmut_cell_creation() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        assert_eq!(cell.radius_um, 25.0);
        assert_eq!(cell.gap_um, 0.2);
        assert_eq!(cell.membrane_thickness_um, 1.5);
        assert_eq!(cell.dc_bias_v, 50.0);
        assert!(cell.electrode_area_um2 > 0.0);
    }

    #[test]
    fn test_cmut_cell_with_material() {
        let cell = CmutCell::with_material(30.0, 0.3, 2.0, 40.0, 200e9, 0.25, 3000.0);
        assert_eq!(cell.youngs_modulus, 200e9);
        assert_eq!(cell.poisson_ratio, 0.25);
        assert_eq!(cell.density, 3000.0);
    }

    #[test]
    fn test_flexural_rigidity() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let d = cell.flexural_rigidity();
        assert!(d > 0.0);
        // D = E*h^3/(12*(1-nu^2))
        let h: f64 = 1.5e-6;
        let expected = SI3N4_YOUNGS_MODULUS * h.powi(3)
            / (12.0 * (1.0 - SI3N4_POISSON * SI3N4_POISSON));
        assert!(approx_eq(d, expected, expected * 1e-10));
    }

    #[test]
    fn test_effective_spring_constant() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let k = cell.effective_spring_constant();
        assert!(k > 0.0);
        let expected = 192.0 * PI * cell.flexural_rigidity() / cell.radius_m().powi(2);
        assert!(approx_eq(k, expected, expected * 1e-10));
    }

    #[test]
    fn test_resonance_frequency_mhz_range() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let f0 = cell.resonance_frequency_hz();
        // For a 25 um radius Si3N4 membrane, resonance should be in MHz range
        assert!(f0 > 1.0e6, "f0 = {} should be > 1 MHz", f0);
        assert!(f0 < 100.0e6, "f0 = {} should be < 100 MHz", f0);
    }

    #[test]
    fn test_resonance_scales_with_radius() {
        let cell_small = CmutCell::new(20.0, 0.2, 1.5, 50.0);
        let cell_large = CmutCell::new(40.0, 0.2, 1.5, 50.0);
        // Smaller membrane => higher resonance (f0 ~ 1/a^2)
        assert!(cell_small.resonance_frequency_hz() > cell_large.resonance_frequency_hz());
    }

    #[test]
    fn test_collapse_voltage() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let v_c = cell.collapse_voltage();
        assert!(v_c > 0.0);
        assert!(v_c > 10.0, "Collapse voltage {} should be > 10 V", v_c);
    }

    #[test]
    fn test_static_deflection_below_collapse() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 10.0); // low bias
        let v_c = cell.collapse_voltage();
        assert!(cell.dc_bias_v < v_c, "Bias should be below collapse");
        let x = cell.static_deflection_m();
        assert!(x.is_some(), "Should converge below collapse");
        let deflection = x.unwrap();
        assert!(deflection > 0.0);
        assert!(deflection < cell.gap_m());
    }

    #[test]
    fn test_static_deflection_zero_bias() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 0.0);
        let x = cell.static_deflection_m();
        assert!(x.is_some());
        assert!(x.unwrap().abs() < 1e-20);
    }

    #[test]
    fn test_capacitance_positive() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 10.0);
        let c = cell.capacitance();
        assert!(c > 0.0);
        assert!(c.is_finite());
    }

    #[test]
    fn test_capacitance_at_deflection() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 0.0);
        let c0 = cell.capacitance_at_deflection(0.0);
        let c1 = cell.capacitance_at_deflection(cell.gap_m() * 0.5);
        // Closer plates => higher capacitance
        assert!(c1 > c0);
    }

    #[test]
    fn test_fringing_field_factor_gt_1() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let ff = cell.fringing_field_factor();
        assert!(ff > 1.0, "Fringing factor {} should be > 1", ff);
    }

    #[test]
    fn test_capacitance_corrected_gt_uncorrected() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 10.0);
        let c = cell.capacitance();
        let cc = cell.capacitance_corrected();
        assert!(cc > c);
    }

    #[test]
    fn test_transmit_sensitivity() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let s_tx = cell.transmit_sensitivity();
        assert!(s_tx > 0.0);
    }

    #[test]
    fn test_receive_sensitivity() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let s_rx = cell.receive_sensitivity();
        assert!(s_rx > 0.0);
    }

    // -- CmutArray --

    #[test]
    fn test_array_creation_uniform() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(64, 150.0, cell);
        assert_eq!(array.num_elements, 64);
        assert_eq!(array.element_configs.len(), 64);
    }

    #[test]
    fn test_element_positions_symmetric() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(8, 100.0, cell);
        let pos = array.element_positions_um();
        assert_eq!(pos.len(), 8);
        // Should be centred around zero
        let sum: f64 = pos.iter().sum();
        assert!(sum.abs() < TOL, "Positions should be centred, sum={}", sum);
    }

    #[test]
    fn test_focus_delays_on_axis() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(8, 300.0, cell);
        // Focus on axis at depth 30mm
        let delays = array.focus_delays(0.0, 30_000.0, 1540.0);
        assert_eq!(delays.len(), 8);
        // Edge elements should have larger delays (they're farther from focus)
        // Actually, delay = (max_dist - dist)/c, so edge elements that are farther
        // get smaller delays. The centre elements (closest to focus on axis at same z)
        // wait while edges fire first.
        // For on-axis focus, centre element is closest => max delay
        let mid = delays.len() / 2;
        assert!(delays[mid] >= delays[0] - 1e-12 || delays[mid - 1] >= delays[0] - 1e-12);
    }

    #[test]
    fn test_delay_and_sum_basic() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(4, 300.0, cell);
        // Simple RF data: all same => output is just sum
        let rf: Vec<Vec<f64>> = vec![vec![1.0; 100]; 4];
        let delays = vec![0.0; 4];
        let result = array.delay_and_sum(&rf, &delays, 40e6, Apodization::Rectangular);
        assert_eq!(result.len(), 100);
        // With zero delay and rectangular window, output should be close to 4.0
        assert!(approx_eq(result[50], 4.0, 0.1));
    }

    #[test]
    fn test_delay_and_sum_hann() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(4, 300.0, cell);
        let rf: Vec<Vec<f64>> = vec![vec![1.0; 100]; 4];
        let delays = vec![0.0; 4];
        let result = array.delay_and_sum(&rf, &delays, 40e6, Apodization::Hann);
        assert_eq!(result.len(), 100);
        // With Hann window, sum should be less than 4.0 (edges suppressed)
        assert!(result[50] < 4.0);
        assert!(result[50] > 0.0);
    }

    #[test]
    fn test_dynamic_focus() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(4, 300.0, cell);
        let rf: Vec<Vec<f64>> = vec![vec![1.0; 64]; 4];
        let result = array.dynamic_focus(&rf, 0.0, 40e6, 1540.0, Apodization::Rectangular);
        assert_eq!(result.len(), 64);
        // Should produce non-zero output
        assert!(result.iter().any(|&v| v > 0.0));
    }

    // -- Apodization --

    #[test]
    fn test_apodization_rectangular() {
        let w = apodization_weights(8, Apodization::Rectangular);
        assert_eq!(w.len(), 8);
        assert!(w.iter().all(|&v| approx_eq(v, 1.0, TOL)));
    }

    #[test]
    fn test_apodization_hann() {
        let w = apodization_weights(8, Apodization::Hann);
        assert_eq!(w.len(), 8);
        // Hann window: edges should be near zero
        assert!(w[0].abs() < TOL);
        assert!(w[7].abs() < TOL);
        // Middle should be near 1
        assert!(w[4] > 0.5);
    }

    #[test]
    fn test_apodization_hamming() {
        let w = apodization_weights(8, Apodization::Hamming);
        assert_eq!(w.len(), 8);
        // Hamming window: edges at 0.08, not zero
        assert!(w[0] > 0.05);
        assert!(w[0] < 0.15);
    }

    // -- Pulse-echo processing --

    #[test]
    fn test_generate_tx_pulse() {
        let pulse = generate_tx_pulse(5.0e6, 0.6, 40.0e6, 3.0);
        assert!(!pulse.is_empty());
        // Should have oscillations that pass through zero
        let has_positive = pulse.iter().any(|&v| v > 0.01);
        let has_negative = pulse.iter().any(|&v| v < -0.01);
        assert!(has_positive && has_negative);
    }

    #[test]
    fn test_tgc_curve_increases() {
        let tgc = tgc_curve(100, 40.0e6, 1540.0, 0.5, 5.0);
        assert_eq!(tgc.len(), 100);
        // First sample at depth 0 => gain = 1
        assert!(approx_eq(tgc[0], 1.0, 0.01));
        // Gain increases with depth
        assert!(tgc[99] > tgc[0]);
    }

    #[test]
    fn test_apply_tgc() {
        let signal = vec![1.0; 10];
        let tgc = vec![2.0; 10];
        let result = apply_tgc(&signal, &tgc);
        assert!(result.iter().all(|&v| approx_eq(v, 2.0, TOL)));
    }

    #[test]
    fn test_envelope_detect_sine() {
        // Envelope of a pure sine should be roughly constant
        let n = 256;
        let f = 5.0e6;
        let fs = 40.0e6;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f * i as f64 / fs).sin())
            .collect();
        let env = envelope_detect(&signal);
        assert_eq!(env.len(), n);
        // Middle portion should be near 1.0 (edge effects expected)
        let mid = &env[n / 4..3 * n / 4];
        let avg: f64 = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!(
            approx_eq(avg, 1.0, 0.15),
            "Average envelope {} should be near 1.0",
            avg
        );
    }

    #[test]
    fn test_log_compress_range() {
        let env = vec![1.0, 0.1, 0.01, 0.001, 0.0001];
        let compressed = log_compress(&env, 60.0);
        assert_eq!(compressed.len(), 5);
        // Max should be 1.0
        assert!(approx_eq(compressed[0], 1.0, 0.01));
        // All values in [0, 1]
        assert!(compressed.iter().all(|&v| v >= 0.0 && v <= 1.0));
        // Monotonically decreasing
        for i in 1..compressed.len() {
            assert!(compressed[i] <= compressed[i - 1] + TOL);
        }
    }

    #[test]
    fn test_process_a_line() {
        let n = 128;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0e6 * i as f64 / 40.0e6).sin())
            .collect();
        let result = process_a_line(&signal, 40.0e6, 1540.0, 0.5, 5.0, 60.0);
        assert_eq!(result.len(), n);
        assert!(result.iter().all(|&v| v >= 0.0 && v <= 1.0));
    }

    #[test]
    fn test_bmode_image() {
        let n_lines = 16;
        let n_samples = 64;
        let rf_lines: Vec<Vec<f64>> = (0..n_lines)
            .map(|_| {
                (0..n_samples)
                    .map(|i| (2.0 * PI * 5.0e6 * i as f64 / 40.0e6).sin())
                    .collect()
            })
            .collect();
        let (pixels, w, h) = bmode_image(&rf_lines, 40.0e6, 1540.0, 0.5, 5.0, 60.0);
        assert_eq!(w, n_lines);
        assert_eq!(h, n_samples);
        assert_eq!(pixels.len(), w * h);
    }

    // -- Acoustic parameters --

    #[test]
    fn test_acoustic_impedance() {
        // Soft tissue: rho ~ 1000, c ~ 1540 => Z ~ 1.54 MRayl
        let z = acoustic_impedance(1000.0, 1540.0);
        assert!(approx_eq(z, 1_540_000.0, 1.0));
    }

    #[test]
    fn test_reflection_coefficient_same_medium() {
        let z = acoustic_impedance(1000.0, 1540.0);
        let r = reflection_coefficient(z, z);
        assert!(r.abs() < TOL);
    }

    #[test]
    fn test_reflection_coefficient_different_media() {
        let z1 = acoustic_impedance(1000.0, 1540.0); // soft tissue
        let z2 = acoustic_impedance(1700.0, 3500.0); // bone-like
        let r = reflection_coefficient(z1, z2);
        assert!(r.abs() > 0.0 && r.abs() < 1.0);
        // bone > tissue => positive reflection
        assert!(r > 0.0);
    }

    #[test]
    fn test_transmission_coefficient() {
        let z1 = acoustic_impedance(1000.0, 1540.0);
        let z2 = acoustic_impedance(1000.0, 1540.0);
        let t = transmission_coefficient(z1, z2);
        assert!(approx_eq(t, 1.0, TOL));
    }

    #[test]
    fn test_reflection_plus_transmission() {
        let z1 = acoustic_impedance(1000.0, 1540.0);
        let z2 = acoustic_impedance(1700.0, 3500.0);
        let r = reflection_coefficient(z1, z2);
        let t = transmission_coefficient(z1, z2);
        // R + T should not equal 1 (that's for intensity, not amplitude)
        // For pressure: T = 1 + R
        assert!(approx_eq(t, 1.0 + r, TOL));
    }

    #[test]
    fn test_attenuation_db() {
        // 0.5 dB/cm/MHz at 5 MHz, 10 cm depth => 25 dB
        let att = attenuation_db(10.0, 5.0, 0.5, 1.0);
        assert!(approx_eq(att, 25.0, TOL));
    }

    // -- Resolution --

    #[test]
    fn test_axial_resolution() {
        // c=1540, BW=3 MHz => delta_z = 1540/(2*3e6) ~ 0.257 mm
        let az = axial_resolution(1540.0, 3.0e6);
        assert!(approx_eq(az, 1540.0 / 6.0e6, TOL));
    }

    #[test]
    fn test_lateral_resolution_focused() {
        let lambda = wavelength(1540.0, 5.0e6);
        let lr = lateral_resolution_focused(lambda, 0.03, 0.01);
        // lambda ~ 0.000308 m, F/D=3 => lr ~ 0.000924 m
        assert!(lr > 0.0);
        assert!(lr < 0.01);
    }

    #[test]
    fn test_wavelength() {
        let w = wavelength(1540.0, 5.0e6);
        assert!(approx_eq(w, 1540.0 / 5.0e6, TOL));
    }

    // -- Harmonic imaging --

    #[test]
    fn test_pulse_inversion() {
        // Fundamental: sin(x), second harmonic: sin(2x)
        // Positive pulse: sin(x) + sin(2x)
        // Negative pulse: -sin(x) + sin(2x)  (invert fundamental only)
        let n = 256;
        let pos: Vec<f64> = (0..n)
            .map(|i| {
                let x = 2.0 * PI * i as f64 / n as f64;
                (x).sin() + 0.3 * (2.0 * x).sin()
            })
            .collect();
        let neg: Vec<f64> = (0..n)
            .map(|i| {
                let x = 2.0 * PI * i as f64 / n as f64;
                -(x).sin() + 0.3 * (2.0 * x).sin()
            })
            .collect();
        let result = pulse_inversion(&pos, &neg);
        // Fundamental cancels, second harmonic doubles
        for i in 0..n {
            let x = 2.0 * PI * i as f64 / n as f64;
            let expected = 0.6 * (2.0 * x).sin();
            assert!(
                approx_eq(result[i], expected, 1e-10),
                "i={}: {} vs {}",
                i,
                result[i],
                expected
            );
        }
    }

    #[test]
    fn test_extract_second_harmonic() {
        // Signal with fundamental at 5 MHz and second harmonic at 10 MHz
        let n = 256;
        let fs = 40.0e6;
        let f0 = 5.0e6;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * f0 * t).sin() + 0.5 * (2.0 * PI * 2.0 * f0 * t).sin()
            })
            .collect();
        let harmonic = extract_second_harmonic(&signal, f0, 2.0e6, fs);
        assert_eq!(harmonic.len(), n);
        // The harmonic content should be non-zero
        let energy: f64 = harmonic.iter().map(|&v| v * v).sum();
        assert!(energy > 0.0);
    }

    // -- Element characterisation --

    #[test]
    fn test_element_impedance_spectrum() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let (freqs, mags, phases) = element_impedance_spectrum(&cell, 1.0e6, 20.0e6, 100);
        assert_eq!(freqs.len(), 100);
        assert_eq!(mags.len(), 100);
        assert_eq!(phases.len(), 100);
        // All magnitudes positive
        assert!(mags.iter().all(|&m| m > 0.0));
    }

    #[test]
    fn test_fractional_bandwidth() {
        // Create a bandpass-like spectrum
        let n = 100;
        let mut mag = vec![0.0_f64; n];
        let freqs: Vec<f64> = (0..n).map(|i| 1.0e6 + (i as f64 / n as f64) * 10.0e6).collect();
        let f_center = 5.0e6;
        let bw = 2.0e6;
        for i in 0..n {
            let df = freqs[i] - f_center;
            mag[i] = (-df * df / (2.0 * (bw / 2.355_f64).powi(2))).exp();
        }
        let fbw = fractional_bandwidth_6db(&mag, &freqs);
        assert!(fbw > 0.0);
        assert!(fbw < 1.0);
    }

    #[test]
    fn test_center_frequency() {
        let freqs = vec![1.0e6, 2.0e6, 3.0e6, 4.0e6, 5.0e6];
        let mag = vec![0.1, 0.3, 1.0, 0.5, 0.1];
        let fc = center_frequency(&mag, &freqs);
        assert!(approx_eq(fc, 3.0e6, TOL));
    }

    #[test]
    fn test_crosstalk_db() {
        // Larger pitch => less crosstalk (more negative dB)
        let ct_small = crosstalk_db(100.0, 5.0e6, 1540.0);
        let ct_large = crosstalk_db(300.0, 5.0e6, 1540.0);
        assert!(ct_large < ct_small, "Larger pitch should give lower crosstalk");
    }

    #[test]
    fn test_crosstalk_negative() {
        let ct = crosstalk_db(150.0, 5.0e6, 1540.0);
        assert!(ct < 0.0, "Crosstalk should be negative dB, got {}", ct);
    }

    // -- DFT internal --

    #[test]
    fn test_dft_roundtrip() {
        let original = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut re = original.clone();
        let mut im = vec![0.0; 8];
        dft_inplace(&mut re, &mut im, false);
        dft_inplace(&mut re, &mut im, true);
        for i in 0..original.len() {
            assert!(
                approx_eq(re[i], original[i], 1e-10),
                "DFT roundtrip failed at {}: {} vs {}",
                i,
                re[i],
                original[i]
            );
        }
    }

    #[test]
    fn test_dft_dc_component() {
        let signal = vec![3.0; 8];
        let mut re = signal.clone();
        let mut im = vec![0.0; 8];
        dft_inplace(&mut re, &mut im, false);
        // DC component should be N * value = 24
        assert!(approx_eq(re[0], 24.0, 1e-10));
        // Other bins should be near zero
        for i in 1..8 {
            assert!(re[i].abs() < 1e-10 && im[i].abs() < 1e-10);
        }
    }

    // -- Edge cases --

    #[test]
    fn test_empty_envelope_detect() {
        let result = envelope_detect(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_empty_log_compress() {
        let result = log_compress(&[], 60.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_log_compress_all_zeros() {
        let result = log_compress(&[0.0, 0.0], 60.0);
        assert!(result.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_delay_and_sum_empty() {
        let cell = CmutCell::new(25.0, 0.2, 1.5, 50.0);
        let array = CmutArray::new_uniform(4, 300.0, cell);
        let result = array.delay_and_sum(&[], &[], 40e6, Apodization::Rectangular);
        assert!(result.is_empty());
    }
}
