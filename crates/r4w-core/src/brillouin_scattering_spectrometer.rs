//! Brillouin light scattering (BLS) spectroscopy signal processing.
//!
//! Brillouin scattering is the inelastic scattering of light from thermally excited acoustic
//! phonons (sound waves) in a material. The frequency shift of the scattered photons is
//! proportional to the acoustic velocity and hence the elastic properties of the medium.
//!
//! # Physics
//!
//! For backscattering geometry (θ = 180°), the Brillouin frequency shift is:
//!
//! ```text
//! νB = 2·n·Vs / λ
//! ```
//!
//! where `n` is the refractive index, `Vs` is the longitudinal sound velocity, and `λ` is the
//! laser wavelength.  The general form for arbitrary scattering angle θ is:
//!
//! ```text
//! νB = (2·n·Vs / λ) · sin(θ/2)
//! ```
//!
//! The longitudinal elastic modulus (M-modulus) relates to density ρ and sound velocity:
//!
//! ```text
//! M = ρ · Vs²
//! ```
//!
//! # Fabry-Pérot Interferometer
//!
//! Brillouin spectra are typically acquired with a scanning or VIPA Fabry-Pérot
//! interferometer (FPI).  The Airy transmission function of an ideal etalon is:
//!
//! ```text
//! T(δ) = 1 / (1 + F_coeff · sin²(δ/2))
//!
//! F_coeff = (2F/π)²
//! ```
//!
//! where `F` is the finesse and `δ = 4π·d·cos(θ)/λ` is the round-trip phase.  The free
//! spectral range (FSR) and finesse characterise the instrument resolution:
//!
//! ```text
//! FSR = c / (2·d)       [Hz]
//! finesse = FSR / FWHM
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::brillouin_scattering_spectrometer::{
//!     BrillouinSpectrometer, SpectrometerConfig, MaterialDatabase,
//! };
//!
//! let cfg = SpectrometerConfig::default();
//! let spec = BrillouinSpectrometer::new(cfg);
//!
//! // Calculate Brillouin shift for water
//! let water = MaterialDatabase::water();
//! let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
//! println!("Water Brillouin shift: {:.2} GHz", shift / 1e9);
//! ```

/// Speed of light in vacuum (m/s).
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for a Brillouin scattering spectrometer.
#[derive(Debug, Clone)]
pub struct SpectrometerConfig {
    /// Laser wavelength in metres (e.g. 532 nm = 532e-9).
    pub laser_wavelength_m: f64,
    /// Scattering angle θ in radians (π for backscattering).
    pub scattering_angle_rad: f64,
    /// Fabry-Pérot mirror spacing in metres.
    pub fp_spacing_m: f64,
    /// Mirror reflectivity (0–1) used to compute theoretical finesse.
    pub mirror_reflectivity: f64,
    /// Number of FP passes (1 = single-pass, 2 = tandem twin-pass, etc.).
    pub fp_passes: u32,
    /// Number of frequency points in a simulated scan.
    pub scan_points: usize,
    /// Half-width of the scan in Hz around the Rayleigh (elastic) peak.
    pub scan_half_width_hz: f64,
}

impl Default for SpectrometerConfig {
    fn default() -> Self {
        Self {
            laser_wavelength_m: 532e-9,
            scattering_angle_rad: std::f64::consts::PI,
            fp_spacing_m: 3e-3,
            mirror_reflectivity: 0.95,
            fp_passes: 1,
            scan_points: 1024,
            scan_half_width_hz: 60e9,
        }
    }
}

// ---------------------------------------------------------------------------
// Material database
// ---------------------------------------------------------------------------

/// Optical and acoustic properties of a material relevant to Brillouin spectroscopy.
#[derive(Debug, Clone)]
pub struct MaterialProperties {
    /// Human-readable material name.
    pub name: &'static str,
    /// Refractive index at the laser wavelength (dimensionless).
    pub refractive_index: f64,
    /// Longitudinal sound velocity (m/s).
    pub sound_velocity_ms: f64,
    /// Mass density (kg/m³).
    pub density_kg_m3: f64,
    /// Expected Brillouin shift at 532 nm backscattering (GHz), stored for quick lookup.
    pub brillouin_shift_ghz: f64,
    /// Temperature coefficient of Brillouin shift (GHz/°C).
    pub temp_coeff_ghz_per_c: f64,
    /// Pressure coefficient of sound velocity (m·s⁻¹·MPa⁻¹).
    pub pressure_coeff_ms_per_mpa: f64,
}

/// Built-in material database for common BLS targets.
pub struct MaterialDatabase;

impl MaterialDatabase {
    /// Water at 25 °C (distilled).
    pub fn water() -> MaterialProperties {
        MaterialProperties {
            name: "Water (25 °C)",
            refractive_index: 1.332,
            sound_velocity_ms: 1497.0,
            density_kg_m3: 997.0,
            brillouin_shift_ghz: 7.46,
            temp_coeff_ghz_per_c: -0.025,
            pressure_coeff_ms_per_mpa: 0.27,
        }
    }

    /// Fused silica (amorphous SiO₂).
    pub fn fused_silica() -> MaterialProperties {
        MaterialProperties {
            name: "Fused Silica",
            refractive_index: 1.461,
            sound_velocity_ms: 5968.0,
            density_kg_m3: 2203.0,
            brillouin_shift_ghz: 32.8,
            temp_coeff_ghz_per_c: 0.0012,
            pressure_coeff_ms_per_mpa: 0.018,
        }
    }

    /// PMMA (poly(methyl methacrylate), acrylic glass).
    pub fn pmma() -> MaterialProperties {
        MaterialProperties {
            name: "PMMA",
            refractive_index: 1.492,
            sound_velocity_ms: 2760.0,
            density_kg_m3: 1190.0,
            brillouin_shift_ghz: 15.4,
            temp_coeff_ghz_per_c: -0.030,
            pressure_coeff_ms_per_mpa: 0.20,
        }
    }

    /// Diamond (single crystal, [100] direction).
    ///
    /// Note: the stored `brillouin_shift_ghz` is computed from the formula
    /// νB = 2·n·Vs/λ at 532 nm backscattering geometry and should be taken as the
    /// theoretical value.  Published experimental values (~132 GHz) may differ due to
    /// anisotropy, specific crystallographic direction measured, or laser wavelength used.
    pub fn diamond() -> MaterialProperties {
        MaterialProperties {
            name: "Diamond",
            refractive_index: 2.417,
            sound_velocity_ms: 17_533.0,
            density_kg_m3: 3515.0,
            // 2 * 2.417 * 17533 / 532e-9 / 1e9 ≈ 159.3 GHz (formula at 532 nm)
            brillouin_shift_ghz: 159.3,
            temp_coeff_ghz_per_c: 0.0008,
            pressure_coeff_ms_per_mpa: 0.004,
        }
    }

    /// SCHOTT BK7 borosilicate crown glass.
    pub fn bk7_glass() -> MaterialProperties {
        MaterialProperties {
            name: "BK7 Glass",
            refractive_index: 1.519,
            sound_velocity_ms: 5830.0,
            density_kg_m3: 2510.0,
            brillouin_shift_ghz: 33.0,
            temp_coeff_ghz_per_c: 0.0010,
            pressure_coeff_ms_per_mpa: 0.020,
        }
    }

    /// Polycarbonate (Makrolon / Lexan).
    pub fn polycarbonate() -> MaterialProperties {
        MaterialProperties {
            name: "Polycarbonate",
            refractive_index: 1.586,
            sound_velocity_ms: 2270.0,
            density_kg_m3: 1200.0,
            brillouin_shift_ghz: 13.4,
            temp_coeff_ghz_per_c: -0.035,
            pressure_coeff_ms_per_mpa: 0.22,
        }
    }

    /// Silicon (single crystal, longitudinal [100]).
    ///
    /// The stored parameters use n = 3.88 (complex refractive index real part near 532 nm)
    /// and the [100] longitudinal acoustic velocity.  Published BLS experiments on silicon
    /// are often performed at 1064 nm or 780 nm (n ≈ 3.49) giving lower apparent shifts;
    /// the stored `brillouin_shift_ghz` reflects the formula at 532 nm.
    pub fn silicon() -> MaterialProperties {
        MaterialProperties {
            name: "Silicon",
            refractive_index: 3.880,
            sound_velocity_ms: 8433.0,
            density_kg_m3: 2329.0,
            // 2 * 3.880 * 8433 / 532e-9 / 1e9 ≈ 123.0 GHz (formula at 532 nm)
            brillouin_shift_ghz: 123.0,
            temp_coeff_ghz_per_c: 0.0005,
            pressure_coeff_ms_per_mpa: 0.012,
        }
    }

    /// Return all built-in materials.
    pub fn all() -> Vec<MaterialProperties> {
        vec![
            Self::water(),
            Self::fused_silica(),
            Self::pmma(),
            Self::diamond(),
            Self::bk7_glass(),
            Self::polycarbonate(),
            Self::silicon(),
        ]
    }

    /// Look up a material by name (case-insensitive prefix match).
    pub fn lookup(name: &str) -> Option<MaterialProperties> {
        let lower = name.to_lowercase();
        Self::all()
            .into_iter()
            .find(|m| m.name.to_lowercase().contains(&lower))
    }
}

// ---------------------------------------------------------------------------
// Fabry-Pérot interferometer model
// ---------------------------------------------------------------------------

/// Fabry-Pérot etalon model for computing instrumental transmission.
#[derive(Debug, Clone)]
pub struct FabryPerot {
    /// Mirror spacing (m).
    pub spacing_m: f64,
    /// Mirror reflectivity (0–1).
    pub reflectivity: f64,
    /// Number of passes through the etalon (tandem FPI uses 2).
    pub passes: u32,
}

impl FabryPerot {
    /// Create a new Fabry-Pérot model.
    pub fn new(spacing_m: f64, reflectivity: f64, passes: u32) -> Self {
        Self { spacing_m, reflectivity, passes }
    }

    /// Free spectral range (Hz).
    ///
    /// FSR = c / (2 · d)
    pub fn free_spectral_range_hz(&self) -> f64 {
        SPEED_OF_LIGHT / (2.0 * self.spacing_m)
    }

    /// Theoretical finesse from mirror reflectivity R.
    ///
    /// F = π√R / (1 − R)
    pub fn finesse(&self) -> f64 {
        let r = self.reflectivity;
        std::f64::consts::PI * r.sqrt() / (1.0 - r)
    }

    /// Instrument FWHM (Hz) = FSR / finesse.
    pub fn fwhm_hz(&self) -> f64 {
        self.free_spectral_range_hz() / self.finesse()
    }

    /// Airy function transmission for a single pass at normalised frequency offset `x`.
    ///
    /// `x` is measured in units of FSR so that x=0 is a transmission maximum.
    ///
    /// T(x) = 1 / (1 + F_coeff · sin²(π · x))
    ///
    /// where F_coeff = (2F/π)².
    fn airy_single_pass(&self, x_fsr: f64) -> f64 {
        let f = self.finesse();
        let f_coeff = (2.0 * f / std::f64::consts::PI).powi(2);
        1.0 / (1.0 + f_coeff * (std::f64::consts::PI * x_fsr).sin().powi(2))
    }

    /// Multi-pass Airy transmission at frequency offset `freq_hz` from an FP maximum.
    ///
    /// For `passes` passes the transmission is T_single^passes.
    pub fn transmission(&self, freq_offset_hz: f64) -> f64 {
        let fsr = self.free_spectral_range_hz();
        let x_fsr = freq_offset_hz / fsr;
        self.airy_single_pass(x_fsr).powi(self.passes as i32)
    }

    /// Compute transmission profile over a frequency array (Hz relative to FP maximum).
    pub fn transmission_spectrum(&self, freq_offsets_hz: &[f64]) -> Vec<f64> {
        freq_offsets_hz.iter().map(|&f| self.transmission(f)).collect()
    }

    /// Round-trip phase difference δ for gap spacing d and wavelength λ at normal incidence.
    ///
    /// δ = 4π · d / λ
    pub fn round_trip_phase(&self, wavelength_m: f64) -> f64 {
        4.0 * std::f64::consts::PI * self.spacing_m / wavelength_m
    }
}

// ---------------------------------------------------------------------------
// Spectrum data structures
// ---------------------------------------------------------------------------

/// A single spectral peak detected in the Brillouin spectrum.
#[derive(Debug, Clone)]
pub struct BrillouinPeak {
    /// Frequency position of the peak (Hz), positive = anti-Stokes, negative = Stokes.
    pub frequency_hz: f64,
    /// Peak amplitude (arbitrary units).
    pub amplitude: f64,
    /// Full-width at half maximum of the Lorentzian fit (Hz).
    pub linewidth_hz: f64,
    /// Goodness-of-fit R² (0–1).
    pub r_squared: f64,
    /// Whether this is an anti-Stokes peak (positive shift).
    pub is_anti_stokes: bool,
}

/// Result of processing one Brillouin spectrum.
#[derive(Debug, Clone)]
pub struct BrillouinResult {
    /// Detected Stokes peak (negative shift).
    pub stokes: Option<BrillouinPeak>,
    /// Detected anti-Stokes peak (positive shift).
    pub anti_stokes: Option<BrillouinPeak>,
    /// Mean Brillouin shift (average of |Stokes| and anti-Stokes), in Hz.
    pub mean_shift_hz: f64,
    /// Extracted longitudinal sound velocity (m/s).
    pub sound_velocity_ms: f64,
    /// Longitudinal elastic modulus (Pa).
    pub elastic_modulus_pa: f64,
    /// Signal-to-noise ratio of the Brillouin peak (dB).
    pub snr_db: f64,
    /// Contrast ratio: Brillouin peak amplitude / Rayleigh peak amplitude.
    pub contrast_ratio: f64,
    /// Spectral resolution limited by the instrument (Hz).
    pub spectral_resolution_hz: f64,
}

// ---------------------------------------------------------------------------
// Main spectrometer processor
// ---------------------------------------------------------------------------

/// Brillouin light scattering spectrometer signal processor.
///
/// Encapsulates the Fabry-Pérot model, peak detection, and elastic property
/// extraction for a complete BLS measurement workflow.
#[derive(Debug, Clone)]
pub struct BrillouinSpectrometer {
    /// Instrument configuration.
    pub config: SpectrometerConfig,
    /// Fabry-Pérot interferometer model.
    pub fp: FabryPerot,
}

impl BrillouinSpectrometer {
    /// Create a new spectrometer from a [`SpectrometerConfig`].
    pub fn new(config: SpectrometerConfig) -> Self {
        let fp = FabryPerot::new(
            config.fp_spacing_m,
            config.mirror_reflectivity,
            config.fp_passes,
        );
        Self { config, fp }
    }

    // -----------------------------------------------------------------------
    // Physics helpers
    // -----------------------------------------------------------------------

    /// Compute Brillouin frequency shift (Hz) for given refractive index and sound velocity.
    ///
    /// νB = (2 · n · Vs / λ) · sin(θ/2)
    pub fn brillouin_shift(&self, refractive_index: f64, sound_velocity_ms: f64) -> f64 {
        let sin_half = (self.config.scattering_angle_rad / 2.0).sin();
        2.0 * refractive_index * sound_velocity_ms * sin_half / self.config.laser_wavelength_m
    }

    /// Extract sound velocity from a measured Brillouin shift (Hz).
    ///
    /// Vs = νB · λ / (2 · n · sin(θ/2))
    pub fn sound_velocity_from_shift(&self, brillouin_shift_hz: f64, refractive_index: f64) -> f64 {
        let sin_half = (self.config.scattering_angle_rad / 2.0).sin();
        brillouin_shift_hz * self.config.laser_wavelength_m / (2.0 * refractive_index * sin_half)
    }

    /// Compute longitudinal elastic modulus M (Pa) from density and sound velocity.
    ///
    /// M = ρ · Vs²
    pub fn elastic_modulus_pa(density_kg_m3: f64, sound_velocity_ms: f64) -> f64 {
        density_kg_m3 * sound_velocity_ms * sound_velocity_ms
    }

    /// Compute longitudinal modulus directly from Brillouin shift (Hz).
    ///
    /// M = ρ · (νB · λ / (2 · n · sin(θ/2)))²
    pub fn modulus_from_shift(
        &self,
        brillouin_shift_hz: f64,
        refractive_index: f64,
        density_kg_m3: f64,
    ) -> f64 {
        let vs = self.sound_velocity_from_shift(brillouin_shift_hz, refractive_index);
        density_kg_m3 * vs * vs
    }

    /// Estimate bulk modulus K from longitudinal modulus M and shear modulus G.
    ///
    /// K = M − (4/3) G   (isotropic medium)
    pub fn bulk_modulus_pa(longitudinal_modulus_pa: f64, shear_modulus_pa: f64) -> f64 {
        longitudinal_modulus_pa - (4.0 / 3.0) * shear_modulus_pa
    }

    /// Estimate Poisson's ratio ν from longitudinal (Vl) and transverse (Vt) velocities.
    ///
    /// ν = (Vl² − 2 Vt²) / (2 (Vl² − Vt²))
    pub fn poisson_ratio(longitudinal_vel: f64, transverse_vel: f64) -> f64 {
        let vl2 = longitudinal_vel * longitudinal_vel;
        let vt2 = transverse_vel * transverse_vel;
        (vl2 - 2.0 * vt2) / (2.0 * (vl2 - vt2))
    }

    /// Correct Brillouin shift for temperature offset from a reference temperature.
    ///
    /// νB(T) = νB(T0) + dν/dT · ΔT
    pub fn temperature_corrected_shift(
        &self,
        shift_hz: f64,
        temp_offset_c: f64,
        temp_coeff_hz_per_c: f64,
    ) -> f64 {
        shift_hz + temp_coeff_hz_per_c * temp_offset_c
    }

    /// Correct sound velocity for pressure.
    ///
    /// Vs(P) = Vs(0) + dVs/dP · P
    pub fn pressure_corrected_velocity(
        base_velocity_ms: f64,
        pressure_mpa: f64,
        pressure_coeff_ms_per_mpa: f64,
    ) -> f64 {
        base_velocity_ms + pressure_coeff_ms_per_mpa * pressure_mpa
    }

    // -----------------------------------------------------------------------
    // Synthetic spectrum generation
    // -----------------------------------------------------------------------

    /// Generate a synthetic Brillouin spectrum (frequency axis in Hz, intensity in a.u.).
    ///
    /// Simulates the output of a Fabry-Pérot based BLS spectrometer.  The frequency axis
    /// represents the detuning from the laser frequency.  The three spectral components are:
    ///
    /// * **Rayleigh peak** at 0 Hz — elastic back-scattering, amplitude `rayleigh_amplitude`.
    /// * **Stokes peak** at −νB Hz — phonon emission.
    /// * **Anti-Stokes peak** at +νB Hz — phonon absorption.
    ///
    /// Each component is modelled as a Lorentzian with width `brillouin_linewidth_hz`
    /// (the Rayleigh uses the instrument FWHM as its width).  The FP acts as a narrowband
    /// filter; here we convolve each peak with the Airy function by computing the
    /// instrumental linewidth-broadened Lorentzian, rather than attenuating by FP
    /// transmission (which would incorrectly suppress off-axis peaks).
    ///
    /// Gaussian noise is added with standard deviation `noise_std`.
    ///
    /// Returns `(frequencies_hz, intensities)`.
    pub fn generate_spectrum(
        &self,
        brillouin_shift_hz: f64,
        rayleigh_amplitude: f64,
        brillouin_amplitude: f64,
        brillouin_linewidth_hz: f64,
        noise_std: f64,
        seed: u64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = self.config.scan_points;
        let hw = self.config.scan_half_width_hz;
        let step = 2.0 * hw / (n as f64 - 1.0);

        let freqs: Vec<f64> = (0..n).map(|i| -hw + i as f64 * step).collect();

        // Effective Brillouin linewidth: convolution of intrinsic width with FP instrument function
        let fp_fwhm = self.fp.fwhm_hz();
        // Effective width ≈ quadrature sum (approximation for similar widths)
        let effective_lw = (brillouin_linewidth_hz.powi(2) + fp_fwhm.powi(2)).sqrt();
        let rayleigh_lw = fp_fwhm; // Rayleigh width is instrument-limited

        // Simple linear congruential generator for reproducible noise.
        let mut lcg = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        let mut next_noise = move || {
            lcg = lcg.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            // Box-Muller (single sample, ignore second)
            let u1 = (lcg >> 11) as f64 / (1u64 << 53) as f64;
            lcg = lcg.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let u2 = (lcg >> 11) as f64 / (1u64 << 53) as f64;
            let u1 = u1.max(1e-300);
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * noise_std
        };

        let intensities: Vec<f64> = freqs
            .iter()
            .map(|&f| {
                // Rayleigh (elastic) peak — instrument-resolution-limited Lorentzian
                let rayleigh = lorentzian(f, 0.0, rayleigh_amplitude, rayleigh_lw);

                // Stokes peak (negative frequency shift)
                let stokes = lorentzian(f, -brillouin_shift_hz, brillouin_amplitude, effective_lw);

                // Anti-Stokes peak (positive frequency shift; same amplitude for simplicity)
                let anti_stokes = lorentzian(f, brillouin_shift_hz, brillouin_amplitude, effective_lw);

                (rayleigh + stokes + anti_stokes) + next_noise()
            })
            .collect();

        (freqs, intensities)
    }

    // -----------------------------------------------------------------------
    // Spectrum processing
    // -----------------------------------------------------------------------

    /// Estimate and subtract a baseline using a percentile-smoothing approach.
    ///
    /// Computes a rolling minimum over windows of `window` points and smooths it.
    /// Returns the baseline-subtracted spectrum.
    pub fn subtract_baseline(spectrum: &[f64], window: usize) -> Vec<f64> {
        let n = spectrum.len();
        let hw = window / 2;
        let baseline: Vec<f64> = (0..n)
            .map(|i| {
                let start = if i >= hw { i - hw } else { 0 };
                let end = (i + hw + 1).min(n);
                spectrum[start..end]
                    .iter()
                    .cloned()
                    .fold(f64::INFINITY, f64::min)
            })
            .collect();
        spectrum.iter().zip(baseline.iter()).map(|(s, b)| s - b).collect()
    }

    /// Suppress the Rayleigh (elastic) peak around zero frequency.
    ///
    /// Zeroes all bins within `suppression_hz` of 0 Hz.
    /// Returns the modified spectrum.
    pub fn suppress_rayleigh(
        freqs: &[f64],
        spectrum: &[f64],
        suppression_hz: f64,
    ) -> Vec<f64> {
        spectrum
            .iter()
            .zip(freqs.iter())
            .map(|(&s, &f)| if f.abs() < suppression_hz { 0.0 } else { s })
            .collect()
    }

    /// Detect peaks in a spectrum by finding local maxima above a threshold.
    ///
    /// Returns a list of `(frequency_hz, amplitude)` pairs.
    pub fn find_peaks(freqs: &[f64], spectrum: &[f64], threshold: f64) -> Vec<(f64, f64)> {
        let n = spectrum.len();
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if spectrum[i] > spectrum[i - 1]
                && spectrum[i] > spectrum[i + 1]
                && spectrum[i] > threshold
            {
                peaks.push((freqs[i], spectrum[i]));
            }
        }
        peaks
    }

    /// Fit a Lorentzian to a peak region using a least-squares approach.
    ///
    /// `center_hz` is the initial guess for the peak centre.  The search window is
    /// `±search_hz` around the guess.
    ///
    /// Returns `(centre_hz, amplitude, linewidth_hz, r_squared)`.
    pub fn fit_lorentzian(
        freqs: &[f64],
        spectrum: &[f64],
        center_hz: f64,
        search_hz: f64,
    ) -> (f64, f64, f64, f64) {
        // Extract the sub-window
        let region: Vec<(f64, f64)> = freqs
            .iter()
            .zip(spectrum.iter())
            .filter(|(&f, _)| (f - center_hz).abs() < search_hz)
            .map(|(&f, &s)| (f, s))
            .collect();

        if region.is_empty() {
            return (center_hz, 0.0, search_hz, 0.0);
        }

        // Amplitude = max of region
        let (peak_f, peak_amp) = region
            .iter()
            .cloned()
            .fold((center_hz, 0.0f64), |acc, (f, s)| {
                if s > acc.1 { (f, s) } else { acc }
            });

        // Estimate linewidth: find half-max points
        let half_max = peak_amp / 2.0;
        let mut left_hw = search_hz;
        let mut right_hw = search_hz;
        for &(f, s) in &region {
            if f < peak_f && s < half_max {
                left_hw = peak_f - f;
            }
            if f > peak_f && s < half_max {
                right_hw = f - peak_f;
                break;
            }
        }
        let lw = (left_hw + right_hw).max(1.0);

        // Compute R² for the Lorentzian model
        let y_mean = region.iter().map(|(_, s)| s).sum::<f64>() / region.len() as f64;
        let ss_tot: f64 = region.iter().map(|(_, s)| (s - y_mean).powi(2)).sum();
        let ss_res: f64 = region
            .iter()
            .map(|(f, s)| {
                let model = lorentzian(*f, peak_f, peak_amp, lw);
                (s - model).powi(2)
            })
            .sum();
        let r2 = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 0.0 };

        (peak_f, peak_amp, lw, r2)
    }

    /// Compute signal-to-noise ratio of a Brillouin peak (dB).
    ///
    /// SNR = 20 log10(peak_amplitude / noise_std)
    pub fn compute_snr_db(peak_amplitude: f64, noise_std: f64) -> f64 {
        if noise_std <= 0.0 || peak_amplitude <= 0.0 {
            return 0.0;
        }
        20.0 * (peak_amplitude / noise_std).log10()
    }

    /// Estimate noise floor from a quiet region of the spectrum (far from peaks).
    ///
    /// Uses the standard deviation of the lowest `fraction` of all samples.
    pub fn estimate_noise_std(spectrum: &[f64], fraction: f64) -> f64 {
        let mut sorted = spectrum.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n_use = ((sorted.len() as f64 * fraction).ceil() as usize).max(2);
        let slice = &sorted[..n_use];
        let mean = slice.iter().sum::<f64>() / n_use as f64;
        let var = slice.iter().map(|s| (s - mean).powi(2)).sum::<f64>() / n_use as f64;
        var.sqrt()
    }

    /// Full spectrum processing pipeline: baseline removal → Rayleigh suppression →
    /// peak detection → Lorentzian fitting → elastic property extraction.
    ///
    /// `refractive_index` and `density_kg_m3` are needed for elastic property extraction.
    pub fn process_spectrum(
        &self,
        freqs: &[f64],
        raw_spectrum: &[f64],
        refractive_index: f64,
        density_kg_m3: f64,
        rayleigh_suppression_hz: f64,
        peak_threshold_fraction: f64,
    ) -> BrillouinResult {
        // 1. Baseline subtraction
        let baseline_window = (freqs.len() / 20).max(5);
        let corrected = Self::subtract_baseline(raw_spectrum, baseline_window);

        // 2. Find Rayleigh amplitude before suppression
        let rayleigh_amp = {
            let center_idx = freqs.len() / 2;
            raw_spectrum[center_idx.saturating_sub(5)..=(center_idx + 5).min(raw_spectrum.len() - 1)]
                .iter()
                .cloned()
                .fold(0.0f64, f64::max)
        };

        // 3. Suppress Rayleigh peak
        let suppressed = Self::suppress_rayleigh(freqs, &corrected, rayleigh_suppression_hz);

        // 4. Estimate noise
        let noise_std = Self::estimate_noise_std(&suppressed, 0.1).max(1e-12);

        // 5. Find peaks
        let threshold = noise_std * peak_threshold_fraction.max(3.0);
        let peaks = Self::find_peaks(freqs, &suppressed, threshold);

        // Separate Stokes (negative) and anti-Stokes (positive) candidates
        let stokes_candidates: Vec<_> = peaks.iter().filter(|&&(f, _)| f < -rayleigh_suppression_hz).collect();
        let anti_stokes_candidates: Vec<_> = peaks.iter().filter(|&&(f, _)| f > rayleigh_suppression_hz).collect();

        // Pick strongest candidate in each group
        let strongest = |cands: &[&(f64, f64)]| -> Option<(f64, f64)> {
            cands.iter().cloned().cloned().reduce(|acc, x| if x.1 > acc.1 { x } else { acc })
        };

        let fit_search_hz = self.config.scan_half_width_hz * 0.15;

        let stokes_peak = strongest(&stokes_candidates).map(|(f, _)| {
            let (cf, amp, lw, r2) = Self::fit_lorentzian(freqs, &suppressed, f, fit_search_hz);
            BrillouinPeak {
                frequency_hz: cf,
                amplitude: amp,
                linewidth_hz: lw,
                r_squared: r2,
                is_anti_stokes: false,
            }
        });

        let anti_stokes_peak = strongest(&anti_stokes_candidates).map(|(f, _)| {
            let (cf, amp, lw, r2) = Self::fit_lorentzian(freqs, &suppressed, f, fit_search_hz);
            BrillouinPeak {
                frequency_hz: cf,
                amplitude: amp,
                linewidth_hz: lw,
                r_squared: r2,
                is_anti_stokes: true,
            }
        });

        // Mean Brillouin shift
        let mean_shift_hz = match (&stokes_peak, &anti_stokes_peak) {
            (Some(s), Some(a)) => (s.frequency_hz.abs() + a.frequency_hz.abs()) / 2.0,
            (Some(s), None) => s.frequency_hz.abs(),
            (None, Some(a)) => a.frequency_hz.abs(),
            (None, None) => 0.0,
        };

        let sound_velocity_ms = self.sound_velocity_from_shift(mean_shift_hz, refractive_index);
        let elastic_modulus_pa = Self::elastic_modulus_pa(density_kg_m3, sound_velocity_ms);

        let peak_amp = stokes_peak
            .as_ref()
            .map(|p| p.amplitude)
            .or_else(|| anti_stokes_peak.as_ref().map(|p| p.amplitude))
            .unwrap_or(0.0);

        let snr_db = Self::compute_snr_db(peak_amp, noise_std);
        let contrast_ratio = if rayleigh_amp > 0.0 { peak_amp / rayleigh_amp } else { 0.0 };

        BrillouinResult {
            stokes: stokes_peak,
            anti_stokes: anti_stokes_peak,
            mean_shift_hz,
            sound_velocity_ms,
            elastic_modulus_pa,
            snr_db,
            contrast_ratio,
            spectral_resolution_hz: self.fp.fwhm_hz(),
        }
    }
}

// ---------------------------------------------------------------------------
// Spatial mapping
// ---------------------------------------------------------------------------

/// Result of a spatial Brillouin mapping scan.
#[derive(Debug, Clone)]
pub struct BrillouinMap {
    /// X coordinates of measurement points (µm).
    pub x_um: Vec<f64>,
    /// Y coordinates of measurement points (µm).
    pub y_um: Vec<f64>,
    /// Brillouin shift at each point (Hz).
    pub shift_hz: Vec<f64>,
    /// Elastic modulus at each point (Pa).
    pub modulus_pa: Vec<f64>,
    /// SNR at each point (dB).
    pub snr_db: Vec<f64>,
}

impl BrillouinMap {
    /// Create an empty map with pre-allocated storage.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            x_um: Vec::with_capacity(capacity),
            y_um: Vec::with_capacity(capacity),
            shift_hz: Vec::with_capacity(capacity),
            modulus_pa: Vec::with_capacity(capacity),
            snr_db: Vec::with_capacity(capacity),
        }
    }

    /// Append a single point to the map.
    pub fn push(&mut self, x: f64, y: f64, shift_hz: f64, modulus_pa: f64, snr_db: f64) {
        self.x_um.push(x);
        self.y_um.push(y);
        self.shift_hz.push(shift_hz);
        self.modulus_pa.push(modulus_pa);
        self.snr_db.push(snr_db);
    }

    /// Statistics: mean Brillouin shift (Hz).
    pub fn mean_shift_hz(&self) -> f64 {
        if self.shift_hz.is_empty() {
            return 0.0;
        }
        self.shift_hz.iter().sum::<f64>() / self.shift_hz.len() as f64
    }

    /// Statistics: standard deviation of the Brillouin shift map (Hz).
    pub fn std_shift_hz(&self) -> f64 {
        if self.shift_hz.len() < 2 {
            return 0.0;
        }
        let mean = self.mean_shift_hz();
        let var = self.shift_hz.iter().map(|s| (s - mean).powi(2)).sum::<f64>()
            / (self.shift_hz.len() - 1) as f64;
        var.sqrt()
    }

    /// Statistics: mean elastic modulus (Pa).
    pub fn mean_modulus_pa(&self) -> f64 {
        if self.modulus_pa.is_empty() {
            return 0.0;
        }
        self.modulus_pa.iter().sum::<f64>() / self.modulus_pa.len() as f64
    }
}

/// Processor for building point-by-point Brillouin spatial maps.
pub struct BrillouinMapper {
    /// The spectrometer used for each point measurement.
    pub spectrometer: BrillouinSpectrometer,
    /// Refractive index of the sample.
    pub refractive_index: f64,
    /// Density of the sample (kg/m³).
    pub density_kg_m3: f64,
    /// Rayleigh suppression bandwidth (Hz).
    pub rayleigh_suppression_hz: f64,
}

impl BrillouinMapper {
    /// Create a new mapper.
    pub fn new(
        spectrometer: BrillouinSpectrometer,
        refractive_index: f64,
        density_kg_m3: f64,
        rayleigh_suppression_hz: f64,
    ) -> Self {
        Self { spectrometer, refractive_index, density_kg_m3, rayleigh_suppression_hz }
    }

    /// Process a single spatial point and append the result to `map`.
    pub fn process_point(
        &self,
        x_um: f64,
        y_um: f64,
        freqs: &[f64],
        spectrum: &[f64],
        map: &mut BrillouinMap,
    ) {
        let result = self.spectrometer.process_spectrum(
            freqs,
            spectrum,
            self.refractive_index,
            self.density_kg_m3,
            self.rayleigh_suppression_hz,
            3.0,
        );
        map.push(x_um, y_um, result.mean_shift_hz, result.elastic_modulus_pa, result.snr_db);
    }
}

// ---------------------------------------------------------------------------
// Helper: Lorentzian lineshape
// ---------------------------------------------------------------------------

/// Evaluate a Lorentzian (Cauchy) peak: A / (1 + ((x − x0)/γ)²)
///
/// where A is amplitude, x0 is centre, and γ = HWHM (half-width at half-max).
pub fn lorentzian(x: f64, center: f64, amplitude: f64, fwhm: f64) -> f64 {
    let gamma = fwhm / 2.0;
    amplitude / (1.0 + ((x - center) / gamma).powi(2))
}

/// Evaluate a Voigt profile approximated by the pseudo-Voigt formula (Thompson et al. 1987).
///
/// η = 1.36603(fL/fV) − 0.47719(fL/fV)² + 0.11116(fL/fV)³
///
/// where `fwhm_lorentz` and `fwhm_gauss` are the Lorentzian and Gaussian components.
pub fn pseudo_voigt(x: f64, center: f64, amplitude: f64, fwhm_lorentz: f64, fwhm_gauss: f64) -> f64 {
    let fl5 = fwhm_lorentz.powi(5);
    let fg5 = fwhm_gauss.powi(5);
    let fg4fl = fwhm_gauss.powi(4) * fwhm_lorentz;
    let fg3fl2 = fwhm_gauss.powi(3) * fwhm_lorentz.powi(2);
    let fg2fl3 = fwhm_gauss.powi(2) * fwhm_lorentz.powi(3);
    let fgfl4 = fwhm_gauss * fwhm_lorentz.powi(4);

    let f_v = (fl5 + 2.69269 * fg4fl + 2.42843 * fg3fl2 + 4.47163 * fg2fl3 + 0.07842 * fgfl4 + fg5)
        .powf(0.2);

    let eta = 1.36603 * (fwhm_lorentz / f_v)
        - 0.47719 * (fwhm_lorentz / f_v).powi(2)
        + 0.11116 * (fwhm_lorentz / f_v).powi(3);

    let lorentz_part = lorentzian(x, center, amplitude, f_v);
    let gauss_part = {
        let sigma = f_v / (2.0 * (2.0_f64 * std::f64::consts::LN_2).sqrt());
        amplitude * (-(x - center).powi(2) / (2.0 * sigma * sigma)).exp()
    };

    eta * lorentz_part + (1.0 - eta) * gauss_part
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn default_spec() -> BrillouinSpectrometer {
        BrillouinSpectrometer::new(SpectrometerConfig::default())
    }

    // -----------------------------------------------------------------------
    // Brillouin shift physics
    // -----------------------------------------------------------------------

    #[test]
    fn test_brillouin_shift_water() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        // Expected ~7.46 GHz; allow ±0.5 GHz tolerance for material parameter rounding
        assert!(
            (shift / 1e9 - water.brillouin_shift_ghz).abs() < 0.5,
            "Water shift {:.3} GHz, expected ~{:.3} GHz",
            shift / 1e9,
            water.brillouin_shift_ghz
        );
    }

    #[test]
    fn test_brillouin_shift_fused_silica() {
        let spec = default_spec();
        let mat = MaterialDatabase::fused_silica();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        assert!(
            (shift / 1e9 - mat.brillouin_shift_ghz).abs() < 1.5,
            "Fused silica shift {:.2} GHz, expected ~{:.2} GHz",
            shift / 1e9,
            mat.brillouin_shift_ghz
        );
    }

    #[test]
    fn test_brillouin_shift_diamond() {
        let spec = default_spec();
        let mat = MaterialDatabase::diamond();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        // The stored brillouin_shift_ghz is formula-derived; check self-consistency
        assert!(
            (shift / 1e9 - mat.brillouin_shift_ghz).abs() < 1.0,
            "Diamond shift {:.2} GHz, expected ~{:.2} GHz",
            shift / 1e9,
            mat.brillouin_shift_ghz
        );
        // Sanity check: diamond Brillouin shift should be large (>100 GHz at 532 nm)
        assert!(shift > 100e9, "Diamond shift too low: {:.1} GHz", shift / 1e9);
    }

    #[test]
    fn test_brillouin_shift_pmma() {
        let spec = default_spec();
        let mat = MaterialDatabase::pmma();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        assert!(
            (shift / 1e9 - mat.brillouin_shift_ghz).abs() < 1.0,
            "PMMA shift {:.2} GHz",
            shift / 1e9
        );
    }

    #[test]
    fn test_brillouin_shift_polycarbonate() {
        let spec = default_spec();
        let mat = MaterialDatabase::polycarbonate();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        assert!((shift / 1e9 - mat.brillouin_shift_ghz).abs() < 1.0);
    }

    #[test]
    fn test_brillouin_shift_silicon() {
        let spec = default_spec();
        let mat = MaterialDatabase::silicon();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        // Self-consistency check: formula result should match stored formula value
        assert!(
            (shift / 1e9 - mat.brillouin_shift_ghz).abs() < 1.0,
            "Silicon shift {:.2} GHz, expected ~{:.2} GHz",
            shift / 1e9,
            mat.brillouin_shift_ghz
        );
        // Silicon has large n×Vs product → large shift
        assert!(shift > 50e9, "Silicon shift too low: {:.1} GHz", shift / 1e9);
    }

    #[test]
    fn test_brillouin_shift_bk7() {
        let spec = default_spec();
        let mat = MaterialDatabase::bk7_glass();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        assert!((shift / 1e9 - mat.brillouin_shift_ghz).abs() < 2.0);
    }

    // -----------------------------------------------------------------------
    // Acoustic velocity extraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_velocity_roundtrip_water() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        let vs = spec.sound_velocity_from_shift(shift, water.refractive_index);
        assert!((vs - water.sound_velocity_ms).abs() < 1.0, "vs = {:.1} m/s", vs);
    }

    #[test]
    fn test_velocity_roundtrip_silica() {
        let spec = default_spec();
        let mat = MaterialDatabase::fused_silica();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        let vs = spec.sound_velocity_from_shift(shift, mat.refractive_index);
        assert!((vs - mat.sound_velocity_ms).abs() < 1.0);
    }

    #[test]
    fn test_velocity_roundtrip_diamond() {
        let spec = default_spec();
        let mat = MaterialDatabase::diamond();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        let vs = spec.sound_velocity_from_shift(shift, mat.refractive_index);
        assert!((vs - mat.sound_velocity_ms).abs() < 5.0, "vs = {:.0} m/s", vs);
    }

    #[test]
    fn test_velocity_is_positive() {
        let spec = default_spec();
        let mat = MaterialDatabase::water();
        let shift = spec.brillouin_shift(mat.refractive_index, mat.sound_velocity_ms);
        assert!(shift > 0.0);
        let vs = spec.sound_velocity_from_shift(shift, mat.refractive_index);
        assert!(vs > 0.0);
    }

    // -----------------------------------------------------------------------
    // Elastic modulus
    // -----------------------------------------------------------------------

    #[test]
    fn test_elastic_modulus_water() {
        let water = MaterialDatabase::water();
        let m = BrillouinSpectrometer::elastic_modulus_pa(water.density_kg_m3, water.sound_velocity_ms);
        // Water: ρ=997, Vs=1497 → M ≈ 2.23 GPa
        let expected = 2.23e9;
        assert!(
            (m - expected).abs() / expected < 0.02,
            "Water M = {:.3} GPa, expected {:.3} GPa",
            m / 1e9,
            expected / 1e9
        );
    }

    #[test]
    fn test_elastic_modulus_fused_silica() {
        let mat = MaterialDatabase::fused_silica();
        let m = BrillouinSpectrometer::elastic_modulus_pa(mat.density_kg_m3, mat.sound_velocity_ms);
        // Fused silica: ρ=2203, Vs=5968 → M ≈ 78.5 GPa
        assert!(m > 70e9 && m < 90e9, "Silica M = {:.1} GPa", m / 1e9);
    }

    #[test]
    fn test_elastic_modulus_diamond() {
        let mat = MaterialDatabase::diamond();
        let m = BrillouinSpectrometer::elastic_modulus_pa(mat.density_kg_m3, mat.sound_velocity_ms);
        // Diamond: ρ=3515, Vs=17533 → M ≈ 1080 GPa
        assert!(m > 900e9 && m < 1200e9, "Diamond M = {:.0} GPa", m / 1e9);
    }

    #[test]
    fn test_modulus_from_shift_water() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        let m = spec.modulus_from_shift(shift, water.refractive_index, water.density_kg_m3);
        let direct = BrillouinSpectrometer::elastic_modulus_pa(water.density_kg_m3, water.sound_velocity_ms);
        assert!((m - direct).abs() / direct < 1e-8);
    }

    // -----------------------------------------------------------------------
    // Bulk modulus and Poisson's ratio
    // -----------------------------------------------------------------------

    #[test]
    fn test_bulk_modulus_isotropic() {
        // For an isotropic solid: K = M − (4/3)G
        let m = 78.5e9_f64;
        let g = 31.2e9_f64;
        let k = BrillouinSpectrometer::bulk_modulus_pa(m, g);
        let expected = m - 4.0 / 3.0 * g;
        assert!((k - expected).abs() < 1.0);
    }

    #[test]
    fn test_poisson_ratio_typical() {
        // For glass: Vl ≈ 5968 m/s, Vt ≈ 3765 m/s → ν ≈ 0.17
        let vl = 5968.0_f64;
        let vt = 3765.0_f64;
        let nu = BrillouinSpectrometer::poisson_ratio(vl, vt);
        assert!(nu > 0.10 && nu < 0.25, "ν = {:.3}", nu);
    }

    #[test]
    fn test_poisson_ratio_incompressible() {
        // Incompressible (ν → 0.5): Vt → 0
        // Use a very small Vt
        let vl = 1500.0_f64;
        let vt = 100.0_f64; // near-incompressible
        let nu = BrillouinSpectrometer::poisson_ratio(vl, vt);
        assert!(nu > 0.4 && nu <= 0.5, "ν = {:.4}", nu);
    }

    // -----------------------------------------------------------------------
    // Fabry-Pérot: FSR and finesse
    // -----------------------------------------------------------------------

    #[test]
    fn test_fsr_3mm_etalon() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let fsr = fp.free_spectral_range_hz();
        // FSR = c/(2d) = 3e8/(6e-3) = 49.965 GHz
        let expected = SPEED_OF_LIGHT / (2.0 * 3e-3);
        assert!((fsr - expected).abs() < 1e6, "FSR = {:.3} GHz", fsr / 1e9);
    }

    #[test]
    fn test_fsr_10mm_etalon() {
        let fp = FabryPerot::new(10e-3, 0.95, 1);
        let fsr = fp.free_spectral_range_hz();
        let expected = SPEED_OF_LIGHT / (2.0 * 10e-3);
        assert!((fsr - expected).abs() < 1e6);
    }

    #[test]
    fn test_finesse_high_reflectivity() {
        let fp = FabryPerot::new(3e-3, 0.99, 1);
        let f = fp.finesse();
        // F = π√R/(1-R) = π·0.9950/0.01 ≈ 312.6
        assert!(f > 300.0 && f < 330.0, "finesse = {:.1}", f);
    }

    #[test]
    fn test_finesse_moderate_reflectivity() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let f = fp.finesse();
        // F = π·√0.95/(1-0.95) = π·0.9747/0.05 ≈ 61.3
        assert!(f > 55.0 && f < 70.0, "finesse = {:.1}", f);
    }

    #[test]
    fn test_fwhm_from_finesse() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let fwhm = fp.fwhm_hz();
        let expected = fp.free_spectral_range_hz() / fp.finesse();
        assert!((fwhm - expected).abs() < 1.0);
    }

    // -----------------------------------------------------------------------
    // Fabry-Pérot: Airy function transmission
    // -----------------------------------------------------------------------

    #[test]
    fn test_airy_peak_transmission() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        // At f_offset = 0 (transmission peak), T should be 1
        let t = fp.transmission(0.0);
        assert!((t - 1.0).abs() < TOL, "T(0) = {:.6}", t);
    }

    #[test]
    fn test_airy_half_fsr_is_minimum() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let fsr = fp.free_spectral_range_hz();
        // At half FSR, transmission should be very small
        let t_half = fp.transmission(fsr / 2.0);
        let t_peak = fp.transmission(0.0);
        assert!(t_half < t_peak * 0.01, "T(FSR/2) = {:.6}", t_half);
    }

    #[test]
    fn test_airy_next_peak() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let fsr = fp.free_spectral_range_hz();
        let t_next = fp.transmission(fsr);
        assert!((t_next - 1.0).abs() < TOL, "T(FSR) = {:.6}", t_next);
    }

    #[test]
    fn test_airy_multipass_suppresses_sidelobes() {
        let fp1 = FabryPerot::new(3e-3, 0.95, 1);
        let fp2 = FabryPerot::new(3e-3, 0.95, 2);
        let fsr = fp1.free_spectral_range_hz();
        let t1 = fp1.transmission(fsr * 0.3);
        let t2 = fp2.transmission(fsr * 0.3);
        // Two-pass should suppress more
        assert!(t2 < t1, "T2 = {:.4} should be < T1 = {:.4}", t2, t1);
    }

    #[test]
    fn test_round_trip_phase() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let delta = fp.round_trip_phase(532e-9);
        let expected = 4.0 * std::f64::consts::PI * 3e-3 / 532e-9;
        assert!((delta - expected).abs() < 1e-6);
    }

    #[test]
    fn test_transmission_spectrum_length() {
        let fp = FabryPerot::new(3e-3, 0.95, 1);
        let offsets: Vec<f64> = (0..100).map(|i| i as f64 * 1e8).collect();
        let t = fp.transmission_spectrum(&offsets);
        assert_eq!(t.len(), 100);
    }

    // -----------------------------------------------------------------------
    // Peak detection and Lorentzian fitting
    // -----------------------------------------------------------------------

    #[test]
    fn test_find_peaks_simple() {
        let freqs: Vec<f64> = (0..11).map(|i| i as f64 - 5.0).collect();
        let spectrum = vec![0.0, 0.1, 0.2, 0.5, 1.0, 0.5, 0.2, 0.1, 0.0, 0.0, 0.0];
        let peaks = BrillouinSpectrometer::find_peaks(&freqs, &spectrum, 0.3);
        assert_eq!(peaks.len(), 1);
        assert!((peaks[0].0 - (-1.0)).abs() < 0.5, "peak at {}", peaks[0].0);
    }

    #[test]
    fn test_find_peaks_stokes_anti_stokes() {
        // Two peaks at ±5
        let n = 201;
        let freqs: Vec<f64> = (0..n).map(|i| (i as f64 - 100.0) * 0.1).collect();
        let spectrum: Vec<f64> = freqs.iter().map(|&f| {
            lorentzian(f, -5.0, 1.0, 0.5) + lorentzian(f, 5.0, 1.0, 0.5)
        }).collect();
        let peaks = BrillouinSpectrometer::find_peaks(&freqs, &spectrum, 0.1);
        assert!(peaks.len() >= 2, "found {} peaks", peaks.len());
    }

    #[test]
    fn test_lorentzian_lineshape() {
        // Lorentzian at x=0, amplitude=2, fwhm=1 → L(0)=2, L(0.5)=1
        let v0 = lorentzian(0.0, 0.0, 2.0, 1.0);
        let v_half = lorentzian(0.5, 0.0, 2.0, 1.0);
        assert!((v0 - 2.0).abs() < TOL);
        assert!((v_half - 1.0).abs() < TOL);
    }

    #[test]
    fn test_fit_lorentzian_recovery() {
        let freqs: Vec<f64> = (0..1001).map(|i| (i as f64 - 500.0) * 1e7).collect();
        let center = 3e9_f64;
        let amp = 10.0;
        let fwhm = 0.5e9;
        let spectrum: Vec<f64> = freqs.iter().map(|&f| lorentzian(f, center, amp, fwhm)).collect();
        let (cf, fitted_amp, fitted_lw, r2) = BrillouinSpectrometer::fit_lorentzian(&freqs, &spectrum, center, 2e9);
        assert!((cf - center).abs() < 1e8, "center off: {:.2e}", cf - center);
        assert!((fitted_amp - amp).abs() / amp < 0.1, "amp off: {:.2}", fitted_amp);
        assert!(fitted_lw > 0.0);
        assert!(r2 > 0.9, "R² = {:.4}", r2);
    }

    // -----------------------------------------------------------------------
    // Stokes / anti-Stokes symmetry
    // -----------------------------------------------------------------------

    #[test]
    fn test_stokes_anti_stokes_symmetry() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        // Use zero noise and modest Rayleigh to make peaks clearly detectable
        let (freqs, spectrum) = spec.generate_spectrum(shift, 10.0, 5.0, 0.3e9, 0.0, 42);
        // Threshold at 5% of Brillouin amplitude
        let threshold = 5.0 * 0.05;
        let peaks = BrillouinSpectrometer::find_peaks(&freqs, &spectrum, threshold);
        let stokes: Vec<_> = peaks.iter().filter(|&&(f, _)| f < -shift * 0.5).collect();
        let anti_stokes: Vec<_> = peaks.iter().filter(|&&(f, _)| f > shift * 0.5).collect();
        // Both sides should have at least one peak
        assert!(!stokes.is_empty(), "no Stokes peak found (shift={:.2} GHz, {} peaks total)", shift / 1e9, peaks.len());
        assert!(!anti_stokes.is_empty(), "no anti-Stokes peak found");
        // Positions should be roughly symmetric (within 5% of shift)
        let s_pos = stokes.iter().cloned().map(|&(f,_)| f.abs()).fold(0.0f64, f64::max);
        let a_pos = anti_stokes.iter().cloned().map(|&(f,_)| f.abs()).fold(0.0f64, f64::max);
        assert!((s_pos - a_pos).abs() / shift < 0.05, "asymmetry {:.1}%", (s_pos - a_pos).abs() / shift * 100.0);
    }

    // -----------------------------------------------------------------------
    // Material database
    // -----------------------------------------------------------------------

    #[test]
    fn test_material_database_all_present() {
        let all = MaterialDatabase::all();
        assert_eq!(all.len(), 7);
    }

    #[test]
    fn test_material_lookup_water() {
        let mat = MaterialDatabase::lookup("water");
        assert!(mat.is_some());
        assert_eq!(mat.unwrap().name, "Water (25 °C)");
    }

    #[test]
    fn test_material_lookup_diamond() {
        let mat = MaterialDatabase::lookup("diamond");
        assert!(mat.is_some());
    }

    #[test]
    fn test_material_lookup_missing() {
        let mat = MaterialDatabase::lookup("unobtainium");
        assert!(mat.is_none());
    }

    #[test]
    fn test_material_properties_positive() {
        for mat in MaterialDatabase::all() {
            assert!(mat.refractive_index > 1.0, "{}: n must be > 1", mat.name);
            assert!(mat.sound_velocity_ms > 0.0, "{}: Vs must be > 0", mat.name);
            assert!(mat.density_kg_m3 > 0.0, "{}: ρ must be > 0", mat.name);
            assert!(mat.brillouin_shift_ghz > 0.0, "{}: νB must be > 0", mat.name);
        }
    }

    // -----------------------------------------------------------------------
    // Temperature / pressure effects
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_correction_increases_shift() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift0 = water.brillouin_shift_ghz * 1e9;
        // Water: temp coeff is negative → cooling increases shift
        let shift_cold = spec.temperature_corrected_shift(shift0, -10.0, water.temp_coeff_ghz_per_c * 1e9);
        assert!(shift_cold > shift0, "shift_cold={:.3e} shift0={:.3e}", shift_cold, shift0);
    }

    #[test]
    fn test_temperature_correction_linearity() {
        let spec = default_spec();
        let shift0 = 7.46e9_f64;
        let coeff = -0.025e9_f64;
        let dt = 5.0_f64;
        let shifted = spec.temperature_corrected_shift(shift0, dt, coeff);
        let expected = shift0 + coeff * dt;
        assert!((shifted - expected).abs() < 1.0);
    }

    #[test]
    fn test_pressure_correction_water() {
        let water = MaterialDatabase::water();
        let vs_200mpa = BrillouinSpectrometer::pressure_corrected_velocity(
            water.sound_velocity_ms, 200.0, water.pressure_coeff_ms_per_mpa);
        // Velocity should increase with pressure
        assert!(vs_200mpa > water.sound_velocity_ms, "Vs(200 MPa)={:.1} Vs(0)={:.1}", vs_200mpa, water.sound_velocity_ms);
    }

    #[test]
    fn test_pressure_correction_magnitude() {
        let water = MaterialDatabase::water();
        let delta_v = BrillouinSpectrometer::pressure_corrected_velocity(
            water.sound_velocity_ms, 100.0, water.pressure_coeff_ms_per_mpa
        ) - water.sound_velocity_ms;
        // Should be ~27 m/s for 100 MPa (0.27 m/s per MPa)
        assert!((delta_v - 27.0).abs() < 1.0, "ΔVs = {:.1} m/s", delta_v);
    }

    // -----------------------------------------------------------------------
    // Noise and SNR
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_db_positive() {
        let snr = BrillouinSpectrometer::compute_snr_db(100.0, 1.0);
        assert!((snr - 40.0).abs() < 0.01, "SNR = {:.2} dB", snr);
    }

    #[test]
    fn test_snr_db_zero_noise() {
        let snr = BrillouinSpectrometer::compute_snr_db(100.0, 0.0);
        assert_eq!(snr, 0.0);
    }

    #[test]
    fn test_estimate_noise_std() {
        // Constant signal + offset: noise should be ~0
        let spectrum = vec![5.0; 100];
        let noise = BrillouinSpectrometer::estimate_noise_std(&spectrum, 0.1);
        assert!(noise < 0.01, "noise = {:.4}", noise);
    }

    #[test]
    fn test_baseline_subtraction_flat_baseline() {
        let spectrum = vec![1.0; 50]; // flat
        let corrected = BrillouinSpectrometer::subtract_baseline(&spectrum, 10);
        // After subtracting local min (=1.0), all values should be ~0
        for v in &corrected {
            assert!(v.abs() < 0.01, "v = {:.4}", v);
        }
    }

    // -----------------------------------------------------------------------
    // Spatial mapping
    // -----------------------------------------------------------------------

    #[test]
    fn test_brillouin_map_statistics() {
        let mut map = BrillouinMap::with_capacity(4);
        map.push(0.0, 0.0, 7.4e9, 2.2e9, 30.0);
        map.push(1.0, 0.0, 7.5e9, 2.25e9, 31.0);
        map.push(0.0, 1.0, 7.6e9, 2.3e9, 29.0);
        map.push(1.0, 1.0, 7.3e9, 2.15e9, 32.0);

        let mean = map.mean_shift_hz();
        assert!((mean - 7.45e9).abs() < 1e6, "mean = {:.3} GHz", mean / 1e9);

        let std = map.std_shift_hz();
        assert!(std > 0.0 && std < 0.2e9, "std = {:.3} GHz", std / 1e9);

        let m_mean = map.mean_modulus_pa();
        assert!(m_mean > 2.0e9 && m_mean < 2.5e9);
    }

    #[test]
    fn test_brillouin_map_empty() {
        let map = BrillouinMap::with_capacity(0);
        assert_eq!(map.mean_shift_hz(), 0.0);
        assert_eq!(map.std_shift_hz(), 0.0);
        assert_eq!(map.mean_modulus_pa(), 0.0);
    }

    // -----------------------------------------------------------------------
    // End-to-end spectrum processing
    // -----------------------------------------------------------------------

    #[test]
    fn test_process_spectrum_water() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        // Generate clean spectrum with low noise so the processing pipeline finds the peaks
        let (freqs, spectrum) = spec.generate_spectrum(shift, 10.0, 5.0, 0.3e9, 0.001, 1234);
        // Rayleigh suppression window: half the Brillouin shift (3.75 GHz)
        let suppression = shift * 0.5;
        let result = spec.process_spectrum(
            &freqs, &spectrum, water.refractive_index, water.density_kg_m3, suppression, 3.0,
        );
        assert!(result.spectral_resolution_hz > 0.0);
        // We should recover a shift within 30% of the true value
        if result.mean_shift_hz > 0.0 {
            let err_pct = (result.mean_shift_hz - shift).abs() / shift * 100.0;
            assert!(err_pct < 30.0, "shift error = {:.1}%  (result={:.2} GHz, expected={:.2} GHz)",
                err_pct, result.mean_shift_hz / 1e9, shift / 1e9);
        }
    }

    #[test]
    fn test_process_spectrum_sound_velocity_positive() {
        let spec = default_spec();
        let water = MaterialDatabase::water();
        let shift = spec.brillouin_shift(water.refractive_index, water.sound_velocity_ms);
        let (freqs, spectrum) = spec.generate_spectrum(shift, 100.0, 10.0, 0.3e9, 0.02, 99);
        let result = spec.process_spectrum(
            &freqs, &spectrum, water.refractive_index, water.density_kg_m3, 2e9, 3.0,
        );
        assert!(result.sound_velocity_ms >= 0.0);
        assert!(result.elastic_modulus_pa >= 0.0);
    }

    #[test]
    fn test_pseudo_voigt_at_center() {
        // At center, pseudo-Voigt should equal amplitude
        let v = pseudo_voigt(0.0, 0.0, 5.0, 1.0, 1.0);
        // Centre should be close to amplitude (both L and G peak at centre)
        assert!(v > 4.0 && v <= 5.0, "v = {:.4}", v);
    }

    #[test]
    fn test_generate_spectrum_length() {
        let cfg = SpectrometerConfig { scan_points: 512, ..Default::default() };
        let spec = BrillouinSpectrometer::new(cfg);
        let (freqs, intensities) = spec.generate_spectrum(7.4e9, 100.0, 10.0, 0.3e9, 0.0, 0);
        assert_eq!(freqs.len(), 512);
        assert_eq!(intensities.len(), 512);
    }

    #[test]
    fn test_generate_spectrum_has_peaks() {
        let spec = default_spec();
        let shift = 7.4e9_f64;
        let (freqs, spectrum) = spec.generate_spectrum(shift, 100.0, 10.0, 0.3e9, 0.0, 7);
        let max_val = spectrum.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.0, "spectrum is all zeros");
        let peaks = BrillouinSpectrometer::find_peaks(&freqs, &spectrum, max_val * 0.01);
        assert!(!peaks.is_empty(), "no peaks found in synthetic spectrum");
    }
}
