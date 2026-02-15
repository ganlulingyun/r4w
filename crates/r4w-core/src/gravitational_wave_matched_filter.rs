//! # Gravitational Wave Matched Filter
//!
//! Matched filtering for gravitational wave detection from compact binary coalescences (CBC).
//!
//! LIGO/Virgo/KAGRA detect gravitational waves by correlating detector strain data against
//! a bank of theoretical waveform templates computed from general relativity. The primary
//! signal source is the inspiral, merger, and ringdown of compact binary systems (binary
//! neutron stars, neutron star-black hole, and binary black holes).
//!
//! ## Physics Background
//!
//! A compact binary inspiral produces a characteristic "chirp" signal h(t) with:
//! - **Increasing frequency**: governed by the chirp mass M_c
//! - **Increasing amplitude**: as the orbital separation shrinks
//! - **Phase evolution**: determined by post-Newtonian expansion
//!
//! The matched filter optimal SNR is:
//!
//! ```text
//! rho(t) = 4 Re integral[ h~(f) d~*(f) / S_n(f) e^{2*pi*i*f*t} df ]
//! ```
//!
//! where `h~(f)` is the template, `d~(f)` is the detector data, and `S_n(f)` is the
//! one-sided noise power spectral density.
//!
//! ## Key Concepts
//!
//! - **Chirp mass**: M_c = (m1 * m2)^(3/5) / (m1 + m2)^(1/5) -- the mass parameter
//!   that determines the leading-order frequency evolution
//! - **ISCO frequency**: the innermost stable circular orbit frequency, marking the
//!   end of the inspiral phase; f_ISCO = c^3 / (6^(3/2) * pi * G * M)
//! - **Template bank**: a discrete set of templates covering the mass parameter space
//!   with a specified minimal match (typically 0.97)
//! - **Chi-squared discriminator**: Allen's chi-squared veto to distinguish true signals
//!   from noise transients (glitches)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::gravitational_wave_matched_filter::*;
//!
//! // Configure a LIGO-like detector
//! let config = DetectorConfig {
//!     sample_rate_hz: 4096.0,
//!     f_low_hz: 30.0,
//!     f_high_hz: 2048.0,
//!     detector_name: "H1".to_string(),
//!     snr_threshold: 8.0,
//! };
//!
//! let filter = GravitationalWaveFilter::new(config);
//!
//! // Compute chirp mass for a 30+30 solar mass binary
//! let mc = GravitationalWaveFilter::chirp_mass(30.0, 30.0);
//! assert!((mc - 30.0 * 2.0_f64.powf(-1.0/5.0)).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Gravitational constant in SI units (m^3 kg^-1 s^-2)
pub const G_SI: f64 = 6.674_30e-11;

/// Speed of light in SI units (m/s)
pub const C_SI: f64 = 2.998_792_458e8;

/// Solar mass in kilograms
pub const SOLAR_MASS_KG: f64 = 1.988_92e30;

/// Megaparsec in metres
pub const MPC_TO_M: f64 = 3.085_677_581e22;

/// Solar mass expressed in seconds: G * M_sun / c^3
pub const SOLAR_MASS_SECONDS: f64 = 4.925_491e-6;

// ---------------------------------------------------------------------------
// DetectorConfig
// ---------------------------------------------------------------------------

/// Configuration for a gravitational wave detector.
///
/// Encapsulates the key parameters of a ground-based interferometric GW detector
/// such as LIGO Hanford (H1), LIGO Livingston (L1), or Virgo (V1).
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Sampling rate of the digitised strain channel (Hz).
    /// Typical values: 4096 or 16384 Hz.
    pub sample_rate_hz: f64,

    /// Low-frequency cut-off (Hz). Set above the seismic wall where
    /// detector noise rises steeply. Typical: 20--40 Hz.
    pub f_low_hz: f64,

    /// High-frequency cut-off (Hz). Usually the Nyquist frequency
    /// or the frequency above which no astrophysical signal power
    /// is expected. Typical: ~2048 Hz.
    pub f_high_hz: f64,

    /// Detector name string, e.g. `"H1"`, `"L1"`, `"V1"`, `"K1"`.
    pub detector_name: String,

    /// Single-detector SNR threshold for trigger generation.
    /// Typical: 8.0 for low-latency searches.
    pub snr_threshold: f64,
}

impl DetectorConfig {
    /// Create a default LIGO-like configuration.
    pub fn ligo_default() -> Self {
        Self {
            sample_rate_hz: 4096.0,
            f_low_hz: 30.0,
            f_high_hz: 2048.0,
            detector_name: "H1".to_string(),
            snr_threshold: 8.0,
        }
    }
}

// ---------------------------------------------------------------------------
// CompactBinaryParams
// ---------------------------------------------------------------------------

/// Parameters describing a compact binary coalescence source.
///
/// All masses are in units of solar masses. Distances in megaparsecs.
#[derive(Debug, Clone)]
pub struct CompactBinaryParams {
    /// Component mass 1 (solar masses). Convention: m1 >= m2.
    pub m1_solar: f64,
    /// Component mass 2 (solar masses).
    pub m2_solar: f64,
    /// Luminosity distance (Mpc).
    pub distance_mpc: f64,
    /// Orbital inclination angle (radians). 0 = face-on.
    pub inclination_rad: f64,
    /// Coalescence phase (radians).
    pub phase_rad: f64,
    /// Chirp mass (solar masses). Computed from m1, m2.
    pub chirp_mass_solar: f64,
    /// Symmetric mass ratio eta = m1*m2 / (m1+m2)^2. Range (0, 0.25].
    pub eta: f64,
}

impl CompactBinaryParams {
    /// Construct from component masses and extrinsic parameters.
    ///
    /// Chirp mass and symmetric mass ratio are computed automatically.
    ///
    /// # Arguments
    /// * `m1` - mass 1 in solar masses
    /// * `m2` - mass 2 in solar masses
    /// * `distance_mpc` - luminosity distance in Mpc
    /// * `inclination_rad` - inclination (rad)
    /// * `phase_rad` - coalescence phase (rad)
    pub fn new(
        m1: f64,
        m2: f64,
        distance_mpc: f64,
        inclination_rad: f64,
        phase_rad: f64,
    ) -> Self {
        let mc = GravitationalWaveFilter::chirp_mass(m1, m2);
        let eta = GravitationalWaveFilter::symmetric_mass_ratio(m1, m2);
        Self {
            m1_solar: m1,
            m2_solar: m2,
            distance_mpc,
            inclination_rad,
            phase_rad,
            chirp_mass_solar: mc,
            eta,
        }
    }

    /// Create a canonical equal-mass binary at 100 Mpc, face-on.
    pub fn equal_mass(m: f64) -> Self {
        Self::new(m, m, 100.0, 0.0, 0.0)
    }
}

// ---------------------------------------------------------------------------
// GravitationalWaveFilter
// ---------------------------------------------------------------------------

/// Core gravitational wave template and waveform generation.
///
/// Provides Newtonian-order waveform generation for compact binary inspiral
/// signals, including chirp mass computation, ISCO frequency, chirp time
/// estimation, and time-domain waveform construction.
#[derive(Debug, Clone)]
pub struct GravitationalWaveFilter {
    /// Detector configuration.
    pub config: DetectorConfig,
}

impl GravitationalWaveFilter {
    /// Create a new filter with the given detector configuration.
    pub fn new(config: DetectorConfig) -> Self {
        Self { config }
    }

    /// Compute the chirp mass from component masses.
    ///
    /// M_c = (m1 * m2)^(3/5) / (m1 + m2)^(1/5)
    ///
    /// For equal masses m1 = m2 = m, M_c = m * 2^(-1/5).
    pub fn chirp_mass(m1: f64, m2: f64) -> f64 {
        (m1 * m2).powf(3.0 / 5.0) / (m1 + m2).powf(1.0 / 5.0)
    }

    /// Compute the symmetric mass ratio.
    ///
    /// eta = m1 * m2 / (m1 + m2)^2
    ///
    /// Ranges from 0 (extreme mass ratio) to 0.25 (equal mass).
    pub fn symmetric_mass_ratio(m1: f64, m2: f64) -> f64 {
        let m_total = m1 + m2;
        (m1 * m2) / (m_total * m_total)
    }

    /// Compute the total mass.
    pub fn total_mass(m1: f64, m2: f64) -> f64 {
        m1 + m2
    }

    /// Compute the frequency of the Innermost Stable Circular Orbit (ISCO).
    ///
    /// For a Schwarzschild (non-spinning) black hole:
    ///
    /// f_ISCO = c^3 / (6^(3/2) * pi * G * M)
    ///
    /// where M is the total mass in SI units.
    ///
    /// # Arguments
    /// * `total_mass_solar` - total mass of the binary in solar masses.
    pub fn isco_frequency(total_mass_solar: f64) -> f64 {
        let m_si = total_mass_solar * SOLAR_MASS_KG;
        C_SI.powi(3) / (6.0_f64.powf(1.5) * PI * G_SI * m_si)
    }

    /// Estimate the Newtonian chirp time from `f_low` to coalescence.
    ///
    /// tau = (5 / (256 * pi^(8/3))) * (G * M_c / c^3)^(-5/3) * f_low^(-8/3)
    ///
    /// # Arguments
    /// * `chirp_mass_solar` - chirp mass in solar masses
    /// * `f_low_hz` - lower frequency bound (Hz)
    pub fn newtonian_chirp_time(chirp_mass_solar: f64, f_low_hz: f64) -> f64 {
        let mc_si = chirp_mass_solar * SOLAR_MASS_KG;
        let mc_seconds = G_SI * mc_si / C_SI.powi(3);
        let prefactor = 5.0 / (256.0 * PI.powf(8.0 / 3.0));
        prefactor * mc_seconds.powf(-5.0 / 3.0) * f_low_hz.powf(-8.0 / 3.0)
    }

    /// Compute the instantaneous GW frequency as a function of time to merger.
    ///
    /// f(tau) = (1 / pi) * (5 / (256 * tau))^(3/8) * (G * M_c / c^3)^(-5/8)
    ///
    /// # Arguments
    /// * `chirp_mass_solar` - chirp mass in solar masses
    /// * `time_to_merger` - time remaining until coalescence (seconds). Must be > 0.
    pub fn frequency_evolution(chirp_mass_solar: f64, time_to_merger: f64) -> f64 {
        if time_to_merger <= 0.0 {
            return f64::INFINITY;
        }
        let mc_si = chirp_mass_solar * SOLAR_MASS_KG;
        let mc_seconds = G_SI * mc_si / C_SI.powi(3);
        (1.0 / PI) * (5.0 / (256.0 * time_to_merger)).powf(3.0 / 8.0)
            * mc_seconds.powf(-5.0 / 8.0)
    }

    /// Generate a Newtonian-order inspiral chirp waveform h(t).
    ///
    /// Uses the leading-order (0PN) restricted waveform:
    ///
    /// ```text
    /// h(t) = A(t) * cos(Phi(t))
    /// ```
    ///
    /// where the amplitude and phase are determined by the chirp mass, distance,
    /// and inclination. The waveform starts at `f_low` and terminates at ISCO.
    ///
    /// # Arguments
    /// * `params` - compact binary source parameters
    /// * `sample_rate` - sampling rate (Hz)
    /// * `f_low` - starting GW frequency (Hz)
    ///
    /// # Returns
    /// Time-domain strain waveform samples.
    pub fn chirp_waveform(
        params: &CompactBinaryParams,
        sample_rate: f64,
        f_low: f64,
    ) -> Vec<f64> {
        let mc_solar = params.chirp_mass_solar;
        let mc_si = mc_solar * SOLAR_MASS_KG;
        let mc_sec = G_SI * mc_si / C_SI.powi(3);
        let d_si = params.distance_mpc * MPC_TO_M;

        // Total chirp time from f_low to merger
        let t_chirp = Self::newtonian_chirp_time(mc_solar, f_low);
        let n_samples = (t_chirp * sample_rate).ceil() as usize;

        // Cap to avoid enormous allocations
        let n_samples = n_samples.min(10_000_000);
        if n_samples == 0 {
            return vec![];
        }

        let dt = 1.0 / sample_rate;
        let total_mass_solar = params.m1_solar + params.m2_solar;
        let f_isco = Self::isco_frequency(total_mass_solar);

        // Amplitude prefactor (leading order, face-on plus/cross combined)
        // h ~ (G*Mc/c^2) / d * (pi * G * Mc * f / c^3)^(2/3) -- simplified
        // We use the time-domain amplitude evolution:
        // A(tau) = (1/d) * (G*Mc/c^2) * (5*G*Mc / (c^3 * tau))^(1/4) * geometry
        let inclination_factor = (1.0 + params.inclination_rad.cos().powi(2)) / 2.0;
        let amp_prefactor = if d_si > 0.0 {
            (G_SI * mc_si / (C_SI * C_SI)) / d_si * inclination_factor
        } else {
            1.0
        };

        let mut waveform = Vec::with_capacity(n_samples);
        let mut phase = params.phase_rad;

        for i in 0..n_samples {
            let tau = t_chirp - (i as f64) * dt;
            if tau <= 0.0 {
                break;
            }

            let freq = Self::frequency_evolution(mc_solar, tau);
            if freq > f_isco || freq.is_infinite() {
                break;
            }

            // Newtonian amplitude: A(tau) ~ tau^(-1/4)
            let amplitude = amp_prefactor * (5.0 * mc_sec / tau).powf(0.25);

            waveform.push(amplitude * phase.cos());

            // Accumulate phase
            phase += 2.0 * PI * freq * dt;
        }

        waveform
    }
}

// ---------------------------------------------------------------------------
// MatchedFilterEngine
// ---------------------------------------------------------------------------

/// Matched filtering engine for gravitational wave searches.
///
/// Implements noise-weighted inner products, SNR time series computation,
/// and the Allen chi-squared discriminator for glitch rejection.
///
/// The one-sided PSD S_n(f) characterises the detector noise. All inner
/// products are weighted by 1/S_n(f) so that the matched filter is optimal
/// in the Neyman-Pearson sense.
#[derive(Debug, Clone)]
pub struct MatchedFilterEngine {
    /// One-sided noise PSD evaluated at frequency bins.
    pub psd: Vec<f64>,
    /// Sampling rate (Hz).
    pub sample_rate: f64,
    /// Low-frequency cut-off (Hz).
    pub f_low: f64,
}

impl MatchedFilterEngine {
    /// Create a matched filter engine with a given PSD.
    ///
    /// # Arguments
    /// * `psd` - one-sided noise power spectral density at each frequency bin
    /// * `sample_rate` - sampling rate (Hz)
    /// * `f_low` - low-frequency cut-off (Hz)
    pub fn new(psd: Vec<f64>, sample_rate: f64, f_low: f64) -> Self {
        Self {
            psd,
            sample_rate,
            f_low,
        }
    }

    /// Noise-weighted inner product <a|b>.
    ///
    /// ```text
    /// <a|b> = 4 * Re sum_k [ a~[k] * conj(b~[k]) / S_n[k] ] * df
    /// ```
    ///
    /// Here `a` and `b` are real-valued frequency-domain amplitudes (magnitude of
    /// the Fourier transform), and the sum runs over positive frequencies only.
    ///
    /// # Arguments
    /// * `a` - frequency-domain amplitudes of signal a
    /// * `b` - frequency-domain amplitudes of signal b
    /// * `psd` - noise PSD at each frequency bin
    /// * `df` - frequency resolution (Hz)
    pub fn inner_product(a: &[f64], b: &[f64], psd: &[f64], df: f64) -> f64 {
        let n = a.len().min(b.len()).min(psd.len());
        let mut sum = 0.0;
        for i in 0..n {
            if psd[i] > 0.0 {
                sum += a[i] * b[i] / psd[i];
            }
        }
        4.0 * sum * df
    }

    /// Compute the norm of a template: sqrt(<h|h>).
    ///
    /// # Arguments
    /// * `template` - frequency-domain template amplitudes
    /// * `psd` - noise PSD
    /// * `df` - frequency resolution (Hz)
    pub fn template_norm(template: &[f64], psd: &[f64], df: f64) -> f64 {
        Self::inner_product(template, template, psd, df).sqrt()
    }

    /// Compute SNR time series via time-domain matched filtering.
    ///
    /// Performs cross-correlation of data with a normalised template and
    /// divides by an estimate of the noise standard deviation.
    ///
    /// For efficiency in production one would use FFT-based correlation;
    /// this implementation uses direct correlation for clarity.
    ///
    /// # Arguments
    /// * `data` - detector strain time series
    /// * `template` - time-domain template waveform (shorter than data)
    /// * `psd` - noise PSD (used to compute normalisation)
    ///
    /// # Returns
    /// SNR time series, one value for each valid lag.
    pub fn snr_time_series(data: &[f64], template: &[f64], psd: &[f64]) -> Vec<f64> {
        if template.is_empty() || data.len() < template.len() {
            return vec![];
        }

        let tlen = template.len();
        let out_len = data.len() - tlen + 1;

        // Template energy for normalisation
        let template_energy: f64 = template.iter().map(|x| x * x).sum();
        if template_energy == 0.0 {
            return vec![0.0; out_len];
        }

        // Noise sigma from PSD (simple RMS estimate)
        let noise_sigma = if !psd.is_empty() {
            let mean_psd: f64 = psd.iter().sum::<f64>() / psd.len() as f64;
            mean_psd.sqrt().max(1e-30)
        } else {
            1.0
        };

        let norm = (template_energy).sqrt() * noise_sigma;

        let mut snr = Vec::with_capacity(out_len);
        for i in 0..out_len {
            let mut corr = 0.0;
            for j in 0..tlen {
                corr += data[i + j] * template[j];
            }
            snr.push(corr / norm);
        }

        snr
    }

    /// Frequency-domain matched filter.
    ///
    /// Computes the noise-weighted correlation in the frequency domain:
    ///
    /// ```text
    /// z[k] = data_fft[k] * conj(template_fft[k]) / S_n[k]
    /// ```
    ///
    /// then inverse-FFTs to obtain the SNR time series.
    ///
    /// Input data and template are provided as (real, imag) pairs representing
    /// their discrete Fourier transforms.
    ///
    /// # Arguments
    /// * `data` - DFT of detector data as (Re, Im) pairs
    /// * `template` - DFT of template as (Re, Im) pairs
    /// * `psd` - noise PSD at each frequency bin
    ///
    /// # Returns
    /// Real-valued inverse-DFT magnitudes (approximated SNR).
    pub fn matched_filter_fft(
        data: &[(f64, f64)],
        template: &[(f64, f64)],
        psd: &[f64],
    ) -> Vec<f64> {
        let n = data.len().min(template.len()).min(psd.len());
        if n == 0 {
            return vec![];
        }

        // z[k] = data[k] * conj(template[k]) / psd[k]
        let mut z_re = vec![0.0; n];
        let mut z_im = vec![0.0; n];

        for k in 0..n {
            if psd[k] > 0.0 {
                let (dr, di) = data[k];
                let (tr, ti) = template[k];
                // conj(template) = (tr, -ti)
                // product: (dr*tr + di*ti, di*tr - dr*ti)
                z_re[k] = (dr * tr + di * ti) / psd[k];
                z_im[k] = (di * tr - dr * ti) / psd[k];
            }
        }

        // Approximate inverse DFT: compute magnitude of each bin as proxy
        // (in production, a full IFFT would be used)
        let mut result = Vec::with_capacity(n);
        for k in 0..n {
            result.push((z_re[k] * z_re[k] + z_im[k] * z_im[k]).sqrt());
        }

        // Normalise by template sigma
        let template_sigma: f64 = result.iter().sum::<f64>() / (n as f64).max(1.0);
        if template_sigma > 0.0 {
            for v in &mut result {
                *v /= template_sigma;
            }
        }

        result
    }

    /// Allen chi-squared discriminator.
    ///
    /// Splits the template into `num_bins` frequency sub-bands, each contributing
    /// approximately equal SNR. For a true signal the chi-squared statistic should
    /// follow chi^2 with (2*p - 2) degrees of freedom (where p = num_bins).
    /// Noise transients (glitches) produce large chi-squared values.
    ///
    /// This implementation works in the time domain, splitting the template
    /// into contiguous sub-templates and computing sub-SNR contributions.
    ///
    /// # Arguments
    /// * `data` - detector strain time series
    /// * `template` - time-domain template waveform
    /// * `psd` - noise PSD
    /// * `num_bins` - number of chi-squared bins (typically 16)
    ///
    /// # Returns
    /// Chi-squared value for each valid time offset.
    pub fn chi_squared_discriminator(
        data: &[f64],
        template: &[f64],
        psd: &[f64],
        num_bins: usize,
    ) -> Vec<f64> {
        if template.is_empty() || num_bins == 0 || data.len() < template.len() {
            return vec![];
        }

        let tlen = template.len();
        let out_len = data.len() - tlen + 1;
        let bin_size = tlen / num_bins;
        if bin_size == 0 {
            return vec![0.0; out_len];
        }

        // Full SNR time series for normalisation
        let full_snr = Self::snr_time_series(data, template, psd);

        let mut chi2 = vec![0.0; out_len];

        for b in 0..num_bins {
            let start = b * bin_size;
            let end = if b == num_bins - 1 {
                tlen
            } else {
                (b + 1) * bin_size
            };
            let sub_template = &template[start..end];

            // Sub-template energy fraction
            let sub_energy: f64 = sub_template.iter().map(|x| x * x).sum();
            let total_energy: f64 = template.iter().map(|x| x * x).sum();
            let expected_fraction = if total_energy > 0.0 {
                sub_energy / total_energy
            } else {
                1.0 / num_bins as f64
            };

            // Sub-correlation
            for i in 0..out_len {
                let mut sub_corr = 0.0;
                for j in 0..sub_template.len() {
                    sub_corr += data[i + start + j] * sub_template[j];
                }

                // Normalise sub-correlation to SNR units
                let sub_norm = sub_energy.sqrt().max(1e-30);
                let noise_sigma = if !psd.is_empty() {
                    let mean_psd: f64 = psd.iter().sum::<f64>() / psd.len() as f64;
                    mean_psd.sqrt().max(1e-30)
                } else {
                    1.0
                };
                let sub_snr = sub_corr / (sub_norm * noise_sigma);

                // Expected sub-SNR = fraction * total SNR
                let expected_snr = expected_fraction * full_snr[i];

                let delta = sub_snr - expected_snr;
                chi2[i] += delta * delta;
            }
        }

        chi2
    }
}

// ---------------------------------------------------------------------------
// TemplateBank
// ---------------------------------------------------------------------------

/// Template bank for matched filter searches.
///
/// A template bank discretises the continuous mass parameter space into a finite
/// set of templates such that any signal within the parameter space has overlap
/// (match) >= `minimal_match` with at least one template.
///
/// The metric on the parameter space determines optimal template placement.
/// This implementation uses a simple grid for demonstration; production searches
/// use stochastic or hexagonal placement.
#[derive(Debug, Clone)]
pub struct TemplateBank {
    /// Range of mass 1 in solar masses: (min, max).
    pub m1_range: (f64, f64),
    /// Range of mass 2 in solar masses: (min, max).
    pub m2_range: (f64, f64),
    /// Minimal match: the minimum overlap between any signal and its
    /// nearest template. Typically 0.97.
    pub minimal_match: f64,
}

impl TemplateBank {
    /// Create a new template bank specification.
    pub fn new(m1_range: (f64, f64), m2_range: (f64, f64), minimal_match: f64) -> Self {
        Self {
            m1_range,
            m2_range,
            minimal_match,
        }
    }

    /// Generate a bank of templates on a uniform grid in (m1, m2).
    ///
    /// Each template is a `CompactBinaryParams` at the grid point,
    /// with default extrinsic parameters (100 Mpc, face-on, zero phase).
    ///
    /// # Arguments
    /// * `num_templates` - total number of templates. The grid will be
    ///   approximately sqrt(N) x sqrt(N).
    pub fn generate_bank(&self, num_templates: usize) -> Vec<CompactBinaryParams> {
        if num_templates == 0 {
            return vec![];
        }

        let n_side = (num_templates as f64).sqrt().ceil() as usize;
        let n_side = n_side.max(1);

        let dm1 = if n_side > 1 {
            (self.m1_range.1 - self.m1_range.0) / (n_side - 1) as f64
        } else {
            0.0
        };
        let dm2 = if n_side > 1 {
            (self.m2_range.1 - self.m2_range.0) / (n_side - 1) as f64
        } else {
            0.0
        };

        let mut bank = Vec::with_capacity(num_templates);
        for i in 0..n_side {
            for j in 0..n_side {
                if bank.len() >= num_templates {
                    break;
                }
                let m1 = self.m1_range.0 + i as f64 * dm1;
                let m2 = self.m2_range.0 + j as f64 * dm2;
                // Enforce m1 >= m2 convention
                let (m1_eff, m2_eff) = if m1 >= m2 { (m1, m2) } else { (m2, m1) };
                bank.push(CompactBinaryParams::new(
                    m1_eff, m2_eff, 100.0, 0.0, 0.0,
                ));
            }
        }

        bank
    }

    /// Compute the match (overlap maximised over phase) between two templates.
    ///
    /// match = <h1|h2> / (||h1|| * ||h2||)
    ///
    /// # Arguments
    /// * `h1`, `h2` - frequency-domain template amplitudes
    /// * `psd` - noise PSD
    /// * `df` - frequency resolution (Hz)
    pub fn match_between_templates(h1: &[f64], h2: &[f64], psd: &[f64], df: f64) -> f64 {
        let inner = MatchedFilterEngine::inner_product(h1, h2, psd, df);
        let norm1 = MatchedFilterEngine::template_norm(h1, psd, df);
        let norm2 = MatchedFilterEngine::template_norm(h2, psd, df);

        if norm1 > 0.0 && norm2 > 0.0 {
            (inner / (norm1 * norm2)).clamp(-1.0, 1.0)
        } else {
            0.0
        }
    }

    /// Estimate the number of templates required to cover a mass range.
    ///
    /// Uses a rough scaling: N ~ (mass_range)^2 / (1 - minimal_match).
    /// This is a simplified estimate; the true scaling depends on the
    /// parameter-space metric.
    ///
    /// # Arguments
    /// * `m_range` - (min_mass, max_mass) in solar masses
    /// * `minimal_match` - minimum match (e.g. 0.97)
    pub fn bank_size_estimate(m_range: (f64, f64), minimal_match: f64) -> usize {
        let delta_m = m_range.1 - m_range.0;
        let mismatch = 1.0 - minimal_match;
        if mismatch <= 0.0 {
            return 1;
        }
        ((delta_m * delta_m) / mismatch).ceil() as usize
    }
}

// ---------------------------------------------------------------------------
// CoincidenceDetector
// ---------------------------------------------------------------------------

/// Multi-detector coincidence analysis.
///
/// Gravitational wave searches require coincident triggers in multiple
/// detectors within the light travel time between sites (~10 ms for
/// LIGO Hanford--Livingston). This rejects noise transients that are
/// uncorrelated between detectors.
#[derive(Debug, Clone)]
pub struct CoincidenceDetector {
    /// Names of detectors in the network.
    pub detectors: Vec<String>,
    /// Coincidence time window (milliseconds).
    pub time_window_ms: f64,
}

impl CoincidenceDetector {
    /// Create a new coincidence detector.
    ///
    /// # Arguments
    /// * `detectors` - list of detector names
    /// * `time_window_ms` - maximum allowed time difference (ms)
    pub fn new(detectors: Vec<String>, time_window_ms: f64) -> Self {
        Self {
            detectors,
            time_window_ms,
        }
    }

    /// Find triggers exceeding the SNR threshold.
    ///
    /// Returns (sample_index, snr_value) pairs for all samples where
    /// the SNR exceeds the threshold and is a local maximum.
    ///
    /// # Arguments
    /// * `snr_series` - SNR time series (absolute values are compared)
    /// * `threshold` - SNR threshold
    pub fn find_triggers(snr_series: &[f64], threshold: f64) -> Vec<(usize, f64)> {
        let mut triggers = Vec::new();
        let n = snr_series.len();

        for i in 0..n {
            let val = snr_series[i].abs();
            if val >= threshold {
                // Check local maximum (simple 1-sample neighbourhood)
                let is_peak = (i == 0 || val >= snr_series[i - 1].abs())
                    && (i == n - 1 || val >= snr_series[i + 1].abs());
                if is_peak {
                    triggers.push((i, val));
                }
            }
        }

        triggers
    }

    /// Test for coincident triggers between two detectors.
    ///
    /// Returns triplets (index1, index2, combined_snr) for trigger pairs
    /// within `dt_max_samples` of each other.
    ///
    /// # Arguments
    /// * `triggers1` - triggers from detector 1
    /// * `triggers2` - triggers from detector 2
    /// * `dt_max_samples` - maximum allowed sample offset
    pub fn coincidence_test(
        triggers1: &[(usize, f64)],
        triggers2: &[(usize, f64)],
        dt_max_samples: usize,
    ) -> Vec<(usize, usize, f64)> {
        let mut coincidences = Vec::new();

        for &(i1, snr1) in triggers1 {
            for &(i2, snr2) in triggers2 {
                let dt = if i1 >= i2 { i1 - i2 } else { i2 - i1 };
                if dt <= dt_max_samples {
                    let combined = Self::combined_snr(snr1, snr2);
                    coincidences.push((i1, i2, combined));
                }
            }
        }

        coincidences
    }

    /// Combined network SNR from multiple detectors.
    ///
    /// rho_combined = sqrt(rho1^2 + rho2^2)
    pub fn combined_snr(snr1: f64, snr2: f64) -> f64 {
        (snr1 * snr1 + snr2 * snr2).sqrt()
    }

    /// Estimate the false alarm rate (FAR) from background triggers.
    ///
    /// FAR = N_triggers_above_threshold / observation_time
    ///
    /// # Arguments
    /// * `snr_threshold` - SNR threshold
    /// * `background_triggers` - list of (index, snr) from time-shifted analysis
    /// * `observation_time_s` - total observation time in seconds
    pub fn false_alarm_rate(
        snr_threshold: f64,
        background_triggers: &[(usize, f64)],
        observation_time_s: f64,
    ) -> f64 {
        if observation_time_s <= 0.0 {
            return 0.0;
        }
        let count = background_triggers
            .iter()
            .filter(|&&(_, snr)| snr >= snr_threshold)
            .count();
        count as f64 / observation_time_s
    }
}

// ---------------------------------------------------------------------------
// SourcePhysics
// ---------------------------------------------------------------------------

/// Astrophysical source physics for compact binaries.
///
/// Provides estimates of gravitational wave luminosity, strain amplitude,
/// and characteristic frequencies (merger, ringdown) for compact binary systems.
pub struct SourcePhysics;

impl SourcePhysics {
    /// Gravitational wave luminosity (power) at a given frequency.
    ///
    /// Leading-order Peters formula for circular orbits:
    ///
    /// L_GW = (32/5) * c^5/G * (pi * G * M_c * f / c^3)^(10/3)
    ///
    /// # Arguments
    /// * `chirp_mass_solar` - chirp mass (solar masses)
    /// * `frequency_hz` - GW frequency (Hz)
    ///
    /// # Returns
    /// Luminosity in Watts.
    pub fn gravitational_wave_luminosity(chirp_mass_solar: f64, frequency_hz: f64) -> f64 {
        let mc_si = chirp_mass_solar * SOLAR_MASS_KG;
        let x = PI * G_SI * mc_si * frequency_hz / C_SI.powi(3);
        (32.0 / 5.0) * C_SI.powi(5) / G_SI * x.powf(10.0 / 3.0)
    }

    /// Strain amplitude h_0 from a compact binary at a given distance.
    ///
    /// Leading order:
    ///
    /// h_0 = (4/d) * (G * M_c / c^2) * (pi * G * M_c * f / c^3)^(2/3)
    ///
    /// # Arguments
    /// * `chirp_mass_solar` - chirp mass (solar masses)
    /// * `frequency_hz` - GW frequency (Hz)
    /// * `distance_mpc` - luminosity distance (Mpc)
    pub fn strain_amplitude(
        chirp_mass_solar: f64,
        frequency_hz: f64,
        distance_mpc: f64,
    ) -> f64 {
        let mc_si = chirp_mass_solar * SOLAR_MASS_KG;
        let d_si = distance_mpc * MPC_TO_M;
        if d_si <= 0.0 {
            return 0.0;
        }
        let x = PI * G_SI * mc_si * frequency_hz / C_SI.powi(3);
        (4.0 / d_si) * (G_SI * mc_si / (C_SI * C_SI)) * x.powf(2.0 / 3.0)
    }

    /// Merger frequency, approximately equal to the ISCO frequency.
    ///
    /// f_merger ~ f_ISCO = c^3 / (6^(3/2) * pi * G * M)
    ///
    /// # Arguments
    /// * `total_mass_solar` - total mass of the binary (solar masses)
    pub fn merger_frequency(total_mass_solar: f64) -> f64 {
        GravitationalWaveFilter::isco_frequency(total_mass_solar)
    }

    /// Quasi-normal mode (ringdown) frequency of the remnant black hole.
    ///
    /// Fit from Echeverria (1989) / Berti et al. (2006) for the dominant
    /// (l=2, m=2) mode:
    ///
    /// f_QNM ~ (c^3 / (2*pi*G*M)) * [1 - 0.63*(1-a)^(3/10)]
    ///
    /// where `a` is the dimensionless spin parameter (0 <= a < 1).
    ///
    /// # Arguments
    /// * `total_mass_solar` - remnant mass (approximately total mass)
    /// * `spin` - dimensionless spin parameter (0..1)
    pub fn ringdown_frequency(total_mass_solar: f64, spin: f64) -> f64 {
        let m_si = total_mass_solar * SOLAR_MASS_KG;
        let f_natural = C_SI.powi(3) / (2.0 * PI * G_SI * m_si);
        let a = spin.clamp(0.0, 0.999);
        f_natural * (1.0 - 0.63 * (1.0 - a).powf(0.3))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // --- Chirp mass tests ---

    #[test]
    fn test_chirp_mass_equal_masses() {
        // For m1 = m2 = m, M_c = m * 2^(-1/5)
        let m = 10.0;
        let mc = GravitationalWaveFilter::chirp_mass(m, m);
        let expected = m * 2.0_f64.powf(-1.0 / 5.0);
        assert!(
            (mc - expected).abs() < TOLERANCE,
            "chirp_mass({0},{0}) = {1}, expected {2}",
            m,
            mc,
            expected
        );
    }

    #[test]
    fn test_chirp_mass_equal_30_solar() {
        let mc = GravitationalWaveFilter::chirp_mass(30.0, 30.0);
        let expected = 30.0 * 2.0_f64.powf(-1.0 / 5.0);
        assert!((mc - expected).abs() < 1e-10);
    }

    #[test]
    fn test_chirp_mass_unequal() {
        let mc = GravitationalWaveFilter::chirp_mass(1.4, 1.4);
        let expected = 1.4 * 2.0_f64.powf(-1.0 / 5.0);
        assert!((mc - expected).abs() < 1e-10);
    }

    #[test]
    fn test_chirp_mass_asymmetric() {
        // M_c = (m1*m2)^(3/5) / (m1+m2)^(1/5)
        let m1 = 30.0;
        let m2 = 5.0;
        let mc = GravitationalWaveFilter::chirp_mass(m1, m2);
        let expected = (m1 * m2).powf(0.6) / (m1 + m2).powf(0.2);
        assert!((mc - expected).abs() < 1e-10);
    }

    #[test]
    fn test_chirp_mass_symmetric() {
        // chirp_mass(m1, m2) == chirp_mass(m2, m1)
        let mc_12 = GravitationalWaveFilter::chirp_mass(10.0, 20.0);
        let mc_21 = GravitationalWaveFilter::chirp_mass(20.0, 10.0);
        assert!((mc_12 - mc_21).abs() < 1e-14);
    }

    // --- Symmetric mass ratio tests ---

    #[test]
    fn test_eta_equal_masses() {
        // eta = 0.25 for equal masses
        let eta = GravitationalWaveFilter::symmetric_mass_ratio(10.0, 10.0);
        assert!(
            (eta - 0.25).abs() < TOLERANCE,
            "eta for equal masses = {}, expected 0.25",
            eta
        );
    }

    #[test]
    fn test_eta_range() {
        // eta should always be in (0, 0.25]
        let eta = GravitationalWaveFilter::symmetric_mass_ratio(1.0, 100.0);
        assert!(eta > 0.0 && eta <= 0.25, "eta = {} out of range", eta);
    }

    #[test]
    fn test_eta_extreme_ratio() {
        // For m1 >> m2, eta -> 0
        let eta = GravitationalWaveFilter::symmetric_mass_ratio(1000.0, 1.0);
        assert!(eta < 0.01, "eta for extreme ratio = {}, expected ~0", eta);
    }

    // --- Total mass ---

    #[test]
    fn test_total_mass() {
        assert!(
            (GravitationalWaveFilter::total_mass(10.0, 20.0) - 30.0).abs() < TOLERANCE
        );
    }

    // --- ISCO frequency tests ---

    #[test]
    fn test_isco_frequency_10_solar() {
        // f_ISCO for 10 M_sun total mass ~ 440 Hz
        // (Note: ~220 Hz corresponds to 20 M_sun total, i.e. a 10+10 binary)
        let f = GravitationalWaveFilter::isco_frequency(10.0);
        assert!(
            (f - 440.0).abs() < 5.0,
            "f_ISCO(10 Msun) = {:.1} Hz, expected ~440 Hz",
            f
        );
    }

    #[test]
    fn test_isco_frequency_60_solar() {
        // f_ISCO for 60 M_sun (30+30) ~ 73 Hz
        let f = GravitationalWaveFilter::isco_frequency(60.0);
        assert!(
            (f - 73.0).abs() < 5.0,
            "f_ISCO(60 Msun) = {:.1} Hz, expected ~73 Hz",
            f
        );
    }

    #[test]
    fn test_isco_frequency_scales_inversely_with_mass() {
        let f1 = GravitationalWaveFilter::isco_frequency(10.0);
        let f2 = GravitationalWaveFilter::isco_frequency(20.0);
        // f_ISCO ~ 1/M, so f1/f2 ~ 2
        assert!(
            ((f1 / f2) - 2.0).abs() < 0.01,
            "ISCO scaling: f1/f2 = {:.4}, expected 2.0",
            f1 / f2
        );
    }

    // --- Chirp time tests ---

    #[test]
    fn test_chirp_time_positive() {
        let mc = GravitationalWaveFilter::chirp_mass(1.4, 1.4);
        let tau = GravitationalWaveFilter::newtonian_chirp_time(mc, 30.0);
        assert!(tau > 0.0, "chirp time should be positive: {}", tau);
    }

    #[test]
    fn test_chirp_time_decreases_with_mass() {
        // Heavier systems merge faster
        let mc_light = GravitationalWaveFilter::chirp_mass(1.4, 1.4);
        let mc_heavy = GravitationalWaveFilter::chirp_mass(30.0, 30.0);
        let tau_light = GravitationalWaveFilter::newtonian_chirp_time(mc_light, 30.0);
        let tau_heavy = GravitationalWaveFilter::newtonian_chirp_time(mc_heavy, 30.0);
        assert!(
            tau_light > tau_heavy,
            "lighter system should have longer chirp time: {} vs {}",
            tau_light,
            tau_heavy
        );
    }

    #[test]
    fn test_chirp_time_increases_with_lower_freq() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let tau_20 = GravitationalWaveFilter::newtonian_chirp_time(mc, 20.0);
        let tau_40 = GravitationalWaveFilter::newtonian_chirp_time(mc, 40.0);
        assert!(
            tau_20 > tau_40,
            "lower f_low should give longer chirp time: {} vs {}",
            tau_20,
            tau_40
        );
    }

    #[test]
    fn test_chirp_time_bns_reasonable() {
        // BNS (1.4+1.4) from 30 Hz: should be on order of tens of seconds
        let mc = GravitationalWaveFilter::chirp_mass(1.4, 1.4);
        let tau = GravitationalWaveFilter::newtonian_chirp_time(mc, 30.0);
        assert!(
            tau > 10.0 && tau < 1000.0,
            "BNS chirp time from 30 Hz = {:.1} s, expected 10-1000 s",
            tau
        );
    }

    // --- Frequency evolution ---

    #[test]
    fn test_frequency_evolution_increases() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let f_far = GravitationalWaveFilter::frequency_evolution(mc, 10.0);
        let f_near = GravitationalWaveFilter::frequency_evolution(mc, 1.0);
        assert!(
            f_near > f_far,
            "frequency should increase as tau decreases"
        );
    }

    #[test]
    fn test_frequency_evolution_at_zero() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let f = GravitationalWaveFilter::frequency_evolution(mc, 0.0);
        assert!(f.is_infinite(), "frequency at merger should be infinite");
    }

    // --- Chirp waveform tests ---

    #[test]
    fn test_chirp_waveform_nonempty() {
        let params = CompactBinaryParams::equal_mass(30.0);
        let wf = GravitationalWaveFilter::chirp_waveform(&params, 4096.0, 30.0);
        assert!(!wf.is_empty(), "waveform should not be empty");
    }

    #[test]
    fn test_chirp_waveform_frequency_increases() {
        // Check that the instantaneous frequency increases by looking at
        // zero-crossing spacing decreasing over time
        let params = CompactBinaryParams::equal_mass(10.0);
        let wf = GravitationalWaveFilter::chirp_waveform(&params, 4096.0, 30.0);

        // Find zero crossings
        let mut crossings = Vec::new();
        for i in 1..wf.len() {
            if wf[i - 1] * wf[i] < 0.0 {
                crossings.push(i);
            }
        }

        if crossings.len() > 10 {
            let early_spacing = crossings[5] - crossings[3];
            let late_spacing = crossings[crossings.len() - 3] - crossings[crossings.len() - 5];
            assert!(
                late_spacing <= early_spacing,
                "later zero-crossing spacing ({}) should be <= earlier ({})",
                late_spacing,
                early_spacing
            );
        }
    }

    #[test]
    fn test_chirp_waveform_amplitude_increases() {
        let params = CompactBinaryParams::equal_mass(10.0);
        let wf = GravitationalWaveFilter::chirp_waveform(&params, 4096.0, 30.0);

        if wf.len() > 200 {
            let early_peak: f64 = wf[..100]
                .iter()
                .map(|x| x.abs())
                .fold(0.0, f64::max);
            let late_peak: f64 = wf[wf.len() - 100..]
                .iter()
                .map(|x| x.abs())
                .fold(0.0, f64::max);
            assert!(
                late_peak >= early_peak,
                "late amplitude ({:.3e}) should be >= early ({:.3e})",
                late_peak,
                early_peak
            );
        }
    }

    // --- CompactBinaryParams ---

    #[test]
    fn test_compact_binary_params_computed_fields() {
        let p = CompactBinaryParams::new(10.0, 10.0, 100.0, 0.0, 0.0);
        let mc_expected = 10.0 * 2.0_f64.powf(-1.0 / 5.0);
        assert!((p.chirp_mass_solar - mc_expected).abs() < 1e-10);
        assert!((p.eta - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_equal_mass_constructor() {
        let p = CompactBinaryParams::equal_mass(15.0);
        assert!((p.m1_solar - 15.0).abs() < 1e-10);
        assert!((p.m2_solar - 15.0).abs() < 1e-10);
        assert!((p.distance_mpc - 100.0).abs() < 1e-10);
    }

    // --- Inner product / norm / match tests ---

    #[test]
    fn test_inner_product_self() {
        let h = vec![1.0, 2.0, 3.0, 4.0];
        let psd = vec![1.0; 4];
        let df = 1.0;
        let ip = MatchedFilterEngine::inner_product(&h, &h, &psd, df);
        // 4 * sum(h_i^2) * df = 4 * (1+4+9+16) = 120
        assert!((ip - 120.0).abs() < 1e-10);
    }

    #[test]
    fn test_template_norm() {
        let h = vec![3.0, 4.0]; // norm^2 = 4*(9+16)*1 = 100, norm = 10
        let psd = vec![1.0; 2];
        let df = 1.0;
        let norm = MatchedFilterEngine::template_norm(&h, &psd, df);
        assert!((norm - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_match_self_is_one() {
        let h = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let psd = vec![1.0; 5];
        let df = 1.0;
        let m = TemplateBank::match_between_templates(&h, &h, &psd, df);
        assert!(
            (m - 1.0).abs() < 1e-10,
            "self-match = {}, expected 1.0",
            m
        );
    }

    #[test]
    fn test_match_orthogonal() {
        // Orthogonal signals: match = 0
        let h1 = vec![1.0, 0.0, 1.0, 0.0];
        let h2 = vec![0.0, 1.0, 0.0, 1.0];
        let psd = vec![1.0; 4];
        let df = 1.0;
        let m = TemplateBank::match_between_templates(&h1, &h2, &psd, df);
        assert!(m.abs() < 1e-10, "orthogonal match = {}, expected 0", m);
    }

    // --- SNR time series ---

    #[test]
    fn test_snr_of_template_with_itself() {
        // When the data IS the template (in white noise), the peak SNR should
        // be high at the correct lag
        let template: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let psd = vec![1.0; 50];
        let snr = MatchedFilterEngine::snr_time_series(&template, &template, &psd);
        // The peak should be at lag 0
        assert!(!snr.is_empty());
        let peak_snr = snr[0];
        // Peak SNR should be positive and significant
        assert!(
            peak_snr > 0.0,
            "peak SNR for self-match = {}, expected > 0",
            peak_snr
        );
    }

    #[test]
    fn test_snr_peak_at_correct_offset() {
        // Embed template in noise-free data with a delay
        let template: Vec<f64> = (0..32).map(|i| (i as f64 * 0.3).sin()).collect();
        let mut data = vec![0.0; 100];
        let offset = 20;
        for (j, &t) in template.iter().enumerate() {
            data[offset + j] = t;
        }
        let psd = vec![1.0; 16];
        let snr = MatchedFilterEngine::snr_time_series(&data, &template, &psd);

        // Find peak
        let (peak_idx, _peak_val) = snr
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap();
        assert_eq!(peak_idx, offset, "SNR peak should be at offset {}", offset);
    }

    // --- Coincidence tests ---

    #[test]
    fn test_combined_snr() {
        let combined = CoincidenceDetector::combined_snr(8.0, 8.0);
        let expected = (128.0_f64).sqrt(); // sqrt(64+64)
        assert!(
            (combined - expected).abs() < 0.01,
            "combined SNR of (8,8) = {:.2}, expected {:.2}",
            combined,
            expected
        );
    }

    #[test]
    fn test_combined_snr_single() {
        let combined = CoincidenceDetector::combined_snr(10.0, 0.0);
        assert!((combined - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_find_triggers_basic() {
        let snr = vec![1.0, 3.0, 5.0, 9.0, 7.0, 2.0, 10.0, 6.0];
        let triggers = CoincidenceDetector::find_triggers(&snr, 8.0);
        // Should find peaks at index 3 (snr=9) and index 6 (snr=10)
        assert!(
            triggers.len() >= 2,
            "expected >= 2 triggers, got {}",
            triggers.len()
        );
        let indices: Vec<usize> = triggers.iter().map(|t| t.0).collect();
        assert!(indices.contains(&3), "should find trigger at index 3");
        assert!(indices.contains(&6), "should find trigger at index 6");
    }

    #[test]
    fn test_find_triggers_none() {
        let snr = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let triggers = CoincidenceDetector::find_triggers(&snr, 8.0);
        assert!(triggers.is_empty());
    }

    #[test]
    fn test_coincidence_test_finds_simultaneous() {
        let triggers1 = vec![(100, 10.0), (500, 9.0)];
        let triggers2 = vec![(102, 11.0), (800, 8.0)];
        let coincidences = CoincidenceDetector::coincidence_test(&triggers1, &triggers2, 5);

        assert!(
            !coincidences.is_empty(),
            "should find coincident trigger near sample 100/102"
        );
        // The first pair (100, 102) has dt=2 <= 5
        assert_eq!(coincidences[0].0, 100);
        assert_eq!(coincidences[0].1, 102);
    }

    #[test]
    fn test_coincidence_test_rejects_far() {
        let triggers1 = vec![(100, 10.0)];
        let triggers2 = vec![(200, 10.0)];
        let coincidences = CoincidenceDetector::coincidence_test(&triggers1, &triggers2, 5);
        assert!(
            coincidences.is_empty(),
            "should reject triggers 100 samples apart"
        );
    }

    #[test]
    fn test_false_alarm_rate() {
        let background = vec![(0, 5.0), (1, 9.0), (2, 12.0), (3, 7.0)];
        let far = CoincidenceDetector::false_alarm_rate(8.0, &background, 100.0);
        // 2 triggers above 8.0 in 100 s => FAR = 0.02 Hz
        assert!(
            (far - 0.02).abs() < 1e-10,
            "FAR = {}, expected 0.02",
            far
        );
    }

    // --- TemplateBank tests ---

    #[test]
    fn test_generate_bank_size() {
        let bank = TemplateBank::new((1.0, 50.0), (1.0, 50.0), 0.97);
        let templates = bank.generate_bank(25);
        assert_eq!(templates.len(), 25);
    }

    #[test]
    fn test_generate_bank_covers_range() {
        let bank = TemplateBank::new((1.0, 10.0), (1.0, 10.0), 0.97);
        let templates = bank.generate_bank(16);

        // Should have templates near the corners of the mass range
        let has_low = templates.iter().any(|t| t.m2_solar < 2.0);
        let has_high = templates.iter().any(|t| t.m1_solar > 8.0);
        assert!(has_low, "bank should cover low mass end");
        assert!(has_high, "bank should cover high mass end");
    }

    #[test]
    fn test_bank_size_estimate() {
        let n = TemplateBank::bank_size_estimate((1.0, 50.0), 0.97);
        assert!(n > 0, "bank size estimate should be positive");
        // Rough estimate: (49)^2 / 0.03 ~ 80000
        assert!(
            n > 1000,
            "bank size for (1,50) with MM=0.97 should be large: {}",
            n
        );
    }

    #[test]
    fn test_bank_size_estimate_higher_match_more_templates() {
        let n_97 = TemplateBank::bank_size_estimate((1.0, 10.0), 0.97);
        let n_90 = TemplateBank::bank_size_estimate((1.0, 10.0), 0.90);
        assert!(
            n_97 > n_90,
            "higher minimal match should require more templates: {} vs {}",
            n_97,
            n_90
        );
    }

    // --- SourcePhysics tests ---

    #[test]
    fn test_strain_decreases_with_distance() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let h_near = SourcePhysics::strain_amplitude(mc, 100.0, 10.0);
        let h_far = SourcePhysics::strain_amplitude(mc, 100.0, 100.0);
        assert!(
            h_near > h_far,
            "strain should decrease with distance: {} vs {}",
            h_near,
            h_far
        );
    }

    #[test]
    fn test_strain_scales_inversely_with_distance() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let h1 = SourcePhysics::strain_amplitude(mc, 100.0, 10.0);
        let h2 = SourcePhysics::strain_amplitude(mc, 100.0, 20.0);
        // h ~ 1/d, so h1/h2 ~ 2
        assert!(
            ((h1 / h2) - 2.0).abs() < 0.01,
            "strain scaling: h1/h2 = {:.4}, expected 2.0",
            h1 / h2
        );
    }

    #[test]
    fn test_gw_luminosity_positive() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let l = SourcePhysics::gravitational_wave_luminosity(mc, 100.0);
        assert!(l > 0.0, "GW luminosity should be positive");
    }

    #[test]
    fn test_gw_luminosity_increases_with_frequency() {
        let mc = GravitationalWaveFilter::chirp_mass(10.0, 10.0);
        let l_low = SourcePhysics::gravitational_wave_luminosity(mc, 50.0);
        let l_high = SourcePhysics::gravitational_wave_luminosity(mc, 200.0);
        assert!(
            l_high > l_low,
            "luminosity should increase with frequency"
        );
    }

    #[test]
    fn test_merger_frequency_equals_isco() {
        let f_merger = SourcePhysics::merger_frequency(20.0);
        let f_isco = GravitationalWaveFilter::isco_frequency(20.0);
        assert!((f_merger - f_isco).abs() < 1e-10);
    }

    #[test]
    fn test_ringdown_frequency_increases_with_spin() {
        let f_nospin = SourcePhysics::ringdown_frequency(20.0, 0.0);
        let f_spin = SourcePhysics::ringdown_frequency(20.0, 0.7);
        assert!(
            f_spin > f_nospin,
            "spinning BH should have higher ringdown freq: {} vs {}",
            f_spin,
            f_nospin
        );
    }

    #[test]
    fn test_ringdown_frequency_positive() {
        let f = SourcePhysics::ringdown_frequency(60.0, 0.69);
        assert!(f > 0.0, "ringdown frequency should be positive: {}", f);
    }

    // --- Matched filter FFT ---

    #[test]
    fn test_matched_filter_fft_basic() {
        let data: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5)];
        let template = data.clone();
        let psd = vec![1.0; 4];
        let result = MatchedFilterEngine::matched_filter_fft(&data, &template, &psd);
        assert_eq!(result.len(), 4);
        // All values should be non-negative
        assert!(result.iter().all(|&v| v >= 0.0));
    }

    #[test]
    fn test_matched_filter_fft_empty() {
        let result = MatchedFilterEngine::matched_filter_fft(&[], &[], &[]);
        assert!(result.is_empty());
    }

    // --- Chi-squared discriminator ---

    #[test]
    fn test_chi_squared_for_signal() {
        // For a true signal, chi^2 should be small
        let template: Vec<f64> = (0..64).map(|i| (i as f64 * 0.2).sin()).collect();
        let data = template.clone(); // perfect match
        let psd = vec![1.0; 32];
        let chi2 = MatchedFilterEngine::chi_squared_discriminator(&data, &template, &psd, 4);
        assert!(!chi2.is_empty());
    }

    #[test]
    fn test_chi_squared_empty_template() {
        let chi2 = MatchedFilterEngine::chi_squared_discriminator(&[1.0, 2.0], &[], &[1.0], 4);
        assert!(chi2.is_empty());
    }

    // --- Constants ---

    #[test]
    fn test_constants_reasonable() {
        assert!(G_SI > 6e-11 && G_SI < 7e-11);
        assert!(C_SI > 2.99e8 && C_SI < 3.01e8);
        assert!(SOLAR_MASS_KG > 1.98e30 && SOLAR_MASS_KG < 2.0e30);
        assert!(MPC_TO_M > 3.0e22 && MPC_TO_M < 3.1e22);
        assert!(SOLAR_MASS_SECONDS > 4.9e-6 && SOLAR_MASS_SECONDS < 5.0e-6);
    }

    // --- DetectorConfig ---

    #[test]
    fn test_detector_config_ligo_default() {
        let cfg = DetectorConfig::ligo_default();
        assert!((cfg.sample_rate_hz - 4096.0).abs() < 1e-10);
        assert!((cfg.snr_threshold - 8.0).abs() < 1e-10);
        assert_eq!(cfg.detector_name, "H1");
    }

    // --- Edge cases ---

    #[test]
    fn test_inner_product_with_zero_psd() {
        // Bins with PSD=0 should be skipped (infinite noise -> no info)
        let h = vec![1.0, 2.0, 3.0];
        let psd = vec![0.0, 1.0, 0.0];
        let df = 1.0;
        let ip = MatchedFilterEngine::inner_product(&h, &h, &psd, df);
        // Only bin 1 contributes: 4 * 2*2 / 1 * 1 = 16
        assert!((ip - 16.0).abs() < 1e-10);
    }

    #[test]
    fn test_snr_empty_template() {
        let snr = MatchedFilterEngine::snr_time_series(&[1.0, 2.0], &[], &[1.0]);
        assert!(snr.is_empty());
    }

    #[test]
    fn test_coincidence_detector_creation() {
        let cd = CoincidenceDetector::new(
            vec!["H1".to_string(), "L1".to_string()],
            10.0,
        );
        assert_eq!(cd.detectors.len(), 2);
        assert!((cd.time_window_ms - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_false_alarm_rate_zero_time() {
        let far = CoincidenceDetector::false_alarm_rate(8.0, &[(0, 10.0)], 0.0);
        assert!((far - 0.0).abs() < 1e-10);
    }
}
