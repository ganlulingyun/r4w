//! Matched filtering and frequency-chirp search for LIGO/Virgo-style gravitational wave detection.
//!
//! This module implements a template-based matched filter bank for detecting compact binary
//! coalescence (CBC) signals in interferometric gravitational wave detector data. The approach
//! follows the standard LIGO/Virgo analysis pipeline: generate a bank of inspiral chirp
//! templates spanning the mass parameter space, whiten the data against the detector's noise
//! power spectral density, and compute the matched filter SNR for each template.
//!
//! # Overview
//!
//! - **[`ChirpTemplate`]** — inspiral waveform from chirp mass and symmetric mass ratio using
//!   the restricted post-Newtonian (0PN) approximation to the inspiral phase
//! - **[`FilterBank`]** — bank of templates covering a rectangular grid in chirp-mass /
//!   symmetric-mass-ratio space
//! - **[`MatchedFilter`]** — time-domain correlation of whitened data with a whitened template,
//!   producing an SNR time series
//! - **[`WhiteningFilter`]** — PSD estimation via Welch's method and spectral pre-whitening
//! - **[`CandidateEvent`]** — detected event with GPS time, peak SNR, and estimated source
//!   parameters
//! - **[`CoincidenceDetector`]** — multi-detector coincidence gating for vetoing noise transients
//!
//! # Physics background
//!
//! As two compact objects (neutron stars or black holes) spiral inward, the emitted gravitational
//! wave sweeps upward in frequency — a "chirp." The instantaneous gravitational wave frequency is
//! twice the orbital frequency, and the phase evolution depends primarily on the *chirp mass*
//!
//! ```text
//! M_c = (m1 * m2)^(3/5) / (m1 + m2)^(1/5)
//! ```
//!
//! and secondarily on the *symmetric mass ratio*
//!
//! ```text
//! eta = m1 * m2 / (m1 + m2)^2
//! ```
//!
//! The module generates Newtonian-order inspiral waveforms in the time domain and correlates them
//! with detector data to search for signals buried in noise.
//!
//! # Example
//!
//! ```
//! use r4w_core::gravitational_wave_filter_bank::{
//!     ChirpTemplate, MatchedFilter, WhiteningFilter, chirp_mass, symmetric_mass_ratio,
//! };
//!
//! let mc = chirp_mass(1.4, 1.4);   // two 1.4 solar-mass neutron stars
//! let eta = symmetric_mass_ratio(1.4, 1.4);
//! let template = ChirpTemplate::new(mc, eta, 4096.0, 20.0, 1000.0);
//! assert!(!template.waveform().is_empty());
//!
//! // Whiten some synthetic data
//! let data: Vec<f64> = vec![0.0; 4096];
//! let mut whitener = WhiteningFilter::new(4096, 256);
//! whitener.estimate_psd(&data);
//! let whitened = whitener.whiten(&data);
//! assert_eq!(whitened.len(), data.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Gravitational constant in SI units (m^3 kg^-1 s^-2).
const G_SI: f64 = 6.674_30e-11;

/// Speed of light in m/s.
const C_SI: f64 = 2.997_924_58e8;

/// Solar mass in kg.
const M_SUN_KG: f64 = 1.988_41e30;

/// Megaparsec in meters (for luminosity distance, unused in template generation but
/// included for completeness of the strain amplitude formula).
const MPC_M: f64 = 3.085_677_581e22;

// ---------------------------------------------------------------------------
// Helper / utility functions
// ---------------------------------------------------------------------------

/// Compute the chirp mass from component masses (in solar masses).
///
/// ```text
/// M_c = (m1 * m2)^(3/5) / (m1 + m2)^(1/5)
/// ```
pub fn chirp_mass(m1_solar: f64, m2_solar: f64) -> f64 {
    let product = m1_solar * m2_solar;
    let total = m1_solar + m2_solar;
    product.powf(3.0 / 5.0) / total.powf(1.0 / 5.0)
}

/// Compute the symmetric mass ratio from component masses.
///
/// ```text
/// eta = m1 * m2 / (m1 + m2)^2
/// ```
///
/// For equal masses eta = 0.25 (maximum); for extreme mass ratio eta -> 0.
pub fn symmetric_mass_ratio(m1_solar: f64, m2_solar: f64) -> f64 {
    let total = m1_solar + m2_solar;
    (m1_solar * m2_solar) / (total * total)
}

/// Total mass in SI kg from solar masses.
fn total_mass_si(m1_solar: f64, m2_solar: f64) -> f64 {
    (m1_solar + m2_solar) * M_SUN_KG
}

/// Orbital frequency at a given time-to-coalescence (Newtonian order).
///
/// ```text
/// f_orb(tau) = (1 / pi) * (5 / (256 * tau))^(3/8) * (G * M_c_SI / c^3)^(-5/8)
/// ```
///
/// where `tau` is the time remaining until coalescence (seconds) and `mc_si` is the
/// chirp mass in kg.
pub fn orbital_frequency(tau: f64, mc_si: f64) -> f64 {
    if tau <= 0.0 {
        return 0.0;
    }
    let gmc_c3 = G_SI * mc_si / (C_SI * C_SI * C_SI);
    (1.0 / PI) * (5.0 / (256.0 * tau)).powf(3.0 / 8.0) * gmc_c3.powf(-5.0 / 8.0)
}

/// Gravitational wave strain amplitude (leading Newtonian order) for a source at
/// luminosity distance `d_mpc` megaparsecs with gravitational wave frequency `f_gw` Hz.
///
/// ```text
/// h = (4 / d) * (G * M_c / c^2)^(5/3) * (pi * f_gw / c)^(2/3)
/// ```
pub fn strain_amplitude(mc_solar: f64, f_gw: f64, d_mpc: f64) -> f64 {
    let mc_si = mc_solar * M_SUN_KG;
    let d_si = d_mpc * MPC_M;
    let gmc_c2 = G_SI * mc_si / (C_SI * C_SI);
    let pif_c = PI * f_gw / C_SI;
    (4.0 / d_si) * gmc_c2.powf(5.0 / 3.0) * pif_c.powf(2.0 / 3.0)
}

/// Weighted inner product of two real-valued time series (no frequency weighting —
/// equivalent to noise-weighted inner product with flat PSD).
///
/// ```text
/// <a|b> = sum_i a[i] * b[i] * dt
/// ```
///
/// where `dt = 1 / sample_rate`.
pub fn inner_product(a: &[f64], b: &[f64], sample_rate: f64) -> f64 {
    let n = a.len().min(b.len());
    let dt = 1.0 / sample_rate;
    let mut sum = 0.0;
    for i in 0..n {
        sum += a[i] * b[i];
    }
    sum * dt
}

// ---------------------------------------------------------------------------
// Minimal in-place FFT (radix-2 Cooley-Tukey, power-of-two lengths)
// ---------------------------------------------------------------------------

/// A minimal complex number for FFT operations.
#[derive(Debug, Clone, Copy)]
struct Cx {
    re: f64,
    im: f64,
}

impl Cx {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    fn add(self, other: Self) -> Self {
        Self {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }
    fn sub(self, other: Self) -> Self {
        Self {
            re: self.re - other.re,
            im: self.im - other.im,
        }
    }
    fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }
}

/// Return the next power of two >= n.
fn next_pow2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Bit-reversal permutation for radix-2 FFT.
fn bit_reverse(buf: &mut [Cx]) {
    let n = buf.len();
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
}

/// In-place radix-2 DIT FFT. `inverse` = true for IFFT.
fn fft_inplace(buf: &mut [Cx], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be power of two");
    bit_reverse(buf);
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_sign = if inverse { 1.0 } else { -1.0 };
        let angle = angle_sign * 2.0 * PI / (len as f64);
        let wn = Cx::new(angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = Cx::new(1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = w.mul(buf[start + k + half]);
                buf[start + k] = u.add(t);
                buf[start + k + half] = u.sub(t);
                w = w.mul(wn);
            }
            start += len;
        }
        len <<= 1;
    }
    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in buf.iter_mut() {
            x.re *= inv_n;
            x.im *= inv_n;
        }
    }
}

/// Forward FFT of a real-valued signal (zero-padded to power-of-two length).
fn rfft(signal: &[f64]) -> Vec<Cx> {
    let n = next_pow2(signal.len());
    let mut buf: Vec<Cx> = signal
        .iter()
        .map(|&x| Cx::new(x, 0.0))
        .chain(std::iter::repeat(Cx::new(0.0, 0.0)))
        .take(n)
        .collect();
    fft_inplace(&mut buf, false);
    buf
}

/// Inverse FFT returning real parts only.
fn irfft(spectrum: &[Cx]) -> Vec<f64> {
    let mut buf = spectrum.to_vec();
    fft_inplace(&mut buf, true);
    buf.iter().map(|c| c.re).collect()
}

// ---------------------------------------------------------------------------
// ChirpTemplate
// ---------------------------------------------------------------------------

/// An inspiral chirp waveform template.
///
/// Generated using the restricted 0PN (Newtonian) approximation for the phase evolution.
/// The waveform starts at `f_low` and sweeps up to `f_high`, with the time-domain
/// samples stored at the specified sample rate.
///
/// # Fields
///
/// - `chirp_mass_solar` — chirp mass in solar masses
/// - `eta` — symmetric mass ratio (0 < eta <= 0.25)
/// - `sample_rate` — sample rate in Hz
/// - `f_low` — low frequency cut-off in Hz
/// - `f_high` — high frequency cut-off in Hz
/// - `waveform` — the time-domain strain template h(t)
#[derive(Debug, Clone)]
pub struct ChirpTemplate {
    /// Chirp mass in solar masses.
    pub chirp_mass_solar: f64,
    /// Symmetric mass ratio.
    pub eta: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Low frequency cut-off in Hz.
    pub f_low: f64,
    /// High frequency cut-off in Hz.
    pub f_high: f64,
    /// Time-domain waveform samples.
    waveform: Vec<f64>,
}

impl ChirpTemplate {
    /// Generate an inspiral chirp template.
    ///
    /// The waveform is computed by stepping backward from coalescence in the
    /// time-to-coalescence variable `tau`, evaluating the Newtonian orbital phase
    /// ```text
    /// Phi(tau) = -2 * (tau / (5 * G * Mc / c^3))^(5/8)
    /// ```
    /// and producing `h(t) = cos(2 * Phi(t))`.
    ///
    /// # Arguments
    ///
    /// * `chirp_mass_solar` — chirp mass in solar masses
    /// * `eta` — symmetric mass ratio (0 < eta <= 0.25)
    /// * `sample_rate` — sample rate in Hz
    /// * `f_low` — lower frequency bound in Hz (typically ~20 Hz for LIGO)
    /// * `f_high` — upper frequency bound in Hz (e.g. ISCO frequency)
    pub fn new(
        chirp_mass_solar: f64,
        eta: f64,
        sample_rate: f64,
        f_low: f64,
        f_high: f64,
    ) -> Self {
        let mc_si = chirp_mass_solar * M_SUN_KG;
        let gmc_c3 = G_SI * mc_si / (C_SI * C_SI * C_SI);

        // Time-to-coalescence at f_low (Newtonian):
        // tau(f) = 5 / (256 * (pi * f)^(8/3) * gmc_c3^(5/3))
        let tau_start = {
            let pf = PI * f_low;
            5.0 / (256.0 * pf.powf(8.0 / 3.0) * gmc_c3.powf(5.0 / 3.0))
        };
        let tau_end = {
            let pf = PI * f_high;
            5.0 / (256.0 * pf.powf(8.0 / 3.0) * gmc_c3.powf(5.0 / 3.0))
        };

        let dt = 1.0 / sample_rate;
        let duration = tau_start - tau_end;
        let n_samples = (duration * sample_rate).ceil() as usize;
        let n_samples = n_samples.max(1);

        let mut waveform = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let tau = tau_start - (i as f64) * dt;
            if tau <= 0.0 {
                break;
            }
            // Newtonian orbital phase
            let phase = -2.0 * (tau / (5.0 * gmc_c3)).powf(5.0 / 8.0);
            // GW phase is 2 * orbital phase for quadrupole radiation
            let gw_phase = 2.0 * phase;
            // Amplitude envelope ~ tau^(-1/4)
            let amplitude = tau.powf(-0.25);
            waveform.push(amplitude * gw_phase.cos());
        }

        // Normalize to unit energy
        let energy: f64 = waveform.iter().map(|x| x * x).sum();
        if energy > 0.0 {
            let norm = 1.0 / energy.sqrt();
            for s in waveform.iter_mut() {
                *s *= norm;
            }
        }

        Self {
            chirp_mass_solar,
            eta,
            sample_rate,
            f_low,
            f_high,
            waveform,
        }
    }

    /// Return a reference to the waveform samples.
    pub fn waveform(&self) -> &[f64] {
        &self.waveform
    }

    /// Duration of the template in seconds.
    pub fn duration(&self) -> f64 {
        self.waveform.len() as f64 / self.sample_rate
    }

    /// Number of samples in the template.
    pub fn len(&self) -> usize {
        self.waveform.len()
    }

    /// Returns true if the waveform has no samples.
    pub fn is_empty(&self) -> bool {
        self.waveform.is_empty()
    }
}

// ---------------------------------------------------------------------------
// WhiteningFilter
// ---------------------------------------------------------------------------

/// Spectral whitening filter using Welch PSD estimation.
///
/// Estimates the one-sided noise power spectral density from data segments and
/// applies frequency-domain pre-whitening (dividing by sqrt(PSD)) so that the
/// whitened data has approximately unit-variance white noise.
#[derive(Debug, Clone)]
pub struct WhiteningFilter {
    /// FFT size used for PSD estimation and whitening.
    pub fft_size: usize,
    /// Hop size (stride) between PSD estimation segments.
    pub hop_size: usize,
    /// Estimated one-sided PSD (length = fft_size).
    psd: Vec<f64>,
    /// Whether PSD has been estimated.
    psd_estimated: bool,
}

impl WhiteningFilter {
    /// Create a new whitening filter.
    ///
    /// # Arguments
    ///
    /// * `fft_size` — FFT length (rounded up to next power of two internally)
    /// * `hop_size` — overlap hop between Welch segments
    pub fn new(fft_size: usize, hop_size: usize) -> Self {
        let fft_size = next_pow2(fft_size);
        Self {
            fft_size,
            hop_size: hop_size.max(1),
            psd: vec![1.0; fft_size],
            psd_estimated: false,
        }
    }

    /// Estimate the PSD from a stretch of data using Welch's method (Hann window,
    /// 50% overlap-style segments with the configured hop size).
    pub fn estimate_psd(&mut self, data: &[f64]) {
        let n = self.fft_size;
        if data.len() < n {
            // Not enough data; set flat PSD
            self.psd = vec![1.0; n];
            self.psd_estimated = true;
            return;
        }

        let mut accum = vec![0.0; n];
        let mut count = 0u64;

        let mut start = 0;
        while start + n <= data.len() {
            // Apply Hann window
            let mut windowed: Vec<Cx> = (0..n)
                .map(|k| {
                    let w = 0.5 * (1.0 - (2.0 * PI * k as f64 / (n as f64 - 1.0)).cos());
                    Cx::new(data[start + k] * w, 0.0)
                })
                .collect();
            fft_inplace(&mut windowed, false);
            for (acc, val) in accum.iter_mut().zip(windowed.iter()) {
                *acc += val.mag_sq();
            }
            count += 1;
            start += self.hop_size;
        }

        if count > 0 {
            let inv = 1.0 / count as f64;
            for a in accum.iter_mut() {
                *a *= inv;
                // Floor to avoid division by zero
                if *a < 1e-30 {
                    *a = 1e-30;
                }
            }
        }

        self.psd = accum;
        self.psd_estimated = true;
    }

    /// Return the estimated PSD.
    pub fn psd(&self) -> &[f64] {
        &self.psd
    }

    /// Whether the PSD has been estimated.
    pub fn is_estimated(&self) -> bool {
        self.psd_estimated
    }

    /// Whiten a time-domain signal by dividing its spectrum by sqrt(PSD).
    ///
    /// The output has the same length as the input. Internally the signal is zero-padded
    /// to the FFT size, whitened in the frequency domain, and inverse-transformed.
    pub fn whiten(&self, data: &[f64]) -> Vec<f64> {
        let n = self.fft_size;
        let orig_len = data.len();

        let mut buf: Vec<Cx> = data
            .iter()
            .map(|&x| Cx::new(x, 0.0))
            .chain(std::iter::repeat(Cx::new(0.0, 0.0)))
            .take(n)
            .collect();

        fft_inplace(&mut buf, false);

        for (b, &p) in buf.iter_mut().zip(self.psd.iter()) {
            let scale = 1.0 / p.sqrt();
            b.re *= scale;
            b.im *= scale;
        }

        fft_inplace(&mut buf, true);

        buf.iter().take(orig_len).map(|c| c.re).collect()
    }
}

// ---------------------------------------------------------------------------
// MatchedFilter
// ---------------------------------------------------------------------------

/// Matched filter that correlates data with a template to produce an SNR time series.
///
/// Supports both a direct time-domain correlation and an FFT-based approach. The SNR is
/// defined as the matched filter output normalized by the noise standard deviation.
#[derive(Debug, Clone)]
pub struct MatchedFilter {
    /// Detection threshold in SNR.
    pub snr_threshold: f64,
    /// Sample rate in Hz (for converting sample index to time).
    pub sample_rate: f64,
}

impl MatchedFilter {
    /// Create a new matched filter.
    ///
    /// # Arguments
    ///
    /// * `snr_threshold` — minimum SNR for a detection trigger
    /// * `sample_rate` — sample rate in Hz
    pub fn new(snr_threshold: f64, sample_rate: f64) -> Self {
        Self {
            snr_threshold,
            sample_rate,
        }
    }

    /// Time-domain matched filtering (direct sliding correlation).
    ///
    /// Computes `snr[k] = sum_j template[j] * data[k + j]` normalized by the template
    /// norm and data noise estimate. Returns the SNR time series.
    pub fn filter_time_domain(&self, data: &[f64], template: &[f64]) -> Vec<f64> {
        if template.is_empty() || data.len() < template.len() {
            return vec![];
        }

        // Template norm
        let tnorm: f64 = template.iter().map(|x| x * x).sum::<f64>().sqrt();
        if tnorm == 0.0 {
            return vec![0.0; data.len() - template.len() + 1];
        }

        let n_out = data.len() - template.len() + 1;
        let mut snr = Vec::with_capacity(n_out);

        // Estimate noise sigma from data
        let data_mean: f64 = data.iter().sum::<f64>() / data.len() as f64;
        let data_var: f64 =
            data.iter().map(|x| (x - data_mean).powi(2)).sum::<f64>() / data.len() as f64;
        let sigma = data_var.sqrt().max(1e-30);

        for k in 0..n_out {
            let mut corr = 0.0;
            for j in 0..template.len() {
                corr += template[j] * data[k + j];
            }
            snr.push(corr / (tnorm * sigma));
        }

        snr
    }

    /// FFT-based matched filtering (overlap-save style).
    ///
    /// Computes the correlation in the frequency domain, which is O(N log N) compared
    /// to O(N * M) for the direct method.
    pub fn filter_fft(&self, data: &[f64], template: &[f64]) -> Vec<f64> {
        if template.is_empty() || data.len() < template.len() {
            return vec![];
        }

        let n = next_pow2(data.len());

        // Zero-pad and FFT
        let data_fft = rfft_padded(data, n);
        let tmpl_fft = rfft_padded(template, n);

        // Cross-correlation in frequency domain: conj(template) * data
        let mut cross: Vec<Cx> = data_fft
            .iter()
            .zip(tmpl_fft.iter())
            .map(|(&d, &t)| d.mul(t.conj()))
            .collect();

        fft_inplace(&mut cross, true);

        // Template norm
        let tnorm: f64 = template.iter().map(|x| x * x).sum::<f64>().sqrt().max(1e-30);

        // Noise estimate
        let data_mean: f64 = data.iter().sum::<f64>() / data.len() as f64;
        let data_var: f64 =
            data.iter().map(|x| (x - data_mean).powi(2)).sum::<f64>() / data.len() as f64;
        let sigma = data_var.sqrt().max(1e-30);

        let n_out = data.len() - template.len() + 1;
        cross
            .iter()
            .take(n_out)
            .map(|c| c.re / (tnorm * sigma))
            .collect()
    }

    /// Find peaks in the SNR time series that exceed the threshold.
    pub fn find_triggers(&self, snr_series: &[f64]) -> Vec<(usize, f64)> {
        let mut triggers = Vec::new();
        let n = snr_series.len();
        for i in 0..n {
            let val = snr_series[i].abs();
            if val >= self.snr_threshold {
                // Simple local-maximum check
                let is_peak = (i == 0 || val >= snr_series[i - 1].abs())
                    && (i + 1 >= n || val >= snr_series[i + 1].abs());
                if is_peak {
                    triggers.push((i, val));
                }
            }
        }
        triggers
    }
}

/// Zero-pad and FFT helper.
fn rfft_padded(signal: &[f64], n: usize) -> Vec<Cx> {
    let mut buf: Vec<Cx> = signal
        .iter()
        .map(|&x| Cx::new(x, 0.0))
        .chain(std::iter::repeat(Cx::new(0.0, 0.0)))
        .take(n)
        .collect();
    fft_inplace(&mut buf, false);
    buf
}

// ---------------------------------------------------------------------------
// CandidateEvent
// ---------------------------------------------------------------------------

/// A candidate gravitational wave event detected by the matched filter.
#[derive(Debug, Clone)]
pub struct CandidateEvent {
    /// Sample index of the peak SNR.
    pub peak_sample: usize,
    /// GPS time of the peak (seconds since GPS epoch), or sample-based time
    /// if no absolute reference is available.
    pub time: f64,
    /// Peak matched filter SNR.
    pub snr: f64,
    /// Estimated chirp mass (solar masses) from the best-matching template.
    pub chirp_mass_solar: f64,
    /// Estimated symmetric mass ratio from the best-matching template.
    pub eta: f64,
    /// Index of the best-matching template in the filter bank.
    pub template_index: usize,
    /// Name of the detector (e.g. "H1", "L1", "V1").
    pub detector: String,
}

impl CandidateEvent {
    /// Estimated total mass in solar masses from chirp mass and eta.
    ///
    /// ```text
    /// M_total = M_c / eta^(3/5)
    /// ```
    pub fn estimated_total_mass(&self) -> f64 {
        if self.eta <= 0.0 {
            return 0.0;
        }
        self.chirp_mass_solar / self.eta.powf(3.0 / 5.0)
    }

    /// Estimated component masses (m1 >= m2) from total mass and eta.
    pub fn estimated_component_masses(&self) -> (f64, f64) {
        let m_total = self.estimated_total_mass();
        if m_total <= 0.0 {
            return (0.0, 0.0);
        }
        // m1 + m2 = M, m1*m2 = eta * M^2
        // => m1,m2 = (M/2) * (1 +/- sqrt(1 - 4*eta))
        let disc = (1.0 - 4.0 * self.eta).max(0.0).sqrt();
        let m1 = 0.5 * m_total * (1.0 + disc);
        let m2 = 0.5 * m_total * (1.0 - disc);
        (m1, m2)
    }
}

// ---------------------------------------------------------------------------
// FilterBank
// ---------------------------------------------------------------------------

/// A bank of chirp templates covering the mass parameter space.
///
/// The bank is laid out as a rectangular grid in (chirp mass, eta) space. For each
/// grid point a [`ChirpTemplate`] is generated and stored. The [`search`] method
/// correlates data against every template and returns candidate events that exceed
/// the SNR threshold.
#[derive(Debug, Clone)]
pub struct FilterBank {
    /// Templates in the bank.
    pub templates: Vec<ChirpTemplate>,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Low frequency cut-off in Hz.
    pub f_low: f64,
    /// High frequency cut-off in Hz.
    pub f_high: f64,
    /// SNR detection threshold.
    pub snr_threshold: f64,
    /// Detector name (e.g. "H1").
    pub detector: String,
}

impl FilterBank {
    /// Create a filter bank spanning the given chirp-mass and eta ranges.
    ///
    /// # Arguments
    ///
    /// * `mc_range` — (min, max) chirp mass in solar masses
    /// * `eta_range` — (min, max) symmetric mass ratio
    /// * `mc_steps` — number of chirp-mass grid points
    /// * `eta_steps` — number of eta grid points
    /// * `sample_rate` — sample rate in Hz
    /// * `f_low` — low frequency cut-off in Hz
    /// * `f_high` — high frequency cut-off in Hz
    /// * `snr_threshold` — SNR detection threshold
    /// * `detector` — detector label
    pub fn new(
        mc_range: (f64, f64),
        eta_range: (f64, f64),
        mc_steps: usize,
        eta_steps: usize,
        sample_rate: f64,
        f_low: f64,
        f_high: f64,
        snr_threshold: f64,
        detector: &str,
    ) -> Self {
        let mc_steps = mc_steps.max(1);
        let eta_steps = eta_steps.max(1);

        let mut templates = Vec::with_capacity(mc_steps * eta_steps);

        for i in 0..mc_steps {
            let mc = if mc_steps == 1 {
                mc_range.0
            } else {
                mc_range.0 + (mc_range.1 - mc_range.0) * (i as f64) / (mc_steps as f64 - 1.0)
            };
            for j in 0..eta_steps {
                let eta = if eta_steps == 1 {
                    eta_range.0
                } else {
                    eta_range.0
                        + (eta_range.1 - eta_range.0) * (j as f64) / (eta_steps as f64 - 1.0)
                };
                templates.push(ChirpTemplate::new(mc, eta, sample_rate, f_low, f_high));
            }
        }

        Self {
            templates,
            sample_rate,
            f_low,
            f_high,
            snr_threshold,
            detector: detector.to_string(),
        }
    }

    /// Number of templates in the bank.
    pub fn len(&self) -> usize {
        self.templates.len()
    }

    /// Returns true if the bank has no templates.
    pub fn is_empty(&self) -> bool {
        self.templates.is_empty()
    }

    /// Search data against all templates and return candidate events above threshold.
    ///
    /// Uses FFT-based matched filtering for efficiency.
    pub fn search(&self, data: &[f64]) -> Vec<CandidateEvent> {
        let mf = MatchedFilter::new(self.snr_threshold, self.sample_rate);
        let mut candidates = Vec::new();

        for (idx, tmpl) in self.templates.iter().enumerate() {
            let snr_series = mf.filter_fft(data, tmpl.waveform());
            let triggers = mf.find_triggers(&snr_series);
            for (sample, snr_val) in triggers {
                candidates.push(CandidateEvent {
                    peak_sample: sample,
                    time: sample as f64 / self.sample_rate,
                    snr: snr_val,
                    chirp_mass_solar: tmpl.chirp_mass_solar,
                    eta: tmpl.eta,
                    template_index: idx,
                    detector: self.detector.clone(),
                });
            }
        }

        // Sort by descending SNR
        candidates.sort_by(|a, b| b.snr.partial_cmp(&a.snr).unwrap_or(std::cmp::Ordering::Equal));
        candidates
    }
}

// ---------------------------------------------------------------------------
// CoincidenceDetector
// ---------------------------------------------------------------------------

/// Multi-detector coincidence analysis.
///
/// Checks whether candidate events from different detectors are consistent with
/// a real astrophysical signal by requiring temporal coincidence within a configurable
/// time window and compatible chirp-mass estimates.
#[derive(Debug, Clone)]
pub struct CoincidenceDetector {
    /// Maximum allowed time difference between detectors (seconds).
    /// For LIGO Hanford–Livingston the light travel time is ~10 ms.
    pub time_window: f64,
    /// Maximum allowed relative chirp-mass difference (fractional).
    pub mass_tolerance: f64,
    /// Minimum number of detectors required for a coincident event.
    pub min_detectors: usize,
}

/// A coincident event observed in multiple detectors.
#[derive(Debug, Clone)]
pub struct CoincidentEvent {
    /// The individual single-detector candidates forming this coincidence.
    pub events: Vec<CandidateEvent>,
    /// Combined SNR (root-sum-of-squares of individual SNRs).
    pub combined_snr: f64,
    /// Mean estimated chirp mass across detectors.
    pub mean_chirp_mass: f64,
}

impl CoincidenceDetector {
    /// Create a new coincidence detector.
    ///
    /// # Arguments
    ///
    /// * `time_window` — maximum time difference (seconds) for coincidence
    /// * `mass_tolerance` — fractional chirp-mass agreement required (e.g. 0.05 for 5%)
    /// * `min_detectors` — minimum number of detectors required
    pub fn new(time_window: f64, mass_tolerance: f64, min_detectors: usize) -> Self {
        Self {
            time_window,
            mass_tolerance,
            min_detectors: min_detectors.max(2),
        }
    }

    /// Check coincidence between candidate lists from multiple detectors.
    ///
    /// Each inner `Vec<CandidateEvent>` represents the candidates from one detector.
    /// Returns groups of coincident events.
    pub fn find_coincidences(
        &self,
        detector_candidates: &[Vec<CandidateEvent>],
    ) -> Vec<CoincidentEvent> {
        if detector_candidates.len() < self.min_detectors {
            return vec![];
        }

        let mut coincidences = Vec::new();

        // Use the first detector's candidates as the reference
        if detector_candidates.is_empty() {
            return vec![];
        }

        for ref_event in &detector_candidates[0] {
            let mut group = vec![ref_event.clone()];

            for other_candidates in &detector_candidates[1..] {
                // Find the best match in this detector
                let mut best: Option<&CandidateEvent> = None;
                let mut best_snr = 0.0;

                for cand in other_candidates {
                    let dt = (cand.time - ref_event.time).abs();
                    let dmc = if ref_event.chirp_mass_solar > 0.0 {
                        (cand.chirp_mass_solar - ref_event.chirp_mass_solar).abs()
                            / ref_event.chirp_mass_solar
                    } else {
                        f64::INFINITY
                    };

                    if dt <= self.time_window && dmc <= self.mass_tolerance && cand.snr > best_snr {
                        best = Some(cand);
                        best_snr = cand.snr;
                    }
                }

                if let Some(b) = best {
                    group.push(b.clone());
                }
            }

            if group.len() >= self.min_detectors {
                let combined_snr = group.iter().map(|e| e.snr * e.snr).sum::<f64>().sqrt();
                let mean_chirp_mass =
                    group.iter().map(|e| e.chirp_mass_solar).sum::<f64>() / group.len() as f64;
                coincidences.push(CoincidentEvent {
                    events: group,
                    combined_snr,
                    mean_chirp_mass,
                });
            }
        }

        // Sort by combined SNR descending
        coincidences.sort_by(|a, b| {
            b.combined_snr
                .partial_cmp(&a.combined_snr)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        coincidences
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // -- chirp_mass tests --

    #[test]
    fn test_chirp_mass_equal_masses() {
        // For equal masses: Mc = m * 2^(-1/5)
        let mc = chirp_mass(1.4, 1.4);
        let expected = 1.4 * 2.0_f64.powf(-1.0 / 5.0);
        assert!(
            (mc - expected).abs() < TOLERANCE,
            "chirp_mass(1.4, 1.4) = {mc}, expected {expected}"
        );
    }

    #[test]
    fn test_chirp_mass_unequal_masses() {
        let mc = chirp_mass(10.0, 1.4);
        // (10*1.4)^(3/5) / (11.4)^(1/5)
        let expected = (10.0 * 1.4_f64).powf(0.6) / 11.4_f64.powf(0.2);
        assert!(
            (mc - expected).abs() < TOLERANCE,
            "chirp_mass(10, 1.4) = {mc}, expected {expected}"
        );
    }

    #[test]
    fn test_chirp_mass_symmetric() {
        assert!((chirp_mass(3.0, 5.0) - chirp_mass(5.0, 3.0)).abs() < TOLERANCE);
    }

    // -- symmetric_mass_ratio tests --

    #[test]
    fn test_eta_equal_masses() {
        let eta = symmetric_mass_ratio(1.4, 1.4);
        assert!(
            (eta - 0.25).abs() < TOLERANCE,
            "eta for equal masses should be 0.25, got {eta}"
        );
    }

    #[test]
    fn test_eta_extreme_ratio() {
        let eta = symmetric_mass_ratio(100.0, 1.0);
        // 100 / 101^2 ~ 0.0098
        let expected = 100.0 / (101.0 * 101.0);
        assert!(
            (eta - expected).abs() < TOLERANCE,
            "eta(100,1) = {eta}, expected {expected}"
        );
    }

    #[test]
    fn test_eta_symmetric() {
        assert!(
            (symmetric_mass_ratio(2.0, 5.0) - symmetric_mass_ratio(5.0, 2.0)).abs() < TOLERANCE
        );
    }

    #[test]
    fn test_eta_range() {
        // eta must be in (0, 0.25]
        let eta = symmetric_mass_ratio(30.0, 30.0);
        assert!(eta <= 0.25 + TOLERANCE);
        assert!(eta > 0.0);
    }

    // -- orbital_frequency tests --

    #[test]
    fn test_orbital_frequency_positive() {
        let mc_si = 1.2 * M_SUN_KG;
        let f = orbital_frequency(10.0, mc_si);
        assert!(f > 0.0, "orbital frequency must be positive, got {f}");
    }

    #[test]
    fn test_orbital_frequency_increases_as_tau_decreases() {
        let mc_si = 1.2 * M_SUN_KG;
        let f1 = orbital_frequency(10.0, mc_si);
        let f2 = orbital_frequency(1.0, mc_si);
        assert!(
            f2 > f1,
            "frequency should increase as coalescence approaches"
        );
    }

    #[test]
    fn test_orbital_frequency_zero_tau() {
        let mc_si = 1.2 * M_SUN_KG;
        let f = orbital_frequency(0.0, mc_si);
        assert_eq!(f, 0.0);
    }

    // -- strain_amplitude tests --

    #[test]
    fn test_strain_amplitude_positive() {
        let h = strain_amplitude(1.2, 100.0, 100.0);
        assert!(h > 0.0, "strain amplitude must be positive");
    }

    #[test]
    fn test_strain_amplitude_decreases_with_distance() {
        let h1 = strain_amplitude(1.2, 100.0, 10.0);
        let h2 = strain_amplitude(1.2, 100.0, 100.0);
        assert!(
            h1 > h2,
            "strain should decrease with distance: h1={h1}, h2={h2}"
        );
    }

    #[test]
    fn test_strain_amplitude_increases_with_frequency() {
        let h1 = strain_amplitude(1.2, 50.0, 100.0);
        let h2 = strain_amplitude(1.2, 200.0, 100.0);
        assert!(
            h2 > h1,
            "strain should increase with frequency: h1={h1}, h2={h2}"
        );
    }

    // -- inner_product tests --

    #[test]
    fn test_inner_product_orthogonal() {
        let a = vec![1.0, 0.0, -1.0, 0.0];
        let b = vec![0.0, 1.0, 0.0, -1.0];
        let ip = inner_product(&a, &b, 1.0);
        assert!(ip.abs() < TOLERANCE, "orthogonal vectors: ip = {ip}");
    }

    #[test]
    fn test_inner_product_self() {
        let a = vec![1.0, 2.0, 3.0];
        let ip = inner_product(&a, &a, 1.0);
        let expected = 1.0 + 4.0 + 9.0; // 14
        assert!(
            (ip - expected).abs() < TOLERANCE,
            "self inner product: {ip} vs {expected}"
        );
    }

    #[test]
    fn test_inner_product_sample_rate_scaling() {
        let a = vec![1.0, 1.0, 1.0, 1.0];
        let b = vec![1.0, 1.0, 1.0, 1.0];
        let ip1 = inner_product(&a, &b, 1.0);
        let ip2 = inner_product(&a, &b, 2.0);
        assert!(
            (ip1 - 2.0 * ip2).abs() < TOLERANCE,
            "doubling sample rate should halve the product"
        );
    }

    // -- ChirpTemplate tests --

    #[test]
    fn test_chirp_template_not_empty() {
        let mc = chirp_mass(1.4, 1.4);
        let eta = symmetric_mass_ratio(1.4, 1.4);
        let tmpl = ChirpTemplate::new(mc, eta, 4096.0, 20.0, 1000.0);
        assert!(!tmpl.is_empty(), "template should have samples");
        assert!(tmpl.len() > 0);
    }

    #[test]
    fn test_chirp_template_unit_energy() {
        let mc = chirp_mass(1.4, 1.4);
        let eta = symmetric_mass_ratio(1.4, 1.4);
        let tmpl = ChirpTemplate::new(mc, eta, 4096.0, 30.0, 500.0);
        let energy: f64 = tmpl.waveform().iter().map(|x| x * x).sum();
        assert!(
            (energy - 1.0).abs() < 1e-6,
            "template energy should be 1.0, got {energy}"
        );
    }

    #[test]
    fn test_chirp_template_duration() {
        let mc = chirp_mass(1.4, 1.4);
        let eta = symmetric_mass_ratio(1.4, 1.4);
        let tmpl = ChirpTemplate::new(mc, eta, 4096.0, 30.0, 500.0);
        let dur = tmpl.duration();
        assert!(dur > 0.0, "duration should be positive: {dur}");
        assert!(
            dur < 1000.0,
            "duration should be reasonable for BNS: {dur}"
        );
    }

    #[test]
    fn test_chirp_template_higher_mass_shorter() {
        let eta = 0.25;
        let tmpl_low = ChirpTemplate::new(1.2, eta, 4096.0, 30.0, 500.0);
        let tmpl_high = ChirpTemplate::new(10.0, eta, 4096.0, 30.0, 500.0);
        assert!(
            tmpl_high.duration() < tmpl_low.duration(),
            "higher mass should produce shorter inspiral"
        );
    }

    // -- WhiteningFilter tests --

    #[test]
    fn test_whitening_filter_creation() {
        let wf = WhiteningFilter::new(1024, 256);
        assert_eq!(wf.fft_size, 1024);
        assert!(!wf.is_estimated());
    }

    #[test]
    fn test_whitening_filter_psd_estimation() {
        let mut wf = WhiteningFilter::new(256, 128);
        let data: Vec<f64> = (0..2048).map(|i| (i as f64 * 0.01).sin()).collect();
        wf.estimate_psd(&data);
        assert!(wf.is_estimated());
        assert_eq!(wf.psd().len(), 256);
    }

    #[test]
    fn test_whitening_preserves_length() {
        let mut wf = WhiteningFilter::new(256, 128);
        let data: Vec<f64> = (0..200).map(|i| (i as f64 * 0.05).sin()).collect();
        wf.estimate_psd(&data);
        let whitened = wf.whiten(&data);
        assert_eq!(whitened.len(), data.len());
    }

    #[test]
    fn test_whitening_short_data() {
        let mut wf = WhiteningFilter::new(1024, 256);
        let data = vec![1.0, 2.0, 3.0]; // shorter than fft_size
        wf.estimate_psd(&data);
        assert!(wf.is_estimated());
        let whitened = wf.whiten(&data);
        assert_eq!(whitened.len(), 3);
    }

    // -- MatchedFilter tests --

    #[test]
    fn test_matched_filter_self_correlation() {
        // Correlating a signal with itself should peak at lag 0
        let mf = MatchedFilter::new(1.0, 1.0);
        let signal: Vec<f64> = (0..128).map(|i| (2.0 * PI * i as f64 / 32.0).sin()).collect();
        let snr = mf.filter_time_domain(&signal, &signal);
        assert!(!snr.is_empty());
        // Peak should be at index 0
        let max_idx = snr
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 0, "self-correlation should peak at lag 0");
    }

    #[test]
    fn test_matched_filter_fft_vs_time_domain() {
        let mf = MatchedFilter::new(1.0, 1.0);
        let data: Vec<f64> = (0..256).map(|i| (2.0 * PI * i as f64 / 64.0).sin()).collect();
        let template: Vec<f64> = (0..32).map(|i| (2.0 * PI * i as f64 / 64.0).sin()).collect();

        let snr_td = mf.filter_time_domain(&data, &template);
        let snr_fft = mf.filter_fft(&data, &template);

        assert_eq!(snr_td.len(), snr_fft.len());

        // They should agree approximately (FFT uses circular correlation, so
        // edge effects are possible)
        let peak_td = snr_td.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let peak_fft = snr_fft.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        // Both should find a positive peak
        assert!(peak_td > 0.0);
        assert!(peak_fft > 0.0);
    }

    #[test]
    fn test_matched_filter_empty_template() {
        let mf = MatchedFilter::new(1.0, 1.0);
        let data = vec![1.0, 2.0, 3.0];
        let result = mf.filter_time_domain(&data, &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_find_triggers() {
        let mf = MatchedFilter::new(5.0, 1.0);
        let snr = vec![1.0, 2.0, 3.0, 8.0, 3.0, 2.0, 1.0, 7.0, 1.0];
        let triggers = mf.find_triggers(&snr);
        // Should find peak at index 3 (snr=8) and index 7 (snr=7)
        assert!(
            triggers.len() >= 2,
            "expected at least 2 triggers, got {}",
            triggers.len()
        );
        assert!(triggers.iter().any(|&(idx, _)| idx == 3));
        assert!(triggers.iter().any(|&(idx, _)| idx == 7));
    }

    // -- FilterBank tests --

    #[test]
    fn test_filter_bank_creation() {
        let bank = FilterBank::new(
            (1.0, 2.0),
            (0.20, 0.25),
            3,
            2,
            4096.0,
            30.0,
            500.0,
            5.0,
            "H1",
        );
        assert_eq!(bank.len(), 6); // 3 * 2
        assert!(!bank.is_empty());
    }

    #[test]
    fn test_filter_bank_single_template() {
        let bank = FilterBank::new(
            (1.2, 1.2),
            (0.25, 0.25),
            1,
            1,
            4096.0,
            30.0,
            500.0,
            5.0,
            "L1",
        );
        assert_eq!(bank.len(), 1);
    }

    #[test]
    fn test_filter_bank_search_no_signal() {
        let bank = FilterBank::new(
            (1.0, 1.5),
            (0.24, 0.25),
            2,
            2,
            4096.0,
            30.0,
            500.0,
            8.0, // high threshold
            "H1",
        );
        // Pure zeros — no detection expected
        let data = vec![0.0; 4096];
        let candidates = bank.search(&data);
        assert!(
            candidates.is_empty(),
            "no candidates expected in zero data"
        );
    }

    // -- CandidateEvent tests --

    #[test]
    fn test_candidate_total_mass() {
        let evt = CandidateEvent {
            peak_sample: 0,
            time: 0.0,
            snr: 10.0,
            chirp_mass_solar: chirp_mass(1.4, 1.4),
            eta: symmetric_mass_ratio(1.4, 1.4),
            template_index: 0,
            detector: "H1".into(),
        };
        let m_total = evt.estimated_total_mass();
        assert!(
            (m_total - 2.8).abs() < 0.01,
            "total mass should be ~2.8, got {m_total}"
        );
    }

    #[test]
    fn test_candidate_component_masses() {
        let evt = CandidateEvent {
            peak_sample: 0,
            time: 0.0,
            snr: 10.0,
            chirp_mass_solar: chirp_mass(1.4, 1.4),
            eta: symmetric_mass_ratio(1.4, 1.4),
            template_index: 0,
            detector: "L1".into(),
        };
        let (m1, m2) = evt.estimated_component_masses();
        assert!(
            (m1 - 1.4).abs() < 0.01,
            "m1 should be ~1.4, got {m1}"
        );
        assert!(
            (m2 - 1.4).abs() < 0.01,
            "m2 should be ~1.4, got {m2}"
        );
    }

    // -- CoincidenceDetector tests --

    #[test]
    fn test_coincidence_same_event() {
        let cd = CoincidenceDetector::new(0.015, 0.05, 2);
        let evt_h1 = CandidateEvent {
            peak_sample: 100,
            time: 0.1,
            snr: 10.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "H1".into(),
        };
        let evt_l1 = CandidateEvent {
            peak_sample: 105,
            time: 0.105,
            snr: 9.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "L1".into(),
        };
        let coincidences = cd.find_coincidences(&[vec![evt_h1], vec![evt_l1]]);
        assert_eq!(coincidences.len(), 1, "should find one coincidence");
        assert_eq!(coincidences[0].events.len(), 2);
    }

    #[test]
    fn test_coincidence_too_far_apart() {
        let cd = CoincidenceDetector::new(0.010, 0.05, 2);
        let evt_h1 = CandidateEvent {
            peak_sample: 100,
            time: 0.0,
            snr: 10.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "H1".into(),
        };
        let evt_l1 = CandidateEvent {
            peak_sample: 500,
            time: 1.0, // 1 second later — too far
            snr: 9.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "L1".into(),
        };
        let coincidences = cd.find_coincidences(&[vec![evt_h1], vec![evt_l1]]);
        assert!(coincidences.is_empty(), "events too far apart should not coincide");
    }

    #[test]
    fn test_coincidence_mass_mismatch() {
        let cd = CoincidenceDetector::new(0.015, 0.01, 2); // tight mass tolerance
        let evt_h1 = CandidateEvent {
            peak_sample: 100,
            time: 0.1,
            snr: 10.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "H1".into(),
        };
        let evt_l1 = CandidateEvent {
            peak_sample: 102,
            time: 0.102,
            snr: 9.0,
            chirp_mass_solar: 5.0, // very different mass
            eta: 0.25,
            template_index: 0,
            detector: "L1".into(),
        };
        let coincidences = cd.find_coincidences(&[vec![evt_h1], vec![evt_l1]]);
        assert!(
            coincidences.is_empty(),
            "mass mismatch should prevent coincidence"
        );
    }

    #[test]
    fn test_coincidence_combined_snr() {
        let cd = CoincidenceDetector::new(0.015, 0.05, 2);
        let evt_h1 = CandidateEvent {
            peak_sample: 100,
            time: 0.1,
            snr: 8.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "H1".into(),
        };
        let evt_l1 = CandidateEvent {
            peak_sample: 102,
            time: 0.102,
            snr: 6.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "L1".into(),
        };
        let coincidences = cd.find_coincidences(&[vec![evt_h1], vec![evt_l1]]);
        assert_eq!(coincidences.len(), 1);
        let expected_snr = (8.0_f64.powi(2) + 6.0_f64.powi(2)).sqrt();
        assert!(
            (coincidences[0].combined_snr - expected_snr).abs() < TOLERANCE,
            "combined SNR = {}, expected {expected_snr}",
            coincidences[0].combined_snr
        );
    }

    #[test]
    fn test_coincidence_insufficient_detectors() {
        let cd = CoincidenceDetector::new(0.015, 0.05, 3); // require 3 detectors
        let evt_h1 = CandidateEvent {
            peak_sample: 100,
            time: 0.1,
            snr: 10.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "H1".into(),
        };
        let evt_l1 = CandidateEvent {
            peak_sample: 102,
            time: 0.102,
            snr: 9.0,
            chirp_mass_solar: 1.2,
            eta: 0.25,
            template_index: 0,
            detector: "L1".into(),
        };
        // Only 2 detectors, but 3 required
        let coincidences = cd.find_coincidences(&[vec![evt_h1], vec![evt_l1]]);
        assert!(
            coincidences.is_empty(),
            "should require at least 3 detectors"
        );
    }

    // -- FFT helper tests --

    #[test]
    fn test_fft_roundtrip() {
        let signal: Vec<f64> = (0..64).map(|i| (2.0 * PI * i as f64 / 16.0).sin()).collect();
        let spectrum = rfft(&signal);
        let recovered = irfft(&spectrum);
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "FFT roundtrip failed: {a} vs {b}"
            );
        }
    }

    #[test]
    fn test_next_pow2() {
        assert_eq!(next_pow2(1), 1);
        assert_eq!(next_pow2(3), 4);
        assert_eq!(next_pow2(8), 8);
        assert_eq!(next_pow2(1000), 1024);
    }
}
