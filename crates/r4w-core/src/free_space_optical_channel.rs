//! Atmospheric turbulence and scattering model for Free-Space Optical (FSO) communication links.
//!
//! This module provides a comprehensive channel model for FSO links, including:
//! - **Rytov variance** for quantifying scintillation (turbulence) strength
//! - **Log-normal fading** for weak turbulence conditions
//! - **Gamma-gamma fading** for moderate-to-strong turbulence
//! - **Beer-Lambert atmospheric attenuation** (visibility-based)
//! - **Geometric and pointing-error losses**
//! - **Fried parameter** (atmospheric coherence length)
//! - **Link budget** computation
//! - **BER estimation** for OOK and PPM under turbulence
//! - **Outage probability** computation
//!
//! # Example
//!
//! ```
//! use r4w_core::free_space_optical_channel::{FsoChannel, TurbulenceRegime};
//!
//! // 1 km link, 1550 nm wavelength, 10 cm aperture
//! let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
//! let regime = ch.turbulence_regime(1e-14);
//! assert!(matches!(regime, TurbulenceRegime::Weak));
//!
//! let att_db = ch.atmospheric_attenuation_db(10.0); // 10 km visibility
//! assert!(att_db < 0.0); // attenuation is negative in dB (loss)
//!
//! let p_rx = ch.link_budget_dbm(0.0, 3.0, 3.0, 10.0, 1e-14);
//! // p_rx is the received power in dBm
//! assert!(p_rx < 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Turbulence strength classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurbulenceRegime {
    /// Rytov variance < 0.3
    Weak,
    /// 0.3 <= Rytov variance < 5.0
    Moderate,
    /// Rytov variance >= 5.0
    Strong,
}

/// Modulation format for BER estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FsoModulation {
    /// On-Off Keying (intensity modulation / direct detection).
    Ook,
    /// Pulse Position Modulation with the given order M (must be power of 2).
    Ppm(u32),
}

/// Free-Space Optical channel model.
///
/// Parameterised by the fixed link geometry; turbulence strength `Cn2` is
/// supplied per-method so the same struct can be reused across conditions.
#[derive(Debug, Clone)]
pub struct FsoChannel {
    /// Link distance in metres.
    pub distance: f64,
    /// Optical wavelength in metres (e.g. 1550 × 10⁻⁹).
    pub wavelength: f64,
    /// Receiver aperture diameter in metres.
    pub aperture: f64,
    /// Transmitter beam divergence half-angle in radians (default 1 mrad).
    pub beam_divergence: f64,
    /// Pointing-error jitter standard deviation in radians (default 0).
    pub jitter_rad: f64,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl FsoChannel {
    /// Create a channel with the given link distance (m), wavelength (m), and
    /// receiver aperture diameter (m). Beam divergence defaults to 1 mrad and
    /// jitter to zero.
    pub fn new(distance: f64, wavelength: f64, aperture: f64) -> Self {
        Self {
            distance,
            wavelength,
            aperture,
            beam_divergence: 1e-3,
            jitter_rad: 0.0,
        }
    }

    /// Set the transmitter beam divergence half-angle (radians).
    pub fn with_beam_divergence(mut self, div: f64) -> Self {
        self.beam_divergence = div;
        self
    }

    /// Set the pointing-error jitter standard deviation (radians).
    pub fn with_jitter(mut self, jitter: f64) -> Self {
        self.jitter_rad = jitter;
        self
    }

    // -- Turbulence metrics ------------------------------------------------

    /// Optical wave number k = 2π / λ.
    pub fn wavenumber(&self) -> f64 {
        2.0 * PI / self.wavelength
    }

    /// Rytov variance σ²_R for a plane wave propagating over `distance`
    /// through turbulence with refractive-index structure constant `cn2`
    /// (m^{-2/3}).
    ///
    /// σ²_R = 1.23 · Cn² · k^{7/6} · L^{11/6}
    pub fn rytov_variance(&self, cn2: f64) -> f64 {
        let k = self.wavenumber();
        1.23 * cn2 * k.powf(7.0 / 6.0) * self.distance.powf(11.0 / 6.0)
    }

    /// Classify the turbulence regime for the given `cn2`.
    pub fn turbulence_regime(&self, cn2: f64) -> TurbulenceRegime {
        let sigma_r2 = self.rytov_variance(cn2);
        if sigma_r2 < 0.3 {
            TurbulenceRegime::Weak
        } else if sigma_r2 < 5.0 {
            TurbulenceRegime::Moderate
        } else {
            TurbulenceRegime::Strong
        }
    }

    /// Fried parameter r₀ (atmospheric coherence length) in metres.
    ///
    /// r₀ = (0.423 · k² · Cn² · L)^{-3/5}
    pub fn fried_parameter(&self, cn2: f64) -> f64 {
        let k = self.wavenumber();
        (0.423 * k * k * cn2 * self.distance).powf(-3.0 / 5.0)
    }

    // -- Fading models -----------------------------------------------------

    /// Log-normal scintillation index σ²_I (intensity variance) under weak
    /// turbulence, with aperture averaging.
    ///
    /// σ²_I = exp(σ²_ln_I) - 1  where σ²_ln_I ≈ σ²_R · A_f
    ///
    /// Aperture averaging factor A_f = [1 + 1.062 (D/2)² / (λL)]^{-7/6}
    /// (Andrews & Phillips, Eq. 10.78 approximation for a plane wave)
    pub fn log_normal_scintillation_index(&self, cn2: f64) -> f64 {
        let sigma_r2 = self.rytov_variance(cn2);
        let af = self.aperture_averaging_factor();
        let sigma_ln = sigma_r2 * af;
        sigma_ln.exp() - 1.0
    }

    /// Aperture averaging factor (reduction of scintillation due to
    /// finite receiver aperture).  Returns a value in (0, 1].
    pub fn aperture_averaging_factor(&self) -> f64 {
        let d = self.aperture;
        let fresnel_sq = self.wavelength * self.distance; // λL
        let ratio = (d / 2.0) * (d / 2.0) / fresnel_sq;
        (1.0 + 1.062 * ratio).powf(-7.0 / 6.0)
    }

    /// Gamma-gamma fading parameters (α, β) for moderate-to-strong
    /// turbulence.
    ///
    /// These are the shape parameters of the Gamma-Gamma distribution
    /// that models the irradiance fluctuations.
    ///
    /// α = [exp(σ²_x) - 1]⁻¹,  β = [exp(σ²_y) - 1]⁻¹
    ///
    /// where σ²_x and σ²_y are the large-scale and small-scale
    /// log-irradiance variances (Andrews & Phillips, spherical wave
    /// approximation).
    pub fn gamma_gamma_params(&self, cn2: f64) -> (f64, f64) {
        let sigma_r2 = self.rytov_variance(cn2);

        // Spherical-wave model (Andrews & Phillips, Laser Beam Propagation
        // through Random Media, 2nd ed.)
        let sigma_x2 =
            0.49 * sigma_r2 / (1.0 + 1.11 * sigma_r2.powf(6.0 / 5.0)).powf(7.0 / 6.0);
        let sigma_y2 =
            0.51 * sigma_r2 / (1.0 + 0.69 * sigma_r2.powf(6.0 / 5.0)).powf(5.0 / 6.0);

        let alpha = 1.0 / (sigma_x2.exp() - 1.0);
        let beta = 1.0 / (sigma_y2.exp() - 1.0);
        (alpha, beta)
    }

    // -- Atmospheric attenuation -------------------------------------------

    /// Beer-Lambert atmospheric attenuation in dB for a given meteorological
    /// visibility `v_km` (in **kilometres**).
    ///
    /// Attenuation coefficient (per km):
    ///   σ_a = (3.91 / V) · (λ / 550 nm)^{-q}
    ///
    /// where q is the Kim model size-distribution exponent.
    /// Returns a **negative** value (loss).
    pub fn atmospheric_attenuation_db(&self, v_km: f64) -> f64 {
        let lambda_nm = self.wavelength * 1e9;
        let q = kim_exponent(v_km, lambda_nm);
        let sigma_per_km = (3.91 / v_km) * (lambda_nm / 550.0).powf(-q);
        let distance_km = self.distance / 1000.0;
        // Transmission T = exp(-σ · L), in dB: 10 log10(T)
        let t = (-sigma_per_km * distance_km).exp();
        10.0 * t.log10()
    }

    // -- Geometric / pointing-error losses ---------------------------------

    /// Geometric loss in dB due to beam spreading over distance.
    ///
    /// The beam footprint at the receiver has radius ≈ θ_div · L.
    /// Geometric loss = (D_rx / (2 · θ · L))² , converted to dB.
    /// Returns a **negative** value (loss).
    pub fn geometric_loss_db(&self) -> f64 {
        let beam_radius = self.beam_divergence * self.distance;
        if beam_radius <= 0.0 {
            return 0.0;
        }
        let ratio = (self.aperture / 2.0) / beam_radius;
        // Fraction of power captured (capped at 1.0 if aperture > beam)
        let capture = if ratio >= 1.0 { 1.0 } else { ratio * ratio };
        10.0 * capture.log10()
    }

    /// Pointing-error (misalignment) loss in dB.
    ///
    /// Modelled as a Gaussian jitter with std-dev `jitter_rad` on boresight.
    /// Average loss factor ≈ w_eq² / (w_eq² + 4 σ_s²)  where
    /// w_eq = beam_divergence · L (equivalent beam radius at receiver)
    /// and σ_s = jitter_rad · L  (spatial jitter at receiver).
    ///
    /// Returns a **negative** value (loss), or 0 dB if jitter is zero.
    pub fn pointing_error_loss_db(&self) -> f64 {
        if self.jitter_rad <= 0.0 {
            return 0.0;
        }
        let w_eq = self.beam_divergence * self.distance;
        let sigma_s = self.jitter_rad * self.distance;
        let factor = w_eq * w_eq / (w_eq * w_eq + 4.0 * sigma_s * sigma_s);
        10.0 * factor.log10()
    }

    // -- Link budget -------------------------------------------------------

    /// Compute the received power in dBm.
    ///
    /// `p_tx_dbm`  – transmit power (dBm)
    /// `g_tx_db`   – transmitter gain (dB)
    /// `g_rx_db`   – receiver gain (dB)
    /// `v_km`      – meteorological visibility (km)
    /// `cn2`       – refractive-index structure constant (m^{-2/3})
    ///
    /// P_rx = P_tx + G_tx + G_rx + L_geo + L_atm + L_pointing + L_scint
    ///
    /// where L_scint is the mean scintillation loss (≈ −σ²_I / 2 in dB for
    /// log-normal fading).
    pub fn link_budget_dbm(
        &self,
        p_tx_dbm: f64,
        g_tx_db: f64,
        g_rx_db: f64,
        v_km: f64,
        cn2: f64,
    ) -> f64 {
        let l_geo = self.geometric_loss_db();
        let l_atm = self.atmospheric_attenuation_db(v_km);
        let l_point = self.pointing_error_loss_db();
        // Mean scintillation loss for log-normal model
        let si = self.log_normal_scintillation_index(cn2);
        let l_scint = -10.0 * (1.0 + si).log10() / 2.0;

        p_tx_dbm + g_tx_db + g_rx_db + l_geo + l_atm + l_point + l_scint
    }

    // -- BER estimation ----------------------------------------------------

    /// Estimate BER for the given modulation under turbulence-free AWGN,
    /// at the specified electrical SNR (linear, **not** in dB).
    pub fn ber_awgn(&self, snr_linear: f64, modulation: FsoModulation) -> f64 {
        match modulation {
            FsoModulation::Ook => {
                // BER_OOK = 0.5 · erfc(√(SNR / 2))
                0.5 * erfc((snr_linear / 2.0).sqrt())
            }
            FsoModulation::Ppm(m) => {
                // BER_PPM ≈ (M / 2) · Q(√(SNR · log2(M)))
                let log2m = (m as f64).log2();
                let arg = (snr_linear * log2m).sqrt();
                let q_val = 0.5 * erfc(arg / std::f64::consts::SQRT_2);
                (m as f64 / 2.0) * q_val / log2m
            }
        }
    }

    /// Average BER under log-normal turbulence (weak), evaluated by
    /// Gauss-Hermite quadrature.
    ///
    /// `snr_linear` – mean electrical SNR (linear).
    /// `cn2`        – Cn² value.
    pub fn ber_log_normal(
        &self,
        snr_linear: f64,
        cn2: f64,
        modulation: FsoModulation,
    ) -> f64 {
        // σ²_ln_I from the Rytov variance with aperture averaging
        let sigma_r2 = self.rytov_variance(cn2);
        let af = self.aperture_averaging_factor();
        let sigma_ln2 = sigma_r2 * af;
        if sigma_ln2 < 1e-15 {
            return self.ber_awgn(snr_linear, modulation);
        }
        let sigma_ln = sigma_ln2.sqrt();

        // 20-point Gauss-Hermite quadrature
        gauss_hermite_integrate(20, |x| {
            // irradiance I = exp(2σx - σ²)  (log-normal with unit mean)
            let irradiance = (2.0 * sigma_ln * x - sigma_ln2).exp();
            // instantaneous SNR scales linearly with irradiance
            self.ber_awgn(snr_linear * irradiance, modulation)
        })
    }

    /// Average BER under gamma-gamma turbulence (moderate-to-strong),
    /// evaluated by Gauss-Laguerre quadrature.
    ///
    /// Uses the mapping: instantaneous SNR scales with irradiance,
    /// and the irradiance PDF is gamma-gamma(α,β).
    pub fn ber_gamma_gamma(
        &self,
        snr_linear: f64,
        cn2: f64,
        modulation: FsoModulation,
    ) -> f64 {
        let (alpha, beta) = self.gamma_gamma_params(cn2);
        // Approximate by generating quadrature samples of the GG distribution.
        // We use a 32-point Gauss-Laguerre-based approach where we split the
        // GG integral into the product of two gamma integrals.
        //
        // Simplification: for each gamma draw of x ~ Gamma(α), the conditional
        // distribution of I|x is Gamma(β, x/β) with mean x.
        // We approximate this with nested Gauss-Laguerre quadrature (8×8).
        let n = 8;
        let (nodes, weights) = gauss_laguerre_nodes(n);

        let mut total = 0.0;
        for i in 0..n {
            // x ~ Gamma(α, 1/α) → mean 1
            let x_val = nodes[i] / alpha; // change of variable for Gamma(α)
            let w_x = weights[i] * gamma_laguerre_weight(alpha, nodes[i]);
            for j in 0..n {
                let y_val = nodes[j] / beta;
                let w_y = weights[j] * gamma_laguerre_weight(beta, nodes[j]);
                let irradiance = x_val * y_val;
                let ber = self.ber_awgn(snr_linear * irradiance, modulation);
                total += w_x * w_y * ber;
            }
        }
        total.max(0.0).min(0.5)
    }

    // -- Outage probability ------------------------------------------------

    /// Probability that the instantaneous received SNR falls below the
    /// threshold `snr_threshold` under log-normal fading.
    ///
    /// P_out = Φ( (ln(γ_th) - ln(γ_mean) + σ²_I/2 ) / σ_ln )
    pub fn outage_probability_log_normal(
        &self,
        snr_mean_linear: f64,
        snr_threshold_linear: f64,
        cn2: f64,
    ) -> f64 {
        let sigma_r2 = self.rytov_variance(cn2);
        let af = self.aperture_averaging_factor();
        let sigma_ln2 = sigma_r2 * af;
        if sigma_ln2 < 1e-15 {
            // No fading – deterministic comparison.
            return if snr_mean_linear >= snr_threshold_linear {
                0.0
            } else {
                1.0
            };
        }
        let sigma_ln = sigma_ln2.sqrt();
        let arg =
            (snr_threshold_linear.ln() - snr_mean_linear.ln() + sigma_ln2 / 2.0) / sigma_ln;
        // Φ(x) = 0.5 · erfc(-x / √2)
        0.5 * erfc(-arg / std::f64::consts::SQRT_2)
    }

    // -- Complex IQ fading sample ------------------------------------------

    /// Apply a single log-normal fading realisation to a complex IQ sample.
    ///
    /// The sample is multiplied by √I where I is drawn from a log-normal
    /// distribution with the given `sigma_ln2` (log-irradiance variance).
    ///
    /// `rng_uniform` must supply a value in (0, 1]. We use the Box-Muller
    /// transform with two uniform draws (`u1`, `u2`).
    pub fn apply_log_normal_fading(
        sample: (f64, f64),
        sigma_ln2: f64,
        u1: f64,
        u2: f64,
    ) -> (f64, f64) {
        // Box-Muller
        let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
        let sigma_ln = sigma_ln2.sqrt();
        // I = exp(σ·z - σ²/2)  → unit mean
        let irradiance = (sigma_ln * z - sigma_ln2 / 2.0).exp();
        let amp = irradiance.sqrt();
        (sample.0 * amp, sample.1 * amp)
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Kim model for the wavelength-dependent exponent `q` in the
/// Beer-Lambert visibility formula.
fn kim_exponent(v_km: f64, _lambda_nm: f64) -> f64 {
    if v_km > 50.0 {
        1.6
    } else if v_km > 6.0 {
        1.3
    } else if v_km > 1.0 {
        0.16 * v_km + 0.34
    } else if v_km > 0.5 {
        v_km - 0.5
    } else {
        0.0
    }
}

/// Complementary error function (Abramowitz & Stegun 7.1.26 rational
/// approximation, max error < 1.5 × 10⁻⁷).
fn erfc(x: f64) -> f64 {
    if x >= 0.0 {
        erfc_positive(x)
    } else {
        2.0 - erfc_positive(-x)
    }
}

fn erfc_positive(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Gauss-Hermite quadrature: ∫_{-∞}^{∞} f(x)·exp(-x²) dx ≈ Σ w_i · f(x_i)
///
/// We return the integral normalised by √π (i.e. ∫ f(x) · (1/√π) exp(-x²) dx)
/// which is the expectation under a standard normal when f is expressed in
/// terms of the Hermite variable.
fn gauss_hermite_integrate<F: Fn(f64) -> f64>(n: usize, f: F) -> f64 {
    let (nodes, weights) = gauss_hermite_nodes(n);
    let mut sum = 0.0;
    for i in 0..nodes.len() {
        sum += weights[i] * f(nodes[i]);
    }
    sum
}

/// 20-point Gauss-Hermite nodes and weights (normalised so Σ w_i = 1,
/// i.e. weights include the 1/√π factor).
fn gauss_hermite_nodes(n: usize) -> (Vec<f64>, Vec<f64>) {
    // Pre-computed for n = 20 (from Abramowitz & Stegun, Table 25.10).
    // We only store the positive half and mirror.
    debug_assert!(n == 20, "Only n=20 supported");
    let half_nodes: &[f64] = &[
        0.2453407083009,
        0.7374737285454,
        1.2340762153953,
        1.7385377121166,
        2.2549740020893,
        2.7888060584281,
        3.3478545673832,
        3.9447640401156,
        4.6036824495507,
        5.3874808900112,
    ];
    let half_weights: &[f64] = &[
        4.622436696006e-01,
        2.866755053628e-01,
        1.090172060200e-01,
        2.481052088746e-02,
        3.243773342238e-03,
        2.283386360163e-04,
        7.802556478532e-06,
        1.086069370769e-07,
        4.399340992273e-10,
        2.229393645534e-13,
    ];
    // The weights above include exp(x²) compensation and 1/√π, and Σ = 1.
    let inv_sqrt_pi = 1.0 / PI.sqrt();
    let mut nodes = Vec::with_capacity(20);
    let mut weights = Vec::with_capacity(20);
    for i in (0..10).rev() {
        nodes.push(-half_nodes[i]);
        weights.push(half_weights[i] * inv_sqrt_pi);
    }
    for i in 0..10 {
        nodes.push(half_nodes[i]);
        weights.push(half_weights[i] * inv_sqrt_pi);
    }
    (nodes, weights)
}

/// Simple Gauss-Laguerre nodes and weights for n=8 (standard weight
/// function w(x) = exp(-x)).
fn gauss_laguerre_nodes(n: usize) -> (Vec<f64>, Vec<f64>) {
    debug_assert!(n == 8, "Only n=8 supported");
    // Abramowitz & Stegun Table 25.9
    let nodes = vec![
        0.170279632305,
        0.903701776799,
        2.251086629866,
        4.266700170288,
        7.045905402393,
        10.758516010181,
        15.740678641928,
        22.863131736889,
    ];
    let weights = vec![
        3.69188589342e-01,
        4.18786780814e-01,
        1.75794986637e-01,
        3.33434922612e-02,
        2.79453623523e-03,
        9.07650877336e-05,
        8.48574671627e-07,
        1.04800117487e-09,
    ];
    (nodes, weights)
}

/// Weight correction for mapping a Gauss-Laguerre node (which integrates
/// x^0 · exp(-x)) to a Gamma(α) PDF evaluation.
///
/// Gamma(α) PDF ∝ x^{α-1} exp(-αx). Under the substitution t = αx the
/// Gauss-Laguerre integral becomes (1/α) ∫ (t/α)^{α-1} exp(-t) dt.
/// We return  (node/α)^{α-1} / Γ(α) · (1/α) · exp(node) · exp(-node/α · α)
/// but since the GL weights already include exp(-node), we simplify:
///
/// correction = (node)^{α-1} / (α^α · Γ(α)) · exp(node · (1 - 1))
///
/// Actually we directly compute the Gamma(shape, rate=shape) pdf at x = node/shape
/// divided by the GL weight function exp(-node):
///   pdf(x) = shape^shape / Γ(shape) · x^{shape-1} · exp(-shape·x)
///   correction = pdf(node/shape) · (1/shape) / exp(-node)
///            = shape^{shape-1} / Γ(shape) · (node/shape)^{shape-1} · exp(-node + node)
///   (the exp terms cancel)
///   = node^{shape-1} / (shape · Γ(shape))
///
/// We use ln Γ for numerical stability.
fn gamma_laguerre_weight(shape: f64, node: f64) -> f64 {
    if node < 1e-30 {
        return 0.0;
    }
    let log_w = (shape - 1.0) * node.ln() - shape.ln() - ln_gamma(shape);
    log_w.exp()
}

/// Stirling-series approximation of ln Γ(x) for x > 0.
/// Uses Lanczos approximation for small x and Stirling for large x.
fn ln_gamma(x: f64) -> f64 {
    // Lanczos approximation (g = 7, n = 9)
    if x < 0.5 {
        // Reflection formula: Γ(x) = π / (sin(πx) · Γ(1-x))
        let reflected = ln_gamma(1.0 - x);
        (PI / (PI * x).sin()).ln() - reflected
    } else {
        let coefficients: [f64; 9] = [
            0.99999999999980993,
            676.5203681218851,
            -1259.1392167224028,
            771.32342877765313,
            -176.61502916214059,
            12.507343278686905,
            -0.13857109526572012,
            9.9843695780195716e-6,
            1.5056327351493116e-7,
        ];
        let g = 7.0_f64;
        let z = x - 1.0;
        let mut ag = coefficients[0];
        for i in 1..9 {
            ag += coefficients[i] / (z + i as f64);
        }
        let t = z + g + 0.5;
        0.5 * (2.0 * PI).ln() + (z + 0.5) * t.ln() - t + ag.ln()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- Construction & basic accessors ------------------------------------

    #[test]
    fn test_new_defaults() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        assert_eq!(ch.distance, 1000.0);
        assert_eq!(ch.wavelength, 1550e-9);
        assert_eq!(ch.aperture, 0.10);
        assert!(approx_eq(ch.beam_divergence, 1e-3, 1e-9));
        assert_eq!(ch.jitter_rad, 0.0);
    }

    #[test]
    fn test_builder_methods() {
        let ch = FsoChannel::new(500.0, 850e-9, 0.05)
            .with_beam_divergence(2e-3)
            .with_jitter(0.5e-3);
        assert!(approx_eq(ch.beam_divergence, 2e-3, 1e-12));
        assert!(approx_eq(ch.jitter_rad, 0.5e-3, 1e-12));
    }

    // -- Wavenumber --------------------------------------------------------

    #[test]
    fn test_wavenumber() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let k = ch.wavenumber();
        let expected = 2.0 * PI / 1550e-9;
        assert!(approx_eq(k, expected, 1.0)); // ~4.05e6
    }

    // -- Rytov variance & turbulence regime --------------------------------

    #[test]
    fn test_rytov_weak() {
        // 500 m, very weak turbulence
        let ch = FsoChannel::new(500.0, 1550e-9, 0.10);
        let sr2 = ch.rytov_variance(1e-15);
        assert!(sr2 < 0.3, "Expected weak turbulence, got sigma_R2 = {sr2}");
        assert_eq!(ch.turbulence_regime(1e-15), TurbulenceRegime::Weak);
    }

    #[test]
    fn test_rytov_strong() {
        // 5 km, strong turbulence
        let ch = FsoChannel::new(5000.0, 1550e-9, 0.10);
        let sr2 = ch.rytov_variance(1e-12);
        assert!(sr2 >= 5.0, "Expected strong turbulence, got sigma_R2 = {sr2}");
        assert_eq!(ch.turbulence_regime(1e-12), TurbulenceRegime::Strong);
    }

    #[test]
    fn test_rytov_moderate() {
        // Find Cn2 that gives moderate regime for 1 km link
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        // Try Cn2 = 5e-14
        let sr2 = ch.rytov_variance(5e-14);
        // If this isn't moderate, adjust -- but it should be in range.
        // sigma_R2 is proportional to Cn2, so we can scale.
        let cn2_target = 0.5 / sr2 * 5e-14; // target sigma_R2 = 0.5
        let sr2_check = ch.rytov_variance(cn2_target);
        assert!(
            sr2_check >= 0.3 && sr2_check < 5.0,
            "Expected moderate, got {sr2_check}"
        );
        assert_eq!(ch.turbulence_regime(cn2_target), TurbulenceRegime::Moderate);
    }

    // -- Fried parameter ---------------------------------------------------

    #[test]
    fn test_fried_parameter_reasonable() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let r0 = ch.fried_parameter(1e-14);
        // Fried parameter should be positive, typically cm-scale
        assert!(r0 > 0.001 && r0 < 10.0, "r0 = {r0} m seems unreasonable");
    }

    #[test]
    fn test_fried_parameter_decreases_with_cn2() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let r0_weak = ch.fried_parameter(1e-16);
        let r0_strong = ch.fried_parameter(1e-13);
        assert!(
            r0_weak > r0_strong,
            "Fried parameter should decrease with stronger turbulence"
        );
    }

    // -- Aperture averaging ------------------------------------------------

    #[test]
    fn test_aperture_averaging_bounds() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let af = ch.aperture_averaging_factor();
        assert!(af > 0.0 && af <= 1.0, "Aperture averaging = {af}");
    }

    // -- Log-normal scintillation ------------------------------------------

    #[test]
    fn test_log_normal_scintillation_positive() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let si = ch.log_normal_scintillation_index(1e-14);
        assert!(si > 0.0, "Scintillation index should be positive, got {si}");
    }

    // -- Gamma-gamma parameters --------------------------------------------

    #[test]
    fn test_gamma_gamma_params_positive() {
        let ch = FsoChannel::new(2000.0, 1550e-9, 0.10);
        let (alpha, beta) = ch.gamma_gamma_params(1e-13);
        assert!(alpha > 0.0, "alpha = {alpha}");
        assert!(beta > 0.0, "beta = {beta}");
    }

    #[test]
    fn test_gamma_gamma_weak_large_params() {
        // For very weak turbulence, alpha and beta should be large
        let ch = FsoChannel::new(500.0, 1550e-9, 0.10);
        let (alpha, beta) = ch.gamma_gamma_params(1e-16);
        assert!(
            alpha > 10.0 && beta > 10.0,
            "Weak turbulence should give large alpha,beta: alpha={alpha}, beta={beta}"
        );
    }

    // -- Atmospheric attenuation -------------------------------------------

    #[test]
    fn test_atmospheric_attenuation_negative() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let att = ch.atmospheric_attenuation_db(10.0);
        assert!(att < 0.0, "Attenuation should be negative (loss), got {att}");
    }

    #[test]
    fn test_attenuation_increases_with_distance() {
        let ch1 = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let ch2 = FsoChannel::new(5000.0, 1550e-9, 0.10);
        let att1 = ch1.atmospheric_attenuation_db(10.0);
        let att2 = ch2.atmospheric_attenuation_db(10.0);
        assert!(
            att2 < att1,
            "Longer link should have more loss: {att2} vs {att1}"
        );
    }

    // -- Geometric loss ----------------------------------------------------

    #[test]
    fn test_geometric_loss_negative() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let loss = ch.geometric_loss_db();
        assert!(loss < 0.0, "Geometric loss should be negative, got {loss}");
    }

    // -- Pointing-error loss -----------------------------------------------

    #[test]
    fn test_pointing_loss_zero_jitter() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        assert_eq!(ch.pointing_error_loss_db(), 0.0);
    }

    #[test]
    fn test_pointing_loss_with_jitter() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10).with_jitter(0.5e-3);
        let loss = ch.pointing_error_loss_db();
        assert!(
            loss < 0.0,
            "Pointing loss with jitter should be negative, got {loss}"
        );
    }

    // -- Link budget -------------------------------------------------------

    #[test]
    fn test_link_budget_reasonable() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let p_rx = ch.link_budget_dbm(10.0, 0.0, 0.0, 10.0, 1e-14);
        // Should be significantly less than transmitted power
        assert!(
            p_rx < 10.0,
            "Received power should be less than TX, got {p_rx}"
        );
    }

    // -- BER estimation (AWGN) ---------------------------------------------

    #[test]
    fn test_ber_ook_awgn_high_snr() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let ber = ch.ber_awgn(100.0, FsoModulation::Ook);
        assert!(ber < 1e-6, "High-SNR OOK BER should be very low, got {ber}");
    }

    #[test]
    fn test_ber_ppm_awgn_better_than_ook() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let ber_ook = ch.ber_awgn(10.0, FsoModulation::Ook);
        let ber_ppm = ch.ber_awgn(10.0, FsoModulation::Ppm(4));
        // PPM generally achieves lower BER at same SNR due to coding gain
        // (this holds for reasonably high SNR)
        assert!(
            ber_ppm < ber_ook,
            "4-PPM should outperform OOK: PPM={ber_ppm}, OOK={ber_ook}"
        );
    }

    // -- BER under log-normal turbulence -----------------------------------

    #[test]
    fn test_ber_log_normal_worse_than_awgn() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let snr = 20.0;
        let ber_awgn = ch.ber_awgn(snr, FsoModulation::Ook);
        let ber_ln = ch.ber_log_normal(snr, 1e-14, FsoModulation::Ook);
        assert!(
            ber_ln >= ber_awgn * 0.99, // allow small numerical tolerance
            "Turbulence BER should be >= AWGN BER: LN={ber_ln}, AWGN={ber_awgn}"
        );
    }

    // -- Outage probability ------------------------------------------------

    #[test]
    fn test_outage_zero_turbulence() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let p_out = ch.outage_probability_log_normal(10.0, 5.0, 0.0);
        assert!(
            approx_eq(p_out, 0.0, 1e-10),
            "No turbulence, SNR above threshold -> outage ~ 0, got {p_out}"
        );
    }

    #[test]
    fn test_outage_increases_with_threshold() {
        let ch = FsoChannel::new(1000.0, 1550e-9, 0.10);
        let cn2 = 1e-14;
        let p1 = ch.outage_probability_log_normal(10.0, 5.0, cn2);
        let p2 = ch.outage_probability_log_normal(10.0, 9.0, cn2);
        assert!(
            p2 >= p1,
            "Higher threshold should give higher outage: {p2} vs {p1}"
        );
    }

    // -- Complex IQ fading -------------------------------------------------

    #[test]
    fn test_apply_log_normal_fading_unit_mean() {
        // Over many realisations the mean amplitude should be ~1
        let mut sum_amp = 0.0;
        let n = 10_000;
        let sigma_ln2 = 0.2;
        for i in 0..n {
            let u1 = (i as f64 + 0.5) / n as f64; // quasi-uniform in (0,1)
            let u2 = ((i as f64 * 0.618033988) % 1.0).max(1e-10);
            let (re, im) =
                FsoChannel::apply_log_normal_fading((1.0, 0.0), sigma_ln2, u1, u2);
            sum_amp += (re * re + im * im).sqrt();
        }
        let mean_amp = sum_amp / n as f64;
        // Should be close to 1.0 (unit-mean fading)
        assert!(
            approx_eq(mean_amp, 1.0, 0.15),
            "Mean amplitude should be ~1, got {mean_amp}"
        );
    }

    // -- erfc helper -------------------------------------------------------

    #[test]
    fn test_erfc_known_values() {
        assert!(approx_eq(erfc(0.0), 1.0, 1e-7));
        assert!(approx_eq(erfc(1.0), 0.1572992, 1e-5));
        assert!(approx_eq(erfc(-1.0), 1.8427008, 1e-5));
        assert!(erfc(5.0) < 1e-10);
    }

    // -- ln_gamma helper ---------------------------------------------------

    #[test]
    fn test_ln_gamma_factorial() {
        // Gamma(5) = 4! = 24, so ln(24) ~ 3.178
        let val = ln_gamma(5.0);
        assert!(approx_eq(val, 24.0_f64.ln(), 1e-6), "ln Gamma(5) = {val}");
        // Gamma(1) = 1, so ln(1) = 0
        assert!(approx_eq(ln_gamma(1.0), 0.0, 1e-6));
    }

    // -- Kim exponent ------------------------------------------------------

    #[test]
    fn test_kim_exponent_ranges() {
        assert!(approx_eq(kim_exponent(100.0, 1550.0), 1.6, 1e-10));
        assert!(approx_eq(kim_exponent(10.0, 1550.0), 1.3, 1e-10));
        let q = kim_exponent(3.0, 1550.0);
        assert!(q > 0.0 && q < 1.5, "Kim q for V=3 km: {q}");
        assert!(approx_eq(kim_exponent(0.3, 1550.0), 0.0, 1e-10));
    }
}
