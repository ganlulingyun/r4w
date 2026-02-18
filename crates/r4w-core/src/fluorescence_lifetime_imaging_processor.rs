// trace:FR-FLIM | ai:claude
//! # Fluorescence Lifetime Imaging Processor (FLIM)
//!
//! Implements FLIM data analysis for measuring fluorescence decay lifetimes
//! in microscopy, including time-correlated single photon counting (TCSPC)
//! histogram fitting, phasor analysis, and FRET efficiency calculation.
//!
//! ## Physics Background
//!
//! - **Fluorescence decay**: I(t) = A exp(-t/τ) after delta excitation
//! - **TCSPC**: histogram of photon arrival times → decay curve
//! - **Phasor**: Fourier transform at laser rep rate → (g,s) coordinates
//! - **FRET efficiency**: E = 1 - τ_DA/τ_D = R₀⁶/(R₀⁶ + r⁶)
//! - **Anisotropy**: rotational diffusion → depolarization

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// 1. DecayHistogram – TCSPC fluorescence decay histogram
// ---------------------------------------------------------------------------

/// TCSPC fluorescence decay histogram.
#[derive(Debug, Clone)]
pub struct DecayHistogram {
    /// Time bin centres in nanoseconds.
    pub time_bins_ns: Vec<f64>,
    /// Photon counts per bin.
    pub photon_counts: Vec<u64>,
}

impl DecayHistogram {
    /// Create a new histogram from time bins and photon counts.
    ///
    /// # Panics
    /// Panics if the two vectors have different lengths or are empty.
    pub fn new(time_bins_ns: Vec<f64>, photon_counts: Vec<u64>) -> Self {
        assert!(
            !time_bins_ns.is_empty(),
            "time_bins_ns must not be empty"
        );
        assert_eq!(
            time_bins_ns.len(),
            photon_counts.len(),
            "time_bins_ns and photon_counts must have the same length"
        );
        Self {
            time_bins_ns,
            photon_counts,
        }
    }

    /// Total number of detected photons.
    pub fn total_photons(&self) -> u64 {
        self.photon_counts.iter().sum()
    }

    /// Index of the channel with the most photon counts.
    pub fn peak_channel(&self) -> usize {
        self.photon_counts
            .iter()
            .enumerate()
            .max_by_key(|(_, &c)| c)
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Normalise counts to peak value (0..1).
    pub fn normalize(&self) -> Vec<f64> {
        let peak = self.photon_counts[self.peak_channel()] as f64;
        if peak == 0.0 {
            return vec![0.0; self.photon_counts.len()];
        }
        self.photon_counts.iter().map(|&c| c as f64 / peak).collect()
    }

    /// Estimate background level from the tail of the decay.
    ///
    /// `tail_fraction` in (0,1] specifies the fraction of the histogram
    /// (from the end) used for averaging.
    pub fn background_level(&self, tail_fraction: f64) -> f64 {
        let n = self.photon_counts.len();
        let tail_len = ((n as f64) * tail_fraction).ceil().max(1.0) as usize;
        let start = n.saturating_sub(tail_len);
        let sum: u64 = self.photon_counts[start..].iter().sum();
        sum as f64 / (n - start) as f64
    }

    /// Time resolution (bin width) in nanoseconds.
    pub fn time_resolution_ns(&self) -> f64 {
        if self.time_bins_ns.len() < 2 {
            return 0.0;
        }
        self.time_bins_ns[1] - self.time_bins_ns[0]
    }
}

// ---------------------------------------------------------------------------
// 2. MonoExponentialFit – single exponential I(t) = A exp(-t/τ) + B
// ---------------------------------------------------------------------------

/// Result of a single-exponential decay fit.
#[derive(Debug, Clone, Copy)]
pub struct DecayFitResult {
    /// Pre-exponential amplitude A.
    pub amplitude: f64,
    /// Fluorescence lifetime τ (ns).
    pub lifetime_ns: f64,
    /// Background offset B.
    pub background: f64,
    /// Chi-squared statistic.
    pub chi_squared: f64,
}

/// Single-exponential decay fitter.
pub struct MonoExponentialFit;

impl MonoExponentialFit {
    /// Fit I(t) = A·exp(-t/τ) + B to the histogram using weighted linear
    /// regression on ln(I - B), with B estimated from the tail.
    pub fn fit(histogram: &DecayHistogram) -> DecayFitResult {
        let bg = histogram.background_level(0.1);
        let peak = histogram.peak_channel();

        // Collect bins from peak onwards where count > bg+1
        let mut t_vals: Vec<f64> = Vec::new();
        let mut ln_vals: Vec<f64> = Vec::new();
        let mut weights: Vec<f64> = Vec::new();

        for i in peak..histogram.photon_counts.len() {
            let c = histogram.photon_counts[i] as f64;
            let corrected = c - bg;
            if corrected > 1.0 {
                t_vals.push(histogram.time_bins_ns[i]);
                ln_vals.push(corrected.ln());
                // Poisson weighting: w = count
                weights.push(c.max(1.0));
            }
        }

        if t_vals.len() < 2 {
            return DecayFitResult {
                amplitude: 0.0,
                lifetime_ns: 0.0,
                background: bg,
                chi_squared: f64::INFINITY,
            };
        }

        // Weighted linear regression:  ln(I-B) = ln(A) + (-1/τ)·t
        let (mut sw, mut st, mut sy, mut stt, mut sty) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for i in 0..t_vals.len() {
            let w = weights[i];
            sw += w;
            st += w * t_vals[i];
            sy += w * ln_vals[i];
            stt += w * t_vals[i] * t_vals[i];
            sty += w * t_vals[i] * ln_vals[i];
        }

        let det = sw * stt - st * st;
        if det.abs() < 1e-30 {
            return DecayFitResult {
                amplitude: 0.0,
                lifetime_ns: 0.0,
                background: bg,
                chi_squared: f64::INFINITY,
            };
        }

        let ln_a = (stt * sy - st * sty) / det;
        let slope = (sw * sty - st * sy) / det; // = -1/τ

        let amplitude = ln_a.exp();
        let lifetime_ns = if slope < -1e-30 { -1.0 / slope } else { f64::INFINITY };

        // Compute chi-squared
        let chi_sq = Self::compute_chi_squared(histogram, amplitude, lifetime_ns, bg, peak);

        DecayFitResult {
            amplitude,
            lifetime_ns,
            background: bg,
            chi_squared: chi_sq,
        }
    }

    fn compute_chi_squared(
        histogram: &DecayHistogram,
        amplitude: f64,
        tau: f64,
        bg: f64,
        start: usize,
    ) -> f64 {
        let mut chi_sq = 0.0;
        let mut count = 0;
        for i in start..histogram.photon_counts.len() {
            let observed = histogram.photon_counts[i] as f64;
            if observed < 1.0 {
                continue;
            }
            let t = histogram.time_bins_ns[i];
            let expected = amplitude * (-t / tau).exp() + bg;
            let diff = observed - expected;
            chi_sq += diff * diff / observed.max(1.0);
            count += 1;
        }
        if count > 2 {
            chi_sq / (count as f64 - 2.0)
        } else {
            chi_sq
        }
    }

    /// Reduced chi-squared for a given fit result and histogram.
    pub fn chi_squared_reduced(fit: &DecayFitResult, histogram: &DecayHistogram) -> f64 {
        let peak = histogram.peak_channel();
        Self::compute_chi_squared(
            histogram,
            fit.amplitude,
            fit.lifetime_ns,
            fit.background,
            peak,
        )
    }
}

// ---------------------------------------------------------------------------
// 3. BiExponentialFit – two-component decay
// ---------------------------------------------------------------------------

/// Result of a bi-exponential decay fit.
#[derive(Debug, Clone, Copy)]
pub struct BiExpFitResult {
    pub a1: f64,
    pub tau1_ns: f64,
    pub a2: f64,
    pub tau2_ns: f64,
    pub background: f64,
    pub chi_squared: f64,
}

/// Two-component exponential decay fitter using iterative linear
/// decomposition.
pub struct BiExponentialFit;

impl BiExponentialFit {
    /// Fit I(t) = A₁exp(-t/τ₁) + A₂exp(-t/τ₂) + B.
    ///
    /// Uses alternating linearised least squares seeded with initial guesses.
    pub fn fit(
        histogram: &DecayHistogram,
        tau1_guess: f64,
        tau2_guess: f64,
    ) -> BiExpFitResult {
        let bg = histogram.background_level(0.1);
        let peak = histogram.peak_channel();

        // Collect usable bins
        let mut t_vals: Vec<f64> = Vec::new();
        let mut y_vals: Vec<f64> = Vec::new();
        for i in peak..histogram.photon_counts.len() {
            let c = histogram.photon_counts[i] as f64 - bg;
            if c > 0.5 {
                t_vals.push(histogram.time_bins_ns[i]);
                y_vals.push(c);
            }
        }

        if t_vals.len() < 4 {
            return BiExpFitResult {
                a1: 0.0,
                tau1_ns: tau1_guess,
                a2: 0.0,
                tau2_ns: tau2_guess,
                background: bg,
                chi_squared: f64::INFINITY,
            };
        }

        let mut tau1 = tau1_guess;
        let mut tau2 = tau2_guess;
        let mut a1 = 0.5;
        let mut a2 = 0.5;

        // Iterative refinement (simple Gauss-Seidel style)
        for _iter in 0..50 {
            // Fix τ values, solve for amplitudes via linear least squares
            // Model: y_i = a1 * exp(-t_i/τ1) + a2 * exp(-t_i/τ2)
            // This is linear in (a1, a2) for fixed τ values.
            let (mut e11, mut e12, mut e22, mut ey1, mut ey2) =
                (0.0, 0.0, 0.0, 0.0, 0.0);
            for i in 0..t_vals.len() {
                let t = t_vals[i];
                let f1 = (-t / tau1).exp();
                let f2 = (-t / tau2).exp();
                let y = y_vals[i];
                e11 += f1 * f1;
                e12 += f1 * f2;
                e22 += f2 * f2;
                ey1 += f1 * y;
                ey2 += f2 * y;
            }

            let det = e11 * e22 - e12 * e12;
            if det.abs() < 1e-30 {
                break;
            }
            a1 = (e22 * ey1 - e12 * ey2) / det;
            a2 = (e11 * ey2 - e12 * ey1) / det;

            // Clamp amplitudes to positive
            if a1 < 0.0 {
                a1 = 0.0;
            }
            if a2 < 0.0 {
                a2 = 0.0;
            }

            // Fix amplitudes, refine τ values via gradient step
            // ∂SSR/∂τ1 = Σ 2(y-model) * a1 * (-t/τ1²) * exp(-t/τ1)
            let (mut grad1, mut grad2) = (0.0, 0.0);
            let mut ssr = 0.0;
            for i in 0..t_vals.len() {
                let t = t_vals[i];
                let f1 = (-t / tau1).exp();
                let f2 = (-t / tau2).exp();
                let residual = y_vals[i] - a1 * f1 - a2 * f2;
                ssr += residual * residual;
                grad1 += residual * a1 * (t / (tau1 * tau1)) * f1;
                grad2 += residual * a2 * (t / (tau2 * tau2)) * f2;
            }

            // Adaptive step size
            let scale = 0.01 * tau1.min(tau2);
            let step1 = (grad1 * scale).clamp(-tau1 * 0.1, tau1 * 0.1);
            let step2 = (grad2 * scale).clamp(-tau2 * 0.1, tau2 * 0.1);

            tau1 = (tau1 + step1).max(0.01);
            tau2 = (tau2 + step2).max(0.01);

            if ssr < 1e-20 {
                break;
            }
        }

        // Ensure τ1 < τ2 for consistency
        if tau1 > tau2 {
            std::mem::swap(&mut tau1, &mut tau2);
            std::mem::swap(&mut a1, &mut a2);
        }

        // Compute chi-squared
        let chi_sq = Self::compute_chi_squared(histogram, a1, tau1, a2, tau2, bg, peak);

        BiExpFitResult {
            a1,
            tau1_ns: tau1,
            a2,
            tau2_ns: tau2,
            background: bg,
            chi_squared: chi_sq,
        }
    }

    fn compute_chi_squared(
        histogram: &DecayHistogram,
        a1: f64,
        tau1: f64,
        a2: f64,
        tau2: f64,
        bg: f64,
        start: usize,
    ) -> f64 {
        let mut chi_sq = 0.0;
        let mut count = 0;
        for i in start..histogram.photon_counts.len() {
            let observed = histogram.photon_counts[i] as f64;
            if observed < 1.0 {
                continue;
            }
            let t = histogram.time_bins_ns[i];
            let expected = a1 * (-t / tau1).exp() + a2 * (-t / tau2).exp() + bg;
            let diff = observed - expected;
            chi_sq += diff * diff / observed.max(1.0);
            count += 1;
        }
        if count > 4 {
            chi_sq / (count as f64 - 4.0)
        } else {
            chi_sq
        }
    }

    /// Fractional intensities: f₁ = A₁τ₁/(A₁τ₁+A₂τ₂), f₂ = 1 - f₁.
    pub fn fractional_intensities(result: &BiExpFitResult) -> (f64, f64) {
        let denom = result.a1 * result.tau1_ns + result.a2 * result.tau2_ns;
        if denom.abs() < 1e-30 {
            return (0.5, 0.5);
        }
        let f1 = result.a1 * result.tau1_ns / denom;
        (f1, 1.0 - f1)
    }

    /// Amplitude-weighted lifetime: τ_amp = (A₁τ₁+A₂τ₂)/(A₁+A₂).
    pub fn amplitude_weighted_lifetime(result: &BiExpFitResult) -> f64 {
        let denom = result.a1 + result.a2;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (result.a1 * result.tau1_ns + result.a2 * result.tau2_ns) / denom
    }

    /// Intensity-weighted lifetime: τ_int = (A₁τ₁²+A₂τ₂²)/(A₁τ₁+A₂τ₂).
    pub fn intensity_weighted_lifetime(result: &BiExpFitResult) -> f64 {
        let denom = result.a1 * result.tau1_ns + result.a2 * result.tau2_ns;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (result.a1 * result.tau1_ns * result.tau1_ns
            + result.a2 * result.tau2_ns * result.tau2_ns)
            / denom
    }
}

// ---------------------------------------------------------------------------
// 4. PhasorAnalysis – frequency-domain phasor approach
// ---------------------------------------------------------------------------

/// Phasor coordinates (g, s) in the Fourier domain.
#[derive(Debug, Clone, Copy)]
pub struct Phasor {
    /// Real part (g-coordinate).
    pub g: f64,
    /// Imaginary part (s-coordinate).
    pub s: f64,
}

/// Phasor (frequency-domain) FLIM analysis.
pub struct PhasorAnalysis;

impl PhasorAnalysis {
    /// Compute the phasor from a decay histogram at the given harmonic.
    ///
    /// The angular frequency ω = 2π·harmonic / T_period, where T_period
    /// spans the full time range of the histogram.
    pub fn compute_phasor(histogram: &DecayHistogram, harmonic: usize) -> Phasor {
        let n = histogram.photon_counts.len();
        if n == 0 {
            return Phasor { g: 0.0, s: 0.0 };
        }

        let t_range = histogram.time_bins_ns[n - 1] - histogram.time_bins_ns[0]
            + histogram.time_resolution_ns();
        let omega = 2.0 * PI * (harmonic as f64) / t_range;

        let mut cos_sum = 0.0;
        let mut sin_sum = 0.0;
        let mut total = 0.0;

        for i in 0..n {
            let c = histogram.photon_counts[i] as f64;
            let t = histogram.time_bins_ns[i];
            cos_sum += c * (omega * t).cos();
            sin_sum += c * (omega * t).sin();
            total += c;
        }

        if total < 1.0 {
            return Phasor { g: 0.0, s: 0.0 };
        }

        Phasor {
            g: cos_sum / total,
            s: sin_sum / total,
        }
    }

    /// Estimate the lifetime from phasor coordinates.
    ///
    /// For a single exponential: τ = s/(ω·g).
    pub fn lifetime_from_phasor(phasor: &Phasor, omega: f64) -> f64 {
        if phasor.g.abs() < 1e-30 || omega.abs() < 1e-30 {
            return 0.0;
        }
        phasor.s / (omega * phasor.g)
    }

    /// Check whether the phasor lies on the universal semicircle,
    /// indicating a single-exponential decay.
    ///
    /// The semicircle has centre (0.5, 0) and radius 0.5: (g-0.5)²+s² = 0.25.
    pub fn is_single_exponential(phasor: &Phasor) -> bool {
        let dist = Self::universal_circle_distance(phasor);
        dist < 0.05 // tolerance
    }

    /// Distance from the universal semicircle (g-0.5)²+s² = 0.25.
    pub fn universal_circle_distance(phasor: &Phasor) -> f64 {
        let r = ((phasor.g - 0.5).powi(2) + phasor.s.powi(2)).sqrt();
        (r - 0.5).abs()
    }

    /// For a two-component mixture, compute fractional intensities given
    /// known lifetimes τ₁ and τ₂ and the measured phasor.
    ///
    /// The phasor of a mixture lies on the line connecting the two
    /// pure-component phasors on the semicircle.
    pub fn two_component_fractions(
        phasor: &Phasor,
        tau1: f64,
        tau2: f64,
        omega: f64,
    ) -> (f64, f64) {
        // Phasor of pure component i: g_i = 1/(1+ω²τ²), s_i = ωτ/(1+ω²τ²)
        let denom1 = 1.0 + (omega * tau1).powi(2);
        let g1 = 1.0 / denom1;
        let s1 = omega * tau1 / denom1;

        let denom2 = 1.0 + (omega * tau2).powi(2);
        let g2 = 1.0 / denom2;
        let s2 = omega * tau2 / denom2;

        // phasor = f1·P1 + f2·P2,  f1+f2=1
        // Solve using g-coordinate: f1 = (g - g2)/(g1 - g2)
        let dg = g1 - g2;
        if dg.abs() < 1e-30 {
            return (0.5, 0.5);
        }
        let f1 = (phasor.g - g2) / dg;
        let f1_clamped = f1.clamp(0.0, 1.0);

        // Cross-check with s coordinate
        let ds = s1 - s2;
        let f1_s = if ds.abs() > 1e-30 {
            ((phasor.s - s2) / ds).clamp(0.0, 1.0)
        } else {
            f1_clamped
        };

        // Average both estimates
        let f1_avg = 0.5 * (f1_clamped + f1_s);
        (f1_avg, 1.0 - f1_avg)
    }
}

// ---------------------------------------------------------------------------
// 5. FretCalculator – Förster Resonance Energy Transfer
// ---------------------------------------------------------------------------

/// FRET efficiency and distance calculations.
pub struct FretCalculator;

impl FretCalculator {
    /// FRET efficiency from donor lifetime quenching: E = 1 - τ_DA/τ_D.
    pub fn efficiency_from_lifetimes(tau_da: f64, tau_d: f64) -> f64 {
        if tau_d <= 0.0 {
            return 0.0;
        }
        (1.0 - tau_da / tau_d).clamp(0.0, 1.0)
    }

    /// Donor–acceptor distance from FRET efficiency: r = R₀(1/E - 1)^(1/6).
    pub fn distance_from_efficiency(e: f64, r0_nm: f64) -> f64 {
        if e <= 0.0 || e >= 1.0 {
            return if e <= 0.0 { f64::INFINITY } else { 0.0 };
        }
        r0_nm * (1.0 / e - 1.0).powf(1.0 / 6.0)
    }

    /// Förster radius R₀ (nm) from spectral overlap, orientation factor,
    /// refractive index, and donor quantum yield.
    ///
    /// R₀⁶ = 8.79 × 10⁻²⁵ · κ² · Q_D · J(λ) / n⁴  (in cm⁶)
    /// Then R₀ in nm.
    pub fn forster_radius(
        j_overlap: f64,  // cm³ M⁻¹ (overlap integral)
        kappa_sq: f64,    // orientation factor (2/3 for random)
        n_refr: f64,      // refractive index
        qd: f64,          // donor quantum yield
    ) -> f64 {
        let r0_6_cm6 = 8.79e-25 * kappa_sq * qd * j_overlap / n_refr.powi(4);
        // Convert cm to nm: 1 cm = 1e7 nm, so r₀^6 in nm^6 = r₀^6 cm^6 * (1e7)^6
        let r0_6_nm6 = r0_6_cm6 * 1e42;
        r0_6_nm6.powf(1.0 / 6.0)
    }

    /// FRET efficiency as a function of distance: E = 1/(1+(r/R₀)⁶).
    pub fn efficiency_vs_distance(r0: f64, distances: &[f64]) -> Vec<f64> {
        distances
            .iter()
            .map(|&r| {
                if r0 <= 0.0 {
                    return 0.0;
                }
                1.0 / (1.0 + (r / r0).powi(6))
            })
            .collect()
    }

    /// Common Förster radii (nm) for well-known FRET pairs.
    pub fn r0_cfp_yfp() -> f64 {
        4.9
    }

    /// R₀ for GFP–mCherry pair.
    pub fn r0_gfp_mcherry() -> f64 {
        5.1
    }
}

// ---------------------------------------------------------------------------
// 6. InstrumentResponseFunction – IRF handling
// ---------------------------------------------------------------------------

/// Instrument Response Function for TCSPC.
#[derive(Debug, Clone)]
pub struct InstrumentResponseFunction {
    /// Time bin centres (ns).
    pub time_bins: Vec<f64>,
    /// Normalised IRF values.
    pub response: Vec<f64>,
}

impl InstrumentResponseFunction {
    /// Create a new IRF.
    pub fn new(time_bins: Vec<f64>, response: Vec<f64>) -> Self {
        assert_eq!(time_bins.len(), response.len());
        // Normalise to unit area
        let sum: f64 = response.iter().sum();
        let response = if sum > 0.0 {
            response.iter().map(|&v| v / sum).collect()
        } else {
            response
        };
        Self { time_bins, response }
    }

    /// Full-width at half-maximum of the IRF (ns).
    pub fn fwhm(&self) -> f64 {
        let peak = self
            .response
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let half = peak / 2.0;

        let mut first = None;
        let mut last = None;
        for (i, &v) in self.response.iter().enumerate() {
            if v >= half {
                if first.is_none() {
                    first = Some(i);
                }
                last = Some(i);
            }
        }

        match (first, last) {
            (Some(f), Some(l)) if l > f => self.time_bins[l] - self.time_bins[f],
            _ => 0.0,
        }
    }

    /// Convolve the IRF with a decay model (same time grid).
    pub fn convolve_with_decay(
        irf: &InstrumentResponseFunction,
        decay: &[f64],
    ) -> Vec<f64> {
        let n = irf.response.len().min(decay.len());
        let mut result = vec![0.0; n];
        for i in 0..n {
            let mut sum = 0.0;
            for j in 0..=i {
                sum += irf.response[j] * decay[i - j];
            }
            result[i] = sum;
        }
        result
    }

    /// Iterative deconvolution (Richardson-Lucy style).
    pub fn deconvolve_iterative(
        histogram: &DecayHistogram,
        irf: &InstrumentResponseFunction,
        num_iter: usize,
    ) -> Vec<f64> {
        let n = histogram.photon_counts.len().min(irf.response.len());
        let observed: Vec<f64> = histogram.photon_counts[..n]
            .iter()
            .map(|&c| c as f64)
            .collect();

        // Initial estimate: uniform
        let total: f64 = observed.iter().sum();
        let mut estimate = vec![total / n as f64; n];

        for _iter in 0..num_iter {
            // Forward: convolve estimate with IRF
            let predicted = Self::convolve_with_decay(irf, &estimate);

            // Compute ratio observed / predicted
            let mut ratio = vec![1.0; n];
            for i in 0..n {
                if predicted[i] > 1e-10 {
                    ratio[i] = observed[i] / predicted[i];
                }
            }

            // Backward: correlate ratio with IRF (transpose convolution)
            let mut correction = vec![0.0; n];
            for i in 0..n {
                let mut sum = 0.0;
                for j in 0..n {
                    if j <= i {
                        // Transpose: irf[i-j] when valid
                    }
                    if i + j < n {
                        // Use the IRF in reverse
                    }
                    // Standard RL: correction[i] = Σ_j irf[j] * ratio[i+j]
                    let idx = i + j;
                    if idx < n {
                        sum += irf.response[j] * ratio[idx];
                    } else {
                        break;
                    }
                }
                correction[i] = sum;
            }

            // Update estimate
            for i in 0..n {
                estimate[i] *= correction[i];
                if estimate[i] < 0.0 {
                    estimate[i] = 0.0;
                }
            }
        }

        estimate
    }

    /// Generate a Gaussian IRF centred at the midpoint of time_bins.
    pub fn generate_gaussian_irf(fwhm_ns: f64, time_bins: &[f64]) -> InstrumentResponseFunction {
        let sigma = fwhm_ns / (2.0 * (2.0_f64 * 2.0_f64.ln()).sqrt());
        // sigma = FWHM / (2·sqrt(2·ln2))
        let n = time_bins.len();
        if n == 0 {
            return InstrumentResponseFunction {
                time_bins: vec![],
                response: vec![],
            };
        }
        let centre = (time_bins[0] + time_bins[n - 1]) / 2.0;
        let response: Vec<f64> = time_bins
            .iter()
            .map(|&t| {
                let x = (t - centre) / sigma;
                (-0.5 * x * x).exp()
            })
            .collect();
        InstrumentResponseFunction::new(time_bins.to_vec(), response)
    }
}

// ---------------------------------------------------------------------------
// 7. LifetimeImage – pixel-by-pixel lifetime map
// ---------------------------------------------------------------------------

/// Fit method selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FitMethod {
    MonoExponential,
    RapidLifetimeDetermination,
}

/// Pixel-by-pixel lifetime and chi-squared maps.
#[derive(Debug, Clone)]
pub struct LifetimeMap {
    pub lifetimes: Vec<Vec<f64>>,
    pub chi_squared: Vec<Vec<f64>>,
}

/// FLIM image processor.
pub struct LifetimeImage {
    width: usize,
    height: usize,
    pixels: Vec<Vec<Option<DecayHistogram>>>,
}

impl LifetimeImage {
    /// Create an empty lifetime image.
    pub fn new(width: usize, height: usize) -> Self {
        let row = vec![None; width];
        let pixels = vec![row; height];
        Self {
            width,
            height,
            pixels,
        }
    }

    /// Set the decay histogram for a specific pixel.
    pub fn set_pixel_histogram(&mut self, x: usize, y: usize, histogram: DecayHistogram) {
        if x < self.width && y < self.height {
            self.pixels[y][x] = Some(histogram);
        }
    }

    /// Fit all pixels and return lifetime + chi-squared maps.
    pub fn fit_all_pixels(&self, method: FitMethod) -> LifetimeMap {
        let mut lifetimes = vec![vec![0.0; self.width]; self.height];
        let mut chi_squared = vec![vec![0.0; self.width]; self.height];

        for y in 0..self.height {
            for x in 0..self.width {
                if let Some(ref hist) = self.pixels[y][x] {
                    match method {
                        FitMethod::MonoExponential => {
                            let fit = MonoExponentialFit::fit(hist);
                            lifetimes[y][x] = fit.lifetime_ns;
                            chi_squared[y][x] = fit.chi_squared;
                        }
                        FitMethod::RapidLifetimeDetermination => {
                            let n = hist.time_bins_ns.len();
                            if n >= 4 {
                                let mid = hist.time_bins_ns[n / 2];
                                let t0 = hist.time_bins_ns[0];
                                let t_end = hist.time_bins_ns[n - 1];
                                let tau = TimeGating::rapid_lifetime_determination(
                                    hist,
                                    (t0, mid),
                                    (mid, t_end),
                                );
                                lifetimes[y][x] = tau;
                                chi_squared[y][x] = 0.0; // RLD has no chi-sq
                            }
                        }
                    }
                }
            }
        }

        LifetimeMap {
            lifetimes,
            chi_squared,
        }
    }

    /// Total photon counts per pixel (intensity image).
    pub fn intensity_image(&self) -> Vec<Vec<f64>> {
        let mut img = vec![vec![0.0; self.width]; self.height];
        for y in 0..self.height {
            for x in 0..self.width {
                if let Some(ref hist) = self.pixels[y][x] {
                    img[y][x] = hist.total_photons() as f64;
                }
            }
        }
        img
    }

    /// Histogram of lifetime values across the image.
    pub fn lifetime_histogram(map: &LifetimeMap, num_bins: usize) -> Vec<(f64, usize)> {
        let all_values: Vec<f64> = map
            .lifetimes
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&v| v > 0.0 && v.is_finite())
            .cloned()
            .collect();

        if all_values.is_empty() || num_bins == 0 {
            return vec![];
        }

        let min_val = all_values
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max_val = all_values
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        if (max_val - min_val).abs() < 1e-15 {
            return vec![(min_val, all_values.len())];
        }

        let bin_width = (max_val - min_val) / num_bins as f64;
        let mut bins = vec![0usize; num_bins];

        for &v in &all_values {
            let idx = ((v - min_val) / bin_width).floor() as usize;
            let idx = idx.min(num_bins - 1);
            bins[idx] += 1;
        }

        bins.iter()
            .enumerate()
            .map(|(i, &count)| (min_val + (i as f64 + 0.5) * bin_width, count))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// 8. TimeGating – time-gated intensity analysis
// ---------------------------------------------------------------------------

/// Time-gated analysis methods.
pub struct TimeGating;

impl TimeGating {
    /// Integrated intensity within a time gate [t_start, t_end].
    pub fn gate_intensity(histogram: &DecayHistogram, t_start: f64, t_end: f64) -> f64 {
        let mut sum = 0.0;
        for (i, &t) in histogram.time_bins_ns.iter().enumerate() {
            if t >= t_start && t <= t_end {
                sum += histogram.photon_counts[i] as f64;
            }
        }
        sum
    }

    /// Rapid Lifetime Determination (RLD) from two time gates.
    ///
    /// τ = ΔT / ln(D₁/D₂) where D₁ and D₂ are integrated intensities
    /// over consecutive gates of width ΔT.
    pub fn rapid_lifetime_determination(
        histogram: &DecayHistogram,
        gate1: (f64, f64),
        gate2: (f64, f64),
    ) -> f64 {
        let d1 = Self::gate_intensity(histogram, gate1.0, gate1.1);
        let d2 = Self::gate_intensity(histogram, gate2.0, gate2.1);

        if d1 <= 0.0 || d2 <= 0.0 || d1 <= d2 {
            return 0.0;
        }

        let delta_t = gate2.0 - gate1.0;
        if delta_t <= 0.0 {
            return 0.0;
        }

        delta_t / (d1 / d2).ln()
    }

    /// Compute intensity ratios for multiple gates.
    pub fn multi_gate_ratio(
        histogram: &DecayHistogram,
        gates: &[(f64, f64)],
    ) -> Vec<f64> {
        let intensities: Vec<f64> = gates
            .iter()
            .map(|&(s, e)| Self::gate_intensity(histogram, s, e))
            .collect();

        if intensities.is_empty() {
            return vec![];
        }

        let first = intensities[0];
        if first < 1e-10 {
            return vec![0.0; intensities.len()];
        }

        intensities.iter().map(|&v| v / first).collect()
    }
}

// ---------------------------------------------------------------------------
// 9. DecaySimulator – generate synthetic FLIM data
// ---------------------------------------------------------------------------

/// Synthetic FLIM data generator.
pub struct DecaySimulator;

impl DecaySimulator {
    /// Simple seeded pseudo-random number generator (LCG).
    fn lcg_next(state: &mut u64) -> f64 {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to [0, 1)
        (*state >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Simulate a single-exponential decay histogram.
    pub fn simulate_decay(
        tau_ns: f64,
        num_photons: u64,
        time_range_ns: f64,
        num_bins: usize,
    ) -> DecayHistogram {
        let bin_width = time_range_ns / num_bins as f64;
        let time_bins: Vec<f64> = (0..num_bins)
            .map(|i| (i as f64 + 0.5) * bin_width)
            .collect();
        let mut counts = vec![0u64; num_bins];

        let mut rng_state: u64 = 42;

        for _ in 0..num_photons {
            // Inverse CDF sampling: t = -τ·ln(U)
            let u = Self::lcg_next(&mut rng_state);
            let t = -tau_ns * (1.0 - u).ln(); // avoid ln(0)
            let bin = (t / bin_width).floor() as usize;
            if bin < num_bins {
                counts[bin] += 1;
            }
        }

        DecayHistogram::new(time_bins, counts)
    }

    /// Simulate a bi-exponential decay histogram.
    pub fn simulate_biexp(
        tau1: f64,
        tau2: f64,
        fraction1: f64,
        photons: u64,
        range: f64,
        bins: usize,
    ) -> DecayHistogram {
        let bin_width = range / bins as f64;
        let time_bins: Vec<f64> = (0..bins)
            .map(|i| (i as f64 + 0.5) * bin_width)
            .collect();
        let mut counts = vec![0u64; bins];

        let mut rng_state: u64 = 123;

        for _ in 0..photons {
            let selector = Self::lcg_next(&mut rng_state);
            let tau = if selector < fraction1 { tau1 } else { tau2 };
            let u = Self::lcg_next(&mut rng_state);
            let t = -tau * (1.0 - u).ln();
            let bin = (t / bin_width).floor() as usize;
            if bin < bins {
                counts[bin] += 1;
            }
        }

        DecayHistogram::new(time_bins, counts)
    }

    /// Simulate a decay convolved with an IRF, then sample photons.
    pub fn simulate_with_irf(
        decay: &[f64],
        irf: &InstrumentResponseFunction,
        photons: u64,
    ) -> DecayHistogram {
        let convolved = InstrumentResponseFunction::convolve_with_decay(irf, decay);
        let n = convolved.len();
        if n == 0 || irf.time_bins.is_empty() {
            return DecayHistogram::new(vec![0.0], vec![0]);
        }

        // Build CDF
        let total: f64 = convolved.iter().sum();
        if total < 1e-30 {
            return DecayHistogram::new(irf.time_bins.clone(), vec![0; n]);
        }

        let mut cdf = vec![0.0; n];
        cdf[0] = convolved[0] / total;
        for i in 1..n {
            cdf[i] = cdf[i - 1] + convolved[i] / total;
        }

        let mut counts = vec![0u64; n];
        let mut rng_state: u64 = 999;

        for _ in 0..photons {
            let u = Self::lcg_next(&mut rng_state);
            // Binary search in CDF
            let bin = match cdf.binary_search_by(|v| v.partial_cmp(&u).unwrap()) {
                Ok(i) => i,
                Err(i) => i.min(n - 1),
            };
            counts[bin] += 1;
        }

        DecayHistogram::new(irf.time_bins.clone(), counts)
    }

    /// Add a uniform background to an existing histogram.
    pub fn add_background(histogram: &mut DecayHistogram, bg_per_bin: f64) {
        let mut rng_state: u64 = 7777;
        for c in histogram.photon_counts.iter_mut() {
            // Poisson-like: round bg_per_bin + small random variation
            let noise = (Self::lcg_next(&mut rng_state) - 0.5) * 2.0 * bg_per_bin.sqrt();
            let added = (bg_per_bin + noise).max(0.0).round() as u64;
            *c += added;
        }
    }
}

// ---------------------------------------------------------------------------
// 10. AnisotropyDecay – fluorescence anisotropy
// ---------------------------------------------------------------------------

/// Fluorescence anisotropy calculations.
pub struct AnisotropyDecay;

impl AnisotropyDecay {
    /// Steady-state anisotropy: r = (I_par - G·I_perp)/(I_par + 2G·I_perp).
    pub fn anisotropy(i_parallel: f64, i_perpendicular: f64, g_factor: f64) -> f64 {
        let num = i_parallel - g_factor * i_perpendicular;
        let den = i_parallel + 2.0 * g_factor * i_perpendicular;
        if den.abs() < 1e-30 {
            return 0.0;
        }
        num / den
    }

    /// Estimate the rotational correlation time θ from time-resolved
    /// anisotropy: r(t) = (r₀ - r∞)exp(-t/θ) + r∞.
    ///
    /// Uses log-linear regression on r(t) - r∞.
    pub fn rotational_correlation_time(
        r_0: f64,
        r_inf: f64,
        decay_times: &[f64],
        anisotropy_values: &[f64],
    ) -> f64 {
        if decay_times.len() < 2 || decay_times.len() != anisotropy_values.len() {
            return 0.0;
        }

        let mut t_vals: Vec<f64> = Vec::new();
        let mut ln_vals: Vec<f64> = Vec::new();

        for i in 0..decay_times.len() {
            let r_corrected = anisotropy_values[i] - r_inf;
            if r_corrected > 1e-10 {
                t_vals.push(decay_times[i]);
                ln_vals.push(r_corrected.ln());
            }
        }

        if t_vals.len() < 2 {
            return 0.0;
        }

        // Linear regression on ln(r - r∞) vs t: slope = -1/θ
        let n = t_vals.len() as f64;
        let sum_t: f64 = t_vals.iter().sum();
        let sum_ln: f64 = ln_vals.iter().sum();
        let sum_tt: f64 = t_vals.iter().map(|t| t * t).sum();
        let sum_t_ln: f64 = t_vals
            .iter()
            .zip(ln_vals.iter())
            .map(|(t, l)| t * l)
            .sum();

        let det = n * sum_tt - sum_t * sum_t;
        if det.abs() < 1e-30 {
            return 0.0;
        }

        let slope = (n * sum_t_ln - sum_t * sum_ln) / det;

        if slope < -1e-30 {
            -1.0 / slope
        } else {
            0.0
        }
    }

    /// Fundamental anisotropy from absorption-emission angle α:
    /// r₀ = 0.4 · (3cos²α - 1)/2 = 0.2(3cos²α - 1).
    pub fn fundamental_anisotropy(angle_deg: f64) -> f64 {
        let alpha = angle_deg * PI / 180.0;
        let cos_a = alpha.cos();
        0.2 * (3.0 * cos_a * cos_a - 1.0)
    }

    /// Perrin equation: r = r₀/(1 + τ/θ).
    pub fn perrin_equation(r_0: f64, tau: f64, theta: f64) -> f64 {
        if theta <= 0.0 {
            return 0.0;
        }
        r_0 / (1.0 + tau / theta)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: create a simple exponential decay histogram
    fn make_mono_histogram(tau: f64, num_bins: usize, time_range: f64) -> DecayHistogram {
        let bin_width = time_range / num_bins as f64;
        let time_bins: Vec<f64> = (0..num_bins)
            .map(|i| (i as f64 + 0.5) * bin_width)
            .collect();
        let counts: Vec<u64> = time_bins
            .iter()
            .map(|&t| (10000.0 * (-t / tau).exp()).round() as u64)
            .collect();
        DecayHistogram::new(time_bins, counts)
    }

    // -----------------------------------------------------------------------
    // DecayHistogram tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_histogram_total_photons() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        assert_eq!(h.total_photons(), 175);
    }

    #[test]
    fn test_histogram_peak_channel() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![10, 100, 50]);
        assert_eq!(h.peak_channel(), 1);
    }

    #[test]
    fn test_histogram_peak_channel_first() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![200, 100, 50]);
        assert_eq!(h.peak_channel(), 0);
    }

    #[test]
    fn test_histogram_normalize() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        let norm = h.normalize();
        assert!((norm[0] - 1.0).abs() < 1e-10);
        assert!((norm[1] - 0.5).abs() < 1e-10);
        assert!((norm[2] - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_histogram_normalize_zero() {
        let h = DecayHistogram::new(vec![0.5, 1.5], vec![0, 0]);
        let norm = h.normalize();
        assert_eq!(norm, vec![0.0, 0.0]);
    }

    #[test]
    fn test_histogram_background_level() {
        let h = DecayHistogram::new(
            vec![0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5],
            vec![1000, 500, 250, 125, 63, 31, 15, 8, 5, 3],
        );
        let bg = h.background_level(0.1); // last 1 bin
        assert!((bg - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_histogram_time_resolution() {
        let h = DecayHistogram::new(vec![0.1, 0.2, 0.3], vec![10, 20, 30]);
        assert!((h.time_resolution_ns() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_histogram_single_bin() {
        let h = DecayHistogram::new(vec![1.0], vec![42]);
        assert_eq!(h.total_photons(), 42);
        assert_eq!(h.peak_channel(), 0);
        assert_eq!(h.time_resolution_ns(), 0.0);
    }

    #[test]
    #[should_panic]
    fn test_histogram_empty_panics() {
        DecayHistogram::new(vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_histogram_mismatch_panics() {
        DecayHistogram::new(vec![1.0, 2.0], vec![10]);
    }

    // -----------------------------------------------------------------------
    // MonoExponentialFit tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mono_fit_ideal() {
        let h = make_mono_histogram(4.0, 100, 40.0);
        let fit = MonoExponentialFit::fit(&h);
        assert!(
            (fit.lifetime_ns - 4.0).abs() < 0.5,
            "Expected τ≈4.0, got {}",
            fit.lifetime_ns
        );
        assert!(fit.amplitude > 0.0);
    }

    #[test]
    fn test_mono_fit_short_lifetime() {
        let h = make_mono_histogram(1.0, 100, 10.0);
        let fit = MonoExponentialFit::fit(&h);
        assert!(
            (fit.lifetime_ns - 1.0).abs() < 0.3,
            "Expected τ≈1.0, got {}",
            fit.lifetime_ns
        );
    }

    #[test]
    fn test_mono_fit_long_lifetime() {
        let h = make_mono_histogram(10.0, 200, 50.0);
        let fit = MonoExponentialFit::fit(&h);
        assert!(
            (fit.lifetime_ns - 10.0).abs() < 1.0,
            "Expected τ≈10.0, got {}",
            fit.lifetime_ns
        );
    }

    #[test]
    fn test_mono_fit_chi_squared_reduced() {
        let h = make_mono_histogram(4.0, 100, 40.0);
        let fit = MonoExponentialFit::fit(&h);
        let chi_r = MonoExponentialFit::chi_squared_reduced(&fit, &h);
        // For perfect data, chi_r should be small
        assert!(chi_r < 50.0, "Reduced chi² should be reasonable, got {}", chi_r);
    }

    #[test]
    fn test_mono_fit_background() {
        let h = make_mono_histogram(4.0, 100, 40.0);
        let fit = MonoExponentialFit::fit(&h);
        assert!(fit.background >= 0.0);
    }

    // -----------------------------------------------------------------------
    // BiExponentialFit tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_biexp_fractional_intensities() {
        let result = BiExpFitResult {
            a1: 100.0,
            tau1_ns: 2.0,
            a2: 50.0,
            tau2_ns: 8.0,
            background: 0.0,
            chi_squared: 1.0,
        };
        let (f1, f2) = BiExponentialFit::fractional_intensities(&result);
        // f1 = 100*2/(100*2+50*8) = 200/600 = 1/3
        assert!((f1 - 1.0 / 3.0).abs() < 1e-10);
        assert!((f2 - 2.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_biexp_amplitude_weighted_lifetime() {
        let result = BiExpFitResult {
            a1: 100.0,
            tau1_ns: 2.0,
            a2: 100.0,
            tau2_ns: 8.0,
            background: 0.0,
            chi_squared: 1.0,
        };
        let tau_amp = BiExponentialFit::amplitude_weighted_lifetime(&result);
        // (100*2+100*8)/(100+100) = 1000/200 = 5.0
        assert!((tau_amp - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_biexp_intensity_weighted_lifetime() {
        let result = BiExpFitResult {
            a1: 100.0,
            tau1_ns: 2.0,
            a2: 100.0,
            tau2_ns: 8.0,
            background: 0.0,
            chi_squared: 1.0,
        };
        let tau_int = BiExponentialFit::intensity_weighted_lifetime(&result);
        // (100*4+100*64)/(100*2+100*8) = 6800/1000 = 6.8
        assert!((tau_int - 6.8).abs() < 1e-10);
    }

    #[test]
    fn test_biexp_fit_basic() {
        let h = DecaySimulator::simulate_biexp(2.0, 8.0, 0.5, 100000, 30.0, 150);
        let fit = BiExponentialFit::fit(&h, 2.5, 7.0);
        // Just check that it produces positive results
        assert!(fit.a1 >= 0.0);
        assert!(fit.a2 >= 0.0);
        assert!(fit.tau1_ns > 0.0);
        assert!(fit.tau2_ns > 0.0);
        assert!(fit.tau1_ns <= fit.tau2_ns); // ordered
    }

    #[test]
    fn test_biexp_fractional_intensities_sum_to_one() {
        let result = BiExpFitResult {
            a1: 75.0,
            tau1_ns: 3.0,
            a2: 25.0,
            tau2_ns: 12.0,
            background: 5.0,
            chi_squared: 1.5,
        };
        let (f1, f2) = BiExponentialFit::fractional_intensities(&result);
        assert!((f1 + f2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_biexp_amp_weighted_bounds() {
        let result = BiExpFitResult {
            a1: 50.0,
            tau1_ns: 1.0,
            a2: 50.0,
            tau2_ns: 10.0,
            background: 0.0,
            chi_squared: 1.0,
        };
        let tau_amp = BiExponentialFit::amplitude_weighted_lifetime(&result);
        assert!(tau_amp >= 1.0 && tau_amp <= 10.0);
    }

    #[test]
    fn test_biexp_int_weighted_bounds() {
        let result = BiExpFitResult {
            a1: 50.0,
            tau1_ns: 1.0,
            a2: 50.0,
            tau2_ns: 10.0,
            background: 0.0,
            chi_squared: 1.0,
        };
        let tau_int = BiExponentialFit::intensity_weighted_lifetime(&result);
        assert!(tau_int >= 1.0 && tau_int <= 10.0);
    }

    // -----------------------------------------------------------------------
    // PhasorAnalysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_phasor_single_exponential() {
        // For a perfect single-exponential, phasor should lie on the semicircle
        let h = make_mono_histogram(4.0, 256, 40.0);
        let p = PhasorAnalysis::compute_phasor(&h, 1);
        assert!(p.g > 0.0 && p.g < 1.0);
        assert!(p.s > 0.0);
        let dist = PhasorAnalysis::universal_circle_distance(&p);
        assert!(
            dist < 0.1,
            "Phasor should be near semicircle, distance={}",
            dist
        );
    }

    #[test]
    fn test_phasor_is_single_exponential() {
        let h = make_mono_histogram(4.0, 256, 40.0);
        let p = PhasorAnalysis::compute_phasor(&h, 1);
        assert!(PhasorAnalysis::is_single_exponential(&p));
    }

    #[test]
    fn test_phasor_lifetime_estimate() {
        let tau = 4.0;
        let h = make_mono_histogram(tau, 256, 40.0);
        let n = h.time_bins_ns.len();
        let t_range = h.time_bins_ns[n - 1] - h.time_bins_ns[0] + h.time_resolution_ns();
        let omega = 2.0 * PI / t_range;
        let p = PhasorAnalysis::compute_phasor(&h, 1);
        let tau_est = PhasorAnalysis::lifetime_from_phasor(&p, omega);
        assert!(
            (tau_est - tau).abs() < 1.0,
            "Expected τ≈{}, got {}",
            tau,
            tau_est
        );
    }

    #[test]
    fn test_phasor_two_component_fractions() {
        let tau1: f64 = 2.0;
        let tau2: f64 = 8.0;
        let omega: f64 = 0.5;

        // A pure component-1 phasor
        let denom1 = 1.0 + (omega * tau1).powi(2);
        let g1 = 1.0 / denom1;
        let s1 = omega * tau1 / denom1;
        let p = Phasor { g: g1, s: s1 };

        let (f1, f2) = PhasorAnalysis::two_component_fractions(&p, tau1, tau2, omega);
        assert!(
            (f1 - 1.0).abs() < 0.1,
            "Pure component 1 should give f1≈1, got {}",
            f1
        );
        assert!(f2 < 0.1);
    }

    #[test]
    fn test_phasor_universal_circle_distance_on_circle() {
        // A point on the semicircle: (g, s) = (0.5, 0.5) has radius 0.5 from (0.5, 0)
        let p = Phasor { g: 0.5, s: 0.5 };
        let dist = PhasorAnalysis::universal_circle_distance(&p);
        assert!(dist < 1e-10);
    }

    #[test]
    fn test_phasor_empty_histogram() {
        let h = DecayHistogram::new(vec![1.0], vec![0]);
        let p = PhasorAnalysis::compute_phasor(&h, 1);
        assert_eq!(p.g, 0.0);
        assert_eq!(p.s, 0.0);
    }

    // -----------------------------------------------------------------------
    // FretCalculator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fret_efficiency_basic() {
        let e = FretCalculator::efficiency_from_lifetimes(2.0, 4.0);
        assert!((e - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_fret_efficiency_zero() {
        let e = FretCalculator::efficiency_from_lifetimes(4.0, 4.0);
        assert!((e - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_fret_efficiency_high() {
        let e = FretCalculator::efficiency_from_lifetimes(0.5, 4.0);
        assert!((e - 0.875).abs() < 1e-10);
    }

    #[test]
    fn test_fret_distance() {
        let r0 = 5.0;
        let e = 0.5;
        let r = FretCalculator::distance_from_efficiency(e, r0);
        // E=0.5 → r = R₀
        assert!((r - r0).abs() < 1e-10);
    }

    #[test]
    fn test_fret_distance_high_efficiency() {
        let r0 = 5.0;
        let r = FretCalculator::distance_from_efficiency(0.99, r0);
        assert!(r < r0); // should be closer than R₀
    }

    #[test]
    fn test_fret_distance_low_efficiency() {
        let r0 = 5.0;
        let r = FretCalculator::distance_from_efficiency(0.01, r0);
        assert!(r > r0); // should be farther than R₀
    }

    #[test]
    fn test_fret_efficiency_vs_distance() {
        let r0 = 5.0;
        let distances = vec![0.0, 5.0, 10.0];
        let eff = FretCalculator::efficiency_vs_distance(r0, &distances);
        assert!((eff[0] - 1.0).abs() < 1e-10); // r=0 → E=1
        assert!((eff[1] - 0.5).abs() < 1e-10); // r=R₀ → E=0.5
        assert!(eff[2] < 0.1); // r=2R₀ → E small
    }

    #[test]
    fn test_fret_forster_radius() {
        // Known CFP-YFP: R₀ ≈ 4.9 nm
        // This is a sanity check with approximate values
        let r0 = FretCalculator::forster_radius(
            1.0e-13, // J(λ) in cm³/M
            2.0 / 3.0,
            1.33,
            0.4,
        );
        assert!(r0 > 0.0);
        assert!(r0.is_finite());
    }

    #[test]
    fn test_fret_r0_values() {
        assert!((FretCalculator::r0_cfp_yfp() - 4.9).abs() < 1e-10);
        assert!((FretCalculator::r0_gfp_mcherry() - 5.1).abs() < 1e-10);
    }

    #[test]
    fn test_fret_efficiency_clamped() {
        // τ_DA > τ_D should give 0 (clamped)
        let e = FretCalculator::efficiency_from_lifetimes(5.0, 4.0);
        assert_eq!(e, 0.0);
    }

    #[test]
    fn test_fret_distance_edge_cases() {
        assert!(FretCalculator::distance_from_efficiency(0.0, 5.0).is_infinite());
        assert!((FretCalculator::distance_from_efficiency(1.0, 5.0) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // InstrumentResponseFunction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_irf_fwhm() {
        let irf = InstrumentResponseFunction::generate_gaussian_irf(
            0.5,
            &(0..100).map(|i| i as f64 * 0.1).collect::<Vec<_>>(),
        );
        let fwhm = irf.fwhm();
        assert!(
            (fwhm - 0.5).abs() < 0.2,
            "Expected FWHM≈0.5, got {}",
            fwhm
        );
    }

    #[test]
    fn test_irf_normalised() {
        let irf = InstrumentResponseFunction::new(
            vec![0.0, 1.0, 2.0],
            vec![1.0, 2.0, 1.0],
        );
        let sum: f64 = irf.response.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10, "IRF should be normalised, sum={}", sum);
    }

    #[test]
    fn test_irf_convolve_delta() {
        // Convolving with a delta function should reproduce the IRF
        let irf = InstrumentResponseFunction::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 1.0, 0.0, 0.0, 0.0],
        );
        let decay = vec![1.0, 0.0, 0.0, 0.0, 0.0];
        let result = InstrumentResponseFunction::convolve_with_decay(&irf, &decay);
        assert_eq!(result.len(), 5);
        // Result should be proportional to IRF
        assert!(result[0] > 0.0 || result[1] > 0.0);
    }

    #[test]
    fn test_irf_convolve_length() {
        let irf = InstrumentResponseFunction::new(
            vec![0.0, 1.0, 2.0],
            vec![0.5, 1.0, 0.5],
        );
        let decay = vec![1.0, 0.5, 0.25];
        let result = InstrumentResponseFunction::convolve_with_decay(&irf, &decay);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_gaussian_irf_generation() {
        let bins: Vec<f64> = (0..200).map(|i| i as f64 * 0.05).collect();
        let irf = InstrumentResponseFunction::generate_gaussian_irf(1.0, &bins);
        assert_eq!(irf.response.len(), 200);
        let sum: f64 = irf.response.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_deconvolve_iterative_preserves_shape() {
        let tau = 4.0;
        let bins: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let irf = InstrumentResponseFunction::generate_gaussian_irf(0.3, &bins);
        let decay: Vec<f64> = bins.iter().map(|&t| 1000.0 * (-t / tau).exp()).collect();
        let convolved = InstrumentResponseFunction::convolve_with_decay(&irf, &decay);

        let counts: Vec<u64> = convolved.iter().map(|&v| v.round().max(0.0) as u64).collect();
        let h = DecayHistogram::new(bins.clone(), counts);

        let deconv = InstrumentResponseFunction::deconvolve_iterative(&h, &irf, 10);
        assert_eq!(deconv.len(), 100);
        // Deconvolved should have a peak
        let peak = deconv.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(peak > 0.0);
    }

    // -----------------------------------------------------------------------
    // LifetimeImage tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lifetime_image_creation() {
        let img = LifetimeImage::new(4, 3);
        assert_eq!(img.width, 4);
        assert_eq!(img.height, 3);
    }

    #[test]
    fn test_lifetime_image_set_pixel() {
        let mut img = LifetimeImage::new(2, 2);
        let h = make_mono_histogram(4.0, 50, 20.0);
        img.set_pixel_histogram(0, 0, h);
        let intensity = img.intensity_image();
        assert!(intensity[0][0] > 0.0);
        assert_eq!(intensity[0][1], 0.0);
    }

    #[test]
    fn test_lifetime_image_fit_mono() {
        let mut img = LifetimeImage::new(2, 1);
        let h1 = make_mono_histogram(2.0, 50, 20.0);
        let h2 = make_mono_histogram(6.0, 50, 20.0);
        img.set_pixel_histogram(0, 0, h1);
        img.set_pixel_histogram(1, 0, h2);

        let map = img.fit_all_pixels(FitMethod::MonoExponential);
        assert!(map.lifetimes[0][0] > 0.0);
        assert!(map.lifetimes[0][1] > 0.0);
        // Pixel 1 should have longer lifetime than pixel 0
        assert!(map.lifetimes[0][1] > map.lifetimes[0][0]);
    }

    #[test]
    fn test_lifetime_image_intensity() {
        let mut img = LifetimeImage::new(2, 2);
        img.set_pixel_histogram(
            0,
            0,
            DecayHistogram::new(vec![0.5, 1.5], vec![100, 50]),
        );
        let intensity = img.intensity_image();
        assert!((intensity[0][0] - 150.0).abs() < 1e-10);
    }

    #[test]
    fn test_lifetime_histogram() {
        let map = LifetimeMap {
            lifetimes: vec![vec![1.0, 2.0, 3.0, 4.0, 5.0]],
            chi_squared: vec![vec![0.0; 5]],
        };
        let hist = LifetimeImage::lifetime_histogram(&map, 5);
        assert_eq!(hist.len(), 5);
        let total: usize = hist.iter().map(|(_, c)| c).sum();
        assert_eq!(total, 5);
    }

    #[test]
    fn test_lifetime_image_rld_method() {
        let mut img = LifetimeImage::new(1, 1);
        let h = make_mono_histogram(4.0, 100, 40.0);
        img.set_pixel_histogram(0, 0, h);
        let map = img.fit_all_pixels(FitMethod::RapidLifetimeDetermination);
        assert!(map.lifetimes[0][0] > 0.0);
    }

    // -----------------------------------------------------------------------
    // TimeGating tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gate_intensity_full() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        let total = TimeGating::gate_intensity(&h, 0.0, 3.0);
        assert!((total - 175.0).abs() < 1e-10);
    }

    #[test]
    fn test_gate_intensity_partial() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        let intensity = TimeGating::gate_intensity(&h, 1.0, 2.0);
        assert!((intensity - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_rapid_lifetime_determination() {
        let h = make_mono_histogram(4.0, 100, 40.0);
        let tau = TimeGating::rapid_lifetime_determination(
            &h,
            (0.0, 10.0),
            (10.0, 20.0),
        );
        assert!(
            tau > 0.0,
            "RLD should produce a positive lifetime, got {}",
            tau
        );
        assert!(
            (tau - 4.0).abs() < 2.0,
            "RLD estimate should be near 4.0 ns, got {}",
            tau
        );
    }

    #[test]
    fn test_multi_gate_ratio() {
        let h = make_mono_histogram(4.0, 100, 40.0);
        let gates = vec![(0.0, 5.0), (5.0, 10.0), (10.0, 15.0)];
        let ratios = TimeGating::multi_gate_ratio(&h, &gates);
        assert_eq!(ratios.len(), 3);
        assert!((ratios[0] - 1.0).abs() < 1e-10); // first gate ratio = 1.0
        assert!(ratios[1] < ratios[0]); // decaying
        assert!(ratios[2] < ratios[1]); // decaying
    }

    #[test]
    fn test_gate_intensity_empty_range() {
        let h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        let intensity = TimeGating::gate_intensity(&h, 5.0, 10.0);
        assert!((intensity - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // DecaySimulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_decay_total_photons() {
        let h = DecaySimulator::simulate_decay(4.0, 10000, 20.0, 100);
        // Not all photons fall within range, but most should
        assert!(h.total_photons() > 5000);
        assert!(h.total_photons() <= 10000);
    }

    #[test]
    fn test_simulate_decay_decreasing() {
        let h = DecaySimulator::simulate_decay(4.0, 100000, 20.0, 20);
        // First few bins should have more than last few bins
        let first_sum: u64 = h.photon_counts[..5].iter().sum();
        let last_sum: u64 = h.photon_counts[15..].iter().sum();
        assert!(first_sum > last_sum);
    }

    #[test]
    fn test_simulate_biexp() {
        let h = DecaySimulator::simulate_biexp(2.0, 8.0, 0.5, 50000, 30.0, 100);
        assert!(h.total_photons() > 0);
        assert_eq!(h.photon_counts.len(), 100);
    }

    #[test]
    fn test_simulate_with_irf() {
        let bins: Vec<f64> = (0..50).map(|i| i as f64 * 0.5).collect();
        let irf = InstrumentResponseFunction::generate_gaussian_irf(0.5, &bins);
        let decay: Vec<f64> = bins.iter().map(|&t| (-t / 4.0).exp()).collect();
        let h = DecaySimulator::simulate_with_irf(&decay, &irf, 10000);
        assert!(h.total_photons() > 0);
    }

    #[test]
    fn test_add_background() {
        let mut h = DecayHistogram::new(vec![0.5, 1.5, 2.5], vec![100, 50, 25]);
        let orig_total = h.total_photons();
        DecaySimulator::add_background(&mut h, 10.0);
        assert!(h.total_photons() > orig_total);
    }

    #[test]
    fn test_simulate_decay_bin_count() {
        let h = DecaySimulator::simulate_decay(3.0, 1000, 15.0, 50);
        assert_eq!(h.photon_counts.len(), 50);
        assert_eq!(h.time_bins_ns.len(), 50);
    }

    // -----------------------------------------------------------------------
    // AnisotropyDecay tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_anisotropy_basic() {
        // I_par = 3, I_perp = 1, G = 1
        let r = AnisotropyDecay::anisotropy(3.0, 1.0, 1.0);
        // r = (3-1)/(3+2) = 2/5 = 0.4
        assert!((r - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_anisotropy_isotropic() {
        // Equal intensities → r = 0
        let r = AnisotropyDecay::anisotropy(1.0, 1.0, 1.0);
        assert!((r - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_anisotropy_with_g_factor() {
        let r = AnisotropyDecay::anisotropy(3.0, 2.0, 0.5);
        // r = (3 - 0.5*2)/(3 + 2*0.5*2) = (3-1)/(3+2) = 2/5 = 0.4
        assert!((r - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_fundamental_anisotropy_parallel() {
        // α = 0° → cos²(0) = 1 → r₀ = 0.2(3-1) = 0.4
        let r0 = AnisotropyDecay::fundamental_anisotropy(0.0);
        assert!((r0 - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_fundamental_anisotropy_magic_angle() {
        // Magic angle: α = 54.7356° → r₀ = 0
        let r0 = AnisotropyDecay::fundamental_anisotropy(54.7356);
        assert!(r0.abs() < 0.01);
    }

    #[test]
    fn test_fundamental_anisotropy_perpendicular() {
        // α = 90° → cos²(90) = 0 → r₀ = 0.2(0-1) = -0.2
        let r0 = AnisotropyDecay::fundamental_anisotropy(90.0);
        assert!((r0 - (-0.2)).abs() < 1e-10);
    }

    #[test]
    fn test_perrin_equation() {
        // r = 0.4/(1+4/2) = 0.4/3
        let r = AnisotropyDecay::perrin_equation(0.4, 4.0, 2.0);
        assert!((r - 0.4 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_perrin_equation_fast_rotation() {
        // θ → ∞ means no depolarisation
        let r = AnisotropyDecay::perrin_equation(0.4, 4.0, 1e10);
        assert!((r - 0.4).abs() < 1e-4);
    }

    #[test]
    fn test_perrin_equation_slow_rotation() {
        // θ → 0 means complete depolarisation
        let r = AnisotropyDecay::perrin_equation(0.4, 4.0, 0.001);
        assert!(r < 0.001);
    }

    #[test]
    fn test_rotational_correlation_time() {
        // Synthetic: r(t) = 0.3·exp(-t/5.0) + 0.0
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 0.5).collect();
        let r_values: Vec<f64> = times.iter().map(|&t| 0.3 * (-t / 5.0).exp()).collect();
        let theta = AnisotropyDecay::rotational_correlation_time(0.3, 0.0, &times, &r_values);
        assert!(
            (theta - 5.0).abs() < 1.0,
            "Expected θ≈5.0, got {}",
            theta
        );
    }

    #[test]
    fn test_rotational_correlation_time_with_residual() {
        let times: Vec<f64> = (0..30).map(|i| i as f64 * 0.5).collect();
        let r_values: Vec<f64> = times
            .iter()
            .map(|&t| 0.3 * (-t / 3.0).exp() + 0.05)
            .collect();
        let theta =
            AnisotropyDecay::rotational_correlation_time(0.3, 0.05, &times, &r_values);
        assert!(
            (theta - 3.0).abs() < 1.0,
            "Expected θ≈3.0, got {}",
            theta
        );
    }

    // -----------------------------------------------------------------------
    // Integration / cross-component tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_then_fit() {
        let h = DecaySimulator::simulate_decay(3.5, 100000, 25.0, 128);
        let fit = MonoExponentialFit::fit(&h);
        assert!(
            (fit.lifetime_ns - 3.5).abs() < 1.0,
            "Fitted lifetime should be near 3.5, got {}",
            fit.lifetime_ns
        );
    }

    #[test]
    fn test_simulate_then_phasor() {
        let h = DecaySimulator::simulate_decay(5.0, 50000, 30.0, 128);
        let p = PhasorAnalysis::compute_phasor(&h, 1);
        assert!(PhasorAnalysis::is_single_exponential(&p) || PhasorAnalysis::universal_circle_distance(&p) < 0.15);
    }

    #[test]
    fn test_simulate_then_rld() {
        let h = DecaySimulator::simulate_decay(4.0, 100000, 25.0, 100);
        let tau = TimeGating::rapid_lifetime_determination(&h, (0.0, 8.0), (8.0, 16.0));
        assert!(tau > 0.0);
    }

    #[test]
    fn test_fret_roundtrip() {
        let r0: f64 = 5.0;
        let r: f64 = 6.0;
        let e = 1.0 / (1.0 + (r / r0).powi(6));
        let r_back = FretCalculator::distance_from_efficiency(e, r0);
        assert!(
            (r_back - r).abs() < 1e-8,
            "Roundtrip: expected r={}, got {}",
            r,
            r_back
        );
    }

    #[test]
    fn test_fret_efficiency_vs_distance_monotonic() {
        let r0 = 5.0;
        let distances: Vec<f64> = (0..20).map(|i| i as f64 * 0.5).collect();
        let eff = FretCalculator::efficiency_vs_distance(r0, &distances);
        for i in 1..eff.len() {
            assert!(
                eff[i] <= eff[i - 1] + 1e-10,
                "Efficiency should be monotonically decreasing"
            );
        }
    }

    #[test]
    fn test_lifetime_histogram_empty() {
        let map = LifetimeMap {
            lifetimes: vec![vec![0.0; 3]],
            chi_squared: vec![vec![0.0; 3]],
        };
        let hist = LifetimeImage::lifetime_histogram(&map, 10);
        assert!(hist.is_empty()); // All zeros filtered out
    }

    #[test]
    fn test_phasor_second_harmonic() {
        let h = make_mono_histogram(4.0, 256, 40.0);
        let p1 = PhasorAnalysis::compute_phasor(&h, 1);
        let p2 = PhasorAnalysis::compute_phasor(&h, 2);
        // Higher harmonics move closer to the origin
        let r1 = (p1.g * p1.g + p1.s * p1.s).sqrt();
        let r2 = (p2.g * p2.g + p2.s * p2.s).sqrt();
        assert!(r2 < r1, "Second harmonic phasor should be closer to origin");
    }
}
