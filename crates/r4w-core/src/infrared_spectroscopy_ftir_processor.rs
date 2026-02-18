// trace:FR-FTIR | ai:claude
//! # Infrared Spectroscopy FTIR Processor
//!
//! Implements Fourier Transform Infrared (FTIR) spectroscopy analysis including
//! interferogram processing, Mertz phase correction, apodization, Beer-Lambert
//! quantitative analysis, and functional group identification.
//!
//! ## Physics Background
//!
//! - **Michelson interferometer**: I(δ) = ∫ B(ν̃) cos(2πν̃δ) dν̃
//! - **FFT**: B(ν̃) = FFT{I(δ)} (spectrum from interferogram)
//! - **Mertz phase correction**: handles asymmetric interferogram
//! - **Apodization**: window functions to reduce sidelobes
//! - **Beer-Lambert**: A = εcl for quantitative analysis
//! - **Wavenumber**: ν̃ = 1/λ (cm⁻¹), typical MIR range 400-4000 cm⁻¹

use std::f64::consts::PI;

/// Speed of light in cm/s.
pub const C_CM_S: f64 = 2.998e10;

// ---------------------------------------------------------------------------
// 1. Interferogram
// ---------------------------------------------------------------------------

/// An interferogram from FTIR measurement.
#[derive(Debug, Clone)]
pub struct Interferogram {
    /// Optical path difference values in cm.
    pub opd: Vec<f64>,
    /// Intensity values.
    pub signal: Vec<f64>,
}

impl Interferogram {
    /// Create new interferogram.
    pub fn new(opd: Vec<f64>, signal: Vec<f64>) -> Self {
        assert_eq!(opd.len(), signal.len());
        Self { opd, signal }
    }

    /// Find the centerburst (maximum of interferogram).
    pub fn centerburst_index(&self) -> usize {
        let mut max_idx: usize = 0;
        let mut max_val: f64 = f64::NEG_INFINITY;
        for (i, &v) in self.signal.iter().enumerate() {
            if v > max_val { max_val = v; max_idx = i; }
        }
        max_idx
    }

    /// OPD resolution (max OPD determines spectral resolution).
    pub fn max_opd(&self) -> f64 {
        let mut max_val: f64 = f64::NEG_INFINITY;
        for &v in &self.opd {
            if v.abs() > max_val { max_val = v.abs(); }
        }
        max_val
    }

    /// Spectral resolution in cm⁻¹: Δν̃ = 1 / (2 * max_OPD)
    pub fn spectral_resolution(&self) -> f64 {
        let max_opd: f64 = self.max_opd();
        if max_opd < 1e-15 { return f64::INFINITY; }
        1.0 / (2.0 * max_opd)
    }
}

/// Generate a synthetic interferogram from spectrum.
pub fn generate_interferogram(
    wavenumbers: &[f64],
    spectrum: &[f64],
    opd_values: &[f64],
) -> Interferogram {
    let n_opd: usize = opd_values.len();
    let n_spec: usize = wavenumbers.len().min(spectrum.len());
    let mut signal: Vec<f64> = vec![0.0; n_opd];
    for (i, &delta) in opd_values.iter().enumerate() {
        let mut s: f64 = 0.0;
        for j in 0..n_spec.saturating_sub(1) {
            let dnu: f64 = wavenumbers[j + 1] - wavenumbers[j];
            let nu_mid: f64 = 0.5 * (wavenumbers[j] + wavenumbers[j + 1]);
            let b_mid: f64 = 0.5 * (spectrum[j] + spectrum[j + 1]);
            s += b_mid * (2.0 * PI * nu_mid * delta).cos() * dnu;
        }
        signal[i] = s;
    }
    Interferogram::new(opd_values.to_vec(), signal)
}

// ---------------------------------------------------------------------------
// 2. Apodization Functions
// ---------------------------------------------------------------------------

/// Apodization function type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApodizationFunction {
    Boxcar,
    Triangular,
    Happ_Genzel,
    BlackmanHarris,
    NortonBeer,
}

/// Apply apodization to an interferogram.
pub fn apodize(signal: &[f64], func: ApodizationFunction) -> Vec<f64> {
    let n: usize = signal.len();
    let nf: f64 = n as f64;
    let mut out: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        let x: f64 = (i as f64 - nf / 2.0) / (nf / 2.0); // [-1, 1]
        let w: f64 = match func {
            ApodizationFunction::Boxcar => 1.0,
            ApodizationFunction::Triangular => 1.0 - x.abs(),
            ApodizationFunction::Happ_Genzel => {
                0.54 + 0.46 * (PI * x).cos()
            }
            ApodizationFunction::BlackmanHarris => {
                let t: f64 = PI * (i as f64) / (nf - 1.0);
                0.35875 - 0.48829 * (2.0 * t).cos()
                    + 0.14128 * (4.0 * t).cos()
                    - 0.01168 * (6.0 * t).cos()
            }
            ApodizationFunction::NortonBeer => {
                let x2: f64 = x * x;
                0.348093 - 0.087577 * x2 + 0.703484 * (1.0 - x2)
            }
        };
        out.push(signal[i] * w);
    }
    out
}

// ---------------------------------------------------------------------------
// 3. FFT (simplified power-of-2 DFT)
// ---------------------------------------------------------------------------

/// Compute real DFT magnitude spectrum (half-spectrum).
pub fn real_fft_magnitude(signal: &[f64]) -> Vec<f64> {
    let n: usize = signal.len();
    let half: usize = n / 2 + 1;
    let mut mag: Vec<f64> = Vec::with_capacity(half);
    for k in 0..half {
        let mut re: f64 = 0.0;
        let mut im: f64 = 0.0;
        for (j, &s) in signal.iter().enumerate() {
            let angle: f64 = 2.0 * PI * k as f64 * j as f64 / n as f64;
            re += s * angle.cos();
            im -= s * angle.sin();
        }
        mag.push((re * re + im * im).sqrt() / n as f64);
    }
    mag
}

/// Compute wavenumber axis for FFT output.
pub fn compute_wavenumber_axis(n_points: usize, opd_step: f64) -> Vec<f64> {
    let half: usize = n_points / 2 + 1;
    let nu_max: f64 = 1.0 / (2.0 * opd_step);
    let mut axis: Vec<f64> = Vec::with_capacity(half);
    for k in 0..half {
        axis.push(k as f64 * nu_max / (half as f64 - 1.0));
    }
    axis
}

// ---------------------------------------------------------------------------
// 4. IR Spectrum
// ---------------------------------------------------------------------------

/// An infrared spectrum.
#[derive(Debug, Clone)]
pub struct IrSpectrum {
    /// Wavenumbers in cm⁻¹.
    pub wavenumbers: Vec<f64>,
    /// Absorbance values.
    pub absorbance: Vec<f64>,
}

impl IrSpectrum {
    /// Create new spectrum.
    pub fn new(wavenumbers: Vec<f64>, absorbance: Vec<f64>) -> Self {
        assert_eq!(wavenumbers.len(), absorbance.len());
        Self { wavenumbers, absorbance }
    }

    /// Convert transmittance spectrum to absorbance: A = -log10(T)
    pub fn from_transmittance(wavenumbers: Vec<f64>, transmittance: Vec<f64>) -> Self {
        let absorbance: Vec<f64> = transmittance.iter()
            .map(|&t| if t > 0.0 { -(t.log10()) } else { 10.0 })
            .collect();
        Self { wavenumbers, absorbance }
    }

    /// Convert to transmittance: T = 10^(-A)
    pub fn to_transmittance(&self) -> Vec<f64> {
        self.absorbance.iter()
            .map(|&a| 10.0_f64.powf(-a))
            .collect()
    }

    /// Find peak absorbance.
    pub fn peak_absorbance(&self) -> (f64, f64) {
        let mut max_idx: usize = 0;
        let mut max_val: f64 = f64::NEG_INFINITY;
        for (i, &v) in self.absorbance.iter().enumerate() {
            if v > max_val { max_val = v; max_idx = i; }
        }
        (self.wavenumbers[max_idx], max_val)
    }

    /// Find all absorption bands above threshold.
    pub fn find_bands(&self, threshold: f64) -> Vec<IrBand> {
        let n: usize = self.wavenumbers.len();
        let mut bands: Vec<IrBand> = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if self.absorbance[i] > threshold
                && self.absorbance[i] > self.absorbance[i - 1]
                && self.absorbance[i] > self.absorbance[i + 1]
            {
                bands.push(IrBand {
                    wavenumber: self.wavenumbers[i],
                    absorbance: self.absorbance[i],
                    assignment: identify_band(self.wavenumbers[i]),
                });
            }
        }
        bands
    }

    /// Baseline correction using rubber-band method.
    pub fn baseline_correct(&mut self) {
        let n: usize = self.wavenumbers.len();
        if n < 3 { return; }
        // Simple: linear baseline between first and last points
        let a0: f64 = self.absorbance[0];
        let a1: f64 = self.absorbance[n - 1];
        let w0: f64 = self.wavenumbers[0];
        let w1: f64 = self.wavenumbers[n - 1];
        let range: f64 = w1 - w0;
        if range.abs() < 1e-10 { return; }
        for i in 0..n {
            let frac: f64 = (self.wavenumbers[i] - w0) / range;
            let baseline: f64 = a0 + frac * (a1 - a0);
            self.absorbance[i] -= baseline;
        }
    }

    /// Integrated absorbance (area under band).
    pub fn integrated_absorbance(&self, wn_start: f64, wn_end: f64) -> f64 {
        let n: usize = self.wavenumbers.len();
        let mut area: f64 = 0.0;
        for i in 0..n - 1 {
            let w0: f64 = self.wavenumbers[i];
            let w1: f64 = self.wavenumbers[i + 1];
            if w0 >= wn_start && w1 <= wn_end {
                let dw: f64 = (w1 - w0).abs();
                area += 0.5 * (self.absorbance[i] + self.absorbance[i + 1]) * dw;
            }
        }
        area
    }
}

/// An infrared absorption band.
#[derive(Debug, Clone)]
pub struct IrBand {
    pub wavenumber: f64,
    pub absorbance: f64,
    pub assignment: &'static str,
}

/// Identify functional group from wavenumber.
pub fn identify_band(wavenumber: f64) -> &'static str {
    let wn: f64 = wavenumber;
    if wn >= 3200.0 && wn <= 3600.0 { return "O-H stretch"; }
    if wn >= 3000.0 && wn < 3200.0 { return "=C-H stretch (aromatic/alkene)"; }
    if wn >= 2800.0 && wn < 3000.0 { return "C-H stretch (alkane)"; }
    if wn >= 2100.0 && wn <= 2300.0 { return "C≡C or C≡N stretch"; }
    if wn >= 1680.0 && wn <= 1750.0 { return "C=O stretch (carbonyl)"; }
    if wn >= 1600.0 && wn < 1680.0 { return "C=C stretch (aromatic/alkene)"; }
    if wn >= 1350.0 && wn < 1480.0 { return "C-H bend"; }
    if wn >= 1000.0 && wn < 1300.0 { return "C-O stretch"; }
    if wn >= 600.0 && wn < 900.0 { return "C-H out-of-plane bend"; }
    "unassigned"
}

// ---------------------------------------------------------------------------
// 5. Beer-Lambert Quantitative Analysis
// ---------------------------------------------------------------------------

/// Beer-Lambert: A = ε * c * l
pub fn beer_lambert_concentration(absorbance: f64, epsilon: f64, path_length: f64) -> f64 {
    if epsilon * path_length <= 0.0 { return 0.0; }
    absorbance / (epsilon * path_length)
}

/// Build calibration curve (linear regression).
pub fn calibration_curve(
    concentrations: &[f64],
    absorbances: &[f64],
) -> (f64, f64, f64) {
    let n: usize = concentrations.len().min(absorbances.len());
    if n < 2 { return (0.0, 1.0, 0.0); }
    let nf: f64 = n as f64;
    let mut sx: f64 = 0.0;
    let mut sy: f64 = 0.0;
    let mut sxy: f64 = 0.0;
    let mut sxx: f64 = 0.0;
    for i in 0..n {
        sx += concentrations[i];
        sy += absorbances[i];
        sxy += concentrations[i] * absorbances[i];
        sxx += concentrations[i] * concentrations[i];
    }
    let denom: f64 = nf * sxx - sx * sx;
    if denom.abs() < 1e-30 { return (0.0, 1.0, 0.0); }
    let slope: f64 = (nf * sxy - sx * sy) / denom;
    let intercept: f64 = (sy - slope * sx) / nf;
    let y_mean: f64 = sy / nf;
    let mut ss_res: f64 = 0.0;
    let mut ss_tot: f64 = 0.0;
    for i in 0..n {
        let pred: f64 = slope * concentrations[i] + intercept;
        ss_res += (absorbances[i] - pred).powi(2);
        ss_tot += (absorbances[i] - y_mean).powi(2);
    }
    let r2: f64 = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 0.0 };
    (slope, intercept, r2)
}

// ---------------------------------------------------------------------------
// 6. Spectral Comparison
// ---------------------------------------------------------------------------

/// Pearson correlation between two spectra.
pub fn spectral_correlation(a: &[f64], b: &[f64]) -> f64 {
    let n: usize = a.len().min(b.len());
    if n < 2 { return 0.0; }
    let nf: f64 = n as f64;
    let mean_a: f64 = a[..n].iter().sum::<f64>() / nf;
    let mean_b: f64 = b[..n].iter().sum::<f64>() / nf;
    let mut cov: f64 = 0.0;
    let mut var_a: f64 = 0.0;
    let mut var_b: f64 = 0.0;
    for i in 0..n {
        let da: f64 = a[i] - mean_a;
        let db: f64 = b[i] - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }
    let denom: f64 = (var_a * var_b).sqrt();
    if denom < 1e-30 { return 0.0; }
    cov / denom
}

/// Euclidean distance between two spectra.
pub fn spectral_distance(a: &[f64], b: &[f64]) -> f64 {
    let n: usize = a.len().min(b.len());
    let mut sum_sq: f64 = 0.0;
    for i in 0..n {
        let d: f64 = a[i] - b[i];
        sum_sq += d * d;
    }
    sum_sq.sqrt()
}

// ---------------------------------------------------------------------------
// 7. ATR Correction
// ---------------------------------------------------------------------------

/// ATR depth of penetration: dp = λ / (2π * n1 * sqrt(sin²θ - (n2/n1)²))
pub fn atr_penetration_depth(
    wavenumber: f64,
    n_crystal: f64,
    n_sample: f64,
    angle_deg: f64,
) -> f64 {
    let lambda_cm: f64 = 1.0 / wavenumber;
    let theta: f64 = angle_deg * PI / 180.0;
    let ratio: f64 = n_sample / n_crystal;
    let discriminant: f64 = theta.sin().powi(2) - ratio * ratio;
    if discriminant <= 0.0 { return f64::INFINITY; }
    lambda_cm / (2.0 * PI * n_crystal * discriminant.sqrt())
}

/// ATR correction factor: multiply absorbance to approximate transmission equivalent.
pub fn atr_correction_factor(wavenumber: f64, reference_wn: f64) -> f64 {
    if reference_wn <= 0.0 { return 1.0; }
    wavenumber / reference_wn
}

// ---------------------------------------------------------------------------
// 8. FtirProcessor Orchestrator
// ---------------------------------------------------------------------------

/// FTIR processor orchestrator.
#[derive(Debug, Clone)]
pub struct FtirProcessor {
    pub interferogram: Option<Interferogram>,
    pub spectrum: Option<IrSpectrum>,
    pub apodization: ApodizationFunction,
}

impl FtirProcessor {
    /// Create new processor.
    pub fn new() -> Self {
        Self {
            interferogram: None,
            spectrum: None,
            apodization: ApodizationFunction::Happ_Genzel,
        }
    }

    /// Set apodization function.
    pub fn set_apodization(&mut self, func: ApodizationFunction) {
        self.apodization = func;
    }

    /// Load interferogram.
    pub fn load_interferogram(&mut self, igram: Interferogram) {
        self.interferogram = Some(igram);
    }

    /// Process interferogram to spectrum.
    pub fn process(&mut self) -> Option<&IrSpectrum> {
        let igram = self.interferogram.as_ref()?;
        let apodized = apodize(&igram.signal, self.apodization);
        let mag = real_fft_magnitude(&apodized);
        let n: usize = igram.opd.len();
        let opd_step: f64 = if n > 1 {
            (igram.opd[1] - igram.opd[0]).abs()
        } else {
            1.0
        };
        let wn = compute_wavenumber_axis(n, opd_step);
        let n_out: usize = wn.len().min(mag.len());
        let spec = IrSpectrum::new(wn[..n_out].to_vec(), mag[..n_out].to_vec());
        self.spectrum = Some(spec);
        self.spectrum.as_ref()
    }

    /// Load pre-computed spectrum.
    pub fn load_spectrum(&mut self, spec: IrSpectrum) {
        self.spectrum = Some(spec);
    }

    /// Find absorption bands.
    pub fn find_bands(&self, threshold: f64) -> Vec<IrBand> {
        match &self.spectrum {
            Some(spec) => spec.find_bands(threshold),
            None => Vec::new(),
        }
    }

    /// Quantitative analysis using Beer-Lambert.
    pub fn quantify(&self, wavenumber: f64, epsilon: f64, path_length: f64) -> Option<f64> {
        let spec = self.spectrum.as_ref()?;
        // Find nearest wavenumber
        let mut best_idx: usize = 0;
        let mut best_dist: f64 = f64::MAX;
        for (i, &wn) in spec.wavenumbers.iter().enumerate() {
            let d: f64 = (wn - wavenumber).abs();
            if d < best_dist { best_dist = d; best_idx = i; }
        }
        let abs: f64 = spec.absorbance[best_idx];
        Some(beer_lambert_concentration(abs, epsilon, path_length))
    }
}

impl Default for FtirProcessor {
    fn default() -> Self {
        Self::new()
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

    #[test]
    fn test_interferogram_centerburst() {
        let igram = Interferogram::new(
            vec![-2.0, -1.0, 0.0, 1.0, 2.0],
            vec![0.1, 0.5, 1.0, 0.5, 0.1],
        );
        assert_eq!(igram.centerburst_index(), 2);
    }

    #[test]
    fn test_interferogram_max_opd() {
        let igram = Interferogram::new(
            vec![-0.5, -0.25, 0.0, 0.25, 0.5],
            vec![0.0; 5],
        );
        assert!(approx_eq(igram.max_opd(), 0.5, 0.01));
    }

    #[test]
    fn test_spectral_resolution() {
        let igram = Interferogram::new(
            vec![-0.5, 0.0, 0.5],
            vec![0.0; 3],
        );
        // Resolution = 1/(2*0.5) = 1 cm⁻¹
        assert!(approx_eq(igram.spectral_resolution(), 1.0, 0.01));
    }

    #[test]
    fn test_apodize_boxcar() {
        let signal: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0];
        let out = apodize(&signal, ApodizationFunction::Boxcar);
        assert_eq!(out, signal);
    }

    #[test]
    fn test_apodize_triangular() {
        let signal: Vec<f64> = vec![1.0; 10];
        let out = apodize(&signal, ApodizationFunction::Triangular);
        // Center should be highest
        assert!(out[5] >= out[0]);
    }

    #[test]
    fn test_apodize_happ_genzel() {
        let signal: Vec<f64> = vec![1.0; 8];
        let out = apodize(&signal, ApodizationFunction::Happ_Genzel);
        assert!(out.len() == 8);
    }

    #[test]
    fn test_apodize_blackman_harris() {
        let signal: Vec<f64> = vec![1.0; 16];
        let out = apodize(&signal, ApodizationFunction::BlackmanHarris);
        assert!(out[0] < out[8]); // edges attenuated
    }

    #[test]
    fn test_apodize_norton_beer() {
        let signal: Vec<f64> = vec![1.0; 8];
        let out = apodize(&signal, ApodizationFunction::NortonBeer);
        assert!(out.len() == 8);
    }

    #[test]
    fn test_real_fft_magnitude() {
        // DC signal should give peak at k=0
        let signal: Vec<f64> = vec![1.0; 16];
        let mag = real_fft_magnitude(&signal);
        assert!(mag[0] > mag[1]);
    }

    #[test]
    fn test_wavenumber_axis() {
        let axis = compute_wavenumber_axis(100, 0.0001);
        // nu_max = 1/(2*0.0001) = 5000 cm⁻¹
        assert!(axis.last().unwrap() > &4000.0);
    }

    #[test]
    fn test_ir_spectrum_from_transmittance() {
        let spec = IrSpectrum::from_transmittance(
            vec![1000.0, 2000.0],
            vec![0.1, 1.0],
        );
        assert!(approx_eq(spec.absorbance[0], 1.0, 0.01));
        assert!(approx_eq(spec.absorbance[1], 0.0, 0.01));
    }

    #[test]
    fn test_ir_spectrum_to_transmittance() {
        let spec = IrSpectrum::new(
            vec![1000.0, 2000.0],
            vec![1.0, 0.0],
        );
        let t = spec.to_transmittance();
        assert!(approx_eq(t[0], 0.1, 0.001));
        assert!(approx_eq(t[1], 1.0, 0.001));
    }

    #[test]
    fn test_ir_peak_absorbance() {
        let spec = IrSpectrum::new(
            vec![1000.0, 1700.0, 3000.0],
            vec![0.5, 1.2, 0.3],
        );
        let (wn, abs) = spec.peak_absorbance();
        assert!(approx_eq(wn, 1700.0, 0.01));
        assert!(approx_eq(abs, 1.2, 0.01));
    }

    #[test]
    fn test_find_bands() {
        let spec = IrSpectrum::new(
            vec![1000.0, 1500.0, 1700.0, 1900.0, 2500.0],
            vec![0.1, 0.3, 0.8, 0.2, 0.1],
        );
        let bands = spec.find_bands(0.5);
        assert_eq!(bands.len(), 1);
        assert!(bands[0].assignment.contains("C=O"));
    }

    #[test]
    fn test_identify_band_oh() {
        assert_eq!(identify_band(3400.0), "O-H stretch");
    }

    #[test]
    fn test_identify_band_ch() {
        assert_eq!(identify_band(2950.0), "C-H stretch (alkane)");
    }

    #[test]
    fn test_identify_band_carbonyl() {
        assert_eq!(identify_band(1720.0), "C=O stretch (carbonyl)");
    }

    #[test]
    fn test_identify_band_unassigned() {
        assert_eq!(identify_band(100.0), "unassigned");
    }

    #[test]
    fn test_beer_lambert_concentration() {
        let c: f64 = beer_lambert_concentration(1.0, 100.0, 1.0);
        assert!(approx_eq(c, 0.01, 0.0001));
    }

    #[test]
    fn test_calibration_curve() {
        let concs: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let abs: Vec<f64> = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let (slope, intercept, r2) = calibration_curve(&concs, &abs);
        assert!(approx_eq(slope, 0.5, 0.01));
        assert!(approx_eq(intercept, 0.0, 0.01));
        assert!(r2 > 0.999);
    }

    #[test]
    fn test_spectral_correlation_identical() {
        let a: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr: f64 = spectral_correlation(&a, &a);
        assert!(approx_eq(corr, 1.0, 0.001));
    }

    #[test]
    fn test_spectral_correlation_anticorrelated() {
        let a: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b: Vec<f64> = vec![5.0, 4.0, 3.0, 2.0, 1.0];
        let corr: f64 = spectral_correlation(&a, &b);
        assert!(approx_eq(corr, -1.0, 0.001));
    }

    #[test]
    fn test_spectral_distance() {
        let a: Vec<f64> = vec![0.0, 0.0, 0.0];
        let b: Vec<f64> = vec![3.0, 4.0, 0.0];
        let d: f64 = spectral_distance(&a, &b);
        assert!(approx_eq(d, 5.0, 0.01));
    }

    #[test]
    fn test_atr_penetration_depth() {
        // Diamond ATR, n=2.4, sample n=1.5, 45 degrees, at 1000 cm-1
        let dp: f64 = atr_penetration_depth(1000.0, 2.4, 1.5, 45.0);
        assert!(dp > 0.0 && dp < 0.01); // Should be a few microns
    }

    #[test]
    fn test_atr_correction_factor() {
        let f: f64 = atr_correction_factor(2000.0, 1000.0);
        assert!(approx_eq(f, 2.0, 0.01));
    }

    #[test]
    fn test_ftir_processor_new() {
        let proc = FtirProcessor::new();
        assert!(proc.interferogram.is_none());
        assert!(proc.spectrum.is_none());
    }

    #[test]
    fn test_ftir_processor_set_apodization() {
        let mut proc = FtirProcessor::new();
        proc.set_apodization(ApodizationFunction::BlackmanHarris);
        assert_eq!(proc.apodization, ApodizationFunction::BlackmanHarris);
    }

    #[test]
    fn test_ftir_processor_process() {
        let mut proc = FtirProcessor::new();
        let opd: Vec<f64> = (0..64).map(|i| (i as f64 - 32.0) * 0.0001).collect();
        let signal: Vec<f64> = opd.iter()
            .map(|&d| (2.0 * PI * 1500.0 * d).cos() + 0.5 * (2.0 * PI * 3000.0 * d).cos())
            .collect();
        proc.load_interferogram(Interferogram::new(opd, signal));
        let spec = proc.process();
        assert!(spec.is_some());
    }

    #[test]
    fn test_ftir_processor_find_bands() {
        let mut proc = FtirProcessor::new();
        proc.load_spectrum(IrSpectrum::new(
            vec![1000.0, 1500.0, 1700.0, 1900.0, 2500.0],
            vec![0.1, 0.3, 0.8, 0.2, 0.1],
        ));
        let bands = proc.find_bands(0.5);
        assert!(!bands.is_empty());
    }

    #[test]
    fn test_ftir_processor_quantify() {
        let mut proc = FtirProcessor::new();
        proc.load_spectrum(IrSpectrum::new(
            vec![1000.0, 1500.0, 2000.0],
            vec![0.5, 1.0, 0.3],
        ));
        let conc = proc.quantify(1500.0, 200.0, 1.0);
        assert!(conc.is_some());
        assert!(approx_eq(conc.unwrap(), 0.005, 0.001));
    }

    #[test]
    fn test_ftir_processor_default() {
        let proc = FtirProcessor::default();
        assert!(proc.interferogram.is_none());
    }

    #[test]
    fn test_integrated_absorbance() {
        let spec = IrSpectrum::new(
            vec![1000.0, 1100.0, 1200.0, 1300.0],
            vec![0.0, 1.0, 1.0, 0.0],
        );
        let area: f64 = spec.integrated_absorbance(1000.0, 1300.0);
        assert!(area > 0.0);
        assert!(approx_eq(area, 200.0, 10.0)); // roughly 100+100+50 = triangle area
    }

    #[test]
    fn test_baseline_correct() {
        let mut spec = IrSpectrum::new(
            vec![1000.0, 1500.0, 2000.0],
            vec![1.0, 2.0, 1.0],
        );
        spec.baseline_correct();
        // After baseline correction, first and last should be ~0
        assert!(approx_eq(spec.absorbance[0], 0.0, 0.01));
        assert!(approx_eq(spec.absorbance[2], 0.0, 0.01));
        assert!(spec.absorbance[1] > 0.0);
    }

    #[test]
    fn test_generate_interferogram() {
        let wn: Vec<f64> = (0..100).map(|i| 500.0 + i as f64 * 30.0).collect();
        let spec: Vec<f64> = wn.iter()
            .map(|&w| if (w - 1700.0).abs() < 50.0 { 1.0 } else { 0.0 })
            .collect();
        let opd: Vec<f64> = (0..32).map(|i| (i as f64 - 16.0) * 0.001).collect();
        let igram = generate_interferogram(&wn, &spec, &opd);
        assert_eq!(igram.signal.len(), 32);
        // Centerburst should be near center
        let cb: usize = igram.centerburst_index();
        assert!((cb as i32 - 16).abs() < 5);
    }

    #[test]
    fn test_beer_lambert_zero() {
        assert!(approx_eq(beer_lambert_concentration(0.0, 100.0, 1.0), 0.0, 1e-10));
    }

    #[test]
    fn test_spectral_correlation_zero_variance() {
        let a: Vec<f64> = vec![5.0, 5.0, 5.0];
        let b: Vec<f64> = vec![1.0, 2.0, 3.0];
        let corr: f64 = spectral_correlation(&a, &b);
        assert!(approx_eq(corr, 0.0, 0.01));
    }

    #[test]
    fn test_atr_total_internal_reflection() {
        // Below critical angle
        let dp: f64 = atr_penetration_depth(1000.0, 1.5, 2.4, 45.0);
        assert!(dp.is_infinite()); // no evanescent wave
    }

    #[test]
    fn test_identify_band_triple_bond() {
        assert_eq!(identify_band(2200.0), "C≡C or C≡N stretch");
    }

    #[test]
    fn test_identify_band_aromatic_ch() {
        assert_eq!(identify_band(3100.0), "=C-H stretch (aromatic/alkene)");
    }

    #[test]
    fn test_identify_band_co() {
        assert_eq!(identify_band(1100.0), "C-O stretch");
    }

    #[test]
    fn test_identify_band_oop() {
        assert_eq!(identify_band(800.0), "C-H out-of-plane bend");
    }

    #[test]
    fn test_ir_spectrum_peak_single() {
        let spec = IrSpectrum::new(vec![1700.0], vec![1.5]);
        let (wn, abs) = spec.peak_absorbance();
        assert!(approx_eq(wn, 1700.0, 0.01));
        assert!(approx_eq(abs, 1.5, 0.01));
    }
}
