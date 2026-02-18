// isoelectric_focusing_processor.rs
//
// Isoelectric Focusing (IEF) signal processing for protein separation.
// pH gradient analysis, pI determination, Henderson-Hasselbalch charge modeling,
// band resolution, focusing kinetics, ampholyte conductivity, pI marker calibration.

/// Standard amino acid pKa: (pKa_NH3+, pKa_COOH, pKa_sidechain). 0.0 = no ionizable sidechain.
pub const AMINO_ACID_PKA: [(f64, f64, f64); 20] = [
    (9.87,2.34,0.0),(9.09,1.82,12.48),(8.80,2.02,3.65),(9.82,1.99,3.65),
    (10.78,1.71,8.33),(9.13,2.17,4.25),(9.67,2.17,0.0),(9.60,2.34,0.0),
    (9.17,1.77,6.00),(9.76,2.32,0.0),(9.74,2.33,0.0),(8.95,2.16,10.54),
    (9.21,2.28,0.0),(9.31,2.20,0.0),(10.60,1.99,0.0),(9.15,2.21,0.0),
    (9.12,2.09,0.0),(9.39,2.38,0.0),(9.11,2.20,10.07),(9.72,2.29,0.0),
];

/// IEF data point: position along gel/capillary and UV/fluorescence intensity.
#[derive(Debug, Clone, Copy)]
pub struct IefDataPoint { pub position_cm: f64, pub intensity: f64 }

/// Detected band (peak) in the IEF profile.
#[derive(Debug, Clone, Copy)]
pub struct IefBand {
    pub position_cm: f64, pub peak_intensity: f64,
    pub fwhm_cm: f64, pub pi_estimate: f64,
}

/// pH gradient linearity assessment result.
#[derive(Debug, Clone)]
pub struct GradientLinearityResult {
    pub slope: f64, pub intercept: f64, pub r_squared: f64, pub max_residual: f64,
}

/// Ionizable group on a protein.
#[derive(Debug, Clone, Copy)]
pub struct IonizableGroup {
    pub pka: f64, pub charge_protonated: f64, pub charge_deprotonated: f64, pub count: u32,
}

/// Known pI calibration marker protein.
#[derive(Debug, Clone, Copy)]
pub struct PiMarker { pub known_pi: f64, pub position_cm: f64 }

/// pI marker calibration result (linear regression pH vs position).
#[derive(Debug, Clone)]
pub struct PiCalibrationResult {
    pub slope: f64, pub intercept: f64, pub r_squared: f64, pub residuals: Vec<f64>,
}

/// Conductivity measurement at a gel position.
#[derive(Debug, Clone, Copy)]
pub struct ConductivityPoint { pub position_cm: f64, pub conductivity_ms_cm: f64 }

/// Focusing time estimation result.
#[derive(Debug, Clone, Copy)]
pub struct FocusingTimeResult {
    pub time_s: f64, pub diffusion_coeff: f64, pub mobility: f64, pub field_v_cm: f64,
}

/// Peak capacity (resolution) result.
#[derive(Debug, Clone, Copy)]
pub struct PeakCapacityResult { pub peak_capacity: f64, pub ph_range: f64, pub min_delta_pi: f64 }

// ---------- 1. IEF Profile ----------

/// Build IEF profile from parallel position/intensity arrays.
pub fn build_profile(positions: &[f64], intensities: &[f64]) -> Vec<IefDataPoint> {
    let n = positions.len().min(intensities.len());
    (0..n).map(|i| IefDataPoint { position_cm: positions[i], intensity: intensities[i] }).collect()
}

/// Detect bands (peaks) in profile. Handles plateau peaks by reporting midpoint.
pub fn detect_bands(profile: &[IefDataPoint], min_intensity: f64) -> Vec<IefBand> {
    if profile.len() < 3 { return Vec::new(); }
    let mut bands = Vec::new();
    let n = profile.len();
    let mut i = 1;
    while i < n - 1 {
        let curr = profile[i].intensity;
        if curr < min_intensity || curr < profile[i - 1].intensity { i += 1; continue; }
        let start = i;
        while i + 1 < n && profile[i + 1].intensity == curr { i += 1; }
        let end = i;
        if profile[start - 1].intensity < curr && (end + 1 >= n || profile[end + 1].intensity < curr) {
            let mid = (start + end) / 2;
            bands.push(IefBand {
                position_cm: profile[mid].position_cm, peak_intensity: curr,
                fwhm_cm: estimate_fwhm(profile, mid), pi_estimate: 0.0,
            });
        }
        i += 1;
    }
    bands
}

fn estimate_fwhm(profile: &[IefDataPoint], ci: usize) -> f64 {
    let hm = profile[ci].intensity * 0.5;
    let mut left = profile[ci].position_cm;
    for j in (0..ci).rev() {
        if profile[j].intensity <= hm {
            let d = profile[j + 1].intensity - profile[j].intensity;
            let f = if d.abs() > 1e-30 { (hm - profile[j].intensity) / d } else { 0.5 };
            left = profile[j].position_cm + f * (profile[j + 1].position_cm - profile[j].position_cm);
            break;
        }
    }
    let mut right = profile[ci].position_cm;
    for j in (ci + 1)..profile.len() {
        if profile[j].intensity <= hm {
            let d = profile[j - 1].intensity - profile[j].intensity;
            let f = if d.abs() > 1e-30 { (hm - profile[j].intensity) / d } else { 0.5 };
            right = profile[j].position_cm - f * (profile[j].position_cm - profile[j - 1].position_cm);
            break;
        }
    }
    (right - left).abs()
}

// ---------- 2. pI Determination ----------

/// Determine pI from band position in a linear pH gradient.
pub fn pi_from_position(pos: f64, ph_low: f64, ph_high: f64, gel_len: f64) -> f64 {
    if gel_len.abs() < 1e-30 { return 0.0; }
    ph_low + (ph_high - ph_low) * pos / gel_len
}

/// Assign pI values to detected bands using a linear pH gradient.
pub fn assign_pi_values(bands: &mut [IefBand], ph_low: f64, ph_high: f64, gel_len: f64) {
    for b in bands.iter_mut() { b.pi_estimate = pi_from_position(b.position_cm, ph_low, ph_high, gel_len); }
}

// ---------- 3. Henderson-Hasselbalch ----------

/// Charge of a single ionizable group at given pH via Henderson-Hasselbalch.
/// Fraction protonated = 10^(pKa-pH) / (1 + 10^(pKa-pH)).
pub fn henderson_hasselbalch_charge(ph: f64, pka: f64, ch_prot: f64, ch_deprot: f64) -> f64 {
    let f = fraction_protonated(ph, pka);
    ch_prot * f + ch_deprot * (1.0 - f)
}

/// Fraction of group in protonated form at given pH.
pub fn fraction_protonated(ph: f64, pka: f64) -> f64 {
    let e = pka - ph;
    if e > 30.0 { 1.0 } else if e < -30.0 { 0.0 } else { let p = 10.0_f64.powf(e); p / (1.0 + p) }
}

// ---------- 4. Protein Net Charge ----------

/// Net charge of protein at given pH from its ionizable groups.
pub fn protein_net_charge(ph: f64, groups: &[IonizableGroup]) -> f64 {
    groups.iter().map(|g| henderson_hasselbalch_charge(ph, g.pka, g.charge_protonated, g.charge_deprotonated) * g.count as f64).sum()
}

/// Charge vs pH curve over a range.
pub fn charge_vs_ph(groups: &[IonizableGroup], ph_min: f64, ph_max: f64, steps: usize) -> Vec<(f64, f64)> {
    if steps == 0 { return Vec::new(); }
    let s = if steps > 1 { (ph_max - ph_min) / (steps - 1) as f64 } else { 0.0 };
    (0..steps).map(|i| { let ph = ph_min + i as f64 * s; (ph, protein_net_charge(ph, groups)) }).collect()
}

/// Estimate pI by bisection (pH where net charge = 0).
pub fn estimate_pi(groups: &[IonizableGroup], ph_min: f64, ph_max: f64) -> f64 {
    let (mut lo, mut hi) = (ph_min, ph_max);
    if protein_net_charge(lo, groups) < protein_net_charge(hi, groups) { std::mem::swap(&mut lo, &mut hi); }
    for _ in 0..100 {
        let mid = (lo + hi) * 0.5;
        let c = protein_net_charge(mid, groups);
        if c.abs() < 1e-6 { return mid; }
        if c > 0.0 { lo = mid; } else { hi = mid; }
    }
    (lo + hi) * 0.5
}

/// Build ionizable groups from amino acid composition. `comp`: (aa_index, count).
/// Always includes N-terminal (pKa 8.0, +1/0) and C-terminal (pKa 3.1, 0/-1).
pub fn build_protein_groups(comp: &[(usize, u32)]) -> Vec<IonizableGroup> {
    let mut g = vec![
        IonizableGroup { pka: 8.0, charge_protonated: 1.0, charge_deprotonated: 0.0, count: 1 },
        IonizableGroup { pka: 3.1, charge_protonated: 0.0, charge_deprotonated: -1.0, count: 1 },
    ];
    for &(idx, cnt) in comp {
        if idx >= 20 || cnt == 0 { continue; }
        let pka_side = AMINO_ACID_PKA[idx].2;
        if pka_side < 0.1 { continue; }
        let (cp, cd) = match idx {
            3|4|5|18 => (0.0, -1.0), 1|8|11 => (1.0, 0.0), _ => continue,
        };
        g.push(IonizableGroup { pka: pka_side, charge_protonated: cp, charge_deprotonated: cd, count: cnt });
    }
    g
}

// ---------- 5. pH Gradient Linearity ----------

/// Assess linearity of measured pH gradient. Input: (position_cm, pH) pairs.
pub fn assess_gradient_linearity(meas: &[(f64, f64)]) -> GradientLinearityResult {
    if meas.len() < 2 { return GradientLinearityResult { slope: 0.0, intercept: 0.0, r_squared: 0.0, max_residual: 0.0 }; }
    let x: Vec<f64> = meas.iter().map(|m| m.0).collect();
    let y: Vec<f64> = meas.iter().map(|m| m.1).collect();
    let (slope, intercept, r_squared) = linreg(&x, &y);
    let max_residual = meas.iter().map(|&(xi, yi)| (yi - (slope * xi + intercept)).abs()).fold(0.0_f64, f64::max);
    GradientLinearityResult { slope, intercept, r_squared, max_residual }
}

fn linreg(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 { return (0.0, 0.0, 0.0); }
    let nf = n as f64;
    let (sx, sy): (f64, f64) = (x[..n].iter().sum(), y[..n].iter().sum());
    let sxx: f64 = x[..n].iter().map(|v| v * v).sum();
    let sxy: f64 = x[..n].iter().zip(&y[..n]).map(|(a, b)| a * b).sum();
    let d = nf * sxx - sx * sx;
    if d.abs() < 1e-30 { return (0.0, sy / nf, 0.0); }
    let slope = (nf * sxy - sx * sy) / d;
    let intercept = (sy - slope * sx) / nf;
    let ym = sy / nf;
    let ss_tot: f64 = y[..n].iter().map(|v| (v - ym).powi(2)).sum();
    let ss_res: f64 = x[..n].iter().zip(&y[..n]).map(|(xi, yi)| (yi - (slope * xi + intercept)).powi(2)).sum();
    (slope, intercept, if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 1.0 })
}

// ---------- 6. Band Width and Resolution ----------

/// Minimum resolvable pI difference (Svensson-Rilbe):
/// delta_pI = 3 * sqrt(D * (dpH/dx) / (E * |du/dpH|))
pub fn min_delta_pi(diff: f64, dph_dx: f64, field: f64, du_dph: f64) -> f64 {
    if field.abs() < 1e-30 || du_dph.abs() < 1e-30 { return f64::INFINITY; }
    let a = diff * dph_dx / (field * du_dph.abs());
    if a < 0.0 { f64::INFINITY } else { 3.0 * a.sqrt() }
}

/// Peak capacity = pH_range / min_delta_pI.
pub fn peak_capacity(ph_range: f64, diff: f64, dph_dx: f64, field: f64, du_dph: f64) -> PeakCapacityResult {
    let mdp = min_delta_pi(diff, dph_dx, field, du_dph);
    let nc = if mdp.is_finite() && mdp > 0.0 { ph_range / mdp } else { 0.0 };
    PeakCapacityResult { peak_capacity: nc, ph_range, min_delta_pi: mdp }
}

/// Resolution between two bands: Rs = 2*|x1-x2| / (w1+w2), w = 1.7*FWHM.
pub fn band_resolution(b1: &IefBand, b2: &IefBand) -> f64 {
    let d = b1.fwhm_cm * 1.7 + b2.fwhm_cm * 1.7;
    if d < 1e-30 { 0.0 } else { 2.0 * (b1.position_cm - b2.position_cm).abs() / d }
}

// ---------- 7. Focusing Time ----------

/// Estimate focusing time: t = L / (mobility * E).
pub fn estimate_focusing_time(gel_len: f64, mobility: f64, field: f64) -> FocusingTimeResult {
    let v = mobility * field;
    let t = if v.abs() > 1e-30 { gel_len / v } else { f64::INFINITY };
    FocusingTimeResult { time_s: t, diffusion_coeff: 5e-7, mobility, field_v_cm: field }
}

/// Steady-state band sigma = sqrt(D / (E * |du/dpH| * dpH/dx)).
pub fn steady_state_bandwidth(diff: f64, field: f64, du_dph: f64, dph_dx: f64) -> f64 {
    let d = field * du_dph.abs() * dph_dx.abs();
    if d < 1e-30 { f64::INFINITY } else { (diff / d).sqrt() }
}

// ---------- 8. Ampholyte Conductivity ----------

/// Parabolic conductivity profile: kappa(x) = kmin + (kmax-kmin)*(2*(x-L/2)/L)^2.
pub fn ampholyte_conductivity_profile(gel_len: f64, n: usize, kmin: f64, kmax: f64) -> Vec<ConductivityPoint> {
    if n == 0 { return Vec::new(); }
    let mid = gel_len * 0.5;
    let range = kmax - kmin;
    let step = if n > 1 { gel_len / (n - 1) as f64 } else { 0.0 };
    (0..n).map(|i| {
        let x = i as f64 * step;
        let u = if gel_len > 1e-30 { 2.0 * (x - mid) / gel_len } else { 0.0 };
        ConductivityPoint { position_cm: x, conductivity_ms_cm: kmin + range * u * u }
    }).collect()
}

/// Average conductivity across profile.
pub fn average_conductivity(p: &[ConductivityPoint]) -> f64 {
    if p.is_empty() { return 0.0; }
    p.iter().map(|c| c.conductivity_ms_cm).sum::<f64>() / p.len() as f64
}

/// Gel current (mA) = V * A * kappa_avg / L.
pub fn gel_current_ma(v: f64, gel_len: f64, area: f64, kappa: f64) -> f64 {
    if gel_len.abs() < 1e-30 { 0.0 } else { v * area * kappa / gel_len }
}

// ---------- 9. pI Markers Calibration ----------

/// Calibrate pH gradient using known pI markers via linear regression.
pub fn calibrate_pi_markers(markers: &[PiMarker]) -> PiCalibrationResult {
    if markers.len() < 2 { return PiCalibrationResult { slope: 0.0, intercept: 0.0, r_squared: 0.0, residuals: Vec::new() }; }
    let x: Vec<f64> = markers.iter().map(|m| m.position_cm).collect();
    let y: Vec<f64> = markers.iter().map(|m| m.known_pi).collect();
    let (slope, intercept, r2) = linreg(&x, &y);
    let res = markers.iter().map(|m| m.known_pi - (slope * m.position_cm + intercept)).collect();
    PiCalibrationResult { slope, intercept, r_squared: r2, residuals: res }
}

/// Look up pI from calibrated linear regression.
pub fn pi_from_calibration(pos: f64, cal: &PiCalibrationResult) -> f64 { cal.slope * pos + cal.intercept }

// ---------- 10. IefProcessor ----------

/// Orchestrator for IEF signal processing.
pub struct IefProcessor {
    pub ph_low: f64, pub ph_high: f64, pub gel_length_cm: f64, pub field_v_cm: f64,
    pub bands: Vec<IefBand>, pub calibration: Option<PiCalibrationResult>,
    pub conductivity: Vec<ConductivityPoint>,
}

impl IefProcessor {
    /// Create new processor with pH range, gel length, and field strength.
    pub fn new(ph_low: f64, ph_high: f64, gel_length_cm: f64, field_v_cm: f64) -> Self {
        Self { ph_low, ph_high, gel_length_cm, field_v_cm, bands: Vec::new(), calibration: None, conductivity: Vec::new() }
    }
    /// Standard IPG strip: pH 3-10, 7 cm, 200 V/cm.
    pub fn standard_ipg_strip() -> Self { Self::new(3.0, 10.0, 7.0, 200.0) }
    /// Narrow-range IPG strip: pH 4-7, 11 cm, 300 V/cm.
    pub fn narrow_range_ipg() -> Self { Self::new(4.0, 7.0, 11.0, 300.0) }

    /// Detect bands and assign pI values (calibrated or linear gradient).
    pub fn process_profile(&mut self, profile: &[IefDataPoint], min_intensity: f64) {
        self.bands = detect_bands(profile, min_intensity);
        if let Some(ref cal) = self.calibration {
            for b in &mut self.bands { b.pi_estimate = pi_from_calibration(b.position_cm, cal); }
        } else {
            assign_pi_values(&mut self.bands, self.ph_low, self.ph_high, self.gel_length_cm);
        }
    }
    /// Calibrate using pI marker proteins.
    pub fn calibrate(&mut self, markers: &[PiMarker]) { self.calibration = Some(calibrate_pi_markers(markers)); }
    /// Compute conductivity profile.
    pub fn compute_conductivity(&mut self, n: usize, kmin: f64, kmax: f64) {
        self.conductivity = ampholyte_conductivity_profile(self.gel_length_cm, n, kmin, kmax);
    }
    /// pH gradient slope (pH/cm).
    pub fn ph_gradient_slope(&self) -> f64 {
        if self.gel_length_cm.abs() < 1e-30 { 0.0 } else { (self.ph_high - self.ph_low) / self.gel_length_cm }
    }
    /// Estimate peak capacity for this configuration.
    pub fn estimate_peak_capacity(&self, diff: f64, du_dph: f64) -> PeakCapacityResult {
        peak_capacity(self.ph_high - self.ph_low, diff, self.ph_gradient_slope(), self.field_v_cm, du_dph)
    }
    /// Estimate focusing time for given mobility.
    pub fn estimate_focusing_time(&self, mobility: f64) -> FocusingTimeResult {
        estimate_focusing_time(self.gel_length_cm, mobility, self.field_v_cm)
    }
    /// Resolution between two bands by index.
    pub fn resolution_between(&self, i: usize, j: usize) -> f64 {
        if i >= self.bands.len() || j >= self.bands.len() { return 0.0; }
        band_resolution(&self.bands[i], &self.bands[j])
    }
    /// Get all pI values.
    pub fn pi_values(&self) -> Vec<f64> { self.bands.iter().map(|b| b.pi_estimate).collect() }
    /// Number of detected bands.
    pub fn num_bands(&self) -> usize { self.bands.len() }
}

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    fn gauss(center: f64, sigma: f64, n: usize, len: f64) -> Vec<IefDataPoint> {
        (0..n).map(|i| {
            let x = i as f64 * len / (n - 1) as f64;
            IefDataPoint { position_cm: x, intensity: (-0.5 * ((x - center) / sigma).powi(2)).exp() }
        }).collect()
    }
    fn two_gauss(c1: f64, c2: f64, sigma: f64, n: usize, len: f64) -> Vec<IefDataPoint> {
        (0..n).map(|i| {
            let x = i as f64 * len / (n - 1) as f64;
            IefDataPoint { position_cm: x,
                intensity: (-0.5*((x-c1)/sigma).powi(2)).exp() + (-0.5*((x-c2)/sigma).powi(2)).exp() }
        }).collect()
    }
    fn simple_groups() -> Vec<IonizableGroup> {
        vec![
            IonizableGroup { pka: 8.0, charge_protonated: 1.0, charge_deprotonated: 0.0, count: 1 },
            IonizableGroup { pka: 3.1, charge_protonated: 0.0, charge_deprotonated: -1.0, count: 1 },
            IonizableGroup { pka: 6.0, charge_protonated: 1.0, charge_deprotonated: 0.0, count: 1 },
            IonizableGroup { pka: 3.65, charge_protonated: 0.0, charge_deprotonated: -1.0, count: 1 },
        ]
    }

    // 1. Profile
    #[test] fn test_build_profile() { let p = build_profile(&[0.0,1.0,2.0], &[0.1,0.5,0.2]); assert_eq!(p.len(), 3); }
    #[test] fn test_build_profile_unequal() { assert_eq!(build_profile(&[0.0,1.0], &[0.1,0.5,0.9]).len(), 2); }
    #[test] fn test_detect_single_band() {
        let b = detect_bands(&gauss(3.5, 0.3, 201, 7.0), 0.1);
        assert_eq!(b.len(), 1); assert!(approx_eq(b[0].position_cm, 3.5, 0.1));
    }
    #[test] fn test_detect_plateau_band() { assert_eq!(detect_bands(&gauss(3.5, 0.3, 200, 7.0), 0.1).len(), 1); }
    #[test] fn test_detect_two_bands() { assert_eq!(detect_bands(&two_gauss(2.0, 5.0, 0.3, 201, 7.0), 0.1).len(), 2); }
    #[test] fn test_no_bands_threshold() { assert_eq!(detect_bands(&gauss(3.5, 0.3, 201, 7.0), 2.0).len(), 0); }
    #[test] fn test_fwhm() {
        let b = detect_bands(&gauss(3.5, 0.3, 501, 7.0), 0.1);
        assert_eq!(b.len(), 1); assert!(approx_eq(b[0].fwhm_cm, 2.355 * 0.3, 0.05));
    }
    #[test] fn test_empty_profile() { assert!(detect_bands(&[], 0.1).is_empty()); }

    // 2. pI Determination
    #[test] fn test_pi_midpoint() { assert!(approx_eq(pi_from_position(3.5, 3.0, 10.0, 7.0), 6.5, 0.01)); }
    #[test] fn test_pi_endpoints() {
        assert!(approx_eq(pi_from_position(0.0, 3.0, 10.0, 7.0), 3.0, 0.01));
        assert!(approx_eq(pi_from_position(7.0, 3.0, 10.0, 7.0), 10.0, 0.01));
    }
    #[test] fn test_assign_pi() {
        let mut b = vec![
            IefBand { position_cm: 1.0, peak_intensity: 1.0, fwhm_cm: 0.1, pi_estimate: 0.0 },
            IefBand { position_cm: 5.0, peak_intensity: 1.0, fwhm_cm: 0.1, pi_estimate: 0.0 },
        ];
        assign_pi_values(&mut b, 3.0, 10.0, 7.0);
        assert!(approx_eq(b[0].pi_estimate, 4.0, 0.01));
        assert!(approx_eq(b[1].pi_estimate, 8.0, 0.01));
    }

    // 3. Henderson-Hasselbalch
    #[test] fn test_hh_at_pka() { assert!(approx_eq(henderson_hasselbalch_charge(7.0, 7.0, 1.0, 0.0), 0.5, 0.01)); }
    #[test] fn test_hh_protonated() { assert!(approx_eq(henderson_hasselbalch_charge(2.0, 10.0, 1.0, 0.0), 1.0, 0.01)); }
    #[test] fn test_hh_deprotonated() { assert!(approx_eq(henderson_hasselbalch_charge(14.0, 4.0, 0.0, -1.0), -1.0, 0.01)); }
    #[test] fn test_frac_prot_pka() { assert!(approx_eq(fraction_protonated(5.0, 5.0), 0.5, 0.001)); }
    #[test] fn test_frac_prot_low() { assert!(fraction_protonated(1.0, 7.0) > 0.99); }
    #[test] fn test_frac_prot_high() { assert!(fraction_protonated(14.0, 4.0) < 0.01); }

    // 4. Net Charge
    #[test] fn test_charge_low_ph() { assert!(approx_eq(protein_net_charge(1.0, &simple_groups()), 2.0, 0.1)); }
    #[test] fn test_charge_high_ph() { assert!(approx_eq(protein_net_charge(14.0, &simple_groups()), -2.0, 0.1)); }
    #[test] fn test_pi_exists() { let pi = estimate_pi(&simple_groups(), 1.0, 14.0); assert!(pi > 3.0 && pi < 8.0); }
    #[test] fn test_charge_at_pi_zero() {
        let g = simple_groups(); let pi = estimate_pi(&g, 1.0, 14.0);
        assert!(approx_eq(protein_net_charge(pi, &g), 0.0, 0.01));
    }
    #[test] fn test_charge_curve() {
        let c = charge_vs_ph(&simple_groups(), 1.0, 14.0, 100);
        assert_eq!(c.len(), 100); assert!(c[0].1 > 0.0); assert!(c[99].1 < 0.0);
    }
    #[test] fn test_build_groups_comp() { assert_eq!(build_protein_groups(&[(11,2),(3,1),(8,1)]).len(), 5); }
    #[test] fn test_build_groups_empty() { assert_eq!(build_protein_groups(&[]).len(), 2); }

    // 5. Gradient Linearity
    #[test] fn test_linear_perfect() {
        let d: Vec<_> = (0..10).map(|i| { let x = i as f64; (x, 3.0 + 0.7 * x) }).collect();
        let r = assess_gradient_linearity(&d);
        assert!(approx_eq(r.slope, 0.7, 0.001)); assert!(r.r_squared > 0.999);
    }
    #[test] fn test_linear_nonlinear() {
        let d: Vec<_> = (0..10).map(|i| { let x = i as f64; (x, 3.0 + 0.5*x + 0.05*x*x) }).collect();
        assert!(assess_gradient_linearity(&d).max_residual > 0.01);
    }
    #[test] fn test_linear_insufficient() { assert!(approx_eq(assess_gradient_linearity(&[(1.0,5.0)]).r_squared, 0.0, 1e-10)); }

    // 6. Resolution
    #[test] fn test_min_delta_pi_val() { let d = min_delta_pi(5e-7, 1.0, 200.0, 1e-5); assert!(d > 0.0 && d < 1.0); }
    #[test] fn test_peak_cap() { assert!(peak_capacity(7.0, 5e-7, 1.0, 200.0, 1e-5).peak_capacity > 1.0); }
    #[test] fn test_resolution_well_sep() {
        let b1 = IefBand { position_cm:1.0, peak_intensity:1.0, fwhm_cm:0.2, pi_estimate:4.0 };
        let b2 = IefBand { position_cm:3.0, peak_intensity:1.0, fwhm_cm:0.2, pi_estimate:6.0 };
        assert!(band_resolution(&b1, &b2) > 1.5);
    }
    #[test] fn test_resolution_overlap() {
        let b1 = IefBand { position_cm:1.0, peak_intensity:1.0, fwhm_cm:0.5, pi_estimate:4.0 };
        let b2 = IefBand { position_cm:1.3, peak_intensity:1.0, fwhm_cm:0.5, pi_estimate:4.3 };
        assert!(band_resolution(&b1, &b2) < 1.0);
    }

    // 7. Focusing Time
    #[test] fn test_focus_time() { assert!(approx_eq(estimate_focusing_time(7.0, 3e-5, 200.0).time_s, 7.0/0.006, 1.0)); }
    #[test] fn test_bandwidth() { let s = steady_state_bandwidth(5e-7, 200.0, 1e-5, 1.0); assert!(s > 0.0 && s < 1.0); }
    #[test] fn test_focus_zero_field() { assert!(estimate_focusing_time(7.0, 3e-5, 0.0).time_s.is_infinite()); }

    // 8. Conductivity
    #[test] fn test_cond_parabolic() {
        let p = ampholyte_conductivity_profile(7.0, 100, 0.5, 2.0);
        assert!(p[50].conductivity_ms_cm < p[0].conductivity_ms_cm);
    }
    #[test] fn test_cond_endpoints() {
        let p = ampholyte_conductivity_profile(7.0, 100, 0.5, 2.0);
        assert!(approx_eq(p[0].conductivity_ms_cm, 2.0, 0.05));
    }
    #[test] fn test_avg_cond() { assert!(approx_eq(average_conductivity(&ampholyte_conductivity_profile(7.0, 101, 1.0, 3.0)), 1.667, 0.05)); }
    #[test] fn test_current() { assert!(approx_eq(gel_current_ma(1000.0, 7.0, 0.1, 1.0), 14.29, 0.1)); }
    #[test] fn test_cond_empty() { assert!(ampholyte_conductivity_profile(7.0, 0, 0.5, 2.0).is_empty()); }

    // 9. Calibration
    #[test] fn test_calibrate_markers() {
        let m = vec![PiMarker{known_pi:3.5,position_cm:0.5}, PiMarker{known_pi:5.1,position_cm:2.0},
            PiMarker{known_pi:6.8,position_cm:3.5}, PiMarker{known_pi:8.3,position_cm:5.0}, PiMarker{known_pi:9.8,position_cm:6.5}];
        assert!(calibrate_pi_markers(&m).r_squared > 0.99);
    }
    #[test] fn test_pi_from_cal() {
        let m = vec![PiMarker{known_pi:3.0,position_cm:0.0}, PiMarker{known_pi:10.0,position_cm:7.0}];
        assert!(approx_eq(pi_from_calibration(3.5, &calibrate_pi_markers(&m)), 6.5, 0.01));
    }
    #[test] fn test_cal_residuals() {
        let m = vec![PiMarker{known_pi:4.0,position_cm:1.0}, PiMarker{known_pi:6.0,position_cm:3.0}, PiMarker{known_pi:8.0,position_cm:5.0}];
        for r in &calibrate_pi_markers(&m).residuals { assert!(r.abs() < 0.001); }
    }

    // 10. Processor
    #[test] fn test_proc_standard() { let p = IefProcessor::standard_ipg_strip(); assert!(approx_eq(p.ph_low, 3.0, 0.01)); }
    #[test] fn test_proc_narrow() { let p = IefProcessor::narrow_range_ipg(); assert!(approx_eq(p.ph_high, 7.0, 0.01)); }
    #[test] fn test_proc_slope() { assert!(approx_eq(IefProcessor::standard_ipg_strip().ph_gradient_slope(), 1.0, 0.01)); }
    #[test] fn test_proc_profile() {
        let mut p = IefProcessor::standard_ipg_strip();
        p.process_profile(&two_gauss(2.0, 5.0, 0.3, 501, 7.0), 0.1);
        assert_eq!(p.num_bands(), 2);
    }
    #[test] fn test_proc_calibrated() {
        let mut p = IefProcessor::standard_ipg_strip();
        p.calibrate(&[PiMarker{known_pi:3.0,position_cm:0.0}, PiMarker{known_pi:10.0,position_cm:7.0}]);
        p.process_profile(&gauss(3.5, 0.3, 501, 7.0), 0.1);
        assert_eq!(p.num_bands(), 1); assert!(approx_eq(p.bands[0].pi_estimate, 6.5, 0.1));
    }
    #[test] fn test_proc_resolution() {
        let mut p = IefProcessor::standard_ipg_strip();
        p.process_profile(&two_gauss(2.0, 5.0, 0.3, 501, 7.0), 0.1);
        assert!(p.resolution_between(0, 1) > 1.0);
    }
    #[test] fn test_proc_res_invalid() { assert!(approx_eq(IefProcessor::standard_ipg_strip().resolution_between(0,1), 0.0, 1e-10)); }
    #[test] fn test_proc_peak_cap() { assert!(IefProcessor::standard_ipg_strip().estimate_peak_capacity(5e-7, 1e-5).peak_capacity > 1.0); }
    #[test] fn test_proc_focus_time() { let t = IefProcessor::standard_ipg_strip().estimate_focusing_time(3e-5); assert!(t.time_s > 0.0); }
    #[test] fn test_proc_conductivity() {
        let mut p = IefProcessor::standard_ipg_strip(); p.compute_conductivity(50, 0.3, 1.5);
        assert_eq!(p.conductivity.len(), 50);
    }
}
