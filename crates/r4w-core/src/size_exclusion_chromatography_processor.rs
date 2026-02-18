// size_exclusion_chromatography_processor.rs
//
// Size Exclusion Chromatography (SEC / GPC) signal processing.
// MW calibration, Mn/Mw/PDI, Mark-Houwink viscosity, universal calibration,
// peak detection, cumulative/differential MW distributions.

/// A single chromatogram data point.
#[derive(Debug, Clone, Copy)]
pub struct SecDataPoint {
    /// Elution volume in mL.
    pub elution_volume_ml: f64,
    /// Detector response (e.g. refractive index, UV absorbance).
    pub response: f64,
}

/// SEC chromatogram: (elution volume, detector response) series.
#[derive(Debug, Clone)]
pub struct SecChromatogram {
    pub points: Vec<SecDataPoint>,
}

/// Linear calibration: log10(MW) = intercept + slope * Ve.
#[derive(Debug, Clone, Copy)]
pub struct LinearCalibration {
    pub intercept: f64,
    pub slope: f64,
    pub r_squared: f64,
}

/// Molecular weight averages.
#[derive(Debug, Clone, Copy)]
pub struct MwAverages {
    /// Number-average MW.
    pub mn: f64,
    /// Weight-average MW.
    pub mw: f64,
    /// Polydispersity index Mw/Mn.
    pub pdi: f64,
    /// Z-average MW.
    pub mz: f64,
    /// Peak MW (at detector maximum).
    pub mp: f64,
}

/// Mark-Houwink parameters: [eta] = K * M^a.
#[derive(Debug, Clone, Copy)]
pub struct MarkHouwinkParams {
    pub k: f64,
    pub a: f64,
}

/// Universal calibration: log10([eta]*M) = intercept + slope * Ve.
#[derive(Debug, Clone, Copy)]
pub struct UniversalCalibration {
    pub intercept: f64,
    pub slope: f64,
    pub r_squared: f64,
}

/// A detected chromatogram peak.
#[derive(Debug, Clone, Copy)]
pub struct ChromPeak {
    pub index: usize,
    pub elution_volume_ml: f64,
    pub response: f64,
    pub mw: f64,
}

/// MW distribution bin.
#[derive(Debug, Clone, Copy)]
pub struct MwDistBin {
    pub log_mw: f64,
    pub mw: f64,
    /// Differential weight fraction dw/d(logM).
    pub dw_dlogm: f64,
    /// Cumulative weight fraction (0..1).
    pub cumulative: f64,
}

/// Main SEC processor orchestrating all analysis.
pub struct SecProcessor {
    pub chromatogram: SecChromatogram,
    pub calibration: Option<LinearCalibration>,
    pub mark_houwink: Option<MarkHouwinkParams>,
    pub universal_cal: Option<UniversalCalibration>,
    pub averages: Option<MwAverages>,
}

// --- 1. Chromatogram Construction ---

impl SecChromatogram {
    /// Create an empty chromatogram.
    pub fn new() -> Self { Self { points: Vec::new() } }

    /// Create from parallel slices of elution volume and response.
    pub fn from_slices(volumes: &[f64], responses: &[f64]) -> Self {
        let n = volumes.len().min(responses.len());
        let points = (0..n).map(|i| SecDataPoint {
            elution_volume_ml: volumes[i], response: responses[i],
        }).collect();
        Self { points }
    }

    /// Add a single data point.
    pub fn add_point(&mut self, volume_ml: f64, response: f64) {
        self.points.push(SecDataPoint { elution_volume_ml: volume_ml, response });
    }

    /// Number of data points.
    pub fn len(&self) -> usize { self.points.len() }
    /// Whether chromatogram is empty.
    pub fn is_empty(&self) -> bool { self.points.is_empty() }
    /// Elution volumes as a vector.
    pub fn volumes(&self) -> Vec<f64> { self.points.iter().map(|p| p.elution_volume_ml).collect() }
    /// Detector responses as a vector.
    pub fn responses(&self) -> Vec<f64> { self.points.iter().map(|p| p.response).collect() }
    /// Maximum detector response.
    pub fn max_response(&self) -> f64 {
        self.points.iter().map(|p| p.response).fold(f64::NEG_INFINITY, f64::max)
    }

    /// Subtract a linear baseline between first and last points.
    pub fn subtract_baseline(&mut self) {
        let n = self.points.len();
        if n < 2 { return; }
        let v0 = self.points[0].elution_volume_ml;
        let r0 = self.points[0].response;
        let v1 = self.points[n - 1].elution_volume_ml;
        let r1 = self.points[n - 1].response;
        let dv = v1 - v0;
        if dv.abs() < 1e-30 { return; }
        let slope = (r1 - r0) / dv;
        for p in &mut self.points {
            p.response -= r0 + slope * (p.elution_volume_ml - v0);
        }
    }
}

// --- 2. Molecular Weight Calibration: log10(MW) = A + B * Ve ---

/// Fit a linear calibration from narrow MW standards: (elution_volume, known_mw).
pub fn fit_linear_calibration(standards: &[(f64, f64)]) -> LinearCalibration {
    let n = standards.len() as f64;
    if n < 2.0 { return LinearCalibration { intercept: 0.0, slope: 0.0, r_squared: 0.0 }; }
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for &(ve, mw) in standards {
        let y = mw.log10();
        sx += ve; sy += y; sxx += ve * ve; sxy += ve * y;
    }
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 { return LinearCalibration { intercept: 0.0, slope: 0.0, r_squared: 0.0 }; }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    let y_mean = sy / n;
    let ss_tot: f64 = standards.iter().map(|&(_, mw)| (mw.log10() - y_mean).powi(2)).sum();
    let ss_res: f64 = standards.iter().map(|&(ve, mw)| (mw.log10() - intercept - slope * ve).powi(2)).sum();
    let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    LinearCalibration { intercept, slope, r_squared }
}

/// Calculate MW from elution volume using linear calibration.
pub fn mw_from_ve(cal: &LinearCalibration, ve: f64) -> f64 {
    pow10(cal.intercept + cal.slope * ve)
}

fn pow10(x: f64) -> f64 { (x * std::f64::consts::LN_10).exp() }

// --- 3, 4, 5. Number/Weight Average MW and PDI ---

/// Compute Mn, Mw, Mz, Mp, and PDI from chromatogram and calibration.
pub fn compute_mw_averages(chrom: &SecChromatogram, cal: &LinearCalibration) -> MwAverages {
    let (mut mp, mut max_resp) = (0.0, f64::NEG_INFINITY);
    let (mut sum_ni, mut sum_ni_mi, mut sum_ni_mi2, mut sum_ni_mi3) = (0.0, 0.0, 0.0, 0.0);
    for p in &chrom.points {
        if p.response <= 0.0 { continue; }
        let mi = mw_from_ve(cal, p.elution_volume_ml);
        if mi <= 0.0 { continue; }
        let ni = p.response / mi;
        sum_ni += ni;
        sum_ni_mi += ni * mi;
        sum_ni_mi2 += ni * mi * mi;
        sum_ni_mi3 += ni * mi * mi * mi;
        if p.response > max_resp { max_resp = p.response; mp = mi; }
    }
    if sum_ni < 1e-30 || sum_ni_mi < 1e-30 {
        return MwAverages { mn: 0.0, mw: 0.0, pdi: 1.0, mz: 0.0, mp };
    }
    let mn = sum_ni_mi / sum_ni;
    let mw = sum_ni_mi2 / sum_ni_mi;
    let mz = sum_ni_mi3 / sum_ni_mi2;
    let pdi = if mn > 1e-30 { mw / mn } else { 1.0 };
    MwAverages { mn, mw, pdi, mz, mp }
}

// --- 6. Viscosity-Average MW (Mark-Houwink) ---

impl MarkHouwinkParams {
    /// Create Mark-Houwink parameters.
    pub fn new(k: f64, a: f64) -> Self { Self { k, a } }
    /// Intrinsic viscosity: [eta] = K * M^a.
    pub fn intrinsic_viscosity(&self, mw: f64) -> f64 { self.k * mw.powf(self.a) }
    /// Hydrodynamic volume: [eta] * M.
    pub fn hydrodynamic_volume(&self, mw: f64) -> f64 { self.intrinsic_viscosity(mw) * mw }
}

/// Viscosity-average MW: Mv = (sum(Ni*Mi^(1+a)) / sum(Ni*Mi))^(1/a).
pub fn viscosity_average_mw(chrom: &SecChromatogram, cal: &LinearCalibration, mh: &MarkHouwinkParams) -> f64 {
    let (mut sum_w, mut sum_wa) = (0.0, 0.0);
    for p in &chrom.points {
        if p.response <= 0.0 { continue; }
        let mi = mw_from_ve(cal, p.elution_volume_ml);
        if mi <= 0.0 { continue; }
        let ni = p.response / mi;
        sum_w += ni * mi;
        sum_wa += ni * mi.powf(1.0 + mh.a);
    }
    if sum_w < 1e-30 || mh.a.abs() < 1e-30 { return 0.0; }
    (sum_wa / sum_w).powf(1.0 / mh.a)
}

// --- 7. Universal Calibration ---

/// Fit universal calibration: log10([eta]*M) = A + B * Ve.
/// Standards: (Ve, MW, intrinsic_viscosity).
pub fn fit_universal_calibration(standards: &[(f64, f64, f64)]) -> UniversalCalibration {
    let n = standards.len() as f64;
    if n < 2.0 { return UniversalCalibration { intercept: 0.0, slope: 0.0, r_squared: 0.0 }; }
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for &(ve, mw, iv) in standards {
        let y = (iv * mw).log10();
        sx += ve; sy += y; sxx += ve * ve; sxy += ve * y;
    }
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 { return UniversalCalibration { intercept: 0.0, slope: 0.0, r_squared: 0.0 }; }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    let y_mean = sy / n;
    let ss_tot: f64 = standards.iter().map(|&(_, mw, iv)| ((iv * mw).log10() - y_mean).powi(2)).sum();
    let ss_res: f64 = standards.iter().map(|&(ve, mw, iv)| ((iv * mw).log10() - intercept - slope * ve).powi(2)).sum();
    let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    UniversalCalibration { intercept, slope, r_squared }
}

/// MW from universal calibration + Mark-Houwink at given Ve.
pub fn mw_from_universal(ucal: &UniversalCalibration, mh: &MarkHouwinkParams, ve: f64) -> f64 {
    let log_hv = ucal.intercept + ucal.slope * ve;
    let log_k = if mh.k > 0.0 { mh.k.log10() } else { 0.0 };
    let exp = 1.0 + mh.a;
    if exp.abs() < 1e-30 { return 0.0; }
    pow10((log_hv - log_k) / exp)
}

// --- 8. Peak Detection ---

/// Find peaks in the chromatogram (local maxima above threshold).
pub fn find_peaks(chrom: &SecChromatogram, threshold: f64) -> Vec<ChromPeak> {
    let n = chrom.points.len();
    if n < 3 { return Vec::new(); }
    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        let c = chrom.points[i].response;
        if c > chrom.points[i - 1].response && c > chrom.points[i + 1].response && c > threshold {
            peaks.push(ChromPeak {
                index: i, elution_volume_ml: chrom.points[i].elution_volume_ml,
                response: c, mw: 0.0,
            });
        }
    }
    peaks
}

/// Find peaks and assign MW via calibration.
pub fn find_peaks_with_mw(chrom: &SecChromatogram, cal: &LinearCalibration, threshold: f64) -> Vec<ChromPeak> {
    let mut peaks = find_peaks(chrom, threshold);
    for peak in &mut peaks { peak.mw = mw_from_ve(cal, peak.elution_volume_ml); }
    peaks
}

/// Find the single highest peak MW.
pub fn find_peak_mw(chrom: &SecChromatogram, cal: &LinearCalibration) -> f64 {
    let mut best_ve = 0.0;
    let mut best_r = f64::NEG_INFINITY;
    for p in &chrom.points {
        if p.response > best_r { best_r = p.response; best_ve = p.elution_volume_ml; }
    }
    mw_from_ve(cal, best_ve)
}

// --- 9. Cumulative and Differential MW Distribution ---

/// Compute MW distribution from chromatogram. Bins sorted by increasing MW.
pub fn mw_distribution(chrom: &SecChromatogram, cal: &LinearCalibration, num_bins: usize) -> Vec<MwDistBin> {
    if chrom.is_empty() || num_bins == 0 { return Vec::new(); }
    let mut pairs: Vec<(f64, f64)> = Vec::new();
    for p in &chrom.points {
        if p.response <= 0.0 { continue; }
        let mi = mw_from_ve(cal, p.elution_volume_ml);
        if mi > 0.0 { pairs.push((mi.log10(), p.response)); }
    }
    if pairs.is_empty() { return Vec::new(); }
    let log_min = pairs.iter().map(|&(l, _)| l).fold(f64::INFINITY, f64::min);
    let log_max = pairs.iter().map(|&(l, _)| l).fold(f64::NEG_INFINITY, f64::max);
    let range = log_max - log_min;
    if range < 1e-30 { return Vec::new(); }
    let bw = range / num_bins as f64;
    let mut bin_w = vec![0.0_f64; num_bins];
    for &(log_m, w) in &pairs {
        let idx = ((log_m - log_min) / bw) as usize;
        bin_w[idx.min(num_bins - 1)] += w;
    }
    let total: f64 = bin_w.iter().sum();
    if total < 1e-30 { return Vec::new(); }
    let mut cum = 0.0_f64;
    (0..num_bins).map(|i| {
        let lc = log_min + (i as f64 + 0.5) * bw;
        cum += bin_w[i] / total;
        MwDistBin { log_mw: lc, mw: pow10(lc), dw_dlogm: bin_w[i] / (total * bw), cumulative: cum }
    }).collect()
}

// --- 10. SecProcessor Orchestrator ---

impl SecProcessor {
    /// Create a new SEC processor with given chromatogram.
    pub fn new(chromatogram: SecChromatogram) -> Self {
        Self { chromatogram, calibration: None, mark_houwink: None, universal_cal: None, averages: None }
    }
    /// Create from volume/response slices.
    pub fn from_slices(volumes: &[f64], responses: &[f64]) -> Self {
        Self::new(SecChromatogram::from_slices(volumes, responses))
    }
    /// Set MW calibration from standards.
    pub fn calibrate(&mut self, standards: &[(f64, f64)]) -> &LinearCalibration {
        self.calibration = Some(fit_linear_calibration(standards));
        self.calibration.as_ref().unwrap()
    }
    /// Set Mark-Houwink parameters.
    pub fn set_mark_houwink(&mut self, k: f64, a: f64) {
        self.mark_houwink = Some(MarkHouwinkParams::new(k, a));
    }
    /// Set universal calibration from standards with viscosity data.
    pub fn calibrate_universal(&mut self, standards: &[(f64, f64, f64)]) {
        self.universal_cal = Some(fit_universal_calibration(standards));
    }
    /// Subtract baseline from chromatogram.
    pub fn subtract_baseline(&mut self) { self.chromatogram.subtract_baseline(); }
    /// Compute MW averages (requires calibration).
    pub fn compute_averages(&mut self) -> Option<MwAverages> {
        let cal = self.calibration.as_ref()?;
        let avg = compute_mw_averages(&self.chromatogram, cal);
        self.averages = Some(avg);
        Some(avg)
    }
    /// Viscosity-average MW (requires calibration + Mark-Houwink).
    pub fn viscosity_average(&self) -> Option<f64> {
        let cal = self.calibration.as_ref()?;
        let mh = self.mark_houwink.as_ref()?;
        Some(viscosity_average_mw(&self.chromatogram, cal, mh))
    }
    /// Find chromatogram peaks.
    pub fn find_peaks(&self, threshold: f64) -> Vec<ChromPeak> {
        match &self.calibration {
            Some(cal) => find_peaks_with_mw(&self.chromatogram, cal, threshold),
            None => find_peaks(&self.chromatogram, threshold),
        }
    }
    /// Compute MW distribution.
    pub fn mw_distribution(&self, num_bins: usize) -> Vec<MwDistBin> {
        match &self.calibration {
            Some(cal) => mw_distribution(&self.chromatogram, cal, num_bins),
            None => Vec::new(),
        }
    }
    /// MW at a given elution volume (requires calibration).
    pub fn mw_at_volume(&self, ve: f64) -> Option<f64> {
        Some(mw_from_ve(self.calibration.as_ref()?, ve))
    }
    /// MW via universal calibration (requires universal cal + Mark-Houwink).
    pub fn mw_at_volume_universal(&self, ve: f64) -> Option<f64> {
        Some(mw_from_universal(self.universal_cal.as_ref()?, self.mark_houwink.as_ref()?, ve))
    }
    /// Summary string.
    pub fn summary(&self) -> String {
        let mut s = format!("SEC Analysis Summary\n  Data points: {}\n", self.chromatogram.len());
        if let Some(cal) = &self.calibration {
            s.push_str(&format!("  Calibration: log(MW) = {:.4} + {:.4} * Ve (R2={:.4})\n",
                cal.intercept, cal.slope, cal.r_squared));
        }
        if let Some(avg) = &self.averages {
            s.push_str(&format!("  Mn = {:.0}\n  Mw = {:.0}\n  PDI = {:.3}\n  Mz = {:.0}\n  Mp = {:.0}\n",
                avg.mn, avg.mw, avg.pdi, avg.mz, avg.mp));
        }
        s
    }
}

// --- Helpers ---

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

fn make_gaussian_chrom(ve_s: f64, ve_e: f64, n: usize, ctr: f64, sig: f64, amp: f64) -> SecChromatogram {
    let mut c = SecChromatogram::new();
    for i in 0..n {
        let ve = ve_s + (ve_e - ve_s) * i as f64 / (n - 1) as f64;
        c.add_point(ve, amp * (-(ve - ctr).powi(2) / (2.0 * sig * sig)).exp());
    }
    c
}

fn make_bimodal_chrom(vs: f64, ve: f64, n: usize, c1: f64, s1: f64, a1: f64, c2: f64, s2: f64, a2: f64) -> SecChromatogram {
    let mut c = SecChromatogram::new();
    for i in 0..n {
        let v = vs + (ve - vs) * i as f64 / (n - 1) as f64;
        let r = a1 * (-(v - c1).powi(2) / (2.0 * s1 * s1)).exp()
               + a2 * (-(v - c2).powi(2) / (2.0 * s2 * s2)).exp();
        c.add_point(v, r);
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ps_stds() -> Vec<(f64, f64)> {
        vec![(14.0, 2e6), (16.0, 5e5), (18.0, 1e5), (20.0, 2e4), (22.0, 5e3), (24.0, 1e3)]
    }

    // Chromatogram construction
    #[test] fn test_chrom_empty() { let c = SecChromatogram::new(); assert!(c.is_empty()); assert_eq!(c.len(), 0); }
    #[test] fn test_chrom_from_slices() {
        let c = SecChromatogram::from_slices(&[10.0, 12.0, 14.0], &[0.1, 0.5, 0.2]);
        assert_eq!(c.len(), 3); assert!(approx_eq(c.points[1].response, 0.5, 1e-10));
    }
    #[test] fn test_chrom_add_point() {
        let mut c = SecChromatogram::new(); c.add_point(15.0, 1.0); c.add_point(16.0, 2.0); assert_eq!(c.len(), 2);
    }
    #[test] fn test_chrom_vols_resps() {
        let c = SecChromatogram::from_slices(&[10.0, 12.0], &[0.1, 0.2]);
        assert_eq!(c.volumes().len(), 2); assert!(approx_eq(c.responses()[1], 0.2, 1e-10));
    }
    #[test] fn test_chrom_max_resp() {
        let c = SecChromatogram::from_slices(&[10.0, 12.0, 14.0], &[0.1, 0.9, 0.3]);
        assert!(approx_eq(c.max_response(), 0.9, 1e-10));
    }
    #[test] fn test_baseline_sub() {
        let mut c = SecChromatogram::from_slices(&[10.0, 15.0, 20.0], &[1.0, 5.0, 3.0]);
        c.subtract_baseline();
        assert!(approx_eq(c.points[0].response, 0.0, 1e-10));
        assert!(approx_eq(c.points[2].response, 0.0, 1e-10));
        assert!(c.points[1].response > 0.0);
    }

    // Linear calibration
    #[test] fn test_fit_cal() {
        let cal = fit_linear_calibration(&ps_stds());
        assert!(cal.slope < 0.0); assert!(cal.r_squared > 0.95);
    }
    #[test] fn test_cal_intercept_positive() { assert!(fit_linear_calibration(&ps_stds()).intercept > 0.0); }
    #[test] fn test_mw_ve_monotone() {
        let cal = fit_linear_calibration(&ps_stds());
        assert!(mw_from_ve(&cal, 16.0) > mw_from_ve(&cal, 20.0));
    }
    #[test] fn test_cal_roundtrip() {
        let cal = fit_linear_calibration(&ps_stds());
        for &(ve, mw_k) in &ps_stds() {
            let r = mw_from_ve(&cal, ve) / mw_k;
            assert!(r > 0.3 && r < 3.0);
        }
    }
    #[test] fn test_cal_insufficient() {
        assert!(approx_eq(fit_linear_calibration(&[(15.0, 1e5)]).r_squared, 0.0, 1e-10));
    }

    // MW averages
    #[test] fn test_mw_averages() {
        let avg = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()));
        assert!(avg.mn > 0.0); assert!(avg.mw > 0.0); assert!(avg.mw >= avg.mn);
    }
    #[test] fn test_pdi_narrow() {
        let avg = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 0.2, 1.0), &fit_linear_calibration(&ps_stds()));
        assert!(avg.pdi < 1.1); assert!(avg.pdi >= 1.0);
    }
    #[test] fn test_pdi_broad() {
        let avg = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 3.0, 1.0), &fit_linear_calibration(&ps_stds()));
        assert!(avg.pdi > 1.1);
    }
    #[test] fn test_mz_ge_mw() {
        let avg = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 2.0, 1.0), &fit_linear_calibration(&ps_stds()));
        assert!(avg.mz >= avg.mw);
    }
    #[test] fn test_mp_positive() {
        let avg = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()));
        assert!(avg.mp > 0.0);
    }
    #[test] fn test_empty_avg() {
        let avg = compute_mw_averages(&SecChromatogram::new(), &fit_linear_calibration(&ps_stds()));
        assert!(approx_eq(avg.mn, 0.0, 1e-10));
    }

    // Mark-Houwink & viscosity-average
    #[test] fn test_mh_iv() { assert!(MarkHouwinkParams::new(1.14e-4, 0.716).intrinsic_viscosity(1e5) > 0.0); }
    #[test] fn test_mh_hv() { assert!(MarkHouwinkParams::new(1.14e-4, 0.716).hydrodynamic_volume(1e5) > 0.0); }
    #[test] fn test_mv() {
        let cal = fit_linear_calibration(&ps_stds());
        let ch = make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0);
        let mh = MarkHouwinkParams::new(1.14e-4, 0.716);
        let mv = viscosity_average_mw(&ch, &cal, &mh);
        let avg = compute_mw_averages(&ch, &cal);
        assert!(mv > 0.0); assert!(mv >= avg.mn * 0.5); assert!(mv <= avg.mw * 2.0);
    }
    #[test] fn test_mv_theta() {
        let mv = viscosity_average_mw(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0),
            &fit_linear_calibration(&ps_stds()), &MarkHouwinkParams::new(8e-4, 0.5));
        assert!(mv > 0.0);
    }

    // Universal calibration
    #[test] fn test_ucal_fit() {
        let mh = MarkHouwinkParams::new(1.14e-4, 0.716);
        let stds: Vec<_> = ps_stds().iter().map(|&(v, m)| (v, m, mh.intrinsic_viscosity(m))).collect();
        let uc = fit_universal_calibration(&stds);
        assert!(uc.slope < 0.0); assert!(uc.r_squared > 0.95);
    }
    #[test] fn test_ucal_mw() {
        let mh = MarkHouwinkParams::new(1.14e-4, 0.716);
        let stds: Vec<_> = ps_stds().iter().map(|&(v, m)| (v, m, mh.intrinsic_viscosity(m))).collect();
        assert!(mw_from_universal(&fit_universal_calibration(&stds), &mh, 18.0) > 0.0);
    }
    #[test] fn test_ucal_monotone() {
        let mh = MarkHouwinkParams::new(1.14e-4, 0.716);
        let stds: Vec<_> = ps_stds().iter().map(|&(v, m)| (v, m, mh.intrinsic_viscosity(m))).collect();
        let uc = fit_universal_calibration(&stds);
        assert!(mw_from_universal(&uc, &mh, 16.0) > mw_from_universal(&uc, &mh, 20.0));
    }

    // Peak detection
    #[test] fn test_single_peak() {
        let peaks = find_peaks(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), 0.01);
        assert_eq!(peaks.len(), 1); assert!(approx_eq(peaks[0].elution_volume_ml, 18.0, 0.2));
    }
    #[test] fn test_bimodal_peaks() {
        assert_eq!(find_peaks(&make_bimodal_chrom(12.0, 28.0, 200, 16.0, 1.0, 1.0, 22.0, 1.0, 0.8), 0.01).len(), 2);
    }
    #[test] fn test_peaks_with_mw() {
        let peaks = find_peaks_with_mw(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 0.01);
        assert_eq!(peaks.len(), 1); assert!(peaks[0].mw > 0.0);
    }
    #[test] fn test_peak_mw() {
        assert!(find_peak_mw(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds())) > 0.0);
    }
    #[test] fn test_peak_threshold() {
        let ch = make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0);
        assert!(find_peaks(&ch, 0.01).len() >= find_peaks(&ch, 10.0).len());
    }

    // MW distribution
    #[test] fn test_dist_bins() {
        assert_eq!(mw_distribution(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 20).len(), 20);
    }
    #[test] fn test_dist_cum_monotone() {
        let d = mw_distribution(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 20);
        for i in 1..d.len() { assert!(d[i].cumulative >= d[i-1].cumulative - 1e-10); }
    }
    #[test] fn test_dist_cum_one() {
        let d = mw_distribution(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 20);
        assert!(approx_eq(d.last().unwrap().cumulative, 1.0, 0.01));
    }
    #[test] fn test_dist_diff_pos() {
        for b in &mw_distribution(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 20) {
            assert!(b.dw_dlogm >= 0.0);
        }
    }
    #[test] fn test_dist_empty() {
        assert!(mw_distribution(&SecChromatogram::new(), &fit_linear_calibration(&ps_stds()), 20).is_empty());
    }

    // SecProcessor orchestrator
    #[test] fn test_proc_new() {
        let p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        assert_eq!(p.chromatogram.len(), 200); assert!(p.calibration.is_none());
    }
    #[test] fn test_proc_slices() {
        let v: Vec<f64> = (0..100).map(|i| 10.0 + 0.2 * i as f64).collect();
        let r: Vec<f64> = v.iter().map(|&ve| (-(ve - 18.0).powi(2) / 4.5).exp()).collect();
        assert_eq!(SecProcessor::from_slices(&v, &r).chromatogram.len(), 100);
    }
    #[test] fn test_proc_calibrate() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        assert!(p.calibrate(&ps_stds()).slope < 0.0); assert!(p.calibration.is_some());
    }
    #[test] fn test_proc_averages() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds());
        let a = p.compute_averages().unwrap();
        assert!(a.mn > 0.0); assert!(a.mw >= a.mn); assert!(a.pdi >= 1.0);
    }
    #[test] fn test_proc_mv() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); p.set_mark_houwink(1.14e-4, 0.716);
        assert!(p.viscosity_average().unwrap() > 0.0);
    }
    #[test] fn test_proc_mv_none() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); assert!(p.viscosity_average().is_none());
    }
    #[test] fn test_proc_peaks() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); let pk = p.find_peaks(0.01);
        assert_eq!(pk.len(), 1); assert!(pk[0].mw > 0.0);
    }
    #[test] fn test_proc_dist() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); assert_eq!(p.mw_distribution(30).len(), 30);
    }
    #[test] fn test_proc_mw_vol() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); assert!(p.mw_at_volume(18.0).unwrap() > 0.0);
    }
    #[test] fn test_proc_mw_vol_none() {
        assert!(SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0)).mw_at_volume(18.0).is_none());
    }
    #[test] fn test_proc_ucal_mw() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.set_mark_houwink(1.14e-4, 0.716);
        let mh = p.mark_houwink.as_ref().unwrap();
        let stds: Vec<_> = ps_stds().iter().map(|&(v, m)| (v, m, mh.intrinsic_viscosity(m))).collect();
        p.calibrate_universal(&stds);
        assert!(p.mw_at_volume_universal(18.0).unwrap() > 0.0);
    }
    #[test] fn test_proc_baseline() {
        let mut ch = make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0);
        for pt in &mut ch.points { pt.response += 0.05 * pt.elution_volume_ml; }
        let mut p = SecProcessor::new(ch);
        p.subtract_baseline();
        assert!(p.chromatogram.points[0].response.abs() < 1e-10);
        assert!(p.chromatogram.points.last().unwrap().response.abs() < 1e-10);
    }
    #[test] fn test_proc_summary() {
        let mut p = SecProcessor::new(make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0));
        p.calibrate(&ps_stds()); p.compute_averages();
        let s = p.summary(); assert!(s.contains("Mn")); assert!(s.contains("Mw")); assert!(s.contains("PDI"));
    }
    #[test] fn test_pow10() {
        assert!(approx_eq(pow10(0.0), 1.0, 1e-10)); assert!(approx_eq(pow10(1.0), 10.0, 1e-10));
        assert!(approx_eq(pow10(3.0), 1000.0, 1e-6)); assert!(approx_eq(pow10(5.0), 100000.0, 0.1));
    }
    #[test] fn test_bimodal_pdi() {
        let cal = fit_linear_calibration(&ps_stds());
        let au = compute_mw_averages(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &cal);
        let ab = compute_mw_averages(&make_bimodal_chrom(12.0, 28.0, 200, 16.0, 1.0, 1.0, 22.0, 1.0, 1.0), &cal);
        assert!(ab.pdi > au.pdi);
    }
    #[test] fn test_dist_bin_mw_pos() {
        for b in &mw_distribution(&make_gaussian_chrom(12.0, 28.0, 200, 18.0, 1.5, 1.0), &fit_linear_calibration(&ps_stds()), 10) {
            assert!(b.mw > 0.0); assert!(b.log_mw.is_finite());
        }
    }
}
