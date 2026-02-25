// Near-Infrared Spectroscopy Processor
//
// NIR spectroscopy signal processing module covering spectral preprocessing,
// PCA, PLS regression, spectral band assignments, quantitative analysis,
// and quality metrics for chemometric applications.
//
// Key features:
// - SNV, MSC, Savitzky-Golay derivatives, baseline correction
// - PCA via power iteration eigendecomposition
// - PLS1 regression via NIPALS algorithm
// - Beer-Lambert law, multi-component analysis
// - Outlier detection (Hotelling's T², Q-residual, Mahalanobis distance)
// - VIP scores for variable importance

/// NIR wavelength range definitions (nm)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WavelengthRange {
    pub start_nm: f64,
    pub end_nm: f64,
    pub label: &'static str,
}

impl WavelengthRange {
    pub const NIR_FULL: WavelengthRange = WavelengthRange {
        start_nm: 780.0,
        end_nm: 2500.0,
        label: "Full NIR",
    };

    pub const OH_OVERTONE: WavelengthRange = WavelengthRange {
        start_nm: 1400.0,
        end_nm: 1450.0,
        label: "O-H overtone (moisture)",
    };

    pub const CH_STRETCHING: WavelengthRange = WavelengthRange {
        start_nm: 1680.0,
        end_nm: 1740.0,
        label: "C-H stretching (fats/oils)",
    };

    pub const NH_STRETCHING: WavelengthRange = WavelengthRange {
        start_nm: 1500.0,
        end_nm: 1550.0,
        label: "N-H stretching (protein)",
    };

    pub const OH_COMBINATION: WavelengthRange = WavelengthRange {
        start_nm: 1900.0,
        end_nm: 1950.0,
        label: "O-H combination (water)",
    };

    pub const CH_COMBINATION: WavelengthRange = WavelengthRange {
        start_nm: 2260.0,
        end_nm: 2380.0,
        label: "C-H combination (starch)",
    };

    /// Check if a wavelength falls within this range
    pub fn contains(&self, wavelength_nm: f64) -> bool {
        wavelength_nm >= self.start_nm && wavelength_nm <= self.end_nm
    }
}

/// Standard NIR band assignments
pub const NIR_BANDS: [WavelengthRange; 5] = [
    WavelengthRange::OH_OVERTONE,
    WavelengthRange::CH_STRETCHING,
    WavelengthRange::NH_STRETCHING,
    WavelengthRange::OH_COMBINATION,
    WavelengthRange::CH_COMBINATION,
];

/// Identify which NIR band a wavelength belongs to
pub fn identify_band(wavelength_nm: f64) -> Option<&'static WavelengthRange> {
    for band in &NIR_BANDS {
        if band.contains(wavelength_nm) {
            return Some(band);
        }
    }
    None
}

/// NIR spectrum data container
#[derive(Debug, Clone)]
pub struct NirSpectrum {
    /// Wavelengths in nm
    pub wavelengths: Vec<f64>,
    /// Absorbance (or reflectance) values
    pub values: Vec<f64>,
}

impl NirSpectrum {
    pub fn new(wavelengths: Vec<f64>, values: Vec<f64>) -> Self {
        assert_eq!(wavelengths.len(), values.len());
        Self { wavelengths, values }
    }

    /// Number of spectral points
    pub fn len(&self) -> usize {
        self.wavelengths.len()
    }

    /// Whether the spectrum is empty
    pub fn is_empty(&self) -> bool {
        self.wavelengths.is_empty()
    }

    /// Convert reflectance to absorbance: A = -log10(R)
    pub fn reflectance_to_absorbance(&self) -> NirSpectrum {
        let abs_values: Vec<f64> = self
            .values
            .iter()
            .map(|&r| {
                let r_clamped = r.max(1e-10);
                -(r_clamped.ln() / std::f64::consts::LN_10)
            })
            .collect();
        NirSpectrum::new(self.wavelengths.clone(), abs_values)
    }

    /// Convert absorbance to reflectance: R = 10^(-A)
    pub fn absorbance_to_reflectance(&self) -> NirSpectrum {
        let refl_values: Vec<f64> = self
            .values
            .iter()
            .map(|&a| {
                let exponent = -a * std::f64::consts::LN_10;
                exponent.exp()
            })
            .collect();
        NirSpectrum::new(self.wavelengths.clone(), refl_values)
    }

    /// Extract values within a wavelength range
    pub fn extract_range(&self, range: &WavelengthRange) -> NirSpectrum {
        let mut wl = Vec::new();
        let mut vals = Vec::new();
        for (i, &w) in self.wavelengths.iter().enumerate() {
            if range.contains(w) {
                wl.push(w);
                vals.push(self.values[i]);
            }
        }
        NirSpectrum::new(wl, vals)
    }
}

// ============================================================================
// Spectral Preprocessing
// ============================================================================

/// Standard Normal Variate: x_snv = (x - mean(x)) / std(x)
pub fn snv(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len() as f64;
    if n < 2.0 {
        return spectrum.to_vec();
    }
    let mean = spectrum.iter().sum::<f64>() / n;
    let variance = spectrum.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
    let std_dev = variance.sqrt();
    if std_dev < 1e-15 {
        return vec![0.0; spectrum.len()];
    }
    spectrum.iter().map(|&x| (x - mean) / std_dev).collect()
}

/// Multiplicative Scatter Correction against a reference spectrum
pub fn msc(spectrum: &[f64], reference: &[f64]) -> Vec<f64> {
    assert_eq!(spectrum.len(), reference.len());
    let n = spectrum.len() as f64;
    if n < 2.0 {
        return spectrum.to_vec();
    }

    // Fit linear regression: spectrum = a + b * reference
    let mean_x = reference.iter().sum::<f64>() / n;
    let mean_y = spectrum.iter().sum::<f64>() / n;

    let mut ss_xx = 0.0;
    let mut ss_xy = 0.0;
    for i in 0..spectrum.len() {
        let dx = reference[i] - mean_x;
        let dy = spectrum[i] - mean_y;
        ss_xx += dx * dx;
        ss_xy += dx * dy;
    }

    let b = if ss_xx.abs() > 1e-15 {
        ss_xy / ss_xx
    } else {
        1.0
    };
    let a = mean_y - b * mean_x;

    // Correct: (spectrum - a) / b
    if b.abs() < 1e-15 {
        return spectrum.to_vec();
    }
    spectrum.iter().map(|&y| (y - a) / b).collect()
}

/// MSC for a set of spectra (rows), computing mean reference
pub fn msc_batch(spectra: &[Vec<f64>]) -> Vec<Vec<f64>> {
    if spectra.is_empty() {
        return vec![];
    }
    let n_vars = spectra[0].len();
    let n_samples = spectra.len();

    // Compute mean spectrum as reference
    let mut reference = vec![0.0; n_vars];
    for s in spectra {
        for (j, &v) in s.iter().enumerate() {
            reference[j] += v;
        }
    }
    for r in &mut reference {
        *r /= n_samples as f64;
    }

    spectra.iter().map(|s| msc(s, &reference)).collect()
}

/// Moving average smoothing
pub fn moving_average(spectrum: &[f64], window: usize) -> Vec<f64> {
    let n = spectrum.len();
    if n == 0 || window == 0 {
        return spectrum.to_vec();
    }
    let half = window / 2;
    let mut result = vec![0.0; n];
    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let count = (end - start) as f64;
        let sum: f64 = spectrum[start..end].iter().sum();
        result[i] = sum / count;
    }
    result
}

/// Savitzky-Golay coefficients for polynomial fitting
/// Returns convolution coefficients for smoothing or derivative
fn sg_coefficients(window: usize, poly_order: usize, deriv_order: usize) -> Vec<f64> {
    assert!(window >= poly_order + 1);
    assert!(window % 2 == 1, "Window must be odd");
    let half = (window / 2) as i64;

    // Build Vandermonde-like matrix J
    let m = window;
    let p = poly_order + 1;

    // J[i][k] = i_val^k where i_val = i - half
    let mut j_mat = vec![vec![0.0; p]; m];
    for i in 0..m {
        let x = (i as i64 - half) as f64;
        let mut xk = 1.0;
        for k in 0..p {
            j_mat[i][k] = xk;
            xk *= x;
        }
    }

    // Compute J^T * J
    let mut jtj = vec![vec![0.0; p]; p];
    for r in 0..p {
        for c in 0..p {
            for i in 0..m {
                jtj[r][c] += j_mat[i][r] * j_mat[i][c];
            }
        }
    }

    // Invert J^T * J using Gauss-Jordan
    let mut aug = vec![vec![0.0; 2 * p]; p];
    for r in 0..p {
        for c in 0..p {
            aug[r][c] = jtj[r][c];
        }
        aug[r][p + r] = 1.0;
    }

    for col in 0..p {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..p {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            continue;
        }
        for c in 0..(2 * p) {
            aug[col][c] /= pivot;
        }
        for row in 0..p {
            if row != col {
                let factor = aug[row][col];
                for c in 0..(2 * p) {
                    aug[row][c] -= factor * aug[col][c];
                }
            }
        }
    }

    // Extract inverse
    let mut inv = vec![vec![0.0; p]; p];
    for r in 0..p {
        for c in 0..p {
            inv[r][c] = aug[r][p + c];
        }
    }

    // C = (J^T J)^-1 * J^T, take row deriv_order
    // coeffs[i] = sum_k inv[deriv_order][k] * J[i][k]
    // Then multiply by deriv_order! for derivative
    let factorial = |n: usize| -> f64 {
        let mut f = 1.0;
        for i in 2..=n {
            f *= i as f64;
        }
        f
    };

    let scale = factorial(deriv_order);
    let mut coeffs = vec![0.0; m];
    for i in 0..m {
        for k in 0..p {
            coeffs[i] += inv[deriv_order][k] * j_mat[i][k];
        }
        coeffs[i] *= scale;
    }
    coeffs
}

/// Savitzky-Golay smoothing
pub fn sg_smooth(spectrum: &[f64], window: usize, poly_order: usize) -> Vec<f64> {
    let w = if window % 2 == 0 { window + 1 } else { window };
    let coeffs = sg_coefficients(w, poly_order, 0);
    convolve_same(spectrum, &coeffs)
}

/// Savitzky-Golay derivative
pub fn sg_derivative(
    spectrum: &[f64],
    window: usize,
    poly_order: usize,
    deriv_order: usize,
) -> Vec<f64> {
    let w = if window % 2 == 0 { window + 1 } else { window };
    let coeffs = sg_coefficients(w, poly_order, deriv_order);
    convolve_same(spectrum, &coeffs)
}

/// Same-length convolution with zero-padding
fn convolve_same(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let k = kernel.len();
    let half = k / 2;
    let mut result = vec![0.0; n];

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..k {
            let idx = i as i64 + j as i64 - half as i64;
            if idx >= 0 && (idx as usize) < n {
                sum += signal[idx as usize] * kernel[j];
            }
        }
        result[i] = sum;
    }
    result
}

/// Norris-Williams derivative (gap-segment derivative)
pub fn norris_williams_derivative(spectrum: &[f64], gap: usize, segment: usize) -> Vec<f64> {
    let n = spectrum.len();
    let mut result = vec![0.0; n];
    let half_seg = segment / 2;

    for i in 0..n {
        // Average over segment before
        let start_left = if i >= gap + half_seg {
            i - gap - half_seg
        } else {
            0
        };
        let end_left = if i >= gap { (i - gap + half_seg + 1).min(n) } else { 0 };

        // Average over segment after
        let start_right = if i + gap >= half_seg {
            (i + gap - half_seg).min(n)
        } else {
            0
        };
        let end_right = (i + gap + half_seg + 1).min(n);

        if end_left > start_left && end_right > start_right {
            let avg_left: f64 =
                spectrum[start_left..end_left].iter().sum::<f64>() / (end_left - start_left) as f64;
            let avg_right: f64 = spectrum[start_right..end_right].iter().sum::<f64>()
                / (end_right - start_right) as f64;
            result[i] = avg_right - avg_left;
        }
    }
    result
}

/// Linear baseline correction (subtract line from first to last point)
pub fn baseline_linear(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len();
    if n < 2 {
        return spectrum.to_vec();
    }
    let y0 = spectrum[0];
    let y1 = spectrum[n - 1];
    let slope = (y1 - y0) / (n - 1) as f64;
    (0..n).map(|i| spectrum[i] - (y0 + slope * i as f64)).collect()
}

/// Polynomial baseline correction
pub fn baseline_polynomial(spectrum: &[f64], order: usize) -> Vec<f64> {
    let n = spectrum.len();
    if n <= order {
        return spectrum.to_vec();
    }

    // Fit polynomial using least squares
    let x_vals: Vec<f64> = (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
    let coeffs = polyfit(&x_vals, spectrum, order);

    // Evaluate baseline and subtract
    (0..n)
        .map(|i| {
            let x = x_vals[i];
            let baseline = polyeval(&coeffs, x);
            spectrum[i] - baseline
        })
        .collect()
}

/// Fit polynomial of given order to data (least squares)
fn polyfit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let p = order + 1;

    // Build normal equations X^T X * c = X^T y
    let mut xtx = vec![vec![0.0; p]; p];
    let mut xty = vec![0.0; p];

    for i in 0..n {
        let mut xi = 1.0;
        for j in 0..p {
            let mut xij = xi;
            for k in 0..p {
                xtx[j][k] += xij;
                xij *= x[i];
            }
            xty[j] += xi * y[i];
            xi *= x[i];
        }
    }

    // Solve via Gauss elimination
    solve_linear(&xtx, &xty)
}

/// Evaluate polynomial given coefficients [c0, c1, c2, ...] -> c0 + c1*x + c2*x^2 + ...
fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xk = 1.0;
    for &c in coeffs {
        result += c * xk;
        xk *= x;
    }
    result
}

/// Solve Ax = b via Gaussian elimination with partial pivoting
fn solve_linear(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    let mut aug = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n] = b[i];
    }

    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            continue;
        }
        for c in col..=n {
            aug[col][c] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row][col];
                for c in col..=n {
                    aug[row][c] -= factor * aug[col][c];
                }
            }
        }
    }

    (0..n).map(|i| aug[i][n]).collect()
}

/// Mean centering
pub fn mean_center(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len() as f64;
    if n < 1.0 {
        return spectrum.to_vec();
    }
    let mean = spectrum.iter().sum::<f64>() / n;
    spectrum.iter().map(|&x| x - mean).collect()
}

/// Autoscaling (mean center + unit variance)
pub fn autoscale(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len() as f64;
    if n < 2.0 {
        return spectrum.to_vec();
    }
    let mean = spectrum.iter().sum::<f64>() / n;
    let variance = spectrum.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
    let std_dev = variance.sqrt();
    if std_dev < 1e-15 {
        return vec![0.0; spectrum.len()];
    }
    spectrum.iter().map(|&x| (x - mean) / std_dev).collect()
}

/// Spectral preprocessor pipeline
#[derive(Debug, Clone)]
pub enum PreprocessStep {
    Snv,
    Msc(Vec<f64>),
    SgSmooth { window: usize, poly_order: usize },
    SgDerivative { window: usize, poly_order: usize, deriv_order: usize },
    MovingAverage(usize),
    NorrisWilliams { gap: usize, segment: usize },
    BaselineLinear,
    BaselinePolynomial(usize),
    MeanCenter,
    Autoscale,
}

#[derive(Debug, Clone)]
pub struct SpectralPreprocessor {
    pub steps: Vec<PreprocessStep>,
}

impl SpectralPreprocessor {
    pub fn new() -> Self {
        Self { steps: Vec::new() }
    }

    pub fn add_step(&mut self, step: PreprocessStep) -> &mut Self {
        self.steps.push(step);
        self
    }

    pub fn process(&self, spectrum: &[f64]) -> Vec<f64> {
        let mut data = spectrum.to_vec();
        for step in &self.steps {
            data = match step {
                PreprocessStep::Snv => snv(&data),
                PreprocessStep::Msc(reference) => msc(&data, reference),
                PreprocessStep::SgSmooth { window, poly_order } => sg_smooth(&data, *window, *poly_order),
                PreprocessStep::SgDerivative { window, poly_order, deriv_order } => {
                    sg_derivative(&data, *window, *poly_order, *deriv_order)
                }
                PreprocessStep::MovingAverage(w) => moving_average(&data, *w),
                PreprocessStep::NorrisWilliams { gap, segment } => {
                    norris_williams_derivative(&data, *gap, *segment)
                }
                PreprocessStep::BaselineLinear => baseline_linear(&data),
                PreprocessStep::BaselinePolynomial(order) => baseline_polynomial(&data, *order),
                PreprocessStep::MeanCenter => mean_center(&data),
                PreprocessStep::Autoscale => autoscale(&data),
            };
        }
        data
    }
}

// ============================================================================
// Matrix utilities
// ============================================================================

/// Compute mean of each column in a matrix (rows = samples, cols = variables)
fn column_means(data: &[Vec<f64>]) -> Vec<f64> {
    let n = data.len() as f64;
    let p = data[0].len();
    let mut means = vec![0.0; p];
    for row in data {
        for (j, &v) in row.iter().enumerate() {
            means[j] += v;
        }
    }
    for m in &mut means {
        *m /= n;
    }
    means
}

/// Mean-center a data matrix in place, return means
fn mean_center_matrix(data: &mut [Vec<f64>]) -> Vec<f64> {
    let means = column_means(data);
    for row in data.iter_mut() {
        for (j, v) in row.iter_mut().enumerate() {
            *v -= means[j];
        }
    }
    means
}

/// Compute covariance matrix from mean-centered data
fn covariance_matrix(data: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = data.len() as f64;
    let p = data[0].len();
    let mut cov = vec![vec![0.0; p]; p];
    for row in data {
        for i in 0..p {
            for j in i..p {
                cov[i][j] += row[i] * row[j];
            }
        }
    }
    for i in 0..p {
        for j in i..p {
            cov[i][j] /= n - 1.0;
            cov[j][i] = cov[i][j];
        }
    }
    cov
}

/// Power iteration to find dominant eigenvector
fn power_iteration(matrix: &[Vec<f64>], max_iter: usize, tol: f64) -> (f64, Vec<f64>) {
    let n = matrix.len();
    let mut v = vec![1.0 / (n as f64).sqrt(); n];
    let mut eigenvalue = 0.0;

    for _ in 0..max_iter {
        // w = A * v
        let mut w = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                w[i] += matrix[i][j] * v[j];
            }
        }

        // Eigenvalue estimate
        let new_eigenvalue: f64 = w.iter().zip(v.iter()).map(|(&wi, &vi)| wi * vi).sum();

        // Normalize
        let norm: f64 = w.iter().map(|&x| x * x).sum::<f64>().sqrt();
        if norm < 1e-15 {
            break;
        }
        for x in &mut w {
            *x /= norm;
        }

        if (new_eigenvalue - eigenvalue).abs() < tol {
            return (new_eigenvalue, w);
        }
        eigenvalue = new_eigenvalue;
        v = w;
    }
    (eigenvalue, v)
}

/// Extract multiple eigenvalues/eigenvectors via deflation
fn eigen_decomposition(
    matrix: &[Vec<f64>],
    n_components: usize,
    max_iter: usize,
    tol: f64,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = matrix.len();
    let nc = n_components.min(n);
    let mut mat = matrix.to_vec();
    let mut eigenvalues = Vec::with_capacity(nc);
    let mut eigenvectors = Vec::with_capacity(nc);

    for _ in 0..nc {
        let (eval, evec) = power_iteration(&mat, max_iter, tol);
        eigenvalues.push(eval);
        eigenvectors.push(evec.clone());

        // Deflate: A = A - lambda * v * v^T
        for i in 0..n {
            for j in 0..n {
                mat[i][j] -= eval * evec[i] * evec[j];
            }
        }
    }
    (eigenvalues, eigenvectors)
}

// ============================================================================
// PCA Model
// ============================================================================

/// PCA model result
#[derive(Debug, Clone)]
pub struct PcaModel {
    /// Eigenvalues (variance explained per component)
    pub eigenvalues: Vec<f64>,
    /// Eigenvectors (loadings), each is a column vector
    pub loadings: Vec<Vec<f64>>,
    /// Scores T = X_centered * P
    pub scores: Vec<Vec<f64>>,
    /// Column means from training data
    pub means: Vec<f64>,
    /// Total variance
    pub total_variance: f64,
    /// Number of components
    pub n_components: usize,
}

impl PcaModel {
    /// Fit PCA on data matrix (rows = samples, cols = variables)
    pub fn fit(data: &[Vec<f64>], n_components: usize) -> Self {
        let mut centered = data.to_vec();
        let means = mean_center_matrix(&mut centered);

        let cov = covariance_matrix(&centered);
        let total_variance: f64 = (0..cov.len()).map(|i| cov[i][i]).sum();

        let (eigenvalues, eigenvectors) =
            eigen_decomposition(&cov, n_components, 1000, 1e-10);

        // Compute scores
        let n_samples = data.len();
        let nc = eigenvalues.len();
        let mut scores = vec![vec![0.0; nc]; n_samples];
        for i in 0..n_samples {
            for k in 0..nc {
                for j in 0..centered[0].len() {
                    scores[i][k] += centered[i][j] * eigenvectors[k][j];
                }
            }
        }

        PcaModel {
            eigenvalues,
            loadings: eigenvectors,
            scores,
            means,
            total_variance,
            n_components: nc,
        }
    }

    /// Explained variance ratio for each component
    pub fn explained_variance_ratio(&self) -> Vec<f64> {
        if self.total_variance < 1e-15 {
            return vec![0.0; self.eigenvalues.len()];
        }
        self.eigenvalues
            .iter()
            .map(|&ev| ev / self.total_variance)
            .collect()
    }

    /// Cumulative explained variance
    pub fn cumulative_variance(&self) -> Vec<f64> {
        let ratios = self.explained_variance_ratio();
        let mut cum = Vec::with_capacity(ratios.len());
        let mut sum = 0.0;
        for r in &ratios {
            sum += r;
            cum.push(sum);
        }
        cum
    }

    /// Hotelling's T-squared statistic for each sample
    pub fn hotellings_t2(&self) -> Vec<f64> {
        let n = self.scores.len();
        let nc = self.n_components;
        let mut t2 = vec![0.0; n];
        for i in 0..n {
            for k in 0..nc {
                if self.eigenvalues[k] > 1e-15 {
                    t2[i] += self.scores[i][k] * self.scores[i][k] / self.eigenvalues[k];
                }
            }
        }
        t2
    }

    /// Q-residual (Squared Prediction Error / SPE) for each sample
    pub fn q_residual(&self, data: &[Vec<f64>]) -> Vec<f64> {
        let n = data.len();
        let p = data[0].len();
        let nc = self.n_components;
        let mut q = vec![0.0; n];

        for i in 0..n {
            // Reconstruct: x_hat = scores * loadings + mean
            let mut residual = vec![0.0; p];
            for j in 0..p {
                let mut reconstructed = self.means[j];
                for k in 0..nc {
                    reconstructed += self.scores[i][k] * self.loadings[k][j];
                }
                residual[j] = data[i][j] - reconstructed;
            }
            q[i] = residual.iter().map(|&r| r * r).sum();
        }
        q
    }

    /// Project new sample into PCA space
    pub fn transform(&self, sample: &[f64]) -> Vec<f64> {
        let nc = self.n_components;
        let p = sample.len();
        let mut scores = vec![0.0; nc];
        for k in 0..nc {
            for j in 0..p {
                scores[k] += (sample[j] - self.means[j]) * self.loadings[k][j];
            }
        }
        scores
    }
}

// ============================================================================
// PLS Regression (NIPALS)
// ============================================================================

/// PLS regression model
#[derive(Debug, Clone)]
pub struct PlsModel {
    /// X weights
    pub weights: Vec<Vec<f64>>,
    /// X loadings
    pub x_loadings: Vec<Vec<f64>>,
    /// Y loadings
    pub y_loadings: Vec<f64>,
    /// X scores
    pub x_scores: Vec<Vec<f64>>,
    /// Y scores
    pub y_scores: Vec<Vec<f64>>,
    /// Regression coefficients (one per wavelength)
    pub coefficients: Vec<f64>,
    /// X column means
    pub x_means: Vec<f64>,
    /// Y mean
    pub y_mean: f64,
    /// Number of latent variables
    pub n_components: usize,
    /// VIP scores
    pub vip_scores: Vec<f64>,
}

impl PlsModel {
    /// Fit PLS1 model using NIPALS
    /// x: matrix of spectra (rows = samples)
    /// y: vector of reference values
    pub fn fit(x: &[Vec<f64>], y: &[f64], n_components: usize) -> Self {
        let n = x.len();
        let p = x[0].len();
        let nc = n_components.min(n).min(p);

        // Mean center X and Y
        let mut x_centered: Vec<Vec<f64>> = x.to_vec();
        let x_means = mean_center_matrix(&mut x_centered);
        let y_mean = y.iter().sum::<f64>() / n as f64;
        let mut y_centered: Vec<f64> = y.iter().map(|&yi| yi - y_mean).collect();

        let mut weights_all = Vec::with_capacity(nc);
        let mut x_loadings_all = Vec::with_capacity(nc);
        let mut y_loadings_all = Vec::with_capacity(nc);
        let mut x_scores_all = Vec::with_capacity(nc);
        let mut y_scores_all = Vec::with_capacity(nc);

        for _ in 0..nc {
            // w = X^T * u / (u^T * u), start with u = y
            let mut u = y_centered.clone();

            let mut w = vec![0.0; p];
            let mut t = vec![0.0; n];
            let mut p_loading = vec![0.0; p];
            let mut q: f64;

            for _iter in 0..200 {
                // w = X^T * u / (u^T * u)
                let utu: f64 = u.iter().map(|&ui| ui * ui).sum();
                if utu < 1e-30 {
                    break;
                }
                for j in 0..p {
                    w[j] = 0.0;
                    for i in 0..n {
                        w[j] += x_centered[i][j] * u[i];
                    }
                    w[j] /= utu;
                }
                // Normalize w
                let w_norm = w.iter().map(|&wi| wi * wi).sum::<f64>().sqrt();
                if w_norm < 1e-15 {
                    break;
                }
                for wi in &mut w {
                    *wi /= w_norm;
                }

                // t = X * w
                for i in 0..n {
                    t[i] = 0.0;
                    for j in 0..p {
                        t[i] += x_centered[i][j] * w[j];
                    }
                }

                // q = y^T * t / (t^T * t)
                let ttt: f64 = t.iter().map(|&ti| ti * ti).sum();
                if ttt < 1e-30 {
                    break;
                }
                q = y_centered.iter().zip(t.iter()).map(|(&yi, &ti)| yi * ti).sum::<f64>() / ttt;

                // u_new = y * q / (q * q)
                let u_new: Vec<f64> = y_centered.iter().map(|&yi| yi * q / (q * q + 1e-30)).collect();

                let diff: f64 = u_new
                    .iter()
                    .zip(u.iter())
                    .map(|(&a, &b)| (a - b) * (a - b))
                    .sum::<f64>()
                    .sqrt();
                u = u_new;
                if diff < 1e-10 {
                    break;
                }
            }

            // t = X * w
            let ttt: f64 = t.iter().map(|&ti| ti * ti).sum();
            if ttt < 1e-30 {
                break;
            }

            // p = X^T * t / (t^T * t)
            for j in 0..p {
                p_loading[j] = 0.0;
                for i in 0..n {
                    p_loading[j] += x_centered[i][j] * t[i];
                }
                p_loading[j] /= ttt;
            }

            // q = y^T * t / (t^T * t)
            q = y_centered.iter().zip(t.iter()).map(|(&yi, &ti)| yi * ti).sum::<f64>() / ttt;

            // Deflate X and Y
            for i in 0..n {
                for j in 0..p {
                    x_centered[i][j] -= t[i] * p_loading[j];
                }
                y_centered[i] -= t[i] * q;
            }

            weights_all.push(w.clone());
            x_loadings_all.push(p_loading.clone());
            y_loadings_all.push(q);
            x_scores_all.push(t.clone());
            y_scores_all.push(y_centered.clone());
        }

        // Compute regression coefficients: B = W * (P^T * W)^{-1} * Q
        let coefficients = compute_pls_coefficients(
            &weights_all,
            &x_loadings_all,
            &y_loadings_all,
            p,
        );

        // Compute VIP scores
        let vip = compute_vip(&weights_all, &x_scores_all, &y_loadings_all, p);

        PlsModel {
            weights: weights_all,
            x_loadings: x_loadings_all,
            y_loadings: y_loadings_all,
            x_scores: x_scores_all,
            y_scores: y_scores_all,
            coefficients,
            x_means,
            y_mean,
            n_components: nc,
            vip_scores: vip,
        }
    }

    /// Predict Y for a single sample
    pub fn predict(&self, sample: &[f64]) -> f64 {
        let mut y_pred = self.y_mean;
        for (j, &xj) in sample.iter().enumerate() {
            if j < self.coefficients.len() {
                y_pred += (xj - self.x_means[j]) * self.coefficients[j];
            }
        }
        y_pred
    }

    /// Predict Y for multiple samples
    pub fn predict_batch(&self, samples: &[Vec<f64>]) -> Vec<f64> {
        samples.iter().map(|s| self.predict(s)).collect()
    }
}

/// Compute PLS regression coefficients
fn compute_pls_coefficients(
    weights: &[Vec<f64>],
    loadings: &[Vec<f64>],
    y_loadings: &[f64],
    p: usize,
) -> Vec<f64> {
    let nc = weights.len();
    if nc == 0 {
        return vec![0.0; p];
    }

    // P^T * W (nc x nc matrix)
    let mut ptw = vec![vec![0.0; nc]; nc];
    for i in 0..nc {
        for j in 0..nc {
            for k in 0..p {
                ptw[i][j] += loadings[i][k] * weights[j][k];
            }
        }
    }

    // Invert P^T * W
    let ptw_inv = invert_matrix(&ptw);
    let ptw_inv = if ptw_inv.is_empty() {
        let mut fallback = vec![vec![0.0; nc]; nc];
        for i in 0..nc {
            fallback[i][i] = 1.0;
        }
        fallback
    } else {
        ptw_inv
    };

    // B = W * (P^T W)^-1 * Q
    let mut coeffs = vec![0.0; p];
    for j in 0..p {
        for a in 0..nc {
            let mut sum = 0.0;
            for b in 0..nc {
                sum += ptw_inv[a][b] * y_loadings[b];
            }
            coeffs[j] += weights[a][j] * sum;
        }
    }
    coeffs
}

/// Invert a matrix using Gauss-Jordan
fn invert_matrix(matrix: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = matrix.len();
    let mut aug = vec![vec![0.0; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = matrix[i][j];
        }
        aug[i][n + i] = 1.0;
    }

    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return vec![];
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for c in 0..(2 * n) {
            aug[col][c] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row][col];
                for c in 0..(2 * n) {
                    aug[row][c] -= factor * aug[col][c];
                }
            }
        }
    }

    let mut inv = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    inv
}

/// Compute VIP (Variable Importance in Projection) scores
fn compute_vip(
    weights: &[Vec<f64>],
    x_scores: &[Vec<f64>],
    y_loadings: &[f64],
    p: usize,
) -> Vec<f64> {
    let nc = weights.len();
    if nc == 0 {
        return vec![0.0; p];
    }

    // SS = sum of q_a^2 * ||t_a||^2 for explained Y variance per component
    let mut ss = Vec::with_capacity(nc);
    let mut total_ss = 0.0;
    for a in 0..nc {
        let t_norm_sq: f64 = x_scores[a].iter().map(|&t| t * t).sum();
        let s = y_loadings[a] * y_loadings[a] * t_norm_sq;
        ss.push(s);
        total_ss += s;
    }

    if total_ss < 1e-30 {
        return vec![0.0; p];
    }

    let mut vip = vec![0.0; p];
    for j in 0..p {
        let mut sum = 0.0;
        for a in 0..nc {
            sum += ss[a] * weights[a][j] * weights[a][j];
        }
        vip[j] = (p as f64 * sum / total_ss).sqrt();
    }
    vip
}

// ============================================================================
// Prediction Result and Cross-Validation
// ============================================================================

/// Prediction result with confidence
#[derive(Debug, Clone)]
pub struct PredictionResult {
    pub predicted: f64,
    pub confidence_lower: f64,
    pub confidence_upper: f64,
}

/// Leave-one-out cross-validation for PLS
pub fn pls_cross_validate(
    x: &[Vec<f64>],
    y: &[f64],
    max_components: usize,
) -> Vec<f64> {
    let n = x.len();
    let max_nc = max_components.min(n - 1).min(x[0].len());
    let mut rmsecv = Vec::with_capacity(max_nc);

    for nc in 1..=max_nc {
        let mut sse = 0.0;
        for i in 0..n {
            // Leave out sample i
            let mut x_train: Vec<Vec<f64>> = Vec::with_capacity(n - 1);
            let mut y_train: Vec<f64> = Vec::with_capacity(n - 1);
            for j in 0..n {
                if j != i {
                    x_train.push(x[j].clone());
                    y_train.push(y[j]);
                }
            }
            let model = PlsModel::fit(&x_train, &y_train, nc);
            let y_pred = model.predict(&x[i]);
            sse += (y[i] - y_pred) * (y[i] - y_pred);
        }
        rmsecv.push((sse / n as f64).sqrt());
    }
    rmsecv
}

/// Find optimal number of components (minimum RMSECV)
pub fn optimal_components(rmsecv: &[f64]) -> usize {
    if rmsecv.is_empty() {
        return 1;
    }
    let mut best_idx = 0;
    let mut best_val = rmsecv[0];
    for (i, &v) in rmsecv.iter().enumerate() {
        if v < best_val {
            best_val = v;
            best_idx = i;
        }
    }
    best_idx + 1 // 1-based component count
}

/// RMSEP (Root Mean Square Error of Prediction)
pub fn rmsep(actual: &[f64], predicted: &[f64]) -> f64 {
    assert_eq!(actual.len(), predicted.len());
    let n = actual.len() as f64;
    let sse: f64 = actual
        .iter()
        .zip(predicted.iter())
        .map(|(&a, &p)| (a - p) * (a - p))
        .sum();
    (sse / n).sqrt()
}

/// R-squared (coefficient of determination)
pub fn r_squared(actual: &[f64], predicted: &[f64]) -> f64 {
    let n = actual.len() as f64;
    let mean = actual.iter().sum::<f64>() / n;
    let ss_tot: f64 = actual.iter().map(|&a| (a - mean) * (a - mean)).sum();
    let ss_res: f64 = actual
        .iter()
        .zip(predicted.iter())
        .map(|(&a, &p)| (a - p) * (a - p))
        .sum();
    if ss_tot < 1e-15 {
        return 0.0;
    }
    1.0 - ss_res / ss_tot
}

/// Bias (mean error)
pub fn bias(actual: &[f64], predicted: &[f64]) -> f64 {
    let n = actual.len() as f64;
    actual
        .iter()
        .zip(predicted.iter())
        .map(|(&a, &p)| p - a)
        .sum::<f64>()
        / n
}

/// Ratio of Prediction Deviation: RPD = SD(reference) / RMSEP
pub fn rpd(actual: &[f64], predicted: &[f64]) -> f64 {
    let n = actual.len() as f64;
    let mean = actual.iter().sum::<f64>() / n;
    let sd = (actual.iter().map(|&a| (a - mean) * (a - mean)).sum::<f64>() / (n - 1.0)).sqrt();
    let rmse = rmsep(actual, predicted);
    if rmse < 1e-15 {
        return f64::INFINITY;
    }
    sd / rmse
}

// ============================================================================
// Quantitative Analysis
// ============================================================================

/// Beer-Lambert law: A = epsilon * c * l
pub fn beer_lambert(molar_absorptivity: f64, concentration: f64, path_length: f64) -> f64 {
    molar_absorptivity * concentration * path_length
}

/// Inverse Beer-Lambert: c = A / (epsilon * l)
pub fn beer_lambert_concentration(
    absorbance: f64,
    molar_absorptivity: f64,
    path_length: f64,
) -> f64 {
    if (molar_absorptivity * path_length).abs() < 1e-15 {
        return 0.0;
    }
    absorbance / (molar_absorptivity * path_length)
}

/// Multi-component analysis: solve A = E * c * l for concentrations
/// absorbances: measured absorbance at each wavelength
/// absorptivities: matrix [n_wavelengths x n_components] of molar absorptivities
/// path_length: cell path length in cm
pub fn multicomponent_analysis(
    absorbances: &[f64],
    absorptivities: &[Vec<f64>],
    path_length: f64,
) -> Vec<f64> {
    let n_wl = absorbances.len();
    let n_comp = absorptivities[0].len();

    // Build normal equations: (E^T E) * c = E^T * (A/l)
    let mut ete = vec![vec![0.0; n_comp]; n_comp];
    let mut eta = vec![0.0; n_comp];

    for i in 0..n_wl {
        let a_over_l = absorbances[i] / path_length;
        for j in 0..n_comp {
            eta[j] += absorptivities[i][j] * a_over_l;
            for k in 0..n_comp {
                ete[j][k] += absorptivities[i][j] * absorptivities[i][k];
            }
        }
    }

    solve_linear(&ete, &eta)
}

/// Standard Error of Laboratory (SEL)
pub fn sel(reference_values: &[f64], replicates: &[Vec<f64>]) -> f64 {
    let n = reference_values.len();
    if n == 0 {
        return 0.0;
    }
    let mut sse = 0.0;
    let mut count = 0;
    for i in 0..n {
        let mean = replicates[i].iter().sum::<f64>() / replicates[i].len() as f64;
        for &r in &replicates[i] {
            sse += (r - mean) * (r - mean);
            count += 1;
        }
    }
    if count <= n {
        return 0.0;
    }
    (sse / (count - n) as f64).sqrt()
}

// ============================================================================
// Quality Metrics and Outlier Detection
// ============================================================================

/// Mahalanobis distance from mean using PCA scores
pub fn mahalanobis_distance(scores: &[f64], eigenvalues: &[f64]) -> f64 {
    let mut d2 = 0.0;
    for (i, &s) in scores.iter().enumerate() {
        if i < eigenvalues.len() && eigenvalues[i] > 1e-15 {
            d2 += s * s / eigenvalues[i];
        }
    }
    d2.sqrt()
}

/// Leverage (hat matrix diagonal) for a sample in PCA space
/// h_i = t_i^T * (T^T T)^{-1} * t_i + 1/n
pub fn leverage(scores: &[f64], eigenvalues: &[f64], n_samples: usize) -> f64 {
    let mut h = 1.0 / n_samples as f64;
    for (i, &s) in scores.iter().enumerate() {
        if i < eigenvalues.len() && eigenvalues[i] > 1e-15 {
            let n = n_samples as f64;
            h += s * s / (eigenvalues[i] * (n - 1.0));
        }
    }
    h
}

/// Studentized residual
pub fn studentized_residual(
    actual: f64,
    predicted: f64,
    rmse: f64,
    leverage_value: f64,
) -> f64 {
    let denom = rmse * (1.0 - leverage_value).max(1e-15).sqrt();
    (actual - predicted) / denom
}

/// F-ratio for testing number of PCA components
/// Compares variance explained by last component to residual variance
pub fn f_ratio_test(eigenvalues: &[f64], n_components: usize, _n_samples: usize, n_vars: usize) -> f64 {
    if n_components == 0 || n_components >= eigenvalues.len() {
        return 0.0;
    }
    let residual_variance: f64 = eigenvalues[n_components..].iter().sum::<f64>()
        / (n_vars - n_components) as f64;
    if residual_variance < 1e-15 {
        return f64::INFINITY;
    }
    eigenvalues[n_components - 1] / residual_variance
}

/// Slope and bias correction for predictions
pub fn slope_bias_correction(actual: &[f64], predicted: &[f64]) -> (f64, f64) {
    let n = actual.len() as f64;
    let mean_a = actual.iter().sum::<f64>() / n;
    let mean_p = predicted.iter().sum::<f64>() / n;

    let mut ss_pp = 0.0;
    let mut ss_pa = 0.0;
    for i in 0..actual.len() {
        let dp = predicted[i] - mean_p;
        let da = actual[i] - mean_a;
        ss_pp += dp * dp;
        ss_pa += dp * da;
    }

    let slope = if ss_pp.abs() > 1e-15 { ss_pa / ss_pp } else { 1.0 };
    let intercept = mean_a - slope * mean_p;
    (slope, intercept)
}

/// Apply slope/bias correction to predictions
pub fn apply_correction(predicted: &[f64], slope: f64, intercept: f64) -> Vec<f64> {
    predicted.iter().map(|&p| slope * p + intercept).collect()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- Wavelength Range Tests ---

    #[test]
    fn test_nir_full_range() {
        let range = WavelengthRange::NIR_FULL;
        assert!(range.contains(780.0));
        assert!(range.contains(1500.0));
        assert!(range.contains(2500.0));
        assert!(!range.contains(779.0));
        assert!(!range.contains(2501.0));
    }

    #[test]
    fn test_oh_overtone_range() {
        let range = WavelengthRange::OH_OVERTONE;
        assert!(range.contains(1425.0));
        assert!(!range.contains(1399.0));
        assert!(!range.contains(1451.0));
    }

    #[test]
    fn test_ch_stretching_range() {
        let range = WavelengthRange::CH_STRETCHING;
        assert!(range.contains(1700.0));
        assert!(!range.contains(1679.0));
    }

    #[test]
    fn test_nh_stretching_range() {
        let range = WavelengthRange::NH_STRETCHING;
        assert!(range.contains(1520.0));
        assert!(!range.contains(1560.0));
    }

    #[test]
    fn test_band_identification() {
        let band = identify_band(1420.0);
        assert!(band.is_some());
        assert_eq!(band.unwrap().label, "O-H overtone (moisture)");

        let band = identify_band(1710.0);
        assert!(band.is_some());
        assert_eq!(band.unwrap().label, "C-H stretching (fats/oils)");

        let band = identify_band(1000.0);
        assert!(band.is_none());
    }

    #[test]
    fn test_oh_combination_band() {
        let band = identify_band(1920.0);
        assert!(band.is_some());
        assert_eq!(band.unwrap().label, "O-H combination (water)");
    }

    #[test]
    fn test_ch_combination_band() {
        let band = identify_band(2300.0);
        assert!(band.is_some());
        assert_eq!(band.unwrap().label, "C-H combination (starch)");
    }

    // --- NirSpectrum Tests ---

    #[test]
    fn test_spectrum_creation() {
        let wl = vec![1000.0, 1100.0, 1200.0];
        let vals = vec![0.5, 0.6, 0.7];
        let spec = NirSpectrum::new(wl, vals);
        assert_eq!(spec.len(), 3);
        assert!(!spec.is_empty());
    }

    #[test]
    fn test_reflectance_to_absorbance() {
        let wl = vec![1000.0, 1500.0];
        let refl = vec![1.0, 0.1];
        let spec = NirSpectrum::new(wl, refl);
        let abs_spec = spec.reflectance_to_absorbance();
        assert!(approx_eq(abs_spec.values[0], 0.0, 1e-10));
        assert!(approx_eq(abs_spec.values[1], 1.0, 1e-10));
    }

    #[test]
    fn test_absorbance_to_reflectance() {
        let wl = vec![1000.0, 1500.0];
        let abs_vals = vec![0.0, 1.0];
        let spec = NirSpectrum::new(wl, abs_vals);
        let refl_spec = spec.absorbance_to_reflectance();
        assert!(approx_eq(refl_spec.values[0], 1.0, 1e-10));
        assert!(approx_eq(refl_spec.values[1], 0.1, 1e-10));
    }

    #[test]
    fn test_extract_range() {
        let wl = vec![1000.0, 1200.0, 1420.0, 1440.0, 1600.0];
        let vals = vec![0.1, 0.2, 0.3, 0.4, 0.5];
        let spec = NirSpectrum::new(wl, vals);
        let extracted = spec.extract_range(&WavelengthRange::OH_OVERTONE);
        assert_eq!(extracted.len(), 2);
        assert!(approx_eq(extracted.values[0], 0.3, EPS));
        assert!(approx_eq(extracted.values[1], 0.4, EPS));
    }

    // --- SNV Tests ---

    #[test]
    fn test_snv_basic() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = snv(&data);
        let mean = result.iter().sum::<f64>() / result.len() as f64;
        assert!(approx_eq(mean, 0.0, 1e-10));
        let variance =
            result.iter().map(|&x| x * x).sum::<f64>() / (result.len() as f64 - 1.0);
        assert!(approx_eq(variance, 1.0, 1e-10));
    }

    #[test]
    fn test_snv_constant_spectrum() {
        let data = vec![5.0, 5.0, 5.0];
        let result = snv(&data);
        for &v in &result {
            assert!(approx_eq(v, 0.0, 1e-10));
        }
    }

    // --- MSC Tests ---

    #[test]
    fn test_msc_identity() {
        let reference = vec![1.0, 2.0, 3.0, 4.0];
        let spectrum = reference.clone();
        let result = msc(&spectrum, &reference);
        for (i, &v) in result.iter().enumerate() {
            assert!(approx_eq(v, reference[i], 1e-8));
        }
    }

    #[test]
    fn test_msc_scaled_spectrum() {
        let reference = vec![1.0, 2.0, 3.0, 4.0];
        let spectrum: Vec<f64> = reference.iter().map(|&x| 2.0 * x + 0.5).collect();
        let result = msc(&spectrum, &reference);
        for (i, &v) in result.iter().enumerate() {
            assert!(approx_eq(v, reference[i], 0.1));
        }
    }

    #[test]
    fn test_msc_batch() {
        let spectra = vec![
            vec![1.0, 2.0, 3.0],
            vec![2.0, 4.0, 6.0],
            vec![0.5, 1.0, 1.5],
        ];
        let result = msc_batch(&spectra);
        assert_eq!(result.len(), 3);
    }

    // --- Smoothing and Derivative Tests ---

    #[test]
    fn test_moving_average() {
        let data = vec![1.0, 3.0, 5.0, 3.0, 1.0];
        let result = moving_average(&data, 3);
        assert!(approx_eq(result[2], 11.0 / 3.0, EPS));
    }

    #[test]
    fn test_sg_smooth_preserves_constant() {
        let data = vec![5.0; 11];
        let result = sg_smooth(&data, 5, 2);
        for &v in &result[2..9] {
            assert!(approx_eq(v, 5.0, 0.01));
        }
    }

    #[test]
    fn test_sg_first_derivative() {
        let data: Vec<f64> = (0..21).map(|i| i as f64).collect();
        let deriv = sg_derivative(&data, 5, 2, 1);
        for &v in &deriv[3..18] {
            assert!(approx_eq(v, 1.0, 0.1));
        }
    }

    #[test]
    fn test_sg_second_derivative() {
        let data: Vec<f64> = (0..21).map(|i| (i as f64) * (i as f64)).collect();
        let deriv = sg_derivative(&data, 5, 3, 2);
        for &v in &deriv[4..17] {
            assert!(approx_eq(v, 2.0, 0.5));
        }
    }

    #[test]
    fn test_norris_williams_derivative() {
        let data: Vec<f64> = (0..20).map(|i| i as f64 * 2.0).collect();
        let deriv = norris_williams_derivative(&data, 2, 1);
        for &v in &deriv[3..17] {
            assert!(v > 0.0);
        }
    }

    // --- Baseline Correction Tests ---

    #[test]
    fn test_baseline_linear() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = baseline_linear(&data);
        for &v in &result {
            assert!(approx_eq(v, 0.0, EPS));
        }
    }

    #[test]
    fn test_baseline_polynomial() {
        let data: Vec<f64> = (0..11)
            .map(|i| {
                let x = i as f64 / 10.0;
                0.5 * x * x + 0.1 * x
            })
            .collect();
        let result = baseline_polynomial(&data, 2);
        for &v in &result {
            assert!(approx_eq(v, 0.0, 0.01));
        }
    }

    // --- Mean Centering and Autoscaling ---

    #[test]
    fn test_mean_center() {
        let data = vec![2.0, 4.0, 6.0];
        let result = mean_center(&data);
        assert!(approx_eq(result[0], -2.0, EPS));
        assert!(approx_eq(result[1], 0.0, EPS));
        assert!(approx_eq(result[2], 2.0, EPS));
    }

    #[test]
    fn test_autoscale() {
        let data = vec![2.0, 4.0, 6.0];
        let result = autoscale(&data);
        let mean = result.iter().sum::<f64>() / 3.0;
        assert!(approx_eq(mean, 0.0, EPS));
        let var = result.iter().map(|&x| x * x).sum::<f64>() / 2.0;
        assert!(approx_eq(var, 1.0, EPS));
    }

    // --- SpectralPreprocessor Pipeline ---

    #[test]
    fn test_preprocessor_pipeline() {
        let mut pp = SpectralPreprocessor::new();
        pp.add_step(PreprocessStep::MovingAverage(3));
        pp.add_step(PreprocessStep::Snv);
        let data = vec![1.0, 3.0, 5.0, 3.0, 1.0];
        let result = pp.process(&data);
        assert_eq!(result.len(), 5);
        let mean = result.iter().sum::<f64>() / result.len() as f64;
        assert!(approx_eq(mean, 0.0, 1e-10));
    }

    // --- PCA Tests ---

    #[test]
    fn test_pca_basic() {
        let data = vec![
            vec![1.0, 2.0],
            vec![2.0, 4.0],
            vec![3.0, 6.0],
            vec![4.0, 8.0],
            vec![5.0, 10.0],
        ];
        let model = PcaModel::fit(&data, 2);
        assert_eq!(model.n_components, 2);
        let var_ratios = model.explained_variance_ratio();
        assert!(var_ratios[0] > 0.99);
    }

    #[test]
    fn test_pca_explained_variance() {
        // Use data with clear variance structure for power iteration
        let data = vec![
            vec![10.0, 1.0, 0.1],
            vec![20.0, 2.0, 0.2],
            vec![30.0, 3.0, 0.3],
            vec![40.0, 4.0, 0.4],
            vec![50.0, 5.0, 0.5],
        ];
        let model = PcaModel::fit(&data, 3);
        let var_ratios = model.explained_variance_ratio();
        // First PC should dominate
        assert!(var_ratios[0] > 0.9);
        // All ratios should be non-negative
        for &r in &var_ratios {
            assert!(r >= -0.01);
        }
    }

    #[test]
    fn test_pca_hotellings_t2() {
        let data = vec![
            vec![1.0, 2.0],
            vec![2.0, 3.0],
            vec![3.0, 4.0],
            vec![100.0, 200.0],
        ];
        let model = PcaModel::fit(&data, 2);
        let t2 = model.hotellings_t2();
        assert!(t2[3] > t2[0]);
        assert!(t2[3] > t2[1]);
        assert!(t2[3] > t2[2]);
    }

    #[test]
    fn test_pca_q_residual() {
        let data = vec![
            vec![1.0, 2.0, 3.0],
            vec![2.0, 4.0, 6.0],
            vec![3.0, 6.0, 9.0],
        ];
        let model = PcaModel::fit(&data, 1);
        let q = model.q_residual(&data);
        for &qi in &q {
            assert!(qi < 1.0);
        }
    }

    #[test]
    fn test_pca_transform() {
        let data = vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
            vec![1.0, 1.0],
            vec![0.0, 0.0],
        ];
        let model = PcaModel::fit(&data, 2);
        let new_sample = vec![0.5, 0.5];
        let scores = model.transform(&new_sample);
        assert_eq!(scores.len(), 2);
    }

    // --- PLS Tests ---

    #[test]
    fn test_pls_linear_relationship() {
        let x = vec![
            vec![1.0, 2.0, 3.0],
            vec![2.0, 3.0, 4.0],
            vec![3.0, 4.0, 5.0],
            vec![4.0, 5.0, 6.0],
            vec![5.0, 6.0, 7.0],
        ];
        let y: Vec<f64> = x.iter().map(|row| row.iter().sum()).collect();
        let model = PlsModel::fit(&x, &y, 2);
        let predictions = model.predict_batch(&x);
        let r2 = r_squared(&y, &predictions);
        assert!(r2 > 0.99);
    }

    #[test]
    fn test_pls_prediction() {
        let x = vec![
            vec![1.0, 0.5],
            vec![2.0, 1.0],
            vec![3.0, 1.5],
            vec![4.0, 2.0],
            vec![5.0, 2.5],
        ];
        let y = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        let model = PlsModel::fit(&x, &y, 2);
        let pred = model.predict(&vec![3.0, 1.5]);
        assert!(approx_eq(pred, 30.0, 2.0));
    }

    #[test]
    fn test_pls_vip_scores() {
        let x = vec![
            vec![1.0, 0.0, 0.1],
            vec![2.0, 0.0, 0.2],
            vec![3.0, 0.0, 0.3],
            vec![4.0, 0.0, 0.4],
            vec![5.0, 0.0, 0.5],
        ];
        let y = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let model = PlsModel::fit(&x, &y, 2);
        assert_eq!(model.vip_scores.len(), 3);
    }

    // --- Cross-Validation Tests ---

    #[test]
    fn test_rmsep_calculation() {
        let actual = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let predicted = vec![1.1, 2.2, 2.9, 3.8, 5.1];
        let err = rmsep(&actual, &predicted);
        assert!(err > 0.0);
        assert!(err < 0.3);
    }

    #[test]
    fn test_r_squared_perfect() {
        let actual = vec![1.0, 2.0, 3.0];
        let predicted = vec![1.0, 2.0, 3.0];
        let r2 = r_squared(&actual, &predicted);
        assert!(approx_eq(r2, 1.0, EPS));
    }

    #[test]
    fn test_r_squared_poor() {
        let actual = vec![1.0, 2.0, 3.0];
        let predicted = vec![3.0, 2.0, 1.0];
        let r2 = r_squared(&actual, &predicted);
        assert!(r2 < 0.0);
    }

    #[test]
    fn test_bias_calculation() {
        let actual = vec![1.0, 2.0, 3.0];
        let predicted = vec![1.5, 2.5, 3.5];
        let b = bias(&actual, &predicted);
        assert!(approx_eq(b, 0.5, EPS));
    }

    #[test]
    fn test_rpd_calculation() {
        let actual = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let predicted = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let rpd_val = rpd(&actual, &predicted);
        assert!(rpd_val > 1000.0);
    }

    #[test]
    fn test_optimal_components() {
        let rmsecv = vec![1.5, 1.2, 1.0, 1.1, 1.3];
        let opt = optimal_components(&rmsecv);
        assert_eq!(opt, 3);
    }

    // --- Beer-Lambert Tests ---

    #[test]
    fn test_beer_lambert_law() {
        let a = beer_lambert(100.0, 0.01, 1.0);
        assert!(approx_eq(a, 1.0, EPS));
    }

    #[test]
    fn test_beer_lambert_concentration() {
        let c = beer_lambert_concentration(1.0, 100.0, 1.0);
        assert!(approx_eq(c, 0.01, EPS));
    }

    #[test]
    fn test_multicomponent_analysis() {
        let absorptivities = vec![
            vec![10.0, 0.0],
            vec![0.0, 20.0],
        ];
        let concentrations = vec![0.05, 0.03];
        let absorbances: Vec<f64> = absorptivities
            .iter()
            .map(|row| row[0] * concentrations[0] + row[1] * concentrations[1])
            .collect();
        let result = multicomponent_analysis(&absorbances, &absorptivities, 1.0);
        assert!(approx_eq(result[0], 0.05, 1e-8));
        assert!(approx_eq(result[1], 0.03, 1e-8));
    }

    // --- Outlier Detection Tests ---

    #[test]
    fn test_mahalanobis_distance() {
        let scores = vec![1.0, 2.0];
        let eigenvalues = vec![1.0, 1.0];
        let d = mahalanobis_distance(&scores, &eigenvalues);
        assert!(approx_eq(d, 5.0_f64.sqrt(), EPS));
    }

    #[test]
    fn test_leverage_calculation() {
        let scores = vec![0.0, 0.0];
        let eigenvalues = vec![1.0, 1.0];
        let h = leverage(&scores, &eigenvalues, 10);
        assert!(approx_eq(h, 0.1, EPS));
    }

    #[test]
    fn test_studentized_residual() {
        let sr = studentized_residual(5.0, 4.0, 1.0, 0.1);
        let expected = 1.0 / (0.9_f64.sqrt());
        assert!(approx_eq(sr, expected, 0.01));
    }

    #[test]
    fn test_f_ratio() {
        let eigenvalues = vec![10.0, 5.0, 1.0, 0.5, 0.1];
        let f = f_ratio_test(&eigenvalues, 2, 50, 5);
        let residual_mean = (1.0 + 0.5 + 0.1) / 3.0;
        let expected = 5.0 / residual_mean;
        assert!(approx_eq(f, expected, 0.01));
    }

    // --- Slope/Bias Correction ---

    #[test]
    fn test_slope_bias_correction() {
        let actual = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let predicted = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let (slope, intercept) = slope_bias_correction(&actual, &predicted);
        let corrected = apply_correction(&predicted, slope, intercept);
        for (i, &c) in corrected.iter().enumerate() {
            assert!(approx_eq(c, actual[i], 0.01));
        }
    }

    #[test]
    fn test_apply_correction() {
        let pred = vec![1.0, 2.0, 3.0];
        let result = apply_correction(&pred, 2.0, 1.0);
        assert!(approx_eq(result[0], 3.0, EPS));
        assert!(approx_eq(result[1], 5.0, EPS));
        assert!(approx_eq(result[2], 7.0, EPS));
    }

    // --- PLS Cross-Validation ---

    #[test]
    fn test_pls_cross_validation() {
        let x = vec![
            vec![1.0, 2.0],
            vec![2.0, 3.0],
            vec![3.0, 4.0],
            vec![4.0, 5.0],
            vec![5.0, 6.0],
        ];
        let y = vec![3.0, 5.0, 7.0, 9.0, 11.0];
        let rmsecv = pls_cross_validate(&x, &y, 2);
        assert!(!rmsecv.is_empty());
        assert!(rmsecv[0] < 2.0);
    }

    // --- SEL ---

    #[test]
    fn test_sel_calculation() {
        let reference = vec![10.0, 20.0];
        let replicates = vec![vec![9.8, 10.2, 10.0], vec![19.9, 20.1, 20.0]];
        let sel_val = sel(&reference, &replicates);
        assert!(sel_val > 0.0);
        assert!(sel_val < 1.0);
    }

    // --- Reflectance/Absorbance Roundtrip ---

    #[test]
    fn test_reflectance_absorbance_roundtrip() {
        let wl = vec![1000.0, 1500.0, 2000.0];
        let refl = vec![0.5, 0.3, 0.8];
        let spec = NirSpectrum::new(wl, refl.clone());
        let abs_spec = spec.reflectance_to_absorbance();
        let back = abs_spec.absorbance_to_reflectance();
        for i in 0..refl.len() {
            assert!(approx_eq(back.values[i], refl[i], 1e-10));
        }
    }

    // --- Preprocessor with SG derivative ---

    #[test]
    fn test_preprocessor_sg_derivative_step() {
        let mut pp = SpectralPreprocessor::new();
        pp.add_step(PreprocessStep::SgDerivative {
            window: 5,
            poly_order: 2,
            deriv_order: 1,
        });
        let data: Vec<f64> = (0..15).map(|i| i as f64 * 2.0).collect();
        let result = pp.process(&data);
        assert_eq!(result.len(), 15);
        for &v in &result[3..12] {
            assert!(approx_eq(v, 2.0, 0.2));
        }
    }

    // --- PCA Cumulative Variance ---

    #[test]
    fn test_pca_cumulative_variance_monotonic() {
        let data = vec![
            vec![1.0, 0.5, 0.2],
            vec![2.0, 1.5, 0.3],
            vec![3.0, 2.0, 0.8],
            vec![4.0, 3.5, 0.1],
            vec![5.0, 4.0, 0.9],
        ];
        let model = PcaModel::fit(&data, 3);
        let cum = model.cumulative_variance();
        for i in 1..cum.len() {
            assert!(cum[i] >= cum[i - 1] - 1e-10);
        }
    }

    // --- Edge case: single sample ---

    #[test]
    fn test_snv_single_value() {
        let data = vec![5.0];
        let result = snv(&data);
        assert_eq!(result.len(), 1);
    }

    // --- Convolution helper ---

    #[test]
    fn test_convolve_same_identity() {
        let signal = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let kernel = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let result = convolve_same(&signal, &kernel);
        assert!(approx_eq(result[2], 1.0, EPS));
    }

    // --- PLS with more components ---

    #[test]
    fn test_pls_multiple_components() {
        let x = vec![
            vec![1.0, 0.0, 0.5],
            vec![0.0, 1.0, 0.3],
            vec![1.0, 1.0, 0.8],
            vec![2.0, 0.0, 1.0],
            vec![0.0, 2.0, 0.6],
            vec![2.0, 2.0, 1.6],
        ];
        let y = vec![1.5, 1.3, 2.8, 3.0, 2.6, 5.6];
        let model = PlsModel::fit(&x, &y, 3);
        let predictions = model.predict_batch(&x);
        let r2 = r_squared(&y, &predictions);
        assert!(r2 > 0.9);
    }

    // --- VIP score length check ---

    #[test]
    fn test_vip_scores_length() {
        let x = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![2.0, 3.0, 4.0, 5.0],
            vec![3.0, 4.0, 5.0, 6.0],
            vec![4.0, 5.0, 6.0, 7.0],
        ];
        let y = vec![10.0, 14.0, 18.0, 22.0];
        let model = PlsModel::fit(&x, &y, 2);
        assert_eq!(model.vip_scores.len(), 4);
    }
}
