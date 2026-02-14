//! # Thermal Imaging Processor
//!
//! Processes thermal infrared imagery for temperature measurement, hotspot
//! detection, and non-uniformity correction (NUC). Supports radiometric
//! calibration from raw sensor counts to absolute temperature.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw Counts -> NUC (gain/offset) -> Radiance -> Atmospheric Correction -> Temperature (Planck)
//! ```
//!
//! ## Key Equations
//!
//! - **Planck Spectral Radiance**: L(T, lambda) = (2*h*c^2 / lambda^5) / (exp(h*c / (lambda*k*T)) - 1)
//! - **Stefan-Boltzmann**: P = epsilon * sigma * A * T^4
//! - **Two-Point NUC**: corrected = gain * raw + offset
//! - **NETD**: delta_T / (signal / noise)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::thermal_imaging_processor::{ThermalConfig, ThermalProcessor};
//!
//! let config = ThermalConfig {
//!     sensor_rows: 4,
//!     sensor_cols: 4,
//!     pixel_pitch_um: 17.0,
//!     integration_time_us: 16000.0,
//!     ambient_temp_c: 25.0,
//!     emissivity: 0.95,
//! };
//!
//! let processor = ThermalProcessor::new(config);
//! let raw_counts = vec![vec![3000u16; 4]; 4];
//! let frame = processor.process_frame(&raw_counts);
//! assert!(frame.mean_temp > -40.0 && frame.mean_temp < 500.0);
//! ```

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Planck constant (J*s).
const PLANCK_H: f64 = 6.626_070_15e-34;

/// Speed of light (m/s).
const SPEED_OF_LIGHT: f64 = 2.997_924_58e8;

/// Boltzmann constant (J/K).
const BOLTZMANN_K: f64 = 1.380_649e-23;

/// Stefan-Boltzmann constant (W / m^2 / K^4).
const STEFAN_BOLTZMANN_SIGMA: f64 = 5.670_374_419e-8;

/// Default long-wave infrared centre wavelength (micrometres).
const DEFAULT_LWIR_WAVELENGTH_UM: f64 = 10.0;

/// Default calibration gain (counts -> radiance).
const DEFAULT_CAL_GAIN: f64 = 1.0e-6;

/// Default calibration offset.
const DEFAULT_CAL_OFFSET: f64 = 0.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Sensor and scene configuration for thermal processing.
#[derive(Debug, Clone)]
pub struct ThermalConfig {
    /// Number of sensor rows (pixels).
    pub sensor_rows: usize,
    /// Number of sensor columns (pixels).
    pub sensor_cols: usize,
    /// Pixel pitch in micrometres.
    pub pixel_pitch_um: f64,
    /// Detector integration time in microseconds.
    pub integration_time_us: f64,
    /// Ambient / background temperature in degrees Celsius.
    pub ambient_temp_c: f64,
    /// Surface emissivity (0.0 .. 1.0, default 0.95).
    pub emissivity: f64,
}

impl Default for ThermalConfig {
    fn default() -> Self {
        Self {
            sensor_rows: 240,
            sensor_cols: 320,
            pixel_pitch_um: 17.0,
            integration_time_us: 16000.0,
            ambient_temp_c: 25.0,
            emissivity: 0.95,
        }
    }
}

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// A detected thermal hotspot (connected region above threshold).
#[derive(Debug, Clone, PartialEq)]
pub struct Hotspot {
    /// Row index of the hottest pixel in this region.
    pub row: usize,
    /// Column index of the hottest pixel in this region.
    pub col: usize,
    /// Temperature of the hottest pixel (degrees Celsius).
    pub temp_c: f64,
    /// Number of pixels in the connected region.
    pub area_pixels: usize,
}

/// Processed thermal frame with calibrated temperatures and analytics.
#[derive(Debug, Clone)]
pub struct ThermalFrame {
    /// Temperature map in degrees Celsius (row-major).
    pub temperature_c: Vec<Vec<f64>>,
    /// Minimum temperature across the frame.
    pub min_temp: f64,
    /// Maximum temperature across the frame.
    pub max_temp: f64,
    /// Mean temperature across the frame.
    pub mean_temp: f64,
    /// Detected hotspots (empty if none above threshold).
    pub hotspots: Vec<Hotspot>,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Main thermal imaging processor.
///
/// Takes raw sensor counts and produces calibrated temperature maps with
/// hotspot detection.
#[derive(Debug, Clone)]
pub struct ThermalProcessor {
    config: ThermalConfig,
    /// Calibration gain (counts -> radiance). Per-pixel maps can be applied
    /// separately via [`non_uniformity_correction`].
    cal_gain: f64,
    /// Calibration offset.
    cal_offset: f64,
    /// Centre wavelength for Planck inversion (micrometres).
    wavelength_um: f64,
    /// Hotspot detection threshold above mean (degrees Celsius).
    hotspot_threshold_above_mean: f64,
}

impl ThermalProcessor {
    /// Create a new processor with the given configuration.
    ///
    /// Uses default calibration constants; call [`with_calibration`] to
    /// override.
    pub fn new(config: ThermalConfig) -> Self {
        Self {
            config,
            cal_gain: DEFAULT_CAL_GAIN,
            cal_offset: DEFAULT_CAL_OFFSET,
            wavelength_um: DEFAULT_LWIR_WAVELENGTH_UM,
            hotspot_threshold_above_mean: 10.0,
        }
    }

    /// Override the linear calibration gain and offset.
    pub fn with_calibration(mut self, gain: f64, offset: f64) -> Self {
        self.cal_gain = gain;
        self.cal_offset = offset;
        self
    }

    /// Override the centre wavelength (micrometres).
    pub fn with_wavelength(mut self, wavelength_um: f64) -> Self {
        self.wavelength_um = wavelength_um;
        self
    }

    /// Override the hotspot detection threshold (degrees above mean).
    pub fn with_hotspot_threshold(mut self, threshold_c: f64) -> Self {
        self.hotspot_threshold_above_mean = threshold_c;
        self
    }

    /// Process a raw frame of 16-bit sensor counts into a calibrated
    /// [`ThermalFrame`].
    ///
    /// # Arguments
    /// * `raw_counts` - 2-D array of raw sensor counts (rows x cols).
    ///
    /// # Panics
    /// Panics if `raw_counts` dimensions do not match `config.sensor_rows`
    /// and `config.sensor_cols`.
    pub fn process_frame(&self, raw_counts: &[Vec<u16>]) -> ThermalFrame {
        assert_eq!(
            raw_counts.len(),
            self.config.sensor_rows,
            "row count mismatch"
        );
        for (i, row) in raw_counts.iter().enumerate() {
            assert_eq!(
                row.len(),
                self.config.sensor_cols,
                "col count mismatch in row {}",
                i
            );
        }

        // Convert counts -> radiance -> temperature
        let rows = self.config.sensor_rows;
        let cols = self.config.sensor_cols;
        let mut temperature_c = vec![vec![0.0_f64; cols]; rows];
        let mut min_temp = f64::MAX;
        let mut max_temp = f64::MIN;
        let mut sum_temp = 0.0_f64;

        for r in 0..rows {
            for c in 0..cols {
                let radiance = counts_to_radiance(raw_counts[r][c], self.cal_gain, self.cal_offset);
                // Account for emissivity: apparent radiance includes
                // reflected ambient contribution.
                let corrected_radiance = emissivity_correct(
                    radiance,
                    self.config.emissivity,
                    self.config.ambient_temp_c,
                    self.wavelength_um,
                );
                let temp_k = planck_radiance_to_temp(corrected_radiance, self.wavelength_um);
                let temp_c = temp_k - 273.15;
                temperature_c[r][c] = temp_c;
                if temp_c < min_temp {
                    min_temp = temp_c;
                }
                if temp_c > max_temp {
                    max_temp = temp_c;
                }
                sum_temp += temp_c;
            }
        }

        let total_pixels = (rows * cols) as f64;
        let mean_temp = if total_pixels > 0.0 {
            sum_temp / total_pixels
        } else {
            0.0
        };

        let threshold = mean_temp + self.hotspot_threshold_above_mean;
        let hotspots = detect_hotspots(&temperature_c, threshold);

        ThermalFrame {
            temperature_c,
            min_temp,
            max_temp,
            mean_temp,
            hotspots,
        }
    }
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Convert raw sensor counts to spectral radiance using linear calibration.
///
/// `radiance = gain * counts + offset`
///
/// Units of radiance depend on the calibration coefficients; typically
/// W / (m^2 * sr * um).
pub fn counts_to_radiance(counts: u16, gain: f64, offset: f64) -> f64 {
    gain * (counts as f64) + offset
}

/// Planck spectral radiance for a blackbody at temperature `temp_k` (Kelvin)
/// and wavelength `wavelength_um` (micrometres).
///
/// L(T, lambda) = (2*h*c^2 / lambda^5) / (exp(h*c / (lambda*k*T)) - 1)
///
/// Returns W / (m^2 * sr * m). To get per-micrometre multiply by 1e-6 or
/// adjust lambda to metres directly (this function handles the conversion).
pub fn planck_temp_to_radiance(temp_k: f64, wavelength_um: f64) -> f64 {
    if temp_k <= 0.0 || wavelength_um <= 0.0 {
        return 0.0;
    }
    let lambda = wavelength_um * 1.0e-6; // convert um -> m
    let c1 = 2.0 * PLANCK_H * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    let c2 = PLANCK_H * SPEED_OF_LIGHT / (BOLTZMANN_K * temp_k);
    let exponent = c2 / lambda;
    // Guard against overflow: if exponent > ~700, exp() overflows f64.
    if exponent > 700.0 {
        return 0.0;
    }
    c1 / (lambda.powi(5) * (exponent.exp() - 1.0))
}

/// Invert the Planck law: given spectral radiance and wavelength, compute the
/// equivalent blackbody temperature in Kelvin.
///
/// T = (h*c) / (lambda * k * ln(1 + 2*h*c^2 / (lambda^5 * L)))
pub fn planck_radiance_to_temp(radiance: f64, wavelength_um: f64) -> f64 {
    if radiance <= 0.0 || wavelength_um <= 0.0 {
        return 0.0;
    }
    let lambda = wavelength_um * 1.0e-6;
    let c1 = 2.0 * PLANCK_H * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    let c2 = PLANCK_H * SPEED_OF_LIGHT / BOLTZMANN_K;
    let arg = 1.0 + c1 / (lambda.powi(5) * radiance);
    if arg <= 1.0 {
        return 0.0;
    }
    c2 / (lambda * arg.ln())
}

/// Two-point non-uniformity correction (NUC).
///
/// Applies per-pixel gain and offset maps to raw sensor data:
///
/// `corrected[r][c] = gain_map[r][c] * frame[r][c] + offset_map[r][c]`
///
/// # Panics
/// Panics if `frame`, `gain_map`, and `offset_map` have different dimensions.
pub fn non_uniformity_correction(
    frame: &[Vec<u16>],
    gain_map: &[Vec<f64>],
    offset_map: &[Vec<f64>],
) -> Vec<Vec<f64>> {
    let rows = frame.len();
    assert_eq!(rows, gain_map.len(), "gain_map row count mismatch");
    assert_eq!(rows, offset_map.len(), "offset_map row count mismatch");

    let mut result = Vec::with_capacity(rows);
    for r in 0..rows {
        let cols = frame[r].len();
        assert_eq!(cols, gain_map[r].len(), "gain_map col mismatch row {}", r);
        assert_eq!(
            cols,
            offset_map[r].len(),
            "offset_map col mismatch row {}",
            r
        );
        let mut row = Vec::with_capacity(cols);
        for c in 0..cols {
            row.push(gain_map[r][c] * (frame[r][c] as f64) + offset_map[r][c]);
        }
        result.push(row);
    }
    result
}

/// Detect thermal hotspots using connected-component analysis.
///
/// Any pixel with temperature >= `threshold_c` is considered part of a
/// hotspot. Adjacent pixels (4-connected) are merged into a single hotspot
/// whose reported position is the hottest pixel in the component.
pub fn detect_hotspots(temps: &[Vec<f64>], threshold_c: f64) -> Vec<Hotspot> {
    if temps.is_empty() {
        return Vec::new();
    }
    let rows = temps.len();
    let cols = temps[0].len();
    let mut visited = vec![vec![false; cols]; rows];
    let mut hotspots = Vec::new();

    for r in 0..rows {
        for c in 0..cols {
            if !visited[r][c] && temps[r][c] >= threshold_c {
                // BFS flood fill
                let mut stack: Vec<(usize, usize)> = vec![(r, c)];
                visited[r][c] = true;
                let mut area = 0usize;
                let mut best_row = r;
                let mut best_col = c;
                let mut best_temp = temps[r][c];

                while let Some((cr, cc)) = stack.pop() {
                    area += 1;
                    if temps[cr][cc] > best_temp {
                        best_temp = temps[cr][cc];
                        best_row = cr;
                        best_col = cc;
                    }
                    // 4-connected neighbours
                    let neighbours: [(isize, isize); 4] =
                        [(-1, 0), (1, 0), (0, -1), (0, 1)];
                    for (dr, dc) in &neighbours {
                        let nr = cr as isize + dr;
                        let nc = cc as isize + dc;
                        if nr >= 0
                            && nr < rows as isize
                            && nc >= 0
                            && nc < cols as isize
                        {
                            let nr = nr as usize;
                            let nc = nc as usize;
                            if !visited[nr][nc] && temps[nr][nc] >= threshold_c {
                                visited[nr][nc] = true;
                                stack.push((nr, nc));
                            }
                        }
                    }
                }

                hotspots.push(Hotspot {
                    row: best_row,
                    col: best_col,
                    temp_c: best_temp,
                    area_pixels: area,
                });
            }
        }
    }

    hotspots
}

/// Simplified atmospheric transmission factor.
///
/// Models attenuation through the atmosphere for LWIR thermal imaging,
/// inspired by simplified LOWTRAN curves. The transmission decreases with
/// distance, humidity, and atmospheric temperature (which controls water
/// vapour absorption).
///
/// Returns a value in (0.0, 1.0].
///
/// # Arguments
/// * `distance_m` - Path length through the atmosphere (metres).
/// * `humidity_percent` - Relative humidity (0 - 100).
/// * `temp_c` - Ambient atmospheric temperature (degrees Celsius).
pub fn atmospheric_transmission(distance_m: f64, humidity_percent: f64, temp_c: f64) -> f64 {
    if distance_m <= 0.0 {
        return 1.0;
    }
    // Water vapour density approximation (g/m^3) from humidity and temperature
    // using simplified Magnus formula for saturation vapour pressure.
    let temp_k = temp_c + 273.15;
    let es = 6.1078 * ((17.269 * temp_c) / (237.29 + temp_c)).exp(); // hPa
    let water_vapour_density = (humidity_percent / 100.0) * es * 216.7 / temp_k; // g/m^3

    // Absorption coefficient (simplified): combines H2O and CO2 contributions
    // in the 8-14 um LWIR window. Typical values 0.0001-0.001 per metre for
    // moderate humidity.
    let alpha = 4.0e-4 * (1.0 + 0.02 * water_vapour_density); // per metre

    let tau = (-alpha * distance_m).exp();
    tau.clamp(0.0, 1.0)
}

/// Total radiant power emitted by a surface via the Stefan-Boltzmann law.
///
/// P = epsilon * sigma * A * T^4
///
/// # Arguments
/// * `temp_k` - Surface temperature (Kelvin).
/// * `area_m2` - Emitting area (square metres).
/// * `emissivity` - Surface emissivity (0.0 .. 1.0).
///
/// Returns power in Watts.
pub fn stefan_boltzmann_power(temp_k: f64, area_m2: f64, emissivity: f64) -> f64 {
    if temp_k <= 0.0 || area_m2 <= 0.0 || emissivity <= 0.0 {
        return 0.0;
    }
    emissivity * STEFAN_BOLTZMANN_SIGMA * area_m2 * temp_k.powi(4)
}

/// Noise Equivalent Temperature Difference (NETD).
///
/// NETD quantifies the minimum detectable temperature difference of a
/// thermal sensor. A lower NETD indicates a more sensitive detector.
///
/// NETD = delta_T / SNR, where SNR = responsivity * signal_radiance / noise
///
/// For simplicity this function computes:
///
/// NETD = noise_counts / (responsivity * dL/dT)
///
/// where dL/dT is evaluated numerically at `signal_temp_k` using the Planck
/// function at 10 um LWIR.
///
/// # Arguments
/// * `signal_temp_k` - Scene temperature (Kelvin).
/// * `noise_counts` - RMS noise in raw counts.
/// * `responsivity` - Detector responsivity (counts per unit radiance).
///
/// Returns NETD in Kelvin.
pub fn netd(signal_temp_k: f64, noise_counts: f64, responsivity: f64) -> f64 {
    if signal_temp_k <= 0.0 || noise_counts <= 0.0 || responsivity <= 0.0 {
        return 0.0;
    }
    let wl = DEFAULT_LWIR_WAVELENGTH_UM;
    let dt = 0.1; // small delta for numerical derivative
    let l1 = planck_temp_to_radiance(signal_temp_k - dt / 2.0, wl);
    let l2 = planck_temp_to_radiance(signal_temp_k + dt / 2.0, wl);
    let dl_dt = (l2 - l1) / dt;
    if dl_dt <= 0.0 {
        return 0.0;
    }
    noise_counts / (responsivity * dl_dt)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Correct measured radiance for emissivity by subtracting the reflected
/// ambient component.
///
/// L_target = (L_measured - (1 - eps) * L_ambient) / eps
fn emissivity_correct(
    measured_radiance: f64,
    emissivity: f64,
    ambient_temp_c: f64,
    wavelength_um: f64,
) -> f64 {
    if emissivity >= 1.0 {
        return measured_radiance;
    }
    let ambient_k = ambient_temp_c + 273.15;
    let l_ambient = planck_temp_to_radiance(ambient_k, wavelength_um);
    let reflected = (1.0 - emissivity) * l_ambient;
    let corrected = (measured_radiance - reflected) / emissivity;
    // Radiance cannot be negative for physical scenes.
    if corrected > 0.0 {
        corrected
    } else {
        measured_radiance / emissivity
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Planck law tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_planck_roundtrip_room_temp() {
        // 300 K (~27 C) at 10 um LWIR
        let temp_k = 300.0;
        let wl = 10.0;
        let radiance = planck_temp_to_radiance(temp_k, wl);
        assert!(radiance > 0.0);
        let recovered = planck_radiance_to_temp(radiance, wl);
        assert!(
            (recovered - temp_k).abs() < 0.01,
            "roundtrip failed: {} vs {}",
            recovered,
            temp_k
        );
    }

    #[test]
    fn test_planck_roundtrip_high_temp() {
        // 1000 K at 4 um MWIR
        let temp_k = 1000.0;
        let wl = 4.0;
        let radiance = planck_temp_to_radiance(temp_k, wl);
        let recovered = planck_radiance_to_temp(radiance, wl);
        assert!(
            (recovered - temp_k).abs() < 0.01,
            "roundtrip failed: {} vs {}",
            recovered,
            temp_k
        );
    }

    #[test]
    fn test_planck_roundtrip_cryogenic() {
        // 77 K (liquid nitrogen) at 10 um
        let temp_k = 77.0;
        let wl = 10.0;
        let radiance = planck_temp_to_radiance(temp_k, wl);
        let recovered = planck_radiance_to_temp(radiance, wl);
        assert!(
            (recovered - temp_k).abs() < 0.05,
            "roundtrip failed: {} vs {}",
            recovered,
            temp_k
        );
    }

    #[test]
    fn test_planck_zero_temp() {
        assert_eq!(planck_temp_to_radiance(0.0, 10.0), 0.0);
    }

    #[test]
    fn test_planck_negative_temp() {
        assert_eq!(planck_temp_to_radiance(-100.0, 10.0), 0.0);
    }

    #[test]
    fn test_planck_zero_wavelength() {
        assert_eq!(planck_temp_to_radiance(300.0, 0.0), 0.0);
    }

    #[test]
    fn test_planck_inverse_zero_radiance() {
        assert_eq!(planck_radiance_to_temp(0.0, 10.0), 0.0);
    }

    #[test]
    fn test_planck_inverse_negative_radiance() {
        assert_eq!(planck_radiance_to_temp(-1.0, 10.0), 0.0);
    }

    #[test]
    fn test_planck_radiance_increases_with_temp() {
        let wl = 10.0;
        let r1 = planck_temp_to_radiance(300.0, wl);
        let r2 = planck_temp_to_radiance(400.0, wl);
        let r3 = planck_temp_to_radiance(500.0, wl);
        assert!(r2 > r1, "radiance should increase with temperature");
        assert!(r3 > r2, "radiance should increase with temperature");
    }

    // -----------------------------------------------------------------------
    // Counts to radiance
    // -----------------------------------------------------------------------

    #[test]
    fn test_counts_to_radiance_basic() {
        let r = counts_to_radiance(1000, 2.0e-6, 0.0);
        assert!((r - 2.0e-3).abs() < 1.0e-12);
    }

    #[test]
    fn test_counts_to_radiance_with_offset() {
        let r = counts_to_radiance(0, 1.0, 5.0);
        assert!((r - 5.0).abs() < 1.0e-12);
    }

    #[test]
    fn test_counts_to_radiance_max_counts() {
        let r = counts_to_radiance(u16::MAX, 1.0e-6, 0.0);
        assert!((r - 65535.0e-6).abs() < 1.0e-12);
    }

    // -----------------------------------------------------------------------
    // Non-uniformity correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_nuc_identity() {
        // Gain = 1, Offset = 0 -> output equals input (as f64)
        let frame = vec![vec![100u16, 200], vec![300, 400]];
        let gain = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
        let offset = vec![vec![0.0, 0.0], vec![0.0, 0.0]];
        let result = non_uniformity_correction(&frame, &gain, &offset);
        assert!((result[0][0] - 100.0).abs() < 1.0e-12);
        assert!((result[1][1] - 400.0).abs() < 1.0e-12);
    }

    #[test]
    fn test_nuc_gain_and_offset() {
        let frame = vec![vec![1000u16]];
        let gain = vec![vec![1.1]];
        let offset = vec![vec![-50.0]];
        let result = non_uniformity_correction(&frame, &gain, &offset);
        let expected = 1.1 * 1000.0 - 50.0;
        assert!(
            (result[0][0] - expected).abs() < 1.0e-10,
            "NUC result {} != expected {}",
            result[0][0],
            expected
        );
    }

    #[test]
    fn test_nuc_corrects_nonuniformity() {
        // Simulate a sensor where pixel (0,0) is "hot" (high gain) and
        // pixel (0,1) is "cold" (low gain). After NUC both should be
        // similar.
        let frame = vec![vec![2000u16, 2000]];
        // Compensating gains
        let gain = vec![vec![0.8, 1.2]];
        let offset = vec![vec![100.0, -100.0]];
        let result = non_uniformity_correction(&frame, &gain, &offset);
        let p0 = result[0][0]; // 0.8 * 2000 + 100 = 1700
        let p1 = result[0][1]; // 1.2 * 2000 - 100 = 2300
        assert!((p0 - 1700.0).abs() < 1.0e-10);
        assert!((p1 - 2300.0).abs() < 1.0e-10);
    }

    #[test]
    #[should_panic(expected = "gain_map row count mismatch")]
    fn test_nuc_dimension_mismatch() {
        let frame = vec![vec![100u16]];
        let gain = vec![vec![1.0], vec![1.0]]; // wrong rows
        let offset = vec![vec![0.0]];
        let _ = non_uniformity_correction(&frame, &gain, &offset);
    }

    // -----------------------------------------------------------------------
    // Hotspot detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_hotspots_single() {
        let temps = vec![
            vec![20.0, 20.0, 20.0],
            vec![20.0, 80.0, 20.0],
            vec![20.0, 20.0, 20.0],
        ];
        let hotspots = detect_hotspots(&temps, 50.0);
        assert_eq!(hotspots.len(), 1);
        assert_eq!(hotspots[0].row, 1);
        assert_eq!(hotspots[0].col, 1);
        assert!((hotspots[0].temp_c - 80.0).abs() < 1.0e-12);
        assert_eq!(hotspots[0].area_pixels, 1);
    }

    #[test]
    fn test_detect_hotspots_connected_region() {
        let temps = vec![
            vec![20.0, 60.0, 20.0],
            vec![60.0, 70.0, 60.0],
            vec![20.0, 60.0, 20.0],
        ];
        let hotspots = detect_hotspots(&temps, 50.0);
        assert_eq!(hotspots.len(), 1);
        assert_eq!(hotspots[0].area_pixels, 5); // cross shape
        assert!((hotspots[0].temp_c - 70.0).abs() < 1.0e-12);
        assert_eq!(hotspots[0].row, 1);
        assert_eq!(hotspots[0].col, 1);
    }

    #[test]
    fn test_detect_hotspots_multiple_disjoint() {
        let temps = vec![
            vec![80.0, 20.0, 90.0],
            vec![20.0, 20.0, 20.0],
            vec![70.0, 20.0, 20.0],
        ];
        let hotspots = detect_hotspots(&temps, 50.0);
        assert_eq!(hotspots.len(), 3);
    }

    #[test]
    fn test_detect_hotspots_none() {
        let temps = vec![vec![20.0, 21.0], vec![19.0, 22.0]];
        let hotspots = detect_hotspots(&temps, 50.0);
        assert!(hotspots.is_empty());
    }

    #[test]
    fn test_detect_hotspots_empty_frame() {
        let temps: Vec<Vec<f64>> = Vec::new();
        let hotspots = detect_hotspots(&temps, 50.0);
        assert!(hotspots.is_empty());
    }

    #[test]
    fn test_detect_hotspots_entire_frame() {
        // When threshold is below all pixels, entire frame is one hotspot.
        let temps = vec![vec![60.0, 70.0], vec![65.0, 80.0]];
        let hotspots = detect_hotspots(&temps, 50.0);
        assert_eq!(hotspots.len(), 1);
        assert_eq!(hotspots[0].area_pixels, 4);
        assert!((hotspots[0].temp_c - 80.0).abs() < 1.0e-12);
    }

    // -----------------------------------------------------------------------
    // Atmospheric transmission
    // -----------------------------------------------------------------------

    #[test]
    fn test_atmospheric_transmission_zero_distance() {
        let t = atmospheric_transmission(0.0, 50.0, 25.0);
        assert!((t - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn test_atmospheric_transmission_short_range() {
        // At 10 m, attenuation should be very small.
        let t = atmospheric_transmission(10.0, 50.0, 25.0);
        assert!(t > 0.99, "short range transmission {} should be > 0.99", t);
    }

    #[test]
    fn test_atmospheric_transmission_decreases_with_distance() {
        let t1 = atmospheric_transmission(100.0, 50.0, 25.0);
        let t2 = atmospheric_transmission(1000.0, 50.0, 25.0);
        let t3 = atmospheric_transmission(5000.0, 50.0, 25.0);
        assert!(t2 < t1, "transmission should decrease with distance");
        assert!(t3 < t2, "transmission should decrease with distance");
    }

    #[test]
    fn test_atmospheric_transmission_decreases_with_humidity() {
        let t_dry = atmospheric_transmission(1000.0, 10.0, 25.0);
        let t_wet = atmospheric_transmission(1000.0, 90.0, 25.0);
        assert!(
            t_wet < t_dry,
            "higher humidity {} should reduce transmission vs {}",
            t_wet,
            t_dry
        );
    }

    #[test]
    fn test_atmospheric_transmission_bounds() {
        // Even at long range the result should stay in (0, 1].
        let t = atmospheric_transmission(100_000.0, 100.0, 40.0);
        assert!(t >= 0.0 && t <= 1.0, "transmission {} out of bounds", t);
    }

    // -----------------------------------------------------------------------
    // Stefan-Boltzmann
    // -----------------------------------------------------------------------

    #[test]
    fn test_stefan_boltzmann_basic() {
        // Blackbody at 300 K, 1 m^2, emissivity 1.0
        let p = stefan_boltzmann_power(300.0, 1.0, 1.0);
        let expected = STEFAN_BOLTZMANN_SIGMA * 300.0_f64.powi(4);
        assert!(
            (p - expected).abs() < 0.01,
            "SB power {} != expected {}",
            p,
            expected
        );
    }

    #[test]
    fn test_stefan_boltzmann_scales_with_emissivity() {
        let p1 = stefan_boltzmann_power(500.0, 1.0, 1.0);
        let p2 = stefan_boltzmann_power(500.0, 1.0, 0.5);
        assert!(
            (p2 - p1 * 0.5).abs() < 0.01,
            "emissivity scaling failed"
        );
    }

    #[test]
    fn test_stefan_boltzmann_zero_temp() {
        assert_eq!(stefan_boltzmann_power(0.0, 1.0, 1.0), 0.0);
    }

    #[test]
    fn test_stefan_boltzmann_scales_with_area() {
        let p1 = stefan_boltzmann_power(400.0, 1.0, 0.9);
        let p2 = stefan_boltzmann_power(400.0, 2.0, 0.9);
        assert!(
            (p2 - 2.0 * p1).abs() < 0.01,
            "area scaling failed: {} vs {}",
            p2,
            2.0 * p1
        );
    }

    // -----------------------------------------------------------------------
    // NETD
    // -----------------------------------------------------------------------

    #[test]
    fn test_netd_positive() {
        let result = netd(300.0, 10.0, 1.0e6);
        assert!(result > 0.0, "NETD should be positive");
    }

    #[test]
    fn test_netd_lower_noise_gives_lower_netd() {
        let n1 = netd(300.0, 10.0, 1.0e6);
        let n2 = netd(300.0, 5.0, 1.0e6);
        assert!(n2 < n1, "lower noise should give lower NETD");
    }

    #[test]
    fn test_netd_higher_responsivity_gives_lower_netd() {
        let n1 = netd(300.0, 10.0, 1.0e6);
        let n2 = netd(300.0, 10.0, 2.0e6);
        assert!(n2 < n1, "higher responsivity should give lower NETD");
    }

    #[test]
    fn test_netd_zero_noise() {
        assert_eq!(netd(300.0, 0.0, 1.0e6), 0.0);
    }

    #[test]
    fn test_netd_zero_responsivity() {
        assert_eq!(netd(300.0, 10.0, 0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // ThermalProcessor integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_basic() {
        let config = ThermalConfig {
            sensor_rows: 3,
            sensor_cols: 3,
            pixel_pitch_um: 17.0,
            integration_time_us: 16000.0,
            ambient_temp_c: 25.0,
            emissivity: 0.95,
        };
        let processor = ThermalProcessor::new(config);
        let raw = vec![vec![3000u16; 3]; 3];
        let frame = processor.process_frame(&raw);
        assert_eq!(frame.temperature_c.len(), 3);
        assert_eq!(frame.temperature_c[0].len(), 3);
        // All pixels identical -> min == max == mean
        assert!((frame.min_temp - frame.max_temp).abs() < 1.0e-10);
        assert!((frame.mean_temp - frame.min_temp).abs() < 1.0e-10);
    }

    #[test]
    #[should_panic(expected = "row count mismatch")]
    fn test_processor_wrong_rows() {
        let config = ThermalConfig {
            sensor_rows: 2,
            sensor_cols: 2,
            ..ThermalConfig::default()
        };
        let processor = ThermalProcessor::new(config);
        let raw = vec![vec![1000u16; 2]]; // only 1 row
        processor.process_frame(&raw);
    }

    #[test]
    fn test_processor_hotspot_detection() {
        let config = ThermalConfig {
            sensor_rows: 3,
            sensor_cols: 3,
            pixel_pitch_um: 17.0,
            integration_time_us: 16000.0,
            ambient_temp_c: 25.0,
            emissivity: 1.0,
        };
        // Use calibration that maps counts directly to known radiance values.
        // Place a much higher count at centre pixel.
        let processor = ThermalProcessor::new(config).with_hotspot_threshold(5.0);
        let mut raw = vec![vec![3000u16; 3]; 3];
        raw[1][1] = 60000; // significantly hotter
        let frame = processor.process_frame(&raw);
        assert!(
            frame.max_temp > frame.min_temp,
            "centre pixel should be hotter"
        );
        // The hotspot should be detected if the centre pixel is well above
        // the mean by more than the threshold.
        if frame.max_temp - frame.mean_temp > 5.0 {
            assert!(!frame.hotspots.is_empty(), "should detect hotspot");
        }
    }

    #[test]
    fn test_processor_with_calibration() {
        let config = ThermalConfig {
            sensor_rows: 1,
            sensor_cols: 1,
            pixel_pitch_um: 17.0,
            integration_time_us: 16000.0,
            ambient_temp_c: 25.0,
            emissivity: 1.0,
        };
        let p1 = ThermalProcessor::new(config.clone());
        let p2 = p1.clone().with_calibration(2.0e-6, 0.0);
        let raw = vec![vec![5000u16]];
        let f1 = p1.process_frame(&raw);
        let f2 = p2.process_frame(&raw);
        // Different calibration should give different temperatures
        assert!(
            (f1.mean_temp - f2.mean_temp).abs() > 0.01,
            "calibration should affect temperature: {} vs {}",
            f1.mean_temp,
            f2.mean_temp
        );
    }

    #[test]
    fn test_processor_default_config() {
        let config = ThermalConfig::default();
        assert_eq!(config.sensor_rows, 240);
        assert_eq!(config.sensor_cols, 320);
        assert!((config.emissivity - 0.95).abs() < 1.0e-12);
    }

    #[test]
    fn test_emissivity_correction() {
        // With emissivity = 1.0, measured radiance should pass through unchanged.
        let wl = 10.0;
        let r = planck_temp_to_radiance(350.0, wl);
        let corrected = emissivity_correct(r, 1.0, 25.0, wl);
        assert!(
            (corrected - r).abs() < 1.0e-15,
            "emissivity=1 should not change radiance"
        );
    }

    #[test]
    fn test_emissivity_correction_graybody() {
        // For a grey body, corrected radiance should be higher than measured
        // (because some of the measured signal is reflected ambient).
        let wl = 10.0;
        let target_k = 350.0;
        let ambient_c = 25.0;
        let eps = 0.8;
        let l_target = planck_temp_to_radiance(target_k, wl);
        let l_ambient = planck_temp_to_radiance(ambient_c + 273.15, wl);
        let l_measured = eps * l_target + (1.0 - eps) * l_ambient;
        let corrected = emissivity_correct(l_measured, eps, ambient_c, wl);
        assert!(
            (corrected - l_target).abs() / l_target < 0.001,
            "emissivity correction should recover target radiance: {} vs {}",
            corrected,
            l_target
        );
    }
}
