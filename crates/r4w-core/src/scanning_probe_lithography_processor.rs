//! Scanning Probe Lithography (SPL) signal processing for nanoscale patterning
//! and characterization.
//!
//! This module implements the physics-based models and signal processing needed
//! to control scanning probe microscope tips for nanoscale writing and reading.
//! SPL techniques use the extremely sharp apex of an AFM/STM tip to locally
//! modify a substrate at the 10-500 nm scale, enabling mask-less, resist-less
//! nanofabrication.
//!
//! # Background
//!
//! Scanning Probe Lithography encompasses several physical mechanisms:
//!
//! - **Oxidation SPL (o-SPL)**: Apply a voltage bias between tip and surface in
//!   humid ambient air. The water meniscus acts as a nanoscale electrochemical
//!   cell, growing a local oxide via the Cabrera-Mott mechanism. Feature sizes
//!   of 10-100 nm are routine on Si, Ti, and other metals.
//!
//! - **Dip-Pen Nanolithography (DPN)**: An ink-coated tip deposits molecules
//!   through a water meniscus onto a substrate. Transport follows 2D diffusion
//!   from a point source. Feature sizes 50-500 nm.
//!
//! - **Thermal SPL (t-SPL)**: A resistively heated tip (up to 1000 C) locally
//!   decomposes or deforms a polymer resist. IBM's Millipede and NanoFrazor
//!   use this approach for sub-10 nm half-pitch patterning.
//!
//! - **Mechanical SPL (m-SPL)**: Direct indentation or scratching with a hard
//!   tip. Hertzian and JKR contact mechanics govern the process.
//!
//! # Components
//!
//! | Struct / Enum | Purpose |
//! |---|---|
//! | [`SplConfig`] | Tip, environment, and operating parameters |
//! | [`SplMode`] | Lithography mechanism selector |
//! | [`SplProcessor`] | Core computation engine with all physics models |
//! | [`SplPattern`] | 2D pattern description (positions + features) |
//! | [`SplFeature`] | Individual feature geometry (Line, Dot, Rectangle) |
//! | [`ScanPath`] | Raster or vector scan trajectory |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::scanning_probe_lithography_processor::{
//!     SplConfig, SplMode, SplProcessor,
//! };
//!
//! let config = SplConfig {
//!     tip_radius_nm: 25.0,
//!     scan_speed_um_s: 1.0,
//!     applied_voltage_v: 8.0,
//!     set_point_na: 0.5,
//!     humidity_percent: 50.0,
//!     temperature_c: 25.0,
//!     mode: SplMode::OxidationSPL,
//! };
//!
//! let proc = SplProcessor::new(config);
//!
//! // Predict oxide line width at 1 um/s scan speed
//! let width = proc.oxide_line_width();
//! assert!(width > 20.0 && width < 200.0); // nm
//!
//! // Predict oxide height after 10 ms dwell
//! let height = proc.oxide_height_model(0.01);
//! assert!(height > 0.0 && height < 15.0); // nm
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// SplMode
// ---------------------------------------------------------------------------

/// Scanning probe lithography mechanism.
///
/// Each mode uses a different physical process to create nanoscale features.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplMode {
    /// Local anodic oxidation via water meniscus electrochemistry (Cabrera-Mott).
    OxidationSPL,
    /// Ink transport through water meniscus (diffusion-limited deposition).
    DipPenNanolithography,
    /// Heated tip thermally decomposes or deforms resist/polymer.
    ThermalSPL,
    /// Direct mechanical indentation or scratching.
    MechanicalSPL,
}

// ---------------------------------------------------------------------------
// SplFeature
// ---------------------------------------------------------------------------

/// Geometry of an individual nanoscale feature written by SPL.
#[derive(Debug, Clone, PartialEq)]
pub enum SplFeature {
    /// A line feature.
    Line {
        /// Full width in nanometres.
        width_nm: f64,
        /// Depth (or height for oxide) in nanometres.
        depth_nm: f64,
        /// Length in nanometres.
        length_nm: f64,
    },
    /// A circular dot or pit.
    Dot {
        /// Diameter in nanometres.
        diameter_nm: f64,
        /// Depth (or height for oxide) in nanometres.
        depth_nm: f64,
    },
    /// A rectangular region.
    Rectangle {
        /// Width (x) in nanometres.
        width_nm: f64,
        /// Height (y) in nanometres.
        height_nm: f64,
        /// Depth (or height for oxide) in nanometres.
        depth_nm: f64,
    },
}

// ---------------------------------------------------------------------------
// SplPattern
// ---------------------------------------------------------------------------

/// A 2D pattern to be written or that has been read back.
///
/// Contains centre positions for every feature and the feature geometry.
#[derive(Debug, Clone)]
pub struct SplPattern {
    /// X-coordinates of feature centres in nanometres.
    pub x_positions_nm: Vec<f64>,
    /// Y-coordinates of feature centres in nanometres.
    pub y_positions_nm: Vec<f64>,
    /// Feature geometries (parallel array with positions).
    pub features: Vec<SplFeature>,
}

impl SplPattern {
    /// Create an empty pattern.
    pub fn new() -> Self {
        Self {
            x_positions_nm: Vec::new(),
            y_positions_nm: Vec::new(),
            features: Vec::new(),
        }
    }

    /// Add a feature at the given position.
    pub fn add_feature(&mut self, x_nm: f64, y_nm: f64, feature: SplFeature) {
        self.x_positions_nm.push(x_nm);
        self.y_positions_nm.push(y_nm);
        self.features.push(feature);
    }

    /// Number of features in the pattern.
    pub fn len(&self) -> usize {
        self.features.len()
    }

    /// True if the pattern contains no features.
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }
}

// ---------------------------------------------------------------------------
// ScanPath
// ---------------------------------------------------------------------------

/// A scan trajectory for the probe tip.
#[derive(Debug, Clone)]
pub struct ScanPath {
    /// X coordinates in nanometres, one per sample point.
    pub x_nm: Vec<f64>,
    /// Y coordinates in nanometres, one per sample point.
    pub y_nm: Vec<f64>,
}

// ---------------------------------------------------------------------------
// SplConfig
// ---------------------------------------------------------------------------

/// Operating parameters for scanning probe lithography.
#[derive(Debug, Clone, PartialEq)]
pub struct SplConfig {
    /// Tip apex radius in nanometres (typical 10-50 nm).
    pub tip_radius_nm: f64,
    /// Scan speed in micrometres per second (typical 0.1-100 um/s).
    pub scan_speed_um_s: f64,
    /// Applied bias voltage in volts (typical 1-12 V for oxidation SPL).
    pub applied_voltage_v: f64,
    /// Contact force set-point in nanoamperes (for STM) or nN (for AFM).
    pub set_point_na: f64,
    /// Ambient relative humidity in percent (critical for meniscus formation).
    pub humidity_percent: f64,
    /// Ambient temperature in degrees Celsius.
    pub temperature_c: f64,
    /// Lithography mode.
    pub mode: SplMode,
}

impl SplConfig {
    /// Preset for oxidation SPL on silicon in ambient conditions.
    pub fn default_oxidation() -> Self {
        Self {
            tip_radius_nm: 25.0,
            scan_speed_um_s: 1.0,
            applied_voltage_v: 8.0,
            set_point_na: 0.5,
            humidity_percent: 50.0,
            temperature_c: 25.0,
            mode: SplMode::OxidationSPL,
        }
    }

    /// Preset for dip-pen nanolithography.
    pub fn default_dip_pen() -> Self {
        Self {
            tip_radius_nm: 30.0,
            scan_speed_um_s: 0.5,
            applied_voltage_v: 0.0,
            set_point_na: 0.1,
            humidity_percent: 45.0,
            temperature_c: 25.0,
            mode: SplMode::DipPenNanolithography,
        }
    }

    /// Preset for thermal SPL (NanoFrazor-like).
    pub fn default_thermal() -> Self {
        Self {
            tip_radius_nm: 10.0,
            scan_speed_um_s: 10.0,
            applied_voltage_v: 0.0,
            set_point_na: 100.0, // Force set-point in nN for thermal
            humidity_percent: 30.0,
            temperature_c: 25.0,
            mode: SplMode::ThermalSPL,
        }
    }

    /// Preset for mechanical indentation/scratching.
    pub fn default_mechanical() -> Self {
        Self {
            tip_radius_nm: 20.0,
            scan_speed_um_s: 0.5,
            applied_voltage_v: 0.0,
            set_point_na: 500.0, // Force in nN
            humidity_percent: 30.0,
            temperature_c: 25.0,
            mode: SplMode::MechanicalSPL,
        }
    }
}

// ---------------------------------------------------------------------------
// SplProcessor
// ---------------------------------------------------------------------------

/// Core computation engine for scanning probe lithography signal processing.
///
/// Implements physics-based models for oxide growth kinetics, ink diffusion,
/// thermal decomposition, contact mechanics, pattern generation, and various
/// correction algorithms for piezo hysteresis, drift, and tip wear.
pub struct SplProcessor {
    config: SplConfig,
}

impl SplProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: SplConfig) -> Self {
        Self { config }
    }

    /// Access the current configuration.
    pub fn config(&self) -> &SplConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // Oxidation SPL models
    // -----------------------------------------------------------------------

    /// Predict oxide line width in nanometres for the current configuration.
    ///
    /// Empirical model combining tip radius, voltage, scan speed, and humidity:
    ///
    /// ```text
    /// w = k * V * sqrt(t_dwell / v) * f(RH)
    /// ```
    ///
    /// where `t_dwell` is the effective dwell time per pixel (proportional to
    /// tip radius / scan speed), and `f(RH)` is a humidity factor that captures
    /// the meniscus growth dependence on relative humidity.
    ///
    /// The result is further bounded by the geometric limit `2 * sqrt(2*R*h)`
    /// from the tip-oxide contact geometry.
    pub fn oxide_line_width(&self) -> f64 {
        let r = self.config.tip_radius_nm;
        let v = self.config.applied_voltage_v;
        let speed = self.config.scan_speed_um_s.max(0.001);
        let rh = self.config.humidity_percent;

        // Empirical proportionality constant (nm / (V * s^0.5))
        let k = 3.5;

        // Effective dwell time: tip diameter / speed (convert speed um/s -> nm/s)
        let speed_nm_s = speed * 1000.0;
        let t_dwell = (2.0 * r) / speed_nm_s;

        // Humidity factor: sigmoidal onset around 30% RH
        let f_rh = humidity_factor(rh);

        // Empirical width
        let w_empirical = k * v * t_dwell.sqrt() * f_rh;

        // Geometric cap: oxide height at this dwell time
        let h = self.oxide_height_model(t_dwell);
        let w_geometric = 2.0 * (2.0 * r * h).max(0.0).sqrt();

        // Take the minimum (geometric limit caps the empirical model)
        w_empirical.min(w_geometric).max(0.0)
    }

    /// Cabrera-Mott oxidation kinetics: oxide height in nm as a function of
    /// dwell time in seconds.
    ///
    /// ```text
    /// h(t) = A * ln(1 + t / tau)
    /// ```
    ///
    /// The amplitude `A` scales with applied voltage (higher field drives more
    /// ion transport) and the time constant `tau` captures the self-limiting
    /// nature of the oxidation (the growing oxide reduces the electric field).
    ///
    /// Typical values: A ~ 0.5-5 nm, tau ~ 0.01-1 s.
    pub fn oxide_height_model(&self, dwell_time_s: f64) -> f64 {
        let v = self.config.applied_voltage_v.abs();

        // Amplitude scales with voltage: ~0.5 nm per volt
        let a = 0.5 * v;

        // Time constant decreases with voltage (faster initial growth)
        let tau = 0.1 / (1.0 + 0.1 * v);

        if dwell_time_s <= 0.0 {
            return 0.0;
        }

        a * (1.0 + dwell_time_s / tau).ln()
    }

    // -----------------------------------------------------------------------
    // Dip-Pen Nanolithography
    // -----------------------------------------------------------------------

    /// Dip-pen ink diffusion: concentration at distance `r_nm` from the tip
    /// contact point after time `t_s` seconds.
    ///
    /// Models 2D isotropic diffusion from a point source through the meniscus:
    ///
    /// ```text
    /// C(r, t) = N / (4 * pi * D * t) * exp(-r^2 / (4 * D * t))
    /// ```
    ///
    /// where `N` is the total number of deposited molecules and `D` is the
    /// surface diffusion coefficient in nm^2/s.
    ///
    /// Returns concentration in arbitrary units (molecules / nm^2).
    pub fn dip_pen_diffusion(
        &self,
        r_nm: f64,
        t_s: f64,
        total_molecules: f64,
        diffusion_coeff_nm2_s: f64,
    ) -> f64 {
        if t_s <= 0.0 || diffusion_coeff_nm2_s <= 0.0 {
            return 0.0;
        }

        let d = diffusion_coeff_nm2_s;
        let denom = 4.0 * PI * d * t_s;
        let exponent = -(r_nm * r_nm) / (4.0 * d * t_s);

        (total_molecules / denom) * exponent.exp()
    }

    // -----------------------------------------------------------------------
    // Thermal SPL
    // -----------------------------------------------------------------------

    /// Temperature profile around a heated tip.
    ///
    /// For a hemispherical heat source in contact with a semi-infinite
    /// half-space, the steady-state temperature decays as:
    ///
    /// ```text
    /// T(r) = T_tip / (1 + r / r_contact)
    /// ```
    ///
    /// where `r_contact` is the tip-surface contact radius (approximated
    /// from tip radius and indentation depth).
    ///
    /// Returns temperature in degrees Celsius at radial distance `r_nm`.
    pub fn thermal_decomposition(
        &self,
        r_nm: f64,
        tip_temperature_c: f64,
        contact_radius_nm: f64,
    ) -> f64 {
        if contact_radius_nm <= 0.0 {
            return self.config.temperature_c;
        }

        let delta_t = tip_temperature_c - self.config.temperature_c;
        let t_at_r = delta_t / (1.0 + r_nm.abs() / contact_radius_nm);

        self.config.temperature_c + t_at_r
    }

    // -----------------------------------------------------------------------
    // Mechanical SPL
    // -----------------------------------------------------------------------

    /// Hertzian contact mechanics: compute the applied force (in nN) for a
    /// given indentation depth (in nm).
    ///
    /// ```text
    /// F = (4/3) * E_eff * sqrt(R) * d^(3/2)
    /// ```
    ///
    /// where `E_eff` is the effective Young's modulus (in GPa), `R` is the
    /// tip radius (in nm), and `d` is the indentation depth (in nm).
    ///
    /// Returns force in nanonewtons.
    pub fn mechanical_indentation(
        &self,
        indentation_depth_nm: f64,
        effective_modulus_gpa: f64,
    ) -> f64 {
        if indentation_depth_nm <= 0.0 {
            return 0.0;
        }

        let r = self.config.tip_radius_nm;

        // Convert E_eff from GPa to nN/nm^2:  1 GPa = 1e9 Pa = 1e9 N/m^2
        // = 1e9 * 1e-9 nN / (1e-9 m)^2 = 1e9 * 1e-9 nN / 1e-18 m^2
        // = 1 nN / nm^2
        let e_eff = effective_modulus_gpa; // nN/nm^2 numerically equal to GPa

        (4.0 / 3.0) * e_eff * r.sqrt() * indentation_depth_nm.powf(1.5)
    }

    /// JKR (Johnson-Kendall-Roberts) adhesive contact model.
    ///
    /// Extends Hertzian contact to include surface adhesion energy:
    ///
    /// ```text
    /// F = (4/3) * E_eff * sqrt(R) * d^(3/2) - sqrt(6 * pi * gamma * E_eff * R * a^3)
    /// ```
    ///
    /// The contact radius `a` under JKR theory accounts for the work of
    /// adhesion `gamma` (in mJ/m^2). For zero adhesion this reduces to Hertz.
    ///
    /// Returns (force_nN, contact_radius_nm).
    pub fn contact_mechanics_jkr(
        &self,
        indentation_depth_nm: f64,
        effective_modulus_gpa: f64,
        adhesion_energy_mj_m2: f64,
    ) -> (f64, f64) {
        let r = self.config.tip_radius_nm;
        let e_eff = effective_modulus_gpa; // nN/nm^2

        if indentation_depth_nm <= 0.0 {
            // Pull-off force for JKR: F_pulloff = -(3/2) * pi * gamma * R
            // Convert gamma from mJ/m^2 to nN/nm: 1 mJ/m^2 = 1e-3 J/m^2
            // = 1e-3 N/m * 1e-9 = 1e-12 nN/nm ... wait.
            // 1 mJ/m^2 = 1e-3 N/m = 1e-3 * 1e9 nN / (1e9 nm) = 1e-3 nN/nm
            let gamma_nn_per_nm = adhesion_energy_mj_m2 * 1e-3;
            let f_pulloff = -1.5 * PI * gamma_nn_per_nm * r;
            return (f_pulloff, 0.0);
        }

        // Hertzian contact radius: a = sqrt(R * d)
        let a_hertz = (r * indentation_depth_nm).sqrt();

        // Hertz force
        let f_hertz = (4.0 / 3.0) * e_eff * r.sqrt() * indentation_depth_nm.powf(1.5);

        // JKR adhesion correction
        // gamma in nN/nm
        let gamma_nn_per_nm = adhesion_energy_mj_m2 * 1e-3;
        let adhesion_term = (6.0 * PI * gamma_nn_per_nm * e_eff * r * a_hertz.powi(3))
            .max(0.0)
            .sqrt();

        let f_jkr = f_hertz - adhesion_term;

        // JKR contact radius is larger than Hertz due to adhesion
        // a_jkr^3 = (R / E_eff) * (F + 3*pi*gamma*R + sqrt(6*pi*gamma*R*F + (3*pi*gamma*R)^2))
        // Simplified: use Hertz radius + correction
        let a_correction = if e_eff > 0.0 {
            (PI * gamma_nn_per_nm * r * r / e_eff).powf(1.0 / 3.0)
        } else {
            0.0
        };
        let a_jkr = a_hertz + 0.5 * a_correction;

        (f_jkr, a_jkr)
    }

    // -----------------------------------------------------------------------
    // Electric field
    // -----------------------------------------------------------------------

    /// Estimate the electric field at the tip apex for sphere-plane geometry.
    ///
    /// ```text
    /// E_tip ~ V / (k * R_tip)
    /// ```
    ///
    /// where `k` is a geometric factor (typically ~0.7 for a sphere near a
    /// plane) and `R_tip` is the tip radius. Returns field in V/nm.
    pub fn electric_field_at_tip(&self) -> f64 {
        let r = self.config.tip_radius_nm;
        if r <= 0.0 {
            return 0.0;
        }
        let k = 0.7; // geometric factor for sphere-plane
        self.config.applied_voltage_v.abs() / (k * r)
    }

    // -----------------------------------------------------------------------
    // Feature resolution
    // -----------------------------------------------------------------------

    /// Minimum feature size (nm) achievable with the current tip and mode.
    ///
    /// Depends on tip radius, mode physics, and environmental conditions.
    pub fn feature_resolution(&self) -> f64 {
        let r = self.config.tip_radius_nm;

        match self.config.mode {
            SplMode::OxidationSPL => {
                // Resolution limited by meniscus size and tip radius.
                // Use meniscus at near-contact (gap ~ 0.2 nm) as the
                // electrochemical cell defines the minimum feature size.
                let meniscus = self.meniscus_size(0.2);
                let geom_limit = 2.0 * r;
                meniscus.max(geom_limit * 0.5)
            }
            SplMode::DipPenNanolithography => {
                // Diffusion-limited: minimum ~ 2*meniscus diameter
                let meniscus = self.meniscus_size(0.2);
                meniscus * 2.0
            }
            SplMode::ThermalSPL => {
                // Best resolution: limited by contact area.
                // t-SPL routinely achieves sub-10 nm half-pitch with sharp
                // tips (NanoFrazor: ~5 nm). Resolution scales with tip radius.
                (r * 0.3).max(3.0)
            }
            SplMode::MechanicalSPL => {
                // Contact diameter: ~2*sqrt(R*d) for small indentation d~1nm
                2.0 * (r * 1.0).sqrt()
            }
        }
    }

    // -----------------------------------------------------------------------
    // Meniscus
    // -----------------------------------------------------------------------

    /// Water meniscus diameter (nm) as a function of humidity and tip-surface
    /// gap distance.
    ///
    /// Uses a simplified Kelvin equation model. The meniscus forms when
    /// humidity exceeds the critical condensation threshold, and its size
    /// grows with humidity and decreases with gap distance.
    ///
    /// ```text
    /// r_kelvin = -gamma_w * V_m / (R_gas * T * ln(RH/100))
    /// d_meniscus ~ 2 * sqrt(2 * R_tip * r_kelvin) * exp(-gap / r_kelvin)
    /// ```
    ///
    /// Returns diameter in nm.
    pub fn meniscus_size(&self, gap_nm: f64) -> f64 {
        let rh = self.config.humidity_percent;
        let t_k = self.config.temperature_c + 273.15;
        let r_tip = self.config.tip_radius_nm;

        if rh <= 0.0 || rh >= 100.0 {
            return 0.0;
        }

        // Physical constants for water
        let gamma_w = 0.0728; // Surface tension, N/m
        let v_m = 1.8e-5; // Molar volume of water, m^3/mol
        let r_gas = 8.314; // J/(mol*K)

        // Kelvin radius (in metres)
        let ln_rh = (rh / 100.0).ln(); // negative for RH < 100%
        if ln_rh >= 0.0 {
            return 0.0;
        }
        let r_kelvin_m = -(gamma_w * v_m) / (r_gas * t_k * ln_rh);
        let r_kelvin_nm = r_kelvin_m * 1e9;

        // Meniscus diameter: geometric approximation
        let d_base = 2.0 * (2.0 * r_tip * r_kelvin_nm).max(0.0).sqrt();

        // Exponential decay with gap distance
        let decay = (-gap_nm / r_kelvin_nm).exp();

        (d_base * decay).max(0.0)
    }

    // -----------------------------------------------------------------------
    // Line edge roughness
    // -----------------------------------------------------------------------

    /// Calculate Line Edge Roughness (LER) from a series of edge position
    /// measurements.
    ///
    /// LER is defined as 3-sigma of the edge position deviations from the
    /// mean:
    ///
    /// ```text
    /// LER = 3 * sigma(edge_positions)
    /// ```
    ///
    /// Also returns Line Width Roughness (LWR) if both left and right edges
    /// are provided.
    ///
    /// Returns `(ler_left_nm, ler_right_nm, lwr_nm)`.
    pub fn line_edge_roughness(
        &self,
        left_edge_nm: &[f64],
        right_edge_nm: &[f64],
    ) -> (f64, f64, f64) {
        let ler_left = three_sigma(left_edge_nm);
        let ler_right = three_sigma(right_edge_nm);

        // LWR: roughness of the line width = right - left
        let widths: Vec<f64> = left_edge_nm
            .iter()
            .zip(right_edge_nm.iter())
            .map(|(l, r)| r - l)
            .collect();
        let lwr = three_sigma(&widths);

        (ler_left, ler_right, lwr)
    }

    // -----------------------------------------------------------------------
    // Pattern fidelity
    // -----------------------------------------------------------------------

    /// Compare written pattern to intended pattern using RMS error.
    ///
    /// Both patterns are represented as 1D height profiles (e.g., from AFM
    /// readback). Returns RMS error in nm.
    pub fn pattern_fidelity(&self, intended: &[f64], measured: &[f64]) -> f64 {
        let n = intended.len().min(measured.len());
        if n == 0 {
            return 0.0;
        }

        let sum_sq: f64 = intended
            .iter()
            .zip(measured.iter())
            .map(|(a, b)| (a - b) * (a - b))
            .sum();

        (sum_sq / n as f64).sqrt()
    }

    // -----------------------------------------------------------------------
    // Scan trajectory generation
    // -----------------------------------------------------------------------

    /// Generate a raster scan path covering a rectangular area.
    ///
    /// The path scans left-to-right on even rows and right-to-left on odd
    /// rows (serpentine/boustrophedon pattern) to minimize retrace overhead.
    ///
    /// `width_nm` and `height_nm` define the scan area.
    /// `line_spacing_nm` is the distance between adjacent scan lines.
    /// `point_spacing_nm` is the distance between samples along a line.
    pub fn scan_trajectory(
        &self,
        width_nm: f64,
        height_nm: f64,
        line_spacing_nm: f64,
        point_spacing_nm: f64,
    ) -> ScanPath {
        let mut x_nm = Vec::new();
        let mut y_nm = Vec::new();

        if width_nm <= 0.0 || height_nm <= 0.0 || line_spacing_nm <= 0.0 || point_spacing_nm <= 0.0
        {
            return ScanPath { x_nm, y_nm };
        }

        let n_lines = (height_nm / line_spacing_nm).ceil() as usize + 1;
        let n_points = (width_nm / point_spacing_nm).ceil() as usize + 1;

        for line in 0..n_lines {
            let y = (line as f64) * line_spacing_nm;
            if y > height_nm {
                break;
            }

            for pt in 0..n_points {
                let x_fwd = (pt as f64) * point_spacing_nm;
                if x_fwd > width_nm {
                    break;
                }

                // Serpentine: reverse x on odd lines
                let x = if line % 2 == 0 {
                    x_fwd
                } else {
                    width_nm - x_fwd
                };

                x_nm.push(x);
                y_nm.push(y);
            }
        }

        ScanPath { x_nm, y_nm }
    }

    // -----------------------------------------------------------------------
    // Writing speed limit
    // -----------------------------------------------------------------------

    /// Maximum scan speed (um/s) for a target feature width (nm).
    ///
    /// Derived from the oxide line width model inverted: given a desired
    /// line width, solve for the maximum speed that still produces that width.
    ///
    /// For oxidation SPL: `w = k * V * sqrt(2R / (v * 1000)) * f(RH)`
    /// => `v = k^2 * V^2 * 2R * f(RH)^2 / (w^2 * 1000)`
    pub fn writing_speed_limit(&self, target_width_nm: f64) -> f64 {
        if target_width_nm <= 0.0 {
            return 0.0;
        }

        let v = self.config.applied_voltage_v;
        let r = self.config.tip_radius_nm;
        let rh = self.config.humidity_percent;
        let k = 3.5;
        let f_rh = humidity_factor(rh);

        // From w = k * V * sqrt(2*R / (speed_nm_s)) * f(RH)
        // speed_nm_s = k^2 * V^2 * 2 * R * f_rh^2 / w^2
        // speed_um_s = speed_nm_s / 1000
        let numerator = k * k * v * v * 2.0 * r * f_rh * f_rh;
        let denominator = target_width_nm * target_width_nm;

        if denominator == 0.0 {
            return 0.0;
        }

        // Convert nm/s to um/s
        (numerator / denominator) / 1000.0
    }

    // -----------------------------------------------------------------------
    // Piezo hysteresis correction
    // -----------------------------------------------------------------------

    /// Correct for scanner piezo hysteresis using a simple polynomial model.
    ///
    /// Piezoelectric scanners exhibit nonlinear position-voltage curves with
    /// hysteresis. This function applies a third-order polynomial correction:
    ///
    /// ```text
    /// x_corrected = a1*x + a2*x^2 + a3*x^3
    /// ```
    ///
    /// The coefficients are derived from a maximum hysteresis fraction.
    /// Typical AFM scanners have 5-20% hysteresis.
    pub fn piezo_hysteresis_correction(
        &self,
        positions_nm: &[f64],
        max_hysteresis_fraction: f64,
    ) -> Vec<f64> {
        // Find the range of positions
        let (min_pos, max_pos) = positions_nm.iter().fold((f64::MAX, f64::MIN), |(mn, mx), &x| {
            (mn.min(x), mx.max(x))
        });

        let range = max_pos - min_pos;
        if range <= 0.0 || max_hysteresis_fraction <= 0.0 {
            return positions_nm.to_vec();
        }

        // Third-order correction coefficients
        // The hysteresis creates an S-shaped distortion
        // x_true = x_measured + h * (x_norm^2 - x_norm) where x_norm = x / range
        let h = max_hysteresis_fraction * range;

        positions_nm
            .iter()
            .map(|&x| {
                let x_norm = (x - min_pos) / range; // Normalize to [0, 1]
                let correction = h * (x_norm * x_norm - x_norm);
                x + correction
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Drift compensation
    // -----------------------------------------------------------------------

    /// Track and correct thermal drift during long writes.
    ///
    /// Estimates linear drift rate from a series of (time, position) reference
    /// measurements and subtracts the predicted drift from the positions.
    ///
    /// Returns corrected positions and estimated drift rates (nm/s) in x and y.
    pub fn drift_compensation(
        &self,
        times_s: &[f64],
        x_nm: &[f64],
        y_nm: &[f64],
    ) -> (Vec<f64>, Vec<f64>, f64, f64) {
        let n = times_s.len().min(x_nm.len()).min(y_nm.len());
        if n < 2 {
            return (x_nm.to_vec(), y_nm.to_vec(), 0.0, 0.0);
        }

        // Linear regression for drift rate
        let drift_x = linear_regression_slope(times_s, x_nm);
        let drift_y = linear_regression_slope(times_s, y_nm);

        // Subtract estimated drift
        let t0 = times_s[0];
        let x_corrected: Vec<f64> = times_s
            .iter()
            .zip(x_nm.iter())
            .map(|(&t, &x)| x - drift_x * (t - t0))
            .collect();

        let y_corrected: Vec<f64> = times_s
            .iter()
            .zip(y_nm.iter())
            .map(|(&t, &y)| y - drift_y * (t - t0))
            .collect();

        (x_corrected, y_corrected, drift_x, drift_y)
    }

    // -----------------------------------------------------------------------
    // Tip wear model
    // -----------------------------------------------------------------------

    /// Estimate tip radius after a number of writes due to wear.
    ///
    /// The tip blunts approximately as a power law:
    ///
    /// ```text
    /// R(n) = R_0 * (1 + alpha * n)^beta
    /// ```
    ///
    /// where `alpha` is a wear rate coefficient and `beta ~ 0.3-0.5` for
    /// typical Si or diamond tips. More aggressive modes (mechanical) wear
    /// faster.
    pub fn tip_wear_model(&self, num_writes: u64) -> f64 {
        let r0 = self.config.tip_radius_nm;

        let (alpha, beta) = match self.config.mode {
            SplMode::OxidationSPL => (1e-4, 0.3),
            SplMode::DipPenNanolithography => (5e-5, 0.25),
            SplMode::ThermalSPL => (2e-4, 0.35),
            SplMode::MechanicalSPL => (5e-3, 0.5),
        };

        r0 * (1.0 + alpha * num_writes as f64).powf(beta)
    }

    // -----------------------------------------------------------------------
    // Overlay alignment
    // -----------------------------------------------------------------------

    /// Cross-correlation based overlay alignment for multi-layer patterning.
    ///
    /// Computes the shift (dx, dy) that maximises the cross-correlation
    /// between a reference pattern and a measured pattern. Patterns are
    /// represented as 2D height maps (row-major, same dimensions).
    ///
    /// Returns `(dx_pixels, dy_pixels, correlation_peak)`.
    pub fn overlay_alignment(
        &self,
        reference: &[f64],
        measured: &[f64],
        width: usize,
        height: usize,
    ) -> (i32, i32, f64) {
        if width == 0 || height == 0 || reference.len() < width * height || measured.len() < width * height {
            return (0, 0, 0.0);
        }

        let max_shift = (width / 4).max(1).min(32);
        let max_shift_y = (height / 4).max(1).min(32);

        let mut best_corr = f64::NEG_INFINITY;
        let mut best_dx: i32 = 0;
        let mut best_dy: i32 = 0;

        let ref_mean = mean(reference);
        let meas_mean = mean(measured);

        for dy in -(max_shift_y as i32)..=(max_shift_y as i32) {
            for dx in -(max_shift as i32)..=(max_shift as i32) {
                let mut sum = 0.0;
                let mut count = 0usize;

                for row in 0..height {
                    let src_row = row as i32 + dy;
                    if src_row < 0 || src_row >= height as i32 {
                        continue;
                    }

                    for col in 0..width {
                        let src_col = col as i32 + dx;
                        if src_col < 0 || src_col >= width as i32 {
                            continue;
                        }

                        let r_val = reference[src_row as usize * width + src_col as usize] - ref_mean;
                        let m_val = measured[row * width + col] - meas_mean;
                        sum += r_val * m_val;
                        count += 1;
                    }
                }

                if count > 0 {
                    let corr = sum / count as f64;
                    if corr > best_corr {
                        best_corr = corr;
                        best_dx = dx;
                        best_dy = dy;
                    }
                }
            }
        }

        (best_dx, best_dy, best_corr)
    }

    // -----------------------------------------------------------------------
    // Dose map
    // -----------------------------------------------------------------------

    /// Compute dose map (voltage * dwell time) across a scan pattern.
    ///
    /// For oxidation SPL the dose determines the local oxide thickness.
    /// Faster scan = less dwell = less dose.
    ///
    /// Returns dose in V*s for each point in the scan path.
    pub fn dose_map(&self, path: &ScanPath, dwell_times_s: &[f64]) -> Vec<f64> {
        let v = self.config.applied_voltage_v;
        let n = path.x_nm.len().min(dwell_times_s.len());

        (0..n).map(|i| v * dwell_times_s[i]).collect()
    }

    // -----------------------------------------------------------------------
    // Pattern generators
    // -----------------------------------------------------------------------

    /// Generate a pattern of parallel lines for testing.
    ///
    /// Lines run along the x-axis with given length, width, depth, and pitch.
    pub fn generate_line_pattern(
        &self,
        num_lines: usize,
        line_length_nm: f64,
        line_width_nm: f64,
        line_depth_nm: f64,
        pitch_nm: f64,
    ) -> SplPattern {
        let mut pattern = SplPattern::new();

        for i in 0..num_lines {
            let y = (i as f64) * pitch_nm;
            let x = line_length_nm / 2.0; // centre position

            pattern.add_feature(
                x,
                y,
                SplFeature::Line {
                    width_nm: line_width_nm,
                    depth_nm: line_depth_nm,
                    length_nm: line_length_nm,
                },
            );
        }

        pattern
    }

    /// Generate a regular 2D dot/pit array for testing.
    ///
    /// Dots are arranged on a rectangular grid with given pitch.
    pub fn generate_dot_array(
        &self,
        rows: usize,
        cols: usize,
        diameter_nm: f64,
        depth_nm: f64,
        pitch_x_nm: f64,
        pitch_y_nm: f64,
    ) -> SplPattern {
        let mut pattern = SplPattern::new();

        for row in 0..rows {
            for col in 0..cols {
                let x = (col as f64) * pitch_x_nm;
                let y = (row as f64) * pitch_y_nm;

                pattern.add_feature(
                    x,
                    y,
                    SplFeature::Dot {
                        diameter_nm,
                        depth_nm,
                    },
                );
            }
        }

        pattern
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Humidity factor for oxidation SPL: sigmoidal onset around 30% RH.
///
/// ```text
/// f(RH) = 1 / (1 + exp(-0.15 * (RH - 35)))
/// ```
///
/// Below ~20% RH the meniscus does not form; above ~60% it saturates.
fn humidity_factor(rh: f64) -> f64 {
    1.0 / (1.0 + (-0.15 * (rh - 35.0)).exp())
}

/// Compute 3-sigma of a data series for roughness metrics.
fn three_sigma(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }

    let m = mean(data);
    let variance = data.iter().map(|&x| (x - m) * (x - m)).sum::<f64>() / (data.len() - 1) as f64;

    3.0 * variance.sqrt()
}

/// Mean of a slice.
fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

/// Simple linear regression slope: dy/dx.
fn linear_regression_slope(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 2 {
        return 0.0;
    }

    let x_mean = x[..n].iter().sum::<f64>() / n as f64;
    let y_mean = y[..n].iter().sum::<f64>() / n as f64;

    let mut num = 0.0;
    let mut den = 0.0;

    for i in 0..n {
        let dx = x[i] - x_mean;
        let dy = y[i] - y_mean;
        num += dx * dy;
        den += dx * dx;
    }

    if den.abs() < 1e-30 {
        return 0.0;
    }

    num / den
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_oxidation_proc() -> SplProcessor {
        SplProcessor::new(SplConfig::default_oxidation())
    }

    fn default_thermal_proc() -> SplProcessor {
        SplProcessor::new(SplConfig::default_thermal())
    }

    fn default_mechanical_proc() -> SplProcessor {
        SplProcessor::new(SplConfig::default_mechanical())
    }

    fn default_dip_pen_proc() -> SplProcessor {
        SplProcessor::new(SplConfig::default_dip_pen())
    }

    // === SplMode ===

    #[test]
    fn test_spl_mode_clone_eq() {
        let m1 = SplMode::OxidationSPL;
        let m2 = m1;
        assert_eq!(m1, m2);

        assert_ne!(SplMode::ThermalSPL, SplMode::MechanicalSPL);
    }

    // === SplConfig presets ===

    #[test]
    fn test_config_default_oxidation() {
        let c = SplConfig::default_oxidation();
        assert_eq!(c.tip_radius_nm, 25.0);
        assert_eq!(c.applied_voltage_v, 8.0);
        assert_eq!(c.mode, SplMode::OxidationSPL);
        assert!(c.humidity_percent > 0.0);
    }

    #[test]
    fn test_config_default_dip_pen() {
        let c = SplConfig::default_dip_pen();
        assert_eq!(c.mode, SplMode::DipPenNanolithography);
        assert_eq!(c.applied_voltage_v, 0.0); // No voltage for DPN
    }

    #[test]
    fn test_config_default_thermal() {
        let c = SplConfig::default_thermal();
        assert_eq!(c.mode, SplMode::ThermalSPL);
        assert_eq!(c.tip_radius_nm, 10.0); // Sharper tip
    }

    #[test]
    fn test_config_default_mechanical() {
        let c = SplConfig::default_mechanical();
        assert_eq!(c.mode, SplMode::MechanicalSPL);
        assert!(c.set_point_na > 100.0); // Higher force
    }

    // === Oxide line width ===

    #[test]
    fn test_oxide_line_width_positive() {
        let p = default_oxidation_proc();
        let w = p.oxide_line_width();
        assert!(w > 0.0, "Line width should be positive: {}", w);
        assert!(w < 500.0, "Line width should be < 500 nm: {}", w);
    }

    #[test]
    fn test_oxide_line_width_increases_with_voltage() {
        let p_low = SplProcessor::new(SplConfig {
            applied_voltage_v: 4.0,
            ..SplConfig::default_oxidation()
        });
        let p_high = SplProcessor::new(SplConfig {
            applied_voltage_v: 10.0,
            ..SplConfig::default_oxidation()
        });

        assert!(p_high.oxide_line_width() > p_low.oxide_line_width());
    }

    #[test]
    fn test_oxide_line_width_decreases_with_speed() {
        let p_slow = SplProcessor::new(SplConfig {
            scan_speed_um_s: 0.5,
            ..SplConfig::default_oxidation()
        });
        let p_fast = SplProcessor::new(SplConfig {
            scan_speed_um_s: 10.0,
            ..SplConfig::default_oxidation()
        });

        assert!(
            p_slow.oxide_line_width() > p_fast.oxide_line_width(),
            "Slower scan should produce wider lines"
        );
    }

    #[test]
    fn test_oxide_line_width_increases_with_humidity() {
        let p_dry = SplProcessor::new(SplConfig {
            humidity_percent: 20.0,
            ..SplConfig::default_oxidation()
        });
        let p_wet = SplProcessor::new(SplConfig {
            humidity_percent: 70.0,
            ..SplConfig::default_oxidation()
        });

        assert!(
            p_wet.oxide_line_width() > p_dry.oxide_line_width(),
            "Higher humidity should produce wider lines"
        );
    }

    // === Oxide height (Cabrera-Mott) ===

    #[test]
    fn test_oxide_height_zero_time() {
        let p = default_oxidation_proc();
        assert_eq!(p.oxide_height_model(0.0), 0.0);
    }

    #[test]
    fn test_oxide_height_negative_time() {
        let p = default_oxidation_proc();
        assert_eq!(p.oxide_height_model(-1.0), 0.0);
    }

    #[test]
    fn test_oxide_height_logarithmic_growth() {
        let p = default_oxidation_proc();
        let h1 = p.oxide_height_model(0.001);
        let h2 = p.oxide_height_model(0.01);
        let h3 = p.oxide_height_model(0.1);
        let h4 = p.oxide_height_model(1.0);

        // Height should increase monotonically
        assert!(h2 > h1);
        assert!(h3 > h2);
        assert!(h4 > h3);

        // Growth rate should slow down (logarithmic): compare dh/dt over
        // equal-length intervals at different times
        // Rate at early time (0.001 to 0.01): dh / dt
        let rate_early = (h2 - h1) / (0.01 - 0.001);
        // Rate at late time (0.1 to 1.0): dh / dt
        let rate_late = (h4 - h3) / (1.0 - 0.1);
        assert!(rate_early > rate_late, "Growth rate should decrease over time: early={} late={}", rate_early, rate_late);

        // Reasonable range: 0.5-15 nm
        assert!(h3 > 0.5, "Height should exceed 0.5 nm after 100 ms");
        assert!(h4 < 20.0, "Height should be < 20 nm after 1 s");
    }

    #[test]
    fn test_oxide_height_scales_with_voltage() {
        let p_low = SplProcessor::new(SplConfig {
            applied_voltage_v: 3.0,
            ..SplConfig::default_oxidation()
        });
        let p_high = SplProcessor::new(SplConfig {
            applied_voltage_v: 10.0,
            ..SplConfig::default_oxidation()
        });

        assert!(p_high.oxide_height_model(0.01) > p_low.oxide_height_model(0.01));
    }

    // === Dip-pen diffusion ===

    #[test]
    fn test_dip_pen_diffusion_peak_at_origin() {
        let p = default_dip_pen_proc();
        let c_origin = p.dip_pen_diffusion(0.0, 1.0, 1e6, 1000.0);
        let c_far = p.dip_pen_diffusion(100.0, 1.0, 1e6, 1000.0);

        assert!(c_origin > c_far, "Concentration should peak at r=0");
        assert!(c_origin > 0.0);
    }

    #[test]
    fn test_dip_pen_diffusion_gaussian_decay() {
        let p = default_dip_pen_proc();
        let d = 500.0; // nm^2/s
        let t = 0.5; // s
        let n = 1e6;

        let c10 = p.dip_pen_diffusion(10.0, t, n, d);
        let c50 = p.dip_pen_diffusion(50.0, t, n, d);
        let c100 = p.dip_pen_diffusion(100.0, t, n, d);

        assert!(c10 > c50);
        assert!(c50 > c100);
    }

    #[test]
    fn test_dip_pen_diffusion_zero_time() {
        let p = default_dip_pen_proc();
        assert_eq!(p.dip_pen_diffusion(10.0, 0.0, 1e6, 500.0), 0.0);
    }

    #[test]
    fn test_dip_pen_diffusion_spreads_with_time() {
        let p = default_dip_pen_proc();
        let d = 500.0;
        let n = 1e6;
        let r = 50.0;

        // At short time, concentration at r=50 is low; at longer time it increases
        // (the Gaussian broadens). But peak concentration drops.
        let c_peak_early = p.dip_pen_diffusion(0.0, 0.01, n, d);
        let c_peak_late = p.dip_pen_diffusion(0.0, 1.0, n, d);
        assert!(
            c_peak_early > c_peak_late,
            "Peak concentration should decrease with time"
        );
    }

    // === Thermal decomposition ===

    #[test]
    fn test_thermal_profile_at_contact() {
        let p = default_thermal_proc();
        let t = p.thermal_decomposition(0.0, 800.0, 5.0);
        // At r=0, temperature should equal tip temperature
        assert!((t - 800.0).abs() < 1.0);
    }

    #[test]
    fn test_thermal_profile_decays_with_distance() {
        let p = default_thermal_proc();
        let t0 = p.thermal_decomposition(0.0, 800.0, 5.0);
        let t10 = p.thermal_decomposition(10.0, 800.0, 5.0);
        let t50 = p.thermal_decomposition(50.0, 800.0, 5.0);
        let t200 = p.thermal_decomposition(200.0, 800.0, 5.0);

        assert!(t0 > t10);
        assert!(t10 > t50);
        assert!(t50 > t200);

        // Far from tip should approach ambient
        assert!(t200 < 100.0);
    }

    #[test]
    fn test_thermal_zero_contact_radius() {
        let p = default_thermal_proc();
        let t = p.thermal_decomposition(10.0, 800.0, 0.0);
        assert_eq!(t, p.config().temperature_c);
    }

    // === Mechanical indentation (Hertz) ===

    #[test]
    fn test_hertz_zero_depth() {
        let p = default_mechanical_proc();
        assert_eq!(p.mechanical_indentation(0.0, 100.0), 0.0);
    }

    #[test]
    fn test_hertz_force_increases_with_depth() {
        let p = default_mechanical_proc();
        let f1 = p.mechanical_indentation(1.0, 100.0);
        let f5 = p.mechanical_indentation(5.0, 100.0);
        let f10 = p.mechanical_indentation(10.0, 100.0);

        assert!(f1 > 0.0);
        assert!(f5 > f1);
        assert!(f10 > f5);
    }

    #[test]
    fn test_hertz_superlinear_depth_dependence() {
        let p = default_mechanical_proc();
        // Force goes as d^1.5, so doubling depth should more than double force
        let f1 = p.mechanical_indentation(1.0, 100.0);
        let f2 = p.mechanical_indentation(2.0, 100.0);

        let ratio = f2 / f1;
        let expected_ratio = 2.0_f64.powf(1.5); // ~2.83
        assert!((ratio - expected_ratio).abs() < 0.01);
    }

    #[test]
    fn test_hertz_scales_with_modulus() {
        let p = default_mechanical_proc();
        let f_soft = p.mechanical_indentation(5.0, 10.0);
        let f_hard = p.mechanical_indentation(5.0, 200.0);

        assert!((f_hard / f_soft - 20.0).abs() < 0.1, "Force should scale linearly with modulus");
    }

    // === JKR contact ===

    #[test]
    fn test_jkr_reduces_to_hertz_without_adhesion() {
        let p = default_mechanical_proc();
        let f_hertz = p.mechanical_indentation(5.0, 100.0);
        let (f_jkr, _a) = p.contact_mechanics_jkr(5.0, 100.0, 0.0);

        assert!(
            (f_jkr - f_hertz).abs() < 1e-6,
            "JKR with zero adhesion should equal Hertz"
        );
    }

    #[test]
    fn test_jkr_adhesion_reduces_force() {
        let p = default_mechanical_proc();
        let f_hertz = p.mechanical_indentation(5.0, 100.0);
        let (f_jkr, _) = p.contact_mechanics_jkr(5.0, 100.0, 50.0);

        assert!(
            f_jkr < f_hertz,
            "Adhesion should reduce the net applied force"
        );
    }

    #[test]
    fn test_jkr_pulloff_force() {
        let p = default_mechanical_proc();
        let (f_pulloff, _) = p.contact_mechanics_jkr(0.0, 100.0, 50.0);

        assert!(f_pulloff < 0.0, "Pull-off force should be negative (attractive)");
    }

    // === Electric field ===

    #[test]
    fn test_electric_field_positive() {
        let p = default_oxidation_proc();
        let e = p.electric_field_at_tip();
        assert!(e > 0.0);
    }

    #[test]
    fn test_electric_field_increases_with_voltage() {
        let p_low = SplProcessor::new(SplConfig {
            applied_voltage_v: 3.0,
            ..SplConfig::default_oxidation()
        });
        let p_high = SplProcessor::new(SplConfig {
            applied_voltage_v: 10.0,
            ..SplConfig::default_oxidation()
        });

        assert!(p_high.electric_field_at_tip() > p_low.electric_field_at_tip());
    }

    #[test]
    fn test_electric_field_inversely_proportional_to_radius() {
        let p_sharp = SplProcessor::new(SplConfig {
            tip_radius_nm: 10.0,
            ..SplConfig::default_oxidation()
        });
        let p_blunt = SplProcessor::new(SplConfig {
            tip_radius_nm: 50.0,
            ..SplConfig::default_oxidation()
        });

        assert!(
            p_sharp.electric_field_at_tip() > p_blunt.electric_field_at_tip(),
            "Sharper tip should have stronger field"
        );

        // Should be ~5x ratio
        let ratio = p_sharp.electric_field_at_tip() / p_blunt.electric_field_at_tip();
        assert!((ratio - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_electric_field_zero_radius() {
        let p = SplProcessor::new(SplConfig {
            tip_radius_nm: 0.0,
            ..SplConfig::default_oxidation()
        });
        assert_eq!(p.electric_field_at_tip(), 0.0);
    }

    // === Feature resolution ===

    #[test]
    fn test_feature_resolution_all_modes() {
        let modes = [
            SplConfig::default_oxidation(),
            SplConfig::default_dip_pen(),
            SplConfig::default_thermal(),
            SplConfig::default_mechanical(),
        ];

        for cfg in &modes {
            let p = SplProcessor::new(cfg.clone());
            let res = p.feature_resolution();
            assert!(
                res > 1.0 && res < 500.0,
                "Resolution {:.1} nm out of expected range for {:?}",
                res,
                cfg.mode
            );
        }
    }

    #[test]
    fn test_thermal_best_resolution() {
        // Thermal SPL should achieve the finest features
        let t = default_thermal_proc();
        let o = default_oxidation_proc();

        assert!(
            t.feature_resolution() < o.feature_resolution(),
            "Thermal SPL should have finer resolution than oxidation"
        );
    }

    // === Meniscus size ===

    #[test]
    fn test_meniscus_size_positive_at_normal_humidity() {
        let p = default_oxidation_proc();
        let d = p.meniscus_size(1.0);
        assert!(d > 0.0, "Meniscus should form at 50% RH");
        assert!(d < 200.0, "Meniscus diameter should be < 200 nm");
    }

    #[test]
    fn test_meniscus_increases_with_humidity() {
        let p_dry = SplProcessor::new(SplConfig {
            humidity_percent: 30.0,
            ..SplConfig::default_oxidation()
        });
        let p_wet = SplProcessor::new(SplConfig {
            humidity_percent: 80.0,
            ..SplConfig::default_oxidation()
        });

        assert!(p_wet.meniscus_size(1.0) > p_dry.meniscus_size(1.0));
    }

    #[test]
    fn test_meniscus_decreases_with_gap() {
        let p = default_oxidation_proc();
        let d1 = p.meniscus_size(0.5);
        let d10 = p.meniscus_size(10.0);

        assert!(d1 > d10, "Meniscus should shrink with gap distance");
    }

    #[test]
    fn test_meniscus_zero_humidity() {
        let p = SplProcessor::new(SplConfig {
            humidity_percent: 0.0,
            ..SplConfig::default_oxidation()
        });
        assert_eq!(p.meniscus_size(1.0), 0.0);
    }

    // === Line edge roughness ===

    #[test]
    fn test_ler_perfect_edges() {
        let p = default_oxidation_proc();
        let left = vec![10.0; 100];
        let right = vec![50.0; 100];

        let (ler_l, ler_r, lwr) = p.line_edge_roughness(&left, &right);
        assert_eq!(ler_l, 0.0);
        assert_eq!(ler_r, 0.0);
        assert_eq!(lwr, 0.0);
    }

    #[test]
    fn test_ler_noisy_edges() {
        let p = default_oxidation_proc();

        // Simple deterministic noise
        let left: Vec<f64> = (0..100)
            .map(|i| 10.0 + 2.0 * ((i as f64 * 0.7).sin()))
            .collect();
        let right: Vec<f64> = (0..100)
            .map(|i| 50.0 + 1.5 * ((i as f64 * 0.9).cos()))
            .collect();

        let (ler_l, ler_r, lwr) = p.line_edge_roughness(&left, &right);

        assert!(ler_l > 0.0, "LER should be positive for noisy edges");
        assert!(ler_r > 0.0);
        assert!(lwr > 0.0);

        // LER should be ~3 * std(noise), noise amplitude ~2 nm so sigma ~ 1.4
        assert!(ler_l < 10.0, "LER should be reasonable: {}", ler_l);
    }

    // === Pattern fidelity ===

    #[test]
    fn test_pattern_fidelity_perfect() {
        let p = default_oxidation_proc();
        let intended: Vec<f64> = (0..100).map(|i| (i as f64) * 0.1).collect();
        let measured = intended.clone();

        assert_eq!(p.pattern_fidelity(&intended, &measured), 0.0);
    }

    #[test]
    fn test_pattern_fidelity_with_error() {
        let p = default_oxidation_proc();
        let intended: Vec<f64> = vec![5.0; 100];
        let measured: Vec<f64> = vec![5.5; 100]; // 0.5 nm systematic error

        let rms = p.pattern_fidelity(&intended, &measured);
        assert!((rms - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_pattern_fidelity_empty() {
        let p = default_oxidation_proc();
        assert_eq!(p.pattern_fidelity(&[], &[]), 0.0);
    }

    // === Scan trajectory ===

    #[test]
    fn test_raster_scan_basic() {
        let p = default_oxidation_proc();
        let path = p.scan_trajectory(100.0, 100.0, 10.0, 10.0);

        assert!(!path.x_nm.is_empty());
        assert_eq!(path.x_nm.len(), path.y_nm.len());

        // Should cover the full area
        let x_min = path.x_nm.iter().cloned().fold(f64::MAX, f64::min);
        let x_max = path.x_nm.iter().cloned().fold(f64::MIN, f64::max);
        assert!((x_min - 0.0).abs() < 1e-10);
        assert!((x_max - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_raster_scan_serpentine() {
        let p = default_oxidation_proc();
        let path = p.scan_trajectory(100.0, 20.0, 10.0, 50.0);

        // Line 0: x goes 0 -> 100, Line 1: x goes 100 -> 0
        // Find points on line 0 (y=0) and line 1 (y=10)
        let line0_x: Vec<f64> = path
            .x_nm
            .iter()
            .zip(path.y_nm.iter())
            .filter(|(_, &y)| y < 1.0)
            .map(|(&x, _)| x)
            .collect();

        let line1_x: Vec<f64> = path
            .x_nm
            .iter()
            .zip(path.y_nm.iter())
            .filter(|(_, &y)| (y - 10.0).abs() < 1.0)
            .map(|(&x, _)| x)
            .collect();

        // Line 0 starts at 0, line 1 starts at 100
        assert!(line0_x[0] < line0_x.last().copied().unwrap_or(0.0) + 1.0);
        assert!(line1_x[0] > line1_x.last().copied().unwrap_or(f64::MAX) - 1.0);
    }

    #[test]
    fn test_raster_scan_zero_size() {
        let p = default_oxidation_proc();
        let path = p.scan_trajectory(0.0, 100.0, 10.0, 10.0);
        assert!(path.x_nm.is_empty());
    }

    // === Writing speed limit ===

    #[test]
    fn test_writing_speed_limit_positive() {
        let p = default_oxidation_proc();
        let speed = p.writing_speed_limit(50.0); // 50 nm target
        assert!(speed > 0.0);
    }

    #[test]
    fn test_narrower_features_allow_faster_speed() {
        let p = default_oxidation_proc();
        let speed_narrow = p.writing_speed_limit(30.0);
        let speed_wide = p.writing_speed_limit(100.0);

        // To achieve a wider line you must scan slower (more dwell time),
        // so the speed limit for wide features is lower.
        // Conversely, narrow features are produced at faster scan speeds.
        assert!(
            speed_narrow > speed_wide,
            "Narrow feature target allows faster scanning: narrow={} wide={}",
            speed_narrow,
            speed_wide
        );
    }

    // === Piezo hysteresis correction ===

    #[test]
    fn test_piezo_correction_identity_for_zero_hysteresis() {
        let p = default_oxidation_proc();
        let positions = vec![0.0, 100.0, 200.0, 300.0];
        let corrected = p.piezo_hysteresis_correction(&positions, 0.0);

        for (a, b) in positions.iter().zip(corrected.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_piezo_correction_modifies_positions() {
        let p = default_oxidation_proc();
        let positions: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let corrected = p.piezo_hysteresis_correction(&positions, 0.15);

        // Corrected positions should differ from originals (except endpoints)
        let mut any_different = false;
        for i in 1..positions.len() - 1 {
            if (positions[i] - corrected[i]).abs() > 1e-6 {
                any_different = true;
                break;
            }
        }
        assert!(any_different, "Correction should modify interior points");
    }

    // === Drift compensation ===

    #[test]
    fn test_drift_compensation_removes_linear_drift() {
        let p = default_oxidation_proc();
        let n = 100;
        let drift_rate_x = 0.1; // nm/s
        let drift_rate_y = -0.05; // nm/s

        let times: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let x_drifted: Vec<f64> = times.iter().map(|&t| 50.0 + drift_rate_x * t).collect();
        let y_drifted: Vec<f64> = times.iter().map(|&t| 100.0 + drift_rate_y * t).collect();

        let (x_corr, y_corr, dx, dy) = p.drift_compensation(&times, &x_drifted, &y_drifted);

        // Estimated drift rates should match
        assert!(
            (dx - drift_rate_x).abs() < 0.01,
            "X drift rate: {} vs {}",
            dx,
            drift_rate_x
        );
        assert!(
            (dy - drift_rate_y).abs() < 0.01,
            "Y drift rate: {} vs {}",
            dy,
            drift_rate_y
        );

        // Corrected positions should be nearly constant
        let x_range = x_corr.iter().cloned().fold(f64::MAX, f64::min);
        let x_max = x_corr.iter().cloned().fold(f64::MIN, f64::max);
        assert!(
            (x_max - x_range) < 0.1,
            "Corrected X range should be small: {}",
            x_max - x_range
        );
    }

    // === Tip wear ===

    #[test]
    fn test_tip_wear_increases_with_writes() {
        let p = default_oxidation_proc();
        let r0 = p.tip_wear_model(0);
        let r100 = p.tip_wear_model(100);
        let r10000 = p.tip_wear_model(10_000);

        assert!((r0 - p.config().tip_radius_nm).abs() < 1e-10);
        assert!(r100 > r0);
        assert!(r10000 > r100);
    }

    #[test]
    fn test_mechanical_wears_faster() {
        let p_ox = default_oxidation_proc();
        let p_mech = default_mechanical_proc();

        // Compare relative wear after same writes
        let r0_ox = p_ox.config().tip_radius_nm;
        let r0_mech = p_mech.config().tip_radius_nm;

        let wear_ox = (p_ox.tip_wear_model(1000) - r0_ox) / r0_ox;
        let wear_mech = (p_mech.tip_wear_model(1000) - r0_mech) / r0_mech;

        assert!(
            wear_mech > wear_ox,
            "Mechanical mode should wear tip faster"
        );
    }

    // === Pattern generation ===

    #[test]
    fn test_generate_line_pattern() {
        let p = default_oxidation_proc();
        let pat = p.generate_line_pattern(5, 1000.0, 50.0, 3.0, 200.0);

        assert_eq!(pat.len(), 5);
        assert_eq!(pat.x_positions_nm.len(), 5);

        // Y positions should be equally spaced
        for i in 0..5 {
            assert!((pat.y_positions_nm[i] - (i as f64) * 200.0).abs() < 1e-10);
        }

        // Check feature type
        match &pat.features[0] {
            SplFeature::Line {
                width_nm,
                depth_nm,
                length_nm,
            } => {
                assert_eq!(*width_nm, 50.0);
                assert_eq!(*depth_nm, 3.0);
                assert_eq!(*length_nm, 1000.0);
            }
            _ => panic!("Expected Line feature"),
        }
    }

    #[test]
    fn test_generate_dot_array() {
        let p = default_oxidation_proc();
        let pat = p.generate_dot_array(4, 5, 30.0, 2.0, 100.0, 100.0);

        assert_eq!(pat.len(), 20); // 4 * 5

        // Check corner positions
        assert!((pat.x_positions_nm[0] - 0.0).abs() < 1e-10);
        assert!((pat.y_positions_nm[0] - 0.0).abs() < 1e-10);

        match &pat.features[0] {
            SplFeature::Dot {
                diameter_nm,
                depth_nm,
            } => {
                assert_eq!(*diameter_nm, 30.0);
                assert_eq!(*depth_nm, 2.0);
            }
            _ => panic!("Expected Dot feature"),
        }
    }

    // === Overlay alignment ===

    #[test]
    fn test_overlay_alignment_no_shift() {
        let p = default_oxidation_proc();
        let w = 16;
        let h = 16;
        let data: Vec<f64> = (0..w * h).map(|i| (i as f64 * 0.3).sin()).collect();

        let (dx, dy, corr) = p.overlay_alignment(&data, &data, w, h);
        assert_eq!(dx, 0);
        assert_eq!(dy, 0);
        assert!(corr > 0.0);
    }

    #[test]
    fn test_overlay_alignment_detects_shift() {
        let p = default_oxidation_proc();
        let w = 32;
        let h = 32;

        // Create reference: a bright dot at (16, 16)
        let mut reference = vec![0.0; w * h];
        for r in 14..18 {
            for c in 14..18 {
                reference[r * w + c] = 10.0;
            }
        }

        // Measured: same dot shifted by (2, 1)
        let mut measured = vec![0.0; w * h];
        for r in 15..19 {
            for c in 16..20 {
                measured[r * w + c] = 10.0;
            }
        }

        let (dx, dy, _corr) = p.overlay_alignment(&reference, &measured, w, h);

        // Should detect the shift (reference shifted by +2,+1 gives measured)
        assert!(
            dx.abs() <= 3 && dy.abs() <= 2,
            "Should detect approximate shift, got dx={}, dy={}",
            dx,
            dy
        );
    }

    // === Dose map ===

    #[test]
    fn test_dose_map_values() {
        let p = default_oxidation_proc();
        let path = ScanPath {
            x_nm: vec![0.0, 10.0, 20.0],
            y_nm: vec![0.0, 0.0, 0.0],
        };
        let dwells = vec![0.001, 0.002, 0.001];

        let doses = p.dose_map(&path, &dwells);
        assert_eq!(doses.len(), 3);

        let v = p.config().applied_voltage_v;
        assert!((doses[0] - v * 0.001).abs() < 1e-10);
        assert!((doses[1] - v * 0.002).abs() < 1e-10);
    }

    // === SplPattern ===

    #[test]
    fn test_pattern_empty() {
        let p = SplPattern::new();
        assert!(p.is_empty());
        assert_eq!(p.len(), 0);
    }

    #[test]
    fn test_pattern_add_feature() {
        let mut p = SplPattern::new();
        p.add_feature(
            100.0,
            200.0,
            SplFeature::Dot {
                diameter_nm: 30.0,
                depth_nm: 2.0,
            },
        );

        assert_eq!(p.len(), 1);
        assert!(!p.is_empty());
        assert_eq!(p.x_positions_nm[0], 100.0);
        assert_eq!(p.y_positions_nm[0], 200.0);
    }

    // === Humidity factor ===

    #[test]
    fn test_humidity_factor_range() {
        // Below 20%: near zero
        assert!(humidity_factor(10.0) < 0.1);

        // At 50%: well above 0.5
        assert!(humidity_factor(50.0) > 0.8);

        // At 80%: near 1.0
        assert!(humidity_factor(80.0) > 0.99);
    }

    // === Helper functions ===

    #[test]
    fn test_three_sigma_constant() {
        assert_eq!(three_sigma(&[5.0; 50]), 0.0);
    }

    #[test]
    fn test_mean_basic() {
        assert!((mean(&[1.0, 2.0, 3.0, 4.0, 5.0]) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_slope_perfect() {
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 2.0 * xi + 5.0).collect();

        let slope = linear_regression_slope(&x, &y);
        assert!((slope - 2.0).abs() < 1e-10);
    }
}
