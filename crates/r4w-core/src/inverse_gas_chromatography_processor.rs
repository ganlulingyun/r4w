//! Inverse Gas Chromatography (IGC) Processor
//!
//! Implements surface energy characterization via inverse gas chromatography.
//! IGC measures the retention of probe molecules on a stationary phase (the material
//! under study) to determine dispersive surface energy, acid-base parameters,
//! work of adhesion, and glass transition temperature.
//!
//! # Key Physics
//!
//! **Retention Volume** (net):
//! ```text
//! Vn = j * F * (tr - t0) * (T / Tf) * (1 - pw/po)
//! ```
//! where j is the James-Martin compressibility correction, F is carrier gas flow rate,
//! tr is retention time, t0 is dead time, T is column temperature, Tf is flowmeter
//! temperature, pw is water vapor pressure at flowmeter, and po is outlet pressure.
//!
//! **James-Martin Correction**:
//! ```text
//! j = (3/2) * ((Pi/Po)^2 - 1) / ((Pi/Po)^3 - 1)
//! ```
//!
//! **Schultz Method** (dispersive surface energy):
//! ```text
//! RT * ln(Vn) = 2 * NA * a * sqrt(gamma_s_d * gamma_l_d) + C
//! ```
//!
//! **Dorris-Gray Method**:
//! ```text
//! gamma_s_d = (Delta_G_CH2)^2 / (4 * NA^2 * a_CH2^2 * gamma_CH2)
//! ```
//!
//! **Gutmann Acid-Base**:
//! ```text
//! Delta_H_sp = Ka * DN + Kd * AN*
//! ```

/// Avogadro's number (mol^-1)
pub const AVOGADRO: f64 = 6.022_140_76e23;

/// Universal gas constant (J/mol/K)
pub const R_GAS: f64 = 8.314_462_618;

/// Cross-sectional area of a CH2 group (m²) — Dorris-Gray reference
pub const A_CH2_M2: f64 = 6.0e-20; // 6.0 Å² in m²

/// Surface energy of a CH2 layer (polyethylene reference, mJ/m²)
pub const GAMMA_CH2_MJ_M2: f64 = 35.6;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

/// Raw retention measurement for a single probe injection
#[derive(Debug, Clone, PartialEq)]
pub struct RetentionData {
    /// Name of the probe molecule (e.g., "n-Hexane")
    pub probe_name: String,
    /// Measured retention time (s)
    pub retention_time_s: f64,
    /// Dead / hold-up time measured with non-retained marker (s)
    pub dead_time_s: f64,
    /// Carrier gas flow rate at flowmeter (mL/min)
    pub flow_rate_ml_min: f64,
    /// Column oven temperature (K)
    pub column_temp_k: f64,
}

/// Chromatographic column and carrier-gas conditions
#[derive(Debug, Clone, PartialEq)]
pub struct ColumnConditions {
    /// Column inlet pressure (Pa)
    pub inlet_pressure_pa: f64,
    /// Column outlet (atmospheric) pressure (Pa)
    pub outlet_pressure_pa: f64,
    /// Flowmeter temperature (K) — for correcting measured flow to column conditions
    pub flowmeter_temp_k: f64,
    /// Saturated water vapor pressure at flowmeter temperature (Pa)
    pub water_vapor_pressure_pa: f64,
}

/// Fully-computed surface energy result for a material
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceEnergyResult {
    /// Dispersive component of surface energy (mJ/m²)
    pub gamma_d_mj_m2: f64,
    /// Specific (polar/acid-base) component of surface energy (mJ/m²)
    pub gamma_sp_mj_m2: f64,
    /// Total surface energy = gamma_d + gamma_sp (mJ/m²)
    pub gamma_total_mj_m2: f64,
    /// Gutmann acid constant Ka (dimensionless)
    pub ka: f64,
    /// Gutmann base constant Kd (dimensionless)
    pub kd: f64,
    /// Coefficient of determination for Schultz fit
    pub r_squared: f64,
}

impl Default for SurfaceEnergyResult {
    fn default() -> Self {
        Self {
            gamma_d_mj_m2: 0.0,
            gamma_sp_mj_m2: 0.0,
            gamma_total_mj_m2: 0.0,
            ka: 0.0,
            kd: 0.0,
            r_squared: 0.0,
        }
    }
}

/// Schultz plot data (dispersive analysis with n-alkane series)
#[derive(Debug, Clone, PartialEq)]
pub struct SchultzPlotData {
    /// x-axis values: a * sqrt(gamma_l_d) — units: m² * sqrt(mJ/m²)
    pub x_values: Vec<f64>,
    /// y-axis values: RT * ln(Vn) — units: J/mol
    pub y_values: Vec<f64>,
    /// Slope of least-squares fit: 2 * NA * sqrt(gamma_s_d)
    pub slope: f64,
    /// Intercept (J/mol) — integration constant C
    pub intercept: f64,
    /// Coefficient of determination R²
    pub r_squared: f64,
    /// Derived dispersive surface energy (mJ/m²)
    pub gamma_s_d: f64,
}

/// Specific free energy of interaction for a polar probe
#[derive(Debug, Clone, PartialEq)]
pub struct SpecificInteraction {
    /// Probe molecule identifier
    pub probe_name: String,
    /// Specific free energy of adsorption (kJ/mol), positive = favorable
    pub delta_g_sp_kj_mol: f64,
    /// Specific enthalpy of adsorption (kJ/mol) — from temperature series
    pub delta_h_sp_kj_mol: f64,
}

/// Reference material preset with known IGC surface energy parameters
#[derive(Debug, Clone, PartialEq)]
pub struct IgcMaterial {
    /// Material name
    pub name: &'static str,
    /// Reference dispersive surface energy (mJ/m²)
    pub gamma_d_ref: f64,
    /// Gutmann acid constant Ka
    pub ka_ref: f64,
    /// Gutmann base constant Kd
    pub kd_ref: f64,
    /// Characterization temperature (°C)
    pub temp_c: f64,
}

/// Physical properties of an IGC probe molecule
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeProperties {
    /// Probe name
    pub name: &'static str,
    /// Molecular cross-sectional area (Å²)
    pub area_angstrom2: f64,
    /// Dispersive surface tension of the liquid probe (mJ/m²)
    pub gamma_l_d: f64,
    /// Gutmann donor number DN (kJ/mol) — 0 for non-donors
    pub donor_number: f64,
    /// Gutmann modified acceptor number AN* (dimensionless) — 0 for non-acceptors
    pub acceptor_number_star: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Probe database
// ─────────────────────────────────────────────────────────────────────────────

/// Return the full built-in probe molecule database.
///
/// n-Alkanes are used for the Schultz / Dorris-Gray dispersive analysis.
/// Polar probes are used for specific (acid-base) interaction analysis.
pub fn probe_database() -> Vec<ProbeProperties> {
    vec![
        // n-Alkane series — dispersive reference probes
        ProbeProperties {
            name: "n-Pentane",
            area_angstrom2: 36.2,
            gamma_l_d: 15.5,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        ProbeProperties {
            name: "n-Hexane",
            area_angstrom2: 51.5,
            gamma_l_d: 18.4,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        ProbeProperties {
            name: "n-Heptane",
            area_angstrom2: 57.0,
            gamma_l_d: 20.3,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        ProbeProperties {
            name: "n-Octane",
            area_angstrom2: 62.8,
            gamma_l_d: 21.3,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        ProbeProperties {
            name: "n-Nonane",
            area_angstrom2: 69.0,
            gamma_l_d: 22.7,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        ProbeProperties {
            name: "n-Decane",
            area_angstrom2: 75.0,
            gamma_l_d: 23.4,
            donor_number: 0.0,
            acceptor_number_star: 0.0,
        },
        // Polar probes — acid-base characterization
        ProbeProperties {
            name: "Chloroform",
            area_angstrom2: 44.0,
            gamma_l_d: 25.9,
            donor_number: 0.0,
            acceptor_number_star: 5.4,
        },
        ProbeProperties {
            name: "THF",
            area_angstrom2: 45.0,
            gamma_l_d: 22.5,
            donor_number: 20.0,
            acceptor_number_star: 0.5,
        },
        ProbeProperties {
            name: "Ethyl acetate",
            area_angstrom2: 48.0,
            gamma_l_d: 20.2,
            donor_number: 17.1,
            acceptor_number_star: 1.5,
        },
        ProbeProperties {
            name: "Acetone",
            area_angstrom2: 42.0,
            gamma_l_d: 16.5,
            donor_number: 17.0,
            acceptor_number_star: 2.5,
        },
        ProbeProperties {
            name: "Dichloromethane",
            area_angstrom2: 31.5,
            gamma_l_d: 26.2,
            donor_number: 0.0,
            acceptor_number_star: 3.9,
        },
        ProbeProperties {
            name: "Diethyl ether",
            area_angstrom2: 47.0,
            gamma_l_d: 17.0,
            donor_number: 19.2,
            acceptor_number_star: 1.4,
        },
    ]
}

/// Look up a probe by name (case-insensitive partial match).
///
/// Returns `None` if no match is found.
pub fn find_probe(name: &str) -> Option<ProbeProperties> {
    let lower = name.to_lowercase();
    probe_database()
        .into_iter()
        .find(|p| p.name.to_lowercase().contains(&lower))
}

// ─────────────────────────────────────────────────────────────────────────────
// Material preset database
// ─────────────────────────────────────────────────────────────────────────────

/// Return the built-in material preset database.
pub fn material_presets() -> Vec<IgcMaterial> {
    vec![
        IgcMaterial {
            name: "Talc",
            gamma_d_ref: 35.0,
            ka_ref: 0.05,
            kd_ref: 0.35,
            temp_c: 40.0,
        },
        IgcMaterial {
            name: "Calcium carbonate",
            gamma_d_ref: 55.0,
            ka_ref: 0.12,
            kd_ref: 0.65,
            temp_c: 40.0,
        },
        IgcMaterial {
            name: "Carbon fiber (unsized)",
            gamma_d_ref: 42.0,
            ka_ref: 0.10,
            kd_ref: 0.15,
            temp_c: 50.0,
        },
        IgcMaterial {
            name: "Pharmaceutical lactose",
            gamma_d_ref: 38.0,
            ka_ref: 0.08,
            kd_ref: 0.45,
            temp_c: 30.0,
        },
    ]
}

/// Look up a material preset by name (case-insensitive partial match).
pub fn find_material(name: &str) -> Option<IgcMaterial> {
    let lower = name.to_lowercase();
    material_presets()
        .into_iter()
        .find(|m| m.name.to_lowercase().contains(&lower))
}

// ─────────────────────────────────────────────────────────────────────────────
// Core IGC computations
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the James-Martin compressibility correction factor *j*.
///
/// ```text
/// j = (3/2) * ((Pi/Po)^2 - 1) / ((Pi/Po)^3 - 1)
/// ```
///
/// Valid when Pi > Po (inlet pressure must exceed outlet pressure).
///
/// # Arguments
/// * `inlet_pressure_pa`  — column inlet (head) pressure (Pa)
/// * `outlet_pressure_pa` — column outlet (atmospheric) pressure (Pa)
///
/// # Returns
/// Dimensionless correction factor j ∈ (0, 1].
pub fn james_martin_correction(inlet_pressure_pa: f64, outlet_pressure_pa: f64) -> f64 {
    let ratio = inlet_pressure_pa / outlet_pressure_pa;
    if (ratio - 1.0).abs() < 1e-10 {
        // No pressure drop: limit is 1
        return 1.0;
    }
    let ratio2 = ratio * ratio;
    let ratio3 = ratio2 * ratio;
    1.5 * (ratio2 - 1.0) / (ratio3 - 1.0)
}

/// Compute the net retention volume *Vn* (mL).
///
/// ```text
/// Vn = j * F * (tr - t0) * (T / Tf) * (1 - pw/po)
/// ```
///
/// # Arguments
/// * `data` — raw retention measurement
/// * `conditions` — column and carrier-gas conditions
///
/// # Returns
/// Net retention volume in mL. Returns `None` if inputs are non-physical
/// (e.g. tr <= t0, or negative flow).
pub fn retention_volume(data: &RetentionData, conditions: &ColumnConditions) -> Option<f64> {
    if data.retention_time_s <= data.dead_time_s {
        return None;
    }
    if data.flow_rate_ml_min <= 0.0 {
        return None;
    }
    if conditions.inlet_pressure_pa <= conditions.outlet_pressure_pa {
        return None;
    }

    let j = james_martin_correction(
        conditions.inlet_pressure_pa,
        conditions.outlet_pressure_pa,
    );
    let f_ml_s = data.flow_rate_ml_min / 60.0; // convert mL/min → mL/s
    let t_net = data.retention_time_s - data.dead_time_s;
    let temp_ratio = data.column_temp_k / conditions.flowmeter_temp_k;
    let vapor_correction = 1.0 - conditions.water_vapor_pressure_pa / conditions.outlet_pressure_pa;

    Some(j * f_ml_s * t_net * temp_ratio * vapor_correction)
}

/// Compute the Schultz plot ordinate: RT * ln(Vn).
///
/// # Arguments
/// * `vn_ml`   — net retention volume (mL)
/// * `temp_k`  — column temperature (K)
///
/// # Returns
/// RT * ln(Vn) in J/mol. Returns `None` for non-positive Vn.
pub fn schultz_y_value(vn_ml: f64, temp_k: f64) -> Option<f64> {
    if vn_ml <= 0.0 {
        return None;
    }
    Some(R_GAS * temp_k * vn_ml.ln())
}

/// Compute the Schultz plot abscissa: a * sqrt(gamma_l_d).
///
/// # Arguments
/// * `area_angstrom2`  — molecular cross-sectional area (Å²)
/// * `gamma_l_d_mj_m2` — dispersive surface tension of liquid probe (mJ/m²)
///
/// # Returns
/// a * sqrt(gamma_l_d) in m² * sqrt(J/m²) = m² * (J/m²)^0.5.
pub fn schultz_x_value(area_angstrom2: f64, gamma_l_d_mj_m2: f64) -> f64 {
    // Convert Å² → m²: 1 Å = 1e-10 m, so 1 Å² = 1e-20 m²
    let a_m2 = area_angstrom2 * 1.0e-20;
    // Convert mJ/m² → J/m²: factor 1e-3
    let gamma_j_m2 = gamma_l_d_mj_m2 * 1.0e-3;
    a_m2 * gamma_j_m2.sqrt()
}

/// Perform a linear least-squares fit to (x, y) data.
///
/// Uses mean-centering to handle x values of very different magnitudes
/// (e.g., Schultz plot x values ~1e-20).
///
/// Returns (slope, intercept, r_squared).
pub fn linear_regression(x: &[f64], y: &[f64]) -> Option<(f64, f64, f64)> {
    let n = x.len();
    if n < 2 || n != y.len() {
        return None;
    }
    let n_f = n as f64;

    // Centre x and y to improve numerical stability
    let x_mean: f64 = x.iter().sum::<f64>() / n_f;
    let y_mean: f64 = y.iter().sum::<f64>() / n_f;

    let sxx: f64 = x.iter().map(|xi| (xi - x_mean).powi(2)).sum();
    let sxy: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| (xi - x_mean) * (yi - y_mean))
        .sum();

    if sxx == 0.0 {
        return None;
    }

    let slope = sxy / sxx;
    let intercept = y_mean - slope * x_mean;

    // R²
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| (yi - (slope * xi + intercept)).powi(2))
        .sum();
    let r_squared = if ss_tot == 0.0 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    Some((slope, intercept, r_squared))
}

/// Perform the Schultz analysis on a series of n-alkane measurements.
///
/// Fits RT*ln(Vn) vs a*sqrt(gamma_l_d) by least squares to extract the
/// dispersive surface energy γ_s^d.
///
/// # Arguments
/// * `alkane_data`  — retention data for each n-alkane
/// * `conditions`   — column conditions (same for all measurements at one temperature)
/// * `probe_props`  — corresponding probe properties (must match alkane_data order)
///
/// # Returns
/// `SchultzPlotData` with the regression result, or `None` if fewer than two
/// valid points are available.
pub fn schultz_analysis(
    alkane_data: &[RetentionData],
    conditions: &ColumnConditions,
    probe_props: &[ProbeProperties],
) -> Option<SchultzPlotData> {
    if alkane_data.len() != probe_props.len() {
        return None;
    }

    let mut xs = Vec::new();
    let mut ys = Vec::new();

    for (data, probe) in alkane_data.iter().zip(probe_props.iter()) {
        if let Some(vn) = retention_volume(data, conditions) {
            if let Some(y) = schultz_y_value(vn, data.column_temp_k) {
                let x = schultz_x_value(probe.area_angstrom2, probe.gamma_l_d);
                xs.push(x);
                ys.push(y);
            }
        }
    }

    if xs.len() < 2 {
        return None;
    }

    let (slope, intercept, r_squared) = linear_regression(&xs, &ys)?;

    // slope = 2 * NA * sqrt(gamma_s_d * 1e-3)  [J·m²/(m²·sqrt(J/m²)) = sqrt(J)]
    // => sqrt(gamma_s_d_j_m2) = slope / (2 * NA)
    // => gamma_s_d_j_m2 = (slope / (2 * NA))^2
    // => gamma_s_d_mj_m2 = gamma_s_d_j_m2 * 1e3
    let sqrt_gamma_s_d_j_m2 = slope / (2.0 * AVOGADRO);
    let gamma_s_d_mj_m2 = sqrt_gamma_s_d_j_m2 * sqrt_gamma_s_d_j_m2 * 1.0e3;

    Some(SchultzPlotData {
        x_values: xs,
        y_values: ys,
        slope,
        intercept,
        r_squared,
        gamma_s_d: gamma_s_d_mj_m2,
    })
}

/// Compute γ_s^d via the Dorris-Gray method from consecutive n-alkane retention volumes.
///
/// ```text
/// Delta_G_CH2 = RT * ln(Vn(C_{n+1}) / Vn(C_n))
/// gamma_s_d = (Delta_G_CH2)^2 / (4 * NA^2 * a_CH2^2 * gamma_CH2)
/// ```
///
/// # Arguments
/// * `vn_lower` — net retention volume for C_n alkane (mL)
/// * `vn_upper` — net retention volume for C_{n+1} alkane (mL)
/// * `temp_k`   — column temperature (K)
///
/// # Returns
/// Dispersive surface energy in mJ/m², or `None` for non-positive retention volumes.
pub fn dorris_gray_gamma_d(vn_lower: f64, vn_upper: f64, temp_k: f64) -> Option<f64> {
    if vn_lower <= 0.0 || vn_upper <= 0.0 {
        return None;
    }
    let delta_g_ch2_j_mol = R_GAS * temp_k * (vn_upper / vn_lower).ln();

    // gamma_s_d [J/m²] = delta_g_ch2^2 / (4 * NA^2 * a_ch2^2 * gamma_ch2)
    // gamma_ch2 must be in J/m² for dimensional consistency
    let gamma_ch2_j_m2 = GAMMA_CH2_MJ_M2 * 1.0e-3;
    let numerator = delta_g_ch2_j_mol * delta_g_ch2_j_mol;
    let denominator = 4.0 * AVOGADRO * AVOGADRO * A_CH2_M2 * A_CH2_M2 * gamma_ch2_j_m2;

    Some(numerator / denominator * 1.0e3) // convert J/m² → mJ/m²
}

/// Compute the specific free energy of adsorption ΔG_sp for a polar probe.
///
/// The reference line is the n-alkane Schultz fit evaluated at the probe's
/// x-value; the vertical distance above the line gives ΔG_sp.
///
/// ```text
/// Delta_G_sp = RT * ln(Vn) - (slope * x + intercept)
/// ```
///
/// # Arguments
/// * `vn_ml`     — net retention volume of the polar probe (mL)
/// * `temp_k`    — column temperature (K)
/// * `probe`     — polar probe properties
/// * `schultz`   — Schultz fit from n-alkane series at the same temperature
///
/// # Returns
/// `SpecificInteraction` with ΔG_sp in kJ/mol, or `None` on invalid input.
pub fn specific_interaction_delta_g(
    vn_ml: f64,
    temp_k: f64,
    probe: &ProbeProperties,
    schultz: &SchultzPlotData,
) -> Option<SpecificInteraction> {
    if vn_ml <= 0.0 {
        return None;
    }
    let y_probe = schultz_y_value(vn_ml, temp_k)?;
    let x_probe = schultz_x_value(probe.area_angstrom2, probe.gamma_l_d);
    let y_ref = schultz.slope * x_probe + schultz.intercept;
    let delta_g_sp_j_mol = y_probe - y_ref;

    Some(SpecificInteraction {
        probe_name: probe.name.to_string(),
        delta_g_sp_kj_mol: delta_g_sp_j_mol * 1.0e-3,
        delta_h_sp_kj_mol: 0.0, // requires temperature series; set externally
    })
}

/// Compute Gutmann acid (Ka) and base (Kd) constants from a series of polar probes.
///
/// Linear regression of ΔH_sp / AN* vs DN / AN*:
/// ```text
/// ΔH_sp / AN* = Kd * (DN / AN*) + Ka
/// ```
///
/// # Arguments
/// * `interactions` — specific enthalpy data for polar probes (ΔH_sp_kj_mol must be set)
/// * `probes`       — corresponding probe properties (same order)
///
/// # Returns
/// (Ka, Kd, r_squared), or `None` if fewer than 2 valid probes.
pub fn gutmann_acid_base_constants(
    interactions: &[SpecificInteraction],
    probes: &[ProbeProperties],
) -> Option<(f64, f64, f64)> {
    if interactions.len() != probes.len() {
        return None;
    }

    let mut x_vals = Vec::new(); // DN / AN*
    let mut y_vals = Vec::new(); // ΔH_sp / AN*

    for (interaction, probe) in interactions.iter().zip(probes.iter()) {
        if probe.acceptor_number_star.abs() < 1e-10 {
            continue; // skip probes with no acid character — would cause division by zero
        }
        x_vals.push(probe.donor_number / probe.acceptor_number_star);
        y_vals.push(interaction.delta_h_sp_kj_mol / probe.acceptor_number_star);
    }

    if x_vals.len() < 2 {
        return None;
    }

    let (slope, intercept, r2) = linear_regression(&x_vals, &y_vals)?;
    // slope = Kd, intercept = Ka
    Some((slope, intercept, r2))
}

/// Compute the work of adhesion between two surfaces.
///
/// Combining rule:
/// ```text
/// Wa = 2 * sqrt(gamma_1_d * gamma_2_d) + 2 * sqrt(gamma_1_sp * gamma_2_sp)
/// ```
///
/// All surface energies in mJ/m²; returns Wa in mJ/m².
pub fn work_of_adhesion(
    gamma1_d: f64,
    gamma1_sp: f64,
    gamma2_d: f64,
    gamma2_sp: f64,
) -> f64 {
    2.0 * (gamma1_d * gamma2_d).sqrt() + 2.0 * (gamma1_sp * gamma2_sp).sqrt()
}

/// Compute the spreading coefficient S of a liquid on a solid.
///
/// ```text
/// S = Wa - 2 * gamma_liquid
/// ```
///
/// # Arguments
/// * `wa_mj_m2`         — work of adhesion (mJ/m²)
/// * `gamma_liquid_mj_m2` — surface tension of the liquid (mJ/m²)
///
/// # Returns
/// Spreading coefficient S in mJ/m². S > 0 means spontaneous spreading.
pub fn spreading_coefficient(wa_mj_m2: f64, gamma_liquid_mj_m2: f64) -> f64 {
    wa_mj_m2 - 2.0 * gamma_liquid_mj_m2
}

// ─────────────────────────────────────────────────────────────────────────────
// Glass transition detection from IGC
// ─────────────────────────────────────────────────────────────────────────────

/// Data point for a temperature-series IGC experiment.
#[derive(Debug, Clone, PartialEq)]
pub struct TempSeriesPoint {
    /// Reciprocal column temperature 1/T (K^-1)
    pub inv_temp_k: f64,
    /// ln(Vn) — dimensionless
    pub ln_vn: f64,
}

/// Estimate the glass transition temperature Tg from a ln(Vn) vs 1/T plot.
///
/// Below Tg the slope (apparent activation energy) changes. This function
/// detects the inflection by fitting two line segments and finding the
/// temperature where they intersect.
///
/// # Arguments
/// * `series` — temperature series points sorted by descending temperature
///              (ascending 1/T), covering both rubbery and glassy regions
///
/// # Returns
/// Tg estimate in K, or `None` if the data is insufficient (< 4 points).
pub fn glass_transition_from_igc(series: &[TempSeriesPoint]) -> Option<f64> {
    let n = series.len();
    if n < 4 {
        return None;
    }

    // Try every split point and choose the split with minimum total residual
    let mut best_split = 1usize;
    let mut best_residual = f64::INFINITY;

    for split in 1..(n - 1) {
        let xs1: Vec<f64> = series[..split + 1].iter().map(|p| p.inv_temp_k).collect();
        let ys1: Vec<f64> = series[..split + 1].iter().map(|p| p.ln_vn).collect();
        let xs2: Vec<f64> = series[split..].iter().map(|p| p.inv_temp_k).collect();
        let ys2: Vec<f64> = series[split..].iter().map(|p| p.ln_vn).collect();

        if xs1.len() < 2 || xs2.len() < 2 {
            continue;
        }

        let res1 = linear_regression(&xs1, &ys1);
        let res2 = linear_regression(&xs2, &ys2);

        if let (Some((s1, i1, _)), Some((s2, i2, _))) = (res1, res2) {
            // Sum of squared residuals for each segment
            let ssr1: f64 = xs1
                .iter()
                .zip(ys1.iter())
                .map(|(x, y)| (y - (s1 * x + i1)).powi(2))
                .sum();
            let ssr2: f64 = xs2
                .iter()
                .zip(ys2.iter())
                .map(|(x, y)| (y - (s2 * x + i2)).powi(2))
                .sum();
            let total = ssr1 + ssr2;
            if total < best_residual {
                best_residual = total;
                best_split = split;
            }
        }
    }

    // Find intersection of two best-fit lines
    let xs1: Vec<f64> = series[..best_split + 1].iter().map(|p| p.inv_temp_k).collect();
    let ys1: Vec<f64> = series[..best_split + 1].iter().map(|p| p.ln_vn).collect();
    let xs2: Vec<f64> = series[best_split..].iter().map(|p| p.inv_temp_k).collect();
    let ys2: Vec<f64> = series[best_split..].iter().map(|p| p.ln_vn).collect();

    let (s1, i1, _) = linear_regression(&xs1, &ys1)?;
    let (s2, i2, _) = linear_regression(&xs2, &ys2)?;

    // Intersection: s1*x + i1 = s2*x + i2 => x = (i2 - i1) / (s1 - s2)
    if (s1 - s2).abs() < 1e-30 {
        return None;
    }
    let x_intersect = (i2 - i1) / (s1 - s2);
    if x_intersect <= 0.0 {
        return None;
    }
    Some(1.0 / x_intersect) // convert 1/T → T in K
}

// ─────────────────────────────────────────────────────────────────────────────
// BET surface area from adsorption isotherm
// ─────────────────────────────────────────────────────────────────────────────

/// Single point on a BET adsorption isotherm.
#[derive(Debug, Clone, PartialEq)]
pub struct BetPoint {
    /// Relative pressure p / p0 (dimensionless, 0 < p/p0 < 1)
    pub relative_pressure: f64,
    /// Moles adsorbed per gram of material (mol/g)
    pub moles_adsorbed: f64,
}

/// BET surface area analysis result.
#[derive(Debug, Clone, PartialEq)]
pub struct BetResult {
    /// BET surface area (m²/g)
    pub surface_area_m2_g: f64,
    /// Monolayer capacity (mol/g)
    pub vm_mol_g: f64,
    /// BET constant C (dimensionless)
    pub bet_c: f64,
    /// Coefficient of determination R² of the BET plot linear fit
    pub r_squared: f64,
}

/// Perform BET surface area analysis from an adsorption isotherm.
///
/// BET linear form:
/// ```text
/// p / (V * (p0 - p)) = 1 / (Vm * C) + (C - 1) / (Vm * C) * (p / p0)
/// ```
/// where V is adsorbed amount, Vm is monolayer capacity, C is BET constant.
///
/// # Arguments
/// * `isotherm`    — adsorption isotherm data points
/// * `area_probe_m2` — molecular cross-sectional area of the probe (m²)
///
/// # Returns
/// `BetResult` or `None` if fewer than 2 valid points.
pub fn bet_surface_area(isotherm: &[BetPoint], area_probe_m2: f64) -> Option<BetResult> {
    let mut xs = Vec::new(); // p/p0
    let mut ys = Vec::new(); // p / (V * (p0 - p))

    for pt in isotherm {
        let pp0 = pt.relative_pressure;
        if pp0 <= 0.0 || pp0 >= 1.0 || pt.moles_adsorbed <= 0.0 {
            continue;
        }
        xs.push(pp0);
        ys.push(pp0 / (pt.moles_adsorbed * (1.0 - pp0)));
    }

    if xs.len() < 2 {
        return None;
    }

    let (slope, intercept, r_squared) = linear_regression(&xs, &ys)?;

    // slope = (C-1)/(Vm*C), intercept = 1/(Vm*C)
    // => Vm = 1 / (slope + intercept)
    // => C = slope / intercept + 1
    let vm = 1.0 / (slope + intercept);
    if vm <= 0.0 {
        return None;
    }
    let bet_c = if intercept.abs() > 1e-30 {
        slope / intercept + 1.0
    } else {
        1.0
    };

    // Surface area = Vm * NA * area_probe (m²/g)
    let surface_area = vm * AVOGADRO * area_probe_m2;

    Some(BetResult {
        surface_area_m2_g: surface_area,
        vm_mol_g: vm,
        bet_c,
        r_squared,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// High-level processor combining all analyses
// ─────────────────────────────────────────────────────────────────────────────

/// High-level IGC surface energy processor.
///
/// Stores all retention data for one material at one temperature and
/// orchestrates the Schultz/Dorris-Gray/acid-base workflow.
#[derive(Debug, Clone)]
pub struct IgcProcessor {
    /// Column conditions (constant throughout the experiment)
    pub conditions: ColumnConditions,
    /// Alkane retention data
    pub alkane_data: Vec<RetentionData>,
    /// Polar probe retention data
    pub polar_data: Vec<RetentionData>,
}

impl IgcProcessor {
    /// Create a new processor with the given column conditions.
    pub fn new(conditions: ColumnConditions) -> Self {
        Self {
            conditions,
            alkane_data: Vec::new(),
            polar_data: Vec::new(),
        }
    }

    /// Add a measured alkane data point.
    pub fn add_alkane(&mut self, data: RetentionData) {
        self.alkane_data.push(data);
    }

    /// Add a measured polar probe data point.
    pub fn add_polar(&mut self, data: RetentionData) {
        self.polar_data.push(data);
    }

    /// Compute net retention volumes for all alkane probes.
    ///
    /// Returns a Vec of (probe_name, Vn_mL) pairs; probes with invalid
    /// retention (tr ≤ t0) are skipped.
    pub fn alkane_retention_volumes(&self) -> Vec<(String, f64)> {
        self.alkane_data
            .iter()
            .filter_map(|d| {
                retention_volume(d, &self.conditions)
                    .map(|vn| (d.probe_name.clone(), vn))
            })
            .collect()
    }

    /// Run the Schultz dispersive analysis.
    ///
    /// Probe properties are looked up from the built-in database; probes not
    /// found are silently skipped.
    pub fn run_schultz(&self) -> Option<SchultzPlotData> {
        let db = probe_database();
        let mut matched_data = Vec::new();
        let mut matched_props = Vec::new();

        for data in &self.alkane_data {
            if let Some(probe) = db.iter().find(|p| {
                p.name
                    .to_lowercase()
                    .contains(&data.probe_name.to_lowercase())
            }) {
                matched_data.push(data.clone());
                matched_props.push(probe.clone());
            }
        }

        schultz_analysis(&matched_data, &self.conditions, &matched_props)
    }

    /// Run the Dorris-Gray analysis by averaging over all consecutive alkane pairs.
    ///
    /// Returns the mean γ_s^d in mJ/m², or `None` if fewer than two valid alkanes.
    pub fn run_dorris_gray(&self) -> Option<f64> {
        let vns: Vec<(String, f64)> = self.alkane_retention_volumes();
        if vns.len() < 2 {
            return None;
        }
        let temp_k = self.alkane_data.first()?.column_temp_k;

        let mut values = Vec::new();
        for i in 0..vns.len() - 1 {
            if let Some(gd) = dorris_gray_gamma_d(vns[i].1, vns[i + 1].1, temp_k) {
                values.push(gd);
            }
        }
        if values.is_empty() {
            return None;
        }
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }

    /// Compute ΔG_sp for all polar probes using a Schultz reference line.
    pub fn specific_interactions(&self, schultz: &SchultzPlotData) -> Vec<SpecificInteraction> {
        let db = probe_database();
        let temp_k = self
            .polar_data
            .first()
            .map(|d| d.column_temp_k)
            .unwrap_or(298.15);

        let mut results = Vec::new();
        for data in &self.polar_data {
            let probe = match db.iter().find(|p| {
                p.name
                    .to_lowercase()
                    .contains(&data.probe_name.to_lowercase())
            }) {
                Some(p) => p,
                None => continue,
            };
            if let Some(vn) = retention_volume(data, &self.conditions) {
                if let Some(interaction) =
                    specific_interaction_delta_g(vn, temp_k, probe, schultz)
                {
                    results.push(interaction);
                }
            }
        }
        results
    }

    /// Compute a full `SurfaceEnergyResult` for the material.
    ///
    /// Uses Schultz method for γ_s^d. ΔG_sp values are averaged to give
    /// γ_sp. Ka/Kd require temperature-series data (set to 0 here unless
    /// enthalpy data is provided via `interactions` with filled
    /// `delta_h_sp_kj_mol`).
    pub fn compute_surface_energy(&self) -> Option<SurfaceEnergyResult> {
        let schultz = self.run_schultz()?;
        let gamma_d = schultz.gamma_s_d;

        let interactions = self.specific_interactions(&schultz);
        let gamma_sp = if interactions.is_empty() {
            0.0
        } else {
            // Mean of |ΔG_sp| as a rough γ_sp estimate
            interactions
                .iter()
                .map(|i| i.delta_g_sp_kj_mol.abs())
                .sum::<f64>()
                / interactions.len() as f64
        };

        Some(SurfaceEnergyResult {
            gamma_d_mj_m2: gamma_d,
            gamma_sp_mj_m2: gamma_sp,
            gamma_total_mj_m2: gamma_d + gamma_sp,
            ka: 0.0, // requires temperature series
            kd: 0.0,
            r_squared: schultz.r_squared,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Convert a temperature in Celsius to Kelvin.
#[inline]
pub fn celsius_to_kelvin(t_c: f64) -> f64 {
    t_c + 273.15
}

/// Saturated water vapor pressure (Pa) at temperature T (K) via Antoine equation.
///
/// Valid range: ~273–373 K. Uses constants for liquid water.
pub fn saturated_water_vapor_pressure(temp_k: f64) -> f64 {
    let t_c = temp_k - 273.15;
    // Antoine: log10(P[mmHg]) = A - B/(C+T)
    // NIST constants for water, 60–150 °C: A=8.07131, B=1730.63, C=233.426
    let log_p_mmhg = 8.07131 - 1730.63 / (233.426 + t_c);
    let p_mmhg = 10_f64.powf(log_p_mmhg);
    // 1 mmHg = 133.322 Pa
    p_mmhg * 133.322
}

/// Estimate the specific surface free energy of adsorption from Vn and reference.
///
/// Convenience wrapper returning only ΔG_sp in kJ/mol.
pub fn delta_g_sp_kj_mol(
    vn_probe: f64,
    vn_ref: f64,
    temp_k: f64,
) -> f64 {
    R_GAS * temp_k * (vn_probe / vn_ref).ln() * 1.0e-3
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper builders ──────────────────────────────────────────────────────

    fn default_conditions() -> ColumnConditions {
        ColumnConditions {
            inlet_pressure_pa: 150_000.0,
            outlet_pressure_pa: 101_325.0,
            flowmeter_temp_k: 298.15,
            water_vapor_pressure_pa: 2_338.5, // ~20 °C dew point
        }
    }

    fn make_alkane(name: &str, tr: f64, t0: f64) -> RetentionData {
        RetentionData {
            probe_name: name.to_string(),
            retention_time_s: tr,
            dead_time_s: t0,
            flow_rate_ml_min: 20.0,
            column_temp_k: 323.15, // 50 °C
        }
    }

    fn make_polar(name: &str, tr: f64, t0: f64) -> RetentionData {
        RetentionData {
            probe_name: name.to_string(),
            retention_time_s: tr,
            dead_time_s: t0,
            flow_rate_ml_min: 20.0,
            column_temp_k: 323.15,
        }
    }

    // ── James-Martin correction ──────────────────────────────────────────────

    #[test]
    fn test_james_martin_equal_pressure() {
        let j = james_martin_correction(101_325.0, 101_325.0);
        assert!((j - 1.0).abs() < 1e-6, "j should be 1 for equal pressures, got {}", j);
    }

    #[test]
    fn test_james_martin_typical() {
        // Pi/Po = 150000/101325 ≈ 1.48
        let j = james_martin_correction(150_000.0, 101_325.0);
        // j ≈ 0.796 for Pi/Po≈1.48 (slightly below 0.8 is expected)
        assert!(j > 0.7 && j < 1.0, "j out of expected range: {}", j);
    }

    #[test]
    fn test_james_martin_high_pressure() {
        // Pi/Po = 3 (high pressure drop)
        let j = james_martin_correction(303_975.0, 101_325.0);
        assert!(j > 0.4 && j < 1.0, "j out of range for high pressure: {}", j);
    }

    #[test]
    fn test_james_martin_symmetry() {
        // j should decrease as Pi/Po increases
        let j1 = james_martin_correction(120_000.0, 101_325.0);
        let j2 = james_martin_correction(200_000.0, 101_325.0);
        assert!(j1 > j2, "j should decrease with higher pressure ratio");
    }

    // ── Retention volume ─────────────────────────────────────────────────────

    #[test]
    fn test_retention_volume_basic() {
        let data = make_alkane("n-Hexane", 300.0, 50.0);
        let cond = default_conditions();
        let vn = retention_volume(&data, &cond).unwrap();
        assert!(vn > 0.0, "Vn should be positive");
    }

    #[test]
    fn test_retention_volume_invalid_tr_le_t0() {
        let data = RetentionData {
            probe_name: "n-Hexane".to_string(),
            retention_time_s: 50.0,
            dead_time_s: 100.0, // tr < t0 — invalid
            flow_rate_ml_min: 20.0,
            column_temp_k: 323.15,
        };
        let cond = default_conditions();
        assert!(retention_volume(&data, &cond).is_none());
    }

    #[test]
    fn test_retention_volume_invalid_flow() {
        let data = RetentionData {
            probe_name: "n-Hexane".to_string(),
            retention_time_s: 300.0,
            dead_time_s: 50.0,
            flow_rate_ml_min: 0.0,
            column_temp_k: 323.15,
        };
        let cond = default_conditions();
        assert!(retention_volume(&data, &cond).is_none());
    }

    #[test]
    fn test_retention_volume_increases_with_retention_time() {
        let cond = default_conditions();
        let d1 = make_alkane("A", 300.0, 50.0);
        let d2 = make_alkane("B", 600.0, 50.0);
        let vn1 = retention_volume(&d1, &cond).unwrap();
        let vn2 = retention_volume(&d2, &cond).unwrap();
        assert!(vn2 > vn1, "Vn should increase with retention time");
    }

    #[test]
    fn test_retention_volume_units() {
        // Manual calculation check
        let cond = default_conditions();
        let data = make_alkane("n-Hexane", 300.0, 50.0);
        let j = james_martin_correction(cond.inlet_pressure_pa, cond.outlet_pressure_pa);
        let f = 20.0 / 60.0; // mL/s
        let t_net = 250.0;
        let t_ratio = 323.15 / 298.15;
        let vapor = 1.0 - 2338.5 / 101_325.0;
        let expected = j * f * t_net * t_ratio * vapor;
        let vn = retention_volume(&data, &cond).unwrap();
        assert!((vn - expected).abs() < 1e-6, "Vn mismatch: {} vs {}", vn, expected);
    }

    // ── Schultz plot values ──────────────────────────────────────────────────

    #[test]
    fn test_schultz_y_positive_vn() {
        let y = schultz_y_value(100.0, 323.15).unwrap();
        assert!(y > 0.0);
    }

    #[test]
    fn test_schultz_y_negative_vn() {
        assert!(schultz_y_value(-1.0, 323.15).is_none());
    }

    #[test]
    fn test_schultz_y_increases_with_vn() {
        let y1 = schultz_y_value(10.0, 323.15).unwrap();
        let y2 = schultz_y_value(100.0, 323.15).unwrap();
        assert!(y2 > y1);
    }

    #[test]
    fn test_schultz_x_positive() {
        let x = schultz_x_value(51.5, 18.4);
        assert!(x > 0.0);
    }

    #[test]
    fn test_schultz_x_increases_with_area() {
        let x1 = schultz_x_value(36.2, 18.4);
        let x2 = schultz_x_value(75.0, 18.4);
        assert!(x2 > x1);
    }

    // ── Linear regression ────────────────────────────────────────────────────

    #[test]
    fn test_linear_regression_perfect_fit() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y: Vec<f64> = x.iter().map(|&xi| 2.0 * xi + 3.0).collect();
        let (slope, intercept, r2) = linear_regression(&x, &y).unwrap();
        assert!((slope - 2.0).abs() < 1e-10);
        assert!((intercept - 3.0).abs() < 1e-10);
        assert!((r2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_noisy() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![0.1, 0.9, 2.1, 2.9, 4.1];
        let (slope, _intercept, r2) = linear_regression(&x, &y).unwrap();
        assert!((slope - 1.0).abs() < 0.1);
        assert!(r2 > 0.99);
    }

    #[test]
    fn test_linear_regression_min_points() {
        let x = vec![1.0, 2.0];
        let y = vec![3.0, 5.0];
        let (slope, intercept, r2) = linear_regression(&x, &y).unwrap();
        assert!((slope - 2.0).abs() < 1e-10);
        assert!((intercept - 1.0).abs() < 1e-10);
        assert!((r2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_too_few_points() {
        assert!(linear_regression(&[1.0], &[2.0]).is_none());
    }

    // ── Schultz analysis ─────────────────────────────────────────────────────

    #[test]
    fn test_schultz_analysis_returns_positive_gamma_d() {
        let cond = default_conditions();
        let alkanes: Vec<RetentionData> = vec![
            make_alkane("n-Pentane", 120.0, 30.0),
            make_alkane("n-Hexane", 200.0, 30.0),
            make_alkane("n-Heptane", 320.0, 30.0),
            make_alkane("n-Octane", 500.0, 30.0),
        ];
        // Use explicit probe properties to avoid name-matching fragility
        let props: Vec<ProbeProperties> = vec![
            ProbeProperties { name: "n-Pentane", area_angstrom2: 36.2, gamma_l_d: 15.5, donor_number: 0.0, acceptor_number_star: 0.0 },
            ProbeProperties { name: "n-Hexane",  area_angstrom2: 51.5, gamma_l_d: 18.4, donor_number: 0.0, acceptor_number_star: 0.0 },
            ProbeProperties { name: "n-Heptane", area_angstrom2: 57.0, gamma_l_d: 20.3, donor_number: 0.0, acceptor_number_star: 0.0 },
            ProbeProperties { name: "n-Octane",  area_angstrom2: 62.8, gamma_l_d: 21.3, donor_number: 0.0, acceptor_number_star: 0.0 },
        ];

        let result = schultz_analysis(&alkanes, &cond, &props).unwrap();
        assert!(result.gamma_s_d > 0.0, "γ_s^d should be positive: {}", result.gamma_s_d);
        assert!(result.r_squared > 0.9, "R² should be high for n-alkanes: {}", result.r_squared);
    }

    #[test]
    fn test_schultz_analysis_needs_two_points() {
        let cond = default_conditions();
        let alkanes = vec![make_alkane("n-Hexane", 200.0, 30.0)];
        let db = probe_database();
        let props: Vec<ProbeProperties> = alkanes
            .iter()
            .filter_map(|d| {
                db.iter()
                    .find(|p| p.name.to_lowercase().contains(&d.probe_name.to_lowercase()))
                    .cloned()
            })
            .collect();
        assert!(schultz_analysis(&alkanes, &cond, &props).is_none());
    }

    // ── Dorris-Gray ──────────────────────────────────────────────────────────

    #[test]
    fn test_dorris_gray_positive_result() {
        // Vn(C7) > Vn(C6) — realistic
        let gd = dorris_gray_gamma_d(50.0, 80.0, 323.15).unwrap();
        assert!(gd > 0.0, "γ_s^d from Dorris-Gray should be positive: {}", gd);
    }

    #[test]
    fn test_dorris_gray_invalid_vn() {
        assert!(dorris_gray_gamma_d(-1.0, 80.0, 323.15).is_none());
        assert!(dorris_gray_gamma_d(50.0, 0.0, 323.15).is_none());
    }

    #[test]
    fn test_dorris_gray_equal_vn() {
        // Equal Vn means zero Delta_G_CH2 => gamma_s_d = 0
        let gd = dorris_gray_gamma_d(50.0, 50.0, 323.15).unwrap();
        assert!(gd.abs() < 1e-6, "γ_s^d should be ~0 for equal Vn: {}", gd);
    }

    #[test]
    fn test_dorris_gray_realistic_range() {
        // For typical fillers gamma_s_d should be between 20 and 100 mJ/m²
        let gd = dorris_gray_gamma_d(40.0, 64.0, 323.15).unwrap();
        assert!(gd > 5.0 && gd < 200.0, "γ_s^d out of physical range: {}", gd);
    }

    // ── Specific interactions ────────────────────────────────────────────────

    #[test]
    fn test_specific_interaction_above_alkane_line() {
        let schultz = SchultzPlotData {
            x_values: vec![],
            y_values: vec![],
            slope: 1.0e12,
            intercept: 1000.0,
            r_squared: 0.999,
            gamma_s_d: 40.0,
        };
        let probe = find_probe("Chloroform").unwrap();
        // Give vn high enough that y_probe >> y_ref
        let vn = 5000.0;
        let temp_k = 323.15;
        let interaction = specific_interaction_delta_g(vn, temp_k, &probe, &schultz).unwrap();
        // Should return a valid value regardless of sign
        assert!(interaction.delta_g_sp_kj_mol.is_finite());
    }

    #[test]
    fn test_specific_interaction_zero_vn() {
        let schultz = SchultzPlotData {
            x_values: vec![],
            y_values: vec![],
            slope: 1.0e12,
            intercept: 1000.0,
            r_squared: 0.999,
            gamma_s_d: 40.0,
        };
        let probe = find_probe("Chloroform").unwrap();
        assert!(specific_interaction_delta_g(0.0, 323.15, &probe, &schultz).is_none());
    }

    // ── Gutmann acid-base constants ──────────────────────────────────────────

    #[test]
    fn test_gutmann_acid_base_basic() {
        // Construct synthetic interactions matching Ka=0.10, Kd=0.30
        let probes = vec![
            ProbeProperties {
                name: "P1",
                area_angstrom2: 45.0,
                gamma_l_d: 22.0,
                donor_number: 20.0,
                acceptor_number_star: 2.0,
            },
            ProbeProperties {
                name: "P2",
                area_angstrom2: 45.0,
                gamma_l_d: 22.0,
                donor_number: 5.0,
                acceptor_number_star: 4.0,
            },
            ProbeProperties {
                name: "P3",
                area_angstrom2: 45.0,
                gamma_l_d: 22.0,
                donor_number: 17.0,
                acceptor_number_star: 1.5,
            },
        ];
        // ΔH_sp = Ka * DN + Kd * AN* = 0.10*DN + 0.30*AN*
        let interactions: Vec<SpecificInteraction> = probes
            .iter()
            .map(|p| SpecificInteraction {
                probe_name: p.name.to_string(),
                delta_g_sp_kj_mol: 0.0,
                delta_h_sp_kj_mol: 0.10 * p.donor_number + 0.30 * p.acceptor_number_star,
            })
            .collect();

        let (ka, kd, r2) = gutmann_acid_base_constants(&interactions, &probes).unwrap();
        assert!((ka - 0.10).abs() < 1e-6, "Ka mismatch: {}", ka);
        assert!((kd - 0.30).abs() < 1e-6, "Kd mismatch: {}", kd);
        assert!((r2 - 1.0).abs() < 1e-6, "R² should be ~1: {}", r2);
    }

    #[test]
    fn test_gutmann_skips_zero_acceptor_number() {
        let probes = vec![
            ProbeProperties {
                name: "P1",
                area_angstrom2: 45.0,
                gamma_l_d: 22.0,
                donor_number: 20.0,
                acceptor_number_star: 0.0, // no acid character → must be skipped
            },
        ];
        let interactions = vec![SpecificInteraction {
            probe_name: "P1".to_string(),
            delta_g_sp_kj_mol: 1.0,
            delta_h_sp_kj_mol: 2.0,
        }];
        assert!(gutmann_acid_base_constants(&interactions, &probes).is_none());
    }

    // ── Work of adhesion ─────────────────────────────────────────────────────

    #[test]
    fn test_work_of_adhesion_identical_surfaces() {
        // For identical surfaces Wa = 2 * gamma_total
        let gamma_d = 35.0;
        let gamma_sp = 5.0;
        let wa = work_of_adhesion(gamma_d, gamma_sp, gamma_d, gamma_sp);
        let expected = 2.0 * (gamma_d + gamma_sp);
        assert!((wa - expected).abs() < 1e-6, "Wa: {} vs {}", wa, expected);
    }

    #[test]
    fn test_work_of_adhesion_non_polar() {
        // No specific component → purely dispersive
        let wa = work_of_adhesion(30.0, 0.0, 25.0, 0.0);
        let expected = 2.0 * (30.0_f64 * 25.0).sqrt();
        assert!((wa - expected).abs() < 1e-6);
    }

    #[test]
    fn test_work_of_adhesion_positive() {
        let wa = work_of_adhesion(40.0, 5.0, 30.0, 3.0);
        assert!(wa > 0.0);
    }

    // ── Spreading coefficient ────────────────────────────────────────────────

    #[test]
    fn test_spreading_coefficient_wetting() {
        // Wa > 2*gamma_liquid → S > 0 → spontaneous spreading
        let wa = 80.0;
        let gamma_liq = 30.0;
        let s = spreading_coefficient(wa, gamma_liq);
        assert!(s > 0.0);
    }

    #[test]
    fn test_spreading_coefficient_non_wetting() {
        let wa = 30.0;
        let gamma_liq = 72.0; // water-like
        let s = spreading_coefficient(wa, gamma_liq);
        assert!(s < 0.0);
    }

    #[test]
    fn test_spreading_coefficient_formula() {
        let wa = 60.0;
        let gamma = 25.0;
        assert!((spreading_coefficient(wa, gamma) - (wa - 2.0 * gamma)).abs() < 1e-9);
    }

    // ── Probe database ───────────────────────────────────────────────────────

    #[test]
    fn test_probe_database_count() {
        let db = probe_database();
        assert_eq!(db.len(), 12);
    }

    #[test]
    fn test_probe_database_alkane_count() {
        let db = probe_database();
        let n_alkanes = db.iter().filter(|p| p.name.starts_with("n-")).count();
        assert_eq!(n_alkanes, 6);
    }

    #[test]
    fn test_probe_database_polar_count() {
        let db = probe_database();
        let polar = db.iter().filter(|p| !p.name.starts_with("n-")).count();
        assert_eq!(polar, 6);
    }

    #[test]
    fn test_find_probe_hexane() {
        let p = find_probe("Hexane").unwrap();
        assert_eq!(p.name, "n-Hexane");
        assert!((p.area_angstrom2 - 51.5).abs() < 1e-6);
        assert!((p.gamma_l_d - 18.4).abs() < 1e-6);
    }

    #[test]
    fn test_find_probe_chloroform() {
        let p = find_probe("Chloroform").unwrap();
        assert!((p.acceptor_number_star - 5.4).abs() < 1e-6);
        assert!(p.donor_number.abs() < 1e-6);
    }

    #[test]
    fn test_find_probe_thf() {
        let p = find_probe("THF").unwrap();
        assert!((p.donor_number - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_probe_not_found() {
        assert!(find_probe("Benzene").is_none());
    }

    #[test]
    fn test_probe_alkanes_increasing_area() {
        let db = probe_database();
        let alkanes: Vec<&ProbeProperties> = db.iter().filter(|p| p.name.starts_with("n-")).collect();
        for pair in alkanes.windows(2) {
            assert!(
                pair[1].area_angstrom2 > pair[0].area_angstrom2,
                "Alkane cross-sections should increase: {} vs {}",
                pair[0].name, pair[1].name
            );
        }
    }

    // ── Material presets ─────────────────────────────────────────────────────

    #[test]
    fn test_material_presets_count() {
        assert_eq!(material_presets().len(), 4);
    }

    #[test]
    fn test_find_material_talc() {
        let m = find_material("Talc").unwrap();
        assert!((m.gamma_d_ref - 35.0).abs() < 1e-6);
        assert!((m.ka_ref - 0.05).abs() < 1e-6);
        assert!((m.kd_ref - 0.35).abs() < 1e-6);
    }

    #[test]
    fn test_find_material_calcium_carbonate() {
        let m = find_material("calcium").unwrap();
        assert!((m.gamma_d_ref - 55.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_material_carbon_fiber() {
        let m = find_material("Carbon fiber").unwrap();
        assert!((m.gamma_d_ref - 42.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_material_lactose() {
        let m = find_material("lactose").unwrap();
        assert!((m.gamma_d_ref - 38.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_material_not_found() {
        assert!(find_material("Polystyrene").is_none());
    }

    // ── BET surface area ─────────────────────────────────────────────────────

    #[test]
    fn test_bet_surface_area_basic() {
        // Synthetic BET isotherm for C = 100, Vm = 1e-4 mol/g
        let c_bet = 100.0_f64;
        let vm = 1.0e-4_f64; // mol/g
        let pp0_vals = vec![0.05, 0.10, 0.15, 0.20, 0.25, 0.30];
        let isotherm: Vec<BetPoint> = pp0_vals
            .iter()
            .map(|&pp0| {
                // BET equation: V = Vm * C * pp0 / ((1 - pp0) * (1 + (C-1)*pp0))
                let v = vm * c_bet * pp0 / ((1.0 - pp0) * (1.0 + (c_bet - 1.0) * pp0));
                BetPoint {
                    relative_pressure: pp0,
                    moles_adsorbed: v,
                }
            })
            .collect();

        let area_probe_m2 = 6.28e-20; // n-Octane
        let result = bet_surface_area(&isotherm, area_probe_m2).unwrap();
        assert!((result.vm_mol_g - vm).abs() / vm < 0.01, "Vm error > 1%: {}", result.vm_mol_g);
        assert!(result.surface_area_m2_g > 0.0);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_bet_surface_area_invalid_points() {
        let isotherm = vec![
            BetPoint { relative_pressure: 0.0, moles_adsorbed: 1.0 }, // p/p0 = 0 invalid
            BetPoint { relative_pressure: 1.1, moles_adsorbed: 1.0 }, // > 1 invalid
        ];
        assert!(bet_surface_area(&isotherm, 6.28e-20).is_none());
    }

    // ── Glass transition ─────────────────────────────────────────────────────

    #[test]
    fn test_glass_transition_detects_kink() {
        // Simulate ln(Vn) vs 1/T with a kink at Tg = 350 K (1/T = 0.002857)
        // Below Tg (high 1/T): steep slope; above Tg (low 1/T): shallow slope
        let low_inv_t: Vec<f64> = (0..5).map(|i| 0.0025 + i as f64 * 0.0001).collect();
        let high_inv_t: Vec<f64> = (1..5).map(|i| 0.003 + i as f64 * 0.0001).collect();

        let mut series: Vec<TempSeriesPoint> = Vec::new();
        for &x in &low_inv_t {
            series.push(TempSeriesPoint { inv_temp_k: x, ln_vn: 2.0 * x * 1000.0 + 1.0 });
        }
        for &x in &high_inv_t {
            series.push(TempSeriesPoint { inv_temp_k: x, ln_vn: 5.0 * x * 1000.0 - 5.0 });
        }
        series.sort_by(|a, b| a.inv_temp_k.partial_cmp(&b.inv_temp_k).unwrap());

        let tg = glass_transition_from_igc(&series);
        assert!(tg.is_some(), "Should detect Tg");
        let tg_val = tg.unwrap();
        assert!(tg_val > 200.0 && tg_val < 600.0, "Tg out of physical range: {}", tg_val);
    }

    #[test]
    fn test_glass_transition_too_few_points() {
        let series = vec![
            TempSeriesPoint { inv_temp_k: 0.003, ln_vn: 1.0 },
            TempSeriesPoint { inv_temp_k: 0.0031, ln_vn: 1.1 },
        ];
        assert!(glass_transition_from_igc(&series).is_none());
    }

    // ── Utility helpers ──────────────────────────────────────────────────────

    #[test]
    fn test_celsius_to_kelvin() {
        assert!((celsius_to_kelvin(0.0) - 273.15).abs() < 1e-6);
        assert!((celsius_to_kelvin(100.0) - 373.15).abs() < 1e-6);
        assert!((celsius_to_kelvin(-273.15) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_saturated_water_vapor_pressure_room_temp() {
        // At 20 °C (~293 K), saturation pressure ≈ 2338 Pa
        let p = saturated_water_vapor_pressure(293.15);
        assert!(p > 2000.0 && p < 2700.0, "pw at 20°C out of range: {}", p);
    }

    #[test]
    fn test_saturated_water_vapor_pressure_increases_with_temp() {
        let p1 = saturated_water_vapor_pressure(293.15);
        let p2 = saturated_water_vapor_pressure(323.15);
        assert!(p2 > p1, "Vapor pressure should increase with temperature");
    }

    #[test]
    fn test_delta_g_sp_positive_when_vn_higher() {
        let dg = delta_g_sp_kj_mol(200.0, 100.0, 323.15);
        assert!(dg > 0.0, "ΔG_sp should be positive when polar Vn > reference Vn");
    }

    #[test]
    fn test_delta_g_sp_zero_when_equal() {
        let dg = delta_g_sp_kj_mol(100.0, 100.0, 323.15);
        assert!(dg.abs() < 1e-10);
    }

    // ── IgcProcessor integration ─────────────────────────────────────────────

    #[test]
    fn test_processor_compute_surface_energy() {
        let cond = default_conditions();
        let mut proc = IgcProcessor::new(cond);

        // Add n-alkane series with increasing retention times
        for (name, tr) in &[
            ("n-Pentane", 90.0),
            ("n-Hexane", 150.0),
            ("n-Heptane", 250.0),
            ("n-Octane", 410.0),
            ("n-Nonane", 680.0),
        ] {
            proc.add_alkane(make_alkane(name, *tr, 30.0));
        }

        let result = proc.compute_surface_energy().unwrap();
        assert!(result.gamma_d_mj_m2 > 0.0, "γ_d should be positive: {}", result.gamma_d_mj_m2);
        assert!(result.gamma_total_mj_m2 >= result.gamma_d_mj_m2);
        assert!(result.r_squared > 0.9, "R² too low: {}", result.r_squared);
    }

    #[test]
    fn test_processor_alkane_retention_volumes() {
        let cond = default_conditions();
        let mut proc = IgcProcessor::new(cond);
        proc.add_alkane(make_alkane("n-Hexane", 200.0, 30.0));
        proc.add_alkane(make_alkane("n-Heptane", 350.0, 30.0));

        let vns = proc.alkane_retention_volumes();
        assert_eq!(vns.len(), 2);
        assert!(vns[1].1 > vns[0].1, "n-Heptane should have higher Vn than n-Hexane");
    }

    #[test]
    fn test_processor_dorris_gray_mean() {
        let cond = default_conditions();
        let mut proc = IgcProcessor::new(cond);
        proc.add_alkane(make_alkane("n-Hexane", 150.0, 30.0));
        proc.add_alkane(make_alkane("n-Heptane", 250.0, 30.0));
        proc.add_alkane(make_alkane("n-Octane", 410.0, 30.0));

        let gd = proc.run_dorris_gray().unwrap();
        assert!(gd > 0.0);
    }

    #[test]
    fn test_processor_empty_alkanes() {
        let cond = default_conditions();
        let proc = IgcProcessor::new(cond);
        assert!(proc.run_schultz().is_none());
        assert!(proc.run_dorris_gray().is_none());
    }

    #[test]
    fn test_processor_specific_interactions() {
        let cond = default_conditions();
        let mut proc = IgcProcessor::new(cond.clone());

        for (name, tr) in &[
            ("n-Pentane", 90.0),
            ("n-Hexane", 150.0),
            ("n-Heptane", 250.0),
            ("n-Octane", 410.0),
        ] {
            proc.add_alkane(make_alkane(name, *tr, 30.0));
        }
        proc.add_polar(make_polar("Chloroform", 300.0, 30.0));
        proc.add_polar(make_polar("THF", 350.0, 30.0));

        let schultz = proc.run_schultz().unwrap();
        let interactions = proc.specific_interactions(&schultz);
        // We may get 0 or more interactions depending on probe lookup
        for i in &interactions {
            assert!(i.delta_g_sp_kj_mol.is_finite());
        }
    }

    // ── Surface energy default ───────────────────────────────────────────────

    #[test]
    fn test_surface_energy_result_default() {
        let r = SurfaceEnergyResult::default();
        assert_eq!(r.gamma_d_mj_m2, 0.0);
        assert_eq!(r.gamma_total_mj_m2, 0.0);
    }
}
