// trace:FR-CRYO-BOILOFF | ai:claude
//! # Cryogenic Helium Boiloff Predictor
//!
//! Predictive modeling for liquid helium (LHe) boiloff rate in cryogenic systems
//! such as MRI magnets, superconducting accelerators, and dilution refrigerators,
//! based on heat load analysis and Dewar thermal modeling.
//!
//! ## Physics Background
//!
//! Liquid helium-4 boils at 4.222 K at 1 atm. The latent heat of vaporization
//! is ~20,700 J/kg, meaning even small heat loads cause significant boiloff.
//! The boiloff rate is: dm/dt = Q_total / L, where L is the latent heat.
//!
//! At 2.172 K (the lambda point), helium undergoes a phase transition from
//! He-I (normal fluid) to He-II (superfluid) with extraordinary thermal
//! conductivity.
//!
//! ## Key Components
//!
//! - **HeliumProperties** - LHe thermodynamic data at cryogenic temperatures
//! - **HeatLoadCalculator** - Multi-source heat load analysis (radiation, conduction, residual gas)
//! - **BoiloffPredictor** - LHe consumption rate prediction
//! - **DewarModel** - Cryostat thermal geometry model
//! - **ThermalConductivityModels** - k(T) for cryogenic materials
//! - **MliPerformance** - Multi-layer insulation modeling
//! - **LevelSensor** - Helium level monitoring and prediction
//! - **QuenchAnalyzer** - Magnet quench helium loss estimation
//! - **CoolingPowerBudget** - Cryocooler capacity analysis
//! - **HeliumRecoverySystem** - Recovery and reliquefaction economics

/// Stefan-Boltzmann constant [W/(m^2·K^4)]
const STEFAN_BOLTZMANN: f64 = 5.670374419e-8;

/// Boltzmann constant [J/K]
const BOLTZMANN_K: f64 = 1.380649e-23;

/// Universal gas constant [J/(mol·K)]
const GAS_CONSTANT: f64 = 8.314462;

/// Helium-4 molar mass [kg/mol]
const HE4_MOLAR_MASS: f64 = 4.002602e-3;

/// Standard atmosphere [Pa]
const ATM_PA: f64 = 101325.0;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Support material for conduction heat load calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SupportMaterial {
    StainlessSteel304,
    CopperRrr100,
    G10Fiberglass,
    Nylon,
    Aluminum6061,
}

/// Neck tube material for Dewar models.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NeckMaterial {
    StainlessSteel304,
    G10Fiberglass,
    ThinWallStainless,
}

// ---------------------------------------------------------------------------
// HeliumProperties
// ---------------------------------------------------------------------------

/// Thermodynamic properties of liquid helium-4.
pub struct HeliumProperties;

impl HeliumProperties {
    /// Latent heat of vaporization at 4.2 K and 1 atm [J/kg].
    pub fn latent_heat_j_per_kg() -> f64 {
        20_700.0
    }

    /// Liquid density at 4.2 K and 1 atm [kg/m^3].
    pub fn density_kg_per_m3() -> f64 {
        125.0
    }

    /// Specific heat capacity at constant pressure as a function of temperature [J/(kg·K)].
    ///
    /// Uses a piecewise model: near the lambda point (2.172 K), Cp diverges.
    /// Below 2.172 K (He-II): Cp ≈ 5200 × (T/2.172)^5.6 J/(kg·K)
    /// Above 2.172 K (He-I): Cp ≈ 5200 × (1 + 0.4 × (T - 2.172)) J/(kg·K)
    /// simplified model capturing qualitative behavior.
    pub fn specific_heat_j_per_kg_k(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        let lambda = 2.172;
        if t_k < lambda {
            // He-II: Cp rises steeply toward lambda point (Debye T^3 at very low T)
            // Simplified power law
            5200.0 * (t_k / lambda).powf(5.6)
        } else if t_k < 4.5 {
            // He-I normal liquid: roughly constant near 5200 J/(kg·K)
            5200.0 * (1.0 + 0.4 * (t_k - lambda))
        } else {
            // Above boiling: gas phase, Cp ≈ 5/2 R/M ≈ 5193 J/(kg·K) for ideal monatomic
            5193.0
        }
    }

    /// Boiling point at a given pressure using Clausius-Clapeyron relation [K].
    ///
    /// T_bp(P) ≈ T_0 / (1 - (R T_0 / L_m) ln(P/P_0))
    /// where T_0 = 4.222 K, P_0 = 1 atm, L_m = latent heat per mole.
    pub fn boiling_point_k(pressure_atm: f64) -> f64 {
        if pressure_atm <= 0.0 {
            return 0.0;
        }
        let t0 = 4.222; // K at 1 atm
        let l_m = Self::latent_heat_j_per_kg() * HE4_MOLAR_MASS; // J/mol
        let ln_ratio = ln_approx(pressure_atm);
        let denom = 1.0 - (GAS_CONSTANT * t0 / l_m) * ln_ratio;
        if denom <= 0.0 {
            return t0 * 2.0; // clamp for extreme pressures
        }
        t0 / denom
    }

    /// Lambda point temperature [K] – He-I to He-II superfluid transition.
    pub fn lambda_point_k() -> f64 {
        2.172
    }

    /// Saturated vapor pressure at temperature T [atm].
    ///
    /// Empirical fit: ln(P/atm) ≈ A - B/T  calibrated so P(4.222K) ≈ 1 atm.
    /// Based on He-4 vapor pressure data near normal boiling range.
    pub fn vapor_pressure_atm(t_k: f64) -> f64 {
        if t_k <= 0.5 {
            return 0.0;
        }
        // Calibrated: at T=4.222 K, P=1 atm → A = B/4.222
        // Using B derived from Clausius-Clapeyron slope near NBP:
        // dln(P)/d(1/T) = -L_m/R ≈ -20700*0.004/8.314 ≈ -9.96
        // B ≈ 9.96, A = 9.96/4.222 ≈ 2.359
        let b = 9.96;
        let a = b / 4.222;
        exp_approx(a - b / t_k)
    }

    /// Specific enthalpy at temperature T [J/kg], integrated from 0 K.
    ///
    /// h(T) = ∫₀ᵀ Cp(T') dT'  (numerical trapezoidal integration).
    pub fn enthalpy_j_per_kg(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        let n = 200;
        let dt = t_k / n as f64;
        let mut h = 0.0;
        for i in 0..n {
            let t0 = i as f64 * dt;
            let t1 = t0 + dt;
            h += 0.5 * (Self::specific_heat_j_per_kg_k(t0) + Self::specific_heat_j_per_kg_k(t1)) * dt;
        }
        h
    }

    /// Density of helium vapor at temperature T and pressure P [kg/m^3].
    /// Ideal gas: ρ = P M / (R T).
    pub fn vapor_density(t_k: f64, pressure_pa: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        pressure_pa * HE4_MOLAR_MASS / (GAS_CONSTANT * t_k)
    }
}

// ---------------------------------------------------------------------------
// ThermalConductivityModels
// ---------------------------------------------------------------------------

/// Temperature-dependent thermal conductivity models for cryogenic materials.
pub struct ThermalConductivityModels;

impl ThermalConductivityModels {
    /// Stainless Steel 304 thermal conductivity [W/(m·K)].
    ///
    /// Approximate fit: k(T) ≈ 0.07 * T for T < 10 K, transitioning
    /// to ~15 W/(m·K) at 300 K.
    pub fn stainless_steel_304(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        if t_k < 10.0 {
            0.07 * t_k
        } else if t_k < 50.0 {
            // Transition region
            0.7 + (t_k - 10.0) * (8.0 - 0.7) / 40.0
        } else {
            // Near-linear approach to 15 W/(m·K) at 300 K
            8.0 + (t_k - 50.0) * (15.0 - 8.0) / 250.0
        }
    }

    /// High-purity copper (RRR ≈ 100) thermal conductivity [W/(m·K)].
    ///
    /// Copper has an enormous conductivity peak near 20-30 K.
    /// k ≈ 20*T for T < 10 K, peaks ~2000 at T ≈ 20 K, then declines to ~400 at 300 K.
    pub fn copper_rrr_100(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        if t_k < 10.0 {
            20.0 * t_k
        } else if t_k < 20.0 {
            200.0 + (t_k - 10.0) * (2000.0 - 200.0) / 10.0
        } else if t_k < 80.0 {
            2000.0 - (t_k - 20.0) * (2000.0 - 600.0) / 60.0
        } else {
            600.0 - (t_k - 80.0) * (600.0 - 400.0) / 220.0
        }
    }

    /// G-10 fiberglass/epoxy composite thermal conductivity [W/(m·K)].
    ///
    /// Very low conductivity at cryogenic temperatures.
    /// k ≈ 0.01 * T for T < 10 K, rising to ~0.5 at 300 K.
    pub fn g10_fiberglass(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        if t_k < 10.0 {
            0.01 * t_k
        } else if t_k < 100.0 {
            0.1 + (t_k - 10.0) * (0.35 - 0.1) / 90.0
        } else {
            0.35 + (t_k - 100.0) * (0.50 - 0.35) / 200.0
        }
    }

    /// Nylon thermal conductivity [W/(m·K)].
    ///
    /// k ≈ 0.005 * T for T < 10 K, rising to ~0.25 at 300 K.
    pub fn nylon(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        if t_k < 10.0 {
            0.005 * t_k
        } else if t_k < 100.0 {
            0.05 + (t_k - 10.0) * (0.17 - 0.05) / 90.0
        } else {
            0.17 + (t_k - 100.0) * (0.25 - 0.17) / 200.0
        }
    }

    /// Aluminum 6061 thermal conductivity [W/(m·K)].
    ///
    /// k ≈ 5*T for T < 10 K, peaks ~600 near 30 K, settles at ~170 at 300 K.
    pub fn aluminum_6061(t_k: f64) -> f64 {
        if t_k <= 0.0 {
            return 0.0;
        }
        if t_k < 10.0 {
            5.0 * t_k
        } else if t_k < 30.0 {
            50.0 + (t_k - 10.0) * (600.0 - 50.0) / 20.0
        } else if t_k < 80.0 {
            600.0 - (t_k - 30.0) * (600.0 - 200.0) / 50.0
        } else {
            200.0 - (t_k - 80.0) * (200.0 - 170.0) / 220.0
        }
    }

    /// Thermal conductivity at temperature T for a given material [W/(m·K)].
    pub fn conductivity(material: SupportMaterial, t_k: f64) -> f64 {
        match material {
            SupportMaterial::StainlessSteel304 => Self::stainless_steel_304(t_k),
            SupportMaterial::CopperRrr100 => Self::copper_rrr_100(t_k),
            SupportMaterial::G10Fiberglass => Self::g10_fiberglass(t_k),
            SupportMaterial::Nylon => Self::nylon(t_k),
            SupportMaterial::Aluminum6061 => Self::aluminum_6061(t_k),
        }
    }

    /// Integrated thermal conductivity ∫_{t_low}^{t_high} k(T) dT  [W/m].
    ///
    /// Numerical trapezoidal integration over temperature.
    pub fn integrated_conductivity(material: SupportMaterial, t_low: f64, t_high: f64) -> f64 {
        if t_high <= t_low {
            return 0.0;
        }
        let n = 500;
        let dt = (t_high - t_low) / n as f64;
        let mut integral = 0.0;
        for i in 0..n {
            let t0 = t_low + i as f64 * dt;
            let t1 = t0 + dt;
            integral +=
                0.5 * (Self::conductivity(material, t0) + Self::conductivity(material, t1)) * dt;
        }
        integral
    }
}

// ---------------------------------------------------------------------------
// MliPerformance
// ---------------------------------------------------------------------------

/// Multi-Layer Insulation (MLI) performance model.
///
/// MLI consists of alternating layers of reflective foil (e.g., aluminized
/// Mylar) and spacer material. Each layer reduces radiation heat transfer
/// by reflecting thermal radiation.
pub struct MliPerformance;

impl MliPerformance {
    /// Effective emissivity of an MLI blanket.
    ///
    /// For N layers with per-layer emissivity ε₀ ≈ 0.03 (aluminized Mylar):
    /// ε_eff ≈ ε₀ / N  (plus conduction through spacers at high packing density).
    ///
    /// At high layer density, solid conduction through spacers dominates and
    /// *increases* with density (more contact points per unit area).
    pub fn effective_emissivity(num_layers: usize, layer_density_per_cm: f64) -> f64 {
        if num_layers == 0 {
            return 1.0;
        }
        let eps0 = 0.03; // emissivity of single aluminized Mylar layer
        let n = num_layers as f64;

        // Radiation component: decreases with more layers
        let eps_rad = eps0 / n;

        // Solid conduction through spacers: increases with packing density.
        // This term is independent of N (denser packing = more contact = more heat).
        // Empirical: eps_cond ≈ 0.002 × density^2.5
        let eps_cond = 0.002 * layer_density_per_cm.powf(2.5);

        eps_rad + eps_cond
    }

    /// Heat flux through MLI [W/m^2].
    ///
    /// q = ε_eff × σ × (T_warm⁴ - T_cold⁴)
    pub fn heat_flux_w_per_m2(
        t_warm: f64,
        t_cold: f64,
        num_layers: usize,
    ) -> f64 {
        let density = 2.0; // typical 20 layers/cm packing → 2.0 layers/cm
        let eps_eff = Self::effective_emissivity(num_layers, density);
        eps_eff * STEFAN_BOLTZMANN * (t_warm.powi(4) - t_cold.powi(4))
    }

    /// Optimal layer density (layers/cm) that minimizes total heat flux.
    ///
    /// At low density: radiation dominates (fewer reflecting layers per cm).
    /// At high density: solid conduction through spacers dominates.
    /// Minimum is typically around 1.5-2.5 layers/cm.
    ///
    /// Uses a combined radiation + conduction model where:
    /// - q_rad ∝ 1/density (fewer layers per cm = more radiation leakage)
    /// - q_cond ∝ density^2 (more contact points = more conduction)
    pub fn optimal_layer_density(t_warm: f64, t_cold: f64) -> f64 {
        let dt4 = t_warm.powi(4) - t_cold.powi(4);
        let dt1 = t_warm - t_cold;

        let mut best_density = 1.5;
        let mut best_flux = f64::MAX;

        // Scan density range 0.5 to 5.0 layers/cm
        for i in 0..200 {
            let density = 0.5 + i as f64 * 0.0225;

            // Radiation flux: decreases with density (more layers per cm)
            let q_rad = (0.03 / density) * STEFAN_BOLTZMANN * dt4;

            // Solid conduction flux: increases with packing density
            // (interstitial gas conduction also increases with contact)
            let q_cond = 2.0e-4 * density * density * dt1;

            let total = q_rad + q_cond;
            if total < best_flux {
                best_flux = total;
                best_density = density;
            }
        }
        best_density
    }

    /// Degradation factor for MLI performance as a function of vacuum quality.
    ///
    /// MLI performs well only in high vacuum (< 10⁻³ Pa).
    /// Factor = 1.0 at perfect vacuum, degrades with increasing pressure.
    pub fn degradation_factor(vacuum_pressure_pa: f64) -> f64 {
        if vacuum_pressure_pa <= 0.0 {
            return 1.0;
        }
        // Empirical: performance degrades roughly as 1 / (1 + P/P_ref)
        // where P_ref ≈ 1e-3 Pa
        let p_ref = 1e-3;
        1.0 / (1.0 + vacuum_pressure_pa / p_ref)
    }
}

// ---------------------------------------------------------------------------
// HeatLoadCalculator
// ---------------------------------------------------------------------------

/// Calculates total heat load on a cryostat from multiple sources.
pub struct HeatLoadCalculator {
    radiation: f64,
    conduction: f64,
    residual_gas: f64,
    current_leads: f64,
    additional: f64,
}

impl HeatLoadCalculator {
    /// Create a new calculator with zero heat loads.
    pub fn new() -> Self {
        Self {
            radiation: 0.0,
            conduction: 0.0,
            residual_gas: 0.0,
            current_leads: 0.0,
            additional: 0.0,
        }
    }

    /// Radiation heat load through MLI [W].
    ///
    /// Q_rad = ε × σ × A × (T_warm⁴ - T_cold⁴) / N_shields
    ///
    /// With N thermal shields, each approximately halving the radiation.
    pub fn radiation_heat_load(
        area_m2: f64,
        emissivity: f64,
        t_warm: f64,
        t_cold: f64,
        num_shields: usize,
    ) -> f64 {
        let q = emissivity * STEFAN_BOLTZMANN * area_m2 * (t_warm.powi(4) - t_cold.powi(4));
        if num_shields == 0 {
            q
        } else {
            // Each radiation shield reduces heat transfer by ~1/(N+1)
            q / (num_shields as f64 + 1.0)
        }
    }

    /// Conduction heat load through a structural support [W].
    ///
    /// Q_cond = (A / L) × ∫_{T_cold}^{T_warm} k(T) dT
    pub fn conduction_heat_load(
        material: SupportMaterial,
        length_m: f64,
        area_m2: f64,
        t_warm: f64,
        t_cold: f64,
    ) -> f64 {
        if length_m <= 0.0 {
            return 0.0;
        }
        let k_int = ThermalConductivityModels::integrated_conductivity(material, t_cold, t_warm);
        area_m2 / length_m * k_int
    }

    /// Residual gas heat load in free molecular regime [W].
    ///
    /// Q_gas = α × P × A × √(T / (2π M R)) × (T_warm - T_cold)
    /// where α is accommodation coefficient (~0.5 for He on metal).
    pub fn residual_gas_heat_load(
        pressure_pa: f64,
        area_m2: f64,
        t_warm: f64,
        t_cold: f64,
    ) -> f64 {
        let alpha = 0.5; // accommodation coefficient for He
        let t_mean = 0.5 * (t_warm + t_cold);
        let factor = sqrt_approx(t_mean / (2.0 * std::f64::consts::PI * HE4_MOLAR_MASS * GAS_CONSTANT));
        alpha * pressure_pa * area_m2 * factor * (t_warm - t_cold)
    }

    /// Heat load from vapor-cooled current leads [W].
    ///
    /// Optimized leads: Q ≈ 47 mW per amp per lead pair (4.2 K to 300 K).
    pub fn current_lead_heat_load(current_a: f64, num_leads: usize) -> f64 {
        // 47 mW/A per lead pair
        0.047 * current_a * (num_leads as f64 / 2.0).max(1.0)
    }

    /// Set radiation heat load component.
    pub fn set_radiation(&mut self, q_w: f64) {
        self.radiation = q_w;
    }

    /// Set conduction heat load component.
    pub fn set_conduction(&mut self, q_w: f64) {
        self.conduction = q_w;
    }

    /// Set residual gas heat load component.
    pub fn set_residual_gas(&mut self, q_w: f64) {
        self.residual_gas = q_w;
    }

    /// Set current lead heat load component.
    pub fn set_current_leads(&mut self, q_w: f64) {
        self.current_leads = q_w;
    }

    /// Set additional heat load (instrumentation wiring, etc.).
    pub fn set_additional(&mut self, q_w: f64) {
        self.additional = q_w;
    }

    /// Total heat load [W] = sum of all contributions.
    pub fn total_heat_load(&self) -> f64 {
        self.radiation + self.conduction + self.residual_gas + self.current_leads + self.additional
    }
}

// ---------------------------------------------------------------------------
// BoiloffPredictor
// ---------------------------------------------------------------------------

/// Predicts liquid helium boiloff rate based on total heat load.
///
/// The fundamental relation: boiloff rate = Q_total / (ρ × L)
/// where ρ is LHe density and L is latent heat.
pub struct BoiloffPredictor {
    total_heat_load_w: f64,
    recovery_efficiency: f64,
}

impl BoiloffPredictor {
    /// Create a predictor for a given total heat load [W].
    pub fn new(total_heat_load_w: f64) -> Self {
        Self {
            total_heat_load_w: total_heat_load_w.max(0.0),
            recovery_efficiency: 0.0,
        }
    }

    /// Boiloff rate in liters per hour.
    ///
    /// dm/dt = Q / L  [kg/s]
    /// dV/dt = dm/dt / ρ  [m³/s]
    /// Convert to liters/hr: × 1000 × 3600
    pub fn boiloff_rate_liters_per_hour(&self) -> f64 {
        let effective_load = self.total_heat_load_w * (1.0 - self.recovery_efficiency);
        let dm_dt = effective_load / HeliumProperties::latent_heat_j_per_kg(); // kg/s
        let dv_dt = dm_dt / HeliumProperties::density_kg_per_m3(); // m³/s
        dv_dt * 1000.0 * 3600.0 // liters/hr
    }

    /// Boiloff rate in kg per hour.
    pub fn boiloff_rate_kg_per_hour(&self) -> f64 {
        let effective_load = self.total_heat_load_w * (1.0 - self.recovery_efficiency);
        let dm_dt = effective_load / HeliumProperties::latent_heat_j_per_kg();
        dm_dt * 3600.0
    }

    /// Hold time in hours for a given volume of LHe [liters].
    pub fn hold_time_hours(&self, volume_liters: f64) -> f64 {
        let rate = self.boiloff_rate_liters_per_hour();
        if rate <= 0.0 {
            return f64::INFINITY;
        }
        volume_liters / rate
    }

    /// Annual helium consumption [liters/year].
    pub fn annual_consumption_liters(&self) -> f64 {
        self.boiloff_rate_liters_per_hour() * 24.0 * 365.25
    }

    /// Return a new predictor with helium recovery efficiency (0.0 - 1.0).
    pub fn with_recovery_efficiency(&self, efficiency: f64) -> BoiloffPredictor {
        BoiloffPredictor {
            total_heat_load_w: self.total_heat_load_w,
            recovery_efficiency: efficiency.clamp(0.0, 1.0),
        }
    }

    /// Return the effective heat load after recovery [W].
    pub fn effective_heat_load_w(&self) -> f64 {
        self.total_heat_load_w * (1.0 - self.recovery_efficiency)
    }
}

// ---------------------------------------------------------------------------
// DewarModel
// ---------------------------------------------------------------------------

/// Cylindrical cryostat (Dewar) thermal model.
pub struct DewarModel {
    inner_radius_m: f64,
    outer_radius_m: f64,
    height_m: f64,
    mli_layers: usize,
}

impl DewarModel {
    /// Create a new Dewar model.
    ///
    /// - `inner_radius_m`: Inner vessel radius [m]
    /// - `outer_radius_m`: Outer vessel radius [m]
    /// - `height_m`: Vessel height [m]
    /// - `mli_layers`: Number of MLI layers in vacuum space
    pub fn new(
        inner_radius_m: f64,
        outer_radius_m: f64,
        height_m: f64,
        mli_layers: usize,
    ) -> Self {
        Self {
            inner_radius_m,
            outer_radius_m,
            height_m,
            mli_layers,
        }
    }

    /// Heat loss through the neck tube [W].
    ///
    /// Neck tube conducts heat from warm flange to cold vessel.
    /// Q = (π × d × t / L) × ∫k(T)dT  for thin-wall tube of thickness t.
    pub fn neck_tube_loss(
        &self,
        diameter_m: f64,
        length_m: f64,
        material: NeckMaterial,
    ) -> f64 {
        if length_m <= 0.0 || diameter_m <= 0.0 {
            return 0.0;
        }
        // Assume thin-wall tube with wall thickness = 1 mm
        let wall_thickness = 0.001;
        let area = std::f64::consts::PI * diameter_m * wall_thickness;

        let mat = match material {
            NeckMaterial::StainlessSteel304 => SupportMaterial::StainlessSteel304,
            NeckMaterial::G10Fiberglass => SupportMaterial::G10Fiberglass,
            NeckMaterial::ThinWallStainless => SupportMaterial::StainlessSteel304,
        };

        let k_int = ThermalConductivityModels::integrated_conductivity(mat, 4.2, 300.0);

        let factor = match material {
            NeckMaterial::ThinWallStainless => 0.5, // thinner wall
            _ => 1.0,
        };

        factor * area / length_m * k_int
    }

    /// Fill level from differential pressure measurement.
    ///
    /// Level = ΔP / (ρ × g × H), where g = 9.81 m/s².
    pub fn fill_level_from_pressure(&self, pressure_pa: f64, height_m: f64) -> f64 {
        if height_m <= 0.0 {
            return 0.0;
        }
        let rho = HeliumProperties::density_kg_per_m3();
        let g = 9.81;
        let level = pressure_pa / (rho * g * height_m);
        level.clamp(0.0, 1.0)
    }

    /// Static heat load calculated from geometry and MLI [W].
    ///
    /// Combines radiation through MLI on cylindrical surfaces plus end caps.
    pub fn static_heat_load(&self) -> f64 {
        let area = self.surface_area();
        MliPerformance::heat_flux_w_per_m2(300.0, 4.2, self.mli_layers) * area
    }

    /// Total surface area of inner vessel [m²] (cylinder + two end caps).
    pub fn surface_area(&self) -> f64 {
        let pi = std::f64::consts::PI;
        let r = self.inner_radius_m;
        let h = self.height_m;
        2.0 * pi * r * h + 2.0 * pi * r * r
    }

    /// Inner volume of the Dewar [liters].
    pub fn inner_volume_liters(&self) -> f64 {
        let pi = std::f64::consts::PI;
        let v_m3 = pi * self.inner_radius_m * self.inner_radius_m * self.height_m;
        v_m3 * 1000.0
    }

    /// Vacuum space gap [m].
    pub fn vacuum_gap(&self) -> f64 {
        self.outer_radius_m - self.inner_radius_m
    }
}

// ---------------------------------------------------------------------------
// LevelSensor
// ---------------------------------------------------------------------------

/// Helium level measurement and prediction methods.
pub struct LevelSensor;

impl LevelSensor {
    /// Fill percentage from differential pressure measurement.
    ///
    /// level = ΔP / (ρ_LHe × g × H_total)
    pub fn from_differential_pressure(dp_pa: f64, height_m: f64) -> f64 {
        if height_m <= 0.0 {
            return 0.0;
        }
        let rho = HeliumProperties::density_kg_per_m3();
        let g = 9.81;
        let level = dp_pa / (rho * g * height_m);
        level.clamp(0.0, 1.0)
    }

    /// Fill percentage from capacitance measurement.
    ///
    /// Liquid He has dielectric constant ε_r ≈ 1.055 vs gas ≈ 1.0.
    /// level = (C_measured - C_empty) / (C_full - C_empty)
    pub fn from_capacitance(c_measured: f64, c_empty: f64, c_full: f64) -> f64 {
        if (c_full - c_empty).abs() < 1e-15 {
            return 0.0;
        }
        let level = (c_measured - c_empty) / (c_full - c_empty);
        level.clamp(0.0, 1.0)
    }

    /// Fill percentage from superconducting wire sensor.
    ///
    /// Superconducting NbTi wire is SC below Tc ≈ 9.2 K (in LHe).
    /// Resistance ratio = R_measured / R_total_normal.
    /// The fraction of wire above the liquid level is resistive.
    /// level = 1 - resistance_ratio
    pub fn from_superconducting_wire(resistance_ratio: f64) -> f64 {
        let level = 1.0 - resistance_ratio;
        level.clamp(0.0, 1.0)
    }

    /// Predict time to refill [hours].
    ///
    /// Time until level drops from current level to minimum level,
    /// given a constant boiloff rate.
    pub fn predict_time_to_refill(
        level_pct: f64,
        boiloff_rate_pct_per_hour: f64,
        min_level_pct: f64,
    ) -> f64 {
        if boiloff_rate_pct_per_hour <= 0.0 {
            return f64::INFINITY;
        }
        let remaining = level_pct - min_level_pct;
        if remaining <= 0.0 {
            return 0.0;
        }
        remaining / boiloff_rate_pct_per_hour
    }
}

// ---------------------------------------------------------------------------
// QuenchAnalyzer
// ---------------------------------------------------------------------------

/// Analyzes helium loss during a superconducting magnet quench event.
///
/// During a quench, stored magnetic energy E = ½LI² is rapidly dissipated
/// as heat, causing violent helium boiloff.
pub struct QuenchAnalyzer;

impl QuenchAnalyzer {
    /// Stored energy in the magnet [J].
    ///
    /// E = ½ L I²
    pub fn quench_energy(inductance_h: f64, current_a: f64) -> f64 {
        0.5 * inductance_h * current_a * current_a
    }

    /// Helium flash-off during quench [liters].
    ///
    /// Volume evaporated = E / (ρ × L)
    pub fn helium_flash_off(energy_j: f64) -> f64 {
        let rho = HeliumProperties::density_kg_per_m3();
        let l = HeliumProperties::latent_heat_j_per_kg();
        let mass_kg = energy_j / l;
        let volume_m3 = mass_kg / rho;
        volume_m3 * 1000.0 // liters
    }

    /// Pressure rise in the Dewar during quench [Pa].
    ///
    /// Assumes gas expands into head space. P = n R T / V_gas
    /// where n = moles evaporated, V_gas = vessel volume.
    pub fn pressure_rise(volume_m3: f64, energy_j: f64) -> f64 {
        if volume_m3 <= 0.0 {
            return 0.0;
        }
        let mass_kg = energy_j / HeliumProperties::latent_heat_j_per_kg();
        let n_moles = mass_kg / HE4_MOLAR_MASS;
        let t_gas = 10.0; // Assume gas warms to ~10 K initially
        n_moles * GAS_CONSTANT * t_gas / volume_m3
    }

    /// Time to recover (refill) after quench [hours].
    ///
    /// Assumes total loss of helium, refill at given rate.
    pub fn recovery_time(volume_liters: f64, refill_rate_liters_per_hour: f64) -> f64 {
        if refill_rate_liters_per_hour <= 0.0 {
            return f64::INFINITY;
        }
        volume_liters / refill_rate_liters_per_hour
    }
}

// ---------------------------------------------------------------------------
// CoolingPowerBudget
// ---------------------------------------------------------------------------

/// Cryocooler capacity analysis for zero-boiloff or reduced-boiloff systems.
pub struct CoolingPowerBudget;

impl CoolingPowerBudget {
    /// Gifford-McMahon (GM) cooler capacity [W] at a given temperature.
    ///
    /// Typical two-stage GM cooler:
    /// - Stage 1 (~40-80 K): 30-50 W
    /// - Stage 2 (~4-10 K): 0.5-2.0 W
    pub fn gifford_mcmahon_capacity(stage: usize, temperature_k: f64) -> f64 {
        match stage {
            1 => {
                // First stage: linear model from 30W at 80K to 50W at 40K
                if temperature_k < 20.0 {
                    5.0
                } else if temperature_k < 80.0 {
                    30.0 + (80.0 - temperature_k) * 0.5
                } else {
                    30.0 - (temperature_k - 80.0) * 0.1
                }
            }
            2 => {
                // Second stage: ~1.5W at 4.2K, dropping off at lower T
                if temperature_k < 2.0 {
                    0.1
                } else if temperature_k < 4.2 {
                    0.1 + (temperature_k - 2.0) * (1.5 - 0.1) / 2.2
                } else if temperature_k < 10.0 {
                    1.5 + (temperature_k - 4.2) * 0.3
                } else {
                    3.0
                }
            }
            _ => 0.0,
        }
    }

    /// Pulse tube cooler (PTC) capacity [W] at a given temperature.
    ///
    /// PTCs have no moving parts at the cold end, offering lower vibration.
    /// Capacity at 4.2 K: typically 0.5-1.5 W for a large unit.
    pub fn pulse_tube_capacity(temperature_k: f64) -> f64 {
        if temperature_k < 2.0 {
            0.05
        } else if temperature_k < 4.2 {
            0.05 + (temperature_k - 2.0) * (1.0 - 0.05) / 2.2
        } else if temperature_k < 20.0 {
            1.0 + (temperature_k - 4.2) * 0.5
        } else if temperature_k < 80.0 {
            8.9 + (temperature_k - 20.0) * 0.5
        } else {
            40.0
        }
    }

    /// Cooling margin (safety factor).
    ///
    /// margin = capacity / heat_load
    /// Values > 1.0 mean cooler can handle the load.
    /// Typical design target: > 1.3 (30% margin).
    pub fn cooling_margin(capacity_w: f64, heat_load_w: f64) -> f64 {
        if heat_load_w <= 0.0 {
            return f64::INFINITY;
        }
        capacity_w / heat_load_w
    }

    /// Whether zero-boiloff operation is feasible.
    ///
    /// Requires cooler capacity > heat load with adequate margin.
    pub fn zero_boiloff_feasible(heat_load_w: f64, cooler_capacity_w: f64) -> bool {
        cooler_capacity_w > heat_load_w * 1.1 // 10% minimum margin
    }
}

// ---------------------------------------------------------------------------
// HeliumRecoverySystem
// ---------------------------------------------------------------------------

/// Cost analysis result for helium consumption.
#[derive(Debug, Clone)]
pub struct CostResult {
    /// Annual helium cost without recovery [$/year].
    pub annual_cost_no_recovery: f64,
    /// Annual helium cost with recovery [$/year].
    pub annual_cost_with_recovery: f64,
    /// Annual savings from recovery [$/year].
    pub annual_savings: f64,
    /// Consumption rate without recovery [liters/year].
    pub consumption_no_recovery: f64,
    /// Consumption rate with recovery [liters/year].
    pub consumption_with_recovery: f64,
}

/// Helium recovery and reliquefaction system modeling.
pub struct HeliumRecoverySystem;

impl HeliumRecoverySystem {
    /// Overall recovery efficiency.
    ///
    /// Accounts for capture rate (bag/pipeline collection) and
    /// purification losses.
    pub fn recovery_efficiency(capture_rate: f64, purification_loss: f64) -> f64 {
        let eff = capture_rate * (1.0 - purification_loss);
        eff.clamp(0.0, 1.0)
    }

    /// Electrical power required for reliquefaction [W].
    ///
    /// Uses Carnot COP with a practical efficiency factor:
    /// COP_real = η_carnot × COP_carnot
    /// COP_carnot = T_cold / (T_warm - T_cold)
    /// Power = rate × ρ × L / COP_real
    pub fn reliquefaction_power_w(rate_liters_per_hour: f64) -> f64 {
        let rho = HeliumProperties::density_kg_per_m3(); // kg/m³
        let l = HeliumProperties::latent_heat_j_per_kg(); // J/kg

        // Mass rate: liters/hr → m³/s → kg/s
        let mass_rate = rate_liters_per_hour / 1000.0 / 3600.0 * rho;
        let cooling_power = mass_rate * l; // W of cooling needed

        // Carnot COP at 4.2K / (300 - 4.2) = 0.0142
        let cop_carnot = 4.2 / (300.0 - 4.2);
        // Real efficiency ~5% of Carnot for helium liquefiers
        let cop_real = cop_carnot * 0.05;

        if cop_real <= 0.0 {
            return 0.0;
        }
        cooling_power / cop_real
    }

    /// Cost analysis for helium consumption.
    pub fn cost_analysis(
        consumption_rate_liters_per_year: f64,
        price_per_liter: f64,
        recovery_efficiency: f64,
    ) -> CostResult {
        let eff = recovery_efficiency.clamp(0.0, 1.0);
        let cost_no_recovery = consumption_rate_liters_per_year * price_per_liter;
        let consumption_with = consumption_rate_liters_per_year * (1.0 - eff);
        let cost_with = consumption_with * price_per_liter;

        CostResult {
            annual_cost_no_recovery: cost_no_recovery,
            annual_cost_with_recovery: cost_with,
            annual_savings: cost_no_recovery - cost_with,
            consumption_no_recovery: consumption_rate_liters_per_year,
            consumption_with_recovery: consumption_with,
        }
    }

    /// Global helium supply context information.
    pub fn global_supply_context() -> &'static str {
        "Helium is a non-renewable resource extracted from natural gas. Global supply \
         is concentrated in a few sources (US BLM reserve depleted 2021, Qatar, Algeria, \
         Russia). Prices have risen 3-5x since 2019. Recovery and recycling are critical \
         for sustainable cryogenic operations. Typical MRI uses 1500-2000 L/year without \
         recovery. Modern zero-boiloff MRI systems with cryocoolers significantly reduce \
         consumption."
    }
}

// ---------------------------------------------------------------------------
// Math helpers (no std math beyond basic ops)
// ---------------------------------------------------------------------------

/// Natural logarithm approximation using the identity:
/// ln(x) = 2 * sum_{k=0}^{N} (1/(2k+1)) * ((x-1)/(x+1))^(2k+1)
fn ln_approx(x: f64) -> f64 {
    if x <= 0.0 {
        return f64::NEG_INFINITY;
    }
    if x == 1.0 {
        return 0.0;
    }

    // Range reduction: ln(x) = ln(m * 2^e) = ln(m) + e*ln(2)
    // where 1 <= m < 2
    let mut mantissa = x;
    let mut exponent: i32 = 0;

    while mantissa >= 2.0 {
        mantissa /= 2.0;
        exponent += 1;
    }
    while mantissa < 1.0 {
        mantissa *= 2.0;
        exponent -= 1;
    }

    let ln2 = 0.693147180559945309;

    // Series for ln(m) where 1 <= m < 2
    let y = (mantissa - 1.0) / (mantissa + 1.0);
    let y2 = y * y;
    let mut result = 0.0;
    let mut power = y;
    for k in 0..30 {
        result += power / (2 * k + 1) as f64;
        power *= y2;
    }
    result *= 2.0;

    result + exponent as f64 * ln2
}

/// Exponential function approximation: exp(x) = e^x
fn exp_approx(x: f64) -> f64 {
    if x > 700.0 {
        return f64::INFINITY;
    }
    if x < -700.0 {
        return 0.0;
    }

    // Range reduction: e^x = 2^(x/ln2) = 2^n * e^r
    // where n = floor(x/ln2) and r = x - n*ln2
    let ln2 = 0.693147180559945309;
    let n = (x / ln2).floor() as i64;
    let r = x - n as f64 * ln2;

    // Taylor series for e^r (|r| < ln2 ≈ 0.693)
    let mut term = 1.0;
    let mut sum = 1.0;
    for k in 1..30 {
        term *= r / k as f64;
        sum += term;
        if term.abs() < 1e-15 {
            break;
        }
    }

    // Multiply by 2^n
    let pow2 = pow2_int(n);
    sum * pow2
}

/// 2^n for integer n
fn pow2_int(n: i64) -> f64 {
    if n >= 0 {
        let mut result = 1.0_f64;
        for _ in 0..n.min(1023) {
            result *= 2.0;
        }
        result
    } else {
        let mut result = 1.0_f64;
        for _ in 0..(-n).min(1074) {
            result /= 2.0;
        }
        result
    }
}

/// Square root approximation using Newton's method.
fn sqrt_approx(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    // Initial guess using bit manipulation concept
    let mut guess = x;
    // Better initial guess
    if x > 1.0 {
        guess = x / 2.0;
    } else {
        guess = (x + 1.0) / 2.0;
    }

    // Newton iterations: x_{n+1} = 0.5 * (x_n + S/x_n)
    for _ in 0..60 {
        let next = 0.5 * (guess + x / guess);
        if (next - guess).abs() < 1e-15 * guess.abs() {
            break;
        }
        guess = next;
    }
    guess
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const REL_EPSILON: f64 = 0.05; // 5% relative tolerance for physics models

    fn assert_approx(a: f64, b: f64, eps: f64) {
        assert!(
            (a - b).abs() < eps,
            "Expected {} ≈ {} (within {}), diff = {}",
            a,
            b,
            eps,
            (a - b).abs()
        );
    }

    fn assert_relative(actual: f64, expected: f64, tol: f64) {
        if expected.abs() < 1e-12 {
            assert!(actual.abs() < tol, "Expected ~0, got {}", actual);
            return;
        }
        let rel = (actual - expected).abs() / expected.abs();
        assert!(
            rel < tol,
            "Expected {} ≈ {} (within {}%), rel diff = {}%",
            actual,
            expected,
            tol * 100.0,
            rel * 100.0
        );
    }

    // -----------------------------------------------------------------------
    // Math helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ln_approx_basic() {
        assert_approx(ln_approx(1.0), 0.0, 1e-12);
        assert_approx(ln_approx(std::f64::consts::E), 1.0, 1e-8);
        assert_approx(ln_approx(2.0), 0.6931471805, 1e-8);
    }

    #[test]
    fn test_ln_approx_range() {
        assert_approx(ln_approx(0.5), -0.6931471805, 1e-8);
        assert_approx(ln_approx(10.0), 2.302585093, 1e-8);
        assert_approx(ln_approx(100.0), 4.605170186, 1e-7);
    }

    #[test]
    fn test_exp_approx_basic() {
        assert_approx(exp_approx(0.0), 1.0, 1e-12);
        assert_approx(exp_approx(1.0), std::f64::consts::E, 1e-8);
        assert_approx(exp_approx(-1.0), 1.0 / std::f64::consts::E, 1e-8);
    }

    #[test]
    fn test_exp_approx_large() {
        assert_approx(exp_approx(10.0), 22026.4657948, 0.01);
        assert!(exp_approx(-100.0) > 0.0);
        assert!(exp_approx(-100.0) < 1e-40);
    }

    #[test]
    fn test_sqrt_approx() {
        assert_approx(sqrt_approx(4.0), 2.0, 1e-12);
        assert_approx(sqrt_approx(9.0), 3.0, 1e-12);
        assert_approx(sqrt_approx(2.0), 1.41421356, 1e-8);
        assert_approx(sqrt_approx(0.25), 0.5, 1e-12);
    }

    #[test]
    fn test_sqrt_zero_negative() {
        assert_approx(sqrt_approx(0.0), 0.0, 1e-15);
        assert_approx(sqrt_approx(-1.0), 0.0, 1e-15);
    }

    // -----------------------------------------------------------------------
    // HeliumProperties tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_latent_heat() {
        assert_approx(HeliumProperties::latent_heat_j_per_kg(), 20700.0, EPSILON);
    }

    #[test]
    fn test_density() {
        assert_approx(HeliumProperties::density_kg_per_m3(), 125.0, EPSILON);
    }

    #[test]
    fn test_specific_heat_positive_temperature() {
        let cp = HeliumProperties::specific_heat_j_per_kg_k(4.2);
        assert!(cp > 0.0);
        assert!(cp > 4000.0); // Should be around 5000-6000 J/(kg·K)
    }

    #[test]
    fn test_specific_heat_zero_temperature() {
        assert_approx(HeliumProperties::specific_heat_j_per_kg_k(0.0), 0.0, EPSILON);
        assert_approx(HeliumProperties::specific_heat_j_per_kg_k(-1.0), 0.0, EPSILON);
    }

    #[test]
    fn test_specific_heat_superfluid() {
        // Below lambda point, Cp should be lower
        let cp_2k = HeliumProperties::specific_heat_j_per_kg_k(2.0);
        let cp_3k = HeliumProperties::specific_heat_j_per_kg_k(3.0);
        assert!(cp_2k < cp_3k, "Cp at 2K ({}) should be < Cp at 3K ({})", cp_2k, cp_3k);
    }

    #[test]
    fn test_specific_heat_gas_phase() {
        let cp_gas = HeliumProperties::specific_heat_j_per_kg_k(10.0);
        assert_relative(cp_gas, 5193.0, 0.01); // monatomic ideal gas
    }

    #[test]
    fn test_boiling_point_1atm() {
        let bp = HeliumProperties::boiling_point_k(1.0);
        assert_relative(bp, 4.222, REL_EPSILON);
    }

    #[test]
    fn test_boiling_point_higher_pressure() {
        let bp_high = HeliumProperties::boiling_point_k(2.0);
        let bp_1atm = HeliumProperties::boiling_point_k(1.0);
        assert!(bp_high > bp_1atm, "Higher pressure should raise boiling point");
    }

    #[test]
    fn test_boiling_point_lower_pressure() {
        let bp_low = HeliumProperties::boiling_point_k(0.5);
        let bp_1atm = HeliumProperties::boiling_point_k(1.0);
        assert!(bp_low < bp_1atm, "Lower pressure should lower boiling point");
    }

    #[test]
    fn test_boiling_point_zero_pressure() {
        assert_approx(HeliumProperties::boiling_point_k(0.0), 0.0, EPSILON);
    }

    #[test]
    fn test_lambda_point() {
        assert_approx(HeliumProperties::lambda_point_k(), 2.172, EPSILON);
    }

    #[test]
    fn test_vapor_pressure_at_boiling() {
        let p = HeliumProperties::vapor_pressure_atm(4.222);
        // Should be approximately 1 atm at boiling point
        assert_relative(p, 1.0, 0.3); // empirical fit, 30% tolerance
    }

    #[test]
    fn test_vapor_pressure_monotonic() {
        let p1 = HeliumProperties::vapor_pressure_atm(3.0);
        let p2 = HeliumProperties::vapor_pressure_atm(4.0);
        let p3 = HeliumProperties::vapor_pressure_atm(5.0);
        assert!(p1 < p2, "Vapor pressure should increase with T");
        assert!(p2 < p3, "Vapor pressure should increase with T");
    }

    #[test]
    fn test_vapor_pressure_low_temp() {
        let p = HeliumProperties::vapor_pressure_atm(0.5);
        assert_approx(p, 0.0, EPSILON);
    }

    #[test]
    fn test_enthalpy_positive() {
        let h = HeliumProperties::enthalpy_j_per_kg(4.2);
        assert!(h > 0.0, "Enthalpy at 4.2K should be positive");
    }

    #[test]
    fn test_enthalpy_monotonic() {
        let h1 = HeliumProperties::enthalpy_j_per_kg(2.0);
        let h2 = HeliumProperties::enthalpy_j_per_kg(3.0);
        let h3 = HeliumProperties::enthalpy_j_per_kg(4.0);
        assert!(h1 < h2, "Enthalpy should increase with temperature");
        assert!(h2 < h3, "Enthalpy should increase with temperature");
    }

    #[test]
    fn test_enthalpy_zero() {
        assert_approx(HeliumProperties::enthalpy_j_per_kg(0.0), 0.0, EPSILON);
    }

    #[test]
    fn test_vapor_density() {
        let rho = HeliumProperties::vapor_density(4.2, ATM_PA);
        // ρ = P M / (R T) = 101325 * 0.004 / (8.314 * 4.2) ≈ 11.6 kg/m³
        assert!(rho > 5.0 && rho < 20.0, "Vapor density {} seems wrong", rho);
    }

    // -----------------------------------------------------------------------
    // ThermalConductivity tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ss304_positive() {
        assert!(ThermalConductivityModels::stainless_steel_304(4.2) > 0.0);
        assert!(ThermalConductivityModels::stainless_steel_304(300.0) > 0.0);
    }

    #[test]
    fn test_ss304_increases_with_temperature() {
        let k4 = ThermalConductivityModels::stainless_steel_304(4.2);
        let k300 = ThermalConductivityModels::stainless_steel_304(300.0);
        assert!(k300 > k4, "SS304 k should increase with T: k(4.2)={}, k(300)={}", k4, k300);
    }

    #[test]
    fn test_copper_peak() {
        let k10 = ThermalConductivityModels::copper_rrr_100(10.0);
        let k20 = ThermalConductivityModels::copper_rrr_100(20.0);
        let k300 = ThermalConductivityModels::copper_rrr_100(300.0);
        assert!(k20 > k10, "Copper k should peak near 20K");
        assert!(k20 > k300, "Copper k at peak should exceed room temp");
    }

    #[test]
    fn test_g10_low_conductivity() {
        let k_g10 = ThermalConductivityModels::g10_fiberglass(4.2);
        let k_ss = ThermalConductivityModels::stainless_steel_304(4.2);
        assert!(k_g10 < k_ss, "G10 should have lower k than SS304 at 4.2K");
    }

    #[test]
    fn test_nylon_low_conductivity() {
        let k_nylon = ThermalConductivityModels::nylon(4.2);
        let k_g10 = ThermalConductivityModels::g10_fiberglass(4.2);
        assert!(k_nylon < k_g10, "Nylon should have lower k than G10");
    }

    #[test]
    fn test_aluminum_high_conductivity() {
        let k_al = ThermalConductivityModels::aluminum_6061(300.0);
        let k_ss = ThermalConductivityModels::stainless_steel_304(300.0);
        assert!(k_al > k_ss, "Aluminum should have higher k than SS304");
    }

    #[test]
    fn test_conductivity_zero_temp() {
        assert_approx(ThermalConductivityModels::stainless_steel_304(0.0), 0.0, EPSILON);
        assert_approx(ThermalConductivityModels::copper_rrr_100(0.0), 0.0, EPSILON);
        assert_approx(ThermalConductivityModels::g10_fiberglass(0.0), 0.0, EPSILON);
    }

    #[test]
    fn test_integrated_conductivity_positive() {
        let ki = ThermalConductivityModels::integrated_conductivity(
            SupportMaterial::StainlessSteel304,
            4.2,
            300.0,
        );
        assert!(ki > 0.0, "Integrated k should be positive");
    }

    #[test]
    fn test_integrated_conductivity_g10_vs_ss() {
        let ki_g10 = ThermalConductivityModels::integrated_conductivity(
            SupportMaterial::G10Fiberglass,
            4.2,
            300.0,
        );
        let ki_ss = ThermalConductivityModels::integrated_conductivity(
            SupportMaterial::StainlessSteel304,
            4.2,
            300.0,
        );
        assert!(ki_g10 < ki_ss, "G10 should have lower integrated k than SS304");
    }

    #[test]
    fn test_integrated_conductivity_reversed_limits() {
        let ki = ThermalConductivityModels::integrated_conductivity(
            SupportMaterial::StainlessSteel304,
            300.0,
            4.2,
        );
        assert_approx(ki, 0.0, EPSILON);
    }

    // -----------------------------------------------------------------------
    // MliPerformance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mli_effective_emissivity_decreases_with_layers() {
        let eps10 = MliPerformance::effective_emissivity(10, 2.0);
        let eps30 = MliPerformance::effective_emissivity(30, 2.0);
        let eps60 = MliPerformance::effective_emissivity(60, 2.0);
        assert!(eps30 < eps10);
        assert!(eps60 < eps30);
    }

    #[test]
    fn test_mli_no_layers() {
        assert_approx(MliPerformance::effective_emissivity(0, 2.0), 1.0, EPSILON);
    }

    #[test]
    fn test_mli_heat_flux_positive() {
        let q = MliPerformance::heat_flux_w_per_m2(300.0, 4.2, 30);
        assert!(q > 0.0, "Heat flux through MLI should be positive");
    }

    #[test]
    fn test_mli_heat_flux_decreases_with_layers() {
        let q10 = MliPerformance::heat_flux_w_per_m2(300.0, 4.2, 10);
        let q30 = MliPerformance::heat_flux_w_per_m2(300.0, 4.2, 30);
        assert!(q30 < q10, "More MLI layers should reduce heat flux");
    }

    #[test]
    fn test_mli_optimal_density() {
        let density = MliPerformance::optimal_layer_density(300.0, 4.2);
        assert!(density > 0.5 && density < 5.0, "Optimal density {} out of range", density);
    }

    #[test]
    fn test_mli_degradation_perfect_vacuum() {
        assert_approx(MliPerformance::degradation_factor(0.0), 1.0, EPSILON);
    }

    #[test]
    fn test_mli_degradation_poor_vacuum() {
        let factor = MliPerformance::degradation_factor(1.0); // 1 Pa
        assert!(factor < 0.01, "MLI should be severely degraded at 1 Pa: {}", factor);
    }

    // -----------------------------------------------------------------------
    // HeatLoadCalculator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_radiation_heat_load_positive() {
        let q = HeatLoadCalculator::radiation_heat_load(1.0, 0.03, 300.0, 4.2, 0);
        assert!(q > 0.0);
    }

    #[test]
    fn test_radiation_heat_load_with_shields() {
        let q_no_shield = HeatLoadCalculator::radiation_heat_load(1.0, 0.03, 300.0, 4.2, 0);
        let q_1_shield = HeatLoadCalculator::radiation_heat_load(1.0, 0.03, 300.0, 4.2, 1);
        let q_3_shields = HeatLoadCalculator::radiation_heat_load(1.0, 0.03, 300.0, 4.2, 3);
        assert!(q_1_shield < q_no_shield);
        assert!(q_3_shields < q_1_shield);
    }

    #[test]
    fn test_conduction_heat_load_g10() {
        let q = HeatLoadCalculator::conduction_heat_load(
            SupportMaterial::G10Fiberglass,
            0.5,
            1e-4,
            300.0,
            4.2,
        );
        assert!(q > 0.0, "Conduction heat load should be positive");
    }

    #[test]
    fn test_conduction_heat_load_material_comparison() {
        let q_g10 = HeatLoadCalculator::conduction_heat_load(
            SupportMaterial::G10Fiberglass,
            0.5,
            1e-4,
            300.0,
            4.2,
        );
        let q_ss = HeatLoadCalculator::conduction_heat_load(
            SupportMaterial::StainlessSteel304,
            0.5,
            1e-4,
            300.0,
            4.2,
        );
        assert!(q_g10 < q_ss, "G10 should conduct less heat than SS304");
    }

    #[test]
    fn test_conduction_zero_length() {
        let q = HeatLoadCalculator::conduction_heat_load(
            SupportMaterial::StainlessSteel304,
            0.0,
            1e-4,
            300.0,
            4.2,
        );
        assert_approx(q, 0.0, EPSILON);
    }

    #[test]
    fn test_residual_gas_heat_load() {
        let q = HeatLoadCalculator::residual_gas_heat_load(1e-4, 1.0, 300.0, 4.2);
        assert!(q > 0.0, "Residual gas heat should be positive");
    }

    #[test]
    fn test_residual_gas_proportional_to_pressure() {
        let q1 = HeatLoadCalculator::residual_gas_heat_load(1e-4, 1.0, 300.0, 4.2);
        let q2 = HeatLoadCalculator::residual_gas_heat_load(1e-3, 1.0, 300.0, 4.2);
        assert_relative(q2 / q1, 10.0, 0.01);
    }

    #[test]
    fn test_current_lead_heat_load() {
        let q = HeatLoadCalculator::current_lead_heat_load(100.0, 2);
        // 47 mW/A per lead pair × 100 A = 4.7 W
        assert_relative(q, 4.7, 0.01);
    }

    #[test]
    fn test_current_lead_zero_current() {
        let q = HeatLoadCalculator::current_lead_heat_load(0.0, 2);
        assert_approx(q, 0.0, EPSILON);
    }

    #[test]
    fn test_total_heat_load() {
        let mut calc = HeatLoadCalculator::new();
        calc.set_radiation(0.5);
        calc.set_conduction(0.3);
        calc.set_residual_gas(0.1);
        calc.set_current_leads(1.0);
        calc.set_additional(0.2);
        assert_approx(calc.total_heat_load(), 2.1, EPSILON);
    }

    // -----------------------------------------------------------------------
    // BoiloffPredictor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_boiloff_rate_1w() {
        let pred = BoiloffPredictor::new(1.0);
        let rate = pred.boiloff_rate_liters_per_hour();
        // 1W → dm/dt = 1/20700 kg/s = 4.831e-5 kg/s
        // dV/dt = 4.831e-5 / 125 m³/s = 3.865e-7 m³/s
        // = 3.865e-4 L/s = 1.391 L/hr
        assert_relative(rate, 1.391, 0.02);
    }

    #[test]
    fn test_boiloff_rate_kg() {
        let pred = BoiloffPredictor::new(1.0);
        let rate_kg = pred.boiloff_rate_kg_per_hour();
        let rate_l = pred.boiloff_rate_liters_per_hour();
        // Should be consistent: rate_kg = rate_l * density / 1000
        let density = HeliumProperties::density_kg_per_m3();
        assert_relative(rate_kg, rate_l * density / 1000.0, 0.01);
    }

    #[test]
    fn test_hold_time() {
        let pred = BoiloffPredictor::new(1.0);
        let rate = pred.boiloff_rate_liters_per_hour();
        let hold = pred.hold_time_hours(100.0);
        assert_relative(hold, 100.0 / rate, 0.01);
    }

    #[test]
    fn test_hold_time_zero_heat_load() {
        let pred = BoiloffPredictor::new(0.0);
        assert!(pred.hold_time_hours(100.0).is_infinite());
    }

    #[test]
    fn test_annual_consumption() {
        let pred = BoiloffPredictor::new(1.0);
        let annual = pred.annual_consumption_liters();
        let hourly = pred.boiloff_rate_liters_per_hour();
        assert_relative(annual, hourly * 24.0 * 365.25, 0.001);
    }

    #[test]
    fn test_recovery_reduces_boiloff() {
        let pred = BoiloffPredictor::new(1.0);
        let rate_no_recovery = pred.boiloff_rate_liters_per_hour();

        let pred_with_recovery = pred.with_recovery_efficiency(0.8);
        let rate_with_recovery = pred_with_recovery.boiloff_rate_liters_per_hour();

        assert_relative(rate_with_recovery, rate_no_recovery * 0.2, 0.01);
    }

    #[test]
    fn test_recovery_clamps() {
        let pred = BoiloffPredictor::new(1.0);
        let p100 = pred.with_recovery_efficiency(1.5);
        assert_approx(p100.boiloff_rate_liters_per_hour(), 0.0, 1e-10);
    }

    #[test]
    fn test_effective_heat_load() {
        let pred = BoiloffPredictor::new(10.0).with_recovery_efficiency(0.5);
        assert_approx(pred.effective_heat_load_w(), 5.0, EPSILON);
    }

    // -----------------------------------------------------------------------
    // DewarModel tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dewar_surface_area() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        let expected = 2.0 * std::f64::consts::PI * 0.2 * 1.0
            + 2.0 * std::f64::consts::PI * 0.2 * 0.2;
        assert_relative(dewar.surface_area(), expected, 0.001);
    }

    #[test]
    fn test_dewar_inner_volume() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        let expected_m3 = std::f64::consts::PI * 0.04 * 1.0;
        let expected_liters = expected_m3 * 1000.0;
        assert_relative(dewar.inner_volume_liters(), expected_liters, 0.001);
    }

    #[test]
    fn test_dewar_vacuum_gap() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        assert_approx(dewar.vacuum_gap(), 0.1, EPSILON);
    }

    #[test]
    fn test_dewar_static_heat_load_positive() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        assert!(dewar.static_heat_load() > 0.0);
    }

    #[test]
    fn test_dewar_neck_tube_loss() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        let q = dewar.neck_tube_loss(0.05, 0.5, NeckMaterial::StainlessSteel304);
        assert!(q > 0.0, "Neck tube loss should be positive");
    }

    #[test]
    fn test_dewar_neck_tube_thin_wall_less() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        let q_normal = dewar.neck_tube_loss(0.05, 0.5, NeckMaterial::StainlessSteel304);
        let q_thin = dewar.neck_tube_loss(0.05, 0.5, NeckMaterial::ThinWallStainless);
        assert!(q_thin < q_normal, "Thin-wall should have less heat leak");
    }

    #[test]
    fn test_dewar_fill_level_from_pressure() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        // Full vessel: ΔP = ρgh = 125 * 9.81 * 1.0 = 1226.25 Pa
        let level = dewar.fill_level_from_pressure(1226.25, 1.0);
        assert_relative(level, 1.0, 0.01);
    }

    #[test]
    fn test_dewar_fill_level_half() {
        let dewar = DewarModel::new(0.2, 0.3, 1.0, 30);
        let half_dp = 125.0 * 9.81 * 1.0 * 0.5;
        let level = dewar.fill_level_from_pressure(half_dp, 1.0);
        assert_relative(level, 0.5, 0.01);
    }

    // -----------------------------------------------------------------------
    // LevelSensor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_level_dp_full() {
        let dp = 125.0 * 9.81 * 1.0; // full
        let level = LevelSensor::from_differential_pressure(dp, 1.0);
        assert_relative(level, 1.0, 0.01);
    }

    #[test]
    fn test_level_dp_empty() {
        let level = LevelSensor::from_differential_pressure(0.0, 1.0);
        assert_approx(level, 0.0, EPSILON);
    }

    #[test]
    fn test_level_capacitance() {
        let level = LevelSensor::from_capacitance(1.5, 1.0, 2.0);
        assert_approx(level, 0.5, EPSILON);
    }

    #[test]
    fn test_level_capacitance_empty() {
        let level = LevelSensor::from_capacitance(1.0, 1.0, 2.0);
        assert_approx(level, 0.0, EPSILON);
    }

    #[test]
    fn test_level_capacitance_full() {
        let level = LevelSensor::from_capacitance(2.0, 1.0, 2.0);
        assert_approx(level, 1.0, EPSILON);
    }

    #[test]
    fn test_level_sc_wire_full() {
        // Fully immersed: R = 0 → ratio = 0 → level = 1.0
        let level = LevelSensor::from_superconducting_wire(0.0);
        assert_approx(level, 1.0, EPSILON);
    }

    #[test]
    fn test_level_sc_wire_empty() {
        // Fully exposed: R = R_total → ratio = 1.0 → level = 0.0
        let level = LevelSensor::from_superconducting_wire(1.0);
        assert_approx(level, 0.0, EPSILON);
    }

    #[test]
    fn test_level_sc_wire_half() {
        let level = LevelSensor::from_superconducting_wire(0.5);
        assert_approx(level, 0.5, EPSILON);
    }

    #[test]
    fn test_predict_time_to_refill() {
        // 80% full, losing 2% per hour, minimum 20%
        let hours = LevelSensor::predict_time_to_refill(80.0, 2.0, 20.0);
        assert_approx(hours, 30.0, EPSILON);
    }

    #[test]
    fn test_predict_time_already_below_min() {
        let hours = LevelSensor::predict_time_to_refill(10.0, 2.0, 20.0);
        assert_approx(hours, 0.0, EPSILON);
    }

    #[test]
    fn test_predict_time_zero_boiloff() {
        let hours = LevelSensor::predict_time_to_refill(80.0, 0.0, 20.0);
        assert!(hours.is_infinite());
    }

    // -----------------------------------------------------------------------
    // QuenchAnalyzer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_quench_energy() {
        // E = 0.5 * 10H * 100A² = 50,000 J
        let e = QuenchAnalyzer::quench_energy(10.0, 100.0);
        assert_approx(e, 50_000.0, EPSILON);
    }

    #[test]
    fn test_quench_energy_mri_magnet() {
        // Typical MRI: L ≈ 50 H, I ≈ 400 A → E = 4 MJ
        let e = QuenchAnalyzer::quench_energy(50.0, 400.0);
        assert_approx(e, 4_000_000.0, EPSILON);
    }

    #[test]
    fn test_helium_flash_off() {
        let e = 50_000.0; // 50 kJ
        let liters = QuenchAnalyzer::helium_flash_off(e);
        // mass = 50000/20700 = 2.415 kg
        // volume = 2.415/125 = 0.01932 m³ = 19.32 liters
        assert_relative(liters, 19.32, 0.02);
    }

    #[test]
    fn test_pressure_rise() {
        let energy = 50_000.0; // 50 kJ
        let volume = 0.1; // 100 liters = 0.1 m³
        let dp = QuenchAnalyzer::pressure_rise(volume, energy);
        assert!(dp > 0.0, "Pressure rise should be positive");
    }

    #[test]
    fn test_pressure_rise_zero_volume() {
        assert_approx(QuenchAnalyzer::pressure_rise(0.0, 1000.0), 0.0, EPSILON);
    }

    #[test]
    fn test_recovery_time() {
        let time = QuenchAnalyzer::recovery_time(1000.0, 100.0);
        assert_approx(time, 10.0, EPSILON);
    }

    #[test]
    fn test_recovery_time_zero_rate() {
        assert!(QuenchAnalyzer::recovery_time(100.0, 0.0).is_infinite());
    }

    // -----------------------------------------------------------------------
    // CoolingPowerBudget tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gm_stage1_capacity() {
        let cap = CoolingPowerBudget::gifford_mcmahon_capacity(1, 50.0);
        assert!(cap > 20.0, "GM stage 1 at 50K should provide >20W, got {}", cap);
    }

    #[test]
    fn test_gm_stage2_at_4k() {
        let cap = CoolingPowerBudget::gifford_mcmahon_capacity(2, 4.2);
        assert!(cap > 0.5 && cap < 3.0, "GM stage 2 at 4.2K: {} W", cap);
    }

    #[test]
    fn test_gm_stage2_decreases_at_lower_t() {
        let cap_4k = CoolingPowerBudget::gifford_mcmahon_capacity(2, 4.2);
        let cap_2k = CoolingPowerBudget::gifford_mcmahon_capacity(2, 2.0);
        assert!(cap_2k < cap_4k, "Capacity should decrease at lower T");
    }

    #[test]
    fn test_pulse_tube_at_4k() {
        let cap = CoolingPowerBudget::pulse_tube_capacity(4.2);
        assert!(cap > 0.3 && cap < 2.0, "PTC at 4.2K: {} W", cap);
    }

    #[test]
    fn test_cooling_margin() {
        let margin = CoolingPowerBudget::cooling_margin(2.0, 1.5);
        assert_relative(margin, 1.333, 0.01);
    }

    #[test]
    fn test_cooling_margin_zero_load() {
        assert!(CoolingPowerBudget::cooling_margin(2.0, 0.0).is_infinite());
    }

    #[test]
    fn test_zero_boiloff_feasible() {
        assert!(CoolingPowerBudget::zero_boiloff_feasible(1.0, 1.5));
        assert!(!CoolingPowerBudget::zero_boiloff_feasible(1.0, 1.05));
        assert!(!CoolingPowerBudget::zero_boiloff_feasible(1.0, 0.8));
    }

    // -----------------------------------------------------------------------
    // HeliumRecoverySystem tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_recovery_efficiency() {
        let eff = HeliumRecoverySystem::recovery_efficiency(0.95, 0.05);
        assert_relative(eff, 0.9025, 0.001);
    }

    #[test]
    fn test_recovery_efficiency_perfect() {
        let eff = HeliumRecoverySystem::recovery_efficiency(1.0, 0.0);
        assert_approx(eff, 1.0, EPSILON);
    }

    #[test]
    fn test_recovery_efficiency_clamp() {
        let eff = HeliumRecoverySystem::recovery_efficiency(1.5, -0.1);
        assert!(eff <= 1.0);
    }

    #[test]
    fn test_reliquefaction_power() {
        let power = HeliumRecoverySystem::reliquefaction_power_w(1.0);
        // Should be significant power (kW range) for 1 L/hr
        assert!(power > 100.0, "Reliquefaction should need significant power: {} W", power);
    }

    #[test]
    fn test_reliquefaction_power_scales() {
        let p1 = HeliumRecoverySystem::reliquefaction_power_w(1.0);
        let p2 = HeliumRecoverySystem::reliquefaction_power_w(2.0);
        assert_relative(p2 / p1, 2.0, 0.01);
    }

    #[test]
    fn test_cost_analysis() {
        let result = HeliumRecoverySystem::cost_analysis(1000.0, 30.0, 0.8);
        assert_approx(result.annual_cost_no_recovery, 30000.0, EPSILON);
        assert_approx(result.annual_cost_with_recovery, 6000.0, EPSILON);
        assert_approx(result.annual_savings, 24000.0, EPSILON);
        assert_approx(result.consumption_no_recovery, 1000.0, EPSILON);
        assert_approx(result.consumption_with_recovery, 200.0, EPSILON);
    }

    #[test]
    fn test_cost_analysis_no_recovery() {
        let result = HeliumRecoverySystem::cost_analysis(1000.0, 30.0, 0.0);
        assert_approx(result.annual_savings, 0.0, EPSILON);
    }

    #[test]
    fn test_global_supply_context_not_empty() {
        let ctx = HeliumRecoverySystem::global_supply_context();
        assert!(!ctx.is_empty());
        assert!(ctx.contains("Helium"));
    }

    // -----------------------------------------------------------------------
    // Integration / scenario tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_typical_mri_scenario() {
        // Typical clinical MRI without cryocooler: ~1.5 W total heat load
        // At 1.391 L/hr/W → ~2.09 L/hr → ~18,300 L/yr without recovery
        // With recovery or lower heat load (modern MRI ~0.1-0.3 W), consumption drops.
        // Here we test the raw calculation at 1.5 W.
        let pred = BoiloffPredictor::new(1.5);
        let annual = pred.annual_consumption_liters();
        assert!(
            annual > 10000.0 && annual < 25000.0,
            "MRI annual consumption {} L/yr at 1.5W seems wrong",
            annual
        );
    }

    #[test]
    fn test_zero_boiloff_mri() {
        let heat_load = 0.8; // W, well-insulated modern Dewar
        let cooler_cap = CoolingPowerBudget::gifford_mcmahon_capacity(2, 4.2);
        let feasible = CoolingPowerBudget::zero_boiloff_feasible(heat_load, cooler_cap);
        assert!(feasible, "Modern MRI with GM cooler should achieve zero-boiloff");
    }

    #[test]
    fn test_dewar_full_workflow() {
        let dewar = DewarModel::new(0.15, 0.25, 0.8, 40);
        let static_q = dewar.static_heat_load();
        let neck_q = dewar.neck_tube_loss(0.04, 0.4, NeckMaterial::StainlessSteel304);
        let total_q = static_q + neck_q;

        let pred = BoiloffPredictor::new(total_q);
        let volume = dewar.inner_volume_liters();
        let hold_time = pred.hold_time_hours(volume);

        assert!(hold_time > 0.0, "Hold time should be positive");
        assert!(volume > 0.0, "Volume should be positive");
    }

    #[test]
    fn test_quench_scenario() {
        // Large accelerator magnet: L=200 H, I=5000 A
        let energy = QuenchAnalyzer::quench_energy(200.0, 5000.0);
        assert_approx(energy, 2.5e9, 1e6); // 2.5 GJ

        let flash = QuenchAnalyzer::helium_flash_off(energy);
        assert!(flash > 1000.0, "Should evaporate significant amount: {} L", flash);

        let dp = QuenchAnalyzer::pressure_rise(10.0, energy);
        assert!(dp > ATM_PA, "Quench should create significant pressure rise");
    }
}
