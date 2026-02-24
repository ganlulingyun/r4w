//! Atom Probe Tomography (APT) Data Processing
//!
//! This module implements 3D atomic-scale compositional analysis from APT
//! experiments. APT works by field-evaporating atoms from a sharp needle-shaped
//! specimen using a high DC voltage (and optionally laser/voltage pulses). Each
//! evaporated ion is recorded by a position-sensitive detector with nanosecond
//! time-of-flight (TOF) resolution, enabling both chemical identification
//! (via mass-to-charge ratio) and 3D spatial reconstruction.
//!
//! # Key Physical Quantities
//!
//! - **m/n** – mass-to-charge ratio (Da = Daltons per elementary charge)
//! - **TOF** – time-of-flight (ns)
//! - **V_dc** – standing DC voltage (kV)
//! - **V_pulse** – superimposed voltage pulse (kV)
//! - **η** – detection efficiency (typically 0.37–0.80)
//! - **k_f** – field factor (unitless, ~3.3–5.5)
//! - **ξ** – image compression factor (typically 1.3–1.8)
//! - **β** – detector efficiency parameter
//!
//! # References
//!
//! - Bas, P. et al., "A general protocol for the reconstruction of needle-shaped
//!   samples for atom probe: application to the characterization of oxide scales",
//!   Appl. Surf. Sci. 87/88, 298–304, 1995.
//! - Blavette, D. et al., "An atom probe for three-dimensional tomography",
//!   Nature 363, 432–435, 1993.
//! - Hellman, O.C. et al., "Analysis of three-dimensional atom-probe data by the
//!   proximity histogram", Microscopy and Microanalysis 6, 437–444, 2000.

use std::collections::HashMap;

// ─────────────────────────────────────────────────────────────────────────────
// Physical constants
// ─────────────────────────────────────────────────────────────────────────────

/// Elementary charge (C)
const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;
/// Unified atomic mass unit (kg)
const ATOMIC_MASS_UNIT: f64 = 1.660_539_066_6e-27;
/// 1 Dalton in kg
const DALTON_KG: f64 = ATOMIC_MASS_UNIT;
/// Conversion: 1 kV = 1000 V
const KV_TO_V: f64 = 1000.0;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

/// A single detected ion hit.
#[derive(Debug, Clone)]
pub struct IonHit {
    /// Time-of-flight in nanoseconds.
    pub tof_ns: f64,
    /// DC standing voltage at the time of evaporation (kV).
    pub v_dc_kv: f64,
    /// Pulse voltage (kV). Zero for DC-only instruments.
    pub v_pulse_kv: f64,
    /// Detector X position (mm).
    pub x_det: f64,
    /// Detector Y position (mm).
    pub y_det: f64,
    /// Sequence number (pulse index).
    pub pulse_index: u64,
}

/// Reconstructed 3D atom position with chemical assignment.
#[derive(Debug, Clone)]
pub struct ReconstructedAtom {
    /// X coordinate (nm).
    pub x_nm: f64,
    /// Y coordinate (nm).
    pub y_nm: f64,
    /// Z depth (nm, increasing away from the tip apex).
    pub z_nm: f64,
    /// Assigned mass-to-charge ratio (Da/e).
    pub mass_to_charge: f64,
    /// Element symbol (e.g., "Fe", "C").
    pub element: String,
    /// Charge state (ionisation number).
    pub charge_state: u8,
}

/// Isotope definition for peak ranging.
#[derive(Debug, Clone)]
pub struct Isotope {
    /// Element symbol.
    pub element: String,
    /// Mass number (A).
    pub mass_number: u32,
    /// Natural abundance (0–1).
    pub abundance: f64,
    /// Monoisotopic mass (Da).
    pub mass_da: f64,
    /// Common charge states in APT.
    pub charge_states: Vec<u8>,
}

/// A mass spectrum peak range.
#[derive(Debug, Clone)]
pub struct PeakRange {
    /// Element symbol assigned to this range.
    pub element: String,
    /// Charge state.
    pub charge_state: u8,
    /// Lower bound of the range (Da/e).
    pub lower_da: f64,
    /// Upper bound of the range (Da/e).
    pub upper_da: f64,
    /// Theoretical m/n centre (Da/e).
    pub theoretical_mn: f64,
}

/// Cluster found by maximum-separation method.
#[derive(Debug, Clone)]
pub struct AtomCluster {
    /// Cluster index.
    pub id: usize,
    /// Indices into the reconstructed atom list.
    pub atom_indices: Vec<usize>,
    /// Centre of mass (nm).
    pub centroid_nm: [f64; 3],
    /// Guinier radius (nm).
    pub guinier_radius_nm: f64,
    /// Number density (#/nm³) relative to analysis volume.
    pub number_density: f64,
}

/// Material preset with common APT elements.
#[derive(Debug, Clone)]
pub struct MaterialPreset {
    /// Name of the alloy system.
    pub name: String,
    /// Nominal composition as (element, atomic fraction).
    pub composition: Vec<(String, f64)>,
    /// Representative evaporation field (V/nm).
    pub evaporation_field: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Mass-to-charge and TOF conversion
// ─────────────────────────────────────────────────────────────────────────────

/// Calculate mass-to-charge ratio (Da/e) from time-of-flight.
///
/// Uses the simplified kinetic energy relation:
///
/// m/n = 2 · e · (V_dc + V_pulse) · (t / L)²  / (2 · u)
///
/// Simplified: m/n [Da] = 2·n·e·V·(t/L)² / (2·u)  → after cancellation:
///
/// `m/n [Da/e] = (e / u) · V [V] · (t_s / L_m)²`
///
/// where t_s = tof_ns × 10⁻⁹ s and L_m = flight_length_mm × 10⁻³ m.
///
/// # Arguments
/// * `tof_ns` – time-of-flight in nanoseconds.
/// * `v_dc_kv` – DC voltage in kilovolts.
/// * `v_pulse_kv` – pulse voltage in kilovolts (add to DC).
/// * `flight_length_mm` – effective flight path length in millimetres.
///
/// # Returns
/// Mass-to-charge ratio in Daltons per elementary charge (Da/e).
pub fn tof_to_mass_to_charge(
    tof_ns: f64,
    v_dc_kv: f64,
    v_pulse_kv: f64,
    flight_length_mm: f64,
) -> f64 {
    let v_total = (v_dc_kv + v_pulse_kv) * KV_TO_V; // V
    let t_s = tof_ns * 1.0e-9; // s
    let l_m = flight_length_mm * 1.0e-3; // m
    if l_m == 0.0 || t_s <= 0.0 || v_total <= 0.0 {
        return 0.0;
    }
    // kinetic energy conservation: q·V = ½·m·v² → m/q = 2·V·(t/L)²·(e/u) in Da/e
    let ratio = t_s / l_m;
    (ELEMENTARY_CHARGE / DALTON_KG) * v_total * ratio * ratio
}

/// Inverse: compute TOF (ns) from a known m/n value.
///
/// # Arguments
/// * `mn_da` – mass-to-charge ratio in Da/e.
/// * `v_dc_kv` – DC voltage in kV.
/// * `v_pulse_kv` – pulse voltage in kV.
/// * `flight_length_mm` – flight path in mm.
pub fn mass_to_charge_to_tof(
    mn_da: f64,
    v_dc_kv: f64,
    v_pulse_kv: f64,
    flight_length_mm: f64,
) -> f64 {
    let v_total = (v_dc_kv + v_pulse_kv) * KV_TO_V;
    let l_m = flight_length_mm * 1.0e-3;
    if v_total <= 0.0 || mn_da <= 0.0 {
        return 0.0;
    }
    // t = L * sqrt(mn_da * u / (e * V))
    let t_s = l_m * (mn_da * DALTON_KG / (ELEMENTARY_CHARGE * v_total)).sqrt();
    t_s * 1.0e9 // ns
}

// ─────────────────────────────────────────────────────────────────────────────
// Mass spectrum
// ─────────────────────────────────────────────────────────────────────────────

/// Compute a binned mass spectrum histogram.
///
/// # Arguments
/// * `mn_values` – slice of m/n values (Da/e).
/// * `mn_min` – lower bound of spectrum (Da/e).
/// * `mn_max` – upper bound of spectrum (Da/e).
/// * `num_bins` – number of bins.
///
/// # Returns
/// Vector of (bin_centre_da, count) pairs.
pub fn compute_mass_spectrum(
    mn_values: &[f64],
    mn_min: f64,
    mn_max: f64,
    num_bins: usize,
) -> Vec<(f64, u64)> {
    if num_bins == 0 || mn_max <= mn_min {
        return Vec::new();
    }
    let bin_width = (mn_max - mn_min) / num_bins as f64;
    let mut counts = vec![0u64; num_bins];
    for &mn in mn_values {
        if mn >= mn_min && mn < mn_max {
            let idx = ((mn - mn_min) / bin_width) as usize;
            let idx = idx.min(num_bins - 1);
            counts[idx] += 1;
        }
    }
    counts
        .into_iter()
        .enumerate()
        .map(|(i, c)| (mn_min + (i as f64 + 0.5) * bin_width, c))
        .collect()
}

/// Background noise floor estimation using the minimum-average method.
///
/// Computes the mean count in the lowest `fraction` of bins (by count) to
/// estimate detector noise.
///
/// # Arguments
/// * `spectrum` – output of [`compute_mass_spectrum`].
/// * `fraction` – fraction of bins to average (e.g., 0.1 = lowest 10%).
///
/// # Returns
/// Estimated background counts per bin.
pub fn estimate_background(spectrum: &[(f64, u64)], fraction: f64) -> f64 {
    if spectrum.is_empty() || fraction <= 0.0 {
        return 0.0;
    }
    let mut counts: Vec<u64> = spectrum.iter().map(|&(_, c)| c).collect();
    counts.sort_unstable();
    let n = ((counts.len() as f64 * fraction).ceil() as usize).max(1);
    let n = n.min(counts.len());
    let sum: u64 = counts[..n].iter().sum();
    sum as f64 / n as f64
}

/// Signal-to-noise ratio for a peak range.
///
/// SNR = (peak_count - background * width_bins) / sqrt(background * width_bins)
///
/// where width_bins is the number of bins in [lower_da, upper_da].
///
/// # Returns
/// SNR value; `f64::INFINITY` if background is zero.
pub fn peak_snr(
    spectrum: &[(f64, u64)],
    lower_da: f64,
    upper_da: f64,
    background_per_bin: f64,
) -> f64 {
    let peak_count: u64 = spectrum
        .iter()
        .filter(|&&(mn, _)| mn >= lower_da && mn <= upper_da)
        .map(|&(_, c)| c)
        .sum();
    let width_bins = spectrum
        .iter()
        .filter(|&&(mn, _)| mn >= lower_da && mn <= upper_da)
        .count() as f64;
    if width_bins == 0.0 {
        return 0.0;
    }
    let bg = background_per_bin * width_bins;
    let signal = peak_count as f64 - bg;
    if bg <= 0.0 {
        return f64::INFINITY;
    }
    signal / bg.sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────
// Isotope library
// ─────────────────────────────────────────────────────────────────────────────

/// Return a built-in isotope library for common APT elements.
pub fn builtin_isotope_library() -> Vec<Isotope> {
    vec![
        // Iron
        Isotope { element: "Fe".into(), mass_number: 54, abundance: 0.0585, mass_da: 53.940, charge_states: vec![2, 3] },
        Isotope { element: "Fe".into(), mass_number: 56, abundance: 0.9175, mass_da: 55.935, charge_states: vec![2, 3] },
        Isotope { element: "Fe".into(), mass_number: 57, abundance: 0.0212, mass_da: 56.935, charge_states: vec![2, 3] },
        Isotope { element: "Fe".into(), mass_number: 58, abundance: 0.0028, mass_da: 57.933, charge_states: vec![2, 3] },
        // Carbon
        Isotope { element: "C".into(),  mass_number: 12, abundance: 0.9893, mass_da: 12.000, charge_states: vec![1, 2] },
        Isotope { element: "C".into(),  mass_number: 13, abundance: 0.0107, mass_da: 13.003, charge_states: vec![1, 2] },
        // Manganese
        Isotope { element: "Mn".into(), mass_number: 55, abundance: 1.0000, mass_da: 54.938, charge_states: vec![2, 3] },
        // Silicon
        Isotope { element: "Si".into(), mass_number: 28, abundance: 0.9223, mass_da: 27.977, charge_states: vec![2, 3] },
        Isotope { element: "Si".into(), mass_number: 29, abundance: 0.0467, mass_da: 28.976, charge_states: vec![2, 3] },
        Isotope { element: "Si".into(), mass_number: 30, abundance: 0.0310, mass_da: 29.974, charge_states: vec![2, 3] },
        // Aluminium
        Isotope { element: "Al".into(), mass_number: 27, abundance: 1.0000, mass_da: 26.982, charge_states: vec![2, 3] },
        // Copper
        Isotope { element: "Cu".into(), mass_number: 63, abundance: 0.6915, mass_da: 62.930, charge_states: vec![1, 2] },
        Isotope { element: "Cu".into(), mass_number: 65, abundance: 0.3085, mass_da: 64.928, charge_states: vec![1, 2] },
        // Magnesium
        Isotope { element: "Mg".into(), mass_number: 24, abundance: 0.7899, mass_da: 23.985, charge_states: vec![2, 3] },
        Isotope { element: "Mg".into(), mass_number: 25, abundance: 0.1000, mass_da: 24.985, charge_states: vec![2, 3] },
        Isotope { element: "Mg".into(), mass_number: 26, abundance: 0.1101, mass_da: 25.983, charge_states: vec![2, 3] },
        // Zinc
        Isotope { element: "Zn".into(), mass_number: 64, abundance: 0.4863, mass_da: 63.929, charge_states: vec![2] },
        Isotope { element: "Zn".into(), mass_number: 66, abundance: 0.2790, mass_da: 65.926, charge_states: vec![2] },
        Isotope { element: "Zn".into(), mass_number: 68, abundance: 0.1875, mass_da: 67.925, charge_states: vec![2] },
        // Nickel
        Isotope { element: "Ni".into(), mass_number: 58, abundance: 0.6808, mass_da: 57.935, charge_states: vec![2, 3] },
        Isotope { element: "Ni".into(), mass_number: 60, abundance: 0.2622, mass_da: 59.931, charge_states: vec![2, 3] },
        Isotope { element: "Ni".into(), mass_number: 62, abundance: 0.0364, mass_da: 61.928, charge_states: vec![2, 3] },
        // Chromium
        Isotope { element: "Cr".into(), mass_number: 52, abundance: 0.8379, mass_da: 51.941, charge_states: vec![2, 3] },
        Isotope { element: "Cr".into(), mass_number: 53, abundance: 0.0950, mass_da: 52.941, charge_states: vec![2, 3] },
        // Cobalt
        Isotope { element: "Co".into(), mass_number: 59, abundance: 1.0000, mass_da: 58.933, charge_states: vec![2, 3] },
        // Nitrogen (for steel)
        Isotope { element: "N".into(),  mass_number: 14, abundance: 0.9963, mass_da: 14.003, charge_states: vec![1, 2] },
        // Oxygen
        Isotope { element: "O".into(),  mass_number: 16, abundance: 0.9976, mass_da: 15.995, charge_states: vec![1, 2] },
        // Phosphorus
        Isotope { element: "P".into(),  mass_number: 31, abundance: 1.0000, mass_da: 30.974, charge_states: vec![1, 2] },
    ]
}

/// Generate peak ranges from an isotope library for a given charge state.
///
/// The range is set to ±`half_width_da` around the theoretical m/n.
pub fn generate_peak_ranges(
    library: &[Isotope],
    charge_state: u8,
    half_width_da: f64,
) -> Vec<PeakRange> {
    let mut ranges = Vec::new();
    for iso in library {
        if iso.charge_states.contains(&charge_state) {
            let mn = iso.mass_da / charge_state as f64;
            ranges.push(PeakRange {
                element: iso.element.clone(),
                charge_state,
                lower_da: mn - half_width_da,
                upper_da: mn + half_width_da,
                theoretical_mn: mn,
            });
        }
    }
    ranges
}

/// Count ions within each peak range and return elemental counts.
///
/// # Returns
/// Map from element symbol to total detected count.
pub fn range_ions(mn_values: &[f64], ranges: &[PeakRange]) -> HashMap<String, u64> {
    let mut counts: HashMap<String, u64> = HashMap::new();
    for &mn in mn_values {
        for range in ranges {
            if mn >= range.lower_da && mn <= range.upper_da {
                *counts.entry(range.element.clone()).or_insert(0) += 1;
                break; // assign to first matching range only
            }
        }
    }
    counts
}

// ─────────────────────────────────────────────────────────────────────────────
// Kingham curves (field ionisation charge-state ratios)
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate the Kingham charge-state ratio R = P(q+1) / P(q) for a given
/// evaporation field.
///
/// The Kingham model gives the probability ratio of successive charge states
/// during field evaporation. This simplified parameterisation uses an
/// exponential model fit to published Kingham curves:
///
/// `R(F) = A · exp(B · F)`
///
/// where F is the evaporation field in V/nm.
///
/// # Arguments
/// * `evaporation_field_v_per_nm` – local evaporation field at the tip surface.
/// * `element` – element symbol for preset A/B parameters.
///
/// # Returns
/// Probability ratio P(q+1)/P(q); values > 1 favour higher charge states.
pub fn kingham_charge_ratio(evaporation_field_v_per_nm: f64, element: &str) -> f64 {
    // Simplified A, B parameters from fits to Kingham curves (Kingham 1982)
    let (a, b) = match element {
        "Fe" => (1.2e-4_f64, 0.085_f64),
        "Ni" => (0.8e-4_f64, 0.090_f64),
        "Al" => (2.5e-3_f64, 0.065_f64),
        "Cu" => (3.0e-3_f64, 0.060_f64),
        "W"  => (1.0e-5_f64, 0.120_f64),
        _    => (1.0e-3_f64, 0.075_f64), // generic default
    };
    a * (b * evaporation_field_v_per_nm).exp()
}

/// Probability of charge state `q` given total field for a single element.
///
/// Normalises over charge states 1–4 using the Kingham ratios.
pub fn kingham_charge_probability(
    evaporation_field_v_per_nm: f64,
    element: &str,
    target_charge: u8,
) -> f64 {
    let r = kingham_charge_ratio(evaporation_field_v_per_nm, element);
    // P(1) = 1 (unnormalised), P(2) = R·P(1), P(3) = R·P(2), ...
    let unnorm: Vec<f64> = (1u8..=4).map(|q| r.powi(q as i32 - 1)).collect();
    let total: f64 = unnorm.iter().sum();
    if target_charge < 1 || target_charge > 4 {
        return 0.0;
    }
    unnorm[(target_charge - 1) as usize] / total
}

// ─────────────────────────────────────────────────────────────────────────────
// Spatial reconstruction (Bas et al. protocol)
// ─────────────────────────────────────────────────────────────────────────────

/// Parameters for the Bas et al. point-projection reconstruction.
#[derive(Debug, Clone)]
pub struct ReconstructionParams {
    /// Initial tip radius (nm).
    pub tip_radius_nm: f64,
    /// Image compression factor ξ (dimensionless, typically 1.3–1.8).
    pub image_compression_factor: f64,
    /// Field factor k_f (dimensionless, typically 3.3–5.5).
    pub field_factor: f64,
    /// Detector-to-specimen distance L (mm).
    pub flight_length_mm: f64,
    /// Evaporation field F_ev (V/nm).
    pub evaporation_field_v_per_nm: f64,
    /// Detection efficiency η (0–1).
    pub detection_efficiency: f64,
    /// Atomic volume Ω (nm³/atom). Typical steel: ~0.0118.
    pub atomic_volume_nm3: f64,
}

impl Default for ReconstructionParams {
    fn default() -> Self {
        Self {
            tip_radius_nm: 50.0,
            image_compression_factor: 1.56,
            field_factor: 4.0,
            flight_length_mm: 90.0,
            evaporation_field_v_per_nm: 33.0,
            detection_efficiency: 0.5,
            atomic_volume_nm3: 0.0118,
        }
    }
}

/// Compute the tip radius from the instantaneous DC voltage.
///
/// R_tip = V_dc / (k_f · F_ev)
///
/// where V_dc is in volts, k_f is the field factor, F_ev is in V/nm.
///
/// # Returns
/// Tip radius in nm.
pub fn tip_radius_from_voltage(v_dc_kv: f64, params: &ReconstructionParams) -> f64 {
    let v_dc = v_dc_kv * KV_TO_V; // V
    let f_ev_v_per_m = params.evaporation_field_v_per_nm * 1.0e9; // V/m
    v_dc / (params.field_factor * f_ev_v_per_m) * 1.0e9 // nm
}

/// Reconstruct the (x, y, z) position of a single atom using the
/// Bas point-projection protocol.
///
/// # Arguments
/// * `hit` – detector hit data.
/// * `params` – reconstruction parameters.
/// * `z_offset_nm` – accumulated depth offset (nm) from previously evaporated atoms.
///
/// # Returns
/// `(x_nm, y_nm, z_nm)` in specimen coordinates.
pub fn reconstruct_position(
    hit: &IonHit,
    params: &ReconstructionParams,
    z_offset_nm: f64,
) -> (f64, f64, f64) {
    let r_tip = tip_radius_from_voltage(hit.v_dc_kv, params);
    let l_mm = params.flight_length_mm;
    let xi = params.image_compression_factor;
    // Lateral projection
    let x_nm = hit.x_det / (xi * l_mm) * r_tip;
    let y_nm = hit.y_det / (xi * l_mm) * r_tip;
    // Depth increment per atom (Bas formula):
    // Δz = Ω / (π · R² · η) where Ω = atomic volume, R = tip radius, η = efficiency
    let delta_z = params.atomic_volume_nm3
        / (std::f64::consts::PI * r_tip * r_tip * params.detection_efficiency);
    let z_nm = z_offset_nm + delta_z;
    (x_nm, y_nm, z_nm)
}

/// Reconstruct a full dataset from a list of ion hits.
///
/// Assigns dummy element "?" and charge state 0 (for composition analysis,
/// range the resulting atoms separately).
///
/// # Returns
/// Vector of reconstructed atoms with positions only.
pub fn reconstruct_dataset(hits: &[IonHit], params: &ReconstructionParams) -> Vec<ReconstructedAtom> {
    let mut atoms = Vec::with_capacity(hits.len());
    let mut z_nm = 0.0;
    for hit in hits {
        let (x, y, z) = reconstruct_position(hit, params, z_nm);
        // Update z_nm for next atom
        let r_tip = tip_radius_from_voltage(hit.v_dc_kv, params);
        let delta_z = params.atomic_volume_nm3
            / (std::f64::consts::PI * r_tip * r_tip * params.detection_efficiency);
        z_nm += delta_z;
        let mn = tof_to_mass_to_charge(
            hit.tof_ns,
            hit.v_dc_kv,
            hit.v_pulse_kv,
            params.flight_length_mm,
        );
        atoms.push(ReconstructedAtom {
            x_nm: x,
            y_nm: y,
            z_nm: z,
            mass_to_charge: mn,
            element: "?".to_string(),
            charge_state: 0,
        });
    }
    atoms
}

// ─────────────────────────────────────────────────────────────────────────────
// Voltage curve analysis
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate evaporation field from a voltage–radius curve.
///
/// Given pairs (V_dc_kv, R_tip_nm), compute F_ev = V / (k_f · R) for each point.
///
/// # Returns
/// Mean estimated evaporation field in V/nm.
pub fn estimate_evaporation_field(
    v_dc_kv_history: &[f64],
    field_factor: f64,
) -> Vec<f64> {
    v_dc_kv_history
        .iter()
        .map(|&v| {
            // Use a representative tip radius; in practice the user supplies it.
            // Here we return the voltage in units compatible with the field calculation.
            v * KV_TO_V / (field_factor * 1.0e9) * 1.0e9 // V/nm via R=1 nm placeholder
        })
        .collect()
}

/// Compute the mean evaporation field from per-pulse voltage and known radii.
///
/// # Arguments
/// * `v_dc_kv_history` – DC voltage for each ion pulse (kV).
/// * `r_tip_nm_history` – corresponding tip radii (nm).
/// * `field_factor` – k_f.
///
/// # Returns
/// Mean evaporation field in V/nm.
pub fn mean_evaporation_field(
    v_dc_kv_history: &[f64],
    r_tip_nm_history: &[f64],
    field_factor: f64,
) -> f64 {
    let n = v_dc_kv_history.len().min(r_tip_nm_history.len());
    if n == 0 {
        return 0.0;
    }
    let sum: f64 = v_dc_kv_history[..n]
        .iter()
        .zip(r_tip_nm_history[..n].iter())
        .map(|(&v, &r)| {
            if r > 0.0 {
                (v * KV_TO_V) / (field_factor * r)
            } else {
                0.0
            }
        })
        .sum();
    sum / n as f64
}

// ─────────────────────────────────────────────────────────────────────────────
// Composition profiles (proxigram / 1D concentration)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute a 1D composition profile (proxigram) along the Z axis.
///
/// # Arguments
/// * `atoms` – reconstructed and ranged atoms.
/// * `element` – element of interest.
/// * `bin_width_nm` – depth bin width in nm.
///
/// # Returns
/// Vector of `(z_centre_nm, atomic_fraction)` pairs.
pub fn proxigram(
    atoms: &[ReconstructedAtom],
    element: &str,
    bin_width_nm: f64,
) -> Vec<(f64, f64)> {
    if atoms.is_empty() || bin_width_nm <= 0.0 {
        return Vec::new();
    }
    let z_min = atoms.iter().map(|a| a.z_nm).fold(f64::INFINITY, f64::min);
    let z_max = atoms.iter().map(|a| a.z_nm).fold(f64::NEG_INFINITY, f64::max);
    let num_bins = ((z_max - z_min) / bin_width_nm).ceil() as usize + 1;
    let mut total = vec![0u64; num_bins];
    let mut target = vec![0u64; num_bins];
    for atom in atoms {
        let idx = ((atom.z_nm - z_min) / bin_width_nm) as usize;
        let idx = idx.min(num_bins - 1);
        total[idx] += 1;
        if atom.element == element {
            target[idx] += 1;
        }
    }
    total
        .iter()
        .zip(target.iter())
        .enumerate()
        .map(|(i, (&tot, &tgt))| {
            let z_c = z_min + (i as f64 + 0.5) * bin_width_nm;
            let frac = if tot > 0 { tgt as f64 / tot as f64 } else { 0.0 };
            (z_c, frac)
        })
        .collect()
}

/// Compute cumulative composition as function of atom sequence index.
///
/// Useful for verifying analysis integrity (should converge to bulk composition).
pub fn cumulative_composition(atoms: &[ReconstructedAtom], element: &str) -> Vec<f64> {
    let mut running_total = 0u64;
    let mut running_target = 0u64;
    atoms
        .iter()
        .map(|a| {
            running_total += 1;
            if a.element == element {
                running_target += 1;
            }
            running_target as f64 / running_total as f64
        })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Binomial frequency distribution (randomness test)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute binomial probability mass function.
///
/// P(k; n, p) = C(n,k) · p^k · (1-p)^(n-k)
pub fn binomial_pmf(n: u64, k: u64, p: f64) -> f64 {
    if k > n {
        return 0.0;
    }
    let log_p = log_binomial_coeff(n, k) + k as f64 * p.ln() + (n - k) as f64 * (1.0 - p).ln();
    log_p.exp()
}

fn log_binomial_coeff(n: u64, k: u64) -> f64 {
    if k == 0 || k == n {
        return 0.0;
    }
    let k = k.min(n - k); // symmetry
    let mut result = 0.0_f64;
    for i in 0..k {
        result += ((n - i) as f64).ln() - ((i + 1) as f64).ln();
    }
    result
}

/// Compute observed frequency distribution of solute atoms per block of size `block_size`.
///
/// Used to test for chemical ordering or clustering against binomial expectation.
///
/// # Arguments
/// * `atoms` – ranged reconstructed atoms.
/// * `element` – solute of interest.
/// * `block_size` – number of consecutive atoms per block.
///
/// # Returns
/// `(observed, expected)` – both as vectors of `(count, frequency)`.
pub fn frequency_distribution(
    atoms: &[ReconstructedAtom],
    element: &str,
    block_size: usize,
) -> (Vec<(usize, f64)>, Vec<(usize, f64)>) {
    if atoms.is_empty() || block_size == 0 {
        return (Vec::new(), Vec::new());
    }
    let total_target: u64 = atoms.iter().filter(|a| a.element == element).count() as u64;
    let p = total_target as f64 / atoms.len() as f64;
    let mut obs_counts: HashMap<usize, u64> = HashMap::new();
    let num_blocks = atoms.len() / block_size;
    for b in 0..num_blocks {
        let block = &atoms[b * block_size..(b + 1) * block_size];
        let c = block.iter().filter(|a| a.element == element).count();
        *obs_counts.entry(c).or_insert(0) += 1;
    }
    let max_k = *obs_counts.keys().max().unwrap_or(&0);
    let observed: Vec<(usize, f64)> = (0..=max_k)
        .map(|k| {
            let freq = obs_counts.get(&k).copied().unwrap_or(0) as f64 / num_blocks as f64;
            (k, freq)
        })
        .collect();
    let expected: Vec<(usize, f64)> = (0..=max_k)
        .map(|k| {
            let freq = binomial_pmf(block_size as u64, k as u64, p);
            (k, freq)
        })
        .collect();
    (observed, expected)
}

// ─────────────────────────────────────────────────────────────────────────────
// Cluster analysis (maximum separation method)
// ─────────────────────────────────────────────────────────────────────────────

/// Squared Euclidean distance between two atoms (nm²).
#[inline]
fn sq_dist(a: &ReconstructedAtom, b: &ReconstructedAtom) -> f64 {
    let dx = a.x_nm - b.x_nm;
    let dy = a.y_nm - b.y_nm;
    let dz = a.z_nm - b.z_nm;
    dx * dx + dy * dy + dz * dz
}

/// Maximum separation (d_max) clustering for solute atoms.
///
/// Two solute atoms belong to the same cluster if their separation ≤ `d_max_nm`.
/// Uses a simple union-find approach.
///
/// # Arguments
/// * `atoms` – all atoms.
/// * `element` – solute element to cluster.
/// * `d_max_nm` – maximum separation distance (nm).
/// * `n_min` – minimum atoms to qualify as a cluster.
/// * `analysis_volume_nm3` – volume of the analysis region (nm³).
///
/// # Returns
/// Vector of identified clusters.
pub fn maximum_separation_clustering(
    atoms: &[ReconstructedAtom],
    element: &str,
    d_max_nm: f64,
    n_min: usize,
    analysis_volume_nm3: f64,
) -> Vec<AtomCluster> {
    // Collect solute indices
    let solute_idx: Vec<usize> = atoms
        .iter()
        .enumerate()
        .filter(|(_, a)| a.element == element)
        .map(|(i, _)| i)
        .collect();
    let n = solute_idx.len();
    let d2 = d_max_nm * d_max_nm;
    // Union-find
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut Vec<usize>, x: usize) -> usize {
        if parent[x] != x {
            parent[x] = find(parent, parent[x]);
        }
        parent[x]
    }
    for i in 0..n {
        for j in (i + 1)..n {
            if sq_dist(&atoms[solute_idx[i]], &atoms[solute_idx[j]]) <= d2 {
                let pi = find(&mut parent, i);
                let pj = find(&mut parent, j);
                if pi != pj {
                    parent[pj] = pi;
                }
            }
        }
    }
    // Group clusters
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        groups.entry(root).or_default().push(solute_idx[i]);
    }
    let mut clusters = Vec::new();
    for (id, (_, indices)) in groups.iter().enumerate() {
        if indices.len() < n_min {
            continue;
        }
        let cx = indices.iter().map(|&i| atoms[i].x_nm).sum::<f64>() / indices.len() as f64;
        let cy = indices.iter().map(|&i| atoms[i].y_nm).sum::<f64>() / indices.len() as f64;
        let cz = indices.iter().map(|&i| atoms[i].z_nm).sum::<f64>() / indices.len() as f64;
        // Guinier radius: Rg = sqrt(Σr²/N) where r = distance to centroid
        let rg2 = indices
            .iter()
            .map(|&i| {
                let dx = atoms[i].x_nm - cx;
                let dy = atoms[i].y_nm - cy;
                let dz = atoms[i].z_nm - cz;
                dx * dx + dy * dy + dz * dz
            })
            .sum::<f64>()
            / indices.len() as f64;
        let rg = rg2.sqrt();
        let number_density = if analysis_volume_nm3 > 0.0 {
            clusters.len() as f64 / analysis_volume_nm3
        } else {
            0.0
        };
        clusters.push(AtomCluster {
            id,
            atom_indices: indices.clone(),
            centroid_nm: [cx, cy, cz],
            guinier_radius_nm: rg,
            number_density,
        });
    }
    clusters
}

// ─────────────────────────────────────────────────────────────────────────────
// 2D spatial density maps
// ─────────────────────────────────────────────────────────────────────────────

/// Compute a 2D density map (atoms/nm²) projected onto the XY plane.
///
/// # Returns
/// `(map, x_edges, y_edges)` where map is row-major (y × x).
pub fn density_map_xy(
    atoms: &[ReconstructedAtom],
    x_bins: usize,
    y_bins: usize,
) -> (Vec<Vec<f64>>, Vec<f64>, Vec<f64>) {
    if atoms.is_empty() || x_bins == 0 || y_bins == 0 {
        return (Vec::new(), Vec::new(), Vec::new());
    }
    let x_min = atoms.iter().map(|a| a.x_nm).fold(f64::INFINITY, f64::min);
    let x_max = atoms.iter().map(|a| a.x_nm).fold(f64::NEG_INFINITY, f64::max);
    let y_min = atoms.iter().map(|a| a.y_nm).fold(f64::INFINITY, f64::min);
    let y_max = atoms.iter().map(|a| a.y_nm).fold(f64::NEG_INFINITY, f64::max);
    let dx = (x_max - x_min).max(1e-9) / x_bins as f64;
    let dy = (y_max - y_min).max(1e-9) / y_bins as f64;
    let mut map = vec![vec![0.0f64; x_bins]; y_bins];
    for atom in atoms {
        let xi = (((atom.x_nm - x_min) / dx) as usize).min(x_bins - 1);
        let yi = (((atom.y_nm - y_min) / dy) as usize).min(y_bins - 1);
        map[yi][xi] += 1.0;
    }
    let cell_area = dx * dy;
    for row in &mut map {
        for val in row.iter_mut() {
            *val /= cell_area;
        }
    }
    let x_edges: Vec<f64> = (0..=x_bins).map(|i| x_min + i as f64 * dx).collect();
    let y_edges: Vec<f64> = (0..=y_bins).map(|i| y_min + i as f64 * dy).collect();
    (map, x_edges, y_edges)
}

// ─────────────────────────────────────────────────────────────────────────────
// Nearest-neighbour distribution
// ─────────────────────────────────────────────────────────────────────────────

/// Compute nearest-neighbour distances for all atoms of a given element.
///
/// Uses a brute-force O(n²) search. For large datasets (>100k atoms)
/// a k-d tree approach would be preferred.
///
/// # Returns
/// Sorted vector of nearest-neighbour distances (nm).
pub fn nearest_neighbour_distances(atoms: &[ReconstructedAtom], element: &str) -> Vec<f64> {
    let targets: Vec<&ReconstructedAtom> =
        atoms.iter().filter(|a| a.element == element).collect();
    let all: Vec<&ReconstructedAtom> = atoms.iter().collect();
    let mut dists = Vec::with_capacity(targets.len());
    for t in &targets {
        let mut min_d2 = f64::INFINITY;
        for a in &all {
            // Skip self
            if std::ptr::eq(*t as *const _, *a as *const _) {
                continue;
            }
            let dx = t.x_nm - a.x_nm;
            let dy = t.y_nm - a.y_nm;
            let dz = t.z_nm - a.z_nm;
            let d2 = dx * dx + dy * dy + dz * dz;
            if d2 < min_d2 {
                min_d2 = d2;
            }
        }
        dists.push(min_d2.sqrt());
    }
    dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
    dists
}

// ─────────────────────────────────────────────────────────────────────────────
// Detection efficiency correction
// ─────────────────────────────────────────────────────────────────────────────

/// Apply detection efficiency correction to measured atom counts.
///
/// True count estimate = measured count / η
///
/// # Arguments
/// * `measured_count` – number of detected atoms.
/// * `efficiency` – detector efficiency (0–1).
///
/// # Returns
/// Estimated true number of atoms.
pub fn efficiency_corrected_count(measured_count: u64, efficiency: f64) -> f64 {
    if efficiency <= 0.0 {
        return f64::INFINITY;
    }
    measured_count as f64 / efficiency
}

/// Compute detection efficiency correction factor for composition.
///
/// If two elements have different ionisation probability (e.g., due to
/// different charge states being cut by the mass range), a relative efficiency
/// factor can be applied.
///
/// # Returns
/// Corrected atomic fraction of `element`.
pub fn efficiency_corrected_composition(
    counts: &HashMap<String, u64>,
    efficiencies: &HashMap<String, f64>,
) -> HashMap<String, f64> {
    let corrected: HashMap<String, f64> = counts
        .iter()
        .map(|(el, &cnt)| {
            let eta = efficiencies.get(el).copied().unwrap_or(1.0);
            (el.clone(), cnt as f64 / eta.max(1e-10))
        })
        .collect();
    let total: f64 = corrected.values().sum();
    if total <= 0.0 {
        return HashMap::new();
    }
    corrected
        .into_iter()
        .map(|(el, c)| (el, c / total))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Bowl-cap (Bowler-cap) reconstruction model
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the z-depth increment using the bowl-cap (Bowler-cap) model.
///
/// In the bowl-cap model, the apex surface is modelled as a spherical cap.
/// The depth increment per detected atom accounts for the spherical geometry:
///
/// Δz_bowl = Ω / (2π · R² · (1 - cos θ_max) · η)
///
/// where θ_max is the maximum half-angle of the detector acceptance cone.
///
/// # Arguments
/// * `r_tip_nm` – tip radius (nm).
/// * `theta_max_rad` – half acceptance angle (radians).
/// * `atomic_volume_nm3` – atomic volume (nm³).
/// * `efficiency` – detector efficiency.
///
/// # Returns
/// Depth increment per atom (nm).
pub fn bowler_cap_z_increment(
    r_tip_nm: f64,
    theta_max_rad: f64,
    atomic_volume_nm3: f64,
    efficiency: f64,
) -> f64 {
    let solid_angle_factor = 1.0 - theta_max_rad.cos();
    let denom = 2.0 * std::f64::consts::PI * r_tip_nm * r_tip_nm * solid_angle_factor * efficiency;
    if denom == 0.0 {
        return 0.0;
    }
    atomic_volume_nm3 / denom
}

// ─────────────────────────────────────────────────────────────────────────────
// Material presets
// ─────────────────────────────────────────────────────────────────────────────

/// Return built-in material presets for common APT alloy systems.
pub fn builtin_material_presets() -> Vec<MaterialPreset> {
    vec![
        MaterialPreset {
            name: "Steel (Fe-C-Mn-Si)".into(),
            composition: vec![
                ("Fe".into(), 0.9720),
                ("C".into(),  0.0080),
                ("Mn".into(), 0.0120),
                ("Si".into(), 0.0080),
            ],
            evaporation_field: 33.0,
        },
        MaterialPreset {
            name: "Aluminium alloy (Al-Cu-Mg-Zn)".into(),
            composition: vec![
                ("Al".into(), 0.9100),
                ("Cu".into(), 0.0175),
                ("Mg".into(), 0.0250),
                ("Zn".into(), 0.0475),
            ],
            evaporation_field: 19.0,
        },
        MaterialPreset {
            name: "Nickel superalloy (Ni-Al-Cr-Co)".into(),
            composition: vec![
                ("Ni".into(), 0.5800),
                ("Al".into(), 0.1200),
                ("Cr".into(), 0.1900),
                ("Co".into(), 0.1100),
            ],
            evaporation_field: 35.0,
        },
    ]
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper statistics
// ─────────────────────────────────────────────────────────────────────────────

/// Compute mean of a slice.
pub fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

/// Compute standard deviation (population) of a slice.
pub fn std_dev(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let m = mean(data);
    let variance = data.iter().map(|&x| (x - m) * (x - m)).sum::<f64>() / data.len() as f64;
    variance.sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── TOF / m/n conversion ──────────────────────────────────────────────

    #[test]
    fn test_tof_to_mn_basic() {
        // Fe²⁺ at m/n = 27.97 Da/e, V=10 kV, L=90 mm → compute TOF then round-trip.
        let mn_true = 27.97;
        let v = 10.0; // kV
        let l = 90.0; // mm
        let tof = mass_to_charge_to_tof(mn_true, v, 0.0, l);
        assert!(tof > 0.0, "TOF must be positive");
        let mn_calc = tof_to_mass_to_charge(tof, v, 0.0, l);
        assert!((mn_calc - mn_true).abs() < 1e-6, "Round-trip m/n error: {}", mn_calc);
    }

    #[test]
    fn test_tof_to_mn_zero_voltage() {
        let mn = tof_to_mass_to_charge(100.0, 0.0, 0.0, 90.0);
        assert_eq!(mn, 0.0);
    }

    #[test]
    fn test_tof_to_mn_zero_flight_length() {
        let mn = tof_to_mass_to_charge(100.0, 10.0, 0.0, 0.0);
        assert_eq!(mn, 0.0);
    }

    #[test]
    fn test_mn_to_tof_zero_voltage() {
        let tof = mass_to_charge_to_tof(28.0, 0.0, 0.0, 90.0);
        assert_eq!(tof, 0.0);
    }

    #[test]
    fn test_mn_higher_voltage_shorter_tof() {
        let mn = 28.0;
        let l = 90.0;
        let tof_10 = mass_to_charge_to_tof(mn, 10.0, 0.0, l);
        let tof_20 = mass_to_charge_to_tof(mn, 20.0, 0.0, l);
        assert!(tof_10 > tof_20, "Higher V → shorter TOF");
    }

    #[test]
    fn test_pulse_voltage_adds_to_dc() {
        let mn = 28.0;
        let l = 90.0;
        let tof_no_pulse = mass_to_charge_to_tof(mn, 10.0, 0.0, l);
        let tof_pulse = mass_to_charge_to_tof(mn, 8.0, 2.0, l);
        assert!((tof_no_pulse - tof_pulse).abs() < 1e-6);
    }

    // ── Mass spectrum ─────────────────────────────────────────────────────

    #[test]
    fn test_mass_spectrum_empty() {
        let spec = compute_mass_spectrum(&[], 0.0, 100.0, 100);
        // Bins exist but all counts are zero
        let total: u64 = spec.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 0);
    }

    #[test]
    fn test_mass_spectrum_single_bin() {
        let mn = vec![28.0];
        let spec = compute_mass_spectrum(&mn, 0.0, 100.0, 100);
        let total: u64 = spec.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 1);
    }

    #[test]
    fn test_mass_spectrum_counts() {
        let mn = vec![10.0, 10.5, 10.3, 50.0];
        let spec = compute_mass_spectrum(&mn, 0.0, 100.0, 100);
        let total: u64 = spec.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 4);
    }

    #[test]
    fn test_mass_spectrum_bin_centres() {
        let spec = compute_mass_spectrum(&[25.0], 0.0, 100.0, 10);
        // Bin width = 10; 25 falls in bin 2 (centre at 25.0)
        let found = spec.iter().any(|&(c, n)| n > 0 && (c - 25.0).abs() < 5.0);
        assert!(found);
    }

    #[test]
    fn test_background_estimation() {
        let spec: Vec<(f64, u64)> = (0..100).map(|i| (i as f64, i as u64)).collect();
        let bg = estimate_background(&spec, 0.1);
        assert!(bg < 10.0);
    }

    #[test]
    fn test_background_empty() {
        let bg = estimate_background(&[], 0.1);
        assert_eq!(bg, 0.0);
    }

    #[test]
    fn test_peak_snr_positive() {
        let spec: Vec<(f64, u64)> = (0..100)
            .map(|i| (i as f64, if i == 28 { 1000 } else { 5 }))
            .collect();
        let snr = peak_snr(&spec, 27.5, 28.5, 5.0);
        assert!(snr > 0.0, "SNR should be positive for a real peak");
    }

    // ── Isotope library and ranging ───────────────────────────────────────

    #[test]
    fn test_isotope_library_not_empty() {
        let lib = builtin_isotope_library();
        assert!(!lib.is_empty());
    }

    #[test]
    fn test_isotope_library_contains_fe() {
        let lib = builtin_isotope_library();
        assert!(lib.iter().any(|i| i.element == "Fe"));
    }

    #[test]
    fn test_generate_peak_ranges_charge2() {
        let lib = builtin_isotope_library();
        let ranges = generate_peak_ranges(&lib, 2, 0.1);
        assert!(!ranges.is_empty());
        for r in &ranges {
            assert_eq!(r.charge_state, 2);
            assert!(r.lower_da < r.upper_da);
        }
    }

    #[test]
    fn test_range_ions_counts() {
        let lib = builtin_isotope_library();
        let ranges = generate_peak_ranges(&lib, 2, 0.3);
        // Fe56²⁺ is at 55.935/2 ≈ 27.97
        let mn_values = vec![27.97, 27.97, 6.0, 999.0];
        let counts = range_ions(&mn_values, &ranges);
        assert!(counts.get("Fe").copied().unwrap_or(0) >= 2);
    }

    // ── Kingham curves ────────────────────────────────────────────────────

    #[test]
    fn test_kingham_ratio_positive() {
        let r = kingham_charge_ratio(33.0, "Fe");
        assert!(r > 0.0);
    }

    #[test]
    fn test_kingham_ratio_increases_with_field() {
        let r_low = kingham_charge_ratio(20.0, "Fe");
        let r_high = kingham_charge_ratio(40.0, "Fe");
        assert!(r_high > r_low);
    }

    #[test]
    fn test_kingham_probability_sums_to_one() {
        let field = 33.0;
        let el = "Fe";
        let sum: f64 = (1u8..=4).map(|q| kingham_charge_probability(field, el, q)).sum();
        assert!((sum - 1.0).abs() < 1e-9, "Probabilities must sum to 1: {}", sum);
    }

    #[test]
    fn test_kingham_unknown_element() {
        let r = kingham_charge_ratio(30.0, "Xx");
        assert!(r > 0.0);
    }

    // ── Spatial reconstruction ────────────────────────────────────────────

    #[test]
    fn test_tip_radius_from_voltage() {
        let params = ReconstructionParams::default();
        // At default F_ev = 33 V/nm, k_f=4, V=10 kV:
        // R = 10000 V / (4 * 33e9 V/m) * 1e9 nm/m ≈ 75.8 nm
        let r = tip_radius_from_voltage(10.0, &params);
        assert!(r > 0.0 && r < 1000.0, "Tip radius out of expected range: {}", r);
    }

    #[test]
    fn test_tip_radius_scales_with_voltage() {
        let params = ReconstructionParams::default();
        let r1 = tip_radius_from_voltage(5.0, &params);
        let r2 = tip_radius_from_voltage(10.0, &params);
        assert!((r2 - 2.0 * r1).abs() < 1e-6, "Radius should double with doubled voltage");
    }

    #[test]
    fn test_reconstruct_position_origin_for_zero_det() {
        let params = ReconstructionParams::default();
        let hit = IonHit {
            tof_ns: 100.0,
            v_dc_kv: 10.0,
            v_pulse_kv: 0.0,
            x_det: 0.0,
            y_det: 0.0,
            pulse_index: 0,
        };
        let (x, y, _z) = reconstruct_position(&hit, &params, 0.0);
        assert_eq!(x, 0.0);
        assert_eq!(y, 0.0);
    }

    #[test]
    fn test_reconstruct_dataset_length() {
        let params = ReconstructionParams::default();
        let hits: Vec<IonHit> = (0..10)
            .map(|i| IonHit {
                tof_ns: 100.0 + i as f64,
                v_dc_kv: 10.0,
                v_pulse_kv: 0.0,
                x_det: i as f64,
                y_det: 0.0,
                pulse_index: i as u64,
            })
            .collect();
        let atoms = reconstruct_dataset(&hits, &params);
        assert_eq!(atoms.len(), 10);
    }

    #[test]
    fn test_reconstruct_z_increases_monotonically() {
        let params = ReconstructionParams::default();
        let hits: Vec<IonHit> = (0..5)
            .map(|i| IonHit {
                tof_ns: 100.0,
                v_dc_kv: 10.0,
                v_pulse_kv: 0.0,
                x_det: 0.0,
                y_det: 0.0,
                pulse_index: i as u64,
            })
            .collect();
        let atoms = reconstruct_dataset(&hits, &params);
        for w in atoms.windows(2) {
            assert!(w[1].z_nm > w[0].z_nm, "Z must increase monotonically");
        }
    }

    // ── Voltage curve ─────────────────────────────────────────────────────

    #[test]
    fn test_mean_evaporation_field() {
        let v = vec![10.0, 11.0, 12.0]; // kV
        let r = vec![75.0, 82.5, 90.0]; // nm
        let f_mean = mean_evaporation_field(&v, &r, 4.0);
        assert!(f_mean > 0.0);
    }

    #[test]
    fn test_mean_evaporation_field_empty() {
        let f = mean_evaporation_field(&[], &[], 4.0);
        assert_eq!(f, 0.0);
    }

    // ── Proxigram ─────────────────────────────────────────────────────────

    #[test]
    fn test_proxigram_empty() {
        let atoms: Vec<ReconstructedAtom> = Vec::new();
        let pg = proxigram(&atoms, "Fe", 1.0);
        assert!(pg.is_empty());
    }

    #[test]
    fn test_proxigram_single_element() {
        let atoms: Vec<ReconstructedAtom> = (0..10)
            .map(|i| ReconstructedAtom {
                x_nm: 0.0,
                y_nm: 0.0,
                z_nm: i as f64,
                mass_to_charge: 28.0,
                element: "Fe".into(),
                charge_state: 2,
            })
            .collect();
        let pg = proxigram(&atoms, "Fe", 2.0);
        for (_, frac) in &pg {
            // Bins with atoms should have frac=1.0; empty bins have frac=0.0
            assert!(*frac == 0.0 || (*frac - 1.0).abs() < 1e-9, "Fe fraction must be 0 or 1, got {frac}");
        }
    }

    #[test]
    fn test_cumulative_composition_converges() {
        let mut atoms: Vec<ReconstructedAtom> = Vec::new();
        for i in 0..100 {
            atoms.push(ReconstructedAtom {
                x_nm: 0.0, y_nm: 0.0, z_nm: i as f64,
                mass_to_charge: 28.0,
                element: if i % 5 == 0 { "C".into() } else { "Fe".into() },
                charge_state: 2,
            });
        }
        let cc = cumulative_composition(&atoms, "C");
        let last = *cc.last().unwrap();
        assert!((last - 0.2).abs() < 0.01, "20% C expected, got {}", last);
    }

    // ── Binomial distribution ─────────────────────────────────────────────

    #[test]
    fn test_binomial_pmf_sum_to_one() {
        let n = 10;
        let p = 0.3;
        let total: f64 = (0..=n).map(|k| binomial_pmf(n as u64, k, p)).sum();
        assert!((total - 1.0).abs() < 1e-9, "PMF must sum to 1: {}", total);
    }

    #[test]
    fn test_binomial_pmf_k_greater_n() {
        let pmf = binomial_pmf(5, 6, 0.5);
        assert_eq!(pmf, 0.0);
    }

    #[test]
    fn test_binomial_pmf_k0() {
        // P(0; n=1, p=0.3) = 0.7
        let pmf = binomial_pmf(1, 0, 0.3);
        assert!((pmf - 0.7).abs() < 1e-9, "Got {}", pmf);
    }

    // ── Cluster analysis ──────────────────────────────────────────────────

    #[test]
    fn test_clustering_finds_cluster() {
        let mut atoms = Vec::new();
        // Tight cluster of 5 C atoms
        for i in 0..5 {
            atoms.push(ReconstructedAtom {
                x_nm: i as f64 * 0.1,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 6.0,
                element: "C".into(),
                charge_state: 1,
            });
        }
        // Isolated Fe atoms
        for i in 0..20 {
            atoms.push(ReconstructedAtom {
                x_nm: 100.0 + i as f64,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 28.0,
                element: "Fe".into(),
                charge_state: 2,
            });
        }
        let clusters = maximum_separation_clustering(&atoms, "C", 0.5, 3, 1000.0);
        assert_eq!(clusters.len(), 1, "Should find exactly 1 C cluster");
        assert_eq!(clusters[0].atom_indices.len(), 5);
    }

    #[test]
    fn test_clustering_min_size_filter() {
        let mut atoms = Vec::new();
        for i in 0..2 {
            atoms.push(ReconstructedAtom {
                x_nm: i as f64 * 0.1,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 6.0,
                element: "C".into(),
                charge_state: 1,
            });
        }
        // n_min = 5 → no cluster
        let clusters = maximum_separation_clustering(&atoms, "C", 1.0, 5, 1000.0);
        assert!(clusters.is_empty());
    }

    #[test]
    fn test_guinier_radius_nonnegative() {
        let mut atoms = Vec::new();
        for i in 0..6 {
            atoms.push(ReconstructedAtom {
                x_nm: i as f64,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 6.0,
                element: "C".into(),
                charge_state: 1,
            });
        }
        let clusters = maximum_separation_clustering(&atoms, "C", 2.0, 3, 1000.0);
        assert!(!clusters.is_empty());
        assert!(clusters[0].guinier_radius_nm >= 0.0);
    }

    // ── Density map ───────────────────────────────────────────────────────

    #[test]
    fn test_density_map_shape() {
        let atoms: Vec<ReconstructedAtom> = (0..9)
            .map(|i| ReconstructedAtom {
                x_nm: (i % 3) as f64,
                y_nm: (i / 3) as f64,
                z_nm: 0.0,
                mass_to_charge: 28.0,
                element: "Fe".into(),
                charge_state: 2,
            })
            .collect();
        let (map, xe, ye) = density_map_xy(&atoms, 3, 3);
        assert_eq!(map.len(), 3);
        assert_eq!(map[0].len(), 3);
        assert_eq!(xe.len(), 4);
        assert_eq!(ye.len(), 4);
    }

    #[test]
    fn test_density_map_total_atoms() {
        let atoms: Vec<ReconstructedAtom> = (0..10)
            .map(|i| ReconstructedAtom {
                x_nm: i as f64,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 28.0,
                element: "Fe".into(),
                charge_state: 2,
            })
            .collect();
        let (map, xe, ye) = density_map_xy(&atoms, 5, 1);
        let dx = xe[1] - xe[0];
        let dy = ye[1] - ye[0];
        let total_atoms: f64 = map.iter().flatten().sum::<f64>() * dx * dy;
        assert!((total_atoms - 10.0).abs() < 1e-6);
    }

    // ── Nearest-neighbour ─────────────────────────────────────────────────

    #[test]
    fn test_nn_distances_ordered() {
        let atoms: Vec<ReconstructedAtom> = (0..5)
            .map(|i| ReconstructedAtom {
                x_nm: i as f64,
                y_nm: 0.0,
                z_nm: 0.0,
                mass_to_charge: 28.0,
                element: "Fe".into(),
                charge_state: 2,
            })
            .collect();
        let dists = nearest_neighbour_distances(&atoms, "Fe");
        assert_eq!(dists.len(), 5);
        for d in &dists {
            assert!(*d >= 0.0);
        }
    }

    #[test]
    fn test_nn_distances_empty_element() {
        let atoms: Vec<ReconstructedAtom> = vec![ReconstructedAtom {
            x_nm: 0.0, y_nm: 0.0, z_nm: 0.0,
            mass_to_charge: 28.0, element: "Fe".into(), charge_state: 2,
        }];
        let dists = nearest_neighbour_distances(&atoms, "C");
        assert!(dists.is_empty());
    }

    // ── Detection efficiency ──────────────────────────────────────────────

    #[test]
    fn test_efficiency_corrected_count() {
        let corrected = efficiency_corrected_count(37, 0.37);
        assert!((corrected - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_efficiency_corrected_composition_sums_to_one() {
        let mut counts = HashMap::new();
        counts.insert("Fe".to_string(), 900u64);
        counts.insert("C".to_string(),  100u64);
        let mut effs = HashMap::new();
        effs.insert("Fe".to_string(), 0.5_f64);
        effs.insert("C".to_string(),  0.5_f64);
        let comp = efficiency_corrected_composition(&counts, &effs);
        let total: f64 = comp.values().sum();
        assert!((total - 1.0).abs() < 1e-9);
    }

    // ── Bowler-cap ────────────────────────────────────────────────────────

    #[test]
    fn test_bowler_cap_increment_positive() {
        let dz = bowler_cap_z_increment(50.0, 0.5, 0.0118, 0.5);
        assert!(dz > 0.0, "z-increment must be positive");
    }

    #[test]
    fn test_bowler_cap_zero_efficiency() {
        let dz = bowler_cap_z_increment(50.0, 0.5, 0.0118, 0.0);
        assert_eq!(dz, 0.0);
    }

    // ── Material presets ──────────────────────────────────────────────────

    #[test]
    fn test_material_presets_exist() {
        let presets = builtin_material_presets();
        assert_eq!(presets.len(), 3);
    }

    #[test]
    fn test_material_preset_compositions_sum_to_one() {
        for preset in builtin_material_presets() {
            let total: f64 = preset.composition.iter().map(|(_, f)| f).sum();
            assert!((total - 1.0).abs() < 0.01, "{}: sum = {}", preset.name, total);
        }
    }

    #[test]
    fn test_material_preset_fields_positive() {
        for preset in builtin_material_presets() {
            assert!(preset.evaporation_field > 0.0);
        }
    }

    // ── Statistics helpers ────────────────────────────────────────────────

    #[test]
    fn test_mean_empty() {
        assert_eq!(mean(&[]), 0.0);
    }

    #[test]
    fn test_mean_values() {
        assert!((mean(&[1.0, 2.0, 3.0]) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_std_dev_constant() {
        assert_eq!(std_dev(&[5.0, 5.0, 5.0]), 0.0);
    }

    #[test]
    fn test_std_dev_known() {
        let d = std_dev(&[2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]);
        assert!((d - 2.0).abs() < 1e-9, "Expected 2.0, got {}", d);
    }
}
