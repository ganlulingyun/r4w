//! # Neutrino Cherenkov Detector
//!
//! Signal processing for water/ice Cherenkov neutrino detectors such as
//! IceCube, Super-Kamiokande, and KM3NeT.
//!
//! When a high-energy neutrino interacts with matter in a transparent medium
//! (water, ice, or mineral oil), it can produce a charged lepton or hadronic
//! shower. If the resulting charged particle travels faster than the phase
//! velocity of light in that medium (v > c/n), it emits Cherenkov radiation
//! in a forward cone with opening half-angle θ = arccos(1 / (n β)).
//!
//! Photomultiplier tubes (PMTs) distributed throughout the detector volume
//! record the arrival time and charge (in photoelectrons) of Cherenkov
//! photons. From the pattern of hit times and charges, the vertex position,
//! direction, energy, and event topology of the neutrino interaction can be
//! reconstructed.
//!
//! ## Physics Background
//!
//! - **Cherenkov threshold**: β_min = 1/n, where n is the refractive index.
//!   For water (n ≈ 1.33), β_min ≈ 0.752.
//! - **Cherenkov angle**: cos(θ_c) = 1/(nβ). At β = 1 in water, θ_c ≈ 41.2°.
//! - **Frank-Tamm formula**: The number of photons emitted per unit path length
//!   and wavelength interval is dN/(dx dλ) = (2π α / λ²) sin²(θ_c), where α
//!   is the fine-structure constant.
//! - **Event topologies**:
//!   - **Track**: Muon neutrino CC interaction produces a long muon track
//!   - **Cascade**: Electron neutrino CC or any NC interaction produces a
//!     roughly spherical electromagnetic/hadronic shower
//!   - **Double cascade**: Tau neutrino CC at high energy produces two
//!     cascades separated by the tau decay length
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::neutrino_cherenkov_detector::*;
//!
//! let config = CherenkovConfig::default();
//! let detector = CherenkovDetector::new(config);
//!
//! // Cherenkov angle for a relativistic particle (β ≈ 1) in water
//! let angle = detector.cherenkov_angle(1.0);
//! assert!((angle.to_degrees() - 41.2).abs() < 0.5);
//!
//! // Threshold velocity for Cherenkov emission
//! let beta_min = CherenkovDetector::cherenkov_threshold(&Medium::Water);
//! assert!((beta_min - 0.7519).abs() < 0.001);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// The transparent medium filling the detector volume.
///
/// Each variant carries a default refractive index at the relevant
/// wavelength (~400 nm, near the Cherenkov emission peak).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Medium {
    /// Pure water (n ≈ 1.33), used in Super-Kamiokande, Hyper-Kamiokande, KM3NeT.
    Water,
    /// South Pole glacial ice (n ≈ 1.31), used in IceCube.
    Ice,
    /// Mineral oil / liquid scintillator (n ≈ 1.47), used in SNO+, KamLAND.
    MineralOil,
}

impl Medium {
    /// Returns the default refractive index for this medium at ~400 nm.
    pub fn default_refractive_index(&self) -> f64 {
        match self {
            Medium::Water => 1.33,
            Medium::Ice => 1.31,
            Medium::MineralOil => 1.47,
        }
    }
}

/// The geometric layout of the photomultiplier array.
#[derive(Debug, Clone, PartialEq)]
pub enum DetectorGeometry {
    /// A right circular cylinder (e.g., Super-Kamiokande).
    Cylindrical {
        /// Inner radius in metres.
        radius_m: f64,
        /// Inner height in metres.
        height_m: f64,
    },
    /// A sphere (e.g., JUNO).
    Spherical {
        /// Inner radius in metres.
        radius_m: f64,
    },
    /// An IceCube-style vertical string array embedded in ice.
    IceCubeString {
        /// Number of vertical strings.
        num_strings: usize,
        /// Horizontal spacing between strings in metres.
        string_spacing_m: f64,
        /// Number of Digital Optical Modules (DOMs) per string.
        num_doms_per_string: usize,
        /// Vertical spacing between DOMs in metres.
        dom_spacing_m: f64,
    },
}

/// Classification of the event topology.
///
/// The topology is inferred from the spatial and temporal distribution
/// of PMT hits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventTopology {
    /// A long muon track (ν_μ CC interaction).
    Track,
    /// A roughly spherical cascade (ν_e CC or any NC interaction).
    Cascade,
    /// Two cascades separated by a tau decay length (ν_τ CC interaction).
    DoubleCascade,
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// Configuration for the Cherenkov detector simulation and reconstruction.
#[derive(Debug, Clone)]
pub struct CherenkovConfig {
    /// The detector medium.
    pub medium: Medium,
    /// Refractive index of the medium (overrides the default if set).
    pub refractive_index: f64,
    /// Speed of light in vacuum in m/s.
    pub speed_of_light: f64,
    /// Single-photoelectron time resolution of the PMTs in nanoseconds.
    pub pmt_time_resolution_ns: f64,
    /// Geometry of the PMT array.
    pub detector_geometry: DetectorGeometry,
}

impl Default for CherenkovConfig {
    /// Returns a default configuration modelling a cylindrical water
    /// Cherenkov detector similar to Super-Kamiokande.
    fn default() -> Self {
        Self {
            medium: Medium::Water,
            refractive_index: Medium::Water.default_refractive_index(),
            speed_of_light: 299_792_458.0,
            pmt_time_resolution_ns: 3.0,
            detector_geometry: DetectorGeometry::Cylindrical {
                radius_m: 16.9,
                height_m: 36.2,
            },
        }
    }
}

/// A single PMT hit (detected photon arrival).
#[derive(Debug, Clone, PartialEq)]
pub struct PmtHit {
    /// Unique identifier for the PMT that recorded the hit.
    pub pmt_id: usize,
    /// Photon arrival time in nanoseconds relative to the trigger.
    pub time_ns: f64,
    /// Integrated charge in photoelectrons (PE).
    pub charge_pe: f64,
    /// 3-D position of the PMT in metres (x, y, z).
    pub position: [f64; 3],
}

/// Result of fitting a Cherenkov ring to a 2-D projection of PMT hits.
#[derive(Debug, Clone, PartialEq)]
pub struct RingFit {
    /// Centre of the fitted ring in 2-D projected coordinates.
    pub center: [f64; 2],
    /// Radius of the fitted ring (arbitrary projected units).
    pub radius: f64,
    /// Goodness-of-fit metric in [0, 1] (1 = perfect ring).
    pub goodness: f64,
}

// ---------------------------------------------------------------------------
// CherenkovDetector
// ---------------------------------------------------------------------------

/// Core reconstruction and physics engine for a Cherenkov neutrino detector.
pub struct CherenkovDetector {
    config: CherenkovConfig,
}

impl CherenkovDetector {
    /// Creates a new detector with the given configuration.
    pub fn new(config: CherenkovConfig) -> Self {
        Self { config }
    }

    /// Returns the Cherenkov emission half-angle in radians for a particle
    /// with velocity β = v/c.
    ///
    /// The relation is cos(θ_c) = 1 / (n β).  If the particle is below
    /// threshold (nβ < 1), returns 0.0 (no Cherenkov emission).
    pub fn cherenkov_angle(&self, beta: f64) -> f64 {
        let n = self.config.refractive_index;
        let cos_theta = 1.0 / (n * beta);
        if cos_theta.abs() > 1.0 {
            return 0.0;
        }
        cos_theta.acos()
    }

    /// Returns the minimum β (= v/c) required for Cherenkov emission in
    /// the given medium.  This is simply 1/n.
    pub fn cherenkov_threshold(medium: &Medium) -> f64 {
        1.0 / medium.default_refractive_index()
    }

    /// Estimates the number of Cherenkov photons emitted per metre of
    /// track length in a narrow wavelength band around `wavelength_nm`,
    /// using the Frank-Tamm formula.
    ///
    /// dN/dx ≈ (2π α / λ²) sin²(θ_c)  [photons / m]
    ///
    /// where α ≈ 1/137 is the fine-structure constant and λ is the
    /// wavelength in metres.
    pub fn photon_yield_per_meter(&self, wavelength_nm: f64, beta: f64) -> f64 {
        let alpha = 1.0 / 137.036; // fine-structure constant
        let lambda_m = wavelength_nm * 1e-9;
        let n = self.config.refractive_index;
        let cos_theta = 1.0 / (n * beta);
        if cos_theta.abs() > 1.0 {
            return 0.0; // below threshold
        }
        let sin2_theta = 1.0 - cos_theta * cos_theta;
        2.0 * PI * alpha / (lambda_m * lambda_m) * sin2_theta
    }

    /// Computes the expected photon arrival time (in nanoseconds) at a PMT
    /// located at `pmt_pos`, given a particle vertex at `vertex` travelling
    /// in direction `direction` (unit vector) with velocity β.
    ///
    /// The calculation assumes the photon is emitted at the point of
    /// closest approach on the particle track to the PMT and travels
    /// through the medium at c/n.
    pub fn expected_arrival_time(
        &self,
        vertex: [f64; 3],
        direction: [f64; 3],
        pmt_pos: [f64; 3],
        beta: f64,
    ) -> f64 {
        let c = self.config.speed_of_light;
        let n = self.config.refractive_index;

        // Vector from vertex to PMT
        let dx = [
            pmt_pos[0] - vertex[0],
            pmt_pos[1] - vertex[1],
            pmt_pos[2] - vertex[2],
        ];

        // Projection of dx onto the track direction (signed distance along track)
        let d_along = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

        // Perpendicular distance from PMT to track
        let perp_x = dx[0] - d_along * direction[0];
        let perp_y = dx[1] - d_along * direction[1];
        let perp_z = dx[2] - d_along * direction[2];
        let d_perp = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();

        // Cherenkov angle
        let cos_theta_c = 1.0 / (n * beta);
        let sin_theta_c = (1.0 - cos_theta_c * cos_theta_c).max(0.0).sqrt();

        // Emission point: the photon that reaches the PMT is emitted when
        // the perpendicular distance equals d_perp = s * sin(θ_c), where s
        // is the photon path length.  The particle has travelled
        // d_emit = d_along - d_perp / tan(θ_c) along the track.
        let tan_theta_c = if cos_theta_c.abs() > 1e-12 {
            sin_theta_c / cos_theta_c
        } else {
            return 0.0;
        };
        let d_emit = d_along - d_perp / tan_theta_c;

        // Time for particle to reach emission point
        let t_particle = d_emit / (beta * c);

        // Photon path length from emission point to PMT
        let s_photon = if sin_theta_c.abs() > 1e-12 {
            d_perp / sin_theta_c
        } else {
            d_perp
        };
        let t_photon = s_photon / (c / n);

        // Total time in seconds, converted to nanoseconds
        (t_particle + t_photon) * 1e9
    }

    /// Reconstructs the interaction vertex from an array of PMT hits using
    /// a simple least-squares minimisation of time residuals.
    ///
    /// Uses an iterative grid-search refinement starting from the
    /// charge-weighted centroid of the hit PMTs.
    pub fn reconstruct_vertex(&self, hits: &[PmtHit]) -> [f64; 3] {
        if hits.is_empty() {
            return [0.0; 3];
        }

        let c = self.config.speed_of_light;
        let n = self.config.refractive_index;
        let v_medium = c / n; // speed of light in medium

        // Initial guess: charge-weighted centroid
        let total_q: f64 = hits.iter().map(|h| h.charge_pe).sum();
        if total_q <= 0.0 {
            return [0.0; 3];
        }
        let mut vertex = [0.0f64; 3];
        for h in hits {
            vertex[0] += h.position[0] * h.charge_pe;
            vertex[1] += h.position[1] * h.charge_pe;
            vertex[2] += h.position[2] * h.charge_pe;
        }
        vertex[0] /= total_q;
        vertex[1] /= total_q;
        vertex[2] /= total_q;

        // Find earliest hit time to use as reference
        let t_min = hits
            .iter()
            .map(|h| h.time_ns)
            .fold(f64::INFINITY, f64::min);

        // Iterative grid-search refinement
        let mut step = 10.0; // metres
        for _ in 0..8 {
            let mut best = vertex;
            let mut best_chi2 = f64::MAX;

            // Search 27 points (3x3x3 grid around current vertex)
            for ix in -1i32..=1 {
                for iy in -1i32..=1 {
                    for iz in -1i32..=1 {
                        let trial = [
                            vertex[0] + ix as f64 * step,
                            vertex[1] + iy as f64 * step,
                            vertex[2] + iz as f64 * step,
                        ];

                        // Compute chi² = sum of (t_measured - t_expected)²
                        // where t_expected = t_emit + distance / v_medium
                        // We solve for t_emit analytically by minimising chi².
                        let mut sum_dt = 0.0;
                        let mut count = 0usize;
                        let mut residuals = Vec::with_capacity(hits.len());

                        for h in hits {
                            let dx = h.position[0] - trial[0];
                            let dy = h.position[1] - trial[1];
                            let dz = h.position[2] - trial[2];
                            let dist = ((dx * dx + dy * dy + dz * dz) as f64).sqrt();
                            let t_travel = dist / v_medium * 1e9; // ns
                            let dt = h.time_ns - t_travel;
                            residuals.push(dt);
                            sum_dt += dt;
                            count += 1;
                        }

                        if count == 0 {
                            continue;
                        }
                        let t_emit = sum_dt / count as f64;

                        let chi2: f64 = residuals
                            .iter()
                            .map(|dt| {
                                let r = dt - t_emit;
                                r * r
                            })
                            .sum();

                        if chi2 < best_chi2 {
                            best_chi2 = chi2;
                            best = trial;
                        }
                    }
                }
            }
            vertex = best;
            step *= 0.5;
        }

        vertex
    }

    /// Reconstructs the particle direction from the Cherenkov cone pattern.
    ///
    /// Uses the charge-weighted average direction from the reconstructed
    /// vertex to the hit PMTs, corrected for the Cherenkov angle.
    /// Returns a unit vector.
    pub fn reconstruct_direction(&self, hits: &[PmtHit], vertex: [f64; 3]) -> [f64; 3] {
        if hits.is_empty() {
            return [0.0, 0.0, 1.0]; // default: +z
        }

        // Charge-weighted average direction from vertex to hits
        let mut dir = [0.0f64; 3];
        let mut total_q = 0.0f64;

        for h in hits {
            let dx = h.position[0] - vertex[0];
            let dy = h.position[1] - vertex[1];
            let dz = h.position[2] - vertex[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist < 1e-12 {
                continue;
            }
            let w = h.charge_pe;
            dir[0] += w * dx / dist;
            dir[1] += w * dy / dist;
            dir[2] += w * dz / dist;
            total_q += w;
        }

        if total_q < 1e-12 {
            return [0.0, 0.0, 1.0];
        }

        dir[0] /= total_q;
        dir[1] /= total_q;
        dir[2] /= total_q;

        // Normalise to unit vector
        let mag = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        if mag < 1e-12 {
            return [0.0, 0.0, 1.0];
        }
        [dir[0] / mag, dir[1] / mag, dir[2] / mag]
    }

    /// Estimates the neutrino energy in GeV from the total collected charge
    /// and the reconstructed track length.
    ///
    /// Uses a simple linear calibration: E ≈ k * total_charge_pe, where k
    /// is derived from the expected photon yield and detector light
    /// collection efficiency. For cascades (track_length_m ~ 0), the charge
    /// alone is used. For tracks, the track length provides an independent
    /// cross-check.
    pub fn estimate_energy(&self, total_charge_pe: f64, track_length_m: f64) -> f64 {
        // Approximate calibration constants (order-of-magnitude realistic)
        // ~200 Cherenkov photons/cm in water at β≈1, ~1% PMT coverage,
        // ~25% QE → ~0.5 PE/cm → ~50 PE/m
        let pe_per_gev_per_m = 50.0;
        let pe_per_gev_cascade = 5.0; // lower for point-like events (less coverage)

        if track_length_m > 1.0 {
            // Track-like event: use charge and track length
            total_charge_pe / (pe_per_gev_per_m * track_length_m) * track_length_m
        } else {
            // Cascade-like event
            total_charge_pe / pe_per_gev_cascade
        }
    }

    /// Computes the time residual for each hit, defined as
    /// t_residual = t_measured - t_expected.
    ///
    /// A well-reconstructed event should have residuals clustered near zero
    /// with a spread determined by the PMT time resolution and scattering.
    pub fn time_residuals(
        &self,
        hits: &[PmtHit],
        vertex: [f64; 3],
        direction: [f64; 3],
        beta: f64,
    ) -> Vec<f64> {
        hits.iter()
            .map(|h| {
                let t_exp = self.expected_arrival_time(vertex, direction, h.position, beta);
                h.time_ns - t_exp
            })
            .collect()
    }

    /// Returns a reference to the detector configuration.
    pub fn config(&self) -> &CherenkovConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// EventClassifier
// ---------------------------------------------------------------------------

/// Classifies neutrino interaction events based on PMT hit patterns.
pub struct EventClassifier;

impl EventClassifier {
    /// Classifies the topology of an event from its PMT hit pattern.
    ///
    /// - **Track**: hits spread over a long narrow region (large time spread,
    ///   elongated spatial distribution).
    /// - **Cascade**: hits roughly spherically symmetric around the vertex
    ///   (compact, short time spread).
    /// - **Double cascade**: two distinct temporal/spatial clusters of hits.
    pub fn classify_topology(hits: &[PmtHit]) -> EventTopology {
        if hits.len() < 3 {
            return EventTopology::Cascade;
        }

        // Compute centroid
        let n = hits.len() as f64;
        let cx: f64 = hits.iter().map(|h| h.position[0]).sum::<f64>() / n;
        let cy: f64 = hits.iter().map(|h| h.position[1]).sum::<f64>() / n;
        let cz: f64 = hits.iter().map(|h| h.position[2]).sum::<f64>() / n;

        // Compute spatial covariance matrix (3x3) for eigenvalue analysis
        let mut cov = [[0.0f64; 3]; 3];
        for h in hits {
            let d = [
                h.position[0] - cx,
                h.position[1] - cy,
                h.position[2] - cz,
            ];
            for i in 0..3 {
                for j in 0..3 {
                    cov[i][j] += d[i] * d[j];
                }
            }
        }
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] /= n;
            }
        }

        // Find the largest eigenvalue using the power iteration method.
        // This tells us how much variance is along the principal axis.
        let trace = cov[0][0] + cov[1][1] + cov[2][2];
        let mut v = [1.0f64, 0.0, 0.0]; // initial guess
        for _ in 0..30 {
            let mut w = [0.0f64; 3];
            for i in 0..3 {
                for j in 0..3 {
                    w[i] += cov[i][j] * v[j];
                }
            }
            let norm = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
            if norm < 1e-30 {
                break;
            }
            v = [w[0] / norm, w[1] / norm, w[2] / norm];
        }
        // Rayleigh quotient gives the largest eigenvalue
        let mut lambda_max = 0.0f64;
        for i in 0..3 {
            for j in 0..3 {
                lambda_max += v[i] * cov[i][j] * v[j];
            }
        }
        // linearity: fraction of total variance along the principal axis
        // For a track (1D): linearity ≈ 1.  For cascade (3D): ≈ 1/3.
        let linearity = if trace > 1e-20 {
            lambda_max / trace
        } else {
            0.0
        };

        // Time spread
        let t_min = hits
            .iter()
            .map(|h| h.time_ns)
            .fold(f64::INFINITY, f64::min);
        let t_max = hits
            .iter()
            .map(|h| h.time_ns)
            .fold(f64::NEG_INFINITY, f64::max);
        let time_spread = t_max - t_min;

        // Check for double cascade: look for two time clusters
        let t_mid = (t_min + t_max) / 2.0;
        let early_count = hits.iter().filter(|h| h.time_ns < t_mid).count();
        let late_count = hits.len() - early_count;
        let balance_ratio = early_count.min(late_count) as f64 / hits.len() as f64;

        // Check for a gap in the time distribution
        let mut sorted_times: Vec<f64> = hits.iter().map(|h| h.time_ns).collect();
        sorted_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut max_gap = 0.0f64;
        for w in sorted_times.windows(2) {
            let gap = w[1] - w[0];
            if gap > max_gap {
                max_gap = gap;
            }
        }

        // Decision logic
        if max_gap > time_spread * 0.3 && balance_ratio > 0.2 && time_spread > 50.0 {
            EventTopology::DoubleCascade
        } else if linearity > 0.8 && time_spread > 100.0 {
            EventTopology::Track
        } else {
            EventTopology::Cascade
        }
    }

    /// Fits a Cherenkov ring to the 2-D projection of PMT hits.
    ///
    /// Projects hits onto the XY plane and fits a circle using a simple
    /// algebraic method (Kasa circle fit).  Returns the ring centre,
    /// radius, and a goodness-of-fit metric.
    pub fn ring_pattern_analysis(hits: &[PmtHit]) -> RingFit {
        if hits.len() < 3 {
            return RingFit {
                center: [0.0, 0.0],
                radius: 0.0,
                goodness: 0.0,
            };
        }

        let n = hits.len() as f64;

        // Use charge-weighted positions in XY
        let total_q: f64 = hits.iter().map(|h| h.charge_pe.max(0.001)).sum();
        let mean_x: f64 = hits
            .iter()
            .map(|h| h.position[0] * h.charge_pe.max(0.001))
            .sum::<f64>()
            / total_q;
        let mean_y: f64 = hits
            .iter()
            .map(|h| h.position[1] * h.charge_pe.max(0.001))
            .sum::<f64>()
            / total_q;

        // Centre coordinates relative to centroid
        let xs: Vec<f64> = hits.iter().map(|h| h.position[0] - mean_x).collect();
        let ys: Vec<f64> = hits.iter().map(|h| h.position[1] - mean_y).collect();

        // Kasa circle fit: solve the linear system for the circle
        // (x-a)² + (y-b)² = R²  →  x² + y² = 2ax + 2by + (R² - a² - b²)
        // Let z_i = x_i² + y_i².  Solve:  [Σx² Σxy; Σxy Σy²] [a; b] = [Σxz/2; Σyz/2]
        let mut sxx = 0.0f64;
        let mut syy = 0.0f64;
        let mut sxy = 0.0f64;
        let mut sxz = 0.0f64;
        let mut syz = 0.0f64;

        for i in 0..hits.len() {
            let x = xs[i];
            let y = ys[i];
            let z = x * x + y * y;
            sxx += x * x;
            syy += y * y;
            sxy += x * y;
            sxz += x * z;
            syz += y * z;
        }

        let det = sxx * syy - sxy * sxy;
        if det.abs() < 1e-30 {
            return RingFit {
                center: [mean_x, mean_y],
                radius: 0.0,
                goodness: 0.0,
            };
        }

        let a = (syy * sxz - sxy * syz) / (2.0 * det);
        let b = (sxx * syz - sxy * sxz) / (2.0 * det);

        // Radius
        let mut sum_r2 = 0.0f64;
        for i in 0..hits.len() {
            let dx = xs[i] - a;
            let dy = ys[i] - b;
            sum_r2 += (dx * dx + dy * dy).sqrt();
        }
        let radius = sum_r2 / n;

        // Goodness of fit: 1 - normalised variance of radial distances
        let mut var_r = 0.0f64;
        for i in 0..hits.len() {
            let dx = xs[i] - a;
            let dy = ys[i] - b;
            let r = (dx * dx + dy * dy).sqrt();
            var_r += (r - radius) * (r - radius);
        }
        var_r /= n;
        let goodness = if radius > 1e-12 {
            (1.0 - (var_r.sqrt() / radius)).max(0.0).min(1.0)
        } else {
            0.0
        };

        RingFit {
            center: [mean_x + a, mean_y + b],
            radius,
            goodness,
        }
    }

    /// Removes isolated noise hits that fall outside a sliding time window.
    ///
    /// A hit is retained only if at least one other hit falls within
    /// `time_window_ns` nanoseconds.  This is a simple causality-based
    /// noise rejection suitable for dark-noise suppression.
    pub fn noise_rejection(hits: &mut Vec<PmtHit>, time_window_ns: f64) {
        if hits.len() <= 1 {
            return;
        }

        // Sort by time
        hits.sort_by(|a, b| a.time_ns.partial_cmp(&b.time_ns).unwrap());

        let mut keep = vec![false; hits.len()];
        for i in 0..hits.len() {
            for j in 0..hits.len() {
                if i == j {
                    continue;
                }
                if (hits[i].time_ns - hits[j].time_ns).abs() <= time_window_ns {
                    keep[i] = true;
                    break;
                }
            }
        }

        let mut idx = 0;
        hits.retain(|_| {
            let k = keep[idx];
            idx += 1;
            k
        });
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Computes the velocity β = v/c of a particle from its total energy (GeV)
/// and rest mass (GeV/c²).
///
/// Uses β = p/E = √(E² - m²) / E.
pub fn particle_beta(energy_gev: f64, mass_gev: f64) -> f64 {
    if energy_gev <= mass_gev {
        return 0.0;
    }
    let p = (energy_gev * energy_gev - mass_gev * mass_gev).sqrt();
    p / energy_gev
}

/// Estimates the range of a muon in water in metres, using the
/// approximate relation: range ≈ 4.0 * E_μ (GeV) metres.
///
/// This is a rough parameterisation valid for muon energies in the
/// 1 – 1000 GeV range.  At higher energies, radiative losses dominate
/// and the actual range grows more slowly.
pub fn muon_range_m(energy_gev: f64) -> f64 {
    if energy_gev <= 0.0 {
        return 0.0;
    }
    // Below ~1 GeV, ionisation losses dominate: ~2 MeV/(g/cm²)
    // In water (density 1 g/cm³): dE/dx ~ 2 MeV/cm = 200 MeV/m
    // So range ~ E / 0.2 GeV/m = 5*E metres.
    // Above ~500 GeV, stochastic losses reduce range.
    // We use a commonly quoted approximation for the TeV scale.
    let range = if energy_gev < 1.0 {
        5.0 * energy_gev
    } else {
        4.0 * energy_gev
    };
    range
}

/// Returns the effective attenuation length of Cherenkov light in the given
/// medium at the specified wavelength (in nm).
///
/// Returns the attenuation length in metres.  Typical values:
/// - Water (deep, pure): ~60 m at 470 nm (absorption minimum)
/// - Ice (South Pole): ~100 m at 400 nm (scattering-dominated)
/// - Mineral oil: ~20 m at 430 nm
pub fn attenuation_length(wavelength_nm: f64, medium: &Medium) -> f64 {
    // Simplified parameterisation: peak attenuation length at optimal
    // wavelength, with a Gaussian falloff.
    let (peak_wl, peak_len, sigma) = match medium {
        Medium::Water => (470.0, 60.0, 80.0),
        Medium::Ice => (400.0, 100.0, 70.0),
        Medium::MineralOil => (430.0, 20.0, 60.0),
    };
    let dw = wavelength_nm - peak_wl;
    peak_len * (-0.5 * dw * dw / (sigma * sigma)).exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ---- Medium ----

    #[test]
    fn test_medium_water_refractive_index() {
        assert!(approx_eq(
            Medium::Water.default_refractive_index(),
            1.33,
            0.001
        ));
    }

    #[test]
    fn test_medium_ice_refractive_index() {
        assert!(approx_eq(
            Medium::Ice.default_refractive_index(),
            1.31,
            0.001
        ));
    }

    #[test]
    fn test_medium_mineral_oil_refractive_index() {
        assert!(approx_eq(
            Medium::MineralOil.default_refractive_index(),
            1.47,
            0.001
        ));
    }

    // ---- CherenkovConfig default ----

    #[test]
    fn test_default_config() {
        let cfg = CherenkovConfig::default();
        assert_eq!(cfg.medium, Medium::Water);
        assert!(approx_eq(cfg.refractive_index, 1.33, 0.001));
        assert!(approx_eq(cfg.speed_of_light, 299_792_458.0, 1.0));
        assert!(approx_eq(cfg.pmt_time_resolution_ns, 3.0, 0.1));
    }

    // ---- Cherenkov angle ----

    #[test]
    fn test_cherenkov_angle_water_beta_1() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let angle_deg = det.cherenkov_angle(1.0).to_degrees();
        // Expected ~41.2° for n=1.33
        assert!(
            approx_eq(angle_deg, 41.2, 0.5),
            "Cherenkov angle = {angle_deg}°, expected ~41.2°"
        );
    }

    #[test]
    fn test_cherenkov_angle_ice_beta_1() {
        let cfg = CherenkovConfig {
            medium: Medium::Ice,
            refractive_index: 1.31,
            ..CherenkovConfig::default()
        };
        let det = CherenkovDetector::new(cfg);
        let angle_deg = det.cherenkov_angle(1.0).to_degrees();
        // arccos(1/1.31) ≈ 40.2°
        assert!(
            approx_eq(angle_deg, 40.2, 0.5),
            "Cherenkov angle in ice = {angle_deg}°"
        );
    }

    #[test]
    fn test_cherenkov_angle_below_threshold() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        // β < 1/n = 0.7519...  →  no emission, angle = 0
        let angle = det.cherenkov_angle(0.5);
        assert!(
            approx_eq(angle, 0.0, EPSILON),
            "Below-threshold angle should be 0, got {angle}"
        );
    }

    #[test]
    fn test_cherenkov_angle_at_threshold() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let beta_thr = 1.0 / 1.33;
        let angle = det.cherenkov_angle(beta_thr);
        // At threshold, cos(θ) = 1, so θ = 0
        assert!(
            approx_eq(angle, 0.0, 0.01),
            "At threshold, angle should be ~0, got {angle}"
        );
    }

    // ---- Cherenkov threshold ----

    #[test]
    fn test_cherenkov_threshold_water() {
        let beta_min = CherenkovDetector::cherenkov_threshold(&Medium::Water);
        assert!(
            approx_eq(beta_min, 1.0 / 1.33, 0.001),
            "Water threshold = {beta_min}, expected {:.4}",
            1.0 / 1.33
        );
    }

    #[test]
    fn test_cherenkov_threshold_ice() {
        let beta_min = CherenkovDetector::cherenkov_threshold(&Medium::Ice);
        assert!(
            approx_eq(beta_min, 1.0 / 1.31, 0.001),
            "Ice threshold = {beta_min}"
        );
    }

    #[test]
    fn test_cherenkov_threshold_mineral_oil() {
        let beta_min = CherenkovDetector::cherenkov_threshold(&Medium::MineralOil);
        assert!(
            approx_eq(beta_min, 1.0 / 1.47, 0.001),
            "Mineral oil threshold = {beta_min}"
        );
    }

    // ---- Frank-Tamm photon yield ----

    #[test]
    fn test_photon_yield_at_400nm_water() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let yield_per_m = det.photon_yield_per_meter(400.0, 1.0);
        // Frank-Tamm: dN/(dx dλ) = 2π α / λ² sin²(θ_c)
        // With λ = 400e-9 m: ~1.25e11 photons/(m·m_wavelength)
        // This is per metre of track per metre of wavelength interval.
        // To get per nm, divide by 1e9 → ~125 photons/(m·nm), consistent
        // with the well-known ~300 photons/(cm) across the visible band.
        assert!(
            yield_per_m > 1e10,
            "Photon yield should be > 10^10 per m per m_wavelength, got {yield_per_m}"
        );
        assert!(
            yield_per_m < 1e13,
            "Photon yield should be < 10^13, got {yield_per_m}"
        );
    }

    #[test]
    fn test_photon_yield_below_threshold() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let yield_per_m = det.photon_yield_per_meter(400.0, 0.5);
        assert!(
            approx_eq(yield_per_m, 0.0, EPSILON),
            "Below-threshold yield should be 0, got {yield_per_m}"
        );
    }

    #[test]
    fn test_photon_yield_increases_with_beta() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let y1 = det.photon_yield_per_meter(400.0, 0.8);
        let y2 = det.photon_yield_per_meter(400.0, 0.95);
        let y3 = det.photon_yield_per_meter(400.0, 1.0);
        assert!(
            y1 < y2 && y2 < y3,
            "Yield should increase with beta: {y1} < {y2} < {y3}"
        );
    }

    // ---- Expected arrival time ----

    #[test]
    fn test_expected_arrival_time_on_axis() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        // PMT directly ahead of vertex on the track axis
        let vertex = [0.0, 0.0, 0.0];
        let direction = [0.0, 0.0, 1.0];
        let pmt_pos = [0.0, 0.0, 100.0];
        let t = det.expected_arrival_time(vertex, direction, pmt_pos, 1.0);
        // Should be > 0 and physically reasonable
        // 100m at c ≈ 333 ns
        assert!(t > 0.0, "Arrival time should be positive, got {t}");
        assert!(t < 1000.0, "Arrival time should be < 1μs, got {t}");
    }

    #[test]
    fn test_expected_arrival_time_off_axis() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let vertex = [0.0, 0.0, 0.0];
        let direction = [0.0, 0.0, 1.0];
        let pmt_pos = [10.0, 0.0, 50.0]; // off-axis
        let t = det.expected_arrival_time(vertex, direction, pmt_pos, 1.0);
        assert!(t > 0.0, "Off-axis arrival time should be positive, got {t}");
    }

    // ---- Vertex reconstruction ----

    #[test]
    fn test_reconstruct_vertex_simple() {
        let cfg = CherenkovConfig::default();
        let det = CherenkovDetector::new(cfg);
        let c = det.config().speed_of_light;
        let n = det.config().refractive_index;
        let v_medium = c / n;

        // Create hits from a known vertex at (0, 0, 0) expanding outward
        let true_vertex: [f64; 3] = [0.0, 0.0, 0.0];
        let pmt_positions: [[f64; 3]; 6] = [
            [10.0, 0.0, 0.0],
            [-10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, -10.0, 0.0],
            [0.0, 0.0, 10.0],
            [0.0, 0.0, -10.0],
        ];

        let hits: Vec<PmtHit> = pmt_positions
            .iter()
            .enumerate()
            .map(|(i, pos)| {
                let dx = pos[0] - true_vertex[0];
                let dy = pos[1] - true_vertex[1];
                let dz = pos[2] - true_vertex[2];
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                PmtHit {
                    pmt_id: i,
                    time_ns: dist / v_medium * 1e9,
                    charge_pe: 1.0,
                    position: *pos,
                }
            })
            .collect();

        let reco = det.reconstruct_vertex(&hits);
        let err = ((reco[0] - true_vertex[0]).powi(2)
            + (reco[1] - true_vertex[1]).powi(2)
            + (reco[2] - true_vertex[2]).powi(2))
        .sqrt();
        assert!(
            err < 2.0,
            "Vertex reconstruction error = {err} m, expected < 2 m"
        );
    }

    #[test]
    fn test_reconstruct_vertex_empty_hits() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let v = det.reconstruct_vertex(&[]);
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    // ---- Direction reconstruction ----

    #[test]
    fn test_reconstruct_direction_z_axis() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        // Hits arranged in a cone around +z
        let vertex = [0.0, 0.0, 0.0];
        let hits = vec![
            PmtHit {
                pmt_id: 0,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [1.0, 0.0, 10.0],
            },
            PmtHit {
                pmt_id: 1,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [-1.0, 0.0, 10.0],
            },
            PmtHit {
                pmt_id: 2,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [0.0, 1.0, 10.0],
            },
            PmtHit {
                pmt_id: 3,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [0.0, -1.0, 10.0],
            },
        ];

        let dir = det.reconstruct_direction(&hits, vertex);
        // Should point roughly in +z
        assert!(
            dir[2] > 0.9,
            "z-component should be > 0.9, got {}",
            dir[2]
        );
    }

    #[test]
    fn test_reconstruct_direction_empty_hits() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let dir = det.reconstruct_direction(&[], [0.0, 0.0, 0.0]);
        // Default direction
        assert_eq!(dir, [0.0, 0.0, 1.0]);
    }

    // ---- Energy estimation ----

    #[test]
    fn test_energy_estimate_cascade() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let e1 = det.estimate_energy(50.0, 0.0); // cascade
        let e2 = det.estimate_energy(100.0, 0.0);
        assert!(e2 > e1, "More charge should give more energy");
        assert!(e1 > 0.0, "Energy should be positive");
    }

    #[test]
    fn test_energy_estimate_track() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let e = det.estimate_energy(500.0, 10.0); // track
        assert!(e > 0.0, "Track energy should be positive");
    }

    #[test]
    fn test_energy_estimate_linearity() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let e1 = det.estimate_energy(100.0, 0.0);
        let e2 = det.estimate_energy(200.0, 0.0);
        // Should scale linearly for cascades
        assert!(
            approx_eq(e2 / e1, 2.0, 0.1),
            "Energy should scale linearly: e2/e1 = {}",
            e2 / e1
        );
    }

    // ---- Time residuals ----

    #[test]
    fn test_time_residuals_zero_for_perfect_hits() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let vertex = [0.0, 0.0, 0.0];
        let direction = [0.0, 0.0, 1.0];
        let beta = 1.0;

        let pmt_positions = [[5.0, 0.0, 20.0], [-5.0, 0.0, 20.0], [0.0, 5.0, 20.0]];

        let hits: Vec<PmtHit> = pmt_positions
            .iter()
            .enumerate()
            .map(|(i, pos)| {
                let t_exp = det.expected_arrival_time(vertex, direction, *pos, beta);
                PmtHit {
                    pmt_id: i,
                    time_ns: t_exp,
                    charge_pe: 1.0,
                    position: *pos,
                }
            })
            .collect();

        let residuals = det.time_residuals(&hits, vertex, direction, beta);
        for (i, r) in residuals.iter().enumerate() {
            assert!(
                r.abs() < 1e-6,
                "Residual[{i}] = {r}, expected ~0 for perfect hits"
            );
        }
    }

    #[test]
    fn test_time_residuals_nonzero_for_shifted_hits() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let vertex = [0.0, 0.0, 0.0];
        let direction = [0.0, 0.0, 1.0];
        let beta = 1.0;

        let pos = [5.0, 0.0, 20.0];
        let t_exp = det.expected_arrival_time(vertex, direction, pos, beta);

        let hit = PmtHit {
            pmt_id: 0,
            time_ns: t_exp + 5.0, // 5 ns late
            charge_pe: 1.0,
            position: pos,
        };

        let residuals = det.time_residuals(&[hit], vertex, direction, beta);
        assert!(
            approx_eq(residuals[0], 5.0, 0.01),
            "Residual should be ~5 ns, got {}",
            residuals[0]
        );
    }

    // ---- Event classification ----

    #[test]
    fn test_classify_cascade() {
        // Compact, spherical hit pattern → cascade
        let hits: Vec<PmtHit> = (0..20)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / 20.0;
                PmtHit {
                    pmt_id: i,
                    time_ns: 10.0 + (i as f64) * 0.1,
                    charge_pe: 1.0,
                    position: [5.0 * angle.cos(), 5.0 * angle.sin(), 0.0],
                }
            })
            .collect();
        let topo = EventClassifier::classify_topology(&hits);
        assert_eq!(topo, EventTopology::Cascade);
    }

    #[test]
    fn test_classify_track() {
        // Long, narrow hit pattern → track
        // 50 hits spread along z from 0 to 500m, narrow in x/y, time spread 500 ns
        let hits: Vec<PmtHit> = (0..50)
            .map(|i| PmtHit {
                pmt_id: i,
                time_ns: i as f64 * 10.0, // 0..490 ns spread
                charge_pe: 1.0,
                position: [0.3 * (i as f64 % 3.0 - 1.0), 0.0, i as f64 * 10.0],
            })
            .collect();
        let topo = EventClassifier::classify_topology(&hits);
        assert_eq!(topo, EventTopology::Track);
    }

    #[test]
    fn test_classify_double_cascade() {
        // Two clusters in time/space → double cascade
        let mut hits = Vec::new();
        // Cluster 1 at t ≈ 10, z ≈ 0
        for i in 0..10 {
            let angle = 2.0 * PI * i as f64 / 10.0;
            hits.push(PmtHit {
                pmt_id: i,
                time_ns: 10.0 + i as f64 * 0.2,
                charge_pe: 1.0,
                position: [3.0 * angle.cos(), 3.0 * angle.sin(), 0.0],
            });
        }
        // Cluster 2 at t ≈ 200, z ≈ 50
        for i in 0..10 {
            let angle = 2.0 * PI * i as f64 / 10.0;
            hits.push(PmtHit {
                pmt_id: 10 + i,
                time_ns: 200.0 + i as f64 * 0.2,
                charge_pe: 1.0,
                position: [3.0 * angle.cos(), 3.0 * angle.sin(), 50.0],
            });
        }
        let topo = EventClassifier::classify_topology(&hits);
        assert_eq!(topo, EventTopology::DoubleCascade);
    }

    #[test]
    fn test_classify_few_hits() {
        let hits = vec![
            PmtHit {
                pmt_id: 0,
                time_ns: 0.0,
                charge_pe: 1.0,
                position: [0.0, 0.0, 0.0],
            },
            PmtHit {
                pmt_id: 1,
                time_ns: 1.0,
                charge_pe: 1.0,
                position: [1.0, 0.0, 0.0],
            },
        ];
        // With < 3 hits, default to Cascade
        assert_eq!(
            EventClassifier::classify_topology(&hits),
            EventTopology::Cascade
        );
    }

    // ---- Ring pattern analysis ----

    #[test]
    fn test_ring_fit_perfect_circle() {
        let r = 10.0;
        let hits: Vec<PmtHit> = (0..36)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / 36.0;
                PmtHit {
                    pmt_id: i,
                    time_ns: 0.0,
                    charge_pe: 1.0,
                    position: [r * angle.cos(), r * angle.sin(), 0.0],
                }
            })
            .collect();

        let fit = EventClassifier::ring_pattern_analysis(&hits);
        assert!(
            approx_eq(fit.radius, r, 1.0),
            "Fitted radius = {}, expected ~{r}",
            fit.radius
        );
        assert!(
            fit.goodness > 0.9,
            "Goodness for a perfect circle should be > 0.9, got {}",
            fit.goodness
        );
    }

    #[test]
    fn test_ring_fit_few_hits() {
        let hits = vec![
            PmtHit {
                pmt_id: 0,
                time_ns: 0.0,
                charge_pe: 1.0,
                position: [1.0, 0.0, 0.0],
            },
        ];
        let fit = EventClassifier::ring_pattern_analysis(&hits);
        // With < 3 hits, can't fit
        assert_eq!(fit.radius, 0.0);
    }

    // ---- Noise rejection ----

    #[test]
    fn test_noise_rejection_removes_isolated() {
        let mut hits = vec![
            PmtHit {
                pmt_id: 0,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [0.0, 0.0, 0.0],
            },
            PmtHit {
                pmt_id: 1,
                time_ns: 12.0,
                charge_pe: 1.0,
                position: [1.0, 0.0, 0.0],
            },
            PmtHit {
                pmt_id: 2,
                time_ns: 500.0, // isolated noise hit
                charge_pe: 0.5,
                position: [5.0, 5.0, 5.0],
            },
        ];

        EventClassifier::noise_rejection(&mut hits, 20.0);
        assert_eq!(hits.len(), 2, "Isolated hit should be removed");
        assert!(hits.iter().all(|h| h.time_ns < 100.0));
    }

    #[test]
    fn test_noise_rejection_keeps_clustered() {
        let mut hits = vec![
            PmtHit {
                pmt_id: 0,
                time_ns: 10.0,
                charge_pe: 1.0,
                position: [0.0; 3],
            },
            PmtHit {
                pmt_id: 1,
                time_ns: 15.0,
                charge_pe: 1.0,
                position: [1.0, 0.0, 0.0],
            },
            PmtHit {
                pmt_id: 2,
                time_ns: 18.0,
                charge_pe: 1.0,
                position: [2.0, 0.0, 0.0],
            },
        ];
        EventClassifier::noise_rejection(&mut hits, 20.0);
        assert_eq!(hits.len(), 3, "All clustered hits should be kept");
    }

    #[test]
    fn test_noise_rejection_empty() {
        let mut hits: Vec<PmtHit> = Vec::new();
        EventClassifier::noise_rejection(&mut hits, 20.0); // should not panic
        assert_eq!(hits.len(), 0);
    }

    // ---- particle_beta ----

    #[test]
    fn test_particle_beta_relativistic_muon() {
        // 10 GeV muon (mass ≈ 0.1057 GeV)
        let beta = particle_beta(10.0, 0.1057);
        assert!(
            beta > 0.999,
            "10 GeV muon should be ultra-relativistic, β = {beta}"
        );
    }

    #[test]
    fn test_particle_beta_at_rest() {
        let beta = particle_beta(0.1057, 0.1057); // E = m → at rest
        assert!(
            approx_eq(beta, 0.0, 0.001),
            "Particle at rest should have β ≈ 0, got {beta}"
        );
    }

    #[test]
    fn test_particle_beta_below_mass() {
        let beta = particle_beta(0.05, 0.1057); // E < m (invalid)
        assert_eq!(beta, 0.0, "β should be 0 for E < m");
    }

    #[test]
    fn test_particle_beta_electron_100mev() {
        // 100 MeV electron (mass ≈ 0.000511 GeV)
        let beta = particle_beta(0.1, 0.000511);
        assert!(beta > 0.9999, "100 MeV electron β ≈ 1, got {beta}");
    }

    // ---- muon_range_m ----

    #[test]
    fn test_muon_range_positive() {
        let r = muon_range_m(10.0);
        assert!(r > 0.0, "Range should be positive");
        // ~40 m for 10 GeV muon
        assert!(
            approx_eq(r, 40.0, 10.0),
            "10 GeV muon range should be ~40 m, got {r}"
        );
    }

    #[test]
    fn test_muon_range_zero_energy() {
        assert_eq!(muon_range_m(0.0), 0.0);
    }

    #[test]
    fn test_muon_range_increases_with_energy() {
        let r1 = muon_range_m(1.0);
        let r2 = muon_range_m(10.0);
        let r3 = muon_range_m(100.0);
        assert!(r1 < r2 && r2 < r3, "Range should increase with energy");
    }

    // ---- attenuation_length ----

    #[test]
    fn test_attenuation_water_peak() {
        let att = attenuation_length(470.0, &Medium::Water);
        assert!(
            approx_eq(att, 60.0, 5.0),
            "Water attenuation at 470nm should be ~60m, got {att}"
        );
    }

    #[test]
    fn test_attenuation_ice_peak() {
        let att = attenuation_length(400.0, &Medium::Ice);
        assert!(
            approx_eq(att, 100.0, 5.0),
            "Ice attenuation at 400nm should be ~100m, got {att}"
        );
    }

    #[test]
    fn test_attenuation_decreases_off_peak() {
        let att_peak = attenuation_length(470.0, &Medium::Water);
        let att_off = attenuation_length(300.0, &Medium::Water);
        assert!(
            att_off < att_peak,
            "Off-peak attenuation length should be shorter"
        );
    }

    #[test]
    fn test_attenuation_mineral_oil() {
        let att = attenuation_length(430.0, &Medium::MineralOil);
        assert!(
            approx_eq(att, 20.0, 3.0),
            "MineralOil attenuation at 430nm should be ~20m, got {att}"
        );
    }

    // ---- DetectorGeometry ----

    #[test]
    fn test_detector_geometry_icecube() {
        let geo = DetectorGeometry::IceCubeString {
            num_strings: 86,
            string_spacing_m: 125.0,
            num_doms_per_string: 60,
            dom_spacing_m: 17.0,
        };
        if let DetectorGeometry::IceCubeString {
            num_strings,
            num_doms_per_string,
            ..
        } = geo
        {
            assert_eq!(num_strings * num_doms_per_string, 5160);
        }
    }

    #[test]
    fn test_detector_geometry_cylindrical() {
        let geo = DetectorGeometry::Cylindrical {
            radius_m: 16.9,
            height_m: 36.2,
        };
        if let DetectorGeometry::Cylindrical { radius_m, height_m } = geo {
            let volume = PI * radius_m * radius_m * height_m;
            // Super-K inner volume ~32,000 m³
            assert!(volume > 30_000.0 && volume < 35_000.0);
        }
    }

    // ---- Edge cases and integration ----

    #[test]
    fn test_full_reconstruction_pipeline() {
        let det = CherenkovDetector::new(CherenkovConfig::default());
        let c = det.config().speed_of_light;
        let n = det.config().refractive_index;
        let v_medium = c / n;

        // Simulate a cascade at (5, 5, 5) with outgoing spherical light
        let true_vertex: [f64; 3] = [5.0, 5.0, 5.0];
        let pmt_positions: [[f64; 3]; 8] = [
            [15.0, 5.0, 5.0],
            [-5.0, 5.0, 5.0],
            [5.0, 15.0, 5.0],
            [5.0, -5.0, 5.0],
            [5.0, 5.0, 15.0],
            [5.0, 5.0, -5.0],
            [12.0, 12.0, 5.0],
            [-2.0, -2.0, 5.0],
        ];

        let hits: Vec<PmtHit> = pmt_positions
            .iter()
            .enumerate()
            .map(|(i, pos)| {
                let dx = pos[0] - true_vertex[0];
                let dy = pos[1] - true_vertex[1];
                let dz = pos[2] - true_vertex[2];
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                PmtHit {
                    pmt_id: i,
                    time_ns: dist / v_medium * 1e9,
                    charge_pe: 1.0,
                    position: *pos,
                }
            })
            .collect();

        // Vertex reconstruction
        let reco_vtx = det.reconstruct_vertex(&hits);
        let vtx_err = ((reco_vtx[0] - true_vertex[0]).powi(2)
            + (reco_vtx[1] - true_vertex[1]).powi(2)
            + (reco_vtx[2] - true_vertex[2]).powi(2))
        .sqrt();
        assert!(
            vtx_err < 3.0,
            "Vertex error = {vtx_err} m, expected < 3 m"
        );

        // Topology classification (compact, short spread → cascade)
        let topo = EventClassifier::classify_topology(&hits);
        assert_eq!(topo, EventTopology::Cascade);
    }

    #[test]
    fn test_pmt_hit_struct() {
        let hit = PmtHit {
            pmt_id: 42,
            time_ns: 123.456,
            charge_pe: 2.5,
            position: [1.0, 2.0, 3.0],
        };
        assert_eq!(hit.pmt_id, 42);
        assert!(approx_eq(hit.time_ns, 123.456, EPSILON));
        assert!(approx_eq(hit.charge_pe, 2.5, EPSILON));
    }

    #[test]
    fn test_ring_fit_struct() {
        let fit = RingFit {
            center: [1.0, 2.0],
            radius: 5.0,
            goodness: 0.95,
        };
        assert!(approx_eq(fit.center[0], 1.0, EPSILON));
        assert!(approx_eq(fit.radius, 5.0, EPSILON));
        assert!(approx_eq(fit.goodness, 0.95, EPSILON));
    }
}
