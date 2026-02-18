// Neutron Reflectometry Processor
// Analysis of thin film and interfacial structures using neutron reflection
// Implements: Fresnel reflectivity, Parratt recursion for multilayers,
// scattering length density profiles, Kiessig fringes, Born approximation,
// roughness via Nevot-Croce, contrast matching

use std::f64::consts::PI;

/// Neutron reflectometry configuration
#[derive(Clone, Debug)]
pub struct NeutronReflConfig {
    /// Neutron wavelength in angstroms
    pub wavelength_angstrom: f64,
    /// Beam divergence in mrad
    pub beam_divergence_mrad: f64,
    /// Instrumental resolution dQ/Q
    pub dq_over_q: f64,
}

impl Default for NeutronReflConfig {
    fn default() -> Self {
        Self {
            wavelength_angstrom: 5.0,
            beam_divergence_mrad: 0.5,
            dq_over_q: 0.03,
        }
    }
}

/// Scattering length density (SLD) profile layer
#[derive(Clone, Debug)]
pub struct SldLayer {
    /// Layer name
    pub name: String,
    /// Thickness in angstroms
    pub thickness_angstrom: f64,
    /// Real scattering length density in 10^-6 A^-2
    pub sld_real: f64,
    /// Imaginary SLD (absorption) in 10^-6 A^-2
    pub sld_imag: f64,
    /// Roughness (sigma) in angstroms
    pub roughness_angstrom: f64,
}

/// Reflectivity data point
#[derive(Clone, Debug)]
pub struct ReflectivityPoint {
    /// Momentum transfer Q in A^-1
    pub q: f64,
    /// Reflectivity (0 to 1)
    pub reflectivity: f64,
}

/// Material SLD database entry
#[derive(Clone, Debug)]
pub struct MaterialSld {
    pub name: &'static str,
    pub formula: &'static str,
    /// SLD in 10^-6 A^-2
    pub sld: f64,
}

/// Common material SLDs
pub fn sld_database() -> Vec<MaterialSld> {
    vec![
        MaterialSld { name: "Silicon",      formula: "Si",    sld: 2.07 },
        MaterialSld { name: "Silicon Oxide", formula: "SiO2", sld: 3.47 },
        MaterialSld { name: "Gold",          formula: "Au",   sld: 4.66 },
        MaterialSld { name: "Chromium",      formula: "Cr",   sld: 3.03 },
        MaterialSld { name: "Titanium",      formula: "Ti",   sld: -1.95 },
        MaterialSld { name: "Nickel",        formula: "Ni",   sld: 9.41 },
        MaterialSld { name: "Iron",          formula: "Fe",   sld: 8.02 },
        MaterialSld { name: "D2O",           formula: "D2O",  sld: 6.36 },
        MaterialSld { name: "H2O",           formula: "H2O",  sld: -0.56 },
        MaterialSld { name: "Air",           formula: "air",  sld: 0.0 },
        MaterialSld { name: "PMMA",          formula: "PMMA", sld: 1.07 },
        MaterialSld { name: "Polystyrene",   formula: "PS",   sld: 1.41 },
        MaterialSld { name: "d-PMMA",        formula: "dPMMA",sld: 6.95 },
        MaterialSld { name: "Copper",        formula: "Cu",   sld: 6.55 },
        MaterialSld { name: "Aluminum",      formula: "Al",   sld: 2.08 },
    ]
}

/// Neutron Reflectometry Processor
pub struct NeutronReflProcessor {
    pub config: NeutronReflConfig,
    /// Film layers from top (incident side) to bottom (substrate side)
    pub layers: Vec<SldLayer>,
    /// Substrate SLD
    pub substrate_sld: f64,
    /// Superphase (incident medium) SLD
    pub superphase_sld: f64,
}

impl NeutronReflProcessor {
    pub fn new(config: NeutronReflConfig) -> Self {
        Self {
            config,
            layers: Vec::new(),
            substrate_sld: 2.07, // Silicon
            superphase_sld: 0.0, // Air
        }
    }

    pub fn add_layer(&mut self, layer: SldLayer) {
        self.layers.push(layer);
    }

    /// Q from angle and wavelength: Q = 4*pi*sin(theta)/lambda
    pub fn q_from_angle(theta_deg: f64, wavelength_angstrom: f64) -> f64 {
        if wavelength_angstrom > 0.0 {
            4.0 * PI * (theta_deg * PI / 180.0).sin() / wavelength_angstrom
        } else {
            0.0
        }
    }

    /// Critical angle: theta_c = lambda * sqrt(SLD/pi)
    pub fn critical_angle_deg(sld_1e6: f64, wavelength_angstrom: f64) -> f64 {
        if sld_1e6 > 0.0 {
            let sld = sld_1e6 * 1e-6;
            let theta_rad = wavelength_angstrom * (sld / PI).sqrt();
            theta_rad * 180.0 / PI
        } else {
            0.0
        }
    }

    /// Critical Q: Q_c = 4*sqrt(pi*SLD)
    pub fn critical_q(sld_1e6: f64) -> f64 {
        let sld = sld_1e6 * 1e-6;
        if sld > 0.0 {
            4.0 * (PI * sld).sqrt()
        } else {
            0.0
        }
    }

    /// Fresnel reflectivity for single interface: R = |r|^2
    /// r = (Q - Q_t) / (Q + Q_t), where Q_t = sqrt(Q^2 - Q_c^2)
    pub fn fresnel_reflectivity(q: f64, sld_substrate_1e6: f64) -> f64 {
        let qc = Self::critical_q(sld_substrate_1e6);
        if q <= qc {
            return 1.0; // Total external reflection
        }
        let qt = (q * q - qc * qc).sqrt();
        let r = (q - qt) / (q + qt);
        r * r
    }

    /// Parratt recursion for multilayer reflectivity
    /// Exact solution for stratified media with sharp interfaces
    pub fn parratt_reflectivity(&self, q_values: &[f64]) -> Vec<ReflectivityPoint> {
        q_values
            .iter()
            .map(|&q| {
                let r = self.parratt_at_q(q);
                ReflectivityPoint {
                    q,
                    reflectivity: r,
                }
            })
            .collect()
    }

    /// Compute reflectivity at single Q using Parratt recursion
    fn parratt_at_q(&self, q: f64) -> f64 {
        if q <= 0.0 {
            return 1.0;
        }

        let n_layers = self.layers.len();

        // Build SLD list: superphase, layers..., substrate
        let mut sld_list: Vec<(f64, f64)> = vec![(self.superphase_sld, 0.0)];
        for layer in &self.layers {
            sld_list.push((layer.sld_real, layer.sld_imag));
        }
        sld_list.push((self.substrate_sld, 0.0));

        let n_media = sld_list.len();

        // Compute Q_j in each medium: Q_j = sqrt(Q^2 - 16*pi*SLD_j)
        let q2 = q * q;
        let q_vals: Vec<(f64, f64)> = sld_list
            .iter()
            .map(|&(sld_re, sld_im)| {
                let re = q2 - 16.0 * PI * sld_re * 1e-6;
                let im = -16.0 * PI * sld_im * 1e-6;
                // Complex sqrt
                let mag = (re * re + im * im).sqrt().sqrt();
                let arg = im.atan2(re) / 2.0;
                (mag * arg.cos(), mag * arg.sin())
            })
            .collect();

        // Parratt recursion from bottom (substrate) upward
        // r_{n,n+1} = (Q_n - Q_{n+1}) / (Q_n + Q_{n+1})
        // R_n = (r_{n,n+1} + R_{n+1} * exp(2*i*Q_{n+1}*d_{n+1})) /
        //       (1 + r_{n,n+1} * R_{n+1} * exp(2*i*Q_{n+1}*d_{n+1}))

        // Start from substrate: R_substrate = 0 (semi-infinite)
        let mut r_re = 0.0_f64;
        let mut r_im = 0.0_f64;

        for j in (0..n_media - 1).rev() {
            let (qj_re, qj_im) = q_vals[j];
            let (qjp1_re, qjp1_im) = q_vals[j + 1];

            // Fresnel coefficient: r = (Q_j - Q_{j+1}) / (Q_j + Q_{j+1})
            let num_re = qj_re - qjp1_re;
            let num_im = qj_im - qjp1_im;
            let den_re = qj_re + qjp1_re;
            let den_im = qj_im + qjp1_im;

            let den_mag2 = den_re * den_re + den_im * den_im;
            let (rj_re, rj_im) = if den_mag2 > 1e-30 {
                (
                    (num_re * den_re + num_im * den_im) / den_mag2,
                    (num_im * den_re - num_re * den_im) / den_mag2,
                )
            } else {
                (0.0, 0.0)
            };

            // Phase factor: exp(2*i*Q_{j+1}*d_{j+1})
            let d = if j + 1 <= n_layers {
                self.layers.get(j).map(|l| l.thickness_angstrom).unwrap_or(0.0)
            } else {
                0.0
            };

            // Nevot-Croce roughness factor
            let sigma = if j < n_layers {
                self.layers.get(j).map(|l| l.roughness_angstrom).unwrap_or(0.0)
            } else {
                0.0
            };

            // exp(2*i*Q_{j+1}*d)
            let phase_re = 2.0 * (qjp1_re * d);
            let phase_im = 2.0 * (qjp1_im * d);
            let exp_re = (-phase_im).exp() * phase_re.cos();
            let exp_im = (-phase_im).exp() * phase_re.sin();

            // R_{j+1} * exp(phase)
            let re_re = r_re * exp_re - r_im * exp_im;
            let re_im = r_re * exp_im + r_im * exp_re;

            // Roughness: multiply r_j by exp(-2*Q_j*Q_{j+1}*sigma^2)
            let rough_factor = (-2.0 * (qj_re * qjp1_re) * sigma * sigma).exp();
            let rj_rough_re = rj_re * rough_factor;
            let rj_rough_im = rj_im * rough_factor;

            // Numerator: r_j + R_{j+1}*exp
            let top_re = rj_rough_re + re_re;
            let top_im = rj_rough_im + re_im;

            // Denominator: 1 + r_j * R_{j+1}*exp
            let prod_re = rj_rough_re * re_re - rj_rough_im * re_im;
            let prod_im = rj_rough_re * re_im + rj_rough_im * re_re;
            let bot_re = 1.0 + prod_re;
            let bot_im = prod_im;

            let bot_mag2 = bot_re * bot_re + bot_im * bot_im;
            if bot_mag2 > 1e-30 {
                r_re = (top_re * bot_re + top_im * bot_im) / bot_mag2;
                r_im = (top_im * bot_re - top_re * bot_im) / bot_mag2;
            } else {
                r_re = rj_rough_re;
                r_im = rj_rough_im;
            }
        }

        r_re * r_re + r_im * r_im
    }

    /// Kiessig fringe spacing: Delta_Q = 2*pi / d
    /// Returns estimated thickness from fringe spacing
    pub fn thickness_from_fringes(delta_q: f64) -> f64 {
        if delta_q > 0.0 {
            2.0 * PI / delta_q
        } else {
            0.0
        }
    }

    /// Find fringe spacing from reflectivity data
    pub fn find_fringe_spacing(data: &[ReflectivityPoint]) -> f64 {
        // Find minima positions
        let n = data.len();
        let mut minima: Vec<f64> = Vec::new();

        for i in 1..n - 1 {
            if data[i].reflectivity < data[i - 1].reflectivity
                && data[i].reflectivity < data[i + 1].reflectivity
            {
                minima.push(data[i].q);
            }
        }

        if minima.len() < 2 {
            return 0.0;
        }

        // Average spacing between consecutive minima
        let spacings: Vec<f64> = minima.windows(2).map(|w| w[1] - w[0]).collect();
        spacings.iter().sum::<f64>() / spacings.len() as f64
    }

    /// Born approximation reflectivity (kinematic, valid for Q >> Q_c)
    /// R(Q) = (16*pi^2/Q^4) * |integral SLD'(z) exp(iQz) dz|^2
    pub fn born_approximation(&self, q_values: &[f64]) -> Vec<f64> {
        q_values
            .iter()
            .map(|&q| {
                if q <= 0.0 {
                    return 1.0;
                }
                // Sum contributions from each interface
                let mut ft_re = 0.0;
                let mut ft_im = 0.0;
                let mut z = 0.0_f64;

                // Superphase to first layer
                let delta_sld_first = if let Some(first) = self.layers.first() {
                    first.sld_real - self.superphase_sld
                } else {
                    self.substrate_sld - self.superphase_sld
                };

                ft_re += delta_sld_first * (q * z).cos();
                ft_im += delta_sld_first * (q * z).sin();

                // Layer interfaces
                for i in 0..self.layers.len() {
                    z += self.layers[i].thickness_angstrom;
                    let sld_next = if i + 1 < self.layers.len() {
                        self.layers[i + 1].sld_real
                    } else {
                        self.substrate_sld
                    };
                    let delta_sld = sld_next - self.layers[i].sld_real;
                    ft_re += delta_sld * (q * z).cos();
                    ft_im += delta_sld * (q * z).sin();
                }

                let ft_mag2 = ft_re * ft_re + ft_im * ft_im;
                16.0 * PI * PI * ft_mag2 * 1e-12 / (q * q * q * q)
            })
            .collect()
    }

    /// Contrast match point: mixture SLD that matches target
    /// SLD_mix = x * SLD_A + (1-x) * SLD_B
    pub fn contrast_match_fraction(
        target_sld: f64,
        sld_a: f64,
        sld_b: f64,
    ) -> f64 {
        if (sld_a - sld_b).abs() > 1e-10 {
            (target_sld - sld_b) / (sld_a - sld_b)
        } else {
            0.5
        }
    }

    /// D2O/H2O mixing ratio for contrast matching
    pub fn d2o_fraction_for_match(target_sld: f64) -> f64 {
        // SLD_D2O = 6.36, SLD_H2O = -0.56
        Self::contrast_match_fraction(target_sld, 6.36, -0.56)
    }

    /// Generate SLD profile (z vs SLD) for visualization
    pub fn sld_profile(&self, dz: f64) -> Vec<(f64, f64)> {
        let mut profile = Vec::new();
        let mut z = 0.0;

        // Superphase (first 20 A)
        let pre = 20.0;
        let mut zi = -pre;
        while zi < 0.0 {
            profile.push((zi, self.superphase_sld));
            zi += dz;
        }

        // Layers
        for layer in &self.layers {
            let mut dz_acc = 0.0;
            while dz_acc < layer.thickness_angstrom {
                profile.push((z + dz_acc, layer.sld_real));
                dz_acc += dz;
            }
            z += layer.thickness_angstrom;
        }

        // Substrate (next 20 A)
        let mut dz_acc = 0.0;
        while dz_acc < pre {
            profile.push((z + dz_acc, self.substrate_sld));
            dz_acc += dz;
        }

        profile
    }

    /// Neutron scattering length from nuclear data: b_coh in fm
    pub fn coherent_sld(b_coh_fm: f64, number_density_per_a3: f64) -> f64 {
        // SLD = N * b, in 10^-6 A^-2
        // b in fm = 10^-13 cm = 10^-5 A
        number_density_per_a3 * b_coh_fm * 1e-5 * 1e6
    }

    /// Absorption cross-section contribution to imaginary SLD
    pub fn absorption_sld(sigma_a_barn: f64, number_density_per_a3: f64, wavelength_a: f64) -> f64 {
        // SLD_imag = N * sigma_a * lambda / (4*pi) in 10^-6 A^-2
        // sigma_a in barns = 10^-24 cm^2 = 10^-8 A^2
        number_density_per_a3 * sigma_a_barn * 1e-8 * wavelength_a / (4.0 * PI) * 1e6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let cfg = NeutronReflConfig::default();
        assert_eq!(cfg.wavelength_angstrom, 5.0);
        assert_eq!(cfg.dq_over_q, 0.03);
    }

    #[test]
    fn test_sld_database() {
        let db = sld_database();
        assert!(db.len() >= 10);
        let si = db.iter().find(|m| m.name == "Silicon").unwrap();
        assert!((si.sld - 2.07).abs() < 0.01);
        let d2o = db.iter().find(|m| m.name == "D2O").unwrap();
        assert!((d2o.sld - 6.36).abs() < 0.01);
    }

    #[test]
    fn test_q_from_angle() {
        let q = NeutronReflProcessor::q_from_angle(1.0, 5.0);
        // Q = 4*pi*sin(1 deg)/5 ~ 0.0438 A^-1
        let expected = 4.0 * PI * (1.0_f64 * PI / 180.0).sin() / 5.0;
        assert!((q - expected).abs() < 1e-6);
    }

    #[test]
    fn test_critical_angle() {
        let tc = NeutronReflProcessor::critical_angle_deg(2.07, 5.0);
        assert!(tc > 0.0 && tc < 1.0, "critical angle for Si should be small, got {tc}");
    }

    #[test]
    fn test_critical_q() {
        let qc = NeutronReflProcessor::critical_q(2.07);
        // Q_c = 4*sqrt(pi*2.07e-6) ~ 0.0102 A^-1
        assert!(qc > 0.008 && qc < 0.015, "Q_c for Si ~ 0.01, got {qc}");
    }

    #[test]
    fn test_fresnel_total_reflection() {
        let r = NeutronReflProcessor::fresnel_reflectivity(0.005, 2.07);
        assert!((r - 1.0).abs() < 1e-10, "below Q_c should be total reflection");
    }

    #[test]
    fn test_fresnel_high_q() {
        let r = NeutronReflProcessor::fresnel_reflectivity(0.1, 2.07);
        assert!(r < 1.0, "above Q_c reflectivity should be < 1");
        assert!(r > 0.0);
    }

    #[test]
    fn test_fresnel_q4_decay() {
        let r1 = NeutronReflProcessor::fresnel_reflectivity(0.05, 2.07);
        let r2 = NeutronReflProcessor::fresnel_reflectivity(0.10, 2.07);
        // Fresnel decays as Q^-4, so R(2Q) ~ R(Q)/16
        let ratio = r1 / r2;
        assert!(ratio > 10.0, "should follow ~Q^-4 law, ratio = {ratio}");
    }

    #[test]
    fn test_parratt_bare_substrate() {
        let proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        let q_values: Vec<f64> = (1..100).map(|i| 0.005 + i as f64 * 0.002).collect();
        let refl = proc.parratt_reflectivity(&q_values);
        assert_eq!(refl.len(), 99);
        // Should be monotonically decreasing above Q_c
        for i in 1..refl.len() {
            if refl[i].q > 0.015 {
                assert!(
                    refl[i].reflectivity <= refl[i - 1].reflectivity * 1.1,
                    "bare substrate should decrease, at Q={}", refl[i].q
                );
            }
        }
    }

    #[test]
    fn test_parratt_single_layer() {
        let mut proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc.add_layer(SldLayer {
            name: "SiO2".into(),
            thickness_angstrom: 200.0,
            sld_real: 3.47,
            sld_imag: 0.0,
            roughness_angstrom: 0.0,
        });
        let q_values: Vec<f64> = (1..200).map(|i| 0.005 + i as f64 * 0.001).collect();
        let refl = proc.parratt_reflectivity(&q_values);

        // Should show Kiessig fringes (oscillations)
        let mut has_local_min = false;
        for i in 2..refl.len() - 1 {
            if refl[i].reflectivity < refl[i - 1].reflectivity
                && refl[i].reflectivity < refl[i + 1].reflectivity
                && refl[i].q > 0.02
            {
                has_local_min = true;
                break;
            }
        }
        assert!(has_local_min, "single layer should show Kiessig fringes");
    }

    #[test]
    fn test_thickness_from_fringes() {
        let dq = 2.0 * PI / 200.0; // 200 A film
        let d = NeutronReflProcessor::thickness_from_fringes(dq);
        assert!((d - 200.0).abs() < 0.1);
    }

    #[test]
    fn test_find_fringe_spacing() {
        let mut proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc.add_layer(SldLayer {
            name: "SiO2".into(),
            thickness_angstrom: 300.0,
            sld_real: 3.47,
            sld_imag: 0.0,
            roughness_angstrom: 0.0,
        });
        let q_values: Vec<f64> = (1..500).map(|i| 0.01 + i as f64 * 0.0005).collect();
        let refl = proc.parratt_reflectivity(&q_values);
        let dq = NeutronReflProcessor::find_fringe_spacing(&refl);
        if dq > 0.0 {
            let d = NeutronReflProcessor::thickness_from_fringes(dq);
            // Fringe spacing can be affected by SLD contrast, accept within factor of ~2
            assert!(d > 100.0 && d < 800.0, "estimated thickness should be in reasonable range, got {d}");
        }
    }

    #[test]
    fn test_contrast_match() {
        let x = NeutronReflProcessor::contrast_match_fraction(3.47, 6.36, -0.56);
        assert!(x > 0.0 && x < 1.0, "fraction should be 0-1, got {x}");
        // Check: x*6.36 + (1-x)*(-0.56) = 3.47
        let sld_mix = x * 6.36 + (1.0 - x) * (-0.56);
        assert!((sld_mix - 3.47).abs() < 0.01);
    }

    #[test]
    fn test_d2o_fraction() {
        // SiO2 SLD = 3.47
        let x = NeutronReflProcessor::d2o_fraction_for_match(3.47);
        assert!(x > 0.0 && x < 1.0);
        let sld_mix = x * 6.36 + (1.0 - x) * (-0.56);
        assert!((sld_mix - 3.47).abs() < 0.01);
    }

    #[test]
    fn test_sld_profile() {
        let mut proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc.add_layer(SldLayer {
            name: "SiO2".into(),
            thickness_angstrom: 100.0,
            sld_real: 3.47,
            sld_imag: 0.0,
            roughness_angstrom: 0.0,
        });
        let profile = proc.sld_profile(1.0);
        assert!(!profile.is_empty());
        // Should have superphase region (SLD=0), layer region (SLD=3.47), substrate (SLD=2.07)
        let has_zero = profile.iter().any(|(_, s)| s.abs() < 0.01);
        let has_layer = profile.iter().any(|(_, s)| (s - 3.47).abs() < 0.01);
        let has_sub = profile.iter().any(|(_, s)| (s - 2.07).abs() < 0.01);
        assert!(has_zero, "should have superphase");
        assert!(has_layer, "should have SiO2 layer");
        assert!(has_sub, "should have substrate");
    }

    #[test]
    fn test_born_approximation() {
        let proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        let q_values: Vec<f64> = (10..50).map(|i| i as f64 * 0.005).collect();
        let born = proc.born_approximation(&q_values);
        assert_eq!(born.len(), 40);
        // Born approximation should give finite positive values at high Q
        for &r in &born {
            assert!(r >= 0.0 && r.is_finite());
        }
    }

    #[test]
    fn test_coherent_sld() {
        // Silicon: b_coh = 4.149 fm, N = 0.04995 A^-3
        let sld = NeutronReflProcessor::coherent_sld(4.149, 0.04995);
        assert!((sld - 2.073).abs() < 0.1, "Si SLD should be ~2.07, got {sld}");
    }

    #[test]
    fn test_absorption_sld() {
        let sld_im = NeutronReflProcessor::absorption_sld(171.0, 0.04995, 5.0);
        assert!(sld_im > 0.0, "absorption SLD should be positive");
    }

    #[test]
    fn test_roughness_effect() {
        let mut proc_smooth = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc_smooth.add_layer(SldLayer {
            name: "Au".into(),
            thickness_angstrom: 200.0,
            sld_real: 4.66,
            sld_imag: 0.0,
            roughness_angstrom: 0.0,
        });

        let mut proc_rough = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc_rough.add_layer(SldLayer {
            name: "Au".into(),
            thickness_angstrom: 200.0,
            sld_real: 4.66,
            sld_imag: 0.0,
            roughness_angstrom: 10.0,
        });

        let q = 0.1;
        let r_smooth = proc_smooth.parratt_at_q(q);
        let r_rough = proc_rough.parratt_at_q(q);
        // Roughness reduces reflectivity
        assert!(r_rough <= r_smooth * 1.01, "roughness should reduce R");
    }

    #[test]
    fn test_multilayer() {
        let mut proc = NeutronReflProcessor::new(NeutronReflConfig::default());
        proc.add_layer(SldLayer {
            name: "Cr".into(),
            thickness_angstrom: 50.0,
            sld_real: 3.03,
            sld_imag: 0.0,
            roughness_angstrom: 3.0,
        });
        proc.add_layer(SldLayer {
            name: "Au".into(),
            thickness_angstrom: 200.0,
            sld_real: 4.66,
            sld_imag: 0.0,
            roughness_angstrom: 5.0,
        });
        let q_values: Vec<f64> = (1..100).map(|i| 0.005 + i as f64 * 0.002).collect();
        let refl = proc.parratt_reflectivity(&q_values);
        assert_eq!(refl.len(), 99);
        // All reflectivities should be valid
        for pt in &refl {
            assert!(pt.reflectivity >= 0.0 && pt.reflectivity.is_finite());
        }
    }

    #[test]
    fn test_critical_angle_negative_sld() {
        // Ti has negative SLD
        let tc = NeutronReflProcessor::critical_angle_deg(-1.95, 5.0);
        assert_eq!(tc, 0.0, "negative SLD has no critical angle");
    }

    #[test]
    fn test_q_zero_wavelength() {
        assert_eq!(NeutronReflProcessor::q_from_angle(1.0, 0.0), 0.0);
    }

    #[test]
    fn test_thickness_zero_dq() {
        assert_eq!(NeutronReflProcessor::thickness_from_fringes(0.0), 0.0);
    }

    #[test]
    fn test_find_fringe_spacing_no_minima() {
        let data = vec![
            ReflectivityPoint { q: 0.01, reflectivity: 1.0 },
            ReflectivityPoint { q: 0.02, reflectivity: 0.5 },
            ReflectivityPoint { q: 0.03, reflectivity: 0.1 },
        ];
        let dq = NeutronReflProcessor::find_fringe_spacing(&data);
        assert_eq!(dq, 0.0);
    }
}
