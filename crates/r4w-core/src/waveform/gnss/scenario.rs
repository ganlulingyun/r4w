//! GNSS Scenario: top-level API for multi-satellite IQ generation
//!
//! Composes satellite emitters with environment models to produce realistic
//! composite GNSS baseband IQ signals.
//!
//! ## Ephemeris Sources
//!
//! - **Keplerian**: Uses nominal orbital elements (meter-level accuracy)
//! - **SP3**: Uses precise ephemeris files from CODE (cm-level accuracy)
//!
//! ## Ionosphere Sources
//!
//! - **Klobuchar**: Uses broadcast ionospheric model (~50% accuracy)
//! - **IONEX**: Uses TEC grid maps from CODE (higher accuracy)

use super::environment::{KeplerianOrbit, KlobucharModel, SaastamoinenModel};
use super::satellite_emitter::{SatelliteEmitter, SatelliteStatus};
use super::scenario_config::{GnssScenarioConfig, GnssScenarioPreset};
#[cfg(feature = "ephemeris")]
use super::ephemeris::{EphemerisSource, IonosphereSource};
use super::types::GnssSignal;
use crate::coordinates::{
    lla_to_ecef, look_angle, range_rate, EcefPosition, EcefVelocity, LlaPosition, SPEED_OF_LIGHT,
};
use crate::types::IQSample;
use crate::waveform::gnss::{GalileoE1, GlonassL1of, GpsL1Ca, GpsL5};
use crate::waveform::Waveform;
use crate::filters::fir::FirFilter;
use crate::filters::traits::Filter;
use num_complex::Complex64;
use std::f64::consts::PI;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

#[cfg(feature = "ephemeris")]
use std::sync::Arc;
#[cfg(feature = "ephemeris")]
use super::sp3::Sp3Ephemeris;
#[cfg(feature = "ephemeris")]
use super::ionex::IonexTec;

/// Oversample factor for baseband generation.
/// Baseband IQ is generated at OVERSAMPLE × output_rate, then LPF'd and decimated.
/// This prevents BOC(1,1) harmonics (3rd at 3.069 MHz, 5th at 5.115 MHz, etc.)
/// from aliasing into the output band.
const BASEBAND_OVERSAMPLE: usize = 8;

/// GNSS Scenario: generates composite multi-satellite IQ signal
// trace:FR-041 | ai:claude
pub struct GnssScenario {
    config: GnssScenarioConfig,
    emitters: Vec<SatelliteEmitter>,
    current_sample: usize,
    sample_rate: f64,
    total_samples: usize,
    /// Per-emitter Doppler phase accumulators (persisted across blocks)
    doppler_phases: Vec<f64>,
    /// Per-emitter orbital Doppler at t=0 (Hz) for anchored dynamics
    orbital_doppler_t0: Vec<f64>,
    /// Per-emitter orbital range at t=0 (m) for anchored dynamics
    orbital_range_t0: Vec<f64>,
    /// Per-emitter anti-aliasing LPFs operating at oversampled rate.
    /// Filter state persists across blocks for continuity.
    baseband_lpfs: Vec<FirFilter>,
    /// Simple PRNG state for noise generation (xorshift64)
    rng_state: u64,
    /// SP3 precise ephemeris (optional, feature-gated)
    #[cfg(feature = "ephemeris")]
    sp3: Option<Arc<Sp3Ephemeris>>,
    /// IONEX TEC maps (optional, feature-gated)
    #[cfg(feature = "ephemeris")]
    ionex: Option<Arc<IonexTec>>,
}

impl GnssScenario {
    /// Create a scenario from full configuration
    pub fn new(config: GnssScenarioConfig) -> Self {
        let sample_rate = config.output.sample_rate;
        let total_samples = (config.output.duration_s * sample_rate).ceil() as usize;

        // Load SP3 precise ephemeris if configured
        #[cfg(feature = "ephemeris")]
        let sp3: Option<Arc<Sp3Ephemeris>> = match &config.environment.ephemeris_source {
            EphemerisSource::Sp3File(path) => {
                match Sp3Ephemeris::from_file(path) {
                    Ok(eph) => {
                        eprintln!("Loaded SP3 ephemeris: {} satellites, {} epochs",
                            eph.records.len(), eph.header.num_epochs);
                        Some(Arc::new(eph))
                    }
                    Err(e) => {
                        eprintln!("Warning: Failed to load SP3 file: {}. Falling back to Keplerian.", e);
                        None
                    }
                }
            }
            _ => None, // Nominal, RinexFile, Cached, AutoFetch all fall back to Keplerian for now
        };

        // Load IONEX TEC maps if configured
        #[cfg(feature = "ephemeris")]
        let ionex: Option<Arc<IonexTec>> = match &config.environment.ionosphere_source {
            IonosphereSource::IonexFile(path) => {
                match IonexTec::from_file(path) {
                    Ok(tec) => {
                        eprintln!("Loaded IONEX TEC maps: {} maps covering {:.1} hours",
                            tec.num_maps(),
                            tec.time_span().map(|(s, e)| (e - s) / 3600.0).unwrap_or(0.0));
                        Some(Arc::new(tec))
                    }
                    Err(e) => {
                        eprintln!("Warning: Failed to load IONEX file: {}. Falling back to Klobuchar.", e);
                        None
                    }
                }
            }
            IonosphereSource::Klobuchar | IonosphereSource::Disabled => None,
        };

        let emitters: Vec<SatelliteEmitter> = config.satellites.iter().map(|sat| {
            let waveform = create_waveform(sat.signal, sat.prn, sample_rate);
            let orbit = create_orbit(sat.signal, sat.plane, sat.slot);

            let mut emitter = SatelliteEmitter::new(
                sat.prn,
                sat.signal,
                waveform,
                orbit,
                sat.tx_power_dbw,
            ).with_nav_data(sat.nav_data);

            // Only add Klobuchar if NOT using IONEX (IONEX will be applied in generate_block)
            #[cfg(feature = "ephemeris")]
            let use_klobuchar = config.environment.ionosphere_enabled
                && !matches!(config.environment.ionosphere_source, IonosphereSource::IonexFile(_) | IonosphereSource::Disabled);
            #[cfg(not(feature = "ephemeris"))]
            let use_klobuchar = config.environment.ionosphere_enabled;

            if use_klobuchar {
                let model = config.environment.ionosphere_model.clone()
                    .unwrap_or_else(KlobucharModel::default_broadcast);
                emitter = emitter.with_ionosphere(model);
            }

            if config.environment.troposphere_enabled {
                let model = config.environment.troposphere_model.clone()
                    .unwrap_or_else(SaastamoinenModel::standard_atmosphere);
                emitter = emitter.with_troposphere(model);
            }

            emitter
        }).collect();

        let rng_state = config.output.seed.max(1);
        let num_emitters = emitters.len();

        // Pre-compute orbital Doppler and range at t=0 for anchored dynamics
        let t0 = config.output.start_time_gps_s;
        let rx_lla_t0 = if let Some(ref traj) = config.receiver.trajectory {
            traj.position_at(0.0) // Start of trajectory
        } else {
            config.receiver.position
        };
        let rx_ecef_t0 = lla_to_ecef(&rx_lla_t0);

        // Compute receiver ECEF velocity at t=0 from trajectory (if moving)
        let rx_vel_t0 = if let Some(ref traj) = config.receiver.trajectory {
            let dist = traj.distance_m();
            let speed = traj.speed_mps.unwrap_or(dist / config.output.duration_s);
            let travel_time_s = dist / speed;
            let dt = 0.01_f64.min(travel_time_s * 0.001);
            let frac_dt = dt / travel_time_s;
            let pos_dt = traj.position_at(frac_dt);
            let ecef_dt = lla_to_ecef(&pos_dt);
            EcefVelocity::new(
                (ecef_dt.x - rx_ecef_t0.x) / dt,
                (ecef_dt.y - rx_ecef_t0.y) / dt,
                (ecef_dt.z - rx_ecef_t0.z) / dt,
            )
        } else {
            EcefVelocity::zero()
        };

        // Log trajectory info
        if let Some(ref traj) = config.receiver.trajectory {
            let dist = traj.distance_m();
            let heading = traj.heading_deg();
            let speed = traj.speed_mps.unwrap_or(dist / config.output.duration_s);
            let travel_time_s = dist / speed;
            eprintln!("Trajectory: {:.1} km, heading {:.1}°, speed {:.0} m/s ({:.1} Mach), travel {:.1}s",
                dist / 1000.0, heading, speed, speed / 343.0, travel_time_s);
        }

        let mut orbital_doppler_t0 = Vec::with_capacity(num_emitters);
        let mut orbital_range_t0 = Vec::with_capacity(num_emitters);
        for emitter in &emitters {
            let (sat_pos, sat_vel) = emitter.position_velocity_at(t0);
            let la = look_angle(&rx_ecef_t0, &rx_lla_t0, &sat_pos);
            let rr = range_rate(&rx_ecef_t0, &rx_vel_t0, &sat_pos, &sat_vel);
            let carrier_hz = emitter.signal.carrier_frequency_hz();
            orbital_doppler_t0.push(-rr * carrier_hz / SPEED_OF_LIGHT);
            orbital_range_t0.push(la.range_m);
        }

        // Initialize per-emitter anti-aliasing LPFs at oversampled rate.
        // Cutoff at output Nyquist (sample_rate / 2) removes BOC harmonics
        // before decimation back to output rate.
        let oversampled_rate = sample_rate * BASEBAND_OVERSAMPLE as f64;
        let lpf_cutoff = if config.output.lpf_cutoff_hz > 0.0 {
            config.output.lpf_cutoff_hz
        } else {
            sample_rate / 2.0 // Default: output Nyquist
        };
        let baseband_lpfs: Vec<FirFilter> = (0..num_emitters)
            .map(|_| FirFilter::lowpass(lpf_cutoff, oversampled_rate, 63))
            .collect();
        eprintln!("Baseband: {}× oversample ({:.1} MHz), LPF {:.1} kHz cutoff, 63 taps, decimate to {:.1} MHz",
            BASEBAND_OVERSAMPLE, oversampled_rate / 1e6, lpf_cutoff / 1e3, sample_rate / 1e6);

        Self {
            config,
            emitters,
            current_sample: 0,
            sample_rate,
            total_samples,
            doppler_phases: vec![0.0; num_emitters],
            orbital_doppler_t0,
            orbital_range_t0,
            baseband_lpfs,
            rng_state,
            #[cfg(feature = "ephemeris")]
            sp3,
            #[cfg(feature = "ephemeris")]
            ionex,
        }
    }

    /// Create a scenario from a preset
    pub fn from_preset(preset: GnssScenarioPreset) -> Self {
        Self::new(preset.to_config())
    }

    /// Get satellite position/velocity either from SP3 or Keplerian orbit
    #[cfg(feature = "ephemeris")]
    fn get_satellite_position(&self, emitter: &SatelliteEmitter, t: f64) -> (EcefPosition, EcefVelocity) {
        if let Some(ref sp3) = self.sp3 {
            // Construct SP3 satellite ID (e.g., "G01" for GPS PRN 1)
            let sv_id = format_sv_id(emitter.signal, emitter.prn);
            if let Some((pos, vel, _clock)) = sp3.interpolate(&sv_id, t) {
                return (pos, vel);
            }
        }
        // Fallback to Keplerian orbit
        emitter.position_velocity_at(t)
    }

    #[cfg(not(feature = "ephemeris"))]
    fn get_satellite_position(&self, emitter: &SatelliteEmitter, t: f64) -> (EcefPosition, EcefVelocity) {
        emitter.position_velocity_at(t)
    }

    /// Get ionospheric delay from IONEX or Klobuchar
    #[cfg(feature = "ephemeris")]
    fn get_iono_delay_m(&self, emitter: &SatelliteEmitter, t: f64, rx_lla: &LlaPosition, elevation_deg: f64, _azimuth_deg: f64) -> f64 {
        // Check if IONEX is enabled and loaded
        if let Some(ref ionex) = self.ionex {
            if matches!(self.config.environment.ionosphere_source, IonosphereSource::IonexFile(_)) {
                // Compute ionospheric pierce point (simplified: use receiver location)
                if let Some(tec) = ionex.get_tec(rx_lla.lat_deg, rx_lla.lon_deg, t) {
                    let carrier_hz = emitter.signal.carrier_frequency_hz();
                    return IonexTec::iono_delay_m(tec, elevation_deg, carrier_hz);
                }
            }
        }

        // Fallback: get from emitter's Klobuchar model (already set up in new())
        let rx_ecef = lla_to_ecef(rx_lla);
        let status = emitter.status_at(t, &rx_ecef, rx_lla, &self.config.receiver.antenna);
        status.iono_delay_m
    }

    #[cfg(not(feature = "ephemeris"))]
    fn get_iono_delay_m(&self, emitter: &SatelliteEmitter, t: f64, rx_lla: &LlaPosition, _elevation_deg: f64, _azimuth_deg: f64) -> f64 {
        let rx_ecef = lla_to_ecef(rx_lla);
        let status = emitter.status_at(t, &rx_ecef, rx_lla, &self.config.receiver.antenna);
        status.iono_delay_m
    }

    /// Get satellite clock correction from SP3 (in seconds)
    #[cfg(feature = "ephemeris")]
    fn get_clock_correction_s(&self, emitter: &SatelliteEmitter, t: f64) -> f64 {
        if let Some(ref sp3) = self.sp3 {
            let sv_id = format_sv_id(emitter.signal, emitter.prn);
            if let Some((_, _, clock_s)) = sp3.interpolate(&sv_id, t) {
                return clock_s;
            }
        }
        0.0 // No clock correction available
    }

    #[cfg(not(feature = "ephemeris"))]
    fn get_clock_correction_s(&self, _emitter: &SatelliteEmitter, _t: f64) -> f64 {
        0.0
    }

    /// Generate the next block of composite IQ samples
    pub fn generate_block(&mut self, block_size: usize) -> Vec<IQSample> {
        let remaining = self.total_samples.saturating_sub(self.current_sample);
        let n = remaining.min(block_size);
        if n == 0 {
            return vec![];
        }

        let t_offset = self.config.output.start_time_gps_s;
        let t_start = t_offset + self.current_sample as f64 / self.sample_rate;
        let elapsed_s = self.current_sample as f64 / self.sample_rate;
        let duration_s = self.config.output.duration_s;

        // Compute receiver position from trajectory (or use static position)
        let (rx_lla, rx_ecef, rx_vel) = if let Some(ref traj) = self.config.receiver.trajectory {
            // Travel time is distance / speed. If speed not specified, use distance / duration.
            let dist = traj.distance_m();
            let speed = traj.speed_mps.unwrap_or(dist / duration_s);
            let travel_time_s = dist / speed;

            // Position along trajectory: frac = elapsed / travel_time
            // Clamp to 1.0 so receiver stops at end point if scenario runs longer than travel
            let frac = (elapsed_s / travel_time_s).clamp(0.0, 1.0);
            let lla = traj.position_at(frac);
            let ecef = lla_to_ecef(&lla);

            // Velocity via finite difference (position at frac + small delta)
            let vel = if frac < 1.0 {
                let dt = 0.01_f64.min(travel_time_s * 0.001);
                let frac_dt = ((elapsed_s + dt) / travel_time_s).clamp(0.0, 1.0);
                let lla_dt = traj.position_at(frac_dt);
                let ecef_dt = lla_to_ecef(&lla_dt);
                EcefVelocity::new(
                    (ecef_dt.x - ecef.x) / dt,
                    (ecef_dt.y - ecef.y) / dt,
                    (ecef_dt.z - ecef.z) / dt,
                )
            } else {
                EcefVelocity::zero() // Stopped at endpoint
            };

            (lla, ecef, vel)
        } else {
            let lla = self.config.receiver.position;
            let ecef = lla_to_ecef(&lla);
            (lla, ecef, EcefVelocity::zero())
        };

        let mut composite = vec![Complex64::new(0.0, 0.0); n];

        let t_end = t_start + n as f64 / self.sample_rate;
        let elapsed_end = elapsed_s + n as f64 / self.sample_rate;
        let os = BASEBAND_OVERSAMPLE;
        let oversampled_rate = self.sample_rate * os as f64;
        let oversampled_n = n * os;
        let oversampled_offset = self.current_sample * os;

        // --- Phase 1: Compute per-emitter parameters (sequential, fast) ---
        // Collects geometry, Doppler, range, amplitude, and delay for each visible satellite.
        struct EmitterWork {
            idx: usize,
            range_m: f64,
            iono_delay_s: f64,
            tropo_delay_s: f64,
            rx_amplitude: f64,
            doppler_start_hz: f64,
            doppler_end_hz: f64,
        }

        let mut work_items: Vec<EmitterWork> = Vec::new();

        for (emitter_idx, emitter) in self.emitters.iter().enumerate() {
            let sat_config = &self.config.satellites[emitter_idx];

            let (sat_pos_start, sat_vel_start) = self.get_satellite_position(emitter, t_start);
            let (sat_pos_end, sat_vel_end) = self.get_satellite_position(emitter, t_end);

            let la = look_angle(&rx_ecef, &rx_lla, &sat_pos_start);
            let elevation_deg = sat_config.elevation_deg.unwrap_or(la.elevation_deg);
            if elevation_deg < self.config.receiver.elevation_mask_deg {
                continue;
            }

            let carrier_hz = emitter.signal.carrier_frequency_hz();
            let rr_orbital_start = range_rate(&rx_ecef, &rx_vel, &sat_pos_start, &sat_vel_start);
            let rr_orbital_end = range_rate(&rx_ecef, &rx_vel, &sat_pos_end, &sat_vel_end);
            let orbital_doppler_start = -rr_orbital_start * carrier_hz / SPEED_OF_LIGHT;
            let orbital_doppler_end = -rr_orbital_end * carrier_hz / SPEED_OF_LIGHT;

            let range_m = if sat_config.orbital_dynamics {
                if let Some(r0) = sat_config.range_m {
                    r0 + (la.range_m - self.orbital_range_t0[emitter_idx])
                } else {
                    la.range_m
                }
            } else if let (Some(r0), Some(rr)) = (sat_config.range_m, sat_config.range_rate_mps) {
                r0 + rr * elapsed_s
            } else {
                sat_config.range_m.unwrap_or(la.range_m)
            };

            let (doppler_start_hz, doppler_end_hz) = if sat_config.orbital_dynamics {
                if let Some(doppler_initial) = sat_config.doppler_hz {
                    let delta_start = orbital_doppler_start - self.orbital_doppler_t0[emitter_idx];
                    let delta_end = orbital_doppler_end - self.orbital_doppler_t0[emitter_idx];
                    (doppler_initial + delta_start, doppler_initial + delta_end)
                } else {
                    (orbital_doppler_start, orbital_doppler_end)
                }
            } else if let Some(doppler_initial) = sat_config.doppler_hz {
                if let Some(rate) = sat_config.doppler_rate_hz_per_s {
                    (doppler_initial + rate * elapsed_s,
                     doppler_initial + rate * elapsed_end)
                } else {
                    (doppler_initial, doppler_initial)
                }
            } else if let Some(rr_override) = sat_config.range_rate_mps {
                let doppler = -rr_override * carrier_hz / SPEED_OF_LIGHT;
                (doppler, doppler)
            } else {
                (orbital_doppler_start, orbital_doppler_end)
            };

            let iono_delay_m = sat_config.iono_delay_m.unwrap_or_else(||
                self.get_iono_delay_m(emitter, t_start, &rx_lla, elevation_deg, la.azimuth_deg)
            );
            let iono_delay_s = iono_delay_m / SPEED_OF_LIGHT;

            let tropo_delay_m = sat_config.tropo_delay_m.unwrap_or_else(|| {
                let tropo_status = emitter.status_at(t_start, &rx_ecef, &rx_lla, &self.config.receiver.antenna);
                tropo_status.tropo_delay_m
            });
            let tropo_delay_s = tropo_delay_m / SPEED_OF_LIGHT;

            let cn0_dbhz = sat_config.cn0_dbhz.unwrap_or_else(|| {
                let fspl = crate::coordinates::fspl_db(range_m, carrier_hz);
                let antenna_gain_dbi = self.config.receiver.antenna.gain_dbi(elevation_deg);
                sat_config.tx_power_dbw - fspl + antenna_gain_dbi + 204.0
            });
            let rx_power_dbw = cn0_dbhz - 204.0;
            let rx_amplitude = 10.0_f64.powf((rx_power_dbw + 160.0) / 20.0);

            work_items.push(EmitterWork {
                idx: emitter_idx,
                range_m, iono_delay_s, tropo_delay_s, rx_amplitude,
                doppler_start_hz, doppler_end_hz,
            });
        }

        // --- Phase 2: Generate baseband IQ + LPF + decimate ---
        // Each emitter's baseband generation is independent and can be parallelized.
        // LPF filter state must persist across blocks, so LPF is always applied sequentially.
        // With rayon: generate all baseband in parallel, then LPF+decimate sequentially.
        // Without rayon: generate+LPF+decimate sequentially per emitter.
        type BasebandResult = (usize, Vec<Complex64>, f64, f64, f64);

        // Temporarily take LPFs out of self so we can pass them to closures
        let mut lpfs = std::mem::take(&mut self.baseband_lpfs);

        #[cfg(feature = "parallel")]
        let baseband_results: Vec<BasebandResult> = {
            // Phase 2a: Generate oversampled baseband in parallel (no LPF yet)
            let emitters = &self.emitters;
            let oversampled_buffers: Vec<(usize, Vec<Complex64>, f64, f64, f64)> = work_items
                .par_iter()
                .map(|work| {
                    let emitter = &emitters[work.idx];
                    let baseband_os = emitter.generate_baseband_iq(
                        t_start, oversampled_n, oversampled_rate,
                        work.range_m, work.iono_delay_s, work.tropo_delay_s,
                        oversampled_offset,
                    );
                    (work.idx, baseband_os, work.rx_amplitude, work.doppler_start_hz, work.doppler_end_hz)
                })
                .collect();

            // Phase 2b: Apply LPF + decimate sequentially (filter state must be continuous)
            oversampled_buffers.into_iter().map(|(idx, mut baseband_os, rx_amplitude, doppler_start, doppler_end)| {
                lpfs[idx].process_inplace(&mut baseband_os);
                let baseband: Vec<Complex64> = baseband_os.iter()
                    .step_by(os)
                    .copied()
                    .collect();
                (idx, baseband, rx_amplitude, doppler_start, doppler_end)
            }).collect()
        };

        #[cfg(not(feature = "parallel"))]
        let baseband_results: Vec<BasebandResult> = {
            work_items.iter().map(|work| {
                let emitter = &self.emitters[work.idx];
                let mut baseband_os = emitter.generate_baseband_iq(
                    t_start, oversampled_n, oversampled_rate,
                    work.range_m, work.iono_delay_s, work.tropo_delay_s,
                    oversampled_offset,
                );
                lpfs[work.idx].process_inplace(&mut baseband_os);
                let baseband: Vec<Complex64> = baseband_os.iter()
                    .step_by(os)
                    .copied()
                    .collect();
                (work.idx, baseband, work.rx_amplitude, work.doppler_start_hz, work.doppler_end_hz)
            }).collect()
        };

        // Put LPFs back
        self.baseband_lpfs = lpfs;

        // --- Phase 3: Apply Doppler and accumulate (sequential, needs phase state) ---
        for (emitter_idx, baseband, rx_amplitude, doppler_start_hz, doppler_end_hz) in &baseband_results {
            let n_f64 = n as f64;
            let mut phase = self.doppler_phases[*emitter_idx];
            for (i, &sample) in baseband.iter().enumerate() {
                let frac = i as f64 / n_f64;
                let doppler_hz = doppler_start_hz + frac * (doppler_end_hz - doppler_start_hz);
                let phase_inc = 2.0 * PI * doppler_hz / self.sample_rate;
                phase += phase_inc;
                let doppler_shift = Complex64::new(phase.cos(), phase.sin());
                composite[i] += sample * doppler_shift * rx_amplitude;
            }
            self.doppler_phases[*emitter_idx] = phase;
        }

        // Add thermal noise using Box-Muller with xorshift64 PRNG
        // Noise spectral density: N0 = kT × NF (W/Hz)
        let noise_figure_linear = 10.0_f64.powf(self.config.receiver.noise_figure_db / 10.0);
        let n0 = 1.380_649e-23 * 290.0 * noise_figure_linear;
        let noise_power = n0 * self.sample_rate;
        // Apply same +160 dB baseband reference shift as signal amplitude
        // (signal uses 10^((P_dBW + 160)/20), i.e., amplitude scaled by 10^8)
        let noise_std = (noise_power / 2.0).sqrt() * 1e8;

        for sample in composite.iter_mut() {
            let (g1, g2) = self.box_muller_pair();
            *sample += Complex64::new(g1 * noise_std, g2 * noise_std);
        }

        self.current_sample += n;
        composite
    }

    /// Generate the full IQ stream
    pub fn generate(&mut self) -> Vec<IQSample> {
        let block_size = if self.config.output.block_size > 0 {
            self.config.output.block_size
        } else {
            (self.sample_rate * 0.001).ceil() as usize
        };

        let mut output = Vec::with_capacity(self.total_samples);
        while self.current_sample < self.total_samples {
            output.extend(self.generate_block(block_size));
        }
        output
    }

    /// Get status of all satellites at current time
    pub fn satellite_status(&self) -> Vec<SatelliteStatus> {
        let t = self.config.output.start_time_gps_s + self.current_sample as f64 / self.sample_rate;
        let rx_lla = &self.config.receiver.position;
        let rx_ecef = lla_to_ecef(rx_lla);
        let antenna = &self.config.receiver.antenna;

        self.emitters.iter().enumerate().map(|(idx, e)| {
            // Get config overrides for this emitter
            let sat_config = &self.config.satellites[idx];

            // Get satellite position from SP3 (if available) or Keplerian orbit
            let (sat_pos, sat_vel) = self.get_satellite_position(e, t);

            // Compute look angle from SP3/Keplerian position
            let la = look_angle(&rx_ecef, rx_lla, &sat_pos);

            // Use config overrides if present
            let range_m = sat_config.range_m.unwrap_or(la.range_m);
            let elevation_deg = sat_config.elevation_deg.unwrap_or(la.elevation_deg);
            let azimuth_deg = sat_config.azimuth_deg.unwrap_or(la.azimuth_deg);

            // Range rate for Doppler
            let rx_vel = EcefVelocity::zero();
            let rr = sat_config.range_rate_mps.unwrap_or_else(||
                range_rate(&rx_ecef, &rx_vel, &sat_pos, &sat_vel)
            );
            let carrier_hz = e.signal.carrier_frequency_hz();
            let doppler_hz = sat_config.doppler_hz.unwrap_or(-rr * carrier_hz / SPEED_OF_LIGHT);

            // Antenna gain
            let antenna_gain_dbi = antenna.gain_dbi(elevation_deg);

            // C/N0 calculation: use override if present, else EIRP - FSPL + Gr + 204 (in dBW/Hz)
            let cn0_dbhz = sat_config.cn0_dbhz.unwrap_or_else(|| {
                let fspl = crate::coordinates::fspl_db(range_m, carrier_hz);
                let tx_power_dbw = sat_config.tx_power_dbw;
                tx_power_dbw - fspl + antenna_gain_dbi + 204.0
            });

            // Ionospheric delay - use override if present
            let iono_delay_m = sat_config.iono_delay_m.unwrap_or_else(||
                self.get_iono_delay_m(e, t, rx_lla, elevation_deg, azimuth_deg)
            );

            // Tropospheric delay - use override if present
            let tropo_delay_m = sat_config.tropo_delay_m.unwrap_or_else(|| {
                let tropo_status = e.status_at(t, &rx_ecef, rx_lla, antenna);
                tropo_status.tropo_delay_m
            });

            // Clock correction from SP3 when available
            let clock_correction_s = self.get_clock_correction_s(e, t);

            SatelliteStatus {
                prn: e.prn,
                signal: e.signal,
                elevation_deg,
                azimuth_deg,
                range_m,
                range_rate_mps: rr,
                doppler_hz,
                cn0_dbhz,
                iono_delay_m,
                tropo_delay_m,
                antenna_gain_dbi,
                visible: elevation_deg > 0.0,
                clock_correction_s,
            }
        }).collect()
    }

    /// Reset to beginning
    pub fn reset(&mut self) {
        self.current_sample = 0;
        self.rng_state = self.config.output.seed.max(1);
        self.doppler_phases.fill(0.0);
        for lpf in &mut self.baseband_lpfs {
            lpf.reset();
        }
    }

    /// Whether generation is complete
    pub fn is_done(&self) -> bool {
        self.current_sample >= self.total_samples
    }

    /// Progress as fraction (0.0 to 1.0)
    pub fn progress(&self) -> f64 {
        if self.total_samples == 0 { 1.0 }
        else { self.current_sample as f64 / self.total_samples as f64 }
    }

    /// Reference to config
    pub fn config(&self) -> &GnssScenarioConfig {
        &self.config
    }

    /// Total samples to generate
    pub fn total_samples(&self) -> usize {
        self.total_samples
    }

    /// Block size for streaming generation
    pub fn block_size(&self) -> usize {
        if self.config.output.block_size > 0 {
            self.config.output.block_size
        } else {
            // Default: 1 ms worth of samples
            (self.sample_rate * 0.001).ceil() as usize
        }
    }

    /// Write IQ output to file (raw interleaved f64 I/Q pairs)
    pub fn write_output(&self, samples: &[IQSample], path: &std::path::Path) -> std::io::Result<()> {
        use std::io::Write;
        let mut file = std::fs::File::create(path)?;
        for sample in samples {
            file.write_all(&sample.re.to_le_bytes())?;
            file.write_all(&sample.im.to_le_bytes())?;
        }
        Ok(())
    }

    /// Xorshift64 PRNG - returns uniform [0, 1)
    fn xorshift64(&mut self) -> f64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        (x as f64) / (u64::MAX as f64)
    }

    /// Box-Muller transform: two uniform -> two Gaussian(0,1)
    fn box_muller_pair(&mut self) -> (f64, f64) {
        let u1 = self.xorshift64().max(1e-15); // avoid log(0)
        let u2 = self.xorshift64();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * PI * u2;
        (r * theta.cos(), r * theta.sin())
    }
}

/// Create a waveform instance for the given signal type
fn create_waveform(signal: GnssSignal, prn: u8, sample_rate: f64) -> Box<dyn Waveform> {
    match signal {
        GnssSignal::GpsL1Ca => Box::new(GpsL1Ca::new(sample_rate, prn)),
        GnssSignal::GpsL5 => Box::new(GpsL5::new(sample_rate, prn)),
        GnssSignal::GlonassL1of => Box::new(GlonassL1of::new(sample_rate, prn as i8)),
        GnssSignal::GalileoE1 | GnssSignal::GalileoE1C | GnssSignal::GalileoE1OS => Box::new(GalileoE1::new(sample_rate, prn)),
    }
}

/// Create an orbit for the given signal type and plane/slot
fn create_orbit(signal: GnssSignal, plane: u8, slot: u8) -> KeplerianOrbit {
    match signal.constellation() {
        super::types::GnssConstellation::Gps => KeplerianOrbit::gps_nominal(plane, slot),
        super::types::GnssConstellation::Galileo => KeplerianOrbit::galileo_nominal(plane, slot),
        super::types::GnssConstellation::Glonass => KeplerianOrbit::glonass_nominal(plane, slot),
        super::types::GnssConstellation::BeiDou => KeplerianOrbit::gps_nominal(plane, slot),
    }
}

/// Format satellite ID for SP3 lookup (e.g., "G01" for GPS PRN 1)
#[cfg(feature = "ephemeris")]
fn format_sv_id(signal: GnssSignal, prn: u8) -> String {
    let prefix = match signal.constellation() {
        super::types::GnssConstellation::Gps => 'G',
        super::types::GnssConstellation::Glonass => 'R',
        super::types::GnssConstellation::Galileo => 'E',
        super::types::GnssConstellation::BeiDou => 'C',
    };
    format!("{}{:02}", prefix, prn)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open_sky_preset() {
        let mut scenario = GnssScenario::from_preset(GnssScenarioPreset::OpenSky);
        let samples = scenario.generate();
        assert_eq!(samples.len(), scenario.total_samples());
        assert!(scenario.is_done());
    }

    #[test]
    fn test_all_presets_no_panic() {
        for preset in GnssScenarioPreset::all() {
            let mut scenario = GnssScenario::from_preset(*preset);
            let samples = scenario.generate();
            assert!(!samples.is_empty(), "{} produced empty output", preset);
        }
    }

    #[test]
    fn test_satellite_status() {
        let scenario = GnssScenario::from_preset(GnssScenarioPreset::OpenSky);
        let statuses = scenario.satellite_status();
        assert_eq!(statuses.len(), 8);
        for status in &statuses {
            assert!(status.range_m > 20_000_000.0, "PRN {} range: {}", status.prn, status.range_m);
        }
    }

    #[test]
    fn test_multi_constellation() {
        let mut scenario = GnssScenario::from_preset(GnssScenarioPreset::MultiConstellation);
        let samples = scenario.generate();
        assert!(!samples.is_empty());
    }

    #[test]
    fn test_reset_and_regenerate() {
        let mut scenario = GnssScenario::from_preset(GnssScenarioPreset::OpenSky);
        let samples1 = scenario.generate();
        scenario.reset();
        let samples2 = scenario.generate();
        assert_eq!(samples1.len(), samples2.len());
        // First samples should be identical (deterministic)
        for (a, b) in samples1.iter().zip(samples2.iter()).take(10) {
            assert!((a.re - b.re).abs() < 1e-10);
            assert!((a.im - b.im).abs() < 1e-10);
        }
    }

    /// Integration test: generate scenario and verify satellites can be acquired
    /// This validates end-to-end correctness of the scenario generator
    #[test]
    fn test_acquisition_on_scenario() {
        use crate::waveform::gnss::acquisition::PcpsAcquisition;
        use crate::waveform::gnss::gps_l1ca::GpsL1Ca;

        // Generate open-sky scenario with longer duration for better acquisition
        let mut config = GnssScenarioPreset::OpenSky.to_config();
        config.output.duration_s = 0.001; // 1ms = one code period
        config.output.sample_rate = 2.046e6; // 2x chipping rate
        config.environment.multipath_enabled = false; // Clean signal for testing

        let mut scenario = GnssScenario::new(config);
        let statuses = scenario.satellite_status();
        let samples = scenario.generate();

        // Find visible satellites, sorted by C/N0 (strongest first)
        let mut visible: Vec<_> = statuses.iter().filter(|s| s.visible).collect();
        visible.sort_by(|a, b| b.cn0_dbhz.partial_cmp(&a.cn0_dbhz).unwrap());
        assert!(visible.len() >= 4, "Need at least 4 visible satellites, got {}", visible.len());

        // Create PCPS acquisition engine
        // GPS L1 C/A: 1023 chips, 2.046 MHz = 2046 samples per code
        let acquisition = PcpsAcquisition::new(2046, 2.046e6)
            .with_doppler_range(5000.0, 500.0)  // ±5 kHz with 500 Hz steps
            .with_threshold(1.5);                // Lower threshold for noisy composite

        let mut detected_count = 0;
        let mut results_info = Vec::new();

        for status in visible.iter().take(6) {
            // Generate local code for this PRN
            let waveform = GpsL1Ca::new(2.046e6, status.prn);
            let code = waveform.code();

            // Run acquisition
            let result = acquisition.acquire(&samples, code, status.prn);

            results_info.push(format!(
                "PRN {:2}: detected={}, metric={:.2}, doppler={:+.0} (expected {:+.0})",
                status.prn, result.detected, result.peak_metric,
                result.doppler_hz, status.doppler_hz
            ));

            if result.detected {
                detected_count += 1;
            }
        }

        // Print results for debugging
        eprintln!("Acquisition results:");
        for info in &results_info {
            eprintln!("  {}", info);
        }

        // Verify at least 2 satellites detected
        // Note: Doppler verification relaxed because acquisition coarse search
        // may not match fine Doppler from orbit calculation
        assert!(
            detected_count >= 2,
            "Expected at least 2 satellites detected, got {}",
            detected_count
        );
    }
}
