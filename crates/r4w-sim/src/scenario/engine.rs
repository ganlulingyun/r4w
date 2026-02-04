//! Scenario engine: multi-emitter IQ composition
//!
//! Computes per-emitter geometry (range, Doppler, path loss), generates baseband IQ,
//! applies per-emitter channel effects, sums to composite, and adds receiver noise.

use super::config::ScenarioConfig;
use super::emitter::{Emitter, EmitterState};
use super::trajectory::{Trajectory, TrajectoryState};
use r4w_core::coordinates::{range_rate, fspl_db, SPEED_OF_LIGHT};
use r4w_core::types::IQSample;
use num_complex::Complex64;
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;

/// Per-emitter status at a given time
#[derive(Debug, Clone)]
pub struct EmitterStatus {
    pub id: String,
    pub range_m: f64,
    pub doppler_hz: f64,
    pub path_loss_db: f64,
    pub received_power_dbm: f64,
    pub elevation_deg: Option<f64>,
    pub azimuth_deg: Option<f64>,
    pub active: bool,
}

/// The scenario engine composes multiple emitter signals into a single IQ stream
// trace:FR-039 | ai:claude
pub struct ScenarioEngine {
    config: ScenarioConfig,
    emitters: Vec<Box<dyn Emitter>>,
    trajectory: Trajectory,
    rng: StdRng,
    current_sample: usize,
    /// Per-emitter carrier phase accumulators for continuous Doppler
    carrier_phases: Vec<f64>,
}

impl ScenarioEngine {
    pub fn new(
        config: ScenarioConfig,
        emitters: Vec<Box<dyn Emitter>>,
        trajectory: Trajectory,
    ) -> Self {
        let num_emitters = emitters.len();
        let rng = StdRng::seed_from_u64(config.seed);
        Self {
            config,
            emitters,
            trajectory,
            rng,
            current_sample: 0,
            carrier_phases: vec![0.0; num_emitters],
        }
    }

    /// Generate the next block of composite IQ samples
    pub fn generate_block(&mut self) -> Vec<IQSample> {
        let remaining = self.config.total_samples().saturating_sub(self.current_sample);
        let block_size = remaining.min(self.config.block_size);
        if block_size == 0 {
            return vec![];
        }

        let t_start = self.current_sample as f64 / self.config.sample_rate;

        // Get receiver state at block midpoint for geometry calculations
        let t_mid = t_start + (block_size as f64 / 2.0) / self.config.sample_rate;
        let rx_state = self.trajectory.state_at(t_mid);

        // Composite output buffer
        let mut composite = vec![Complex64::new(0.0, 0.0); block_size];

        // Process each emitter
        for (idx, emitter) in self.emitters.iter().enumerate() {
            let em_state = emitter.state_at(t_mid);
            if !em_state.active {
                continue;
            }

            // Geometry
            let range_m = rx_state.position.distance_to(&em_state.position);
            let rr = range_rate(
                &rx_state.position,
                &rx_state.velocity,
                &em_state.position,
                &em_state.velocity,
            );
            let carrier_hz = emitter.carrier_frequency_hz();
            let doppler_hz = -rr * carrier_hz / SPEED_OF_LIGHT;

            // Path loss
            let pl_db = fspl_db(range_m, carrier_hz);

            // Signal power: transmit power - path loss (in dBm)
            let rx_power_dbm = em_state.power_dbm - pl_db;
            let rx_amplitude = 10.0_f64.powf((rx_power_dbm - 30.0) / 20.0); // dBm to linear voltage

            // Generate baseband IQ from emitter
            let baseband = emitter.generate_iq(t_start, block_size, self.config.sample_rate);

            // Apply Doppler shift and amplitude scaling
            for (i, &sample) in baseband.iter().enumerate() {
                let t_sample = (self.current_sample + i) as f64 / self.config.sample_rate;
                // Accumulate phase continuously
                self.carrier_phases[idx] += 2.0 * PI * doppler_hz / self.config.sample_rate;
                // Wrap phase to prevent floating-point precision loss
                if self.carrier_phases[idx].abs() > 1e6 {
                    self.carrier_phases[idx] %= 2.0 * PI;
                }

                let doppler_shift = Complex64::new(
                    self.carrier_phases[idx].cos(),
                    self.carrier_phases[idx].sin(),
                );

                composite[i] += sample * doppler_shift * rx_amplitude;
            }
        }

        // Add receiver noise (AWGN)
        let noise_power = self.config.noise_power_linear();
        let noise_std = (noise_power / 2.0).sqrt(); // per I/Q component
        let normal = Normal::new(0.0, noise_std).unwrap();
        for sample in composite.iter_mut() {
            *sample += Complex64::new(
                normal.sample(&mut self.rng),
                normal.sample(&mut self.rng),
            );
        }

        self.current_sample += block_size;
        composite
    }

    /// Generate all blocks and return the full IQ stream
    pub fn generate_all(&mut self) -> Vec<IQSample> {
        let total = self.config.total_samples();
        let mut output = Vec::with_capacity(total);
        while self.current_sample < total {
            output.extend(self.generate_block());
        }
        output
    }

    /// Reset the engine to the beginning
    pub fn reset(&mut self) {
        self.current_sample = 0;
        self.carrier_phases = vec![0.0; self.emitters.len()];
        self.rng = StdRng::seed_from_u64(self.config.seed);
    }

    /// Get status of all emitters at time t
    pub fn emitter_status(&self, t: f64) -> Vec<EmitterStatus> {
        let rx_state = self.trajectory.state_at(t);
        let rx_lla = r4w_core::coordinates::ecef_to_lla(&rx_state.position);

        self.emitters.iter().map(|emitter| {
            let em_state = emitter.state_at(t);
            let range_m = rx_state.position.distance_to(&em_state.position);
            let rr = range_rate(
                &rx_state.position,
                &rx_state.velocity,
                &em_state.position,
                &em_state.velocity,
            );
            let carrier_hz = emitter.carrier_frequency_hz();
            let doppler_hz = -rr * carrier_hz / SPEED_OF_LIGHT;
            let pl_db = fspl_db(range_m, carrier_hz);
            let rx_power_dbm = em_state.power_dbm - pl_db;

            let la = r4w_core::coordinates::look_angle(
                &rx_state.position,
                &rx_lla,
                &em_state.position,
            );

            EmitterStatus {
                id: emitter.id(),
                range_m,
                doppler_hz,
                path_loss_db: pl_db,
                received_power_dbm: rx_power_dbm,
                elevation_deg: Some(la.elevation_deg),
                azimuth_deg: Some(la.azimuth_deg),
                active: em_state.active,
            }
        }).collect()
    }

    /// Reference to the config
    pub fn config(&self) -> &ScenarioConfig {
        &self.config
    }

    /// Current progress as fraction (0.0 to 1.0)
    pub fn progress(&self) -> f64 {
        let total = self.config.total_samples();
        if total == 0 { 1.0 } else { self.current_sample as f64 / total as f64 }
    }

    /// Whether generation is complete
    pub fn is_done(&self) -> bool {
        self.current_sample >= self.config.total_samples()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use r4w_core::coordinates::LlaPosition;

    /// Simple test emitter that generates a tone
    struct ToneEmitter {
        position: r4w_core::coordinates::EcefPosition,
        freq_hz: f64,
        power_dbm: f64,
    }

    impl Emitter for ToneEmitter {
        fn state_at(&self, _t: f64) -> EmitterState {
            EmitterState {
                position: self.position,
                velocity: r4w_core::coordinates::EcefVelocity::zero(),
                power_dbm: self.power_dbm,
                active: true,
            }
        }

        fn generate_iq(&self, t: f64, num_samples: usize, sample_rate: f64) -> Vec<IQSample> {
            (0..num_samples).map(|i| {
                let t_s = t + i as f64 / sample_rate;
                let phase = 2.0 * PI * 1000.0 * t_s; // 1 kHz tone
                Complex64::new(phase.cos(), phase.sin())
            }).collect()
        }

        fn carrier_frequency_hz(&self) -> f64 { self.freq_hz }
        fn nominal_power_dbm(&self) -> f64 { self.power_dbm }
        fn id(&self) -> String { "tone".to_string() }
    }

    #[test]
    fn test_engine_generates_samples() {
        let config = ScenarioConfig {
            duration_s: 0.001,
            sample_rate: 10000.0,
            center_frequency_hz: 1e9,
            block_size: 100,
            noise_floor_dbw_hz: -250.0, // very low noise for test
            seed: 42,
        };

        let pos = r4w_core::coordinates::lla_to_ecef(&LlaPosition::new(0.0, 0.0, 20_200_000.0));
        let emitter = ToneEmitter {
            position: pos,
            freq_hz: 1e9,
            power_dbm: 50.0,
        };

        let trajectory = Trajectory::Static {
            position: LlaPosition::new(0.0, 0.0, 0.0),
        };

        let mut engine = ScenarioEngine::new(config, vec![Box::new(emitter)], trajectory);
        let samples = engine.generate_all();
        assert_eq!(samples.len(), 10);
        assert!(engine.is_done());
    }
}
