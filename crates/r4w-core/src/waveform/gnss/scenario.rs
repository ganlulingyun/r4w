//! GNSS Scenario: top-level API for multi-satellite IQ generation
//!
//! Composes satellite emitters with environment models to produce realistic
//! composite GNSS baseband IQ signals.

use super::environment::{KeplerianOrbit, KlobucharModel, SaastamoinenModel};
use super::satellite_emitter::{SatelliteEmitter, SatelliteStatus};
use super::scenario_config::{GnssScenarioConfig, GnssScenarioPreset};
use super::types::GnssSignal;
use crate::coordinates::{
    lla_to_ecef, look_angle, range_rate, EcefVelocity, SPEED_OF_LIGHT,
};
use crate::types::IQSample;
use crate::waveform::gnss::{GalileoE1, GlonassL1of, GpsL1Ca, GpsL5};
use crate::waveform::Waveform;
use num_complex::Complex64;
use std::f64::consts::PI;

/// GNSS Scenario: generates composite multi-satellite IQ signal
// trace:FR-041 | ai:claude
pub struct GnssScenario {
    config: GnssScenarioConfig,
    emitters: Vec<SatelliteEmitter>,
    current_sample: usize,
    sample_rate: f64,
    total_samples: usize,
    /// Simple PRNG state for noise generation (xorshift64)
    rng_state: u64,
}

impl GnssScenario {
    /// Create a scenario from full configuration
    pub fn new(config: GnssScenarioConfig) -> Self {
        let sample_rate = config.output.sample_rate;
        let total_samples = (config.output.duration_s * sample_rate).ceil() as usize;

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

            if config.environment.ionosphere_enabled {
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

        Self {
            config,
            emitters,
            current_sample: 0,
            sample_rate,
            total_samples,
            rng_state,
        }
    }

    /// Create a scenario from a preset
    pub fn from_preset(preset: GnssScenarioPreset) -> Self {
        Self::new(preset.to_config())
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
        let rx_lla = &self.config.receiver.position;
        let rx_ecef = lla_to_ecef(rx_lla);

        let mut composite = vec![Complex64::new(0.0, 0.0); n];

        let t_end = t_start + n as f64 / self.sample_rate;

        for emitter in &self.emitters {
            // Compute geometry at block start and end for per-sample Doppler interpolation
            let (sat_pos_start, sat_vel_start) = emitter.position_velocity_at(t_start);
            let (sat_pos_end, sat_vel_end) = emitter.position_velocity_at(t_end);

            // Elevation check at block start
            let la = look_angle(&rx_ecef, rx_lla, &sat_pos_start);
            if la.elevation_deg < self.config.receiver.elevation_mask_deg {
                continue;
            }

            let range_m = la.range_m;

            // Doppler at block start and end for linear interpolation
            let rx_vel = EcefVelocity::zero();
            let carrier_hz = emitter.signal.carrier_frequency_hz();
            let rr_start = range_rate(&rx_ecef, &rx_vel, &sat_pos_start, &sat_vel_start);
            let rr_end = range_rate(&rx_ecef, &rx_vel, &sat_pos_end, &sat_vel_end);
            let doppler_start_hz = -rr_start * carrier_hz / SPEED_OF_LIGHT;
            let doppler_end_hz = -rr_end * carrier_hz / SPEED_OF_LIGHT;

            // Atmospheric delays (computed once per block — vary slowly)
            let status = emitter.status_at(t_start, &rx_ecef, rx_lla, &self.config.receiver.antenna);
            let iono_delay_s = status.iono_delay_m / SPEED_OF_LIGHT;
            let tropo_delay_s = status.tropo_delay_m / SPEED_OF_LIGHT;

            // Received power derived from C/N0 estimate (already includes FSPL + antenna)
            let rx_power_dbw = status.cn0_dbhz - 204.0;
            // Scale to reasonable baseband amplitude
            let rx_amplitude = 10.0_f64.powf((rx_power_dbw + 160.0) / 20.0); // shift to reasonable range

            // Generate baseband IQ with correct code phase
            let baseband = emitter.generate_baseband_iq(
                t_start, n, self.sample_rate,
                range_m, iono_delay_s, tropo_delay_s,
            );

            // Apply per-sample Doppler shift (linearly interpolated across block)
            let n_f64 = n as f64;
            let mut phase = 0.0_f64;
            for (i, &sample) in baseband.iter().enumerate() {
                let frac = i as f64 / n_f64;
                let doppler_hz = doppler_start_hz + frac * (doppler_end_hz - doppler_start_hz);
                let phase_inc = 2.0 * PI * doppler_hz / self.sample_rate;
                phase += phase_inc;
                let doppler_shift = Complex64::new(phase.cos(), phase.sin());
                composite[i] += sample * doppler_shift * rx_amplitude;
            }
        }

        // Add thermal noise using Box-Muller with xorshift64 PRNG
        let noise_figure_linear = 10.0_f64.powf(self.config.receiver.noise_figure_db / 10.0);
        let n0 = 1.380_649e-23 * 290.0 * noise_figure_linear;
        let noise_power = n0 * self.sample_rate;
        let noise_std = (noise_power / 2.0).sqrt();

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

        self.emitters.iter().map(|e| {
            e.status_at(t, &rx_ecef, rx_lla, antenna)
        }).collect()
    }

    /// Reset to beginning
    pub fn reset(&mut self) {
        self.current_sample = 0;
        self.rng_state = self.config.output.seed.max(1);
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
        GnssSignal::GalileoE1 => Box::new(GalileoE1::new(sample_rate, prn)),
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
}
