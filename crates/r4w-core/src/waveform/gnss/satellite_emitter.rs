//! GNSS Satellite Emitter
//!
//! Wraps a GNSS waveform (e.g., GpsL1Ca, GalileoE1) with orbital mechanics
//! and atmospheric models to produce realistic satellite signals.
//! Implements the generic `Emitter` trait from r4w-sim.

use super::environment::{
    AntennaPattern, KeplerianOrbit, KlobucharModel, SaastamoinenModel,
};
use super::types::GnssSignal;
use crate::coordinates::{EcefPosition, EcefVelocity, LlaPosition};
use crate::types::IQSample;
use crate::waveform::Waveform;
use num_complex::Complex64;

/// Status of a satellite at a given instant
#[derive(Debug, Clone)]
pub struct SatelliteStatus {
    pub prn: u8,
    pub signal: GnssSignal,
    pub elevation_deg: f64,
    pub azimuth_deg: f64,
    pub range_m: f64,
    pub doppler_hz: f64,
    pub cn0_dbhz: f64,
    pub iono_delay_m: f64,
    pub tropo_delay_m: f64,
    pub antenna_gain_dbi: f64,
    pub visible: bool,
}

/// A GNSS satellite emitter with orbit and atmospheric models
// trace:FR-041 | ai:claude
pub struct SatelliteEmitter {
    /// PRN number
    pub prn: u8,
    /// Signal type
    pub signal: GnssSignal,
    /// Waveform generator (retained for potential future use in tracking/acquisition)
    _waveform: Box<dyn Waveform>,
    /// Orbital elements
    orbit: KeplerianOrbit,
    /// Transmit power in dBW
    tx_power_dbw: f64,
    /// Ionospheric model (optional)
    iono_model: Option<KlobucharModel>,
    /// Tropospheric model (optional)
    tropo_model: Option<SaastamoinenModel>,
    /// Include navigation data modulation
    nav_data: bool,
    /// Cached code chips for direct phase generation
    code: Vec<i8>,
    /// Code period in seconds
    code_period_s: f64,
    /// Chipping rate
    chipping_rate: f64,
}

impl SatelliteEmitter {
    /// Create a new satellite emitter
    pub fn new(
        prn: u8,
        signal: GnssSignal,
        waveform: Box<dyn Waveform>,
        orbit: KeplerianOrbit,
        tx_power_dbw: f64,
    ) -> Self {
        let code = generate_prn_code(signal, prn);
        let code_period_s = signal.code_period_s();
        let chipping_rate = signal.chipping_rate();

        Self {
            prn,
            signal,
            _waveform: waveform,
            orbit,
            tx_power_dbw,
            iono_model: None,
            tropo_model: None,
            nav_data: true,
            code,
            code_period_s,
            chipping_rate,
        }
    }

    /// Set ionospheric model
    pub fn with_ionosphere(mut self, model: KlobucharModel) -> Self {
        self.iono_model = Some(model);
        self
    }

    /// Set tropospheric model
    pub fn with_troposphere(mut self, model: SaastamoinenModel) -> Self {
        self.tropo_model = Some(model);
        self
    }

    /// Set navigation data modulation
    pub fn with_nav_data(mut self, enabled: bool) -> Self {
        self.nav_data = enabled;
        self
    }

    /// Get position and velocity at time t
    pub fn position_velocity_at(&self, t: f64) -> (EcefPosition, EcefVelocity) {
        self.orbit.position_velocity_at(t)
    }

    /// Compute satellite status relative to a receiver
    pub fn status_at(
        &self,
        t: f64,
        rx_pos: &EcefPosition,
        rx_lla: &LlaPosition,
        antenna: &AntennaPattern,
    ) -> SatelliteStatus {
        let (sat_pos, sat_vel) = self.orbit.position_velocity_at(t);
        let la = crate::coordinates::look_angle(rx_pos, rx_lla, &sat_pos);
        let range_m = la.range_m;
        let elevation_deg = la.elevation_deg;
        let azimuth_deg = la.azimuth_deg;

        // Range rate for Doppler
        let rx_vel = EcefVelocity::zero(); // simplified for status
        let rr = crate::coordinates::range_rate(rx_pos, &rx_vel, &sat_pos, &sat_vel);
        let doppler_hz = -rr * self.signal.carrier_frequency_hz() / crate::coordinates::SPEED_OF_LIGHT;

        // Atmospheric delays
        let iono_delay_m = self.iono_model.as_ref().map(|m| {
            let delay_s = m.delay_seconds(
                elevation_deg.to_radians(),
                azimuth_deg.to_radians(),
                rx_lla.lat_rad(),
                rx_lla.lon_rad(),
                t % 604800.0, // GPS week seconds
            );
            KlobucharModel::delay_to_range_m(delay_s)
        }).unwrap_or(0.0);

        let tropo_delay_m = self.tropo_model.as_ref().map(|m| {
            m.delay_meters(elevation_deg.to_radians())
        }).unwrap_or(0.0);

        let antenna_gain_dbi = antenna.gain_dbi(elevation_deg);

        // Approximate C/N0
        let fspl = crate::coordinates::fspl_db(range_m, self.signal.carrier_frequency_hz());
        let cn0_dbhz = self.tx_power_dbw + 30.0 // dBW to dBm
            - fspl
            + antenna_gain_dbi
            + 204.0; // -kT in dBW/Hz

        SatelliteStatus {
            prn: self.prn,
            signal: self.signal,
            elevation_deg,
            azimuth_deg,
            range_m,
            doppler_hz,
            cn0_dbhz,
            iono_delay_m,
            tropo_delay_m,
            antenna_gain_dbi,
            visible: elevation_deg > 0.0,
        }
    }

    /// Generate baseband IQ with correct code phase from geometric range
    ///
    /// Instead of generating and then delaying, we start the PRN code at the
    /// correct phase based on the pseudorange, avoiding wasted computation.
    pub fn generate_baseband_iq(
        &self,
        t: f64,
        num_samples: usize,
        sample_rate: f64,
        geometric_range_m: f64,
        iono_delay_s: f64,
        tropo_delay_s: f64,
    ) -> Vec<IQSample> {
        let total_delay_s = geometric_range_m / crate::coordinates::SPEED_OF_LIGHT
            + iono_delay_s
            + tropo_delay_s;

        // Code phase from total pseudorange
        let chips_delay = total_delay_s * self.chipping_rate;
        let code_length = self.code.len() as f64;
        let initial_code_phase = chips_delay % code_length;

        // Generate samples with correct code phase
        let samples_per_chip = sample_rate / self.chipping_rate;
        let mut output = Vec::with_capacity(num_samples);

        // Simple nav data: flip every N code periods
        let code_periods_per_bit = (1.0 / (self.signal.nav_data_rate_bps() * self.code_period_s)) as usize;

        for i in 0..num_samples {
            let t_sample = t + i as f64 / sample_rate;
            let chip_idx_f = initial_code_phase + i as f64 / samples_per_chip;
            let chip_idx = (chip_idx_f % code_length) as usize;

            let code_val = self.code[chip_idx.min(self.code.len() - 1)] as f64;

            // Navigation data bit (slow modulation)
            let nav_bit = if self.nav_data {
                let code_period_idx = (t_sample / self.code_period_s) as usize;
                let bit_idx = code_period_idx / code_periods_per_bit;
                // Simple deterministic nav data from PRN
                if (bit_idx + self.prn as usize) % 2 == 0 { 1.0 } else { -1.0 }
            } else {
                1.0
            };

            output.push(Complex64::new(code_val * nav_bit, 0.0));
        }

        output
    }
}

impl std::fmt::Debug for SatelliteEmitter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SatelliteEmitter")
            .field("prn", &self.prn)
            .field("signal", &self.signal)
            .field("tx_power_dbw", &self.tx_power_dbw)
            .finish()
    }
}

/// Generate PRN code for a given signal and PRN number
fn generate_prn_code(signal: GnssSignal, prn: u8) -> Vec<i8> {
    use crate::spreading::PnSequence;

    match signal {
        GnssSignal::GpsL1Ca => {
            let mut gen = super::prn::GpsCaCodeGenerator::new(prn);
            gen.generate_code()
        }
        GnssSignal::GpsL5 => {
            let mut gen = super::prn::GpsL5CodeGenerator::new_i5(prn);
            (0..gen.length()).map(|_| gen.next_chip()).collect()
        }
        GnssSignal::GlonassL1of => {
            let mut gen = super::prn::GlonassCodeGenerator::new(prn as i8);
            gen.generate_code()
        }
        GnssSignal::GalileoE1 => {
            let mut gen = super::prn::GalileoE1CodeGenerator::new(prn);
            (0..gen.length()).map(|_| gen.next_chip()).collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::waveform::gnss::GpsL1Ca;

    #[test]
    fn test_satellite_emitter_creation() {
        let wf = Box::new(GpsL1Ca::new(2_046_000.0, 1));
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GpsL1Ca, wf, orbit, 14.3);
        assert_eq!(emitter.prn, 1);
        assert_eq!(emitter.code.len(), 1023);
    }

    #[test]
    fn test_generate_baseband() {
        let wf = Box::new(GpsL1Ca::new(2_046_000.0, 1));
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GpsL1Ca, wf, orbit, 14.3);

        let samples = emitter.generate_baseband_iq(0.0, 2046, 2_046_000.0, 20_000_000.0, 0.0, 0.0);
        assert_eq!(samples.len(), 2046);
        // All samples should be +/-1 (BPSK)
        for s in &samples {
            assert!((s.re.abs() - 1.0).abs() < 0.01 || s.re.abs() < 0.01);
        }
    }

    #[test]
    fn test_satellite_status() {
        let wf = Box::new(GpsL1Ca::new(2_046_000.0, 1));
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GpsL1Ca, wf, orbit, 14.3)
            .with_ionosphere(KlobucharModel::default_broadcast())
            .with_troposphere(SaastamoinenModel::standard_atmosphere());

        let rx_lla = LlaPosition::new(40.0, -75.0, 50.0);
        let rx_ecef = crate::coordinates::lla_to_ecef(&rx_lla);
        let antenna = AntennaPattern::default_patch();

        let status = emitter.status_at(0.0, &rx_ecef, &rx_lla, &antenna);
        assert_eq!(status.prn, 1);
        assert!(status.range_m > 20_000_000.0); // > 20,000 km
    }
}
