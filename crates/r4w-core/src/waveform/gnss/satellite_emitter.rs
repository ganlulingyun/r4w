//! GNSS Satellite Emitter
//!
//! Wraps a GNSS waveform (e.g., GpsL1Ca, GalileoE1) with orbital mechanics
//! and atmospheric models to produce realistic satellite signals.
//! Implements the generic `Emitter` trait from r4w-sim.

use super::boc::CbocGenerator;
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
    pub range_rate_mps: f64,
    pub doppler_hz: f64,
    pub cn0_dbhz: f64,
    pub iono_delay_m: f64,
    pub tropo_delay_m: f64,
    pub antenna_gain_dbi: f64,
    pub visible: bool,
    /// Satellite clock correction in seconds (from SP3 or broadcast ephemeris)
    pub clock_correction_s: f64,
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
    /// Cached code chips for direct phase generation (E1B for E1OS)
    code: Vec<i8>,
    /// E1C code chips for E1OS composite signal (None for other signals)
    code_e1c: Option<Vec<i8>>,
    /// Code period in seconds
    code_period_s: f64,
    /// Chipping rate
    chipping_rate: f64,
    /// CBOC generator for Galileo E1 I-channel (E1B for E1OS)
    cboc: Option<CbocGenerator>,
    /// CBOC generator for E1C Q-channel (only for E1OS)
    cboc_e1c: Option<CbocGenerator>,
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
        use crate::spreading::PnSequence;

        let code = generate_prn_code(signal, prn);
        let code_period_s = signal.code_period_s();
        let chipping_rate = signal.chipping_rate();

        // For E1OS composite signal, we need both E1B and E1C codes
        let code_e1c = if signal == GnssSignal::GalileoE1OS {
            let mut gen = super::prn::GalileoE1CodeGenerator::new_e1c(prn);
            Some((0..gen.length()).map(|_| gen.next_chip()).collect())
        } else {
            None
        };

        // Create CBOC generator(s) for Galileo E1 channels
        // E1B (data) uses -BOC(6,1), E1C (pilot) uses +BOC(6,1)
        let (cboc, cboc_e1c) = match signal {
            GnssSignal::GalileoE1 => (Some(CbocGenerator::e1b()), None),
            GnssSignal::GalileoE1C => (Some(CbocGenerator::e1c()), None),
            GnssSignal::GalileoE1OS => (Some(CbocGenerator::e1b()), Some(CbocGenerator::e1c())),
            _ => (None, None),
        };

        // Pilot channels have no navigation data; E1OS has nav data on E1B component
        let nav_data = !signal.is_pilot();

        Self {
            prn,
            signal,
            _waveform: waveform,
            orbit,
            tx_power_dbw,
            iono_model: None,
            tropo_model: None,
            nav_data,
            code,
            code_e1c,
            code_period_s,
            chipping_rate,
            cboc,
            cboc_e1c,
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

        // Approximate C/N0 (carrier-to-noise density ratio)
        // C/N0 = EIRP - FSPL + Gr - N0
        // where N0 = kT = -228.6 dBW/K/Hz + 10*log10(290K) = -204 dBW/Hz
        let fspl = crate::coordinates::fspl_db(range_m, self.signal.carrier_frequency_hz());
        let cn0_dbhz = self.tx_power_dbw  // EIRP in dBW (includes ~13 dBi sat antenna)
            - fspl                         // Free space path loss
            + antenna_gain_dbi             // Receiver antenna gain
            + 204.0;                       // -kT in dBW/Hz (adds back thermal noise floor)

        SatelliteStatus {
            prn: self.prn,
            signal: self.signal,
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
            clock_correction_s: 0.0, // Set by scenario when SP3 data is available
        }
    }

    /// Generate baseband IQ with correct code phase from geometric range
    ///
    /// Instead of generating and then delaying, we start the PRN code at the
    /// correct phase based on the pseudorange, avoiding wasted computation.
    ///
    /// For Galileo E1, applies CBOC(6,1,1/11) subcarrier modulation to produce
    /// the characteristic split-spectrum shape required for acquisition/tracking.
    ///
    /// For Galileo E1OS (composite), generates E1B on I-channel and E1C on Q-channel,
    /// matching the real Galileo E1 Open Service signal structure per ICD.
    pub fn generate_baseband_iq(
        &self,
        t: f64,
        num_samples: usize,
        sample_rate: f64,
        geometric_range_m: f64,
        iono_delay_s: f64,
        tropo_delay_s: f64,
        sample_offset: usize,
    ) -> Vec<IQSample> {
        let total_delay_s = geometric_range_m / crate::coordinates::SPEED_OF_LIGHT
            + iono_delay_s
            + tropo_delay_s;

        // Code phase from total pseudorange
        let chips_delay = total_delay_s * self.chipping_rate;
        let code_length = self.code.len() as f64;
        let initial_code_phase = chips_delay % code_length;

        // Full epoch offset from propagation delay — needed for secondary code.
        // initial_code_phase (above) only preserves the within-period phase via modulo.
        // The secondary code needs to know which primary code epoch we're in,
        // so we track the full epochs lost by the modulo operation.
        // E.g., range 23646 km → delay ~78.9 ms → 19.7 epochs → epoch offset 19.
        let initial_epoch_offset = (chips_delay / code_length) as usize;

        // Generate samples with correct code phase
        let samples_per_chip = sample_rate / self.chipping_rate;
        let mut output = Vec::with_capacity(num_samples);

        // Simple nav data: flip every N code periods (only for E1B component)
        let nav_data_rate = self.signal.nav_data_rate_bps();
        let code_periods_per_bit = if nav_data_rate > 0.0 {
            (1.0 / (nav_data_rate * self.code_period_s)) as usize
        } else {
            1 // Avoid division by zero for pilot-only
        };

        // Check if this is a composite E1OS signal or E1C-only
        let is_composite = self.signal == GnssSignal::GalileoE1OS;
        let is_e1c = self.signal == GnssSignal::GalileoE1C;

        // E1C secondary code (25 chips, applied at each 4ms primary code epoch)
        // Creates 100ms total period (25 × 4ms)
        let e1c_secondary = super::prn::GalileoE1CodeGenerator::secondary_code();

        for i in 0..num_samples {
            // Use global sample index (sample_offset + i) so code phase
            // continues correctly across block boundaries
            let global_i = sample_offset + i;
            let chip_idx_f = initial_code_phase + global_i as f64 / samples_per_chip;
            let chip_idx = (chip_idx_f % code_length) as usize;
            let chip_phase = chip_idx_f - chip_idx_f.floor();

            // Primary code epoch index — derived from chip_idx_f so the secondary
            // code transitions EXACTLY at primary code boundaries (chip 4091→0).
            // initial_epoch_offset restores the full-epoch count lost when
            // initial_code_phase was computed via modulo. This ensures the
            // secondary code (25 chips × 4ms = 100ms) reflects the propagation
            // delay, placing the correlation peak at the correct time offset.
            let code_epoch_idx = initial_epoch_offset + (chip_idx_f / code_length) as usize;

            // E1B code value (used for I-channel, or single channel for E1B-only)
            let code_val_e1b = self.code[chip_idx.min(self.code.len() - 1)] as f64;

            // Navigation data bit (slow modulation) - only on E1B component
            let nav_bit = if self.nav_data && nav_data_rate > 0.0 {
                let bit_idx = code_epoch_idx / code_periods_per_bit;
                // Simple deterministic nav data from PRN
                if (bit_idx + self.prn as usize) % 2 == 0 { 1.0 } else { -1.0 }
            } else {
                1.0
            };

            // E1C secondary code chip (changes every 4ms, 25-chip period = 100ms)
            let secondary_chip = e1c_secondary[code_epoch_idx % 25] as f64;

            // BOC(1,1) subcarrier for Galileo E1 signals:
            // Square wave at 1.023 MHz, 2 half-periods per chip → ±1
            // This is the dominant spectral component. BOC(6,1) from full CBOC
            // is omitted because it aliases at typical sample rates (fs ≤ 5 MHz).
            // After the receiver's LPF, the despread output looks like clean BPSK.
            let boc11 = if is_composite || is_e1c
                || self.signal == GnssSignal::GalileoE1
            {
                let half_periods = 2.0; // BOC(1,1): m=1, n=1 → 2*1/1
                let sub_phase = (chip_phase * half_periods) % 2.0;
                if sub_phase < 1.0 { 1.0 } else { -1.0 }
            } else {
                1.0 // Non-Galileo: no subcarrier
            };

            if is_composite {
                // E1OS: Per Galileo ICD, E1 = (1/√2) * [e_E1B(t) - e_E1C(t)]
                // Both components include BOC(1,1) subcarrier on the REAL axis.

                // E1C code value (with secondary code)
                let code_val_e1c = self.code_e1c.as_ref()
                    .map(|c| c[chip_idx.min(c.len() - 1)] as f64)
                    .unwrap_or(0.0);

                let e1b_val = code_val_e1b * nav_bit * boc11;
                let e1c_val = code_val_e1c * secondary_chip * boc11;

                // ICD: s_E1(t) = (1/√2) * [e_E1B - e_E1C], both on I axis
                let scale = 1.0 / 2.0_f64.sqrt();
                output.push(Complex64::new((e1b_val - e1c_val) * scale, 0.0));
            } else if is_e1c {
                // E1C only: code × BOC(1,1) × secondary code on I channel, Q=0
                // The LPF removes high-frequency BOC spectrum; after despreading
                // with a BOC(1,1) replica, output is clean BPSK.
                let modulated_val = code_val_e1b * boc11 * secondary_chip;
                output.push(Complex64::new(modulated_val, 0.0));
            } else {
                // E1B (data channel): code × BOC(1,1) × nav_bit on I channel
                let modulated_val = if self.signal == GnssSignal::GalileoE1 {
                    code_val_e1b * nav_bit * boc11
                } else if let Some(ref cboc) = self.cboc {
                    code_val_e1b * nav_bit * cboc.subcarrier(chip_phase)
                } else {
                    // Plain BPSK for GPS L1 C/A, GLONASS, etc.
                    code_val_e1b * nav_bit
                };

                output.push(Complex64::new(modulated_val, 0.0));
            }
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
            // E1B (data channel)
            let mut gen = super::prn::GalileoE1CodeGenerator::new_e1b(prn);
            (0..gen.length()).map(|_| gen.next_chip()).collect()
        }
        GnssSignal::GalileoE1C => {
            // E1C (pilot channel) - different code than E1B
            let mut gen = super::prn::GalileoE1CodeGenerator::new_e1c(prn);
            (0..gen.length()).map(|_| gen.next_chip()).collect()
        }
        GnssSignal::GalileoE1OS => {
            // E1OS composite: primary code is E1B (E1C is loaded separately in new())
            let mut gen = super::prn::GalileoE1CodeGenerator::new_e1b(prn);
            (0..gen.length()).map(|_| gen.next_chip()).collect()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::waveform::gnss::{GpsL1Ca, GalileoE1};

    #[test]
    fn test_satellite_emitter_creation() {
        let wf = Box::new(GpsL1Ca::new(2_046_000.0, 1));
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GpsL1Ca, wf, orbit, 14.3);
        assert_eq!(emitter.prn, 1);
        assert_eq!(emitter.code.len(), 1023);
        assert!(emitter.cboc.is_none()); // GPS L1 C/A has no CBOC
    }

    #[test]
    fn test_galileo_e1_has_cboc() {
        let wf = Box::new(GalileoE1::new(5_000_000.0, 1));
        let orbit = KeplerianOrbit::galileo_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GalileoE1, wf, orbit, 15.0);
        assert_eq!(emitter.prn, 1);
        assert_eq!(emitter.code.len(), 4092);
        assert!(emitter.cboc.is_some()); // Galileo E1 has CBOC
    }

    #[test]
    fn test_generate_baseband() {
        let wf = Box::new(GpsL1Ca::new(2_046_000.0, 1));
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GpsL1Ca, wf, orbit, 14.3);

        let samples = emitter.generate_baseband_iq(0.0, 2046, 2_046_000.0, 20_000_000.0, 0.0, 0.0, 0);
        assert_eq!(samples.len(), 2046);
        // All samples should be +/-1 (BPSK)
        for s in &samples {
            assert!((s.re.abs() - 1.0).abs() < 0.01 || s.re.abs() < 0.01);
        }
    }

    #[test]
    fn test_galileo_e1_cboc_modulation() {
        let wf = Box::new(GalileoE1::new(5_000_000.0, 1));
        let orbit = KeplerianOrbit::galileo_nominal(0, 0);
        let emitter = SatelliteEmitter::new(1, GnssSignal::GalileoE1, wf, orbit, 15.0);

        // Generate at 5 MHz sample rate (~4.89 samples per chip)
        let samples = emitter.generate_baseband_iq(0.0, 5000, 5_000_000.0, 20_000_000.0, 0.0, 0.0, 0);
        assert_eq!(samples.len(), 5000);

        // CBOC samples should have more sign transitions than simple BPSK
        // Count sign transitions
        let mut transitions = 0;
        for w in samples.windows(2) {
            if w[0].re.signum() != w[1].re.signum() {
                transitions += 1;
            }
        }
        // At 5 MHz / 1.023 Mchip/s = ~4.89 samples/chip, ~1023 chips in 5000 samples
        // BPSK would have ~1023 transitions (one per chip boundary)
        // CBOC has BOC(1,1) subcarrier which adds ~2 transitions per chip (at Nyquist limit)
        // Due to undersampling of BOC(6,1) component, we expect ~1.5x transitions
        // The actual CBOC transitions should be noticeably more than pure BPSK (~1023)
        assert!(transitions > 1200, "CBOC should have more transitions than BPSK, got {} (expected >1200)", transitions);
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
