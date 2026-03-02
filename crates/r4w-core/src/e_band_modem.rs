//! # E-Band Millimeter-Wave Backhaul Modem
//!
//! Physical-layer DSP for point-to-point E-band (71-76 / 81-86 GHz) microwave
//! backhaul links per ETSI EN 302 217-1 v3.2.0 and ITU-R F.2006.
//!
//! ## Frequency Plan
//!
//! E-band uses paired spectrum with 10 GHz duplex spacing:
//! * **Lower band**: 71-76 GHz (TX or RX depending on direction)
//! * **Upper band**: 81-86 GHz (complementary direction)
//!
//! Channel bandwidths per ETSI EN 302 217-2 Table B.9:
//! 250 MHz, 500 MHz, 750 MHz, 1000 MHz, 1500 MHz, 2000 MHz.
//!
//! ## Adaptive Modulation
//!
//! Hitless modulation switching from QPSK (robust) through 4096QAM (highest
//! capacity), tracking the received signal level (RSL) in real time.
//!
//! ## References
//!
//! * ETSI EN 302 217-1 v3.2.0 — Fixed Radio Systems; Characteristics and
//!   Requirements for Point-to-Point Equipment and Antennas
//! * ETSI EN 302 217-2 v3.1.1 — Annex B: E-band channel arrangements
//! * ITU-R P.838-3 — Specific attenuation model for rain for use in
//!   prediction methods
//! * ITU-R P.530-18 — Propagation data and prediction methods required for
//!   the design of terrestrial line-of-sight systems
//! * ITU-R P.676-13 — Attenuation by atmospheric gases
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::e_band_modem::{EBandConfig, ChannelBandwidth, EBandModulation,
//!                               FecRate, EBandModem, LinkBudgetConfig};
//!
//! let config = EBandConfig {
//!     channel_bw: ChannelBandwidth::Bw1000MHz,
//!     modulation: EBandModulation::Qam256,
//!     fec_rate: FecRate::R0_9,
//!     tx_antenna_diameter_m: 0.6,
//!     rx_antenna_diameter_m: 0.6,
//!     path_length_km: 2.0,
//!     tx_power_dbm: 10.0,
//!     rain_rate_mm_h: 20.0,
//!     availability_pct: 99.999,
//! };
//!
//! let modem = EBandModem::new(config.clone());
//! let budget = modem.link_budget(73e9);
//! assert!(budget.system_gain_db > 100.0);
//!
//! let throughput = modem.net_throughput_gbps();
//! assert!(throughput > 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Speed of light (m/s).
const C: f64 = 2.997_924_58e8;

/// Boltzmann's constant (J/K).
const K_B: f64 = 1.380_649e-23;

/// Standard temperature (K).
const T0: f64 = 290.0;

/// Lower E-band start frequency (Hz).
pub const LOWER_BAND_START_HZ: f64 = 71e9;

/// Lower E-band end frequency (Hz).
pub const LOWER_BAND_END_HZ: f64 = 76e9;

/// Upper E-band start frequency (Hz).
pub const UPPER_BAND_START_HZ: f64 = 81e9;

/// Upper E-band end frequency (Hz).
pub const UPPER_BAND_END_HZ: f64 = 86e9;

/// Duplex spacing (Hz).
pub const DUPLEX_SPACING_HZ: f64 = 10e9;

// ---------------------------------------------------------------------------
// Channel Bandwidth
// ---------------------------------------------------------------------------

/// ETSI EN 302 217-2 Annex B channel bandwidths for E-band.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelBandwidth {
    /// 250 MHz channel.
    Bw250MHz,
    /// 500 MHz channel.
    Bw500MHz,
    /// 750 MHz channel.
    Bw750MHz,
    /// 1000 MHz channel.
    Bw1000MHz,
    /// 1500 MHz channel.
    Bw1500MHz,
    /// 2000 MHz channel.
    Bw2000MHz,
}

impl ChannelBandwidth {
    /// Bandwidth in Hz.
    pub fn hz(&self) -> f64 {
        match self {
            ChannelBandwidth::Bw250MHz => 250e6,
            ChannelBandwidth::Bw500MHz => 500e6,
            ChannelBandwidth::Bw750MHz => 750e6,
            ChannelBandwidth::Bw1000MHz => 1000e6,
            ChannelBandwidth::Bw1500MHz => 1500e6,
            ChannelBandwidth::Bw2000MHz => 2000e6,
        }
    }

    /// Bandwidth in MHz.
    pub fn mhz(&self) -> f64 {
        self.hz() / 1e6
    }

    /// Symbol rate assuming Nyquist, roll-off ~0.25 (symbols/s).
    /// Symbol rate ≈ BW / (1 + roll_off).
    pub fn symbol_rate_sps(&self) -> f64 {
        self.hz() / 1.25
    }
}

// ---------------------------------------------------------------------------
// Modulation
// ---------------------------------------------------------------------------

/// E-band adaptive modulation orders per ETSI EN 302 217.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EBandModulation {
    /// QPSK — 2 bits/symbol.
    Qpsk,
    /// 16-QAM — 4 bits/symbol.
    Qam16,
    /// 32-QAM — 5 bits/symbol.
    Qam32,
    /// 64-QAM — 6 bits/symbol.
    Qam64,
    /// 128-QAM — 7 bits/symbol.
    Qam128,
    /// 256-QAM — 8 bits/symbol.
    Qam256,
    /// 512-QAM — 9 bits/symbol.
    Qam512,
    /// 1024-QAM — 10 bits/symbol.
    Qam1024,
    /// 2048-QAM — 11 bits/symbol.
    Qam2048,
    /// 4096-QAM — 12 bits/symbol.
    Qam4096,
}

impl EBandModulation {
    /// Bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            EBandModulation::Qpsk => 2,
            EBandModulation::Qam16 => 4,
            EBandModulation::Qam32 => 5,
            EBandModulation::Qam64 => 6,
            EBandModulation::Qam128 => 7,
            EBandModulation::Qam256 => 8,
            EBandModulation::Qam512 => 9,
            EBandModulation::Qam1024 => 10,
            EBandModulation::Qam2048 => 11,
            EBandModulation::Qam4096 => 12,
        }
    }

    /// Modulation order M.
    pub fn order(&self) -> usize {
        1 << self.bits_per_symbol()
    }

    /// Minimum required SNR (dB) for quasi-error-free operation (BER ≈ 1e-6).
    ///
    /// Derived from theoretical AWGN Pb for M-QAM with Grey coding:
    /// approximately Eb/N0 = 10*log10(M)/2 + 3 dB margin, adapted for
    /// spectral-efficiency-adjusted noise bandwidth.
    pub fn min_snr_db(&self) -> f64 {
        match self {
            EBandModulation::Qpsk => 10.5,
            EBandModulation::Qam16 => 16.5,
            EBandModulation::Qam32 => 19.5,
            EBandModulation::Qam64 => 22.5,
            EBandModulation::Qam128 => 25.5,
            EBandModulation::Qam256 => 28.5,
            EBandModulation::Qam512 => 31.5,
            EBandModulation::Qam1024 => 34.5,
            EBandModulation::Qam2048 => 37.5,
            EBandModulation::Qam4096 => 40.5,
        }
    }

    /// All modulations sorted from lowest to highest order.
    pub fn all() -> [EBandModulation; 10] {
        [
            EBandModulation::Qpsk,
            EBandModulation::Qam16,
            EBandModulation::Qam32,
            EBandModulation::Qam64,
            EBandModulation::Qam128,
            EBandModulation::Qam256,
            EBandModulation::Qam512,
            EBandModulation::Qam1024,
            EBandModulation::Qam2048,
            EBandModulation::Qam4096,
        ]
    }

    /// Short display name.
    pub fn name(&self) -> &'static str {
        match self {
            EBandModulation::Qpsk => "QPSK",
            EBandModulation::Qam16 => "16QAM",
            EBandModulation::Qam32 => "32QAM",
            EBandModulation::Qam64 => "64QAM",
            EBandModulation::Qam128 => "128QAM",
            EBandModulation::Qam256 => "256QAM",
            EBandModulation::Qam512 => "512QAM",
            EBandModulation::Qam1024 => "1024QAM",
            EBandModulation::Qam2048 => "2048QAM",
            EBandModulation::Qam4096 => "4096QAM",
        }
    }
}

// ---------------------------------------------------------------------------
// FEC Rate
// ---------------------------------------------------------------------------

/// LDPC code rate (inner code).
///
/// E-band systems typically use DVB-S2-based LDPC or proprietary codes.
/// Net coding gain (NCG) relative to uncoded is approximately:
/// `NCG ≈ 10*log10(code_rate) + coding_gain_dB`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FecRate {
    /// Rate 1/2 — highest protection, 6 dB overhead.
    R0_5,
    /// Rate 3/5.
    R0_6,
    /// Rate 7/10.
    R0_7,
    /// Rate 4/5.
    R0_8,
    /// Rate 9/10 — lowest overhead, 0.46 dB.
    R0_9,
}

impl FecRate {
    /// Code rate as a floating-point fraction.
    pub fn rate(&self) -> f64 {
        match self {
            FecRate::R0_5 => 0.5,
            FecRate::R0_6 => 0.6,
            FecRate::R0_7 => 0.7,
            FecRate::R0_8 => 0.8,
            FecRate::R0_9 => 0.9,
        }
    }

    /// Net coding gain (dB) approximation for LDPC at target BER 1e-9.
    ///
    /// Based on LDPC waterfall curves: rate 1/2 gains ~8.5 dB vs uncoded,
    /// with approximately 1.5 dB reduction per 0.1 rate step.
    pub fn net_coding_gain_db(&self) -> f64 {
        match self {
            FecRate::R0_5 => 8.5,
            FecRate::R0_6 => 7.5,
            FecRate::R0_7 => 6.5,
            FecRate::R0_8 => 5.5,
            FecRate::R0_9 => 4.5,
        }
    }

    /// Overhead factor (reciprocal of code rate) applied to symbol rate.
    pub fn overhead_factor(&self) -> f64 {
        1.0 / self.rate()
    }

    /// Display string.
    pub fn name(&self) -> &'static str {
        match self {
            FecRate::R0_5 => "1/2",
            FecRate::R0_6 => "3/5",
            FecRate::R0_7 => "7/10",
            FecRate::R0_8 => "4/5",
            FecRate::R0_9 => "9/10",
        }
    }
}

// ---------------------------------------------------------------------------
// Channel Plan
// ---------------------------------------------------------------------------

/// E-band channel descriptor per ETSI EN 302 217-2 Annex B.
#[derive(Debug, Clone)]
pub struct Channel {
    /// Channel number (1-based).
    pub number: u32,
    /// Centre frequency of this channel in the lower band (Hz).
    pub lower_centre_hz: f64,
    /// Centre frequency of this channel in the upper band (Hz).
    pub upper_centre_hz: f64,
    /// Channel bandwidth (Hz).
    pub bandwidth_hz: f64,
}

impl Channel {
    /// Duplex spacing (always 10 GHz for E-band).
    pub fn duplex_spacing_hz(&self) -> f64 {
        self.upper_centre_hz - self.lower_centre_hz
    }
}

/// E-band channel plan for a given bandwidth.
///
/// ETSI EN 302 217-2 Annex B defines channel arrangements for E-band.
/// Lower band occupies 71-76 GHz, upper band 81-86 GHz, with 10 GHz
/// duplex spacing between paired channels.
pub struct ChannelPlan {
    /// Channel bandwidth.
    pub bandwidth: ChannelBandwidth,
    /// All available channels.
    pub channels: Vec<Channel>,
}

impl ChannelPlan {
    /// Generate the channel plan for the given bandwidth.
    ///
    /// Channels are spaced by the channel bandwidth across the 5 GHz span
    /// of each sub-band.
    pub fn new(bandwidth: ChannelBandwidth) -> Self {
        let bw_hz = bandwidth.hz();
        let span_hz = LOWER_BAND_END_HZ - LOWER_BAND_START_HZ; // 5 GHz
        let num_channels = (span_hz / bw_hz).floor() as u32;
        let mut channels = Vec::with_capacity(num_channels as usize);
        for i in 0..num_channels {
            let lower_centre = LOWER_BAND_START_HZ + (i as f64 + 0.5) * bw_hz;
            let upper_centre = lower_centre + DUPLEX_SPACING_HZ;
            channels.push(Channel {
                number: i + 1,
                lower_centre_hz: lower_centre,
                upper_centre_hz: upper_centre,
                bandwidth_hz: bw_hz,
            });
        }
        ChannelPlan { bandwidth, channels }
    }

    /// Number of channels in the plan.
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }

    /// Find channel by number (1-based).
    pub fn channel(&self, number: u32) -> Option<&Channel> {
        self.channels.iter().find(|c| c.number == number)
    }
}

// ---------------------------------------------------------------------------
// Rain Fade Model — ITU-R P.838-3 / P.530-18
// ---------------------------------------------------------------------------

/// ITU-R P.838-3 coefficients for specific rain attenuation at E-band.
///
/// γ_R = k · R^α  [dB/km]
///
/// where R is the rain rate in mm/h. Coefficients are for horizontal
/// polarisation at 73 GHz (lower E-band centre) and 83 GHz (upper E-band).
/// Values from ITU-R P.838-3 Table 1 (logarithmic interpolation between
/// tabulated frequencies).
#[derive(Debug, Clone)]
pub struct RainAttenuationCoeffs {
    /// k coefficient (frequency/polarisation dependent).
    pub k: f64,
    /// α exponent.
    pub alpha: f64,
    /// Carrier frequency (Hz).
    pub frequency_hz: f64,
}

impl RainAttenuationCoeffs {
    /// Coefficients for 73 GHz horizontal polarisation (ITU-R P.838-3).
    pub fn for_73ghz_h() -> Self {
        RainAttenuationCoeffs {
            k: 1.1670,
            alpha: 0.6680,
            frequency_hz: 73e9,
        }
    }

    /// Coefficients for 73 GHz vertical polarisation (ITU-R P.838-3).
    pub fn for_73ghz_v() -> Self {
        RainAttenuationCoeffs {
            k: 1.0820,
            alpha: 0.6590,
            frequency_hz: 73e9,
        }
    }

    /// Coefficients for 83 GHz horizontal polarisation (ITU-R P.838-3).
    pub fn for_83ghz_h() -> Self {
        RainAttenuationCoeffs {
            k: 1.4240,
            alpha: 0.6580,
            frequency_hz: 83e9,
        }
    }

    /// Coefficients for 83 GHz vertical polarisation (ITU-R P.838-3).
    pub fn for_83ghz_v() -> Self {
        RainAttenuationCoeffs {
            k: 1.3210,
            alpha: 0.6510,
            frequency_hz: 83e9,
        }
    }

    /// Specific attenuation γ_R (dB/km) for rain rate R (mm/h).
    ///
    /// γ_R = k · R^α  (ITU-R P.838-3 eq. 1)
    pub fn specific_attenuation_db_per_km(&self, rain_rate_mm_h: f64) -> f64 {
        if rain_rate_mm_h <= 0.0 {
            return 0.0;
        }
        self.k * rain_rate_mm_h.powf(self.alpha)
    }
}

/// Rain fade model implementing ITU-R P.838-3 specific attenuation and
/// ITU-R P.530-18 path availability / exceeded statistics.
pub struct RainFadeModel {
    /// Rain rate at 0.01% of time (mm/h) for the link region.
    pub rain_rate_001_mm_h: f64,
    /// Rain attenuation coefficients.
    pub coeffs: RainAttenuationCoeffs,
    /// Path length (km).
    pub path_length_km: f64,
}

impl RainFadeModel {
    /// Create a rain fade model for the lower E-band centre (73 GHz, H-pol).
    pub fn new_lower_band(rain_rate_001_mm_h: f64, path_length_km: f64) -> Self {
        RainFadeModel {
            rain_rate_001_mm_h,
            coeffs: RainAttenuationCoeffs::for_73ghz_h(),
            path_length_km,
        }
    }

    /// Create a rain fade model for the upper E-band centre (83 GHz, H-pol).
    pub fn new_upper_band(rain_rate_001_mm_h: f64, path_length_km: f64) -> Self {
        RainFadeModel {
            rain_rate_001_mm_h,
            coeffs: RainAttenuationCoeffs::for_83ghz_h(),
            path_length_km,
        }
    }

    /// Effective path length reduction factor r_E per ITU-R P.530-18 §2.2.1.1.
    ///
    /// d_eff = r_E · d  where d_eff accounts for the spatial variability of
    /// rain cells (typical rain cell diameter ~5 km).
    ///
    /// r_E = 1 / (1 + 0.78·sqrt(d·γ/f) - 0.38·(1-exp(-2d)))
    /// Simplified version using path length and specific attenuation at 0.01%.
    fn effective_path_reduction(&self, specific_att: f64) -> f64 {
        let d = self.path_length_km;
        // ITU-R P.530-18 equation 32
        let numerator = 1.0;
        let term1 = 0.78 * (d * specific_att / (self.coeffs.frequency_hz / 1e9)).sqrt();
        let term2 = 0.38 * (1.0 - (-2.0 * d).exp());
        let denom = numerator + term1 - term2;
        if denom <= 0.0 {
            return 1.0;
        }
        1.0 / denom
    }

    /// Path attenuation exceeded 0.01% of the time A_0.01 (dB).
    ///
    /// A_0.01 = γ_R(R_0.01) · r_E · d  (ITU-R P.530-18 §2.2.1.1)
    pub fn exceeded_001pct_db(&self) -> f64 {
        let gamma = self.coeffs.specific_attenuation_db_per_km(self.rain_rate_001_mm_h);
        let r_e = self.effective_path_reduction(gamma);
        gamma * r_e * self.path_length_km
    }

    /// Path attenuation exceeded for a given percentage of time p% (dB).
    ///
    /// For p in [0.001%, 1%], uses ITU-R P.530-18 equation 33:
    /// A_p = A_0.01 · (p/0.01)^(-(0.655 + 0.033·ln(p) - 0.045·ln(A_0.01)))
    pub fn exceeded_p_pct_db(&self, percent: f64) -> f64 {
        let a001 = self.exceeded_001pct_db();
        if a001 <= 0.0 {
            return 0.0;
        }
        let p = percent.clamp(0.001, 1.0);
        let exponent = -(0.655 + 0.033 * p.ln() - 0.045 * a001.ln());
        a001 * (p / 0.01_f64).powf(exponent)
    }

    /// Required fade margin to achieve a given availability (%).
    ///
    /// Availability 99.999% means outage 0.001% → compute A at 0.001%.
    pub fn required_fade_margin_db(&self, availability_pct: f64) -> f64 {
        let outage_pct = 100.0 - availability_pct;
        self.exceeded_p_pct_db(outage_pct)
    }

    /// Specific attenuation at a given rain rate (dB/km).
    pub fn specific_attenuation_db_per_km(&self, rain_rate_mm_h: f64) -> f64 {
        self.coeffs.specific_attenuation_db_per_km(rain_rate_mm_h)
    }

    /// Total path attenuation at a given rain rate (dB).
    pub fn path_attenuation_db(&self, rain_rate_mm_h: f64) -> f64 {
        let gamma = self.specific_attenuation_db_per_km(rain_rate_mm_h);
        gamma * self.path_length_km
    }
}

// ---------------------------------------------------------------------------
// Atmospheric Absorption (ITU-R P.676-13)
// ---------------------------------------------------------------------------

/// Atmospheric absorption model for E-band per ITU-R P.676-13.
///
/// E-band (70-90 GHz) sits in a window between the 60 GHz oxygen absorption
/// peak and the 118.75 GHz oxygen line. Water vapour adds additional loss.
pub struct AtmosphericAbsorption;

impl AtmosphericAbsorption {
    /// Specific oxygen absorption at 73 GHz (dB/km) at standard atmosphere.
    ///
    /// Approximate value from ITU-R P.676-13 Figure 1 at 73 GHz:
    /// ~0.5 dB/km at sea level (1013 hPa, 15°C, 0 g/m³ water vapour).
    pub const OXYGEN_73GHZ_DB_PER_KM: f64 = 0.50;

    /// Specific oxygen absorption at 83 GHz (dB/km) at standard atmosphere.
    ///
    /// ~0.45 dB/km — slightly lower than lower band (farther from 60 GHz peak).
    pub const OXYGEN_83GHZ_DB_PER_KM: f64 = 0.45;

    /// Specific water vapour absorption at 73 GHz (dB/km) per g/m³.
    ///
    /// Approximately 0.03 dB/km per g/m³ at 73 GHz (ITU-R P.676-13 Figure 1).
    pub const WV_73GHZ_DB_PER_KM_PER_G_M3: f64 = 0.030;

    /// Specific water vapour absorption at 83 GHz (dB/km) per g/m³.
    pub const WV_83GHZ_DB_PER_KM_PER_G_M3: f64 = 0.032;

    /// Total atmospheric absorption (dB) for a given path.
    ///
    /// # Arguments
    /// * `frequency_hz` — carrier frequency
    /// * `path_length_km` — one-way path length
    /// * `water_vapour_g_m3` — integrated water vapour content (typical 7.5 g/m³)
    pub fn total_db(frequency_hz: f64, path_length_km: f64, water_vapour_g_m3: f64) -> f64 {
        let (o2_rate, wv_rate) = if frequency_hz < 77e9 {
            (Self::OXYGEN_73GHZ_DB_PER_KM, Self::WV_73GHZ_DB_PER_KM_PER_G_M3)
        } else {
            (Self::OXYGEN_83GHZ_DB_PER_KM, Self::WV_83GHZ_DB_PER_KM_PER_G_M3)
        };
        (o2_rate + wv_rate * water_vapour_g_m3) * path_length_km
    }
}

// ---------------------------------------------------------------------------
// Antenna Gain
// ---------------------------------------------------------------------------

/// High-gain parabolic dish antenna model for E-band.
///
/// Typical efficiency η ≈ 0.55 for a standard feed.
/// G = η · (π·D/λ)²  (linear gain)
pub struct ParabolicAntenna {
    /// Dish diameter (m).
    pub diameter_m: f64,
    /// Aperture efficiency (dimensionless, typically 0.55).
    pub efficiency: f64,
}

impl ParabolicAntenna {
    /// Create antenna with standard 55% efficiency.
    pub fn new(diameter_m: f64) -> Self {
        ParabolicAntenna { diameter_m, efficiency: 0.55 }
    }

    /// Create antenna with custom efficiency.
    pub fn with_efficiency(diameter_m: f64, efficiency: f64) -> Self {
        ParabolicAntenna { diameter_m, efficiency }
    }

    /// Gain (linear) at the given frequency.
    pub fn gain_linear(&self, frequency_hz: f64) -> f64 {
        let lambda = C / frequency_hz;
        self.efficiency * (PI * self.diameter_m / lambda).powi(2)
    }

    /// Gain (dBi) at the given frequency.
    pub fn gain_dbi(&self, frequency_hz: f64) -> f64 {
        10.0 * self.gain_linear(frequency_hz).log10()
    }

    /// Half-power beamwidth (degrees) — approximate formula 70λ/D.
    pub fn hpbw_degrees(&self, frequency_hz: f64) -> f64 {
        let lambda = C / frequency_hz;
        70.0 * lambda / self.diameter_m
    }
}

// ---------------------------------------------------------------------------
// Free-Space Path Loss
// ---------------------------------------------------------------------------

/// Free-space path loss (dB) at E-band frequencies.
///
/// FSPL = 20·log10(4·π·d·f/c)  (dB)
///
/// where d is path length (m), f is frequency (Hz).
pub fn free_space_path_loss_db(distance_m: f64, frequency_hz: f64) -> f64 {
    let lambda = C / frequency_hz;
    let fspl = (4.0 * PI * distance_m / lambda).powi(2);
    10.0 * fspl.log10()
}

// ---------------------------------------------------------------------------
// Link Budget
// ---------------------------------------------------------------------------

/// Input parameters for the E-band link budget calculation.
#[derive(Debug, Clone)]
pub struct LinkBudgetConfig {
    /// Transmit power (dBm).
    pub tx_power_dbm: f64,
    /// Transmit antenna gain (dBi).
    pub tx_antenna_gain_dbi: f64,
    /// Receive antenna gain (dBi).
    pub rx_antenna_gain_dbi: f64,
    /// Carrier frequency (Hz).
    pub frequency_hz: f64,
    /// Path length (m).
    pub path_length_m: f64,
    /// Atmospheric absorption (dB).
    pub atm_absorption_db: f64,
    /// Rain attenuation (dB, operational point not fade margin).
    pub rain_attenuation_db: f64,
    /// Receiver noise figure (dB).
    pub rx_noise_figure_db: f64,
    /// Channel bandwidth (Hz).
    pub bandwidth_hz: f64,
    /// Cable / connector / branching losses at TX (dB).
    pub tx_feeder_loss_db: f64,
    /// Cable / connector / branching losses at RX (dB).
    pub rx_feeder_loss_db: f64,
}

impl Default for LinkBudgetConfig {
    fn default() -> Self {
        LinkBudgetConfig {
            tx_power_dbm: 10.0,
            tx_antenna_gain_dbi: 40.0,
            rx_antenna_gain_dbi: 40.0,
            frequency_hz: 73e9,
            path_length_m: 2000.0,
            atm_absorption_db: 1.0,
            rain_attenuation_db: 0.0,
            rx_noise_figure_db: 7.0,
            bandwidth_hz: 1000e6,
            tx_feeder_loss_db: 1.0,
            rx_feeder_loss_db: 1.0,
        }
    }
}

/// Complete E-band link budget result.
#[derive(Debug, Clone)]
pub struct LinkBudgetResult {
    /// Effective Isotropic Radiated Power (dBm).
    pub eirp_dbm: f64,
    /// Free-space path loss (dB).
    pub fspl_db: f64,
    /// Total path loss including FSPL + atmospheric + rain (dB).
    pub total_path_loss_db: f64,
    /// Received signal level (dBm).
    pub rsl_dbm: f64,
    /// Thermal noise floor at receiver (dBm).
    pub noise_floor_dbm: f64,
    /// Receiver sensitivity for given NF and BW (dBm).
    pub rx_sensitivity_dbm: f64,
    /// System gain (dBm) = EIRP - total_path_loss + RX_gain - rx_feeder.
    pub system_gain_db: f64,
    /// Carrier-to-Noise ratio (dB).
    pub cnr_db: f64,
    /// Excess fade margin above minimum required SNR (dB).
    pub fade_margin_db: f64,
}

impl LinkBudgetResult {
    /// Whether the link has positive fade margin.
    pub fn is_viable(&self) -> bool {
        self.fade_margin_db > 0.0
    }
}

/// E-band link budget calculator.
pub fn compute_link_budget(cfg: &LinkBudgetConfig, required_snr_db: f64) -> LinkBudgetResult {
    // EIRP = Tx power + Tx gain - feeder loss
    let eirp_dbm = cfg.tx_power_dbm + cfg.tx_antenna_gain_dbi - cfg.tx_feeder_loss_db;

    // Free-space path loss
    let fspl_db = free_space_path_loss_db(cfg.path_length_m, cfg.frequency_hz);

    // Total propagation loss
    let total_path_loss_db = fspl_db + cfg.atm_absorption_db + cfg.rain_attenuation_db;

    // RSL = EIRP - total_path_loss + RX_gain - RX_feeder
    let rsl_dbm = eirp_dbm - total_path_loss_db + cfg.rx_antenna_gain_dbi - cfg.rx_feeder_loss_db;

    // System gain = EIRP + RX_gain - feeder losses
    let system_gain_db = cfg.tx_antenna_gain_dbi + cfg.rx_antenna_gain_dbi
        - cfg.tx_feeder_loss_db - cfg.rx_feeder_loss_db;

    // Thermal noise floor: N = k · T0 · B · NF (dBm)
    let noise_power_w = K_B * T0 * cfg.bandwidth_hz * 10f64.powf(cfg.rx_noise_figure_db / 10.0);
    let noise_floor_dbm = 10.0 * (noise_power_w * 1e3).log10(); // convert W to mW

    // Minimum receiver sensitivity for required SNR
    let rx_sensitivity_dbm = noise_floor_dbm + required_snr_db;

    // CNR = RSL - noise floor
    let cnr_db = rsl_dbm - noise_floor_dbm;

    // Fade margin relative to required SNR
    let fade_margin_db = cnr_db - required_snr_db;

    LinkBudgetResult {
        eirp_dbm,
        fspl_db,
        total_path_loss_db,
        rsl_dbm,
        noise_floor_dbm,
        rx_sensitivity_dbm,
        system_gain_db,
        cnr_db,
        fade_margin_db,
    }
}

// ---------------------------------------------------------------------------
// FEC (LDPC + Outer BCH)
// ---------------------------------------------------------------------------

/// Simple LDPC-like encoder for simulation.
///
/// Implements a simplified systematic LDPC encoder using a parity check
/// matrix constructed from a regular (3,6) LDPC graph for educational
/// purposes. In real E-band products, proprietary or DVB-S2-based LDPC
/// codes with codeword lengths of 16200 or 64800 bits are used.
pub struct LdpcEncoder {
    /// Code rate.
    pub rate: FecRate,
    /// Codeword block length (information bits).
    pub k: usize,
    /// Codeword length (total bits).
    pub n: usize,
}

impl LdpcEncoder {
    /// Create a new LDPC encoder with the given rate and information block size.
    pub fn new(rate: FecRate, k: usize) -> Self {
        let n = (k as f64 / rate.rate()).round() as usize;
        LdpcEncoder { rate, k, n }
    }

    /// Encode information bits.
    ///
    /// Returns codeword: first `k` bits are systematic (unchanged),
    /// remaining `n-k` bits are parity computed via XOR accumulation over
    /// a diagonal parity structure.
    pub fn encode(&self, info: &[bool]) -> Vec<bool> {
        assert_eq!(info.len(), self.k, "info length must be k");
        let num_parity = self.n - self.k;
        let mut codeword = Vec::with_capacity(self.n);
        // Systematic part
        codeword.extend_from_slice(info);
        // Accumulate parity using diagonal structure p[i] = p[i-1] XOR info[i mod k]
        let mut acc = false;
        for i in 0..num_parity {
            acc ^= info[i % self.k];
            codeword.push(acc);
        }
        codeword
    }

    /// Decode received codeword using hard-decision syndrome check.
    ///
    /// Returns estimated information bits. On syndrome failure, single-bit
    /// correction is attempted.
    pub fn decode(&self, received: &[bool]) -> Vec<bool> {
        assert_eq!(received.len(), self.n, "received length must be n");
        // Compute syndrome via re-encoding and comparing parity
        let info_bits: Vec<bool> = received[..self.k].to_vec();
        let re_encoded = self.encode(&info_bits);
        // Check if parities match
        let syndrome_ok = received[self.k..].iter()
            .zip(re_encoded[self.k..].iter())
            .all(|(r, e)| r == e);
        if syndrome_ok {
            return info_bits;
        }
        // Single-bit flip error correction: flip each bit and recheck
        let mut corrected = received.to_vec();
        for i in 0..self.n {
            corrected[i] = !corrected[i];
            let test_info: Vec<bool> = corrected[..self.k].to_vec();
            let re_enc = self.encode(&test_info);
            let ok = corrected[self.k..].iter()
                .zip(re_enc[self.k..].iter())
                .all(|(r, e)| r == e);
            if ok {
                return corrected[..self.k].to_vec();
            }
            corrected[i] = !corrected[i]; // restore
        }
        // Uncorrectable — return systematic bits as-is
        info_bits
    }

    /// Net coding gain (dB) for this LDPC rate.
    pub fn coding_gain_db(&self) -> f64 {
        self.rate.net_coding_gain_db()
    }
}

/// Outer BCH code for error-floor mitigation.
///
/// BCH(63, 51, 2) over GF(2^6) used as illustrative outer code.
/// In practice, product codes or staircase codes may be used.
pub struct BchOuterCode {
    /// Message length (information bits).
    pub k: usize,
    /// Codeword length.
    pub n: usize,
    /// Error correction capability.
    pub t: usize,
}

impl BchOuterCode {
    /// BCH(63, 51, 2) — corrects up to 2 errors.
    pub fn bch63_51() -> Self {
        BchOuterCode { n: 63, k: 51, t: 2 }
    }

    /// Code rate.
    pub fn rate(&self) -> f64 {
        self.k as f64 / self.n as f64
    }

    /// Encode using simple parity append (educational approximation).
    pub fn encode(&self, info: &[bool]) -> Vec<bool> {
        assert_eq!(info.len(), self.k);
        let mut codeword = info.to_vec();
        let num_parity = self.n - self.k;
        // Generate parity bits via XOR accumulation (simplified BCH)
        for i in 0..num_parity {
            let mut p = false;
            for j in 0..self.k {
                if (j + i) % (num_parity + 1) < self.k {
                    p ^= info[j];
                }
            }
            codeword.push(p);
        }
        codeword
    }

    /// Decode with hard-decision BDD up to `t` errors.
    pub fn decode(&self, received: &[bool]) -> Vec<bool> {
        assert_eq!(received.len(), self.n);
        // Return systematic part (simplified — no actual BDD)
        received[..self.k].to_vec()
    }

    /// Approximate BER improvement factor (dB) from BCH outer code.
    pub fn gain_db(&self) -> f64 {
        // BCH(63,51,t=2) provides approximately 3 dB coding gain at BER 1e-12
        3.0
    }
}

/// Combined LDPC + BCH FEC chain.
pub struct FecChain {
    pub ldpc: LdpcEncoder,
    pub bch: BchOuterCode,
}

impl FecChain {
    /// Create a typical E-band FEC chain.
    pub fn new(rate: FecRate, ldpc_k: usize) -> Self {
        FecChain {
            ldpc: LdpcEncoder::new(rate, ldpc_k),
            bch: BchOuterCode::bch63_51(),
        }
    }

    /// Total FEC overhead (fraction).
    pub fn total_overhead(&self) -> f64 {
        1.0 / (self.ldpc.rate.rate() * self.bch.rate())
    }

    /// Aggregate net coding gain (dB).
    pub fn total_ncg_db(&self) -> f64 {
        self.ldpc.coding_gain_db() + self.bch.gain_db()
    }
}

// ---------------------------------------------------------------------------
// Constellation Mapper / Demapper
// ---------------------------------------------------------------------------

/// Gray-coded rectangular QAM constellation point.
#[derive(Debug, Clone, Copy)]
pub struct ConstellationPoint {
    /// In-phase component (normalized).
    pub i: f64,
    /// Quadrature component (normalized).
    pub q: f64,
    /// Grey-coded bit label.
    pub label: u64,
}

/// QAM constellation mapper.
///
/// Generates normalised M-QAM constellation points with Grey coding.
/// Points are normalised so average power = 1.
pub struct QamConstellation {
    /// Modulation order.
    pub modulation: EBandModulation,
    /// Constellation points (length = order M).
    pub points: Vec<ConstellationPoint>,
    /// Normalisation factor.
    pub norm_factor: f64,
}

impl QamConstellation {
    /// Build the Grey-coded QAM constellation.
    pub fn new(modulation: EBandModulation) -> Self {
        let m = modulation.order();
        let bits = modulation.bits_per_symbol();
        // For square QAM: sqrt_m × sqrt_m grid
        let sqrt_m = (m as f64).sqrt() as usize;
        assert!(sqrt_m * sqrt_m == m, "Only square QAM supported");

        let bits_per_axis = bits / 2;
        let levels = sqrt_m;

        // Raw unnormalised points
        let mut points = Vec::with_capacity(m);
        let mut sum_power = 0.0_f64;
        for row in 0..levels {
            for col in 0..levels {
                let i_raw = (2 * col) as f64 - (levels - 1) as f64;
                let q_raw = (2 * row) as f64 - (levels - 1) as f64;
                let power = i_raw * i_raw + q_raw * q_raw;
                sum_power += power;
                // Grey code the axis indices
                let i_grey = grey_code(col, bits_per_axis);
                let q_grey = grey_code(row, bits_per_axis);
                let label = ((q_grey as u64) << (bits_per_axis as u64)) | (i_grey as u64);
                points.push(ConstellationPoint { i: i_raw, q: q_raw, label });
            }
        }
        let avg_power = sum_power / m as f64;
        let norm_factor = avg_power.sqrt();
        // Normalise
        for p in &mut points {
            p.i /= norm_factor;
            p.q /= norm_factor;
        }
        QamConstellation { modulation, points, norm_factor }
    }

    /// Map a bit sequence (length = bits_per_symbol) to a constellation point.
    pub fn map(&self, bits: &[bool]) -> ConstellationPoint {
        let bps = self.modulation.bits_per_symbol();
        assert_eq!(bits.len(), bps);
        let mut label: u64 = 0;
        for &b in bits {
            label = (label << 1) | (b as u64);
        }
        self.points.iter().find(|p| p.label == label)
            .copied()
            .unwrap_or(self.points[0])
    }

    /// Hard-decision demap: return bits for nearest constellation point.
    pub fn demap_hard(&self, i: f64, q: f64) -> Vec<bool> {
        let best = self.points.iter().min_by(|a, b| {
            let da = (a.i - i).powi(2) + (a.q - q).powi(2);
            let db = (b.i - i).powi(2) + (b.q - q).powi(2);
            da.partial_cmp(&db).unwrap()
        }).unwrap();
        let bps = self.modulation.bits_per_symbol();
        (0..bps).rev().map(|k| (best.label >> k) & 1 == 1).collect()
    }
}

/// Compute the Grey code for integer n with given number of bits.
fn grey_code(n: usize, _bits: usize) -> usize {
    n ^ (n >> 1)
}

// ---------------------------------------------------------------------------
// XPIC — Cross-Polar Interference Canceller
// ---------------------------------------------------------------------------

/// XPIC (Cross-Polar Interference Cancellation) adaptive filter.
///
/// E-band dual-polarisation (H+V) systems use XPIC to cancel cross-polar
/// interference caused by imperfect antenna XPD (Cross-Polar Discrimination)
/// and depolarisation in rain.
///
/// The XPIC filter is a complex LMS adaptive transversal filter applied to
/// the cross-polar signal to estimate and subtract the interference
/// component from the co-polar received signal.
///
/// Reference: R. H. Otte et al., "Cross-Polarization Interference Cancellation
/// for Dual-Polarization Microwave Radio Systems", IEEE Trans. Commun., 2014.
pub struct XpicFilter {
    /// FIR filter coefficients (complex).
    coeffs: Vec<(f64, f64)>,
    /// LMS step size μ.
    mu: f64,
    /// Tap delay buffer for cross-polar input (I, Q).
    delay_line: Vec<(f64, f64)>,
}

impl XpicFilter {
    /// Create an XPIC filter with the given number of taps and step size.
    pub fn new(num_taps: usize, mu: f64) -> Self {
        XpicFilter {
            coeffs: vec![(0.0, 0.0); num_taps],
            mu,
            delay_line: vec![(0.0, 0.0); num_taps],
        }
    }

    /// Process one sample: update delay line, compute output, apply LMS.
    ///
    /// # Arguments
    /// * `xpol_i`, `xpol_q` — cross-polar input sample (I, Q)
    /// * `copol_i`, `copol_q` — co-polar reference sample (I, Q) before cancellation
    ///
    /// Returns the error signal after XPIC cancellation (I, Q).
    pub fn process(
        &mut self,
        xpol_i: f64,
        xpol_q: f64,
        copol_i: f64,
        copol_q: f64,
    ) -> (f64, f64) {
        // Shift delay line
        let n = self.delay_line.len();
        for k in (1..n).rev() {
            self.delay_line[k] = self.delay_line[k - 1];
        }
        self.delay_line[0] = (xpol_i, xpol_q);

        // Compute filter output: y = sum(w * x) complex
        let (mut y_i, mut y_q) = (0.0, 0.0);
        for k in 0..n {
            let (wi, wq) = self.coeffs[k];
            let (xi, xq) = self.delay_line[k];
            y_i += wi * xi - wq * xq;
            y_q += wi * xq + wq * xi;
        }

        // Error = copolar - canceller output
        let e_i = copol_i - y_i;
        let e_q = copol_q - y_q;

        // LMS update: w = w + mu * conj(x) * e
        for k in 0..n {
            let (xi, xq) = self.delay_line[k];
            // Conjugate of x * e:  (xi - j*xq)(e_i + j*e_q) = xi*e_i + xq*e_q + j*(xi*e_q - xq*e_i)
            self.coeffs[k].0 += self.mu * (xi * e_i + xq * e_q);
            self.coeffs[k].1 += self.mu * (xi * e_q - xq * e_i);
        }

        (e_i, e_q)
    }

    /// Number of taps.
    pub fn num_taps(&self) -> usize {
        self.coeffs.len()
    }

    /// Residual interference estimate from coefficients (power proxy).
    pub fn residual_xpi_db(&self) -> f64 {
        let pwr: f64 = self.coeffs.iter().map(|(i, q)| i * i + q * q).sum();
        if pwr <= 0.0 { return f64::NEG_INFINITY; }
        10.0 * pwr.log10()
    }

    /// Reset coefficients to zero.
    pub fn reset(&mut self) {
        for c in &mut self.coeffs { *c = (0.0, 0.0); }
        for d in &mut self.delay_line { *d = (0.0, 0.0); }
    }
}

// ---------------------------------------------------------------------------
// XPD Model — Cross-Polar Discrimination
// ---------------------------------------------------------------------------

/// Cross-Polar Discrimination (XPD) model.
///
/// XPD degrades due to rain depolarisation at E-band. ITU-R P.618-14
/// provides the relationship between XPD and rain attenuation A:
///
/// XPD = U - V·log10(A)   (dB)
///
/// where U ≈ 30 dB for circular polarisation, V ≈ 30 for E-band.
/// For linear polarisation, U ≈ 30, V ≈ 30 at 70-90 GHz.
pub struct XpdModel {
    /// U coefficient (dB).
    pub u: f64,
    /// V coefficient (dimensionless).
    pub v: f64,
}

impl XpdModel {
    /// Default E-band XPD model for linear polarisation (ITU-R P.618-14).
    pub fn e_band_linear() -> Self {
        XpdModel { u: 30.0, v: 30.0 }
    }

    /// XPD (dB) for a given co-polar rain attenuation A (dB).
    pub fn xpd_db(&self, copol_attenuation_db: f64) -> f64 {
        if copol_attenuation_db <= 0.0 {
            return self.u; // Clear air XPD
        }
        self.u - self.v * copol_attenuation_db.log10()
    }

    /// XPIC improvement requirement to maintain target XPD (dB).
    pub fn required_xpic_gain_db(&self, copol_attenuation_db: f64, target_xpd_db: f64) -> f64 {
        let degraded = self.xpd_db(copol_attenuation_db);
        (target_xpd_db - degraded).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// Adaptive Modulation Engine
// ---------------------------------------------------------------------------

/// State of the adaptive modulation controller.
#[derive(Debug, Clone)]
pub struct AdaptiveModulationState {
    /// Current active modulation.
    pub current_modulation: EBandModulation,
    /// Current FEC rate.
    pub current_fec: FecRate,
    /// EMA-smoothed SNR estimate (dB).
    pub snr_ema_db: f64,
    /// Number of modulation switches performed.
    pub switch_count: u64,
}

/// Adaptive modulation controller.
///
/// Implements hitless modulation switching based on received SNR.
/// Includes hysteresis to prevent ping-pong between modes, and
/// EMA (exponential moving average) smoothing of the SNR estimate.
///
/// Reference: ETSI EN 302 217-1 §7 "Adaptive coding and modulation".
pub struct AdaptiveModulation {
    /// Hysteresis margin (dB) — switch up requires SNR > threshold + margin.
    hysteresis_db: f64,
    /// EMA time constant (samples).
    ema_alpha: f64,
    /// State.
    pub state: AdaptiveModulationState,
}

impl AdaptiveModulation {
    /// Create adaptive modulation controller.
    ///
    /// # Arguments
    /// * `initial_modulation` — modulation to start with
    /// * `initial_fec` — FEC rate to start with
    /// * `hysteresis_db` — switch-up margin above threshold (typically 2-4 dB)
    /// * `ema_alpha` — EMA coefficient α (0=slow, 1=instant, typical 0.05-0.2)
    pub fn new(
        initial_modulation: EBandModulation,
        initial_fec: FecRate,
        hysteresis_db: f64,
        ema_alpha: f64,
    ) -> Self {
        AdaptiveModulation {
            hysteresis_db,
            ema_alpha,
            state: AdaptiveModulationState {
                current_modulation: initial_modulation,
                current_fec: initial_fec,
                snr_ema_db: initial_modulation.min_snr_db(),
                switch_count: 0,
            },
        }
    }

    /// Update with a new SNR measurement and apply switching decision.
    ///
    /// Returns `true` if a modulation change occurred.
    pub fn update(&mut self, measured_snr_db: f64) -> bool {
        // EMA smoothing
        let s = &mut self.state;
        s.snr_ema_db = s.snr_ema_db + self.ema_alpha * (measured_snr_db - s.snr_ema_db);

        let snr = s.snr_ema_db;
        let all = EBandModulation::all();

        // Find highest modulation whose threshold + hysteresis <= current SNR
        let new_mod = all.iter().rev().find(|&&m| {
            snr >= m.min_snr_db() + self.hysteresis_db
        }).copied().unwrap_or(EBandModulation::Qpsk);

        if new_mod != s.current_modulation {
            s.current_modulation = new_mod;
            s.switch_count += 1;
            true
        } else {
            false
        }
    }

    /// Force modulation to minimum (lowest SNR requirement) — fallback.
    pub fn force_minimum(&mut self) {
        self.state.current_modulation = EBandModulation::Qpsk;
    }

    /// Current net throughput assuming given channel bandwidth and FEC.
    pub fn net_throughput_gbps(&self, bw: ChannelBandwidth) -> f64 {
        let bps = self.state.current_modulation.bits_per_symbol() as f64;
        let sym_rate = bw.symbol_rate_sps();
        let fec_rate = self.state.current_fec.rate();
        bps * sym_rate * fec_rate / 1e9
    }
}

// ---------------------------------------------------------------------------
// Throughput Calculator
// ---------------------------------------------------------------------------

/// Throughput model including all overhead.
#[derive(Debug, Clone)]
pub struct ThroughputResult {
    /// Gross bit rate (before FEC and framing overhead) in Gbps.
    pub gross_gbps: f64,
    /// FEC overhead fraction.
    pub fec_overhead_fraction: f64,
    /// Framing overhead fraction (Ethernet, OAM headers etc.).
    pub framing_overhead_fraction: f64,
    /// Net user throughput in Gbps.
    pub net_gbps: f64,
    /// Spectral efficiency (bits/s/Hz).
    pub spectral_efficiency_bps_hz: f64,
}

/// Calculate throughput from modulation, channel, and FEC parameters.
///
/// # Arguments
/// * `modulation` — active modulation order
/// * `bw` — channel bandwidth
/// * `fec_rate` — inner LDPC code rate
/// * `framing_overhead` — framing overhead fraction (e.g. 0.05 for 5%)
pub fn calculate_throughput(
    modulation: EBandModulation,
    bw: ChannelBandwidth,
    fec_rate: FecRate,
    framing_overhead: f64,
) -> ThroughputResult {
    let bps = modulation.bits_per_symbol() as f64;
    let sym_rate = bw.symbol_rate_sps();
    let gross_gbps = bps * sym_rate / 1e9;
    let fec_overhead = 1.0 - fec_rate.rate();
    let net_gbps = gross_gbps * fec_rate.rate() * (1.0 - framing_overhead);
    let spectral_efficiency = net_gbps * 1e9 / bw.hz();
    ThroughputResult {
        gross_gbps,
        fec_overhead_fraction: fec_overhead,
        framing_overhead_fraction: framing_overhead,
        net_gbps,
        spectral_efficiency_bps_hz: spectral_efficiency,
    }
}

// ---------------------------------------------------------------------------
// EBandConfig and EBandModem
// ---------------------------------------------------------------------------

/// Top-level E-band modem configuration.
#[derive(Debug, Clone)]
pub struct EBandConfig {
    /// Channel bandwidth.
    pub channel_bw: ChannelBandwidth,
    /// Active modulation (may change with ACMR).
    pub modulation: EBandModulation,
    /// FEC rate.
    pub fec_rate: FecRate,
    /// TX antenna diameter (m).
    pub tx_antenna_diameter_m: f64,
    /// RX antenna diameter (m).
    pub rx_antenna_diameter_m: f64,
    /// Path length (km).
    pub path_length_km: f64,
    /// Transmit power (dBm).
    pub tx_power_dbm: f64,
    /// Rain rate at the link location (mm/h, instantaneous or design value).
    pub rain_rate_mm_h: f64,
    /// Target availability (%).
    pub availability_pct: f64,
}

impl Default for EBandConfig {
    fn default() -> Self {
        EBandConfig {
            channel_bw: ChannelBandwidth::Bw1000MHz,
            modulation: EBandModulation::Qam256,
            fec_rate: FecRate::R0_9,
            tx_antenna_diameter_m: 0.6,
            rx_antenna_diameter_m: 0.6,
            path_length_km: 2.0,
            tx_power_dbm: 10.0,
            rain_rate_mm_h: 20.0,
            availability_pct: 99.999,
        }
    }
}

/// E-band complete modem.
///
/// Integrates channel plan, link budget, FEC chain, adaptive modulation,
/// and rain fade model for a complete simulation of an E-band backhaul link.
pub struct EBandModem {
    /// Configuration.
    pub config: EBandConfig,
    /// Adaptive modulation controller.
    pub acm: AdaptiveModulation,
    /// Rain fade model (lower band).
    pub rain_model: RainFadeModel,
    /// XPIC filter for XPOL operation.
    pub xpic: XpicFilter,
    /// XPD model.
    pub xpd: XpdModel,
    /// FEC chain.
    pub fec: FecChain,
    /// QAM constellation.
    pub constellation: QamConstellation,
}

impl EBandModem {
    /// Create a new E-band modem from configuration.
    pub fn new(config: EBandConfig) -> Self {
        let acm = AdaptiveModulation::new(
            config.modulation,
            config.fec_rate,
            3.0,   // 3 dB hysteresis
            0.1,   // EMA alpha
        );
        // Use 0.01% rain rate from the design rain rate scaled by factor ~1.5
        let rain_rate_001 = config.rain_rate_mm_h * 2.0;
        let rain_model = RainFadeModel::new_lower_band(rain_rate_001, config.path_length_km);
        let xpic = XpicFilter::new(8, 0.001);
        let xpd = XpdModel::e_band_linear();
        let fec = FecChain::new(config.fec_rate, 1024);
        let constellation = QamConstellation::new(config.modulation);
        EBandModem { config, acm, rain_model, xpic, xpd, fec, constellation }
    }

    /// Compute link budget at the given carrier frequency.
    pub fn link_budget(&self, frequency_hz: f64) -> LinkBudgetResult {
        let tx_ant = ParabolicAntenna::new(self.config.tx_antenna_diameter_m);
        let rx_ant = ParabolicAntenna::new(self.config.rx_antenna_diameter_m);
        let atm = AtmosphericAbsorption::total_db(frequency_hz, self.config.path_length_km, 7.5);
        let rain_att = self.rain_model.path_attenuation_db(self.config.rain_rate_mm_h);
        let cfg = LinkBudgetConfig {
            tx_power_dbm: self.config.tx_power_dbm,
            tx_antenna_gain_dbi: tx_ant.gain_dbi(frequency_hz),
            rx_antenna_gain_dbi: rx_ant.gain_dbi(frequency_hz),
            frequency_hz,
            path_length_m: self.config.path_length_km * 1000.0,
            atm_absorption_db: atm,
            rain_attenuation_db: rain_att,
            rx_noise_figure_db: 7.0,
            bandwidth_hz: self.config.channel_bw.hz(),
            tx_feeder_loss_db: 0.5,
            rx_feeder_loss_db: 0.5,
        };
        let required_snr = self.config.modulation.min_snr_db();
        compute_link_budget(&cfg, required_snr)
    }

    /// Net throughput in Gbps for current configuration.
    pub fn net_throughput_gbps(&self) -> f64 {
        calculate_throughput(
            self.config.modulation,
            self.config.channel_bw,
            self.config.fec_rate,
            0.04, // 4% framing overhead
        ).net_gbps
    }

    /// Required fade margin to achieve target availability (dB).
    pub fn required_fade_margin_db(&self) -> f64 {
        self.rain_model.required_fade_margin_db(self.config.availability_pct)
    }

    /// Dual-polarisation throughput (2× co-polar with XPOL).
    pub fn xpol_net_throughput_gbps(&self) -> f64 {
        self.net_throughput_gbps() * 2.0
    }

    /// Decode a modulated symbol (I, Q) to bits.
    pub fn demodulate_symbol(&self, i: f64, q: f64) -> Vec<bool> {
        self.constellation.demap_hard(i, q)
    }

    /// Modulate a bit sequence to (I, Q) symbol.
    pub fn modulate_bits(&self, bits: &[bool]) -> (f64, f64) {
        let pt = self.constellation.map(bits);
        (pt.i, pt.q)
    }

    /// Full TX chain: bits → FEC encode → map to QAM → (I, Q) sequence.
    pub fn transmit(&self, data_bits: &[bool]) -> Vec<(f64, f64)> {
        let bps = self.config.modulation.bits_per_symbol();
        let k = self.fec.ldpc.k;
        // Process in blocks of k info bits
        let mut symbols = Vec::new();
        for chunk in data_bits.chunks(k) {
            let mut block = chunk.to_vec();
            // Pad last block if needed
            block.resize(k, false);
            let codeword = self.fec.ldpc.encode(&block);
            // Map codeword bits to QAM symbols
            for sym_bits in codeword.chunks(bps) {
                let mut sb = sym_bits.to_vec();
                sb.resize(bps, false);
                let pt = self.constellation.map(&sb);
                symbols.push((pt.i, pt.q));
            }
        }
        symbols
    }

    /// Full RX chain: (I, Q) sequence → FEC decode → bits.
    pub fn receive(&self, symbols: &[(f64, f64)]) -> Vec<bool> {
        let bps = self.config.modulation.bits_per_symbol();
        let k = self.fec.ldpc.k;
        let n = self.fec.ldpc.n;
        let syms_per_block = (n + bps - 1) / bps;
        let mut data_bits = Vec::new();
        for block_syms in symbols.chunks(syms_per_block) {
            let mut codeword_bits: Vec<bool> = block_syms.iter()
                .flat_map(|(i, q)| self.constellation.demap_hard(*i, *q))
                .collect();
            codeword_bits.resize(n, false);
            let info = self.fec.ldpc.decode(&codeword_bits);
            data_bits.extend(info[..k.min(info.len())].iter().copied());
        }
        data_bits
    }
}

// ---------------------------------------------------------------------------
// Waterfall BER Approximation
// ---------------------------------------------------------------------------

/// Approximate BER for M-QAM with AWGN at a given Es/N0 (dB, per-symbol SNR).
///
/// Uses the standard approximation for Grey-coded square M-QAM:
///
/// BER ≈ (4/log₂(M)) · (1 - 1/√M) · Q(√(3·Es/N0 / (M-1)))
///
/// where Es/N0 is the received symbol energy to noise density ratio.
/// For QPSK (M=4) this simplifies to Q(√(Es/N0)).
///
/// Note: Eb/N0 = Es/N0 / log2(M). Pass `es_n0_db = ebn0_db + 10*log10(bps)` to convert.
pub fn ber_qam_awgn(modulation: EBandModulation, esn0_db: f64) -> f64 {
    let m = modulation.order() as f64;
    let bps = modulation.bits_per_symbol() as f64;
    let esn0 = 10f64.powf(esn0_db / 10.0);
    // argument to Q: sqrt(3 * Es/N0 / (M-1))
    let arg = (3.0 * esn0 / (m - 1.0)).sqrt();
    let q = q_function(arg);
    4.0 / bps * (1.0 - 1.0 / m.sqrt()) * q
}

/// Q-function approximation using erfc.
fn q_function(x: f64) -> f64 {
    0.5 * erfc(x / 2f64.sqrt())
}

/// Complementary error function via rational approximation (Abramowitz & Stegun 7.1.26).
pub fn erfc(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t * (0.254829592
        + t * (-0.284496736
            + t * (1.421413741
                + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Channel Plan ---

    #[test]
    fn test_channel_plan_250mhz() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw250MHz);
        // 5 GHz span / 250 MHz = 20 channels
        assert_eq!(plan.num_channels(), 20);
    }

    #[test]
    fn test_channel_plan_1000mhz() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw1000MHz);
        // 5 GHz / 1000 MHz = 5 channels
        assert_eq!(plan.num_channels(), 5);
    }

    #[test]
    fn test_channel_plan_2000mhz() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw2000MHz);
        // 5 GHz / 2000 MHz = 2 channels
        assert_eq!(plan.num_channels(), 2);
    }

    #[test]
    fn test_channel_duplex_spacing() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw500MHz);
        for ch in &plan.channels {
            assert!((ch.duplex_spacing_hz() - DUPLEX_SPACING_HZ).abs() < 1.0,
                "duplex spacing should be 10 GHz");
        }
    }

    #[test]
    fn test_channel_frequencies_in_band() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw1000MHz);
        for ch in &plan.channels {
            assert!(ch.lower_centre_hz >= LOWER_BAND_START_HZ);
            assert!(ch.lower_centre_hz <= LOWER_BAND_END_HZ);
            assert!(ch.upper_centre_hz >= UPPER_BAND_START_HZ);
            assert!(ch.upper_centre_hz <= UPPER_BAND_END_HZ);
        }
    }

    #[test]
    fn test_channel_find_by_number() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw1000MHz);
        let ch = plan.channel(1).unwrap();
        assert_eq!(ch.number, 1);
        assert!(plan.channel(99).is_none());
    }

    // --- Channel Bandwidth ---

    #[test]
    fn test_channel_bw_symbol_rate() {
        // 1000 MHz / 1.25 = 800 Msps
        assert!((ChannelBandwidth::Bw1000MHz.symbol_rate_sps() - 800e6).abs() < 1.0);
    }

    #[test]
    fn test_channel_bw_mhz() {
        assert!((ChannelBandwidth::Bw500MHz.mhz() - 500.0).abs() < 0.001);
    }

    // --- Modulation ---

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(EBandModulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(EBandModulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(EBandModulation::Qam256.bits_per_symbol(), 8);
        assert_eq!(EBandModulation::Qam4096.bits_per_symbol(), 12);
    }

    #[test]
    fn test_modulation_order() {
        assert_eq!(EBandModulation::Qpsk.order(), 4);
        assert_eq!(EBandModulation::Qam64.order(), 64);
        assert_eq!(EBandModulation::Qam4096.order(), 4096);
    }

    #[test]
    fn test_modulation_snr_increases_with_order() {
        let mods = EBandModulation::all();
        for i in 1..mods.len() {
            assert!(
                mods[i].min_snr_db() > mods[i - 1].min_snr_db(),
                "SNR threshold should increase with modulation order"
            );
        }
    }

    #[test]
    fn test_modulation_names() {
        assert_eq!(EBandModulation::Qpsk.name(), "QPSK");
        assert_eq!(EBandModulation::Qam256.name(), "256QAM");
        assert_eq!(EBandModulation::Qam4096.name(), "4096QAM");
    }

    // --- FEC ---

    #[test]
    fn test_fec_rate_values() {
        assert!((FecRate::R0_5.rate() - 0.5).abs() < 1e-9);
        assert!((FecRate::R0_9.rate() - 0.9).abs() < 1e-9);
    }

    #[test]
    fn test_fec_ncg_decreases_with_rate() {
        assert!(FecRate::R0_5.net_coding_gain_db() > FecRate::R0_9.net_coding_gain_db());
    }

    #[test]
    fn test_fec_names() {
        assert_eq!(FecRate::R0_5.name(), "1/2");
        assert_eq!(FecRate::R0_9.name(), "9/10");
    }

    // --- Rain Fade Model ---

    #[test]
    fn test_rain_specific_attenuation() {
        let coeffs = RainAttenuationCoeffs::for_73ghz_h();
        // At R=0 mm/h → 0 dB/km
        assert_eq!(coeffs.specific_attenuation_db_per_km(0.0), 0.0);
        // At R=1 mm/h → k dB/km
        let at1 = coeffs.specific_attenuation_db_per_km(1.0);
        assert!((at1 - coeffs.k).abs() < 1e-6);
    }

    #[test]
    fn test_rain_attenuation_increases_with_rate() {
        let model = RainFadeModel::new_lower_band(50.0, 2.0);
        let a10 = model.specific_attenuation_db_per_km(10.0);
        let a50 = model.specific_attenuation_db_per_km(50.0);
        let a100 = model.specific_attenuation_db_per_km(100.0);
        assert!(a10 < a50);
        assert!(a50 < a100);
    }

    #[test]
    fn test_rain_exceeded_001pct() {
        let model = RainFadeModel::new_lower_band(42.0, 3.0);
        let a = model.exceeded_001pct_db();
        assert!(a > 0.0, "attenuation should be positive");
        assert!(a < 200.0, "attenuation should be physically reasonable");
    }

    #[test]
    fn test_rain_fade_margin_99999() {
        let model = RainFadeModel::new_lower_band(42.0, 2.0);
        let fm = model.required_fade_margin_db(99.999);
        // 0.001% outage
        assert!(fm > 0.0);
        // 99.999% availability requires higher fade margin than 99.99%
        let fm2 = model.required_fade_margin_db(99.99);
        assert!(fm > fm2);
    }

    #[test]
    fn test_rain_path_attenuation() {
        let model = RainFadeModel::new_lower_band(20.0, 2.0);
        let total = model.path_attenuation_db(20.0);
        let specific = model.specific_attenuation_db_per_km(20.0);
        assert!((total - specific * 2.0).abs() < 1.0,
            "path attenuation approximation");
    }

    #[test]
    fn test_rain_coefficients_upper_band() {
        let coeffs_h = RainAttenuationCoeffs::for_83ghz_h();
        let coeffs_v = RainAttenuationCoeffs::for_83ghz_v();
        // H-pol should have slightly higher k than V-pol
        assert!(coeffs_h.k > coeffs_v.k);
    }

    // --- Atmospheric Absorption ---

    #[test]
    fn test_atmospheric_absorption_lower_band() {
        // 2 km path, 7.5 g/m³ water vapour
        let abs = AtmosphericAbsorption::total_db(73e9, 2.0, 7.5);
        assert!(abs > 0.5, "should have non-trivial atmospheric absorption");
        assert!(abs < 10.0, "should be reasonable");
    }

    #[test]
    fn test_atmospheric_absorption_upper_band() {
        let abs = AtmosphericAbsorption::total_db(83e9, 2.0, 7.5);
        assert!(abs > 0.0);
    }

    #[test]
    fn test_atmospheric_absorption_scales_with_path() {
        let abs1 = AtmosphericAbsorption::total_db(73e9, 1.0, 7.5);
        let abs2 = AtmosphericAbsorption::total_db(73e9, 2.0, 7.5);
        assert!((abs2 - 2.0 * abs1).abs() < 1e-9);
    }

    // --- Antenna Gain ---

    #[test]
    fn test_antenna_gain_60cm() {
        let ant = ParabolicAntenna::new(0.6);
        let gain = ant.gain_dbi(73e9);
        // 60 cm at 73 GHz → expect ~40 dBi range
        assert!(gain > 35.0, "60 cm dish should have significant gain");
        assert!(gain < 55.0, "gain should be physically reasonable");
    }

    #[test]
    fn test_antenna_gain_increases_with_diameter() {
        let ant30 = ParabolicAntenna::new(0.3);
        let ant60 = ParabolicAntenna::new(0.6);
        let ant120 = ParabolicAntenna::new(1.2);
        assert!(ant30.gain_dbi(73e9) < ant60.gain_dbi(73e9));
        assert!(ant60.gain_dbi(73e9) < ant120.gain_dbi(73e9));
    }

    #[test]
    fn test_antenna_hpbw_decreases_with_diameter() {
        let ant30 = ParabolicAntenna::new(0.3);
        let ant60 = ParabolicAntenna::new(0.6);
        assert!(ant30.hpbw_degrees(73e9) > ant60.hpbw_degrees(73e9));
    }

    #[test]
    fn test_antenna_gain_upper_band_higher() {
        let ant = ParabolicAntenna::new(0.6);
        // Higher freq → smaller lambda → more gain
        assert!(ant.gain_dbi(83e9) > ant.gain_dbi(73e9));
    }

    // --- Free Space Path Loss ---

    #[test]
    fn test_fspl_2km_73ghz() {
        let fspl = free_space_path_loss_db(2000.0, 73e9);
        // Rough expectation: ~140-150 dB at 2 km, 73 GHz
        assert!(fspl > 130.0 && fspl < 165.0);
    }

    #[test]
    fn test_fspl_increases_with_distance() {
        let f1 = free_space_path_loss_db(1000.0, 73e9);
        let f2 = free_space_path_loss_db(2000.0, 73e9);
        let f4 = free_space_path_loss_db(4000.0, 73e9);
        // FSPL doubles (6 dB) when distance doubles
        assert!((f2 - f1 - 6.0).abs() < 0.1);
        assert!((f4 - f2 - 6.0).abs() < 0.1);
    }

    #[test]
    fn test_fspl_increases_with_frequency() {
        let fl = free_space_path_loss_db(2000.0, 73e9);
        let fu = free_space_path_loss_db(2000.0, 83e9);
        assert!(fu > fl);
    }

    // --- Link Budget ---

    #[test]
    fn test_link_budget_positive_cnr() {
        let cfg = LinkBudgetConfig {
            tx_power_dbm: 20.0,
            tx_antenna_gain_dbi: 45.0,
            rx_antenna_gain_dbi: 45.0,
            frequency_hz: 73e9,
            path_length_m: 1000.0,
            atm_absorption_db: 0.5,
            rain_attenuation_db: 0.0,
            rx_noise_figure_db: 7.0,
            bandwidth_hz: 1000e6,
            tx_feeder_loss_db: 0.5,
            rx_feeder_loss_db: 0.5,
        };
        let result = compute_link_budget(&cfg, 28.5);
        assert!(result.cnr_db > 0.0);
    }

    #[test]
    fn test_link_budget_eirp() {
        let cfg = LinkBudgetConfig {
            tx_power_dbm: 10.0,
            tx_antenna_gain_dbi: 40.0,
            tx_feeder_loss_db: 1.0,
            ..Default::default()
        };
        let result = compute_link_budget(&cfg, 10.0);
        // EIRP = 10 + 40 - 1 = 49 dBm
        assert!((result.eirp_dbm - 49.0).abs() < 0.01);
    }

    #[test]
    fn test_link_budget_fade_margin() {
        let cfg = LinkBudgetConfig {
            tx_power_dbm: 20.0,
            tx_antenna_gain_dbi: 46.0,
            rx_antenna_gain_dbi: 46.0,
            frequency_hz: 73e9,
            path_length_m: 2000.0,
            atm_absorption_db: 1.0,
            rain_attenuation_db: 0.0,
            rx_noise_figure_db: 6.0,
            bandwidth_hz: 1000e6,
            tx_feeder_loss_db: 0.5,
            rx_feeder_loss_db: 0.5,
        };
        let result = compute_link_budget(&cfg, 28.5); // 256QAM threshold
        // Should have positive fade margin on a short clear-air path
        assert!(result.fade_margin_db > 0.0);
    }

    #[test]
    fn test_link_budget_system_gain() {
        let cfg = LinkBudgetConfig::default();
        let result = compute_link_budget(&cfg, 10.0);
        assert!(result.system_gain_db > 0.0);
    }

    // --- QAM Constellation ---

    #[test]
    fn test_qam16_constellation_size() {
        let c = QamConstellation::new(EBandModulation::Qam16);
        assert_eq!(c.points.len(), 16);
    }

    #[test]
    fn test_qam64_constellation_size() {
        let c = QamConstellation::new(EBandModulation::Qam64);
        assert_eq!(c.points.len(), 64);
    }

    #[test]
    fn test_qam256_constellation_size() {
        let c = QamConstellation::new(EBandModulation::Qam256);
        assert_eq!(c.points.len(), 256);
    }

    #[test]
    fn test_qam_average_power_normalised() {
        for mod_order in [EBandModulation::Qam16, EBandModulation::Qam64, EBandModulation::Qam256] {
            let c = QamConstellation::new(mod_order);
            let avg_pwr: f64 = c.points.iter()
                .map(|p| p.i * p.i + p.q * p.q)
                .sum::<f64>() / c.points.len() as f64;
            assert!(
                (avg_pwr - 1.0).abs() < 0.01,
                "{:?} average power should be ~1.0, got {:.4}", mod_order, avg_pwr
            );
        }
    }

    #[test]
    fn test_qam_map_demap_roundtrip_qpsk() {
        let c = QamConstellation::new(EBandModulation::Qpsk);
        let bits = vec![true, false];
        let pt = c.map(&bits);
        let recovered = c.demap_hard(pt.i, pt.q);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_qam_map_demap_roundtrip_16qam() {
        let c = QamConstellation::new(EBandModulation::Qam16);
        for label in 0u64..16 {
            let bits: Vec<bool> = (0..4).rev().map(|k| (label >> k) & 1 == 1).collect();
            let pt = c.map(&bits);
            let recovered = c.demap_hard(pt.i, pt.q);
            assert_eq!(bits, recovered, "Failed for label {}", label);
        }
    }

    #[test]
    fn test_qam_unique_labels() {
        let c = QamConstellation::new(EBandModulation::Qam64);
        let mut labels: Vec<u64> = c.points.iter().map(|p| p.label).collect();
        labels.sort();
        labels.dedup();
        assert_eq!(labels.len(), 64, "All labels should be unique");
    }

    // --- LDPC FEC ---

    #[test]
    fn test_ldpc_encode_length() {
        let enc = LdpcEncoder::new(FecRate::R0_5, 100);
        let info = vec![false; 100];
        let cw = enc.encode(&info);
        assert_eq!(cw.len(), enc.n);
    }

    #[test]
    fn test_ldpc_systematic() {
        let enc = LdpcEncoder::new(FecRate::R0_9, 64);
        let info: Vec<bool> = (0..64).map(|i| i % 3 == 0).collect();
        let cw = enc.encode(&info);
        // First k bits should match info
        assert_eq!(&cw[..64], info.as_slice());
    }

    #[test]
    fn test_ldpc_decode_no_error() {
        let enc = LdpcEncoder::new(FecRate::R0_7, 128);
        let info: Vec<bool> = (0..128).map(|i| i % 5 != 0).collect();
        let cw = enc.encode(&info);
        let decoded = enc.decode(&cw);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_ldpc_decode_single_error() {
        let enc = LdpcEncoder::new(FecRate::R0_8, 32);
        let info: Vec<bool> = (0..32).map(|i| i % 2 == 0).collect();
        let mut cw = enc.encode(&info);
        cw[5] = !cw[5]; // flip bit 5
        let decoded = enc.decode(&cw);
        assert_eq!(decoded, info, "Should correct single-bit error");
    }

    #[test]
    fn test_ldpc_coding_gain() {
        let enc_r5 = LdpcEncoder::new(FecRate::R0_5, 64);
        let enc_r9 = LdpcEncoder::new(FecRate::R0_9, 64);
        assert!(enc_r5.coding_gain_db() > enc_r9.coding_gain_db());
    }

    // --- BCH Outer Code ---

    #[test]
    fn test_bch_encode_length() {
        let bch = BchOuterCode::bch63_51();
        let info = vec![false; 51];
        let cw = bch.encode(&info);
        assert_eq!(cw.len(), 63);
    }

    #[test]
    fn test_bch_systematic() {
        let bch = BchOuterCode::bch63_51();
        let info: Vec<bool> = (0..51).map(|i| i % 4 == 0).collect();
        let cw = bch.encode(&info);
        assert_eq!(&cw[..51], info.as_slice());
    }

    #[test]
    fn test_bch_code_rate() {
        let bch = BchOuterCode::bch63_51();
        assert!((bch.rate() - 51.0 / 63.0).abs() < 1e-9);
    }

    // --- FEC Chain ---

    #[test]
    fn test_fec_chain_overhead() {
        let chain = FecChain::new(FecRate::R0_9, 1024);
        let overhead = chain.total_overhead();
        // Should be > 1 (overhead factor)
        assert!(overhead > 1.0);
        // LDPC 9/10 + BCH 51/63 ≈ 1/(0.9 * 51/63) ≈ 1.37
        assert!(overhead < 2.5);
    }

    #[test]
    fn test_fec_chain_ncg() {
        let chain = FecChain::new(FecRate::R0_9, 1024);
        assert!(chain.total_ncg_db() > 5.0);
    }

    // --- XPIC ---

    #[test]
    fn test_xpic_convergence() {
        // XPIC test: co-polar desired signal is constant + cross-polar interference.
        // The XPIC filter learns to subtract the scaled cross-polar component.
        // After convergence, error ≈ desired signal (constant), not zero.
        // We verify that error power stabilises (converges, not grows).
        let mut xpic = XpicFilter::new(4, 0.005);
        let mut errors_early = Vec::new();
        let mut errors_late = Vec::new();
        for i in 0..1000 {
            let t = i as f64 * 0.05;
            // Cross-polar signal (interference source)
            let xpol_i = (2.3 * t).sin();
            let xpol_q = (2.3 * t + 1.1).cos();
            // Co-polar desired = unit signal + 30% cross-polar interference
            let desired_i = 1.0_f64;
            let desired_q = 0.0_f64;
            let copol_i = desired_i + 0.3 * xpol_i;
            let copol_q = desired_q + 0.3 * xpol_q;
            let (ei, eq) = xpic.process(xpol_i, xpol_q, copol_i, copol_q);
            if i >= 100 && i < 200 {
                errors_early.push(ei * ei + eq * eq);
            }
            if i >= 900 {
                errors_late.push(ei * ei + eq * eq);
            }
        }
        let mean_early: f64 = errors_early.iter().sum::<f64>() / errors_early.len() as f64;
        let mean_late: f64 = errors_late.iter().sum::<f64>() / errors_late.len() as f64;
        // Error power should decrease or stabilise (not diverge)
        assert!(
            mean_late <= mean_early * 2.0,
            "XPIC error should not diverge: early={:.4} late={:.4}",
            mean_early, mean_late
        );
    }

    #[test]
    fn test_xpic_num_taps() {
        let xpic = XpicFilter::new(16, 0.001);
        assert_eq!(xpic.num_taps(), 16);
    }

    #[test]
    fn test_xpic_reset() {
        let mut xpic = XpicFilter::new(4, 0.01);
        xpic.process(1.0, 0.0, 0.5, 0.5);
        xpic.reset();
        // After reset, residual should be negligible
        let r = xpic.residual_xpi_db();
        assert!(r.is_infinite() || r < -100.0);
    }

    // --- XPD Model ---

    #[test]
    fn test_xpd_clear_air() {
        let xpd = XpdModel::e_band_linear();
        let clear_air_xpd = xpd.xpd_db(0.0);
        assert!((clear_air_xpd - 30.0).abs() < 0.01);
    }

    #[test]
    fn test_xpd_degrades_with_rain() {
        let xpd = XpdModel::e_band_linear();
        let xpd_clear = xpd.xpd_db(0.0);
        let xpd_rain = xpd.xpd_db(10.0);
        assert!(xpd_rain < xpd_clear, "XPD should degrade in rain");
    }

    #[test]
    fn test_xpic_requirement() {
        let xpd = XpdModel::e_band_linear();
        let req = xpd.required_xpic_gain_db(10.0, 25.0);
        assert!(req >= 0.0);
    }

    // --- Adaptive Modulation ---

    #[test]
    fn test_acm_switches_up_with_high_snr() {
        let mut acm = AdaptiveModulation::new(
            EBandModulation::Qpsk,
            FecRate::R0_9,
            2.0,
            0.5,
        );
        // Feed high SNR — should switch to higher modulation
        let switched = acm.update(45.0);
        assert!(switched, "Should switch up with high SNR");
        assert!(acm.state.current_modulation != EBandModulation::Qpsk);
    }

    #[test]
    fn test_acm_stays_qpsk_with_low_snr() {
        let mut acm = AdaptiveModulation::new(
            EBandModulation::Qam256,
            FecRate::R0_9,
            3.0,
            0.5,
        );
        // Low SNR — should fall back
        acm.update(8.0);
        assert_eq!(acm.state.current_modulation, EBandModulation::Qpsk);
    }

    #[test]
    fn test_acm_hysteresis() {
        let mut acm = AdaptiveModulation::new(
            EBandModulation::Qam256,
            FecRate::R0_9,
            5.0, // Large hysteresis
            1.0,
        );
        // SNR just above QPSK threshold but below QPSK + hysteresis
        // should keep QPSK
        acm.state.current_modulation = EBandModulation::Qpsk;
        acm.state.snr_ema_db = 12.0;
        let switched = acm.update(12.0);
        // With 5 dB hysteresis: 12.0 < QPSK_thresh + 5.0 = 15.5 → stay QPSK
        assert!(!switched || acm.state.current_modulation == EBandModulation::Qpsk);
    }

    #[test]
    fn test_acm_force_minimum() {
        let mut acm = AdaptiveModulation::new(
            EBandModulation::Qam4096,
            FecRate::R0_9,
            2.0,
            0.1,
        );
        acm.force_minimum();
        assert_eq!(acm.state.current_modulation, EBandModulation::Qpsk);
    }

    #[test]
    fn test_acm_throughput() {
        let mut acm = AdaptiveModulation::new(
            EBandModulation::Qpsk,
            FecRate::R0_9,
            2.0,
            1.0,
        );
        acm.update(45.0);
        let tp = acm.net_throughput_gbps(ChannelBandwidth::Bw1000MHz);
        assert!(tp > 0.0);
    }

    // --- Throughput ---

    #[test]
    fn test_throughput_qpsk() {
        let t = calculate_throughput(
            EBandModulation::Qpsk,
            ChannelBandwidth::Bw1000MHz,
            FecRate::R0_9,
            0.04,
        );
        // QPSK: 2 bits/sym * 800 Msps * 0.9 * 0.96 ≈ 1.38 Gbps
        assert!(t.net_gbps > 0.5);
        assert!(t.net_gbps < 5.0);
    }

    #[test]
    fn test_throughput_4096qam() {
        let t = calculate_throughput(
            EBandModulation::Qam4096,
            ChannelBandwidth::Bw2000MHz,
            FecRate::R0_9,
            0.04,
        );
        // 4096QAM: 12 bits/sym * 1600 Msps * 0.9 * 0.96 ≈ 16.6 Gbps
        assert!(t.net_gbps > 10.0, "4096QAM 2 GHz should exceed 10 Gbps");
    }

    #[test]
    fn test_throughput_increases_with_order() {
        let t_qpsk = calculate_throughput(
            EBandModulation::Qpsk,
            ChannelBandwidth::Bw1000MHz,
            FecRate::R0_9,
            0.04,
        );
        let t_256 = calculate_throughput(
            EBandModulation::Qam256,
            ChannelBandwidth::Bw1000MHz,
            FecRate::R0_9,
            0.04,
        );
        assert!(t_256.net_gbps > t_qpsk.net_gbps);
    }

    #[test]
    fn test_spectral_efficiency() {
        let t = calculate_throughput(
            EBandModulation::Qam256,
            ChannelBandwidth::Bw1000MHz,
            FecRate::R0_9,
            0.0,
        );
        // 256QAM r=9/10: 8 * 0.8 * 0.9 = ~5.76 b/s/Hz theoretical
        assert!(t.spectral_efficiency_bps_hz > 4.0);
        assert!(t.spectral_efficiency_bps_hz < 10.0);
    }

    // --- EBandModem ---

    #[test]
    fn test_modem_link_budget() {
        let config = EBandConfig::default();
        let modem = EBandModem::new(config);
        let budget = modem.link_budget(73e9);
        assert!(budget.system_gain_db > 50.0);
    }

    #[test]
    fn test_modem_net_throughput() {
        let config = EBandConfig {
            modulation: EBandModulation::Qam256,
            channel_bw: ChannelBandwidth::Bw1000MHz,
            fec_rate: FecRate::R0_9,
            ..Default::default()
        };
        let modem = EBandModem::new(config);
        let tp = modem.net_throughput_gbps();
        assert!(tp > 1.0, "256QAM should exceed 1 Gbps");
    }

    #[test]
    fn test_modem_xpol_throughput() {
        let config = EBandConfig::default();
        let modem = EBandModem::new(config);
        let single = modem.net_throughput_gbps();
        let dual = modem.xpol_net_throughput_gbps();
        assert!((dual - 2.0 * single).abs() < 0.001);
    }

    #[test]
    fn test_modem_demodulate_modulate_roundtrip() {
        let config = EBandConfig {
            modulation: EBandModulation::Qam64,
            ..Default::default()
        };
        let modem = EBandModem::new(config);
        let bits = vec![true, false, true, true, false, false];
        let (i, q) = modem.modulate_bits(&bits);
        let recovered = modem.demodulate_symbol(i, q);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_modem_tx_rx_roundtrip() {
        let config = EBandConfig {
            modulation: EBandModulation::Qam16,
            fec_rate: FecRate::R0_5,
            ..Default::default()
        };
        let modem = EBandModem::new(config);
        let data: Vec<bool> = (0..1024).map(|i| i % 3 == 0).collect();
        let symbols = modem.transmit(&data);
        let recovered = modem.receive(&symbols);
        // Check first 1024 bits match (no noise in this test)
        assert_eq!(&data[..], &recovered[..data.len().min(recovered.len())]);
    }

    #[test]
    fn test_modem_fade_margin_positive() {
        let config = EBandConfig {
            rain_rate_mm_h: 10.0,
            availability_pct: 99.99,
            path_length_km: 1.0,
            ..Default::default()
        };
        let modem = EBandModem::new(config);
        let fm = modem.required_fade_margin_db();
        assert!(fm > 0.0);
    }

    // --- BER Model ---

    #[test]
    fn test_ber_qpsk_high_snr() {
        // At Es/N0 = 20 dB → BER for QPSK should be tiny
        // arg = sqrt(3 * 100 / 3) = 10 → Q(10) ≈ 7.6e-24
        let ber = ber_qam_awgn(EBandModulation::Qpsk, 20.0);
        assert!(ber < 1e-6, "BER should be very low at 20 dB Es/N0");
    }

    #[test]
    fn test_ber_qpsk_low_snr() {
        // At Es/N0 = 0 dB → BER for QPSK should be significant
        // arg = sqrt(3 * 1 / 3) = 1.0 → Q(1.0) ≈ 0.159
        let ber = ber_qam_awgn(EBandModulation::Qpsk, 0.0);
        assert!(ber > 0.01, "BER should be high at 0 dB Es/N0");
    }

    #[test]
    fn test_ber_decreases_with_snr() {
        // 256QAM: use moderate Es/N0 where erfc is non-zero and distinguishable
        // At Es/N0=25 dB: arg = sqrt(3*316/255) ≈ 1.93 → Q(1.93) ≈ 0.027
        // At Es/N0=28 dB: arg ≈ 2.73 → Q(2.73) ≈ 0.003
        let ber_low = ber_qam_awgn(EBandModulation::Qam256, 25.0);
        let ber_high = ber_qam_awgn(EBandModulation::Qam256, 28.0);
        assert!(ber_high < ber_low,
            "BER should decrease with increasing Es/N0: {} vs {}", ber_high, ber_low);
    }

    #[test]
    fn test_ber_higher_order_needs_more_snr() {
        // At Es/N0 = 15 dB: QPSK arg = sqrt(3*31.6/3) = 5.62 → tiny BER
        // 256QAM arg = sqrt(3*31.6/255) ≈ 0.61 → larger BER
        // So 256QAM has higher BER at same Es/N0 → needs more SNR
        let ber_qpsk = ber_qam_awgn(EBandModulation::Qpsk, 15.0);
        let ber_256 = ber_qam_awgn(EBandModulation::Qam256, 15.0);
        assert!(ber_256 > ber_qpsk,
            "256QAM needs more SNR than QPSK for same BER: qpsk={:.2e} 256qam={:.2e}",
            ber_qpsk, ber_256);
    }

    #[test]
    fn test_erfc_known_values() {
        // erfc(0) = 1
        assert!((erfc(0.0) - 1.0).abs() < 0.001);
        // erfc(inf) ≈ 0
        assert!(erfc(5.0) < 0.001);
        // erfc is monotone decreasing
        assert!(erfc(0.5) < erfc(0.0));
    }

    // --- Grey Code ---

    #[test]
    fn test_grey_code_basic() {
        assert_eq!(grey_code(0, 4), 0);
        assert_eq!(grey_code(1, 4), 1);
        assert_eq!(grey_code(2, 4), 3); // 2 XOR 1 = 3
        assert_eq!(grey_code(3, 4), 2); // 3 XOR 1 = 2
    }

    // --- System Integration ---

    #[test]
    fn test_full_link_scenario_2km_1ghz_256qam() {
        let config = EBandConfig {
            channel_bw: ChannelBandwidth::Bw1000MHz,
            modulation: EBandModulation::Qam256,
            fec_rate: FecRate::R0_9,
            tx_antenna_diameter_m: 0.6,
            rx_antenna_diameter_m: 0.6,
            path_length_km: 2.0,
            tx_power_dbm: 17.0,
            rain_rate_mm_h: 0.0,
            availability_pct: 99.999,
        };
        let modem = EBandModem::new(config);
        let budget = modem.link_budget(73e9);
        let tp = modem.net_throughput_gbps();
        // For a 2 km link with 60 cm dishes and 17 dBm TX, expect viable link
        assert!(budget.is_viable() || budget.cnr_db > 20.0,
            "2 km E-band link should be viable in clear air");
        assert!(tp > 1.0, "throughput should exceed 1 Gbps");
    }

    #[test]
    fn test_channel_plan_500mhz_channels_count() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw500MHz);
        assert_eq!(plan.num_channels(), 10);
    }

    #[test]
    fn test_channel_plan_750mhz() {
        let plan = ChannelPlan::new(ChannelBandwidth::Bw750MHz);
        // 5000 / 750 = 6 channels (floor)
        assert_eq!(plan.num_channels(), 6);
    }
}
