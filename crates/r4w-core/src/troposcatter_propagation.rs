//! Tropospheric scatter (troposcatter) propagation modeling and simulation.
//!
//! This module models beyond-line-of-sight communication via scattering from
//! turbulent regions of the troposphere. It includes path loss computation,
//! scatter volume geometry, atmospheric refractivity profiles, multipath
//! spread estimation, fade margin calculation, time diversity gain, and
//! aperture-to-medium coupling loss.
//!
//! # Background
//!
//! Troposcatter links exploit forward scattering from refractive-index
//! turbulence in the common volume where the transmit and receive antenna
//! beams overlap. Typical link distances range from 100 km to 800 km at
//! frequencies from 0.3 GHz to 10 GHz. The technique was widely used by
//! military networks (AN/TRC-170, USAF 486L) and is still employed today.
//!
//! # Example
//!
//! ```
//! use r4w_core::troposcatter_propagation::{TroposcatterPropagation, AtmosphericProfile};
//!
//! let tropo = TroposcatterPropagation::new(
//!     5.0e9,          // 5 GHz carrier frequency
//!     300_000.0,      // 300 km path distance
//!     40.0,           // TX antenna gain dBi
//!     40.0,           // RX antenna gain dBi
//!     3.0,            // TX antenna diameter in metres
//!     3.0,            // RX antenna diameter in metres
//!     AtmosphericProfile::Standard,
//! );
//!
//! let loss = tropo.total_path_loss_db();
//! assert!(loss > 150.0 && loss < 250.0, "typical troposcatter loss at 300 km: {loss} dB");
//!
//! let vol = tropo.scatter_volume();
//! assert!(vol.height_m > 0.0);
//! assert!(vol.volume_m3 > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Atmospheric profile
// ---------------------------------------------------------------------------

/// Atmospheric refractivity profile selector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AtmosphericProfile {
    /// ITU standard atmosphere (Ns = 315 N-units, gradient = -39 N/km).
    Standard,
    /// Super-refractive / ducting conditions (Ns = 350, gradient = -60 N/km).
    SuperRefractive,
    /// Sub-refractive conditions (Ns = 280, gradient = -20 N/km).
    SubRefractive,
    /// Custom profile with (surface refractivity N-units, gradient N/km).
    Custom {
        surface_refractivity: f64,
        gradient_n_per_km: f64,
    },
}

impl AtmosphericProfile {
    /// Surface refractivity Ns in N-units.
    pub fn surface_refractivity(&self) -> f64 {
        match self {
            Self::Standard => 315.0,
            Self::SuperRefractive => 350.0,
            Self::SubRefractive => 280.0,
            Self::Custom { surface_refractivity, .. } => *surface_refractivity,
        }
    }

    /// Vertical gradient of refractivity in N-units per km (negative = normal).
    pub fn gradient_n_per_km(&self) -> f64 {
        match self {
            Self::Standard => -39.0,
            Self::SuperRefractive => -60.0,
            Self::SubRefractive => -20.0,
            Self::Custom { gradient_n_per_km, .. } => *gradient_n_per_km,
        }
    }

    /// Refractivity at a given height above ground in metres.
    pub fn refractivity_at_height(&self, height_m: f64) -> f64 {
        let ns = self.surface_refractivity();
        let grad = self.gradient_n_per_km();
        (ns + grad * height_m / 1000.0).max(0.0)
    }

    /// Effective Earth radius factor k = 1 / (1 + R * dN/dh).
    /// R = 6371 km, dN/dh in N-units/km (multiplied by 1e-6 for conversion).
    pub fn effective_earth_radius_factor(&self) -> f64 {
        let r_earth_km = 6371.0;
        let dn_dh = self.gradient_n_per_km(); // N/km
        1.0 / (1.0 + r_earth_km * dn_dh * 1e-6)
    }
}

// ---------------------------------------------------------------------------
// Scatter volume result
// ---------------------------------------------------------------------------

/// Describes the common scatter volume where the TX and RX beams overlap.
#[derive(Debug, Clone, Copy)]
pub struct ScatterVolume {
    /// Height of the centre of the scatter volume above ground (metres).
    pub height_m: f64,
    /// Approximate volume in cubic metres.
    pub volume_m3: f64,
    /// Scattering angle in radians (angle between beams at the scatter volume).
    pub scattering_angle_rad: f64,
    /// Cross-section width of the common volume (metres).
    pub cross_section_m: f64,
}

// ---------------------------------------------------------------------------
// Main struct
// ---------------------------------------------------------------------------

/// Models a tropospheric scatter propagation link.
///
/// Computes path loss, scatter volume geometry, multipath spread, fade
/// margins, and related quantities for a single troposcatter hop.
#[derive(Debug, Clone)]
pub struct TroposcatterPropagation {
    /// Carrier frequency in Hz.
    pub frequency_hz: f64,
    /// Great-circle path distance in metres.
    pub distance_m: f64,
    /// Transmit antenna gain in dBi.
    pub tx_gain_dbi: f64,
    /// Receive antenna gain in dBi.
    pub rx_gain_dbi: f64,
    /// Transmit antenna aperture diameter in metres.
    pub tx_diameter_m: f64,
    /// Receive antenna aperture diameter in metres.
    pub rx_diameter_m: f64,
    /// Atmospheric refractivity profile.
    pub atmosphere: AtmosphericProfile,
}

impl TroposcatterPropagation {
    /// Create a new troposcatter propagation model.
    pub fn new(
        frequency_hz: f64,
        distance_m: f64,
        tx_gain_dbi: f64,
        rx_gain_dbi: f64,
        tx_diameter_m: f64,
        rx_diameter_m: f64,
        atmosphere: AtmosphericProfile,
    ) -> Self {
        Self {
            frequency_hz,
            distance_m,
            tx_gain_dbi,
            rx_gain_dbi,
            tx_diameter_m,
            rx_diameter_m,
            atmosphere,
        }
    }

    // -- Helpers --

    /// Wavelength in metres.
    fn wavelength_m(&self) -> f64 {
        299_792_458.0 / self.frequency_hz
    }

    /// Effective Earth radius in metres.
    fn effective_earth_radius_m(&self) -> f64 {
        let k = self.atmosphere.effective_earth_radius_factor();
        6_371_000.0 * k
    }

    // -- Scatter volume geometry --

    /// Scattering angle (radians) -- the angle subtended at the common volume
    /// by the line-of-sight paths from TX and RX.
    ///
    /// Approximation: theta = d / (2 * k * Re) for a symmetric link geometry.
    pub fn scattering_angle_rad(&self) -> f64 {
        let re = self.effective_earth_radius_m();
        self.distance_m / (2.0 * re)
    }

    /// Height of the scatter volume centre above the surface (metres).
    ///
    /// For a symmetric geometry the scatter point is at the midpoint and
    /// its height is approximately h = d^2 / (8 * k * Re).
    pub fn scatter_volume_height_m(&self) -> f64 {
        let re = self.effective_earth_radius_m();
        let d = self.distance_m;
        d * d / (8.0 * re)
    }

    /// Compute the full scatter volume geometry.
    pub fn scatter_volume(&self) -> ScatterVolume {
        let theta = self.scattering_angle_rad();
        let h = self.scatter_volume_height_m();
        let lambda = self.wavelength_m();

        // Half-power beamwidth ~ 1.22 * lambda / D (circular aperture)
        let bw_tx = 1.22 * lambda / self.tx_diameter_m;
        let bw_rx = 1.22 * lambda / self.rx_diameter_m;

        // At the scatter volume the beam widths (cross-range) are:
        let half_d = self.distance_m / 2.0;
        let w_tx = half_d * bw_tx; // cross-section from TX beam
        let w_rx = half_d * bw_rx; // cross-section from RX beam

        // The common-volume cross-section is roughly the smaller of the two.
        let cross = w_tx.min(w_rx);

        // Depth along the path is limited by the overlap of the two beams.
        // A simple model: depth ~ cross / sin(theta/2), clamped.
        let depth = if theta > 1e-12 {
            cross / (theta / 2.0).sin()
        } else {
            cross * 1e6 // degenerate -- very short path
        };

        // Volume ~ pi/4 * cross^2 * depth  (cylinder approximation)
        let volume = PI / 4.0 * cross * cross * depth;

        ScatterVolume {
            height_m: h,
            volume_m3: volume,
            scattering_angle_rad: theta,
            cross_section_m: cross,
        }
    }

    // -- Path loss models --

    /// Free-space path loss in dB.
    pub fn free_space_loss_db(&self) -> f64 {
        let lambda = self.wavelength_m();
        20.0 * (4.0 * PI * self.distance_m / lambda).log10()
    }

    /// Scatter loss (dB) as a function of scattering angle.
    ///
    /// Uses the NBS 101 empirical model: Ls ~ 25 * log10(theta) + 25 * log10(f_GHz) + Ns/10
    /// where theta is the scattering angle in milliradians.
    pub fn scatter_loss_db(&self) -> f64 {
        let theta_mrad = self.scattering_angle_rad() * 1000.0;
        let f_ghz = self.frequency_hz / 1e9;
        let ns = self.atmosphere.surface_refractivity();

        // Clamp theta to avoid log of zero
        let theta_clamped = theta_mrad.max(0.01);

        25.0 * theta_clamped.log10() + 25.0 * f_ghz.log10() + ns / 10.0
    }

    /// Aperture-to-medium coupling loss in dB.
    ///
    /// Large antennas do not achieve their full gain in troposcatter because
    /// the scattered field is not fully coherent across the aperture.
    /// Empirical model: Lc ~ 0.07 * exp(0.055 * (G_tx + G_rx))  dB,
    /// capped at a reasonable maximum.
    pub fn coupling_loss_db(&self) -> f64 {
        let g_total = self.tx_gain_dbi + self.rx_gain_dbi;
        (0.07 * (0.055 * g_total).exp()).min(15.0)
    }

    /// Total one-way path loss in dB.
    ///
    /// L_total = FSPL + L_scatter + L_coupling - G_tx - G_rx
    ///
    /// This is the net loss from TX output to RX input, accounting for
    /// antenna gains, scatter loss, and coupling loss.
    pub fn total_path_loss_db(&self) -> f64 {
        let fspl = self.free_space_loss_db();
        let ls = self.scatter_loss_db();
        let lc = self.coupling_loss_db();
        fspl + ls + lc - self.tx_gain_dbi - self.rx_gain_dbi
    }

    // -- Multipath and fading --

    /// Estimated multipath spread (delay spread) in seconds.
    ///
    /// The multipath spread arises from different scattering paths through
    /// the common volume. It is roughly proportional to the scatter volume
    /// depth divided by the speed of light.
    ///
    /// tau ~ delta_L / c where delta_L is the depth of the common volume.
    pub fn multipath_spread_s(&self) -> f64 {
        let sv = self.scatter_volume();
        let theta = sv.scattering_angle_rad;

        // Path length difference across the common volume
        // delta_L ~ cross_section * theta  (geometry)
        let delta_l = sv.cross_section_m * theta;
        delta_l / 299_792_458.0
    }

    /// Coherence bandwidth in Hz (inverse of multipath spread).
    pub fn coherence_bandwidth_hz(&self) -> f64 {
        let tau = self.multipath_spread_s();
        if tau > 0.0 {
            1.0 / tau
        } else {
            f64::INFINITY
        }
    }

    /// Fade margin (dB) required for a desired link availability.
    ///
    /// Uses the Rayleigh fading approximation for troposcatter:
    /// FM = -10 * log10( -ln(availability) ) for availability close to 1.
    ///
    /// `availability` is a fraction, e.g. 0.999 for 99.9 % availability.
    pub fn fade_margin_db(&self, availability: f64) -> f64 {
        assert!(
            availability > 0.0 && availability < 1.0,
            "availability must be in (0, 1)"
        );
        // For Rayleigh fading, P(signal > threshold) = exp(-threshold/median).
        // We want P(signal > threshold) = availability.
        // threshold/median = -ln(availability).
        // In dB: FM = 10 * log10( -ln(availability) ).
        // We negate to get a positive margin value for high availability.
        -10.0 * (-availability.ln()).log10()
    }

    /// Time diversity gain (dB) from using two time-separated receptions.
    ///
    /// If the fade decorrelation time is shorter than the diversity
    /// interval, the two receptions are independent. For Rayleigh fading
    /// with selection combining of two independent branches the diversity
    /// gain at a given availability level is approximately:
    ///
    /// G_d ~ FM(1-branch) - FM(2-branch) where FM is the fade margin.
    ///
    /// For two-branch selection combining against Rayleigh fading:
    /// P_out = (1 - exp(-gamma/Gamma))^2  (CDF of max of two Rayleigh).
    ///
    /// `availability` is the desired link availability (e.g. 0.999).
    pub fn time_diversity_gain_db(&self, availability: f64) -> f64 {
        assert!(
            availability > 0.0 && availability < 1.0,
            "availability must be in (0, 1)"
        );
        let fm_single = self.fade_margin_db(availability);

        // Two-branch selection combining: outage = (1-avail), so for each
        // branch the per-branch outage = sqrt(1-avail).
        let per_branch_avail = 1.0 - (1.0 - availability).sqrt();
        let fm_dual = self.fade_margin_db_for_outage(1.0 - per_branch_avail);

        (fm_single - fm_dual).max(0.0)
    }

    /// Internal: fade margin for a given outage probability directly.
    /// This is the same formula as `fade_margin_db` but kept separate for clarity.
    fn fade_margin_db_for_outage(&self, availability: f64) -> f64 {
        if availability <= 0.0 || availability >= 1.0 {
            return 0.0;
        }
        -10.0 * (-availability.ln()).log10()
    }

    // -- Atmospheric helpers --

    /// Refractivity at the scatter volume height.
    pub fn scatter_volume_refractivity(&self) -> f64 {
        let h = self.scatter_volume_height_m();
        self.atmosphere.refractivity_at_height(h)
    }

    // -- Complex (IQ) channel tap generation --

    /// Generate a single Rayleigh-faded complex tap using a simple
    /// Box-Muller-like method from a seed.
    ///
    /// Returns `(re, im)` representing a zero-mean unit-variance circular
    /// Gaussian sample, scaled by `amplitude`.
    ///
    /// This is a deterministic pseudo-random generator seeded by the
    /// given `seed` value. It is NOT cryptographically secure and is
    /// intended only for simulation.
    pub fn rayleigh_tap(seed: u64, amplitude: f64) -> (f64, f64) {
        // Simple LCG-based pseudo-random
        let s1 = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let s2 = s1.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);

        // Map to (0, 1)
        let u1 = (s1 >> 11) as f64 / (1u64 << 53) as f64;
        let u2 = (s2 >> 11) as f64 / (1u64 << 53) as f64;

        // Clamp u1 away from zero for log
        let u1c = u1.max(1e-15);

        let r = (-2.0 * u1c.ln()).sqrt();
        let theta = 2.0 * PI * u2;

        (amplitude * r * theta.cos(), amplitude * r * theta.sin())
    }

    /// Generate a set of multipath taps for the troposcatter channel.
    ///
    /// Returns a vector of `(delay_s, (re, im))` tuples representing
    /// the channel impulse response. The number of taps is determined by
    /// the scatter volume geometry; the tap delays are uniformly spaced
    /// across the estimated multipath spread.
    ///
    /// `num_taps` -- number of taps to generate (typically 3-10).
    /// `seed` -- pseudo-random seed for repeatable generation.
    pub fn generate_channel_taps(&self, num_taps: usize, seed: u64) -> Vec<(f64, (f64, f64))> {
        if num_taps == 0 {
            return vec![];
        }

        let tau_max = self.multipath_spread_s();
        let delay_step = if num_taps > 1 {
            tau_max / (num_taps - 1) as f64
        } else {
            0.0
        };

        let mut taps = Vec::with_capacity(num_taps);
        for i in 0..num_taps {
            let delay = i as f64 * delay_step;
            // Exponential power-delay profile: power proportional to exp(-delay / tau_rms)
            let tau_rms = tau_max / 3.0; // RMS delay spread ~ 1/3 of max
            let power = if tau_rms > 0.0 {
                (-delay / tau_rms).exp()
            } else {
                if i == 0 { 1.0 } else { 0.0 }
            };
            let amplitude = power.sqrt();
            let tap = Self::rayleigh_tap(seed.wrapping_add(i as u64 * 7919), amplitude);
            taps.push((delay, tap));
        }

        // Normalise tap powers so that total channel power = 1
        let total_power: f64 = taps.iter().map(|(_, (re, im))| re * re + im * im).sum();
        if total_power > 0.0 {
            let scale = 1.0 / total_power.sqrt();
            for tap in &mut taps {
                tap.1 .0 *= scale;
                tap.1 .1 *= scale;
            }
        }

        taps
    }

    /// Apply the troposcatter channel to a complex IQ signal.
    ///
    /// The signal is convolved with the tapped delay line model.
    /// `signal` is a slice of `(re, im)` tuples at the given `sample_rate_hz`.
    /// Returns the output signal of the same length (truncated convolution).
    pub fn apply_channel(
        &self,
        signal: &[(f64, f64)],
        sample_rate_hz: f64,
        num_taps: usize,
        seed: u64,
    ) -> Vec<(f64, f64)> {
        let taps = self.generate_channel_taps(num_taps, seed);
        let n = signal.len();
        let mut output = vec![(0.0_f64, 0.0_f64); n];

        for (delay, (h_re, h_im)) in &taps {
            let delay_samples = (delay * sample_rate_hz).round() as usize;
            for i in delay_samples..n {
                let (s_re, s_im) = signal[i - delay_samples];
                // Complex multiply: h * s
                output[i].0 += h_re * s_re - h_im * s_im;
                output[i].1 += h_re * s_im + h_im * s_re;
            }
        }

        output
    }

    /// Compute the received power in dBW given transmit power in dBW.
    pub fn received_power_dbw(&self, tx_power_dbw: f64) -> f64 {
        tx_power_dbw - self.total_path_loss_db()
    }

    /// Link margin in dB: received power minus required SNR minus noise floor.
    ///
    /// `tx_power_dbw` -- transmit power in dBW.
    /// `bandwidth_hz` -- receiver bandwidth in Hz.
    /// `required_snr_db` -- minimum SNR for demodulation.
    /// `noise_figure_db` -- receiver noise figure in dB.
    pub fn link_margin_db(
        &self,
        tx_power_dbw: f64,
        bandwidth_hz: f64,
        required_snr_db: f64,
        noise_figure_db: f64,
    ) -> f64 {
        let rx_power = self.received_power_dbw(tx_power_dbw);
        // Thermal noise floor: kTB in dBW
        let k_boltzmann = 1.380649e-23_f64;
        let noise_floor_dbw =
            10.0 * (k_boltzmann * 290.0 * bandwidth_hz).log10() + noise_figure_db;
        rx_power - noise_floor_dbw - required_snr_db
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to create a default link for testing.
    fn default_link() -> TroposcatterPropagation {
        TroposcatterPropagation::new(
            5.0e9,
            300_000.0,
            40.0,
            40.0,
            3.0,
            3.0,
            AtmosphericProfile::Standard,
        )
    }

    #[test]
    fn test_wavelength() {
        let link = default_link();
        let lambda = link.wavelength_m();
        // 5 GHz -> ~0.06 m
        assert!((lambda - 0.059958).abs() < 0.001, "lambda = {lambda}");
    }

    #[test]
    fn test_effective_earth_radius() {
        let link = default_link();
        let re = link.effective_earth_radius_m();
        // Standard atmosphere k ~ 4/3 -> effective radius ~ 8495 km
        assert!(re > 8_000_000.0 && re < 9_000_000.0, "Re = {re}");
    }

    #[test]
    fn test_scattering_angle() {
        let link = default_link();
        let theta = link.scattering_angle_rad();
        // 300 km / (2 * ~8.5e6) ~ 0.0176 rad ~ 1.01 degrees
        assert!(theta > 0.015 && theta < 0.025, "theta = {theta} rad");
    }

    #[test]
    fn test_scatter_volume_height() {
        let link = default_link();
        let h = link.scatter_volume_height_m();
        // d^2/(8kRe) ~ 9e10 / (8 * 8.5e6) ~ 1324 m
        assert!(h > 800.0 && h < 2500.0, "h = {h} m");
    }

    #[test]
    fn test_scatter_volume_struct() {
        let link = default_link();
        let sv = link.scatter_volume();
        assert!(sv.height_m > 0.0);
        assert!(sv.volume_m3 > 0.0, "volume = {}", sv.volume_m3);
        assert!(sv.scattering_angle_rad > 0.0);
        assert!(sv.cross_section_m > 0.0);
    }

    #[test]
    fn test_free_space_loss() {
        let link = default_link();
        let fspl = link.free_space_loss_db();
        // FSPL at 5 GHz, 300 km ~ 157 dB
        assert!(fspl > 150.0 && fspl < 170.0, "FSPL = {fspl} dB");
    }

    #[test]
    fn test_scatter_loss() {
        let link = default_link();
        let ls = link.scatter_loss_db();
        // Should be positive and significant
        assert!(ls > 20.0 && ls < 120.0, "scatter loss = {ls} dB");
    }

    #[test]
    fn test_coupling_loss() {
        let link = default_link();
        let lc = link.coupling_loss_db();
        // With 40+40 = 80 dBi total gain, coupling loss should be notable
        assert!(lc > 0.0 && lc <= 15.0, "coupling loss = {lc} dB");
    }

    #[test]
    fn test_total_path_loss_reasonable() {
        let link = default_link();
        let loss = link.total_path_loss_db();
        // Typical troposcatter loss: 150 -- 230 dB at 300 km
        assert!(
            loss > 120.0 && loss < 260.0,
            "total loss = {loss} dB"
        );
    }

    #[test]
    fn test_path_loss_increases_with_distance() {
        let short = TroposcatterPropagation::new(
            5.0e9, 100_000.0, 40.0, 40.0, 3.0, 3.0, AtmosphericProfile::Standard,
        );
        let long = TroposcatterPropagation::new(
            5.0e9, 500_000.0, 40.0, 40.0, 3.0, 3.0, AtmosphericProfile::Standard,
        );
        assert!(
            long.total_path_loss_db() > short.total_path_loss_db(),
            "longer path should have more loss"
        );
    }

    #[test]
    fn test_path_loss_increases_with_frequency() {
        let low = TroposcatterPropagation::new(
            1.0e9, 300_000.0, 40.0, 40.0, 3.0, 3.0, AtmosphericProfile::Standard,
        );
        let high = TroposcatterPropagation::new(
            10.0e9, 300_000.0, 40.0, 40.0, 3.0, 3.0, AtmosphericProfile::Standard,
        );
        assert!(
            high.total_path_loss_db() > low.total_path_loss_db(),
            "higher frequency should have more loss"
        );
    }

    #[test]
    fn test_multipath_spread() {
        let link = default_link();
        let tau = link.multipath_spread_s();
        // Typically 10 ns to 1 us for troposcatter
        assert!(tau > 1e-12 && tau < 1e-3, "multipath spread = {tau} s");
    }

    #[test]
    fn test_coherence_bandwidth() {
        let link = default_link();
        let bc = link.coherence_bandwidth_hz();
        assert!(bc > 0.0 && bc.is_finite(), "Bc = {bc} Hz");
        // Should be inverse of multipath spread
        let tau = link.multipath_spread_s();
        let expected = 1.0 / tau;
        assert!(
            (bc - expected).abs() / expected < 1e-6,
            "Bc should equal 1/tau"
        );
    }

    #[test]
    fn test_fade_margin_99_percent() {
        let link = default_link();
        let fm = link.fade_margin_db(0.99);
        // At 99% availability with Rayleigh: FM ~ 20 dB
        assert!(fm > 10.0 && fm < 30.0, "FM(99%) = {fm} dB");
    }

    #[test]
    fn test_fade_margin_increases_with_availability() {
        let link = default_link();
        let fm_99 = link.fade_margin_db(0.99);
        let fm_999 = link.fade_margin_db(0.999);
        assert!(
            fm_999 > fm_99,
            "higher availability needs more margin: {fm_999} > {fm_99}"
        );
    }

    #[test]
    fn test_time_diversity_gain() {
        let link = default_link();
        let gd = link.time_diversity_gain_db(0.999);
        // Diversity gain should be positive and meaningful
        assert!(gd > 0.0 && gd < 40.0, "diversity gain = {gd} dB");
    }

    #[test]
    fn test_atmospheric_profiles() {
        let std_prof = AtmosphericProfile::Standard;
        let sup = AtmosphericProfile::SuperRefractive;
        let sub = AtmosphericProfile::SubRefractive;

        assert!((std_prof.surface_refractivity() - 315.0).abs() < 1e-6);
        assert!(sup.surface_refractivity() > std_prof.surface_refractivity());
        assert!(sub.surface_refractivity() < std_prof.surface_refractivity());

        // Super-refractive has more bending -> larger k -> larger effective radius
        assert!(sup.effective_earth_radius_factor() > std_prof.effective_earth_radius_factor());
    }

    #[test]
    fn test_custom_atmospheric_profile() {
        let custom = AtmosphericProfile::Custom {
            surface_refractivity: 330.0,
            gradient_n_per_km: -45.0,
        };
        assert!((custom.surface_refractivity() - 330.0).abs() < 1e-6);
        assert!((custom.gradient_n_per_km() - (-45.0)).abs() < 1e-6);

        let n_at_1km = custom.refractivity_at_height(1000.0);
        assert!((n_at_1km - 285.0).abs() < 1e-6, "N(1km) = {n_at_1km}");
    }

    #[test]
    fn test_rayleigh_tap_deterministic() {
        let (r1, i1) = TroposcatterPropagation::rayleigh_tap(42, 1.0);
        let (r2, i2) = TroposcatterPropagation::rayleigh_tap(42, 1.0);
        assert!((r1 - r2).abs() < 1e-15 && (i1 - i2).abs() < 1e-15);
    }

    #[test]
    fn test_generate_channel_taps() {
        let link = default_link();
        let taps = link.generate_channel_taps(5, 123);
        assert_eq!(taps.len(), 5);

        // Check delays are ordered
        for i in 1..taps.len() {
            assert!(taps[i].0 >= taps[i - 1].0, "delays should be non-decreasing");
        }

        // Check normalisation: total power ~ 1
        let total_power: f64 = taps.iter().map(|(_, (r, i))| r * r + i * i).sum();
        assert!(
            (total_power - 1.0).abs() < 0.01,
            "total power = {total_power}"
        );
    }

    #[test]
    fn test_apply_channel_preserves_length() {
        let link = default_link();
        let signal: Vec<(f64, f64)> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / 100.0;
                (phase.cos(), phase.sin())
            })
            .collect();

        let output = link.apply_channel(&signal, 1e6, 4, 99);
        assert_eq!(output.len(), signal.len());

        // Output should not be all zeros (channel taps are non-zero)
        let energy: f64 = output.iter().map(|(r, i)| r * r + i * i).sum();
        assert!(energy > 0.0, "output should have non-zero energy");
    }

    #[test]
    fn test_link_margin() {
        let link = default_link();
        // 1 kW (30 dBW) transmitter, 1 MHz BW, 10 dB required SNR, 3 dB NF
        let margin = link.link_margin_db(30.0, 1e6, 10.0, 3.0);
        // The margin could be positive or negative depending on the link
        assert!(margin.is_finite(), "margin should be finite: {margin}");
    }

    #[test]
    fn test_received_power() {
        let link = default_link();
        let rx = link.received_power_dbw(30.0);
        let loss = link.total_path_loss_db();
        assert!(
            (rx - (30.0 - loss)).abs() < 1e-6,
            "Prx = Ptx - Ltotal"
        );
    }

    #[test]
    fn test_scatter_volume_refractivity() {
        let link = default_link();
        let n = link.scatter_volume_refractivity();
        let ns = link.atmosphere.surface_refractivity();
        // Refractivity at altitude should be less than surface value
        // (gradient is negative for standard atmosphere)
        assert!(n < ns, "N at height should be less than Ns: {n} < {ns}");
        assert!(n > 0.0, "N should be positive: {n}");
    }
}
