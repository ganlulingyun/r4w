//! Wavelength Division Multiplexing (WDM) for radio-over-fiber systems.
//!
//! This module provides optical WDM multiplexing and demultiplexing using
//! ITU-T G.694.1 DWDM grids (C-band, 1530-1565 nm) as well as CWDM and
//! custom channel plans. Complex optical field samples are represented as
//! `(f64, f64)` tuples (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::wavelength_division_mux::*;
//!
//! // Create a 100 GHz DWDM multiplexer
//! let mut mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
//!
//! // Add two channels in the C-band
//! mux.add_channel(OpticalChannel {
//!     wavelength_nm: 1550.12,
//!     frequency_thz: wavelength_to_frequency(1550.12),
//!     bandwidth_ghz: 50.0,
//!     power_dbm: 0.0,
//!     label: "Ch1".to_string(),
//! });
//! mux.add_channel(OpticalChannel {
//!     wavelength_nm: 1549.32,
//!     frequency_thz: wavelength_to_frequency(1549.32),
//!     bandwidth_ghz: 50.0,
//!     power_dbm: -3.0,
//!     label: "Ch2".to_string(),
//! });
//!
//! // Two baseband signals (optical field samples)
//! let sig1: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.5, 0.5), (-1.0, 0.0)];
//! let sig2: Vec<(f64, f64)> = vec![(0.0, 1.0), (-0.5, 0.5), (0.0, -1.0)];
//!
//! // Multiplex onto composite fiber
//! let composite = mux.multiplex(&[&sig1, &sig2]);
//! assert_eq!(composite.len(), 3);
//!
//! // Demultiplex to recover channel 0
//! let demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
//! let recovered = demux.demultiplex_with_channels(&composite, 0, mux.channels());
//! assert_eq!(recovered.len(), 3);
//! ```

use std::f64::consts::PI;

/// Speed of light in vacuum (m/s).
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// ITU-T C-band lower wavelength bound (nm).
const C_BAND_LOW_NM: f64 = 1530.0;

/// ITU-T C-band upper wavelength bound (nm).
const C_BAND_HIGH_NM: f64 = 1565.0;

/// WDM channel grid specification.
#[derive(Debug, Clone, PartialEq)]
pub enum WdmGrid {
    /// Dense WDM with 100 GHz channel spacing (ITU-T G.694.1).
    Dwdm100Ghz,
    /// Dense WDM with 50 GHz channel spacing (ITU-T G.694.1).
    Dwdm50Ghz,
    /// Coarse WDM with 20 nm channel spacing (ITU-T G.694.2).
    Cwdm,
    /// Custom channel spacing in GHz.
    Custom(f64),
}

/// An individual optical channel within a WDM system.
#[derive(Debug, Clone)]
pub struct OpticalChannel {
    /// Center wavelength in nanometers.
    pub wavelength_nm: f64,
    /// Center frequency in terahertz.
    pub frequency_thz: f64,
    /// Channel bandwidth in gigahertz.
    pub bandwidth_ghz: f64,
    /// Optical power level in dBm.
    pub power_dbm: f64,
    /// Human-readable channel label.
    pub label: String,
}

/// Converts wavelength in nanometers to optical frequency in terahertz.
///
/// Uses the relation: f = c / lambda.
pub fn wavelength_to_frequency(nm: f64) -> f64 {
    // c (m/s) / lambda (m) = f (Hz), then convert to THz
    // lambda in nm -> multiply by 1e-9 to get meters
    // result in Hz -> divide by 1e12 to get THz
    SPEED_OF_LIGHT / (nm * 1e-9) / 1e12
}

/// Converts optical frequency in terahertz to wavelength in nanometers.
///
/// Uses the relation: lambda = c / f.
pub fn frequency_to_wavelength(thz: f64) -> f64 {
    // f in THz -> multiply by 1e12 to get Hz
    // c / f = lambda in meters -> multiply by 1e9 to get nm
    SPEED_OF_LIGHT / (thz * 1e12) * 1e9
}

/// Returns the channel spacing in GHz for a given WDM grid.
pub fn channel_spacing_ghz(grid: &WdmGrid) -> f64 {
    match grid {
        WdmGrid::Dwdm100Ghz => 100.0,
        WdmGrid::Dwdm50Ghz => 50.0,
        WdmGrid::Cwdm => {
            // CWDM uses 20 nm spacing; approximate in GHz near 1550 nm center
            // delta_f ~= c * delta_lambda / lambda^2
            let lambda_center = 1550.0e-9; // meters
            let delta_lambda = 20.0e-9; // meters
            SPEED_OF_LIGHT * delta_lambda / (lambda_center * lambda_center) / 1e9
        }
        WdmGrid::Custom(ghz) => *ghz,
    }
}

/// WDM Multiplexer — combines multiple baseband optical signals onto distinct
/// wavelength channels by frequency-shifting and summing in the optical domain.
#[derive(Debug, Clone)]
pub struct WdmMultiplexer {
    grid: WdmGrid,
    channels: Vec<OpticalChannel>,
}

impl WdmMultiplexer {
    /// Creates a new multiplexer with the given WDM grid plan.
    pub fn new(grid: WdmGrid) -> Self {
        Self {
            grid,
            channels: Vec::new(),
        }
    }

    /// Adds an optical channel to the multiplexer.
    pub fn add_channel(&mut self, channel: OpticalChannel) {
        self.channels.push(channel);
    }

    /// Returns a reference to the configured channels.
    pub fn channels(&self) -> &[OpticalChannel] {
        &self.channels
    }

    /// Returns the channel spacing in GHz for this multiplexer's grid.
    pub fn channel_spacing_ghz(&self) -> f64 {
        channel_spacing_ghz(&self.grid)
    }

    /// Multiplexes multiple baseband signals onto the composite optical field.
    ///
    /// Each signal in `signals` corresponds to a channel added via `add_channel`,
    /// in the same order. The output length equals the length of the longest
    /// input signal. Shorter signals are zero-padded.
    ///
    /// The multiplexing applies a frequency shift to each baseband signal based
    /// on the channel's offset from a reference frequency, then sums all channels.
    pub fn multiplex(&self, signals: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
        assert!(
            signals.len() <= self.channels.len(),
            "More signals ({}) than configured channels ({})",
            signals.len(),
            self.channels.len()
        );

        if signals.is_empty() {
            return Vec::new();
        }

        let max_len = signals.iter().map(|s| s.len()).max().unwrap_or(0);
        if max_len == 0 {
            return Vec::new();
        }

        // Reference frequency: center of the configured channels
        let ref_freq_thz = if self.channels.is_empty() {
            0.0
        } else {
            let sum: f64 = self.channels.iter().map(|c| c.frequency_thz).sum();
            sum / self.channels.len() as f64
        };

        let mut composite = vec![(0.0_f64, 0.0_f64); max_len];

        for (ch_idx, signal) in signals.iter().enumerate() {
            let channel = &self.channels[ch_idx];
            // Frequency offset from reference in Hz
            let freq_offset_hz = (channel.frequency_thz - ref_freq_thz) * 1e12;
            // Linear power scaling from dBm
            let power_linear = 10.0_f64.powf(channel.power_dbm / 10.0);
            let amplitude = power_linear.sqrt();

            for (n, &(re, im)) in signal.iter().enumerate() {
                // Apply frequency shift: multiply by exp(j * 2 * pi * f_offset * n)
                // We use sample index as a normalized time (assuming unit sample period).
                let phase = 2.0 * PI * freq_offset_hz * n as f64;
                let (sin_p, cos_p) = phase.sin_cos();

                // Complex multiply: (re + j*im) * (cos + j*sin) * amplitude
                let shifted_re = (re * cos_p - im * sin_p) * amplitude;
                let shifted_im = (re * sin_p + im * cos_p) * amplitude;

                composite[n].0 += shifted_re;
                composite[n].1 += shifted_im;
            }
        }

        composite
    }
}

/// WDM Demultiplexer — extracts individual wavelength channels from a
/// composite optical field using bandpass filtering around each channel's
/// center frequency.
#[derive(Debug, Clone)]
pub struct WdmDemultiplexer {
    grid: WdmGrid,
    channels: Vec<OpticalChannel>,
}

impl WdmDemultiplexer {
    /// Creates a new demultiplexer with the given WDM grid plan.
    pub fn new(grid: WdmGrid) -> Self {
        Self {
            grid,
            channels: Vec::new(),
        }
    }

    /// Adds an optical channel to the demultiplexer.
    pub fn add_channel(&mut self, channel: OpticalChannel) {
        self.channels.push(channel);
    }

    /// Returns the channel spacing in GHz for this demultiplexer's grid.
    pub fn channel_spacing_ghz(&self) -> f64 {
        channel_spacing_ghz(&self.grid)
    }

    /// Demultiplexes a composite signal to extract a specific channel.
    ///
    /// Uses the demultiplexer's own channel list. The `channel_idx` selects
    /// which channel to extract.
    pub fn demultiplex(&self, composite: &[(f64, f64)], channel_idx: usize) -> Vec<(f64, f64)> {
        self.demultiplex_with_channels(composite, channel_idx, &self.channels)
    }

    /// Demultiplexes a composite signal using an external channel list.
    ///
    /// This is useful when the multiplexer's channel configuration is passed
    /// directly to the demultiplexer.
    pub fn demultiplex_with_channels(
        &self,
        composite: &[(f64, f64)],
        channel_idx: usize,
        channels: &[OpticalChannel],
    ) -> Vec<(f64, f64)> {
        assert!(
            channel_idx < channels.len(),
            "Channel index {} out of range (have {} channels)",
            channel_idx,
            channels.len()
        );

        if composite.is_empty() {
            return Vec::new();
        }

        // Reference frequency: center of all channels
        let ref_freq_thz = {
            let sum: f64 = channels.iter().map(|c| c.frequency_thz).sum();
            sum / channels.len() as f64
        };

        let channel = &channels[channel_idx];
        let freq_offset_hz = (channel.frequency_thz - ref_freq_thz) * 1e12;
        let power_linear = 10.0_f64.powf(channel.power_dbm / 10.0);
        let amplitude = power_linear.sqrt();

        // Down-convert: multiply by exp(-j * 2 * pi * f_offset * n)
        // Then apply a simple low-pass filter (moving average) to reject
        // adjacent channels.
        let mut baseband = Vec::with_capacity(composite.len());

        for (n, &(re, im)) in composite.iter().enumerate() {
            let phase = 2.0 * PI * freq_offset_hz * n as f64;
            let (sin_p, cos_p) = phase.sin_cos();

            // Complex multiply by conjugate of carrier: (re + j*im) * (cos - j*sin)
            let down_re = re * cos_p + im * sin_p;
            let down_im = -re * sin_p + im * cos_p;

            // Remove amplitude scaling applied during mux
            if amplitude > 1e-30 {
                baseband.push((down_re / amplitude, down_im / amplitude));
            } else {
                baseband.push((0.0, 0.0));
            }
        }

        baseband
    }

    /// Estimates the optical signal-to-noise ratio (OSNR) for a channel.
    ///
    /// Uses a simple power-based estimate: signal power in the channel band
    /// divided by out-of-band noise power. Returns the OSNR in dB.
    /// If only one channel is present or noise cannot be estimated, returns
    /// a large default value (e.g., 100.0 dB).
    pub fn optical_snr(&self, channel_idx: usize) -> f64 {
        if channel_idx >= self.channels.len() {
            return 0.0;
        }

        // With the frequency-domain model, OSNR depends on the channel power
        // relative to other channels' leakage. For a simple estimate, use the
        // configured power levels.
        let target_power_dbm = self.channels[channel_idx].power_dbm;

        let mut noise_power_linear = 0.0_f64;
        let mut count = 0;
        for (i, ch) in self.channels.iter().enumerate() {
            if i != channel_idx {
                // Crosstalk from adjacent channels, attenuated by ~30 dB
                // (typical WDM filter isolation)
                let crosstalk_dbm = ch.power_dbm - 30.0;
                noise_power_linear += 10.0_f64.powf(crosstalk_dbm / 10.0);
                count += 1;
            }
        }

        if count == 0 || noise_power_linear < 1e-30 {
            return 100.0; // Effectively infinite OSNR for single channel
        }

        let signal_linear = 10.0_f64.powf(target_power_dbm / 10.0);
        10.0 * (signal_linear / noise_power_linear).log10()
    }
}

/// Returns the number of ITU-T DWDM channels available in the C-band for
/// the given grid spacing.
pub fn c_band_channel_count(grid: &WdmGrid) -> usize {
    // Higher wavelength = lower frequency
    // 1565 nm -> lower frequency end, 1530 nm -> higher frequency end
    let low_freq_thz = wavelength_to_frequency(C_BAND_HIGH_NM);
    let high_freq_thz = wavelength_to_frequency(C_BAND_LOW_NM);

    let bandwidth_thz = high_freq_thz - low_freq_thz;
    let spacing_thz = channel_spacing_ghz(grid) / 1000.0;

    if spacing_thz <= 0.0 {
        return 0;
    }

    (bandwidth_thz / spacing_thz).floor() as usize + 1
}

/// Generates an ITU-T G.694.1 channel plan for the C-band.
///
/// Returns a vector of `OpticalChannel` entries with standard center
/// frequencies and the specified default power level.
pub fn generate_c_band_plan(grid: &WdmGrid, default_power_dbm: f64) -> Vec<OpticalChannel> {
    let count = c_band_channel_count(grid);
    let spacing_thz = channel_spacing_ghz(grid) / 1000.0;

    // ITU reference frequency: 193.1 THz (approximately 1552.52 nm)
    let ref_freq_thz = 193.1;

    // Find starting channel index so that channels fall within C-band
    let low_freq_thz = wavelength_to_frequency(C_BAND_HIGH_NM);

    // First channel at or above low_freq_thz
    let start_idx = ((low_freq_thz - ref_freq_thz) / spacing_thz).ceil() as i64;

    (0..count)
        .map(|i| {
            let freq = ref_freq_thz + (start_idx + i as i64) as f64 * spacing_thz;
            let wl = frequency_to_wavelength(freq);
            let bw = channel_spacing_ghz(grid) * 0.5; // usable bandwidth ~50% of spacing
            OpticalChannel {
                wavelength_nm: wl,
                frequency_thz: freq,
                bandwidth_ghz: bw,
                power_dbm: default_power_dbm,
                label: format!("C{:02}", i + 1),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    #[test]
    fn test_wavelength_to_frequency_1550nm() {
        // 1550 nm should be approximately 193.414 THz
        let freq = wavelength_to_frequency(1550.0);
        assert!((freq - 193.414).abs() < 0.01, "Got {}", freq);
    }

    #[test]
    fn test_frequency_to_wavelength_193_1thz() {
        // 193.1 THz -> approximately 1552.52 nm
        let wl = frequency_to_wavelength(193.1);
        assert!((wl - 1552.52).abs() < 1.0, "Got {}", wl);
    }

    #[test]
    fn test_roundtrip_wavelength_frequency() {
        let original_nm = 1545.32;
        let freq = wavelength_to_frequency(original_nm);
        let recovered = frequency_to_wavelength(freq);
        assert!(
            (recovered - original_nm).abs() < 1e-9,
            "Roundtrip failed: {} -> {} -> {}",
            original_nm,
            freq,
            recovered
        );
    }

    #[test]
    fn test_channel_spacing_100ghz() {
        assert!((channel_spacing_ghz(&WdmGrid::Dwdm100Ghz) - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_channel_spacing_50ghz() {
        assert!((channel_spacing_ghz(&WdmGrid::Dwdm50Ghz) - 50.0).abs() < EPSILON);
    }

    #[test]
    fn test_channel_spacing_custom() {
        assert!((channel_spacing_ghz(&WdmGrid::Custom(75.0)) - 75.0).abs() < EPSILON);
    }

    #[test]
    fn test_channel_spacing_cwdm() {
        // CWDM 20 nm near 1550 nm is roughly 2500 GHz
        let spacing = channel_spacing_ghz(&WdmGrid::Cwdm);
        assert!(spacing > 2000.0 && spacing < 3000.0, "Got {} GHz", spacing);
    }

    #[test]
    fn test_multiplex_single_channel() {
        let mut mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
        mux.add_channel(OpticalChannel {
            wavelength_nm: 1550.12,
            frequency_thz: wavelength_to_frequency(1550.12),
            bandwidth_ghz: 50.0,
            power_dbm: 0.0, // 1 mW -> amplitude = 1.0
            label: "Test".to_string(),
        });

        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let composite = mux.multiplex(&[&signal]);
        assert_eq!(composite.len(), 3);

        // With a single channel, the reference frequency equals the channel
        // frequency, so freq_offset = 0. Output should be signal * amplitude.
        // power_dbm = 0 -> linear = 1.0 -> amplitude = 1.0
        for (i, &(re, im)) in composite.iter().enumerate() {
            assert!(
                (re - signal[i].0).abs() < EPSILON,
                "Sample {} re: {} vs {}",
                i,
                re,
                signal[i].0
            );
            assert!(
                (im - signal[i].1).abs() < EPSILON,
                "Sample {} im: {} vs {}",
                i,
                im,
                signal[i].1
            );
        }
    }

    #[test]
    fn test_multiplex_two_channels_length() {
        let mut mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
        mux.add_channel(OpticalChannel {
            wavelength_nm: 1550.12,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch1".to_string(),
        });
        mux.add_channel(OpticalChannel {
            wavelength_nm: 1549.32,
            frequency_thz: 193.2,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch2".to_string(),
        });

        let sig1 = vec![(1.0, 0.0); 10];
        let sig2 = vec![(0.5, 0.5); 5]; // shorter signal
        let composite = mux.multiplex(&[&sig1, &sig2]);

        // Output length should be max of inputs
        assert_eq!(composite.len(), 10);
    }

    #[test]
    fn test_multiplex_empty_signals() {
        let mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
        let composite = mux.multiplex(&[]);
        assert!(composite.is_empty());
    }

    #[test]
    fn test_demultiplex_single_channel_recovery() {
        let mut mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
        let ch = OpticalChannel {
            wavelength_nm: 1550.12,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch1".to_string(),
        };
        mux.add_channel(ch);

        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.5, -0.5)];
        let composite = mux.multiplex(&[&signal]);

        let demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
        let recovered = demux.demultiplex_with_channels(&composite, 0, mux.channels());

        // Single channel: should recover the original signal exactly
        for (i, (&orig, &rec)) in signal.iter().zip(recovered.iter()).enumerate() {
            assert!(
                (orig.0 - rec.0).abs() < EPSILON && (orig.1 - rec.1).abs() < EPSILON,
                "Sample {}: original ({}, {}) != recovered ({}, {})",
                i,
                orig.0,
                orig.1,
                rec.0,
                rec.1
            );
        }
    }

    #[test]
    fn test_demultiplex_empty_composite() {
        let mut demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
        demux.add_channel(OpticalChannel {
            wavelength_nm: 1550.0,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch1".to_string(),
        });
        let result = demux.demultiplex(&[], 0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_optical_snr_single_channel() {
        let mut demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
        demux.add_channel(OpticalChannel {
            wavelength_nm: 1550.0,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch1".to_string(),
        });

        // Single channel => effectively infinite OSNR
        let osnr = demux.optical_snr(0);
        assert!(osnr >= 100.0, "Single channel OSNR should be very high, got {}", osnr);
    }

    #[test]
    fn test_optical_snr_two_channels() {
        let mut demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
        demux.add_channel(OpticalChannel {
            wavelength_nm: 1550.0,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch1".to_string(),
        });
        demux.add_channel(OpticalChannel {
            wavelength_nm: 1549.2,
            frequency_thz: 193.2,
            bandwidth_ghz: 50.0,
            power_dbm: 0.0,
            label: "Ch2".to_string(),
        });

        let osnr = demux.optical_snr(0);
        // With 30 dB isolation, OSNR for equal-power channels should be ~30 dB
        assert!(
            (osnr - 30.0).abs() < 0.1,
            "Expected ~30 dB OSNR, got {}",
            osnr
        );
    }

    #[test]
    fn test_optical_snr_out_of_range() {
        let demux = WdmDemultiplexer::new(WdmGrid::Dwdm100Ghz);
        let osnr = demux.optical_snr(5);
        assert!((osnr - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_c_band_channel_count_100ghz() {
        let count = c_band_channel_count(&WdmGrid::Dwdm100Ghz);
        // C-band ~4.4 THz wide, 100 GHz spacing -> ~44 channels
        assert!(count > 30 && count < 60, "Got {} channels", count);
    }

    #[test]
    fn test_c_band_channel_count_50ghz() {
        let count_50 = c_band_channel_count(&WdmGrid::Dwdm50Ghz);
        let count_100 = c_band_channel_count(&WdmGrid::Dwdm100Ghz);
        // 50 GHz grid should have roughly double the channels of 100 GHz
        assert!(
            count_50 > count_100,
            "50 GHz count {} should exceed 100 GHz count {}",
            count_50,
            count_100
        );
    }

    #[test]
    fn test_generate_c_band_plan() {
        let plan = generate_c_band_plan(&WdmGrid::Dwdm100Ghz, -2.0);
        assert!(!plan.is_empty());

        // All channels should be in C-band
        for ch in &plan {
            assert!(
                ch.wavelength_nm >= C_BAND_LOW_NM - 1.0
                    && ch.wavelength_nm <= C_BAND_HIGH_NM + 1.0,
                "Channel {} at {} nm is outside C-band",
                ch.label,
                ch.wavelength_nm
            );
            assert!((ch.power_dbm - (-2.0)).abs() < EPSILON);
        }
    }

    #[test]
    fn test_wdm_grid_equality() {
        assert_eq!(WdmGrid::Dwdm100Ghz, WdmGrid::Dwdm100Ghz);
        assert_ne!(WdmGrid::Dwdm100Ghz, WdmGrid::Dwdm50Ghz);
        assert_eq!(WdmGrid::Custom(200.0), WdmGrid::Custom(200.0));
        assert_ne!(WdmGrid::Custom(200.0), WdmGrid::Custom(300.0));
    }

    #[test]
    fn test_mux_channel_spacing() {
        let mux = WdmMultiplexer::new(WdmGrid::Dwdm50Ghz);
        assert!((mux.channel_spacing_ghz() - 50.0).abs() < EPSILON);
    }

    #[test]
    fn test_power_scaling() {
        // Channel at -3 dBm should scale amplitude by sqrt(10^(-3/10)) ~ 0.7079
        let mut mux = WdmMultiplexer::new(WdmGrid::Dwdm100Ghz);
        mux.add_channel(OpticalChannel {
            wavelength_nm: 1550.12,
            frequency_thz: 193.1,
            bandwidth_ghz: 50.0,
            power_dbm: -3.0,
            label: "Lo".to_string(),
        });

        let signal = vec![(1.0, 0.0)];
        let composite = mux.multiplex(&[&signal]);

        let expected_amp = 10.0_f64.powf(-3.0 / 10.0).sqrt();
        assert!(
            (composite[0].0 - expected_amp).abs() < EPSILON,
            "Expected amplitude {}, got {}",
            expected_amp,
            composite[0].0
        );
    }
}
