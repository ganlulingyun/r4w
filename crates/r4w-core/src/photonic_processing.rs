//! Optical signal processing and radio-over-fiber (RoF) interface simulation.
//!
//! This module provides models for photonic components used in radio-over-fiber
//! links: Mach-Zehnder modulators (MZM) for electrical-to-optical conversion,
//! photodiode receivers for optical-to-electrical conversion, chromatic dispersion
//! compensation, and a complete RoF link model.
//!
//! # Example
//!
//! ```
//! use r4w_core::photonic_processing::{
//!     PhotonicModulator, OptoElectronicReceiver, RofLink,
//! };
//!
//! // Create a Mach-Zehnder modulator with Vpi=4V, biased at quadrature
//! let modulator = PhotonicModulator::new(4.0, 2.0);
//!
//! // Create a photodiode receiver with responsivity 0.8 A/W
//! let receiver = OptoElectronicReceiver::new(0.8);
//!
//! // Build a 10 km radio-over-fiber link
//! let link = RofLink::new(modulator, 10.0, receiver);
//!
//! // Transmit an RF signal through the fiber
//! let rf_signal = vec![(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (0.0, -0.5)];
//! let sample_rate = 1e9; // 1 GHz
//! let received = link.transmit(&rf_signal, sample_rate);
//! assert_eq!(received.len(), rf_signal.len());
//!
//! // Check link budget
//! let budget = link.link_budget_db();
//! assert!(budget < 0.0); // Fiber introduces loss
//! ```

use std::f64::consts::PI;

/// Type of optical modulator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModulatorType {
    /// Mach-Zehnder interferometric modulator.
    /// Uses interference between two optical paths to modulate intensity.
    MachZehnder,
    /// Electro-absorption modulator.
    /// Uses the Franz-Keldysh or quantum-confined Stark effect.
    ElectroAbsorption,
}

/// Mach-Zehnder modulator (MZM) model for electrical-to-optical (E/O) conversion.
///
/// The MZM transfer function is:
///   E_out = E_in * cos(pi * V / (2 * Vpi) + phi_bias)
///
/// where Vpi is the half-wave voltage and phi_bias = pi * bias_v / (2 * Vpi).
#[derive(Debug, Clone)]
pub struct PhotonicModulator {
    /// Half-wave voltage (V). The voltage swing needed for a pi phase shift.
    vpi: f64,
    /// DC bias voltage (V).
    bias_v: f64,
    /// Modulator type.
    modulator_type: ModulatorType,
    /// Optical insertion loss in dB (default 3 dB for MZM).
    insertion_loss_db: f64,
}

impl PhotonicModulator {
    /// Create a new Mach-Zehnder modulator.
    ///
    /// # Arguments
    /// * `vpi` - Half-wave voltage in volts (typically 3-6 V)
    /// * `bias_v` - DC bias voltage in volts (quadrature point = Vpi/2)
    pub fn new(vpi: f64, bias_v: f64) -> Self {
        Self {
            vpi,
            bias_v,
            modulator_type: ModulatorType::MachZehnder,
            insertion_loss_db: 3.0,
        }
    }

    /// Create a modulator with a specific type.
    pub fn with_type(mut self, modulator_type: ModulatorType) -> Self {
        self.modulator_type = modulator_type;
        self
    }

    /// Set the insertion loss in dB.
    pub fn with_insertion_loss_db(mut self, loss_db: f64) -> Self {
        self.insertion_loss_db = loss_db;
        self
    }

    /// Get the modulator type.
    pub fn modulator_type(&self) -> ModulatorType {
        self.modulator_type
    }

    /// Get the half-wave voltage.
    pub fn vpi(&self) -> f64 {
        self.vpi
    }

    /// Get the bias voltage.
    pub fn bias_v(&self) -> f64 {
        self.bias_v
    }

    /// Compute the bias phase: phi_bias = pi * bias_v / (2 * Vpi).
    fn bias_phase(&self) -> f64 {
        PI * self.bias_v / (2.0 * self.vpi)
    }

    /// Compute the insertion loss as a linear factor.
    fn insertion_loss_linear(&self) -> f64 {
        10.0_f64.powf(-self.insertion_loss_db / 20.0)
    }

    /// Modulate an RF signal onto an optical carrier (E/O conversion).
    ///
    /// Applies the MZM transfer function to each sample:
    ///   E_out = E_in * cos(pi * V / (2 * Vpi) + phi_bias)
    ///
    /// The input RF voltage modulates the optical field amplitude.
    /// The input `(f64, f64)` represents the complex RF envelope (I, Q).
    /// The output represents the complex optical field.
    ///
    /// For electro-absorption modulators, an exponential transfer is used instead.
    pub fn modulate(&self, rf_signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let phi_bias = self.bias_phase();
        let il = self.insertion_loss_linear();

        rf_signal
            .iter()
            .map(|&(i, q)| {
                let rf_amplitude = (i * i + q * q).sqrt();
                let rf_phase = q.atan2(i);

                let optical_gain = match self.modulator_type {
                    ModulatorType::MachZehnder => {
                        // MZM: E_out = cos(pi * V / (2*Vpi) + phi_bias)
                        (PI * rf_amplitude / (2.0 * self.vpi) + phi_bias).cos()
                    }
                    ModulatorType::ElectroAbsorption => {
                        // EAM: exponential absorption model
                        // T(V) = exp(-alpha * V / Vpi)
                        let alpha = 1.0; // absorption coefficient
                        (-alpha * rf_amplitude / self.vpi).exp()
                    }
                };

                let out_amp = optical_gain.abs() * il;
                // Preserve the RF phase, apply sign from modulator transfer
                let sign = if optical_gain < 0.0 { PI } else { 0.0 };
                let out_phase = rf_phase + sign;

                (out_amp * out_phase.cos(), out_amp * out_phase.sin())
            })
            .collect()
    }
}

/// Opto-electronic receiver with square-law photodiode detection (O/E conversion).
///
/// The photodiode current is proportional to the optical power:
///   I = R * |E|^2
///
/// where R is the responsivity in A/W.
#[derive(Debug, Clone)]
pub struct OptoElectronicReceiver {
    /// Photodiode responsivity in A/W (typically 0.5-0.9 for InGaAs).
    responsivity: f64,
    /// Transimpedance amplifier gain in ohms (converts current to voltage).
    tia_gain: f64,
    /// Thermal noise spectral density in A/sqrt(Hz).
    thermal_noise_density: f64,
}

impl OptoElectronicReceiver {
    /// Create a new opto-electronic receiver.
    ///
    /// # Arguments
    /// * `responsivity` - Photodiode responsivity in A/W (typically 0.5-0.9)
    pub fn new(responsivity: f64) -> Self {
        Self {
            responsivity,
            tia_gain: 1000.0,     // 1 kOhm default TIA
            thermal_noise_density: 0.0, // noiseless by default
        }
    }

    /// Set the transimpedance amplifier gain.
    pub fn with_tia_gain(mut self, gain_ohms: f64) -> Self {
        self.tia_gain = gain_ohms;
        self
    }

    /// Set the thermal noise density.
    pub fn with_thermal_noise(mut self, noise_density: f64) -> Self {
        self.thermal_noise_density = noise_density;
        self
    }

    /// Get the responsivity.
    pub fn responsivity(&self) -> f64 {
        self.responsivity
    }

    /// Detect optical signal using square-law detection (O/E conversion).
    ///
    /// Computes I = R * |E|^2 for each sample.
    /// The output is a baseband electrical signal where the amplitude
    /// corresponds to the detected optical power.
    pub fn detect(&self, optical: &[(f64, f64)]) -> Vec<(f64, f64)> {
        optical
            .iter()
            .map(|&(e_i, e_q)| {
                // Square-law detection: I = R * |E|^2
                let power = e_i * e_i + e_q * e_q;
                let current = self.responsivity * power;
                // TIA converts current to voltage
                let voltage = current * self.tia_gain;
                // Output as real-valued (detected envelope), Q = 0
                (voltage, 0.0)
            })
            .collect()
    }
}

/// Chromatic dispersion compensator for optical fiber.
///
/// Chromatic dispersion causes different frequency components to travel at
/// different speeds through the fiber, broadening pulses. This compensator
/// applies the inverse dispersion in the frequency domain.
///
/// The dispersion phase is:
///   H(f) = exp(-j * pi * D * L * lambda^2 * f^2 / c)
///
/// where D is the dispersion parameter, L is fiber length, lambda is wavelength.
#[derive(Debug, Clone)]
pub struct DispersionCompensator {
    /// Dispersion parameter in ps/(nm*km). Typical SMF-28: ~17 ps/(nm*km).
    dispersion_ps_nm_km: f64,
    /// Fiber length in km.
    length_km: f64,
    /// Operating wavelength in nm (typically 1550 nm for C-band).
    wavelength_nm: f64,
}

impl DispersionCompensator {
    /// Create a new dispersion compensator.
    ///
    /// # Arguments
    /// * `dispersion_ps_nm_km` - Dispersion parameter in ps/(nm*km)
    /// * `length_km` - Fiber length in km
    /// * `wavelength_nm` - Operating wavelength in nm
    pub fn new(dispersion_ps_nm_km: f64, length_km: f64, wavelength_nm: f64) -> Self {
        Self {
            dispersion_ps_nm_km,
            length_km,
            wavelength_nm,
        }
    }

    /// Compute the total accumulated dispersion in ps/nm.
    pub fn total_dispersion_ps_nm(&self) -> f64 {
        self.dispersion_ps_nm_km * self.length_km
    }

    /// Compensate chromatic dispersion in the frequency domain.
    ///
    /// Applies the inverse of the fiber's dispersion transfer function
    /// using FFT -> multiply by H_comp(f) -> IFFT.
    ///
    /// # Arguments
    /// * `signal` - Input optical field samples (I, Q)
    /// * `sample_rate` - Sample rate in Hz
    pub fn compensate(&self, signal: &[(f64, f64)], sample_rate: f64) -> Vec<(f64, f64)> {
        let n = signal.len();
        if n == 0 {
            return vec![];
        }

        // Speed of light in m/s
        let c = 2.998e8;
        // Convert units: D [ps/(nm*km)] -> [s/m^2]: multiply by 1e-6
        let d_si = self.dispersion_ps_nm_km * 1e-6; // s/m^2
        let lambda = self.wavelength_nm * 1e-9; // m
        let length = self.length_km * 1e3; // m

        // Dispersion coefficient: beta2 = -D * lambda^2 / (2*pi*c)
        let beta2 = -d_si * lambda * lambda / (2.0 * PI * c);
        let total_beta2 = beta2 * length;

        // FFT (DFT)
        let mut spectrum = dft(signal);

        // Apply compensation transfer function
        // H_comp(f) = exp(+j * beta2 * L / 2 * (2*pi*f)^2)
        // (positive sign to compensate negative dispersion)
        for k in 0..n {
            let f = if k <= n / 2 {
                k as f64 * sample_rate / n as f64
            } else {
                (k as f64 - n as f64) * sample_rate / n as f64
            };

            let omega = 2.0 * PI * f;
            // Compensation phase (opposite sign to dispersion)
            let phase = 0.5 * total_beta2 * omega * omega;

            let (cos_p, sin_p) = (phase.cos(), phase.sin());
            let (re, im) = spectrum[k];
            // Complex multiply: (re + j*im) * (cos + j*sin)
            spectrum[k] = (re * cos_p - im * sin_p, re * sin_p + im * cos_p);
        }

        // IFFT (IDFT)
        idft(&spectrum)
    }
}

/// Complete radio-over-fiber (RoF) link model.
///
/// Models the full chain: RF input -> E/O modulation -> fiber propagation
/// (with attenuation and dispersion) -> O/E detection -> RF output.
#[derive(Debug, Clone)]
pub struct RofLink {
    /// E/O modulator.
    modulator: PhotonicModulator,
    /// Fiber length in km.
    fiber_length_km: f64,
    /// O/E receiver.
    receiver: OptoElectronicReceiver,
    /// Fiber attenuation in dB/km (typical SMF: 0.2 dB/km at 1550 nm).
    fiber_attenuation_db_km: f64,
    /// Dispersion parameter in ps/(nm*km). Set to 0 to disable.
    dispersion_ps_nm_km: f64,
    /// Operating wavelength in nm.
    wavelength_nm: f64,
}

impl RofLink {
    /// Create a new radio-over-fiber link.
    ///
    /// # Arguments
    /// * `modulator` - Photonic modulator for E/O conversion
    /// * `fiber_length_km` - Length of the fiber in km
    /// * `receiver` - Opto-electronic receiver for O/E conversion
    pub fn new(
        modulator: PhotonicModulator,
        fiber_length_km: f64,
        receiver: OptoElectronicReceiver,
    ) -> Self {
        Self {
            modulator,
            fiber_length_km,
            receiver,
            fiber_attenuation_db_km: 0.2, // typical SMF at 1550 nm
            dispersion_ps_nm_km: 17.0,    // typical SMF-28
            wavelength_nm: 1550.0,        // C-band
        }
    }

    /// Set the fiber attenuation in dB/km.
    pub fn with_attenuation(mut self, db_per_km: f64) -> Self {
        self.fiber_attenuation_db_km = db_per_km;
        self
    }

    /// Set the dispersion parameter. Use 0 to disable dispersion.
    pub fn with_dispersion(mut self, dispersion_ps_nm_km: f64) -> Self {
        self.dispersion_ps_nm_km = dispersion_ps_nm_km;
        self
    }

    /// Set the operating wavelength in nm.
    pub fn with_wavelength(mut self, wavelength_nm: f64) -> Self {
        self.wavelength_nm = wavelength_nm;
        self
    }

    /// Get the fiber length in km.
    pub fn fiber_length_km(&self) -> f64 {
        self.fiber_length_km
    }

    /// Compute the total fiber attenuation in dB.
    pub fn fiber_loss_db(&self) -> f64 {
        self.fiber_attenuation_db_km * self.fiber_length_km
    }

    /// Compute the total link budget in dB.
    ///
    /// Includes modulator insertion loss, fiber attenuation, and
    /// connector losses (estimated at 0.5 dB per connector, 2 connectors).
    pub fn link_budget_db(&self) -> f64 {
        let fiber_loss = self.fiber_loss_db();
        let modulator_loss = self.modulator.insertion_loss_db;
        let connector_loss = 1.0; // 2 connectors x 0.5 dB each
        -(fiber_loss + modulator_loss + connector_loss)
    }

    /// Transmit an RF signal through the radio-over-fiber link.
    ///
    /// Signal flow: RF -> MZM (E/O) -> Fiber (attenuation + dispersion) -> PD (O/E)
    ///
    /// # Arguments
    /// * `rf` - Input RF signal as complex IQ samples
    /// * `sample_rate` - Sample rate in Hz
    pub fn transmit(&self, rf: &[(f64, f64)], sample_rate: f64) -> Vec<(f64, f64)> {
        if rf.is_empty() {
            return vec![];
        }

        // Step 1: E/O conversion via MZM
        let optical = self.modulator.modulate(rf);

        // Step 2: Fiber attenuation
        let attenuation_linear = 10.0_f64.powf(-self.fiber_loss_db() / 20.0);
        let attenuated: Vec<(f64, f64)> = optical
            .iter()
            .map(|&(i, q)| (i * attenuation_linear, q * attenuation_linear))
            .collect();

        // Step 3: Chromatic dispersion (if enabled)
        let after_fiber = if self.dispersion_ps_nm_km.abs() > 1e-12 {
            let compensator = DispersionCompensator::new(
                self.dispersion_ps_nm_km,
                self.fiber_length_km,
                self.wavelength_nm,
            );
            // Apply dispersion (not compensation -- simulating the fiber effect)
            apply_dispersion(&attenuated, &compensator, sample_rate)
        } else {
            attenuated
        };

        // Step 4: O/E conversion via photodiode
        self.receiver.detect(&after_fiber)
    }
}

// ============================================================
// Internal helper functions: minimal DFT/IDFT for dispersion
// ============================================================

/// Compute the Discrete Fourier Transform of a complex signal.
fn dft(signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = signal.len();
    let mut spectrum = vec![(0.0, 0.0); n];

    for k in 0..n {
        let mut re_sum = 0.0;
        let mut im_sum = 0.0;
        for (i, &(re, im)) in signal.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re_sum += re * cos_a - im * sin_a;
            im_sum += re * sin_a + im * cos_a;
        }
        spectrum[k] = (re_sum, im_sum);
    }

    spectrum
}

/// Compute the Inverse Discrete Fourier Transform.
fn idft(spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = spectrum.len();
    let mut signal = vec![(0.0, 0.0); n];
    let scale = 1.0 / n as f64;

    for i in 0..n {
        let mut re_sum = 0.0;
        let mut im_sum = 0.0;
        for (k, &(re, im)) in spectrum.iter().enumerate() {
            let angle = 2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re_sum += re * cos_a - im * sin_a;
            im_sum += re * sin_a + im * cos_a;
        }
        signal[i] = (re_sum * scale, im_sum * scale);
    }

    signal
}

/// Apply fiber dispersion (forward direction) to an optical signal.
///
/// This is the fiber's dispersion effect (not compensation).
fn apply_dispersion(
    signal: &[(f64, f64)],
    compensator: &DispersionCompensator,
    sample_rate: f64,
) -> Vec<(f64, f64)> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let c = 2.998e8;
    let d_si = compensator.dispersion_ps_nm_km * 1e-6;
    let lambda = compensator.wavelength_nm * 1e-9;
    let length = compensator.length_km * 1e3;

    let beta2 = -d_si * lambda * lambda / (2.0 * PI * c);
    let total_beta2 = beta2 * length;

    let mut spectrum = dft(signal);

    for k in 0..n {
        let f = if k <= n / 2 {
            k as f64 * sample_rate / n as f64
        } else {
            (k as f64 - n as f64) * sample_rate / n as f64
        };

        let omega = 2.0 * PI * f;
        // Forward dispersion: negative sign (opposite of compensation)
        let phase = -0.5 * total_beta2 * omega * omega;

        let (cos_p, sin_p) = (phase.cos(), phase.sin());
        let (re, im) = spectrum[k];
        spectrum[k] = (re * cos_p - im * sin_p, re * sin_p + im * cos_p);
    }

    idft(&spectrum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-10;

    fn assert_close(a: f64, b: f64, tol: f64, msg: &str) {
        assert!(
            (a - b).abs() < tol,
            "{}: expected {}, got {}, diff={}",
            msg,
            b,
            a,
            (a - b).abs()
        );
    }

    #[test]
    fn test_modulator_new() {
        let m = PhotonicModulator::new(4.0, 2.0);
        assert_eq!(m.vpi(), 4.0);
        assert_eq!(m.bias_v(), 2.0);
        assert_eq!(m.modulator_type(), ModulatorType::MachZehnder);
    }

    #[test]
    fn test_modulator_type_builder() {
        let m = PhotonicModulator::new(4.0, 2.0)
            .with_type(ModulatorType::ElectroAbsorption);
        assert_eq!(m.modulator_type(), ModulatorType::ElectroAbsorption);
    }

    #[test]
    fn test_mzm_zero_input() {
        // With zero RF input, output depends only on bias
        let m = PhotonicModulator::new(4.0, 2.0);
        let result = m.modulate(&[(0.0, 0.0)]);
        assert_eq!(result.len(), 1);
        // cos(phi_bias) where phi_bias = pi*2/(2*4) = pi/4
        let expected = (PI / 4.0).cos() * m.insertion_loss_linear();
        assert_close(result[0].0, expected, 1e-10, "MZM zero input I");
        assert_close(result[0].1, 0.0, 1e-10, "MZM zero input Q");
    }

    #[test]
    fn test_mzm_null_bias() {
        // At null point (bias = Vpi), MZM output should be near zero for zero input
        let m = PhotonicModulator::new(4.0, 4.0);
        let result = m.modulate(&[(0.0, 0.0)]);
        // phi_bias = pi*4/(2*4) = pi/2 => cos(pi/2) = 0
        assert_close(result[0].0, 0.0, 1e-10, "MZM null bias");
    }

    #[test]
    fn test_mzm_quadrature_bias() {
        // At quadrature (bias = Vpi/2), most linear operating point
        let vpi = 5.0;
        let m = PhotonicModulator::new(vpi, vpi / 2.0);
        let result = m.modulate(&[(0.0, 0.0)]);
        // phi_bias = pi*(Vpi/2)/(2*Vpi) = pi/4
        let expected = (PI / 4.0).cos() * m.insertion_loss_linear();
        assert_close(result[0].0, expected, 1e-10, "MZM quadrature");
    }

    #[test]
    fn test_mzm_preserves_length() {
        let m = PhotonicModulator::new(4.0, 2.0);
        let signal: Vec<(f64, f64)> = (0..100).map(|i| (i as f64 * 0.01, 0.0)).collect();
        let result = m.modulate(&signal);
        assert_eq!(result.len(), 100);
    }

    #[test]
    fn test_eam_zero_input() {
        // EAM with zero input should have maximum transmission
        let m = PhotonicModulator::new(4.0, 2.0)
            .with_type(ModulatorType::ElectroAbsorption);
        let result = m.modulate(&[(0.0, 0.0)]);
        // exp(0) = 1.0, scaled by insertion loss
        let expected = 1.0 * m.insertion_loss_linear();
        assert_close(result[0].0, expected, 1e-10, "EAM zero input");
    }

    #[test]
    fn test_eam_increasing_absorption() {
        // EAM: higher voltage means more absorption (lower output)
        let m = PhotonicModulator::new(4.0, 0.0)
            .with_type(ModulatorType::ElectroAbsorption)
            .with_insertion_loss_db(0.0);
        let r1 = m.modulate(&[(0.5, 0.0)]);
        let r2 = m.modulate(&[(1.0, 0.0)]);
        // Higher input amplitude should give lower output
        assert!(
            r1[0].0 > r2[0].0,
            "EAM: higher voltage should have more absorption"
        );
    }

    #[test]
    fn test_receiver_new() {
        let r = OptoElectronicReceiver::new(0.8);
        assert_eq!(r.responsivity(), 0.8);
    }

    #[test]
    fn test_receiver_square_law() {
        // Square-law: I = R * |E|^2
        let r = OptoElectronicReceiver::new(0.5).with_tia_gain(1.0);
        let optical = vec![(2.0, 0.0)];
        let result = r.detect(&optical);
        // |E|^2 = 4.0, I = 0.5 * 4.0 = 2.0
        assert_close(result[0].0, 2.0, 1e-10, "Square-law detection");
        assert_close(result[0].1, 0.0, 1e-10, "Square-law Q should be 0");
    }

    #[test]
    fn test_receiver_complex_input() {
        // |E|^2 for complex input
        let r = OptoElectronicReceiver::new(1.0).with_tia_gain(1.0);
        let optical = vec![(3.0, 4.0)]; // |E|^2 = 9+16 = 25
        let result = r.detect(&optical);
        assert_close(result[0].0, 25.0, 1e-10, "Square-law complex");
    }

    #[test]
    fn test_receiver_tia_gain() {
        let r = OptoElectronicReceiver::new(1.0).with_tia_gain(100.0);
        let optical = vec![(1.0, 0.0)]; // |E|^2 = 1.0
        let result = r.detect(&optical);
        // I = 1.0 * 1.0 = 1.0, V = 1.0 * 100.0 = 100.0
        assert_close(result[0].0, 100.0, 1e-10, "TIA gain");
    }

    #[test]
    fn test_dispersion_compensator_new() {
        let dc = DispersionCompensator::new(17.0, 50.0, 1550.0);
        assert_close(
            dc.total_dispersion_ps_nm(),
            850.0,
            EPSILON,
            "Total dispersion",
        );
    }

    #[test]
    fn test_dispersion_preserves_length() {
        let dc = DispersionCompensator::new(17.0, 10.0, 1550.0);
        let signal: Vec<(f64, f64)> = (0..64).map(|i| {
            let t = i as f64 / 64.0;
            ((2.0 * PI * t).cos(), (2.0 * PI * t).sin())
        }).collect();
        let result = dc.compensate(&signal, 1e9);
        assert_eq!(result.len(), 64);
    }

    #[test]
    fn test_dispersion_empty_signal() {
        let dc = DispersionCompensator::new(17.0, 10.0, 1550.0);
        let result = dc.compensate(&[], 1e9);
        assert!(result.is_empty());
    }

    #[test]
    fn test_dispersion_zero_dispersion() {
        // With zero dispersion, output should equal input
        let dc = DispersionCompensator::new(0.0, 10.0, 1550.0);
        let signal: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
        ];
        let result = dc.compensate(&signal, 1e9);
        for (i, (&(si, sq), &(ri, rq))) in signal.iter().zip(result.iter()).enumerate() {
            assert_close(ri, si, 1e-8, &format!("Zero disp I[{}]", i));
            assert_close(rq, sq, 1e-8, &format!("Zero disp Q[{}]", i));
        }
    }

    #[test]
    fn test_rof_link_new() {
        let m = PhotonicModulator::new(4.0, 2.0);
        let r = OptoElectronicReceiver::new(0.8);
        let link = RofLink::new(m, 10.0, r);
        assert_eq!(link.fiber_length_km(), 10.0);
    }

    #[test]
    fn test_rof_link_budget() {
        let m = PhotonicModulator::new(4.0, 2.0); // 3 dB insertion loss
        let r = OptoElectronicReceiver::new(0.8);
        let link = RofLink::new(m, 10.0, r); // 10 km * 0.2 dB/km = 2 dB fiber loss
        let budget = link.link_budget_db();
        // -(2.0 + 3.0 + 1.0) = -6.0 dB
        assert_close(budget, -6.0, EPSILON, "Link budget");
    }

    #[test]
    fn test_rof_link_transmit_length() {
        let m = PhotonicModulator::new(4.0, 2.0);
        let r = OptoElectronicReceiver::new(0.8);
        let link = RofLink::new(m, 10.0, r);
        let rf = vec![(0.5, 0.0); 16];
        let result = link.transmit(&rf, 1e9);
        assert_eq!(result.len(), 16);
    }

    #[test]
    fn test_rof_link_empty_input() {
        let m = PhotonicModulator::new(4.0, 2.0);
        let r = OptoElectronicReceiver::new(0.8);
        let link = RofLink::new(m, 10.0, r);
        let result = link.transmit(&[], 1e9);
        assert!(result.is_empty());
    }

    #[test]
    fn test_rof_link_no_dispersion() {
        let m = PhotonicModulator::new(4.0, 2.0);
        let r = OptoElectronicReceiver::new(0.8);
        let link = RofLink::new(m, 10.0, r).with_dispersion(0.0);
        let rf = vec![(0.3, 0.1), (0.1, 0.3), (-0.2, 0.0)];
        let result = link.transmit(&rf, 1e9);
        // Verify all outputs are non-negative power (square-law detection)
        for (i, &(v, _)) in result.iter().enumerate() {
            assert!(v >= 0.0, "Square-law output[{}] should be >= 0, got {}", i, v);
        }
    }

    #[test]
    fn test_dft_idft_roundtrip() {
        let signal = vec![
            (1.0, 0.5), (0.3, -0.2), (-0.5, 0.7), (0.8, -0.1),
        ];
        let spectrum = dft(&signal);
        let recovered = idft(&spectrum);
        for (i, (&(si, sq), &(ri, rq))) in signal.iter().zip(recovered.iter()).enumerate() {
            assert_close(ri, si, 1e-10, &format!("DFT/IDFT roundtrip I[{}]", i));
            assert_close(rq, sq, 1e-10, &format!("DFT/IDFT roundtrip Q[{}]", i));
        }
    }

    #[test]
    fn test_rof_link_attenuation_scaling() {
        // Longer fiber should produce weaker output
        let m1 = PhotonicModulator::new(4.0, 2.0);
        let r1 = OptoElectronicReceiver::new(0.8);
        let link_short = RofLink::new(m1, 1.0, r1).with_dispersion(0.0);

        let m2 = PhotonicModulator::new(4.0, 2.0);
        let r2 = OptoElectronicReceiver::new(0.8);
        let link_long = RofLink::new(m2, 100.0, r2).with_dispersion(0.0);

        let rf = vec![(0.5, 0.0); 4];
        let out_short = link_short.transmit(&rf, 1e9);
        let out_long = link_long.transmit(&rf, 1e9);

        // Short fiber should have stronger signal than long fiber
        assert!(
            out_short[0].0 > out_long[0].0,
            "Short fiber ({}) should produce stronger signal than long fiber ({})",
            out_short[0].0,
            out_long[0].0
        );
    }

    #[test]
    fn test_modulator_insertion_loss_builder() {
        let m = PhotonicModulator::new(4.0, 2.0).with_insertion_loss_db(6.0);
        assert_close(m.insertion_loss_db, 6.0, EPSILON, "Insertion loss set");
        // 6 dB = factor of ~0.501 in amplitude
        assert_close(
            m.insertion_loss_linear(),
            0.5011872336272722,
            1e-6,
            "Insertion loss linear",
        );
    }
}
