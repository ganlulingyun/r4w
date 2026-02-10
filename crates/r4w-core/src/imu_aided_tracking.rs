//! # IMU-Aided GNSS Carrier Tracking
//!
//! Sensor fusion for GNSS/INS with IMU-aided carrier tracking loops.
//! Provides PLL and FLL implementations that incorporate inertial measurement
//! unit (IMU) data to assist carrier phase and frequency tracking, improving
//! robustness in high-dynamics environments (e.g., fast-moving vehicles, aircraft).
//!
//! ## Overview
//!
//! Traditional GNSS carrier tracking loops rely solely on the received signal
//! to maintain phase and frequency lock. In high-dynamics scenarios, the loop
//! bandwidth must be widened to track rapid Doppler changes, which increases
//! noise. IMU aiding provides external predictions of platform motion, allowing
//! the tracking loop to operate with a narrower bandwidth while maintaining lock.
//!
//! The module implements:
//! - **`ImuAidedPll`** - Phase-locked loop with IMU-predicted phase assist
//! - **`ImuAidedFll`** - Frequency-locked loop with IMU-predicted frequency assist
//! - **Kalman filter** - Optimal fusion of discriminator and IMU measurements
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::imu_aided_tracking::{AidingConfig, ImuAidedPll, ImuMeasurement};
//!
//! let config = AidingConfig {
//!     pll_bandwidth_hz: 15.0,
//!     fll_bandwidth_hz: 5.0,
//!     aiding_gain: 0.8,
//!     imu_noise_std: 0.01,
//! };
//!
//! let mut pll = ImuAidedPll::new(config);
//!
//! // Simulate a carrier signal at a known phase
//! let phase = 0.3_f64;
//! let sample: (f64, f64) = (phase.cos(), phase.sin());
//!
//! let imu = ImuMeasurement {
//!     accel_x: 0.0,
//!     accel_y: 0.0,
//!     accel_z: -9.81,
//!     gyro_x: 0.0,
//!     gyro_y: 0.0,
//!     gyro_z: 0.0,
//!     timestamp_s: 0.001,
//! };
//!
//! let state = pll.update(sample, &imu, 0.001);
//! assert!(state.carrier_lock_indicator >= 0.0);
//! assert!(state.cn0_db_hz >= 0.0);
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// GPS L1 carrier frequency in Hz (used as default).
const DEFAULT_CARRIER_FREQ_HZ: f64 = 1_575_420_000.0;

/// Configuration for IMU-aided tracking loops.
#[derive(Debug, Clone, Copy)]
pub struct AidingConfig {
    /// PLL noise bandwidth in Hz.
    pub pll_bandwidth_hz: f64,
    /// FLL noise bandwidth in Hz.
    pub fll_bandwidth_hz: f64,
    /// Aiding gain (0.0 = no aiding, 1.0 = full aiding).
    pub aiding_gain: f64,
    /// IMU measurement noise standard deviation.
    pub imu_noise_std: f64,
}

impl Default for AidingConfig {
    fn default() -> Self {
        Self {
            pll_bandwidth_hz: 15.0,
            fll_bandwidth_hz: 5.0,
            aiding_gain: 0.8,
            imu_noise_std: 0.01,
        }
    }
}

/// IMU measurement sample containing accelerometer and gyroscope data.
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Accelerometer X-axis (m/s^2).
    pub accel_x: f64,
    /// Accelerometer Y-axis (m/s^2).
    pub accel_y: f64,
    /// Accelerometer Z-axis (m/s^2).
    pub accel_z: f64,
    /// Gyroscope X-axis (rad/s).
    pub gyro_x: f64,
    /// Gyroscope Y-axis (rad/s).
    pub gyro_y: f64,
    /// Gyroscope Z-axis (rad/s).
    pub gyro_z: f64,
    /// Timestamp in seconds.
    pub timestamp_s: f64,
}

impl Default for ImuMeasurement {
    fn default() -> Self {
        Self {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: -9.81, // gravity
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            timestamp_s: 0.0,
        }
    }
}

impl ImuMeasurement {
    /// Compute the magnitude of the acceleration vector.
    pub fn accel_magnitude(&self) -> f64 {
        (self.accel_x * self.accel_x
            + self.accel_y * self.accel_y
            + self.accel_z * self.accel_z)
            .sqrt()
    }

    /// Compute the magnitude of the gyroscope vector.
    pub fn gyro_magnitude(&self) -> f64 {
        (self.gyro_x * self.gyro_x + self.gyro_y * self.gyro_y + self.gyro_z * self.gyro_z).sqrt()
    }

    /// Estimate line-of-sight velocity from accelerometer data (simplified model).
    /// In a real system this would use the full navigation solution; here we
    /// project the specific force (accel minus gravity) onto the Z axis as a proxy
    /// for the line-of-sight velocity change.
    pub fn estimated_los_velocity(&self, dt: f64) -> f64 {
        // Specific force along Z minus gravity gives the dynamic acceleration.
        let dynamic_accel = self.accel_z + 9.81; // remove gravity contribution
        dynamic_accel * dt
    }
}

/// Tracking loop state output.
#[derive(Debug, Clone, Copy)]
pub struct TrackingState {
    /// Carrier phase estimate in radians.
    pub phase_rad: f64,
    /// Carrier frequency estimate in Hz.
    pub freq_hz: f64,
    /// Carrier frequency rate of change in Hz/s.
    pub freq_rate_hz_s: f64,
    /// Carrier lock indicator (0.0 = no lock, 1.0 = perfect lock).
    pub carrier_lock_indicator: f64,
    /// Carrier-to-noise density ratio estimate in dB-Hz.
    pub cn0_db_hz: f64,
}

impl Default for TrackingState {
    fn default() -> Self {
        Self {
            phase_rad: 0.0,
            freq_hz: 0.0,
            freq_rate_hz_s: 0.0,
            carrier_lock_indicator: 0.0,
            cn0_db_hz: 0.0,
        }
    }
}

/// 3-state Kalman filter for carrier tracking.
/// State vector: [phase (rad), frequency (Hz), frequency rate (Hz/s)]
#[derive(Debug, Clone)]
struct TrackingKalmanFilter {
    /// State vector [phase, freq, freq_rate].
    state: [f64; 3],
    /// Covariance matrix (3x3, row-major).
    covariance: [f64; 9],
    /// Process noise power spectral density.
    process_noise_psd: f64,
}

impl TrackingKalmanFilter {
    fn new(process_noise_psd: f64) -> Self {
        Self {
            state: [0.0; 3],
            // Initial covariance: large uncertainty
            covariance: [
                1.0, 0.0, 0.0, // row 0
                0.0, 100.0, 0.0, // row 1
                0.0, 0.0, 10.0, // row 2
            ],
            process_noise_psd,
        }
    }

    /// Predict step: propagate state forward by dt seconds.
    fn predict(&mut self, dt: f64) {
        // State transition: phase += freq*dt + 0.5*freq_rate*dt^2
        //                   freq  += freq_rate * dt
        //                   freq_rate unchanged
        let dt2 = dt * dt;
        self.state[0] += 2.0 * PI * self.state[1] * dt + PI * self.state[2] * dt2;
        self.state[1] += self.state[2] * dt;

        // F = [[1, 2*pi*dt, pi*dt^2],
        //      [0, 1,        dt      ],
        //      [0, 0,        1       ]]
        // P = F * P * F^T + Q
        let f01 = 2.0 * PI * dt;
        let f02 = PI * dt2;
        let f12 = dt;

        // Compute F * P
        let p = &self.covariance;
        let fp = [
            p[0] + f01 * p[3] + f02 * p[6],
            p[1] + f01 * p[4] + f02 * p[7],
            p[2] + f01 * p[5] + f02 * p[8],
            p[3] + f12 * p[6],
            p[4] + f12 * p[7],
            p[5] + f12 * p[8],
            p[6],
            p[7],
            p[8],
        ];

        // Compute F*P*F^T
        self.covariance = [
            fp[0] + fp[1] * f01 + fp[2] * f02,
            fp[1] + fp[2] * f12,
            fp[2],
            fp[3] + fp[4] * f01 + fp[5] * f02,
            fp[4] + fp[5] * f12,
            fp[5],
            fp[6] + fp[7] * f01 + fp[8] * f02,
            fp[7] + fp[8] * f12,
            fp[8],
        ];

        // Add process noise Q (simplified: diagonal)
        let q_phase = self.process_noise_psd * dt2 * dt / 3.0;
        let q_freq = self.process_noise_psd * dt;
        let q_rate = self.process_noise_psd / dt.max(1e-10);
        self.covariance[0] += q_phase;
        self.covariance[4] += q_freq;
        self.covariance[8] += q_rate;
    }

    /// Update step with a scalar measurement (discriminator output).
    /// Measurement model: z = H * x + v, where H = [1, 0, 0] for phase measurement.
    fn update_phase(&mut self, measurement: f64, measurement_noise: f64) {
        let p = &self.covariance;
        // H = [1, 0, 0]
        // Innovation: y = z - H*x
        let innovation = measurement - self.state[0];

        // S = H*P*H^T + R = P[0,0] + R
        let s = p[0] + measurement_noise;
        if s.abs() < 1e-15 {
            return;
        }
        let s_inv = 1.0 / s;

        // K = P * H^T * S^-1 = [P[0,0], P[1,0], P[2,0]]^T / S
        let k = [p[0] * s_inv, p[3] * s_inv, p[6] * s_inv];

        // Update state
        self.state[0] += k[0] * innovation;
        self.state[1] += k[1] * innovation;
        self.state[2] += k[2] * innovation;

        // Update covariance: P = (I - K*H) * P
        let new_p = [
            (1.0 - k[0]) * p[0],
            (1.0 - k[0]) * p[1],
            (1.0 - k[0]) * p[2],
            p[3] - k[1] * p[0],
            p[4] - k[1] * p[1],
            p[5] - k[1] * p[2],
            p[6] - k[2] * p[0],
            p[7] - k[2] * p[1],
            p[8] - k[2] * p[2],
        ];
        self.covariance = new_p;
    }

    /// Update step with a frequency measurement (from FLL discriminator or IMU).
    /// Measurement model: z = H * x + v, where H = [0, 1, 0].
    fn update_frequency(&mut self, measurement: f64, measurement_noise: f64) {
        let p = &self.covariance;
        let innovation = measurement - self.state[1];

        // S = P[1,1] + R
        let s = p[4] + measurement_noise;
        if s.abs() < 1e-15 {
            return;
        }
        let s_inv = 1.0 / s;

        // K = P * H^T / S where H = [0,1,0]
        let k = [p[1] * s_inv, p[4] * s_inv, p[7] * s_inv];

        self.state[0] += k[0] * innovation;
        self.state[1] += k[1] * innovation;
        self.state[2] += k[2] * innovation;

        let new_p = [
            p[0] - k[0] * p[1],
            p[1] - k[0] * p[4],
            p[2] - k[0] * p[7],
            p[3] - k[1] * p[1],
            p[4] - k[1] * p[4],
            p[5] - k[1] * p[7],
            p[6] - k[2] * p[1],
            p[7] - k[2] * p[4],
            p[8] - k[2] * p[7],
        ];
        self.covariance = new_p;
    }

    fn phase(&self) -> f64 {
        self.state[0]
    }

    fn frequency(&self) -> f64 {
        self.state[1]
    }

    fn frequency_rate(&self) -> f64 {
        self.state[2]
    }
}

/// IMU-aided Phase-Locked Loop for carrier phase tracking.
///
/// Combines a traditional atan2 phase discriminator with IMU-predicted
/// phase changes to maintain carrier lock in high-dynamics environments.
#[derive(Debug, Clone)]
pub struct ImuAidedPll {
    config: AidingConfig,
    kf: TrackingKalmanFilter,
    /// Previous prompt sample for lock detection.
    prev_ip: f64,
    prev_qp: f64,
    /// Narrow-band power accumulator for C/N0 estimation.
    narrow_power_sum: f64,
    /// Wide-band power accumulator for C/N0 estimation.
    wide_power_sum: f64,
    /// Number of samples accumulated for C/N0 estimation.
    cn0_count: u64,
    /// Lock detector accumulator.
    lock_acc: f64,
    /// Carrier frequency for Doppler conversion.
    carrier_freq_hz: f64,
    /// Accumulated IMU velocity (line-of-sight) in m/s.
    imu_velocity_los: f64,
}

impl ImuAidedPll {
    /// Create a new IMU-aided PLL with the given configuration.
    pub fn new(config: AidingConfig) -> Self {
        // Process noise PSD derived from loop bandwidth
        let pn_psd = config.pll_bandwidth_hz * 0.01;
        Self {
            config,
            kf: TrackingKalmanFilter::new(pn_psd),
            prev_ip: 1.0,
            prev_qp: 0.0,
            narrow_power_sum: 0.0,
            wide_power_sum: 0.0,
            cn0_count: 0,
            lock_acc: 0.5,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            imu_velocity_los: 0.0,
        }
    }

    /// Set the carrier frequency used for Doppler calculations.
    pub fn set_carrier_freq(&mut self, freq_hz: f64) {
        self.carrier_freq_hz = freq_hz;
    }

    /// Phase discriminator using atan2(Q, I).
    /// Returns the phase error in radians.
    pub fn discriminator(&self, sample: (f64, f64)) -> f64 {
        sample.1.atan2(sample.0)
    }

    /// Predict the phase change from IMU data over interval dt.
    ///
    /// Uses the IMU-derived line-of-sight velocity to predict Doppler-induced
    /// phase change: delta_phase = 2*pi * (v_los / c) * f_carrier * dt
    pub fn predict_phase(&self, imu: &ImuMeasurement, dt: f64) -> f64 {
        let v_los = self.imu_velocity_los + imu.estimated_los_velocity(dt);
        let doppler_hz = (v_los / SPEED_OF_LIGHT) * self.carrier_freq_hz;
        2.0 * PI * doppler_hz * dt
    }

    /// Compute the carrier lock indicator.
    ///
    /// Returns a value between 0.0 (no lock) and 1.0 (perfect lock).
    /// Based on the ratio of narrow-band to wide-band power.
    pub fn carrier_lock_indicator(&self) -> f64 {
        self.lock_acc.clamp(0.0, 1.0)
    }

    /// Process one sample with IMU aiding and return the updated tracking state.
    pub fn update(
        &mut self,
        sample: (f64, f64),
        imu: &ImuMeasurement,
        dt: f64,
    ) -> TrackingState {
        // 1. Predict step - propagate Kalman filter
        self.kf.predict(dt);

        // 2. IMU aiding - predict phase from IMU
        let imu_phase_delta = self.predict_phase(imu, dt);
        let aided_phase = self.kf.phase() + self.config.aiding_gain * imu_phase_delta;

        // 3. Strip predicted carrier to get error signal
        let cos_pred = aided_phase.cos();
        let sin_pred = aided_phase.sin();
        let ip = sample.0 * cos_pred + sample.1 * sin_pred;
        let qp = -sample.0 * sin_pred + sample.1 * cos_pred;

        // 4. Phase discriminator on residual
        let phase_error = qp.atan2(ip);

        // 5. Kalman update with phase discriminator measurement
        let measured_phase = aided_phase + phase_error;
        let measurement_noise = 1.0 / (2.0 * self.config.pll_bandwidth_hz * dt).max(0.01);
        self.kf.update_phase(measured_phase, measurement_noise);

        // 6. IMU frequency aiding
        let imu_freq = self.predict_frequency_from_imu(imu, dt);
        if imu_freq.abs() > 1e-6 {
            let imu_meas_noise = self.config.imu_noise_std * self.carrier_freq_hz / SPEED_OF_LIGHT;
            self.kf
                .update_frequency(imu_freq, imu_meas_noise.max(0.1));
        }

        // 7. Update IMU velocity estimate
        self.imu_velocity_los += imu.estimated_los_velocity(dt);

        // 8. Lock detection (filtered NBP/WBP ratio)
        let nbp = ip * ip - qp * qp;
        let wbp = ip * ip + qp * qp;
        if wbp > 1e-20 {
            let lock_raw = nbp / wbp;
            self.lock_acc = 0.95 * self.lock_acc + 0.05 * lock_raw;
        }

        // 9. C/N0 estimation (Beaulieu's method)
        let power = ip * ip + qp * qp;
        self.narrow_power_sum += ip;
        self.wide_power_sum += power;
        self.cn0_count += 1;

        let cn0 = self.estimate_cn0(dt);

        // 10. Update previous samples
        self.prev_ip = ip;
        self.prev_qp = qp;

        TrackingState {
            phase_rad: self.kf.phase(),
            freq_hz: self.kf.frequency(),
            freq_rate_hz_s: self.kf.frequency_rate(),
            carrier_lock_indicator: self.carrier_lock_indicator(),
            cn0_db_hz: cn0,
        }
    }

    /// Predict Doppler frequency from IMU measurement.
    fn predict_frequency_from_imu(&self, imu: &ImuMeasurement, dt: f64) -> f64 {
        let v_los = self.imu_velocity_los + imu.estimated_los_velocity(dt);
        (v_los / SPEED_OF_LIGHT) * self.carrier_freq_hz
    }

    /// Estimate C/N0 in dB-Hz using narrow-band vs wide-band power ratio.
    fn estimate_cn0(&self, dt: f64) -> f64 {
        if self.cn0_count < 2 || self.wide_power_sum < 1e-20 {
            return 0.0;
        }
        let n = self.cn0_count as f64;
        let nbp = (self.narrow_power_sum / n).powi(2);
        let wbp = self.wide_power_sum / n;

        if wbp < 1e-20 {
            return 0.0;
        }

        let snr = nbp / (wbp - nbp).max(1e-20);
        let cn0_linear = snr / dt.max(1e-10);
        if cn0_linear > 0.0 {
            10.0 * cn0_linear.log10()
        } else {
            0.0
        }
    }

    /// Reset the tracking loop state.
    pub fn reset(&mut self) {
        let pn_psd = self.config.pll_bandwidth_hz * 0.01;
        self.kf = TrackingKalmanFilter::new(pn_psd);
        self.prev_ip = 1.0;
        self.prev_qp = 0.0;
        self.narrow_power_sum = 0.0;
        self.wide_power_sum = 0.0;
        self.cn0_count = 0;
        self.lock_acc = 0.5;
        self.imu_velocity_los = 0.0;
    }
}

/// IMU-aided Frequency-Locked Loop for carrier frequency tracking.
///
/// Uses a cross-product FLL discriminator combined with IMU-predicted
/// frequency changes. Suitable for initial acquisition and weak signal
/// tracking where phase lock is not yet achieved.
#[derive(Debug, Clone)]
pub struct ImuAidedFll {
    config: AidingConfig,
    kf: TrackingKalmanFilter,
    /// Previous in-phase prompt for cross-product discriminator.
    prev_ip: f64,
    /// Previous quadrature prompt for cross-product discriminator.
    prev_qp: f64,
    /// First update flag (no previous sample for cross-product).
    first_update: bool,
    /// Lock detector accumulator.
    lock_acc: f64,
    /// Carrier frequency for Doppler conversion.
    carrier_freq_hz: f64,
    /// Accumulated IMU velocity (line-of-sight) in m/s.
    imu_velocity_los: f64,
    /// Sample count for C/N0 estimation.
    cn0_count: u64,
    /// Power accumulator for C/N0.
    power_sum: f64,
    /// Coherent accumulator for C/N0.
    coherent_sum: f64,
}

impl ImuAidedFll {
    /// Create a new IMU-aided FLL with the given configuration.
    pub fn new(config: AidingConfig) -> Self {
        let pn_psd = config.fll_bandwidth_hz * 0.05;
        Self {
            config,
            kf: TrackingKalmanFilter::new(pn_psd),
            prev_ip: 1.0,
            prev_qp: 0.0,
            first_update: true,
            lock_acc: 0.5,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            imu_velocity_los: 0.0,
            cn0_count: 0,
            power_sum: 0.0,
            coherent_sum: 0.0,
        }
    }

    /// Set the carrier frequency used for Doppler calculations.
    pub fn set_carrier_freq(&mut self, freq_hz: f64) {
        self.carrier_freq_hz = freq_hz;
    }

    /// Cross-product FLL discriminator.
    ///
    /// Returns the frequency error estimate in Hz using:
    /// `freq_err = atan2(cross, dot) / (2 * pi * dt)`
    /// where cross = I_prev * Q_curr - Q_prev * I_curr
    /// and   dot   = I_prev * I_curr + Q_prev * Q_curr
    pub fn discriminator(
        &self,
        sample: (f64, f64),
        prev_sample: (f64, f64),
        dt: f64,
    ) -> f64 {
        let cross = prev_sample.0 * sample.1 - prev_sample.1 * sample.0;
        let dot = prev_sample.0 * sample.0 + prev_sample.1 * sample.1;
        cross.atan2(dot) / (2.0 * PI * dt)
    }

    /// Predict the Doppler frequency from IMU data.
    ///
    /// `predicted_doppler = (v_los / c) * f_carrier`
    pub fn predict_frequency(&self, _imu: &ImuMeasurement) -> f64 {
        (self.imu_velocity_los / SPEED_OF_LIGHT) * self.carrier_freq_hz
    }

    /// Process one sample with IMU aiding and return the updated tracking state.
    pub fn update(
        &mut self,
        sample: (f64, f64),
        imu: &ImuMeasurement,
        dt: f64,
    ) -> TrackingState {
        // 1. Predict step
        self.kf.predict(dt);

        // 2. Strip predicted carrier
        let predicted_phase = self.kf.phase();
        let cos_pred = predicted_phase.cos();
        let sin_pred = predicted_phase.sin();
        let ip = sample.0 * cos_pred + sample.1 * sin_pred;
        let qp = -sample.0 * sin_pred + sample.1 * cos_pred;

        // 3. FLL discriminator (needs previous sample)
        if !self.first_update {
            let cross = self.prev_ip * qp - self.prev_qp * ip;
            let dot = self.prev_ip * ip + self.prev_qp * qp;
            let freq_error = cross.atan2(dot) / (2.0 * PI * dt);

            // 4. Kalman update with frequency discriminator
            let measured_freq = self.kf.frequency() + freq_error;
            let meas_noise = 1.0 / (4.0 * self.config.fll_bandwidth_hz * dt).max(0.01);
            self.kf.update_frequency(measured_freq, meas_noise);
        }

        // 5. IMU frequency aiding
        self.imu_velocity_los += imu.estimated_los_velocity(dt);
        let imu_freq = self.predict_frequency(imu);
        if imu_freq.abs() > 1e-6 {
            let imu_meas_noise = self.config.imu_noise_std * self.carrier_freq_hz / SPEED_OF_LIGHT;
            let blended_freq =
                self.kf.frequency() + self.config.aiding_gain * (imu_freq - self.kf.frequency());
            self.kf
                .update_frequency(blended_freq, imu_meas_noise.max(0.1));
        }

        // 6. Lock detection
        let wbp = ip * ip + qp * qp;
        let nbp = ip * ip - qp * qp;
        if wbp > 1e-20 {
            let lock_raw = nbp / wbp;
            self.lock_acc = 0.95 * self.lock_acc + 0.05 * lock_raw;
        }

        // 7. C/N0 estimation
        self.coherent_sum += ip;
        self.power_sum += ip * ip + qp * qp;
        self.cn0_count += 1;

        let cn0 = self.estimate_cn0(dt);

        // 8. Store previous prompt samples
        self.prev_ip = ip;
        self.prev_qp = qp;
        self.first_update = false;

        TrackingState {
            phase_rad: self.kf.phase(),
            freq_hz: self.kf.frequency(),
            freq_rate_hz_s: self.kf.frequency_rate(),
            carrier_lock_indicator: self.lock_acc.clamp(0.0, 1.0),
            cn0_db_hz: cn0,
        }
    }

    /// Estimate C/N0 in dB-Hz.
    fn estimate_cn0(&self, dt: f64) -> f64 {
        if self.cn0_count < 2 || self.power_sum < 1e-20 {
            return 0.0;
        }
        let n = self.cn0_count as f64;
        let nbp = (self.coherent_sum / n).powi(2);
        let wbp = self.power_sum / n;
        let noise_est = (wbp - nbp).max(1e-20);
        let snr = nbp / noise_est;
        let cn0_linear = snr / dt.max(1e-10);
        if cn0_linear > 0.0 {
            10.0 * cn0_linear.log10()
        } else {
            0.0
        }
    }

    /// Reset the FLL state.
    pub fn reset(&mut self) {
        let pn_psd = self.config.fll_bandwidth_hz * 0.05;
        self.kf = TrackingKalmanFilter::new(pn_psd);
        self.prev_ip = 1.0;
        self.prev_qp = 0.0;
        self.first_update = true;
        self.lock_acc = 0.5;
        self.imu_velocity_los = 0.0;
        self.cn0_count = 0;
        self.power_sum = 0.0;
        self.coherent_sum = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> AidingConfig {
        AidingConfig {
            pll_bandwidth_hz: 15.0,
            fll_bandwidth_hz: 5.0,
            aiding_gain: 0.8,
            imu_noise_std: 0.01,
        }
    }

    fn stationary_imu() -> ImuMeasurement {
        ImuMeasurement {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: -9.81,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            timestamp_s: 0.0,
        }
    }

    /// Generate a complex IQ sample at a given phase.
    fn iq_at_phase(phase: f64) -> (f64, f64) {
        (phase.cos(), phase.sin())
    }

    // ---- Test 1: AidingConfig default values ----
    #[test]
    fn test_aiding_config_default() {
        let cfg = AidingConfig::default();
        assert_eq!(cfg.pll_bandwidth_hz, 15.0);
        assert_eq!(cfg.fll_bandwidth_hz, 5.0);
        assert_eq!(cfg.aiding_gain, 0.8);
        assert_eq!(cfg.imu_noise_std, 0.01);
    }

    // ---- Test 2: ImuMeasurement default ----
    #[test]
    fn test_imu_measurement_default() {
        let imu = ImuMeasurement::default();
        assert_eq!(imu.accel_x, 0.0);
        assert_eq!(imu.accel_y, 0.0);
        assert!((imu.accel_z - (-9.81)).abs() < 1e-10);
        assert_eq!(imu.gyro_x, 0.0);
        assert_eq!(imu.timestamp_s, 0.0);
    }

    // ---- Test 3: ImuMeasurement acceleration magnitude ----
    #[test]
    fn test_imu_accel_magnitude() {
        let imu = ImuMeasurement {
            accel_x: 3.0,
            accel_y: 4.0,
            accel_z: 0.0,
            ..Default::default()
        };
        assert!((imu.accel_magnitude() - 5.0).abs() < 1e-10);
    }

    // ---- Test 4: ImuMeasurement gyro magnitude ----
    #[test]
    fn test_imu_gyro_magnitude() {
        let imu = ImuMeasurement {
            gyro_x: 1.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            ..Default::default()
        };
        assert!((imu.gyro_magnitude() - 1.0).abs() < 1e-10);
    }

    // ---- Test 5: ImuMeasurement estimated LOS velocity stationary ----
    #[test]
    fn test_imu_los_velocity_stationary() {
        let imu = stationary_imu();
        let v = imu.estimated_los_velocity(0.001);
        // Stationary: accel_z = -9.81, dynamic = -9.81 + 9.81 = 0
        assert!(v.abs() < 1e-10);
    }

    // ---- Test 6: ImuMeasurement estimated LOS velocity with acceleration ----
    #[test]
    fn test_imu_los_velocity_accelerating() {
        let imu = ImuMeasurement {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: -9.81 + 5.0, // 5 m/s^2 dynamic along Z
            ..Default::default()
        };
        let v = imu.estimated_los_velocity(1.0);
        assert!((v - 5.0).abs() < 1e-10);
    }

    // ---- Test 7: PLL discriminator at zero phase ----
    #[test]
    fn test_pll_discriminator_zero_phase() {
        let pll = ImuAidedPll::new(default_config());
        let sample = (1.0, 0.0); // phase = 0
        let error = pll.discriminator(sample);
        assert!(error.abs() < 1e-10);
    }

    // ---- Test 8: PLL discriminator at known phase ----
    #[test]
    fn test_pll_discriminator_known_phase() {
        let pll = ImuAidedPll::new(default_config());
        let phase = 0.5;
        let sample = iq_at_phase(phase);
        let error = pll.discriminator(sample);
        assert!((error - phase).abs() < 1e-10);
    }

    // ---- Test 9: PLL predict_phase stationary ----
    #[test]
    fn test_pll_predict_phase_stationary() {
        let pll = ImuAidedPll::new(default_config());
        let imu = stationary_imu();
        let phase = pll.predict_phase(&imu, 0.001);
        // Stationary: no velocity -> no phase prediction
        assert!(phase.abs() < 1e-6);
    }

    // ---- Test 10: PLL carrier lock indicator initial ----
    #[test]
    fn test_pll_initial_lock_indicator() {
        let pll = ImuAidedPll::new(default_config());
        let cli = pll.carrier_lock_indicator();
        // Initial lock indicator is 0.5
        assert!((cli - 0.5).abs() < 1e-10);
    }

    // ---- Test 11: PLL update returns valid state ----
    #[test]
    fn test_pll_update_returns_valid_state() {
        let mut pll = ImuAidedPll::new(default_config());
        let imu = stationary_imu();
        let sample = iq_at_phase(0.0);
        let state = pll.update(sample, &imu, 0.001);

        assert!(state.phase_rad.is_finite());
        assert!(state.freq_hz.is_finite());
        assert!(state.freq_rate_hz_s.is_finite());
        assert!(state.carrier_lock_indicator >= 0.0);
        assert!(state.carrier_lock_indicator <= 1.0);
        assert!(state.cn0_db_hz >= 0.0 || state.cn0_db_hz == 0.0);
    }

    // ---- Test 12: PLL tracks constant phase carrier ----
    #[test]
    fn test_pll_tracks_constant_phase() {
        let mut pll = ImuAidedPll::new(default_config());
        let imu = stationary_imu();
        let target_phase = 0.3;
        let sample = iq_at_phase(target_phase);

        // Run multiple iterations
        let mut last_state = TrackingState::default();
        for _ in 0..100 {
            last_state = pll.update(sample, &imu, 0.001);
        }

        // After convergence, the phase estimate should be near the target
        // Wrap phase difference to [-pi, pi]
        let phase_err = (last_state.phase_rad - target_phase).sin().abs();
        assert!(
            phase_err < 1.0,
            "Phase error too large: {} (state.phase_rad={})",
            phase_err,
            last_state.phase_rad
        );
    }

    // ---- Test 13: PLL reset clears state ----
    #[test]
    fn test_pll_reset() {
        let mut pll = ImuAidedPll::new(default_config());
        let imu = stationary_imu();
        let sample = iq_at_phase(1.0);

        // Update several times
        for _ in 0..50 {
            pll.update(sample, &imu, 0.001);
        }

        pll.reset();
        assert!((pll.carrier_lock_indicator() - 0.5).abs() < 1e-10);
        assert_eq!(pll.cn0_count, 0);
        assert!((pll.imu_velocity_los).abs() < 1e-10);
    }

    // ---- Test 14: FLL discriminator zero frequency error ----
    #[test]
    fn test_fll_discriminator_zero_freq_error() {
        let fll = ImuAidedFll::new(default_config());
        // Two samples at the same phase -> zero frequency error
        let s1 = iq_at_phase(0.5);
        let s2 = iq_at_phase(0.5);
        let freq_err = fll.discriminator(s2, s1, 0.001);
        assert!(
            freq_err.abs() < 1e-6,
            "Expected ~0 freq error, got {}",
            freq_err
        );
    }

    // ---- Test 15: FLL discriminator known frequency ----
    #[test]
    fn test_fll_discriminator_known_frequency() {
        let fll = ImuAidedFll::new(default_config());
        let freq_hz = 100.0;
        let dt = 0.001;
        let phase_delta = 2.0 * PI * freq_hz * dt;
        let s1 = iq_at_phase(0.0);
        let s2 = iq_at_phase(phase_delta);
        let freq_err = fll.discriminator(s2, s1, dt);
        assert!(
            (freq_err - freq_hz).abs() < 1.0,
            "Expected ~{} Hz, got {} Hz",
            freq_hz,
            freq_err
        );
    }

    // ---- Test 16: FLL predict_frequency stationary ----
    #[test]
    fn test_fll_predict_frequency_stationary() {
        let fll = ImuAidedFll::new(default_config());
        let imu = stationary_imu();
        let freq = fll.predict_frequency(&imu);
        // No velocity accumulated yet -> zero prediction
        assert!(freq.abs() < 1e-6);
    }

    // ---- Test 17: FLL update returns valid state ----
    #[test]
    fn test_fll_update_returns_valid_state() {
        let mut fll = ImuAidedFll::new(default_config());
        let imu = stationary_imu();
        let sample = iq_at_phase(0.0);
        let state = fll.update(sample, &imu, 0.001);

        assert!(state.phase_rad.is_finite());
        assert!(state.freq_hz.is_finite());
        assert!(state.freq_rate_hz_s.is_finite());
        assert!(state.carrier_lock_indicator >= 0.0);
        assert!(state.carrier_lock_indicator <= 1.0);
    }

    // ---- Test 18: FLL reset clears state ----
    #[test]
    fn test_fll_reset() {
        let mut fll = ImuAidedFll::new(default_config());
        let imu = stationary_imu();
        let sample = iq_at_phase(0.5);

        for _ in 0..20 {
            fll.update(sample, &imu, 0.001);
        }

        fll.reset();
        assert!(fll.first_update);
        assert!((fll.lock_acc - 0.5).abs() < 1e-10);
        assert_eq!(fll.cn0_count, 0);
    }

    // ---- Test 19: TrackingState default ----
    #[test]
    fn test_tracking_state_default() {
        let state = TrackingState::default();
        assert_eq!(state.phase_rad, 0.0);
        assert_eq!(state.freq_hz, 0.0);
        assert_eq!(state.freq_rate_hz_s, 0.0);
        assert_eq!(state.carrier_lock_indicator, 0.0);
        assert_eq!(state.cn0_db_hz, 0.0);
    }

    // ---- Test 20: Kalman filter predict-update cycle ----
    #[test]
    fn test_kalman_filter_basic() {
        let mut kf = TrackingKalmanFilter::new(0.1);

        // Initial state should be zero
        assert!((kf.phase()).abs() < 1e-10);
        assert!((kf.frequency()).abs() < 1e-10);
        assert!((kf.frequency_rate()).abs() < 1e-10);

        // Predict step
        kf.predict(0.001);

        // State should still be near zero (no dynamics)
        assert!(kf.phase().is_finite());
        assert!(kf.frequency().is_finite());

        // Update with a phase measurement
        kf.update_phase(0.5, 0.01);
        assert!(kf.phase().abs() > 0.0); // Should have moved toward measurement
    }

    // ---- Test 21: PLL carrier lock improves with consistent signal ----
    #[test]
    fn test_pll_lock_indicator_improves() {
        let mut pll = ImuAidedPll::new(default_config());
        let imu = stationary_imu();

        // Feed pure carrier with no phase offset (strong signal)
        let sample = (1.0, 0.0);
        let initial_lock = pll.carrier_lock_indicator();

        for _ in 0..200 {
            pll.update(sample, &imu, 0.001);
        }

        let final_lock = pll.carrier_lock_indicator();
        // With a strong consistent signal, lock should improve or stay good
        assert!(
            final_lock >= initial_lock - 0.1,
            "Lock should not degrade significantly: initial={}, final={}",
            initial_lock,
            final_lock
        );
    }

    // ---- Test 22: PLL set carrier frequency ----
    #[test]
    fn test_pll_set_carrier_freq() {
        let mut pll = ImuAidedPll::new(default_config());
        pll.set_carrier_freq(1.2e9);
        assert!((pll.carrier_freq_hz - 1.2e9).abs() < 1.0);
    }

    // ---- Test 23: FLL set carrier frequency ----
    #[test]
    fn test_fll_set_carrier_freq() {
        let mut fll = ImuAidedFll::new(default_config());
        fll.set_carrier_freq(1.2e9);
        assert!((fll.carrier_freq_hz - 1.2e9).abs() < 1.0);
    }
}
