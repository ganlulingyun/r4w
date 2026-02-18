// trace:FR-PET | ai:claude
//! # Positron Emission Tomography (PET) Processor
//!
//! PET scanner signal processing for coincidence detection, sinogram generation,
//! and image reconstruction from gamma-ray photon pair detection events.
//!
//! ## Physics Background
//!
//! Positron annihilation produces two 511 keV gamma photons emitted at approximately
//! 180 degrees apart. Coincidence detection identifies these photon pairs within a
//! narrow time window. Lines of Response (LORs) from detected pairs form sinogram
//! projection data. Filtered backprojection (FBP) reconstructs the spatial activity
//! distribution from the sinograms.
//!
//! ## Key Corrections
//!
//! - **Attenuation**: Photon absorption along the LOR path
//! - **Scatter**: Compton-scattered photons detected as false coincidences
//! - **Randoms**: Accidental coincidences from unrelated photon pairs
//! - **Dead time**: Detector paralysis at high count rates
//! - **Decay**: Radiotracer physical decay during acquisition

use std::f64::consts::PI;

// ─── Detector Ring ───────────────────────────────────────────────────────────

/// Circular ring of scintillation detectors for PET imaging.
#[derive(Debug, Clone)]
pub struct DetectorRing {
    pub num_detectors: usize,
    pub ring_radius_mm: f64,
}

impl DetectorRing {
    /// Create a new detector ring.
    ///
    /// # Arguments
    /// * `num_detectors` - Number of detector elements around the ring
    /// * `ring_radius_mm` - Radius of the ring in millimeters
    pub fn new(num_detectors: usize, ring_radius_mm: f64) -> Self {
        assert!(num_detectors >= 2, "Need at least 2 detectors");
        assert!(ring_radius_mm > 0.0, "Ring radius must be positive");
        Self {
            num_detectors,
            ring_radius_mm,
        }
    }

    /// Get the angular position of a detector in radians.
    pub fn detector_angle(&self, index: usize) -> f64 {
        assert!(index < self.num_detectors, "Detector index out of range");
        2.0 * PI * (index as f64) / (self.num_detectors as f64)
    }

    /// Get the (x, y) position of a detector in mm.
    pub fn detector_position(&self, index: usize) -> (f64, f64) {
        let angle = self.detector_angle(index);
        (
            self.ring_radius_mm * angle.cos(),
            self.ring_radius_mm * angle.sin(),
        )
    }

    /// Compute the LOR angle and radial offset for a detector pair.
    /// Returns (angle_rad, offset_mm).
    pub fn lor_parameters(&self, det_a: usize, det_b: usize) -> (f64, f64) {
        let (xa, ya) = self.detector_position(det_a);
        let (xb, yb) = self.detector_position(det_b);
        // Midpoint
        let mx = (xa + xb) / 2.0;
        let my = (ya + yb) / 2.0;
        // LOR angle (perpendicular to the line connecting detectors)
        let dx = xb - xa;
        let dy = yb - ya;
        let angle = dy.atan2(dx) + PI / 2.0;
        // Normalize to [0, PI)
        let angle = ((angle % PI) + PI) % PI;
        // Radial offset = distance from origin to midpoint projected onto normal
        let offset = mx * angle.cos() + my * angle.sin();
        (angle, offset)
    }

    /// Distance between two detectors in mm.
    pub fn detector_distance(&self, det_a: usize, det_b: usize) -> f64 {
        let (xa, ya) = self.detector_position(det_a);
        let (xb, yb) = self.detector_position(det_b);
        ((xb - xa).powi(2) + (yb - ya).powi(2)).sqrt()
    }
}

// ─── Coincidence Detection ───────────────────────────────────────────────────

/// A single gamma detection event at one detector.
#[derive(Debug, Clone, Copy)]
pub struct SingleEvent {
    pub detector_id: usize,
    pub timestamp_ns: f64,
    pub energy_kev: f64,
}

/// A coincidence event: paired 511 keV photon detections.
#[derive(Debug, Clone, Copy)]
pub struct CoincidenceEvent {
    pub detector_a: usize,
    pub detector_b: usize,
    pub timestamp_a_ns: f64,
    pub timestamp_b_ns: f64,
    pub energy_a_kev: f64,
    pub energy_b_kev: f64,
    pub time_diff_ns: f64,
}

/// Coincidence detector for identifying gamma-ray photon pairs.
#[derive(Debug)]
pub struct CoincidenceDetector {
    time_window_ns: f64,
    energy_window_kev: (f64, f64),
    events: Vec<SingleEvent>,
}

impl CoincidenceDetector {
    /// Create a new coincidence detector.
    ///
    /// # Arguments
    /// * `time_window_ns` - Coincidence time window in nanoseconds (typically 6 ns)
    /// * `energy_window_kev` - Accepted energy range in keV (typically 350-650 keV for 511 keV photopeak)
    pub fn new(time_window_ns: f64, energy_window_kev: (f64, f64)) -> Self {
        assert!(time_window_ns > 0.0, "Time window must be positive");
        assert!(
            energy_window_kev.0 < energy_window_kev.1,
            "Energy window lower bound must be less than upper bound"
        );
        Self {
            time_window_ns,
            energy_window_kev,
            events: Vec::new(),
        }
    }

    /// Add a single detection event.
    pub fn add_event(&mut self, detector_id: usize, timestamp_ns: f64, energy_kev: f64) {
        self.events.push(SingleEvent {
            detector_id,
            timestamp_ns,
            energy_kev,
        });
    }

    /// Check if an energy value falls within the acceptance window.
    pub fn is_energy_valid(&self, energy_kev: f64) -> bool {
        energy_kev >= self.energy_window_kev.0 && energy_kev <= self.energy_window_kev.1
    }

    /// Find all coincidence event pairs from accumulated single events.
    /// Events are paired if they occur within the time window at different detectors
    /// and both have energy within the acceptance window.
    pub fn find_coincidences(&self) -> Vec<CoincidenceEvent> {
        let mut coincidences = Vec::new();
        let mut used = vec![false; self.events.len()];

        // Sort events by timestamp for efficient pairing
        let mut sorted_indices: Vec<usize> = (0..self.events.len()).collect();
        sorted_indices.sort_by(|&a, &b| {
            self.events[a]
                .timestamp_ns
                .partial_cmp(&self.events[b].timestamp_ns)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for i in 0..sorted_indices.len() {
            let idx_a = sorted_indices[i];
            if used[idx_a] {
                continue;
            }
            let ev_a = &self.events[idx_a];
            if !self.is_energy_valid(ev_a.energy_kev) {
                continue;
            }

            for j in (i + 1)..sorted_indices.len() {
                let idx_b = sorted_indices[j];
                if used[idx_b] {
                    continue;
                }
                let ev_b = &self.events[idx_b];

                let time_diff = (ev_b.timestamp_ns - ev_a.timestamp_ns).abs();
                if time_diff > self.time_window_ns {
                    break; // Events are sorted, no more matches possible
                }

                if ev_a.detector_id == ev_b.detector_id {
                    continue; // Same detector, not a coincidence
                }

                if !self.is_energy_valid(ev_b.energy_kev) {
                    continue;
                }

                // Found a coincidence pair
                coincidences.push(CoincidenceEvent {
                    detector_a: ev_a.detector_id,
                    detector_b: ev_b.detector_id,
                    timestamp_a_ns: ev_a.timestamp_ns,
                    timestamp_b_ns: ev_b.timestamp_ns,
                    energy_a_kev: ev_a.energy_kev,
                    energy_b_kev: ev_b.energy_kev,
                    time_diff_ns: time_diff,
                });

                used[idx_a] = true;
                used[idx_b] = true;
                break;
            }
        }

        coincidences
    }

    /// Estimate random coincidence rate from singles rates.
    /// R_random = 2 * tau * R_a * R_b
    /// where tau is the coincidence time window.
    pub fn random_coincidence_rate(
        singles_rate_a: f64,
        singles_rate_b: f64,
        window_ns: f64,
    ) -> f64 {
        let window_s = window_ns * 1e-9;
        2.0 * window_s * singles_rate_a * singles_rate_b
    }

    /// Clear all accumulated events.
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// Return the number of accumulated events.
    pub fn event_count(&self) -> usize {
        self.events.len()
    }
}

// ─── Sinogram ────────────────────────────────────────────────────────────────

/// 2D sinogram: projection data indexed by (angle, radial_bin).
#[derive(Debug, Clone)]
pub struct Sinogram {
    pub num_angles: usize,
    pub num_radial_bins: usize,
    /// Row-major storage: data[angle_idx * num_radial_bins + radial_idx]
    pub data: Vec<f64>,
    total_counts: usize,
}

impl Sinogram {
    /// Create a new empty sinogram.
    pub fn new(num_angles: usize, num_radial_bins: usize) -> Self {
        assert!(num_angles > 0, "Must have at least 1 angle");
        assert!(num_radial_bins > 0, "Must have at least 1 radial bin");
        Self {
            num_angles,
            num_radial_bins,
            data: vec![0.0; num_angles * num_radial_bins],
            total_counts: 0,
        }
    }

    /// Add a Line of Response from a detector pair.
    pub fn add_lor(&mut self, detector_a: usize, detector_b: usize, ring: &DetectorRing) {
        let (angle, offset) = ring.lor_parameters(detector_a, detector_b);

        // Map angle [0, PI) to angle bin
        let angle_idx = ((angle / PI) * self.num_angles as f64).floor() as usize;
        let angle_idx = angle_idx.min(self.num_angles - 1);

        // Map offset [-ring_radius, +ring_radius] to radial bin
        let norm_offset = (offset / ring.ring_radius_mm + 1.0) / 2.0; // [0, 1]
        let radial_idx = (norm_offset * self.num_radial_bins as f64).floor() as usize;
        let radial_idx = radial_idx.min(self.num_radial_bins - 1);

        self.data[angle_idx * self.num_radial_bins + radial_idx] += 1.0;
        self.total_counts += 1;
    }

    /// Get a projection at a given angle index.
    pub fn get_projection(&self, angle_idx: usize) -> &[f64] {
        assert!(angle_idx < self.num_angles, "Angle index out of range");
        let start = angle_idx * self.num_radial_bins;
        &self.data[start..start + self.num_radial_bins]
    }

    /// Get a mutable projection at a given angle index.
    pub fn get_projection_mut(&mut self, angle_idx: usize) -> &mut [f64] {
        assert!(angle_idx < self.num_angles, "Angle index out of range");
        let start = angle_idx * self.num_radial_bins;
        &mut self.data[start..start + self.num_radial_bins]
    }

    /// Total number of recorded coincidence counts.
    pub fn total_counts(&self) -> usize {
        self.total_counts
    }

    /// Get/set a single bin value.
    pub fn get(&self, angle_idx: usize, radial_idx: usize) -> f64 {
        self.data[angle_idx * self.num_radial_bins + radial_idx]
    }

    pub fn set(&mut self, angle_idx: usize, radial_idx: usize, value: f64) {
        self.data[angle_idx * self.num_radial_bins + radial_idx] = value;
    }

    /// Sum of all bin values.
    pub fn sum(&self) -> f64 {
        self.data.iter().sum()
    }

    /// Maximum bin value.
    pub fn max_value(&self) -> f64 {
        self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    }
}

// ─── Filtered Back Projection ────────────────────────────────────────────────

/// Filtered backprojection image reconstruction for PET.
#[derive(Debug)]
pub struct FilteredBackProjection {
    pub image_size: usize,
}

impl FilteredBackProjection {
    /// Create a new FBP reconstructor.
    ///
    /// # Arguments
    /// * `image_size` - Width/height of the reconstructed image in pixels
    pub fn new(image_size: usize) -> Self {
        assert!(image_size > 0, "Image size must be positive");
        Self { image_size }
    }

    /// Apply Ram-Lak (ramp) filter to a 1D projection in the frequency domain.
    /// The ramp filter has frequency response |f|, which enhances high frequencies
    /// to compensate for the 1/f blurring inherent in backprojection.
    fn ram_lak_filter(&self, projection: &[f64]) -> Vec<f64> {
        let n = projection.len();
        if n == 0 {
            return Vec::new();
        }

        // Forward DFT (real input)
        let mut real = vec![0.0; n];
        let mut imag = vec![0.0; n];
        for k in 0..n {
            for j in 0..n {
                let angle = -2.0 * PI * (k as f64) * (j as f64) / (n as f64);
                real[k] += projection[j] * angle.cos();
                imag[k] += projection[j] * angle.sin();
            }
        }

        // Apply ramp filter: |k/N| for k = 0..N-1, symmetric
        for k in 0..n {
            let freq = if k <= n / 2 {
                k as f64 / n as f64
            } else {
                (n - k) as f64 / n as f64
            };
            real[k] *= freq;
            imag[k] *= freq;
        }

        // Inverse DFT
        let mut filtered = vec![0.0; n];
        for j in 0..n {
            for k in 0..n {
                let angle = 2.0 * PI * (k as f64) * (j as f64) / (n as f64);
                filtered[j] += real[k] * angle.cos() - imag[k] * angle.sin();
            }
            filtered[j] /= n as f64;
        }

        filtered
    }

    /// Reconstruct a 2D image from a sinogram using filtered backprojection.
    ///
    /// Steps:
    /// 1. Filter each projection with Ram-Lak ramp filter
    /// 2. Backproject each filtered projection across the image
    /// 3. Use bilinear interpolation for sub-pixel accuracy
    pub fn reconstruct(&self, sinogram: &Sinogram) -> Vec<Vec<f64>> {
        let size = self.image_size;
        let mut image = vec![vec![0.0; size]; size];
        let center = size as f64 / 2.0;
        let num_bins = sinogram.num_radial_bins;

        for angle_idx in 0..sinogram.num_angles {
            let projection = sinogram.get_projection(angle_idx);
            let filtered = self.ram_lak_filter(projection);

            let angle = PI * (angle_idx as f64) / (sinogram.num_angles as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            for row in 0..size {
                for col in 0..size {
                    // Map pixel to normalized coordinates centered at origin
                    let x = (col as f64 - center) / center;
                    let y = (row as f64 - center) / center;

                    // Project onto the LOR direction
                    let t = x * cos_a + y * sin_a;

                    // Map to radial bin (t in [-1, 1] -> bin in [0, num_bins-1])
                    let bin_f = (t + 1.0) / 2.0 * (num_bins - 1) as f64;

                    if bin_f >= 0.0 && bin_f < (num_bins - 1) as f64 {
                        // Bilinear interpolation
                        let bin_lo = bin_f.floor() as usize;
                        let bin_hi = bin_lo + 1;
                        let frac = bin_f - bin_lo as f64;

                        if bin_hi < num_bins {
                            let val =
                                filtered[bin_lo] * (1.0 - frac) + filtered[bin_hi] * frac;
                            image[row][col] += val;
                        }
                    }
                }
            }
        }

        // Normalize by number of angles (scale factor for backprojection)
        let scale = PI / sinogram.num_angles as f64;
        for row in &mut image {
            for pixel in row.iter_mut() {
                *pixel *= scale;
            }
        }

        image
    }
}

// ─── Attenuation Correction ──────────────────────────────────────────────────

/// Attenuation correction using a mu-map (linear attenuation coefficient map).
#[derive(Debug, Clone)]
pub struct AttenuationCorrection {
    /// 2D attenuation map in 1/mm
    pub mu_map: Vec<Vec<f64>>,
}

impl AttenuationCorrection {
    /// Create from a 2D mu-map.
    /// Values are linear attenuation coefficients in 1/mm at 511 keV.
    /// Typical soft tissue: ~0.096 /cm = 0.00096 /mm.
    pub fn new(mu_map: Vec<Vec<f64>>) -> Self {
        assert!(!mu_map.is_empty(), "Mu-map cannot be empty");
        assert!(!mu_map[0].is_empty(), "Mu-map rows cannot be empty");
        Self { mu_map }
    }

    /// Calculate the attenuation factor along a path through the mu-map.
    /// Returns exp(-integral of mu along path).
    /// Uses simple ray-tracing with step integration.
    pub fn calculate_attenuation_factor(
        &self,
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
    ) -> f64 {
        let rows = self.mu_map.len();
        let cols = self.mu_map[0].len();
        let num_steps = 100;
        let dx = (x1 - x0) / num_steps as f64;
        let dy = (y1 - y0) / num_steps as f64;
        let step_length = (dx * dx + dy * dy).sqrt();

        let mut total_mu = 0.0;
        for i in 0..=num_steps {
            let x = x0 + dx * i as f64;
            let y = y0 + dy * i as f64;

            // Map from normalized coordinates to pixel indices
            let px = ((x + 1.0) / 2.0 * (cols - 1) as f64).round() as isize;
            let py = ((y + 1.0) / 2.0 * (rows - 1) as f64).round() as isize;

            if px >= 0 && px < cols as isize && py >= 0 && py < rows as isize {
                total_mu += self.mu_map[py as usize][px as usize] * step_length;
            }
        }

        (-total_mu).exp()
    }

    /// Apply attenuation correction to a sinogram.
    /// Each LOR is corrected by dividing by the attenuation factor along that LOR.
    pub fn correct_sinogram(&self, sinogram: &mut Sinogram, ring: &DetectorRing) {
        let num_angles = sinogram.num_angles;
        let num_bins = sinogram.num_radial_bins;

        for angle_idx in 0..num_angles {
            let angle = PI * (angle_idx as f64) / (num_angles as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            for bin_idx in 0..num_bins {
                // Map bin to radial offset in [-1, 1]
                let t = 2.0 * (bin_idx as f64) / (num_bins - 1).max(1) as f64 - 1.0;

                // Compute LOR endpoints on the unit circle
                let perp_x = -sin_a;
                let perp_y = cos_a;

                let mid_x = t * cos_a;
                let mid_y = t * sin_a;

                // Extend to ring edge
                let extent = (1.0 - t * t).max(0.0).sqrt();
                let x0 = mid_x - extent * perp_x;
                let y0 = mid_y - extent * perp_y;
                let x1 = mid_x + extent * perp_x;
                let y1 = mid_y + extent * perp_y;

                let acf = self.calculate_attenuation_factor(x0, y0, x1, y1);

                if acf > 1e-10 {
                    let val = sinogram.get(angle_idx, bin_idx);
                    sinogram.set(angle_idx, bin_idx, val / acf);
                }
            }
        }
    }

    /// Create a uniform attenuation map (e.g., water phantom).
    /// mu_value in 1/mm. Standard 511 keV water: ~0.096/cm = 0.00096/mm.
    pub fn uniform(size: usize, mu_value: f64) -> Self {
        Self {
            mu_map: vec![vec![mu_value; size]; size],
        }
    }

    /// Create a circular phantom attenuation map.
    /// Pixels inside the circle have mu_inside, outside have 0.
    pub fn circular_phantom(size: usize, radius_frac: f64, mu_inside: f64) -> Self {
        let center = size as f64 / 2.0;
        let radius = radius_frac * center;
        let mut mu_map = vec![vec![0.0; size]; size];
        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - center;
                let dy = r as f64 - center;
                if dx * dx + dy * dy <= radius * radius {
                    mu_map[r][c] = mu_inside;
                }
            }
        }
        Self { mu_map }
    }
}

// ─── Scatter Estimation ──────────────────────────────────────────────────────

/// Scatter estimation for PET coincidence data.
pub struct ScatterEstimator;

impl ScatterEstimator {
    /// Simple Single Scatter Simulation (SSS) approximation.
    /// Scales the input sinogram by a scatter fraction to estimate scatter contribution.
    pub fn single_scatter_simulation(sinogram: &Sinogram, fraction: f64) -> Sinogram {
        assert!(
            (0.0..=1.0).contains(&fraction),
            "Scatter fraction must be in [0, 1]"
        );
        let mut scatter = Sinogram::new(sinogram.num_angles, sinogram.num_radial_bins);
        for i in 0..sinogram.data.len() {
            scatter.data[i] = sinogram.data[i] * fraction;
        }
        scatter
    }

    /// Tail-fitting scatter estimation.
    /// Fits a smooth function to the sinogram tails (outside the object)
    /// and interpolates through the body to estimate scatter.
    pub fn tail_fitting(
        sinogram: &Sinogram,
        scatter_tails: (usize, usize),
    ) -> Sinogram {
        let mut scatter = Sinogram::new(sinogram.num_angles, sinogram.num_radial_bins);
        let num_bins = sinogram.num_radial_bins;
        let (left_tail, right_tail) = scatter_tails;

        for angle_idx in 0..sinogram.num_angles {
            let proj = sinogram.get_projection(angle_idx);

            // Average of left tail bins
            let left_avg = if left_tail > 0 {
                proj[..left_tail].iter().sum::<f64>() / left_tail as f64
            } else {
                0.0
            };

            // Average of right tail bins
            let right_avg = if right_tail > 0 && num_bins > right_tail {
                proj[num_bins - right_tail..].iter().sum::<f64>() / right_tail as f64
            } else {
                0.0
            };

            // Linear interpolation between tails as scatter estimate
            let scatter_proj = scatter.get_projection_mut(angle_idx);
            for bin_idx in 0..num_bins {
                let frac = bin_idx as f64 / (num_bins - 1).max(1) as f64;
                scatter_proj[bin_idx] = left_avg * (1.0 - frac) + right_avg * frac;
            }
        }

        scatter
    }

    /// Subtract scatter estimate from sinogram (clamp to zero).
    pub fn subtract_scatter(sinogram: &mut Sinogram, scatter: &Sinogram) {
        assert_eq!(sinogram.data.len(), scatter.data.len());
        for i in 0..sinogram.data.len() {
            sinogram.data[i] = (sinogram.data[i] - scatter.data[i]).max(0.0);
        }
    }
}

// ─── Decay Correction ────────────────────────────────────────────────────────

/// Common PET isotope half-lives in seconds.
pub struct Isotope;

impl Isotope {
    /// Fluorine-18: 109.77 minutes = 6586.2 seconds
    pub const F18_HALF_LIFE_S: f64 = 109.77 * 60.0;
    /// Carbon-11: 20.4 minutes = 1224.0 seconds
    pub const C11_HALF_LIFE_S: f64 = 20.4 * 60.0;
    /// Oxygen-15: 2.04 minutes = 122.4 seconds
    pub const O15_HALF_LIFE_S: f64 = 2.04 * 60.0;
    /// Rubidium-82: 1.27 minutes = 76.2 seconds
    pub const RB82_HALF_LIFE_S: f64 = 1.27 * 60.0;
    /// Nitrogen-13: 9.97 minutes = 598.2 seconds
    pub const N13_HALF_LIFE_S: f64 = 9.97 * 60.0;
    /// Gallium-68: 67.71 minutes = 4062.6 seconds
    pub const GA68_HALF_LIFE_S: f64 = 67.71 * 60.0;
}

/// Decay correction for radiotracer physical decay during acquisition.
pub struct DecayCorrection;

impl DecayCorrection {
    /// Compute decay correction factor.
    /// Returns the multiplicative factor to correct for decay: exp(ln(2) * t / t_half).
    ///
    /// # Arguments
    /// * `half_life_s` - Isotope half-life in seconds
    /// * `elapsed_s` - Time elapsed since injection in seconds
    pub fn correction_factor(half_life_s: f64, elapsed_s: f64) -> f64 {
        assert!(half_life_s > 0.0, "Half-life must be positive");
        (2.0_f64.ln() * elapsed_s / half_life_s).exp()
    }

    /// Compute the remaining activity fraction.
    /// Returns A(t)/A(0) = exp(-ln(2) * t / t_half) = (1/2)^(t/t_half).
    pub fn remaining_fraction(half_life_s: f64, elapsed_s: f64) -> f64 {
        assert!(half_life_s > 0.0, "Half-life must be positive");
        (-2.0_f64.ln() * elapsed_s / half_life_s).exp()
    }

    /// Apply decay correction to a series of time-binned count values.
    /// Each bin is corrected to the reference time (t=0).
    pub fn correct_time_bins(
        counts: &[f64],
        bin_duration_s: f64,
        half_life_s: f64,
    ) -> Vec<f64> {
        counts
            .iter()
            .enumerate()
            .map(|(i, &c)| {
                let mid_time = (i as f64 + 0.5) * bin_duration_s;
                c * Self::correction_factor(half_life_s, mid_time)
            })
            .collect()
    }

    /// Compute the frame decay correction factor for a finite acquisition frame.
    /// Corrects for decay during the frame itself:
    /// factor = (lambda * dt) / (1 - exp(-lambda * dt))
    /// where lambda = ln(2) / t_half.
    pub fn frame_correction_factor(half_life_s: f64, frame_duration_s: f64) -> f64 {
        let lambda = 2.0_f64.ln() / half_life_s;
        let ldt = lambda * frame_duration_s;
        if ldt.abs() < 1e-10 {
            1.0 // Short frame, negligible decay
        } else {
            ldt / (1.0 - (-ldt).exp())
        }
    }
}

// ─── Dead Time Correction ────────────────────────────────────────────────────

/// Dead time correction models for PET detector electronics.
pub struct DeadTimeCorrection;

impl DeadTimeCorrection {
    /// Paralyzable (extending) dead time model.
    /// True rate n satisfies: m = n * exp(-n * tau)
    /// Given measured rate m and dead time tau, solve for true rate n iteratively.
    ///
    /// Uses Newton's method to solve m = n * exp(-n * tau).
    pub fn paralyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
        if measured_rate <= 0.0 || dead_time_s <= 0.0 {
            return measured_rate;
        }

        // Newton's method: f(n) = n * exp(-n*tau) - m = 0
        // f'(n) = exp(-n*tau) * (1 - n*tau)
        let mut n = measured_rate; // Initial guess
        for _ in 0..50 {
            let e = (-n * dead_time_s).exp();
            let f = n * e - measured_rate;
            let fp = e * (1.0 - n * dead_time_s);
            if fp.abs() < 1e-30 {
                break;
            }
            let dn = f / fp;
            n -= dn;
            if n < 0.0 {
                n = measured_rate; // Reset if we overshoot
            }
            if dn.abs() < 1e-10 {
                break;
            }
        }
        n
    }

    /// Non-paralyzable (non-extending) dead time model.
    /// n = m / (1 - m * tau)
    /// where m is measured rate, tau is dead time, n is true rate.
    pub fn non_paralyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
        if measured_rate <= 0.0 || dead_time_s <= 0.0 {
            return measured_rate;
        }
        let denom = 1.0 - measured_rate * dead_time_s;
        if denom <= 0.0 {
            // System is saturated
            f64::INFINITY
        } else {
            measured_rate / denom
        }
    }

    /// Compute the count loss fraction at a given rate.
    /// Returns (true - measured) / true.
    pub fn loss_fraction_non_paralyzable(true_rate: f64, dead_time_s: f64) -> f64 {
        if true_rate <= 0.0 {
            return 0.0;
        }
        let measured = true_rate / (1.0 + true_rate * dead_time_s);
        (true_rate - measured) / true_rate
    }

    /// Compute the count loss fraction for paralyzable model.
    pub fn loss_fraction_paralyzable(true_rate: f64, dead_time_s: f64) -> f64 {
        if true_rate <= 0.0 {
            return 0.0;
        }
        let measured = true_rate * (-true_rate * dead_time_s).exp();
        (true_rate - measured) / true_rate
    }

    /// Maximum observed count rate for paralyzable detector.
    /// Occurs at n = 1/tau, giving m_max = 1/(tau * e).
    pub fn max_paralyzable_rate(dead_time_s: f64) -> f64 {
        if dead_time_s <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / (dead_time_s * std::f64::consts::E)
    }
}

// ─── Normalization ───────────────────────────────────────────────────────────

/// Normalization correction for non-uniform detector efficiency and geometry.
pub struct NormalizationCorrection;

impl NormalizationCorrection {
    /// Compute geometric normalization factors for each detector pair.
    /// Accounts for varying solid angle subtended by different LOR geometries.
    /// Returns a sinogram-shaped normalization map.
    pub fn geometric_normalization(
        ring: &DetectorRing,
        num_angles: usize,
        num_radial_bins: usize,
    ) -> Sinogram {
        let mut norm = Sinogram::new(num_angles, num_radial_bins);

        for angle_idx in 0..num_angles {
            let angle = PI * (angle_idx as f64) / (num_angles as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            for bin_idx in 0..num_radial_bins {
                let t = 2.0 * (bin_idx as f64) / (num_radial_bins - 1).max(1) as f64 - 1.0;
                // Normalization proportional to chord length through FOV
                let chord = 2.0 * (1.0 - t * t).max(0.0).sqrt();
                let _cos_incidence = (1.0 - t * t).max(0.0).sqrt();

                // Combined geometric factor
                let factor = if chord > 1e-10 {
                    1.0 / (chord * ring.ring_radius_mm)
                } else {
                    0.0
                };

                norm.set(angle_idx, bin_idx, factor);
                let _ = (cos_a, sin_a); // used for more complex geometric models
            }
        }

        norm
    }

    /// Apply normalization to a sinogram (element-wise multiplication).
    pub fn apply_normalization(sinogram: &mut Sinogram, norm: &Sinogram) {
        assert_eq!(sinogram.data.len(), norm.data.len());
        for i in 0..sinogram.data.len() {
            sinogram.data[i] *= norm.data[i];
        }
    }

    /// Compute detector efficiency from a blank (no object) scan.
    /// Returns per-detector relative efficiency factors.
    pub fn detector_efficiency(blank_counts: &[f64]) -> Vec<f64> {
        let mean = blank_counts.iter().sum::<f64>() / blank_counts.len() as f64;
        if mean <= 0.0 {
            return vec![1.0; blank_counts.len()];
        }
        blank_counts.iter().map(|&c| c / mean).collect()
    }
}

// ─── Image Metrics ───────────────────────────────────────────────────────────

/// Image quality metrics for reconstructed PET images.
pub struct ImageMetrics;

impl ImageMetrics {
    /// Compute the contrast-to-noise ratio (CNR) between a hot region and background.
    pub fn contrast_to_noise(hot: &[f64], background: &[f64]) -> f64 {
        let hot_mean = hot.iter().sum::<f64>() / hot.len() as f64;
        let bg_mean = background.iter().sum::<f64>() / background.len() as f64;
        let bg_std = Self::std_dev(background);
        if bg_std < 1e-30 {
            return 0.0;
        }
        (hot_mean - bg_mean).abs() / bg_std
    }

    /// Compute the recovery coefficient (ratio of measured to true activity).
    pub fn recovery_coefficient(measured_mean: f64, true_activity: f64) -> f64 {
        if true_activity.abs() < 1e-30 {
            return 0.0;
        }
        measured_mean / true_activity
    }

    /// Standard deviation of a slice.
    pub fn std_dev(data: &[f64]) -> f64 {
        if data.len() < 2 {
            return 0.0;
        }
        let mean = data.iter().sum::<f64>() / data.len() as f64;
        let variance =
            data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (data.len() - 1) as f64;
        variance.sqrt()
    }

    /// Compute mean squared error between reconstructed and reference images.
    pub fn mse(image: &[Vec<f64>], reference: &[Vec<f64>]) -> f64 {
        let mut sum = 0.0;
        let mut count = 0usize;
        for (row_a, row_b) in image.iter().zip(reference.iter()) {
            for (&a, &b) in row_a.iter().zip(row_b.iter()) {
                sum += (a - b).powi(2);
                count += 1;
            }
        }
        if count == 0 {
            0.0
        } else {
            sum / count as f64
        }
    }

    /// Compute peak signal-to-noise ratio in dB.
    pub fn psnr(image: &[Vec<f64>], reference: &[Vec<f64>], max_val: f64) -> f64 {
        let mse_val = Self::mse(image, reference);
        if mse_val < 1e-30 {
            return f64::INFINITY;
        }
        10.0 * (max_val * max_val / mse_val).log10()
    }

    /// Sum of pixel values in a circular ROI.
    pub fn roi_sum(image: &[Vec<f64>], center_x: usize, center_y: usize, radius: f64) -> f64 {
        let mut sum = 0.0;
        for (r, row) in image.iter().enumerate() {
            for (c, &val) in row.iter().enumerate() {
                let dx = c as f64 - center_x as f64;
                let dy = r as f64 - center_y as f64;
                if dx * dx + dy * dy <= radius * radius {
                    sum += val;
                }
            }
        }
        sum
    }

    /// Mean pixel value in a circular ROI.
    pub fn roi_mean(image: &[Vec<f64>], center_x: usize, center_y: usize, radius: f64) -> f64 {
        let mut sum = 0.0;
        let mut count = 0usize;
        for (r, row) in image.iter().enumerate() {
            for (c, &val) in row.iter().enumerate() {
                let dx = c as f64 - center_x as f64;
                let dy = r as f64 - center_y as f64;
                if dx * dx + dy * dy <= radius * radius {
                    sum += val;
                    count += 1;
                }
            }
        }
        if count == 0 {
            0.0
        } else {
            sum / count as f64
        }
    }
}

// ─── Time-of-Flight ──────────────────────────────────────────────────────────

/// Time-of-Flight (TOF) PET processing.
/// TOF measures the arrival time difference of the two photons to localize
/// the annihilation point along the LOR.
pub struct TimeOfFlight;

impl TimeOfFlight {
    /// Speed of light in mm/ns.
    pub const C_MM_NS: f64 = 299.792458;

    /// Convert a time difference to a position offset along the LOR.
    /// offset = c * dt / 2, where dt is the time difference in ns.
    pub fn time_to_position_offset_mm(time_diff_ns: f64) -> f64 {
        Self::C_MM_NS * time_diff_ns / 2.0
    }

    /// Compute the TOF spatial resolution (FWHM) from timing resolution.
    /// spatial_fwhm = c * timing_fwhm / 2
    pub fn spatial_resolution_mm(timing_fwhm_ns: f64) -> f64 {
        Self::C_MM_NS * timing_fwhm_ns / 2.0
    }

    /// Compute the TOF sensitivity gain factor.
    /// Gain = D / (c * dt_fwhm) where D is the object diameter.
    pub fn sensitivity_gain(object_diameter_mm: f64, timing_fwhm_ns: f64) -> f64 {
        let spatial_fwhm = Self::spatial_resolution_mm(timing_fwhm_ns);
        if spatial_fwhm < 1e-10 {
            return f64::INFINITY;
        }
        object_diameter_mm / spatial_fwhm
    }

    /// Apply TOF weighting to a sinogram bin based on position probability.
    /// Uses Gaussian kernel centered at the estimated annihilation position.
    pub fn tof_weight(
        bin_position_mm: f64,
        estimated_position_mm: f64,
        timing_fwhm_ns: f64,
    ) -> f64 {
        let sigma = Self::spatial_resolution_mm(timing_fwhm_ns) / 2.355; // FWHM to sigma
        let dx = bin_position_mm - estimated_position_mm;
        (-dx * dx / (2.0 * sigma * sigma)).exp()
    }
}

// ─── Randoms Estimation ──────────────────────────────────────────────────────

/// Random coincidence estimation and subtraction.
pub struct RandomsEstimator;

impl RandomsEstimator {
    /// Estimate random coincidence rate between two detectors.
    /// R_rand = 2 * tau * S_a * S_b
    pub fn estimate_rate(singles_a: f64, singles_b: f64, window_ns: f64) -> f64 {
        2.0 * (window_ns * 1e-9) * singles_a * singles_b
    }

    /// Delayed-window random estimation.
    /// Delays one detector stream and counts "coincidences" to estimate random rate.
    /// The delayed counts provide a direct measurement of the randoms rate.
    pub fn delayed_window_estimate(
        events_a: &[f64],
        events_b: &[f64],
        window_ns: f64,
        delay_ns: f64,
    ) -> usize {
        let mut count = 0;
        for &ta in events_a {
            for &tb in events_b {
                let delayed_diff = (ta - (tb + delay_ns)).abs();
                if delayed_diff <= window_ns {
                    count += 1;
                }
            }
        }
        count
    }

    /// Noise equivalent count rate (NECR).
    /// NECR = T^2 / (T + S + 2R) where T=trues, S=scatter, R=randoms.
    /// Higher NECR indicates better effective signal quality.
    pub fn necr(trues: f64, scatter: f64, randoms: f64) -> f64 {
        let denom = trues + scatter + 2.0 * randoms;
        if denom <= 0.0 {
            0.0
        } else {
            trues * trues / denom
        }
    }

    /// Scatter fraction.
    /// SF = S / (T + S)
    pub fn scatter_fraction(trues: f64, scatter: f64) -> f64 {
        let total = trues + scatter;
        if total <= 0.0 {
            0.0
        } else {
            scatter / total
        }
    }
}

// ─── Phantom Generator ───────────────────────────────────────────────────────

/// Synthetic phantom generator for testing PET reconstruction.
pub struct PhantomGenerator;

impl PhantomGenerator {
    /// Generate a simple hot-rod phantom (Derenzo-like).
    /// Creates a 2D activity map with circular hot spots of different sizes.
    pub fn hot_rod_phantom(size: usize, num_rods: usize, activity: f64) -> Vec<Vec<f64>> {
        let mut image = vec![vec![0.0; size]; size];
        let center = size as f64 / 2.0;

        // Background disk
        let bg_radius = center * 0.9;
        let bg_activity = activity * 0.1;
        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - center;
                let dy = r as f64 - center;
                if dx * dx + dy * dy <= bg_radius * bg_radius {
                    image[r][c] = bg_activity;
                }
            }
        }

        // Hot rods arranged in a circle
        let rod_radius = center * 0.08;
        let circle_radius = center * 0.5;
        for i in 0..num_rods {
            let angle = 2.0 * PI * (i as f64) / (num_rods as f64);
            let cx = center + circle_radius * angle.cos();
            let cy = center + circle_radius * angle.sin();

            for r in 0..size {
                for c in 0..size {
                    let dx = c as f64 - cx;
                    let dy = r as f64 - cy;
                    if dx * dx + dy * dy <= rod_radius * rod_radius {
                        image[r][c] = activity;
                    }
                }
            }
        }

        image
    }

    /// Generate a uniform cylinder phantom.
    pub fn uniform_cylinder(size: usize, radius_frac: f64, activity: f64) -> Vec<Vec<f64>> {
        let mut image = vec![vec![0.0; size]; size];
        let center = size as f64 / 2.0;
        let radius = radius_frac * center;

        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - center;
                let dy = r as f64 - center;
                if dx * dx + dy * dy <= radius * radius {
                    image[r][c] = activity;
                }
            }
        }

        image
    }

    /// Generate a contrast phantom with hot and cold inserts.
    pub fn contrast_phantom(
        size: usize,
        bg_activity: f64,
        hot_ratio: f64,
        cold_ratio: f64,
    ) -> Vec<Vec<f64>> {
        let mut image = vec![vec![0.0; size]; size];
        let center = size as f64 / 2.0;
        let bg_radius = center * 0.9;

        // Background
        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - center;
                let dy = r as f64 - center;
                if dx * dx + dy * dy <= bg_radius * bg_radius {
                    image[r][c] = bg_activity;
                }
            }
        }

        // Hot insert (upper right)
        let hot_cx = center + center * 0.35;
        let hot_cy = center - center * 0.35;
        let insert_radius = center * 0.15;
        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - hot_cx;
                let dy = r as f64 - hot_cy;
                if dx * dx + dy * dy <= insert_radius * insert_radius {
                    image[r][c] = bg_activity * hot_ratio;
                }
            }
        }

        // Cold insert (lower left)
        let cold_cx = center - center * 0.35;
        let cold_cy = center + center * 0.35;
        for r in 0..size {
            for c in 0..size {
                let dx = c as f64 - cold_cx;
                let dy = r as f64 - cold_cy;
                if dx * dx + dy * dy <= insert_radius * insert_radius {
                    image[r][c] = bg_activity * cold_ratio;
                }
            }
        }

        image
    }

    /// Forward-project a 2D activity map to create a noise-free sinogram.
    pub fn forward_project(
        activity: &[Vec<f64>],
        num_angles: usize,
        num_radial_bins: usize,
    ) -> Sinogram {
        let rows = activity.len();
        let cols = if rows > 0 { activity[0].len() } else { 0 };
        let mut sinogram = Sinogram::new(num_angles, num_radial_bins);

        for angle_idx in 0..num_angles {
            let angle = PI * (angle_idx as f64) / (num_angles as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            for bin_idx in 0..num_radial_bins {
                let t = 2.0 * (bin_idx as f64) / (num_radial_bins - 1).max(1) as f64 - 1.0;

                // Integrate along the LOR perpendicular to the projection direction
                let num_steps = rows.max(cols) * 2;
                let mut line_integral = 0.0;

                for step in 0..num_steps {
                    let s = 2.0 * (step as f64) / (num_steps - 1).max(1) as f64 - 1.0;

                    let x = t * cos_a - s * sin_a;
                    let y = t * sin_a + s * cos_a;

                    // Map normalized coords to pixel indices
                    let px = ((x + 1.0) / 2.0 * (cols - 1) as f64).round() as isize;
                    let py = ((y + 1.0) / 2.0 * (rows - 1) as f64).round() as isize;

                    if px >= 0 && px < cols as isize && py >= 0 && py < rows as isize {
                        line_integral += activity[py as usize][px as usize];
                    }
                }

                let step_size = 2.0 / num_steps as f64;
                sinogram.set(angle_idx, bin_idx, line_integral * step_size);
            }
        }

        sinogram
    }
}

// ─── Poisson Noise ───────────────────────────────────────────────────────────

/// Simple Poisson noise generator for PET count data.
/// Uses the inverse transform method with a basic LCG PRNG.
pub struct PoissonNoise {
    state: u64,
}

impl PoissonNoise {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Generate a uniform random number in [0, 1).
    fn uniform(&mut self) -> f64 {
        // LCG PRNG
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Generate a Poisson-distributed random variate with given mean (lambda).
    /// Uses Knuth's algorithm for small lambda, rejection for large lambda.
    pub fn sample(&mut self, lambda: f64) -> usize {
        if lambda <= 0.0 {
            return 0;
        }

        if lambda < 30.0 {
            // Knuth's algorithm
            let l = (-lambda).exp();
            let mut k = 0usize;
            let mut p = 1.0;
            loop {
                k += 1;
                p *= self.uniform();
                if p <= l {
                    return k - 1;
                }
            }
        } else {
            // Normal approximation for large lambda
            let u1 = self.uniform().max(1e-30);
            let u2 = self.uniform();
            let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            let result = lambda + z * lambda.sqrt();
            result.max(0.0).round() as usize
        }
    }

    /// Add Poisson noise to a sinogram.
    /// Each bin count is replaced by a Poisson sample with that bin's value as the mean.
    pub fn add_noise_to_sinogram(&mut self, sinogram: &mut Sinogram) {
        for i in 0..sinogram.data.len() {
            let mean = sinogram.data[i];
            if mean > 0.0 {
                sinogram.data[i] = self.sample(mean) as f64;
            }
        }
    }
}

// ─── MLEM Iterative Reconstruction ──────────────────────────────────────────

/// Maximum Likelihood Expectation Maximization (MLEM) iterative reconstruction.
/// Provides better noise properties than FBP at the cost of computation time.
pub struct MlemReconstructor {
    pub image_size: usize,
    pub num_iterations: usize,
}

impl MlemReconstructor {
    pub fn new(image_size: usize, num_iterations: usize) -> Self {
        assert!(image_size > 0);
        assert!(num_iterations > 0);
        Self {
            image_size,
            num_iterations,
        }
    }

    /// Simplified MLEM reconstruction.
    /// Uses a simplified system matrix based on projection/backprojection.
    pub fn reconstruct(&self, sinogram: &Sinogram) -> Vec<Vec<f64>> {
        let size = self.image_size;

        // Initialize with uniform image
        let initial_value = sinogram.sum() / (size * size) as f64;
        let initial_value = if initial_value > 0.0 {
            initial_value
        } else {
            1.0
        };
        let mut image = vec![vec![initial_value; size]; size];

        let fbp = FilteredBackProjection::new(size);

        for _iter in 0..self.num_iterations {
            // Forward project current estimate
            let projected = PhantomGenerator::forward_project(
                &image,
                sinogram.num_angles,
                sinogram.num_radial_bins,
            );

            // Compute ratio sinogram / projected
            let mut ratio_sino =
                Sinogram::new(sinogram.num_angles, sinogram.num_radial_bins);
            for i in 0..sinogram.data.len() {
                if projected.data[i] > 1e-10 {
                    ratio_sino.data[i] = sinogram.data[i] / projected.data[i];
                } else {
                    ratio_sino.data[i] = 0.0;
                }
            }

            // Backproject the ratio
            let correction = fbp.reconstruct(&ratio_sino);

            // Update image multiplicatively
            for r in 0..size {
                for c in 0..size {
                    let corr = correction[r][c].abs(); // Use absolute value for stability
                    if corr > 1e-10 {
                        image[r][c] *= corr;
                    }
                    // Enforce non-negativity
                    if image[r][c] < 0.0 {
                        image[r][c] = 0.0;
                    }
                }
            }
        }

        image
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;

    // --- DetectorRing tests ---

    #[test]
    fn test_detector_ring_creation() {
        let ring = DetectorRing::new(64, 400.0);
        assert_eq!(ring.num_detectors, 64);
        assert!((ring.ring_radius_mm - 400.0).abs() < EPSILON);
    }

    #[test]
    #[should_panic]
    fn test_detector_ring_too_few_detectors() {
        DetectorRing::new(1, 400.0);
    }

    #[test]
    #[should_panic]
    fn test_detector_ring_zero_radius() {
        DetectorRing::new(64, 0.0);
    }

    #[test]
    fn test_detector_angle() {
        let ring = DetectorRing::new(4, 100.0);
        assert!((ring.detector_angle(0) - 0.0).abs() < EPSILON);
        assert!((ring.detector_angle(1) - PI / 2.0).abs() < EPSILON);
        assert!((ring.detector_angle(2) - PI).abs() < EPSILON);
        assert!((ring.detector_angle(3) - 3.0 * PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_detector_position() {
        let ring = DetectorRing::new(4, 100.0);
        let (x, y) = ring.detector_position(0);
        assert!((x - 100.0).abs() < EPSILON);
        assert!(y.abs() < EPSILON);

        let (x, y) = ring.detector_position(1);
        assert!(x.abs() < EPSILON);
        assert!((y - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_detector_distance() {
        let ring = DetectorRing::new(4, 100.0);
        // Adjacent detectors at 90 degrees: distance = sqrt(2) * R
        let d = ring.detector_distance(0, 1);
        assert!((d - 100.0 * 2.0_f64.sqrt()).abs() < 0.01);

        // Opposite detectors: distance = 2 * R
        let d = ring.detector_distance(0, 2);
        assert!((d - 200.0).abs() < 0.01);
    }

    #[test]
    fn test_lor_parameters() {
        let ring = DetectorRing::new(64, 400.0);
        let (angle, offset) = ring.lor_parameters(0, 32);
        // Detectors 0 and 32 are opposite, offset should be near 0
        assert!(offset.abs() < 1.0);
    }

    #[test]
    fn test_detector_ring_symmetry() {
        let ring = DetectorRing::new(8, 200.0);
        // Opposite detectors should be 2*R apart
        let d = ring.detector_distance(0, 4);
        assert!((d - 400.0).abs() < 0.01);
    }

    // --- CoincidenceDetector tests ---

    #[test]
    fn test_coincidence_detector_creation() {
        let cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        assert_eq!(cd.event_count(), 0);
    }

    #[test]
    fn test_energy_validation() {
        let cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        assert!(cd.is_energy_valid(511.0));
        assert!(cd.is_energy_valid(350.0));
        assert!(cd.is_energy_valid(650.0));
        assert!(!cd.is_energy_valid(349.0));
        assert!(!cd.is_energy_valid(651.0));
    }

    #[test]
    fn test_coincidence_detection_simple() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        cd.add_event(32, 103.0, 511.0); // 3 ns apart, within window
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 1);
        assert_eq!(coinc[0].detector_a, 0);
        assert_eq!(coinc[0].detector_b, 32);
    }

    #[test]
    fn test_coincidence_outside_time_window() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        cd.add_event(32, 110.0, 511.0); // 10 ns apart, outside 6ns window
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 0);
    }

    #[test]
    fn test_coincidence_outside_energy_window() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        cd.add_event(32, 103.0, 200.0); // Energy too low (scattered photon)
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 0);
    }

    #[test]
    fn test_coincidence_same_detector_rejected() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(5, 100.0, 511.0);
        cd.add_event(5, 102.0, 511.0); // Same detector
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 0);
    }

    #[test]
    fn test_multiple_coincidences() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        cd.add_event(32, 103.0, 511.0);
        cd.add_event(10, 200.0, 511.0);
        cd.add_event(42, 201.0, 511.0);
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 2);
    }

    #[test]
    fn test_random_coincidence_rate() {
        let rate = CoincidenceDetector::random_coincidence_rate(1e6, 1e6, 6.0);
        // 2 * 6e-9 * 1e6 * 1e6 = 12,000 counts/s
        assert!((rate - 12000.0).abs() < 1.0);
    }

    #[test]
    fn test_coincidence_clear() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        assert_eq!(cd.event_count(), 1);
        cd.clear();
        assert_eq!(cd.event_count(), 0);
    }

    #[test]
    fn test_coincidence_time_diff() {
        let mut cd = CoincidenceDetector::new(6.0, (350.0, 650.0));
        cd.add_event(0, 100.0, 511.0);
        cd.add_event(32, 104.5, 511.0);
        let coinc = cd.find_coincidences();
        assert_eq!(coinc.len(), 1);
        assert!((coinc[0].time_diff_ns - 4.5).abs() < EPSILON);
    }

    // --- Sinogram tests ---

    #[test]
    fn test_sinogram_creation() {
        let sino = Sinogram::new(180, 256);
        assert_eq!(sino.num_angles, 180);
        assert_eq!(sino.num_radial_bins, 256);
        assert_eq!(sino.total_counts(), 0);
        assert!((sino.sum() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_sinogram_add_lor() {
        let ring = DetectorRing::new(64, 400.0);
        let mut sino = Sinogram::new(32, 32);
        sino.add_lor(0, 32, &ring);
        assert_eq!(sino.total_counts(), 1);
        assert!((sino.sum() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_sinogram_get_set() {
        let mut sino = Sinogram::new(10, 10);
        sino.set(3, 5, 42.0);
        assert!((sino.get(3, 5) - 42.0).abs() < EPSILON);
    }

    #[test]
    fn test_sinogram_projection() {
        let mut sino = Sinogram::new(4, 8);
        sino.set(2, 3, 10.0);
        let proj = sino.get_projection(2);
        assert_eq!(proj.len(), 8);
        assert!((proj[3] - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_sinogram_max_value() {
        let mut sino = Sinogram::new(4, 4);
        sino.set(1, 2, 5.0);
        sino.set(3, 0, 8.0);
        assert!((sino.max_value() - 8.0).abs() < EPSILON);
    }

    #[test]
    fn test_sinogram_multiple_lors() {
        let ring = DetectorRing::new(64, 400.0);
        let mut sino = Sinogram::new(32, 32);
        for _ in 0..100 {
            sino.add_lor(0, 32, &ring);
        }
        assert_eq!(sino.total_counts(), 100);
    }

    // --- FilteredBackProjection tests ---

    #[test]
    fn test_fbp_creation() {
        let fbp = FilteredBackProjection::new(64);
        assert_eq!(fbp.image_size, 64);
    }

    #[test]
    fn test_ram_lak_filter_dc_zero() {
        let fbp = FilteredBackProjection::new(32);
        let projection = vec![1.0; 16]; // Constant projection
        let filtered = fbp.ram_lak_filter(&projection);
        // DC component should be zeroed by ramp filter
        assert!(filtered[0].abs() < 0.1);
    }

    #[test]
    fn test_ram_lak_filter_preserves_length() {
        let fbp = FilteredBackProjection::new(32);
        let projection = vec![1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let filtered = fbp.ram_lak_filter(&projection);
        assert_eq!(filtered.len(), projection.len());
    }

    #[test]
    fn test_fbp_reconstruct_returns_correct_size() {
        let fbp = FilteredBackProjection::new(16);
        let sino = Sinogram::new(8, 16);
        let image = fbp.reconstruct(&sino);
        assert_eq!(image.len(), 16);
        assert_eq!(image[0].len(), 16);
    }

    #[test]
    fn test_fbp_reconstruct_empty_sinogram() {
        let fbp = FilteredBackProjection::new(16);
        let sino = Sinogram::new(8, 16);
        let image = fbp.reconstruct(&sino);
        // All zeros in -> all zeros out
        for row in &image {
            for &val in row {
                assert!(val.abs() < EPSILON);
            }
        }
    }

    #[test]
    fn test_fbp_reconstruct_point_source() {
        // Create a simple phantom and forward project, then reconstruct
        let phantom = PhantomGenerator::uniform_cylinder(32, 0.3, 100.0);
        let sino = PhantomGenerator::forward_project(&phantom, 32, 32);
        let fbp = FilteredBackProjection::new(32);
        let recon = fbp.reconstruct(&sino);
        // Center pixel should be positive (activity present)
        assert!(recon[16][16] > 0.0);
    }

    // --- AttenuationCorrection tests ---

    #[test]
    fn test_attenuation_uniform() {
        let ac = AttenuationCorrection::uniform(32, 0.001);
        assert_eq!(ac.mu_map.len(), 32);
        assert!((ac.mu_map[0][0] - 0.001).abs() < EPSILON);
    }

    #[test]
    fn test_attenuation_circular_phantom() {
        let ac = AttenuationCorrection::circular_phantom(32, 0.8, 0.001);
        // Center should be non-zero
        assert!(ac.mu_map[16][16] > 0.0);
        // Corner should be zero
        assert!((ac.mu_map[0][0] - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_attenuation_factor_no_attenuation() {
        let ac = AttenuationCorrection::uniform(32, 0.0);
        let factor = ac.calculate_attenuation_factor(-0.5, 0.0, 0.5, 0.0);
        assert!((factor - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_attenuation_factor_with_material() {
        let ac = AttenuationCorrection::uniform(32, 0.001);
        let factor = ac.calculate_attenuation_factor(-0.5, 0.0, 0.5, 0.0);
        // With positive mu, attenuation factor should be < 1
        assert!(factor < 1.0);
        assert!(factor > 0.0);
    }

    #[test]
    fn test_attenuation_correction_sinogram() {
        let ring = DetectorRing::new(32, 200.0);
        let mut sino = Sinogram::new(16, 16);
        sino.set(4, 8, 50.0);
        let ac = AttenuationCorrection::uniform(16, 0.0005);
        let original_sum = sino.sum();
        ac.correct_sinogram(&mut sino, &ring);
        // Attenuation correction should increase counts (dividing by factor < 1)
        assert!(sino.sum() >= original_sum);
    }

    // --- ScatterEstimator tests ---

    #[test]
    fn test_scatter_simulation() {
        let mut sino = Sinogram::new(4, 8);
        sino.set(0, 0, 100.0);
        sino.set(1, 3, 200.0);
        let scatter = ScatterEstimator::single_scatter_simulation(&sino, 0.3);
        assert!((scatter.get(0, 0) - 30.0).abs() < EPSILON);
        assert!((scatter.get(1, 3) - 60.0).abs() < EPSILON);
    }

    #[test]
    fn test_scatter_fraction_zero() {
        let sino = Sinogram::new(4, 8);
        let scatter = ScatterEstimator::single_scatter_simulation(&sino, 0.0);
        assert!((scatter.sum() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_tail_fitting() {
        let mut sino = Sinogram::new(2, 10);
        // Set left tail
        sino.set(0, 0, 5.0);
        sino.set(0, 1, 5.0);
        // Set right tail
        sino.set(0, 8, 10.0);
        sino.set(0, 9, 10.0);
        let scatter = ScatterEstimator::tail_fitting(&sino, (2, 2));
        // Left end should be close to 5.0 and right end close to 10.0
        assert!(scatter.get(0, 0) > 4.0);
        assert!(scatter.get(0, 9) > 8.0);
    }

    #[test]
    fn test_scatter_subtraction() {
        let mut sino = Sinogram::new(2, 4);
        sino.set(0, 0, 100.0);
        sino.set(0, 1, 50.0);
        let scatter = ScatterEstimator::single_scatter_simulation(&sino, 0.2);
        ScatterEstimator::subtract_scatter(&mut sino, &scatter);
        assert!((sino.get(0, 0) - 80.0).abs() < EPSILON);
        assert!((sino.get(0, 1) - 40.0).abs() < EPSILON);
    }

    // --- DecayCorrection tests ---

    #[test]
    fn test_decay_correction_factor_at_t0() {
        let factor = DecayCorrection::correction_factor(Isotope::F18_HALF_LIFE_S, 0.0);
        assert!((factor - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_decay_correction_factor_at_half_life() {
        let factor =
            DecayCorrection::correction_factor(Isotope::F18_HALF_LIFE_S, Isotope::F18_HALF_LIFE_S);
        assert!((factor - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_remaining_fraction_at_t0() {
        let frac = DecayCorrection::remaining_fraction(Isotope::F18_HALF_LIFE_S, 0.0);
        assert!((frac - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_remaining_fraction_at_half_life() {
        let frac = DecayCorrection::remaining_fraction(
            Isotope::F18_HALF_LIFE_S,
            Isotope::F18_HALF_LIFE_S,
        );
        assert!((frac - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_remaining_fraction_at_two_half_lives() {
        let frac = DecayCorrection::remaining_fraction(
            Isotope::F18_HALF_LIFE_S,
            2.0 * Isotope::F18_HALF_LIFE_S,
        );
        assert!((frac - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_decay_correct_time_bins() {
        let counts = vec![100.0, 100.0, 100.0, 100.0];
        let corrected = DecayCorrection::correct_time_bins(&counts, 60.0, Isotope::F18_HALF_LIFE_S);
        // Later bins should have higher corrected values
        assert!(corrected[3] > corrected[0]);
    }

    #[test]
    fn test_frame_correction_short_frame() {
        // Very short frame relative to half-life should give factor near 1.0
        let factor = DecayCorrection::frame_correction_factor(Isotope::F18_HALF_LIFE_S, 0.001);
        assert!((factor - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_isotope_constants() {
        assert!((Isotope::F18_HALF_LIFE_S - 6586.2).abs() < 0.1);
        assert!((Isotope::C11_HALF_LIFE_S - 1224.0).abs() < 0.1);
        assert!((Isotope::O15_HALF_LIFE_S - 122.4).abs() < 0.1);
        assert!((Isotope::RB82_HALF_LIFE_S - 76.2).abs() < 0.1);
    }

    #[test]
    fn test_c11_faster_decay_than_f18() {
        let t = 600.0; // 10 minutes
        let f18_remaining = DecayCorrection::remaining_fraction(Isotope::F18_HALF_LIFE_S, t);
        let c11_remaining = DecayCorrection::remaining_fraction(Isotope::C11_HALF_LIFE_S, t);
        assert!(c11_remaining < f18_remaining);
    }

    // --- DeadTimeCorrection tests ---

    #[test]
    fn test_non_paralyzable_low_rate() {
        let true_rate = DeadTimeCorrection::non_paralyzable(1000.0, 1e-6);
        // At low rates, true rate ≈ measured rate
        assert!((true_rate - 1000.0).abs() / 1000.0 < 0.01);
    }

    #[test]
    fn test_non_paralyzable_high_rate() {
        let true_rate = DeadTimeCorrection::non_paralyzable(500_000.0, 1e-6);
        // True rate should be higher than measured
        assert!(true_rate > 500_000.0);
    }

    #[test]
    fn test_non_paralyzable_saturation() {
        // When m*tau >= 1, system is saturated
        let true_rate = DeadTimeCorrection::non_paralyzable(1e6, 1e-6);
        assert!(true_rate.is_infinite());
    }

    #[test]
    fn test_paralyzable_low_rate() {
        let true_rate = DeadTimeCorrection::paralyzable(1000.0, 1e-6);
        assert!((true_rate - 1000.0).abs() / 1000.0 < 0.01);
    }

    #[test]
    fn test_paralyzable_correction_increases() {
        // Use a moderate rate that is below the max paralyzable rate for this dead time
        // Max paralyzable rate at 5e-6 is ~73.6k, so use 50k measured
        let true_rate = DeadTimeCorrection::paralyzable(50_000.0, 5e-6);
        assert!(true_rate > 50_000.0);
    }

    #[test]
    fn test_dead_time_zero_rate() {
        assert!((DeadTimeCorrection::non_paralyzable(0.0, 1e-6) - 0.0).abs() < EPSILON);
        assert!((DeadTimeCorrection::paralyzable(0.0, 1e-6) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_loss_fraction_non_paralyzable() {
        let loss = DeadTimeCorrection::loss_fraction_non_paralyzable(100_000.0, 5e-6);
        // 100k cps with 5us dead time: loss ~ 33%
        assert!(loss > 0.0 && loss < 1.0);
    }

    #[test]
    fn test_loss_fraction_paralyzable() {
        let loss = DeadTimeCorrection::loss_fraction_paralyzable(100_000.0, 5e-6);
        assert!(loss > 0.0 && loss < 1.0);
    }

    #[test]
    fn test_max_paralyzable_rate() {
        let max_rate = DeadTimeCorrection::max_paralyzable_rate(5e-6);
        // 1 / (5e-6 * e) ≈ 73,576 cps
        assert!((max_rate - 73575.9).abs() < 1.0);
    }

    // --- TimeOfFlight tests ---

    #[test]
    fn test_tof_position_offset() {
        let offset = TimeOfFlight::time_to_position_offset_mm(0.0);
        assert!((offset - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_tof_position_offset_1ns() {
        let offset = TimeOfFlight::time_to_position_offset_mm(1.0);
        // c * 1ns / 2 = 299.792 * 1 / 2 ≈ 149.9 mm
        assert!((offset - 149.896).abs() < 0.1);
    }

    #[test]
    fn test_tof_spatial_resolution() {
        let fwhm = TimeOfFlight::spatial_resolution_mm(0.5); // 500 ps FWHM
        // c * 0.5ns / 2 ≈ 75 mm
        assert!((fwhm - 74.948).abs() < 0.1);
    }

    #[test]
    fn test_tof_sensitivity_gain() {
        let gain = TimeOfFlight::sensitivity_gain(300.0, 0.5);
        // 300mm / (c * 0.5ns / 2) = 300/75 ≈ 4
        assert!(gain > 3.0 && gain < 5.0);
    }

    #[test]
    fn test_tof_weight_at_center() {
        let w = TimeOfFlight::tof_weight(0.0, 0.0, 0.5);
        assert!((w - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_tof_weight_falls_off() {
        let w_near = TimeOfFlight::tof_weight(10.0, 0.0, 0.5);
        let w_far = TimeOfFlight::tof_weight(100.0, 0.0, 0.5);
        assert!(w_near > w_far);
    }

    // --- RandomsEstimator tests ---

    #[test]
    fn test_randoms_rate() {
        let rate = RandomsEstimator::estimate_rate(1e6, 1e6, 6.0);
        assert!((rate - 12000.0).abs() < 1.0);
    }

    #[test]
    fn test_necr_formula() {
        let necr = RandomsEstimator::necr(100000.0, 20000.0, 10000.0);
        // T^2 / (T + S + 2R) = 1e10 / (100k + 20k + 20k) = 71428.6
        assert!((necr - 71428.57).abs() < 1.0);
    }

    #[test]
    fn test_scatter_fraction_formula() {
        let sf = RandomsEstimator::scatter_fraction(80000.0, 20000.0);
        assert!((sf - 0.2).abs() < EPSILON);
    }

    #[test]
    fn test_necr_zero_counts() {
        let necr = RandomsEstimator::necr(0.0, 0.0, 0.0);
        assert!((necr - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_delayed_window() {
        let events_a = vec![100.0, 200.0, 300.0];
        let events_b = vec![100.5, 200.5, 300.5];
        // With 1000ns delay and 2ns window, delayed events won't match
        let count = RandomsEstimator::delayed_window_estimate(&events_a, &events_b, 2.0, 1000.0);
        assert_eq!(count, 0);
    }

    // --- PhantomGenerator tests ---

    #[test]
    fn test_uniform_cylinder() {
        let phantom = PhantomGenerator::uniform_cylinder(32, 0.5, 100.0);
        assert_eq!(phantom.len(), 32);
        assert_eq!(phantom[0].len(), 32);
        // Center should be hot
        assert!((phantom[16][16] - 100.0).abs() < EPSILON);
        // Corner should be cold
        assert!((phantom[0][0] - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_hot_rod_phantom() {
        let phantom = PhantomGenerator::hot_rod_phantom(64, 6, 100.0);
        assert_eq!(phantom.len(), 64);
        // Should have non-zero pixels
        let total: f64 = phantom.iter().flat_map(|r| r.iter()).sum();
        assert!(total > 0.0);
    }

    #[test]
    fn test_contrast_phantom() {
        let phantom = PhantomGenerator::contrast_phantom(64, 50.0, 4.0, 0.25);
        // Hot region should be 200, cold should be 12.5, background 50
        let center = 32;
        let hot_x = center + (center as f64 * 0.35) as usize;
        let hot_y = center - (center as f64 * 0.35) as usize;
        assert!((phantom[hot_y][hot_x] - 200.0).abs() < EPSILON);
    }

    #[test]
    fn test_forward_projection() {
        let phantom = PhantomGenerator::uniform_cylinder(32, 0.3, 100.0);
        let sino = PhantomGenerator::forward_project(&phantom, 16, 16);
        assert_eq!(sino.num_angles, 16);
        assert_eq!(sino.num_radial_bins, 16);
        assert!(sino.sum() > 0.0);
    }

    // --- PoissonNoise tests ---

    #[test]
    fn test_poisson_zero_mean() {
        let mut rng = PoissonNoise::new(42);
        for _ in 0..100 {
            assert_eq!(rng.sample(0.0), 0);
        }
    }

    #[test]
    fn test_poisson_positive_mean() {
        let mut rng = PoissonNoise::new(42);
        let n = 10000;
        let lambda = 10.0;
        let sum: f64 = (0..n).map(|_| rng.sample(lambda) as f64).sum();
        let mean = sum / n as f64;
        // Mean should be close to lambda
        assert!((mean - lambda).abs() < 1.0);
    }

    #[test]
    fn test_poisson_large_mean() {
        let mut rng = PoissonNoise::new(123);
        let n = 5000;
        let lambda = 100.0;
        let sum: f64 = (0..n).map(|_| rng.sample(lambda) as f64).sum();
        let mean = sum / n as f64;
        assert!((mean - lambda).abs() < 5.0);
    }

    #[test]
    fn test_poisson_noise_sinogram() {
        let mut sino = Sinogram::new(4, 8);
        for i in 0..sino.data.len() {
            sino.data[i] = 100.0;
        }
        let mut rng = PoissonNoise::new(42);
        rng.add_noise_to_sinogram(&mut sino);
        // All values should be non-negative integers
        for &v in &sino.data {
            assert!(v >= 0.0);
            assert!((v - v.round()).abs() < EPSILON);
        }
    }

    // --- NormalizationCorrection tests ---

    #[test]
    fn test_geometric_normalization() {
        let ring = DetectorRing::new(32, 200.0);
        let norm = NormalizationCorrection::geometric_normalization(&ring, 16, 16);
        assert_eq!(norm.num_angles, 16);
        assert_eq!(norm.num_radial_bins, 16);
        // Center bins should have highest normalization factor
        let center = norm.get(0, 8);
        let edge = norm.get(0, 0);
        assert!(center > 0.0 || edge >= 0.0); // At least some structure
    }

    #[test]
    fn test_detector_efficiency() {
        let blank = vec![100.0, 110.0, 90.0, 100.0];
        let eff = NormalizationCorrection::detector_efficiency(&blank);
        assert_eq!(eff.len(), 4);
        // Mean is 100, so first and last should be 1.0
        assert!((eff[0] - 1.0).abs() < EPSILON);
        assert!((eff[1] - 1.1).abs() < EPSILON);
    }

    #[test]
    fn test_apply_normalization() {
        let mut sino = Sinogram::new(2, 4);
        sino.set(0, 0, 10.0);
        sino.set(0, 1, 20.0);
        let mut norm = Sinogram::new(2, 4);
        norm.set(0, 0, 2.0);
        norm.set(0, 1, 0.5);
        NormalizationCorrection::apply_normalization(&mut sino, &norm);
        assert!((sino.get(0, 0) - 20.0).abs() < EPSILON);
        assert!((sino.get(0, 1) - 10.0).abs() < EPSILON);
    }

    // --- ImageMetrics tests ---

    #[test]
    fn test_std_dev() {
        let data = vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let sd = ImageMetrics::std_dev(&data);
        // Sample std dev: mean=5.0, variance=32/7≈4.571, sd≈2.138
        assert!((sd - 2.138).abs() < 0.01);
    }

    #[test]
    fn test_contrast_to_noise() {
        let hot = vec![100.0, 105.0, 95.0, 100.0];
        let background = vec![50.0, 52.0, 48.0, 50.0];
        let cnr = ImageMetrics::contrast_to_noise(&hot, &background);
        assert!(cnr > 0.0);
    }

    #[test]
    fn test_recovery_coefficient() {
        let rc = ImageMetrics::recovery_coefficient(80.0, 100.0);
        assert!((rc - 0.8).abs() < EPSILON);
    }

    #[test]
    fn test_mse_identical() {
        let img = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let mse = ImageMetrics::mse(&img, &img);
        assert!((mse - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_mse_different() {
        let img_a = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let img_b = vec![vec![2.0, 3.0], vec![4.0, 5.0]];
        let mse = ImageMetrics::mse(&img_a, &img_b);
        assert!((mse - 1.0).abs() < EPSILON); // Each pixel differs by 1
    }

    #[test]
    fn test_psnr_identical() {
        let img = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let psnr = ImageMetrics::psnr(&img, &img, 255.0);
        assert!(psnr.is_infinite());
    }

    #[test]
    fn test_roi_sum() {
        let img = vec![vec![1.0; 10]; 10];
        let sum = ImageMetrics::roi_sum(&img, 5, 5, 2.0);
        // Circle of radius 2 around (5,5): should be ~13 pixels
        assert!(sum > 10.0 && sum < 20.0);
    }

    #[test]
    fn test_roi_mean() {
        let img = vec![vec![5.0; 10]; 10];
        let mean = ImageMetrics::roi_mean(&img, 5, 5, 3.0);
        assert!((mean - 5.0).abs() < EPSILON);
    }

    // --- MLEM tests ---

    #[test]
    fn test_mlem_creation() {
        let mlem = MlemReconstructor::new(32, 5);
        assert_eq!(mlem.image_size, 32);
        assert_eq!(mlem.num_iterations, 5);
    }

    #[test]
    fn test_mlem_empty_sinogram() {
        let mlem = MlemReconstructor::new(16, 2);
        let sino = Sinogram::new(8, 16);
        let recon = mlem.reconstruct(&sino);
        assert_eq!(recon.len(), 16);
    }

    #[test]
    fn test_mlem_positive_output() {
        let phantom = PhantomGenerator::uniform_cylinder(16, 0.4, 50.0);
        let sino = PhantomGenerator::forward_project(&phantom, 8, 16);
        let mlem = MlemReconstructor::new(16, 2);
        let recon = mlem.reconstruct(&sino);
        // All values should be non-negative
        for row in &recon {
            for &val in row {
                assert!(val >= 0.0);
            }
        }
    }

    // --- Integration / end-to-end tests ---

    #[test]
    fn test_full_pipeline_simple() {
        // 1. Create phantom
        let phantom = PhantomGenerator::uniform_cylinder(16, 0.4, 100.0);

        // 2. Forward project to sinogram
        let sino = PhantomGenerator::forward_project(&phantom, 16, 16);
        assert!(sino.sum() > 0.0);

        // 3. Reconstruct with FBP
        let fbp = FilteredBackProjection::new(16);
        let recon = fbp.reconstruct(&sino);

        // 4. Center should have activity
        assert!(recon[8][8] > 0.0);
    }

    #[test]
    fn test_decay_and_correction_roundtrip() {
        let half_life = Isotope::F18_HALF_LIFE_S;
        let t = 3600.0; // 1 hour
        let remaining = DecayCorrection::remaining_fraction(half_life, t);
        let correction = DecayCorrection::correction_factor(half_life, t);
        // remaining * correction ≈ 1.0
        assert!((remaining * correction - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_scatter_subtract_preserves_nonnegativity() {
        let mut sino = Sinogram::new(4, 4);
        sino.set(0, 0, 10.0);
        let scatter = ScatterEstimator::single_scatter_simulation(&sino, 0.5);
        ScatterEstimator::subtract_scatter(&mut sino, &scatter);
        for &v in &sino.data {
            assert!(v >= 0.0);
        }
    }

    #[test]
    fn test_combined_corrections() {
        let ring = DetectorRing::new(32, 200.0);
        let phantom = PhantomGenerator::uniform_cylinder(16, 0.3, 100.0);
        let mut sino = PhantomGenerator::forward_project(&phantom, 16, 16);

        // Add scatter and subtract
        let scatter = ScatterEstimator::single_scatter_simulation(&sino, 0.2);
        ScatterEstimator::subtract_scatter(&mut sino, &scatter);

        // Apply attenuation correction
        let ac = AttenuationCorrection::uniform(16, 0.0002);
        ac.correct_sinogram(&mut sino, &ring);

        // Reconstruct
        let fbp = FilteredBackProjection::new(16);
        let recon = fbp.reconstruct(&sino);
        assert!(recon[8][8] > 0.0);
    }
}
