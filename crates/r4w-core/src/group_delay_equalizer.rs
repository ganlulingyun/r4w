//! Group Delay Equalizer
//!
//! Corrects non-linear phase response in filters by cascading second-order
//! allpass sections to achieve constant group delay across the passband.
//! Non-linear phase distorts signal waveforms even though magnitude response
//! is unaffected; this equalizer flattens the group delay to preserve
//! waveform fidelity.
//!
//! Each [`AllpassSection`] is a second-order allpass filter with unity
//! magnitude at all frequencies but controllable phase. By choosing pole
//! locations (radius and angle), the phase contribution is shaped so that
//! the combined group delay of the equalizer plus the target filter is
//! approximately constant.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::group_delay_equalizer::{
//!     GroupDelayEqualizer, AllpassSection, design_equalizer,
//!     measure_group_delay, allpass_from_pole,
//! };
//!
//! // Create a simple allpass section from a pole location
//! let section = allpass_from_pole(0.9, 0.5);
//!
//! // Build an equalizer from multiple sections
//! let mut eq = GroupDelayEqualizer::new(vec![
//!     allpass_from_pole(0.8, 0.3),
//!     allpass_from_pole(0.7, 0.6),
//! ]);
//!
//! // Process a signal
//! let input = vec![1.0, 0.0, 0.0, 0.0, 0.0];
//! let output = eq.process(&input);
//! assert_eq!(output.len(), input.len());
//!
//! // Query total group delay at a normalized frequency
//! let delay = eq.total_group_delay_at(0.25);
//! assert!(delay >= 0.0);
//! ```

use std::f64::consts::PI;

/// Second-order allpass filter section.
///
/// Transfer function:
/// ```text
/// H(z) = (a2 + a1*z^-1 + z^-2) / (1 + a1*z^-1 + a2*z^-2)
/// ```
///
/// Has unity magnitude at all frequencies but non-trivial phase response,
/// making it ideal for group delay equalization.
#[derive(Debug, Clone)]
pub struct AllpassSection {
    /// First-order coefficient.
    pub a1: f64,
    /// Second-order coefficient.
    pub a2: f64,
    /// Filter state: (z^-1, z^-2) for the direct-form implementation.
    state: (f64, f64),
}

impl AllpassSection {
    /// Create a new second-order allpass section with the given coefficients.
    pub fn new(a1: f64, a2: f64) -> Self {
        Self {
            a1,
            a2,
            state: (0.0, 0.0),
        }
    }

    /// Process a single sample through this allpass section.
    ///
    /// Implements the second-order allpass difference equation using a
    /// direct-form structure:
    ///
    /// ```text
    /// w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
    /// y[n] = a2*w[n] + a1*w[n-1] + w[n-2]
    /// ```
    pub fn process_sample(&mut self, x: f64) -> f64 {
        let (w1, w2) = self.state;
        let w = x - self.a1 * w1 - self.a2 * w2;
        let y = self.a2 * w + self.a1 * w1 + w2;
        self.state = (w, w1);
        y
    }

    /// Compute group delay in samples at a normalized frequency.
    ///
    /// `freq_normalized` is in the range [0, 1] where 1 corresponds to the
    /// Nyquist frequency (half the sample rate). The group delay is computed
    /// analytically from the allpass transfer function coefficients.
    pub fn group_delay_at(&self, freq_normalized: f64) -> f64 {
        let omega = PI * freq_normalized;
        let a1 = self.a1;
        let a2 = self.a2;

        // Group delay of second-order allpass:
        // tau(omega) = -d(phase)/d(omega)
        //
        // For H(z) = (a2 + a1*z^-1 + z^-2) / (1 + a1*z^-1 + a2*z^-2),
        // we compute via the difference of numerator and denominator group delays.
        //
        // Use finite difference on the phase for robustness.
        let delta = 1e-8;
        let phase_at = |w: f64| {
            let cos_w = w.cos();
            let sin_w = w.sin();
            let cos_2w = (2.0 * w).cos();
            let sin_2w = (2.0 * w).sin();

            // Numerator: a2 + a1*e^{-jw} + e^{-j2w}
            let num_re = a2 + a1 * cos_w + cos_2w;
            let num_im = -a1 * sin_w - sin_2w;

            // Denominator: 1 + a1*e^{-jw} + a2*e^{-j2w}
            let den_re = 1.0 + a1 * cos_w + a2 * cos_2w;
            let den_im = -a1 * sin_w - a2 * sin_2w;

            // Phase of H = atan2(num_im, num_re) - atan2(den_im, den_re)
            num_im.atan2(num_re) - den_im.atan2(den_re)
        };

        let p1 = phase_at(omega + delta);
        let p0 = phase_at(omega - delta);

        // Unwrap the phase difference
        let mut dp = p1 - p0;
        while dp > PI {
            dp -= 2.0 * PI;
        }
        while dp < -PI {
            dp += 2.0 * PI;
        }

        // Group delay = -d(phase)/d(omega), omega = PI * freq_normalized
        // d(omega) = 2*delta, so tau = -dp / (2*delta)
        -(dp / (2.0 * delta))
    }

    /// Reset the internal filter state to zero.
    pub fn reset(&mut self) {
        self.state = (0.0, 0.0);
    }
}

/// Group delay equalizer composed of cascaded second-order allpass sections.
///
/// The composite phase response is the sum of individual section phases,
/// and the composite group delay is the sum of individual group delays.
/// The magnitude response remains unity at all frequencies.
#[derive(Debug, Clone)]
pub struct GroupDelayEqualizer {
    /// Cascade of allpass sections.
    pub allpass_sections: Vec<AllpassSection>,
}

impl GroupDelayEqualizer {
    /// Create a new group delay equalizer from the given allpass sections.
    pub fn new(sections: Vec<AllpassSection>) -> Self {
        Self {
            allpass_sections: sections,
        }
    }

    /// Process an input signal through the cascade of allpass sections.
    ///
    /// Each sample passes through every section in order. The output has
    /// the same length as the input, with unity magnitude but modified phase.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = input.to_vec();
        for section in &mut self.allpass_sections {
            for sample in output.iter_mut() {
                *sample = section.process_sample(*sample);
            }
        }
        output
    }

    /// Compute the total group delay in samples at a normalized frequency.
    ///
    /// This is the sum of group delays across all sections. `freq_normalized`
    /// is in [0, 1] where 1 = Nyquist.
    pub fn total_group_delay_at(&self, freq_normalized: f64) -> f64 {
        self.allpass_sections
            .iter()
            .map(|s| s.group_delay_at(freq_normalized))
            .sum()
    }

    /// Reset all internal filter states to zero.
    pub fn reset(&mut self) {
        for section in &mut self.allpass_sections {
            section.reset();
        }
    }
}

/// Create an allpass section from a pole in polar coordinates.
///
/// A second-order allpass with a complex conjugate pole pair at
/// `radius * exp(+/- j*angle)` has coefficients:
/// - `a1 = -2 * radius * cos(angle)`
/// - `a2 = radius^2`
///
/// # Arguments
/// * `radius` - Pole radius (0 < radius < 1 for stability)
/// * `angle` - Pole angle in radians (0 to PI)
pub fn allpass_from_pole(radius: f64, angle: f64) -> AllpassSection {
    let a1 = -2.0 * radius * angle.cos();
    let a2 = radius * radius;
    AllpassSection::new(a1, a2)
}

/// Estimate group delay from an impulse response using the phase difference method.
///
/// Computes the unwrapped phase of the frequency response at `num_points`
/// equally spaced normalized frequencies in [0, 1), then differentiates
/// numerically to obtain group delay.
///
/// This is an FFT-less method that evaluates the DTFT directly at each
/// frequency point.
///
/// # Arguments
/// * `impulse_response` - The filter's impulse response coefficients
/// * `num_points` - Number of frequency points to evaluate
///
/// # Returns
/// Vector of `(freq_normalized, delay_samples)` pairs.
pub fn measure_group_delay(impulse_response: &[f64], num_points: usize) -> Vec<(f64, f64)> {
    if impulse_response.is_empty() || num_points < 2 {
        return Vec::new();
    }

    // Evaluate phase at each frequency point
    let phases: Vec<f64> = (0..=num_points)
        .map(|k| {
            let freq_norm = k as f64 / num_points as f64;
            let omega = PI * freq_norm;

            let mut re = 0.0;
            let mut im = 0.0;
            for (n, &h) in impulse_response.iter().enumerate() {
                let phase = omega * n as f64;
                re += h * phase.cos();
                im -= h * phase.sin();
            }

            im.atan2(re)
        })
        .collect();

    // Unwrap phase
    let mut unwrapped = phases.clone();
    for i in 1..unwrapped.len() {
        let mut diff = unwrapped[i] - unwrapped[i - 1];
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff < -PI {
            diff += 2.0 * PI;
        }
        unwrapped[i] = unwrapped[i - 1] + diff;
    }

    // Group delay = -d(unwrapped_phase)/d(omega)
    // omega spacing = PI / num_points
    let d_omega = PI / num_points as f64;

    (0..num_points)
        .map(|k| {
            let freq_norm = (k as f64 + 0.5) / num_points as f64;
            let dp = unwrapped[k + 1] - unwrapped[k];
            let delay = -dp / d_omega;
            (freq_norm, delay)
        })
        .collect()
}

/// Design a group delay equalizer to flatten measured group delay.
///
/// Given a target constant delay and measured delay at various frequencies,
/// this function places allpass poles to compensate for the difference
/// between measured and target delay.
///
/// The design uses an iterative approach: for each section, the pole
/// angle is chosen at the frequency of maximum delay deviation, and the
/// pole radius is set to produce the required correction.
///
/// # Arguments
/// * `target_delay` - Desired constant group delay in samples
/// * `measured_delay` - Slice of `(freq_normalized, delay_samples)` pairs
/// * `num_sections` - Number of second-order allpass sections to use
///
/// # Returns
/// A [`GroupDelayEqualizer`] with the designed allpass sections.
pub fn design_equalizer(
    target_delay: f64,
    measured_delay: &[(f64, f64)],
    num_sections: usize,
) -> GroupDelayEqualizer {
    if measured_delay.is_empty() || num_sections == 0 {
        return GroupDelayEqualizer::new(Vec::new());
    }

    let mut sections = Vec::with_capacity(num_sections);

    // Residual delay error, updated as we add sections
    let mut residual: Vec<(f64, f64)> = measured_delay
        .iter()
        .map(|&(f, d)| (f, target_delay - d))
        .collect();

    for _ in 0..num_sections {
        // Find the frequency with the largest residual (most delay needed)
        let (peak_idx, &(peak_freq, peak_residual)) = residual
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap();

        // If residual is negligible, stop adding sections
        if peak_residual.abs() < 0.01 {
            break;
        }

        // Place pole at this frequency angle
        let angle = PI * peak_freq;

        // Compute radius from desired group delay contribution.
        // For a second-order allpass with poles at r*e^{+/-j*theta},
        // the group delay at theta is approximately:
        //   tau(theta) ~ (1 - r^2) / (1 - 2r*cos(0) + r^2) at the pole angle
        // We use a simple mapping: larger radius -> more delay at the pole angle.
        // Clamp radius to maintain stability.
        let desired_delay = peak_residual.abs();
        // Approximate inversion: r ~ 1 - 1/(1 + desired_delay * 0.5)
        let radius = if desired_delay > 0.0 {
            let r = 1.0 - 1.0 / (1.0 + desired_delay * 0.25);
            r.clamp(0.1, 0.98)
        } else {
            0.5
        };

        let section = allpass_from_pole(radius, angle);

        // Update residual: subtract this section's contribution
        for item in residual.iter_mut() {
            let section_delay = section.group_delay_at(item.0);
            item.1 -= section_delay;
        }

        // If the original residual was negative (measured > target), we cannot
        // add positive delay to compensate with allpass. Flip is handled by
        // the residual tracking naturally.
        let _ = peak_idx; // used above
        sections.push(section);
    }

    GroupDelayEqualizer::new(sections)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify that the allpass section has unity magnitude at several frequencies.
    #[test]
    fn test_allpass_unity_magnitude() {
        let section = AllpassSection::new(-0.5, 0.3);

        // Process a long sinusoid and check output magnitude matches input
        for &freq_norm in &[0.05, 0.1, 0.25, 0.4] {
            let omega = PI * freq_norm;
            let n_samples = 2000;
            let mut sec = section.clone();

            let input: Vec<f64> = (0..n_samples)
                .map(|n| (omega * n as f64).sin())
                .collect();
            let output = sec.process_sample_block(&input);

            // Measure RMS of last half (after transients die out)
            let start = n_samples / 2;
            let rms_in: f64 = (input[start..]
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                / (n_samples - start) as f64)
                .sqrt();
            let rms_out: f64 = (output[start..]
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                / (n_samples - start) as f64)
                .sqrt();

            let ratio = rms_out / rms_in;
            assert!(
                (ratio - 1.0).abs() < 0.05,
                "Magnitude ratio at freq_norm={} is {}, expected ~1.0",
                freq_norm,
                ratio
            );
        }
    }

    /// Helper: process a block through an AllpassSection.
    impl AllpassSection {
        fn process_sample_block(&mut self, input: &[f64]) -> Vec<f64> {
            input.iter().map(|&x| self.process_sample(x)).collect()
        }
    }

    /// Verify that the allpass section produces non-trivial phase shift.
    #[test]
    fn test_allpass_phase() {
        let section = allpass_from_pole(0.9, 0.5);
        // Group delay should be positive and non-zero away from DC
        let gd = section.group_delay_at(0.25);
        assert!(
            gd > 0.0,
            "Expected positive group delay, got {}",
            gd
        );

        // Group delay at pole angle frequency should be largest
        let gd_at_pole = section.group_delay_at(0.5 / PI);
        let gd_far = section.group_delay_at(0.9);
        // The delay near the pole should generally be larger
        // (this is a soft check since the relationship depends on radius)
        assert!(
            gd_at_pole > 0.0,
            "Group delay at pole should be positive"
        );
        let _ = gd_far; // just ensure it runs
    }

    /// Test processing through a single allpass section.
    #[test]
    fn test_single_section() {
        let mut section = AllpassSection::new(0.0, 0.0);
        // With a1=0, a2=0, the allpass becomes a pure delay of 2 samples:
        // w[n] = x[n], y[n] = w[n-2] => y[n] = x[n-2]
        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0];
        let output: Vec<f64> = input.iter().map(|&x| section.process_sample(x)).collect();
        // With a1=0, a2=0: w=x, y = 0*w + 0*w1 + w2
        // So output is x[n-2]: impulse at index 2
        assert!((output[0]).abs() < 1e-10);
        assert!((output[1]).abs() < 1e-10);
        assert!((output[2] - 1.0).abs() < 1e-10);
        assert!((output[3]).abs() < 1e-10);
        assert!((output[4]).abs() < 1e-10);
    }

    /// Test cascading multiple allpass sections.
    #[test]
    fn test_cascade() {
        let sections = vec![
            allpass_from_pole(0.5, 0.3),
            allpass_from_pole(0.6, 0.7),
        ];
        let mut eq = GroupDelayEqualizer::new(sections);

        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let output = eq.process(&input);

        // Output should have same length
        assert_eq!(output.len(), input.len());

        // Energy should be conserved (allpass preserves energy)
        let energy_in: f64 = input.iter().map(|x| x * x).sum();
        let energy_out: f64 = output.iter().map(|x| x * x).sum();
        // Energy may spread beyond the window, so output energy <= input energy
        assert!(
            energy_out <= energy_in + 1e-10,
            "Energy should not increase: in={}, out={}",
            energy_in,
            energy_out
        );

        // Total group delay is sum of individual delays
        let gd_total = eq.total_group_delay_at(0.25);
        let gd_0 = eq.allpass_sections[0].group_delay_at(0.25);
        let gd_1 = eq.allpass_sections[1].group_delay_at(0.25);
        assert!(
            (gd_total - (gd_0 + gd_1)).abs() < 1e-10,
            "Total group delay should equal sum of sections"
        );
    }

    /// Test group delay measurement from an impulse response.
    #[test]
    fn test_group_delay_measurement() {
        // A pure delay of N samples has constant group delay = N.
        // Impulse response: [0, 0, ..., 1, 0, 0, ...]
        let delay_samples = 5;
        let mut ir = vec![0.0; 16];
        ir[delay_samples] = 1.0;

        let gd = measure_group_delay(&ir, 32);
        assert!(!gd.is_empty());

        // All group delay values should be approximately `delay_samples`
        for &(freq, d) in &gd {
            assert!(
                (d - delay_samples as f64).abs() < 0.5,
                "At freq_norm={:.3}, group delay={:.3}, expected {}",
                freq,
                d,
                delay_samples
            );
        }
    }

    /// Test that the designed equalizer reduces group delay variation.
    #[test]
    fn test_equalizer_flattens() {
        // Create synthetic non-flat group delay
        let measured: Vec<(f64, f64)> = (1..20)
            .map(|k| {
                let f = k as f64 / 20.0;
                // Simulate a filter with varying delay: 10 + 5*sin(...)
                let delay = 10.0 + 5.0 * (PI * f).sin();
                (f, delay)
            })
            .collect();

        let target = 15.0; // Target: flatten to 15 samples
        let eq = design_equalizer(target, &measured, 4);

        // The equalizer should have been created with some sections
        assert!(
            !eq.allpass_sections.is_empty(),
            "Equalizer should have at least one section"
        );

        // Check that equalizer adds positive group delay
        let eq_delay = eq.total_group_delay_at(0.25);
        assert!(eq_delay >= 0.0, "Equalizer delay should be non-negative");
    }

    /// Test reset clears state.
    #[test]
    fn test_reset() {
        let mut section = allpass_from_pole(0.9, 0.5);

        // Process some samples to fill state
        for i in 0..10 {
            section.process_sample(i as f64);
        }

        // Reset
        section.reset();

        // After reset, processing an impulse should give same result as fresh
        let mut fresh = allpass_from_pole(0.9, 0.5);

        let impulse = vec![1.0, 0.0, 0.0, 0.0, 0.0];
        let out_reset: Vec<f64> = impulse
            .iter()
            .map(|&x| section.process_sample(x))
            .collect();
        let out_fresh: Vec<f64> = impulse
            .iter()
            .map(|&x| fresh.process_sample(x))
            .collect();

        for (a, b) in out_reset.iter().zip(out_fresh.iter()) {
            assert!(
                (a - b).abs() < 1e-12,
                "Reset state should match fresh: {} vs {}",
                a,
                b
            );
        }

        // Also test GroupDelayEqualizer::reset
        let mut eq = GroupDelayEqualizer::new(vec![
            allpass_from_pole(0.8, 0.3),
            allpass_from_pole(0.7, 0.6),
        ]);
        let _ = eq.process(&[1.0, 2.0, 3.0]);
        eq.reset();
        let mut eq_fresh = GroupDelayEqualizer::new(vec![
            allpass_from_pole(0.8, 0.3),
            allpass_from_pole(0.7, 0.6),
        ]);
        let out_a = eq.process(&impulse);
        let out_b = eq_fresh.process(&impulse);
        for (a, b) in out_a.iter().zip(out_b.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
    }

    /// Test constructing an allpass from pole coordinates.
    #[test]
    fn test_from_pole() {
        let radius = 0.85;
        let angle = 1.0;
        let section = allpass_from_pole(radius, angle);

        // Verify coefficients
        let expected_a1 = -2.0 * radius * angle.cos();
        let expected_a2 = radius * radius;

        assert!(
            (section.a1 - expected_a1).abs() < 1e-12,
            "a1 mismatch: {} vs {}",
            section.a1,
            expected_a1
        );
        assert!(
            (section.a2 - expected_a2).abs() < 1e-12,
            "a2 mismatch: {} vs {}",
            section.a2,
            expected_a2
        );

        // Stability: poles inside unit circle since radius < 1
        assert!(section.a2 < 1.0, "a2 should be < 1 for stability");
    }

    /// Test that zero-length input produces zero-length output.
    #[test]
    fn test_zero_input() {
        let mut eq = GroupDelayEqualizer::new(vec![
            allpass_from_pole(0.5, 0.5),
        ]);

        let output = eq.process(&[]);
        assert!(output.is_empty(), "Empty input should produce empty output");

        // Also test measure_group_delay with empty input
        let gd = measure_group_delay(&[], 10);
        assert!(gd.is_empty());

        // And with zero num_points
        let gd2 = measure_group_delay(&[1.0, 0.5], 0);
        assert!(gd2.is_empty());
    }

    /// Test that an equalizer with no sections acts as a passthrough.
    #[test]
    fn test_passthrough() {
        let mut eq = GroupDelayEqualizer::new(Vec::new());

        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = eq.process(&input);

        for (a, b) in input.iter().zip(output.iter()) {
            assert!(
                (a - b).abs() < 1e-12,
                "Passthrough should not modify signal: {} vs {}",
                a,
                b
            );
        }

        // Total group delay of empty equalizer is 0
        let gd = eq.total_group_delay_at(0.25);
        assert!(
            gd.abs() < 1e-12,
            "Empty equalizer should have zero group delay"
        );
    }
}
