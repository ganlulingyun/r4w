//! Code snippet data structures for the Code Explorer view
//!
//! This module defines the structures used to organize and display
//! educational code snippets with explanations.

/// A single code snippet with its explanation
#[derive(Debug, Clone)]
pub struct CodeSnippet {
    /// Function/method name (e.g., "generate_samples()")
    pub name: &'static str,
    /// Brief one-line description
    pub brief: &'static str,
    /// The actual Rust code
    pub code: &'static str,
    /// Educational explanation of what this code does
    pub explanation: &'static str,
    /// Key DSP concepts demonstrated
    pub concepts: &'static [&'static str],
}

/// A category of related functions (e.g., "Generation", "Modulation")
#[derive(Debug, Clone)]
pub struct CodeCategory {
    /// Category name
    pub name: &'static str,
    /// Brief description of this category
    pub description: &'static str,
    /// Code snippets in this category
    pub snippets: &'static [CodeSnippet],
}

/// All code for a single waveform
#[derive(Debug, Clone)]
pub struct WaveformCode {
    /// Waveform identifier (matches WaveformFactory names)
    pub waveform_id: &'static str,
    /// Display name (e.g., "Continuous Wave (CW)")
    pub display_name: &'static str,
    /// Brief educational introduction
    pub introduction: &'static str,
    /// Complexity level (1-5)
    pub complexity: u8,
    /// Categories of code (references to static categories)
    pub categories: &'static [&'static CodeCategory],
}

/// Registry of all available waveform code
pub struct CodeRegistry;

/// Static array of all waveforms (ordered by complexity)
static ALL_WAVEFORMS: &[&WaveformCode] = &[
    &super::cw_snippets::CW_CODE,
    &super::ook_snippets::OOK_CODE,
    &super::am_snippets::AM_CODE,
    &super::ppm_snippets::PPM_CODE,
    &super::fm_snippets::FM_CODE,
    &super::fsk_snippets::FSK_CODE,
    &super::psk_snippets::PSK_CODE,
    &super::qam_snippets::QAM_CODE,
    &super::ofdm_snippets::OFDM_CODE,
    &super::css_snippets::CSS_CODE,
    &super::dsss_snippets::DSSS_CODE,
    &super::fhss_snippets::FHSS_CODE,
    &super::zigbee_snippets::ZIGBEE_CODE,
    &super::uwb_snippets::UWB_CODE,
    &super::fmcw_snippets::FMCW_CODE,
];

impl CodeRegistry {
    /// Get all available waveforms
    pub fn all_waveforms() -> &'static [&'static WaveformCode] {
        ALL_WAVEFORMS
    }

    /// Get code for a specific waveform by ID
    pub fn get_waveform(id: &str) -> Option<&'static WaveformCode> {
        Self::all_waveforms()
            .iter()
            .find(|w| w.waveform_id.eq_ignore_ascii_case(id))
            .copied()
    }

    /// List all waveform IDs
    pub fn list_waveform_ids() -> Vec<&'static str> {
        Self::all_waveforms()
            .iter()
            .map(|w| w.waveform_id)
            .collect()
    }
}
