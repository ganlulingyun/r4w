//! # Block Gateway
//!
//! Dynamic block discovery and instantiation framework. Provides a
//! registry of available DSP blocks with metadata, enabling runtime
//! block creation by name. Useful for pipeline builders, scripting
//! interfaces, and plugin systems.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::block_gateway::{BlockRegistry, BlockInfo, BlockCategory};
//!
//! let mut registry = BlockRegistry::new();
//! registry.register(BlockInfo {
//!     name: "gain".into(),
//!     category: BlockCategory::Math,
//!     description: "Multiply by constant".into(),
//!     num_inputs: 1,
//!     num_outputs: 1,
//!     parameters: vec![("gain".into(), "1.0".into())],
//! });
//! assert!(registry.find("gain").is_some());
//! ```

use std::collections::HashMap;

/// Block category for organization.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum BlockCategory {
    Source,
    Sink,
    Math,
    Filter,
    Modulation,
    Demodulation,
    Coding,
    Synchronization,
    Channel,
    Analysis,
    Utility,
    Custom(String),
}

impl BlockCategory {
    /// Get display name.
    pub fn name(&self) -> &str {
        match self {
            Self::Source => "Source",
            Self::Sink => "Sink",
            Self::Math => "Math",
            Self::Filter => "Filter",
            Self::Modulation => "Modulation",
            Self::Demodulation => "Demodulation",
            Self::Coding => "Coding",
            Self::Synchronization => "Synchronization",
            Self::Channel => "Channel",
            Self::Analysis => "Analysis",
            Self::Utility => "Utility",
            Self::Custom(s) => s,
        }
    }
}

/// Information about a registered block.
#[derive(Debug, Clone)]
pub struct BlockInfo {
    /// Block name (unique identifier).
    pub name: String,
    /// Category for organization.
    pub category: BlockCategory,
    /// Human-readable description.
    pub description: String,
    /// Number of input ports.
    pub num_inputs: usize,
    /// Number of output ports.
    pub num_outputs: usize,
    /// Parameter names and default values.
    pub parameters: Vec<(String, String)>,
}

impl BlockInfo {
    /// Check if this is a source (no inputs).
    pub fn is_source(&self) -> bool {
        self.num_inputs == 0 && self.num_outputs > 0
    }

    /// Check if this is a sink (no outputs).
    pub fn is_sink(&self) -> bool {
        self.num_inputs > 0 && self.num_outputs == 0
    }

    /// Get parameter default value by name.
    pub fn get_param_default(&self, name: &str) -> Option<&str> {
        self.parameters
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.as_str())
    }
}

/// Registry of available DSP blocks.
#[derive(Debug, Clone)]
pub struct BlockRegistry {
    blocks: HashMap<String, BlockInfo>,
    categories: HashMap<String, Vec<String>>,
}

impl BlockRegistry {
    /// Create a new empty registry.
    pub fn new() -> Self {
        Self {
            blocks: HashMap::new(),
            categories: HashMap::new(),
        }
    }

    /// Register a new block.
    pub fn register(&mut self, info: BlockInfo) {
        let name = info.name.clone();
        let cat = info.category.name().to_string();
        self.categories
            .entry(cat)
            .or_default()
            .push(name.clone());
        self.blocks.insert(name, info);
    }

    /// Find a block by name.
    pub fn find(&self, name: &str) -> Option<&BlockInfo> {
        self.blocks.get(name)
    }

    /// List all registered block names.
    pub fn list_names(&self) -> Vec<&str> {
        let mut names: Vec<&str> = self.blocks.keys().map(|s| s.as_str()).collect();
        names.sort();
        names
    }

    /// List blocks in a category.
    pub fn list_category(&self, category: &str) -> Vec<&BlockInfo> {
        self.categories
            .get(category)
            .map(|names| {
                names
                    .iter()
                    .filter_map(|n| self.blocks.get(n))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// List all categories.
    pub fn categories(&self) -> Vec<&str> {
        let mut cats: Vec<&str> = self.categories.keys().map(|s| s.as_str()).collect();
        cats.sort();
        cats
    }

    /// Get total number of registered blocks.
    pub fn count(&self) -> usize {
        self.blocks.len()
    }

    /// Search blocks by keyword in name or description.
    pub fn search(&self, keyword: &str) -> Vec<&BlockInfo> {
        let kw = keyword.to_lowercase();
        self.blocks
            .values()
            .filter(|info| {
                info.name.to_lowercase().contains(&kw)
                    || info.description.to_lowercase().contains(&kw)
            })
            .collect()
    }

    /// Remove a block by name.
    pub fn unregister(&mut self, name: &str) -> Option<BlockInfo> {
        if let Some(info) = self.blocks.remove(name) {
            let cat = info.category.name().to_string();
            if let Some(names) = self.categories.get_mut(&cat) {
                names.retain(|n| n != name);
                if names.is_empty() {
                    self.categories.remove(&cat);
                }
            }
            Some(info)
        } else {
            None
        }
    }
}

impl Default for BlockRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Create a standard registry with common DSP blocks pre-registered.
pub fn standard_registry() -> BlockRegistry {
    let mut reg = BlockRegistry::new();

    let blocks = vec![
        ("signal_source", BlockCategory::Source, "Generate test signals", 0, 1, vec![("frequency", "1000.0"), ("amplitude", "1.0"), ("sample_rate", "48000.0")]),
        ("noise_source", BlockCategory::Source, "Generate noise", 0, 1, vec![("type", "gaussian"), ("amplitude", "1.0")]),
        ("file_source", BlockCategory::Source, "Read samples from file", 0, 1, vec![("path", ""), ("format", "cf64")]),
        ("null_sink", BlockCategory::Sink, "Discard samples", 1, 0, vec![]),
        ("file_sink", BlockCategory::Sink, "Write samples to file", 1, 0, vec![("path", ""), ("format", "cf64")]),
        ("add", BlockCategory::Math, "Add streams element-wise", 2, 1, vec![]),
        ("multiply", BlockCategory::Math, "Multiply streams", 2, 1, vec![]),
        ("multiply_const", BlockCategory::Math, "Multiply by constant", 1, 1, vec![("constant", "1.0")]),
        ("fir_filter", BlockCategory::Filter, "FIR filter", 1, 1, vec![("taps", ""), ("type", "lowpass")]),
        ("agc", BlockCategory::Synchronization, "Automatic gain control", 1, 1, vec![("target", "1.0"), ("rate", "0.001")]),
    ];

    for (name, cat, desc, ni, no, params) in blocks {
        reg.register(BlockInfo {
            name: name.to_string(),
            category: cat,
            description: desc.to_string(),
            num_inputs: ni,
            num_outputs: no,
            parameters: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        });
    }

    reg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_find() {
        let mut reg = BlockRegistry::new();
        reg.register(BlockInfo {
            name: "gain".into(),
            category: BlockCategory::Math,
            description: "Multiply by constant".into(),
            num_inputs: 1,
            num_outputs: 1,
            parameters: vec![("gain".into(), "1.0".into())],
        });
        assert!(reg.find("gain").is_some());
        assert!(reg.find("nonexistent").is_none());
    }

    #[test]
    fn test_list_names() {
        let mut reg = BlockRegistry::new();
        reg.register(BlockInfo { name: "b".into(), category: BlockCategory::Math, description: String::new(), num_inputs: 1, num_outputs: 1, parameters: vec![] });
        reg.register(BlockInfo { name: "a".into(), category: BlockCategory::Filter, description: String::new(), num_inputs: 1, num_outputs: 1, parameters: vec![] });
        let names = reg.list_names();
        assert_eq!(names, vec!["a", "b"]); // Sorted.
    }

    #[test]
    fn test_list_category() {
        let mut reg = BlockRegistry::new();
        reg.register(BlockInfo { name: "add".into(), category: BlockCategory::Math, description: String::new(), num_inputs: 2, num_outputs: 1, parameters: vec![] });
        reg.register(BlockInfo { name: "mul".into(), category: BlockCategory::Math, description: String::new(), num_inputs: 2, num_outputs: 1, parameters: vec![] });
        reg.register(BlockInfo { name: "fir".into(), category: BlockCategory::Filter, description: String::new(), num_inputs: 1, num_outputs: 1, parameters: vec![] });
        let math = reg.list_category("Math");
        assert_eq!(math.len(), 2);
    }

    #[test]
    fn test_categories() {
        let reg = standard_registry();
        let cats = reg.categories();
        assert!(cats.contains(&"Source"));
        assert!(cats.contains(&"Math"));
    }

    #[test]
    fn test_search() {
        let reg = standard_registry();
        let results = reg.search("noise");
        assert!(!results.is_empty());
        assert!(results.iter().any(|b| b.name == "noise_source"));
    }

    #[test]
    fn test_unregister() {
        let mut reg = standard_registry();
        let count_before = reg.count();
        let removed = reg.unregister("signal_source");
        assert!(removed.is_some());
        assert_eq!(reg.count(), count_before - 1);
        assert!(reg.find("signal_source").is_none());
    }

    #[test]
    fn test_is_source_sink() {
        let info = BlockInfo { name: "src".into(), category: BlockCategory::Source, description: String::new(), num_inputs: 0, num_outputs: 1, parameters: vec![] };
        assert!(info.is_source());
        assert!(!info.is_sink());
        let sink = BlockInfo { name: "sink".into(), category: BlockCategory::Sink, description: String::new(), num_inputs: 1, num_outputs: 0, parameters: vec![] };
        assert!(sink.is_sink());
        assert!(!sink.is_source());
    }

    #[test]
    fn test_param_default() {
        let info = BlockInfo { name: "test".into(), category: BlockCategory::Utility, description: String::new(), num_inputs: 1, num_outputs: 1, parameters: vec![("freq".into(), "1000".into())] };
        assert_eq!(info.get_param_default("freq"), Some("1000"));
        assert_eq!(info.get_param_default("missing"), None);
    }

    #[test]
    fn test_standard_registry() {
        let reg = standard_registry();
        assert!(reg.count() >= 10);
        // Sources should exist.
        assert!(reg.find("signal_source").is_some());
        assert!(reg.find("noise_source").is_some());
    }

    #[test]
    fn test_custom_category() {
        let mut reg = BlockRegistry::new();
        reg.register(BlockInfo {
            name: "my_block".into(),
            category: BlockCategory::Custom("MyPlugin".into()),
            description: "Custom plugin block".into(),
            num_inputs: 1,
            num_outputs: 1,
            parameters: vec![],
        });
        let cats = reg.categories();
        assert!(cats.contains(&"MyPlugin"));
    }
}
