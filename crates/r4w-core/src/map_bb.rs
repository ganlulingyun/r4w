//! Map BB — General purpose symbol remapper (lookup table)
//!
//! Remaps input bytes through an arbitrary lookup table. Each input byte
//! is used as an index into the mapping table, producing the corresponding
//! output byte. Useful for symbol alphabet conversions, Gray code mapping,
//! bit manipulation, and custom encodings.
//! GNU Radio equivalent: `map_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::map_bb::MapBB;
//!
//! // Invert bits: 0→1, 1→0
//! let map = MapBB::from_pairs(&[(0, 1), (1, 0)]);
//! let input = vec![0u8, 1, 0, 1, 1];
//! let output = map.process(&input);
//! assert_eq!(output, vec![1, 0, 1, 0, 0]);
//!
//! // Gray code: 0→0, 1→1, 2→3, 3→2
//! let gray = MapBB::new(&[0, 1, 3, 2]);
//! assert_eq!(gray.process(&[0, 1, 2, 3]), vec![0, 1, 3, 2]);
//! ```

/// General-purpose byte-to-byte mapper using a lookup table.
#[derive(Debug, Clone)]
pub struct MapBB {
    /// Lookup table (256 entries, indexed by input byte).
    table: [u8; 256],
}

impl MapBB {
    /// Create from a slice of output values. Index = input, value = output.
    /// Entries beyond the slice length default to identity (passthrough).
    pub fn new(mapping: &[u8]) -> Self {
        let mut table = [0u8; 256];
        // Initialize to identity
        for i in 0..256 {
            table[i] = i as u8;
        }
        // Apply user mapping
        for (i, &v) in mapping.iter().enumerate() {
            if i < 256 {
                table[i] = v;
            }
        }
        Self { table }
    }

    /// Create from input→output pairs.
    pub fn from_pairs(pairs: &[(u8, u8)]) -> Self {
        let mut table = [0u8; 256];
        for i in 0..256 {
            table[i] = i as u8;
        }
        for &(input, output) in pairs {
            table[input as usize] = output;
        }
        Self { table }
    }

    /// Create from a full 256-entry table.
    pub fn from_table(table: [u8; 256]) -> Self {
        Self { table }
    }

    /// Create identity mapping (passthrough).
    pub fn identity() -> Self {
        let mut table = [0u8; 256];
        for i in 0..256 {
            table[i] = i as u8;
        }
        Self { table }
    }

    /// Create a bit inversion map (0↔1).
    pub fn bit_invert() -> Self {
        Self::from_pairs(&[(0, 1), (1, 0)])
    }

    /// Create a modular arithmetic map: output = (input + offset) % modulus.
    pub fn modular_add(offset: u8, modulus: u8) -> Self {
        let modulus = modulus.max(1);
        let mut table = [0u8; 256];
        for i in 0..256 {
            table[i] = ((i as u16 + offset as u16) % modulus as u16) as u8;
        }
        Self { table }
    }

    /// Process a single byte.
    #[inline]
    pub fn map(&self, input: u8) -> u8 {
        self.table[input as usize]
    }

    /// Process a block of bytes.
    pub fn process(&self, input: &[u8]) -> Vec<u8> {
        input.iter().map(|&x| self.table[x as usize]).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&self, data: &mut [u8]) {
        for x in data.iter_mut() {
            *x = self.table[*x as usize];
        }
    }

    /// Get the lookup table.
    pub fn table(&self) -> &[u8; 256] {
        &self.table
    }

    /// Create inverse mapping (if bijective). Returns None if not invertible.
    pub fn inverse(&self) -> Option<Self> {
        let mut inv = [0u8; 256];
        let mut used = [false; 256];
        for (i, &v) in self.table.iter().enumerate() {
            if used[v as usize] {
                return None; // Not bijective
            }
            used[v as usize] = true;
            inv[v as usize] = i as u8;
        }
        Some(Self { table: inv })
    }

    /// Compose two maps: output = other(self(input)).
    pub fn compose(&self, other: &MapBB) -> MapBB {
        let mut table = [0u8; 256];
        for i in 0..256 {
            table[i] = other.table[self.table[i] as usize];
        }
        MapBB { table }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let map = MapBB::identity();
        let input: Vec<u8> = (0..10).collect();
        assert_eq!(map.process(&input), input);
    }

    #[test]
    fn test_bit_invert() {
        let map = MapBB::bit_invert();
        assert_eq!(map.process(&[0, 1, 0, 1]), vec![1, 0, 1, 0]);
    }

    #[test]
    fn test_custom_mapping() {
        let map = MapBB::new(&[3, 2, 1, 0]); // Reverse 0-3
        assert_eq!(map.process(&[0, 1, 2, 3]), vec![3, 2, 1, 0]);
    }

    #[test]
    fn test_from_pairs() {
        let map = MapBB::from_pairs(&[(0, 10), (1, 20), (2, 30)]);
        assert_eq!(map.map(0), 10);
        assert_eq!(map.map(1), 20);
        assert_eq!(map.map(2), 30);
        assert_eq!(map.map(3), 3); // Identity for unmapped
    }

    #[test]
    fn test_modular_add() {
        let map = MapBB::modular_add(1, 4); // (x+1) % 4
        assert_eq!(map.process(&[0, 1, 2, 3]), vec![1, 2, 3, 0]);
    }

    #[test]
    fn test_gray_code() {
        let gray = MapBB::new(&[0, 1, 3, 2]);
        assert_eq!(gray.process(&[0, 1, 2, 3]), vec![0, 1, 3, 2]);
    }

    #[test]
    fn test_inverse() {
        let map = MapBB::new(&[2, 0, 1, 3]); // Permutation
        let inv = map.inverse().unwrap();
        // map(0)=2, so inv(2)=0
        assert_eq!(inv.map(2), 0);
        assert_eq!(inv.map(0), 1);
        assert_eq!(inv.map(1), 2);
    }

    #[test]
    fn test_inverse_not_bijective() {
        let map = MapBB::new(&[0, 0, 0]); // Not bijective (all map to 0)
        assert!(map.inverse().is_none());
    }

    #[test]
    fn test_compose() {
        let a = MapBB::from_pairs(&[(0, 1), (1, 2)]);
        let b = MapBB::from_pairs(&[(1, 10), (2, 20)]);
        let c = a.compose(&b);
        assert_eq!(c.map(0), 10); // a(0)=1, b(1)=10
        assert_eq!(c.map(1), 20); // a(1)=2, b(2)=20
    }

    #[test]
    fn test_inplace() {
        let map = MapBB::bit_invert();
        let mut data = vec![0, 1, 0, 1];
        map.process_inplace(&mut data);
        assert_eq!(data, vec![1, 0, 1, 0]);
    }

    #[test]
    fn test_empty() {
        let map = MapBB::identity();
        assert!(map.process(&[]).is_empty());
    }

    #[test]
    fn test_full_range() {
        let map = MapBB::identity();
        for i in 0u8..=255 {
            assert_eq!(map.map(i), i);
        }
    }

    #[test]
    fn test_roundtrip() {
        let map = MapBB::new(&[3, 2, 1, 0]);
        let inv = map.inverse().unwrap();
        let input: Vec<u8> = (0..4).collect();
        let encoded = map.process(&input);
        let decoded = inv.process(&encoded);
        assert_eq!(decoded, input);
    }
}
