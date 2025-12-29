//! FPGA partition isolation for hardware-enforced waveform separation.
//!
//! This module provides tools for isolating waveforms using FPGA
//! hardware mechanisms like AXI firewalls and partial reconfiguration.
//!
//! See [ISOLATION_GUIDE.md](../../../docs/ISOLATION_GUIDE.md) for details.

use crate::error::{Result, SandboxError};

/// Configuration for an FPGA isolation partition.
#[derive(Debug, Clone)]
pub struct FpgaPartitionConfig {
    /// Partition identifier
    pub partition_id: u32,
    /// Base address of the partition's AXI region
    pub base_address: u64,
    /// Size of the partition's address space
    pub address_size: u64,
    /// Classification level for this partition
    pub classification: String,
    /// Whether AXI firewall is enabled
    pub firewall_enabled: bool,
    /// Allowed AXI master IDs
    pub allowed_masters: Vec<u32>,
}

impl Default for FpgaPartitionConfig {
    fn default() -> Self {
        Self {
            partition_id: 0,
            base_address: 0,
            address_size: 0x10000, // 64KB default
            classification: "UNCLASSIFIED".to_string(),
            firewall_enabled: true,
            allowed_masters: vec![],
        }
    }
}

/// An isolated FPGA partition for waveform processing.
///
/// Each partition has its own address space isolated by the AXI firewall,
/// preventing unauthorized access between waveforms.
pub struct FpgaPartition {
    config: FpgaPartitionConfig,
    #[allow(dead_code)]
    state: PartitionState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PartitionState {
    Uninitialized,
    Configured,
    Active,
    Locked,
}

impl FpgaPartition {
    /// Create a new FPGA partition with the given configuration.
    pub fn new(config: FpgaPartitionConfig) -> Result<Self> {
        Ok(Self {
            config,
            state: PartitionState::Uninitialized,
        })
    }

    /// Get the partition configuration.
    pub fn config(&self) -> &FpgaPartitionConfig {
        &self.config
    }

    /// Configure the AXI firewall for this partition.
    ///
    /// This sets up hardware protection to prevent unauthorized
    /// access from other partitions.
    pub fn configure_firewall(&mut self) -> Result<()> {
        if !self.config.firewall_enabled {
            return Ok(());
        }

        // In a real implementation, this would:
        // 1. Write to AXI firewall configuration registers
        // 2. Set up address range protection
        // 3. Configure allowed master IDs
        // 4. Enable the firewall

        tracing::info!(
            "Configuring AXI firewall for partition {} at 0x{:X}",
            self.config.partition_id,
            self.config.base_address
        );

        self.state = PartitionState::Configured;
        Ok(())
    }

    /// Activate the partition for waveform processing.
    pub fn activate(&mut self) -> Result<()> {
        if self.state != PartitionState::Configured {
            return Err(SandboxError::FpgaError(
                "partition must be configured before activation".to_string(),
            ));
        }

        tracing::info!("Activating FPGA partition {}", self.config.partition_id);
        self.state = PartitionState::Active;
        Ok(())
    }

    /// Lock the partition, preventing further configuration changes.
    pub fn lock(&mut self) -> Result<()> {
        if self.state != PartitionState::Active {
            return Err(SandboxError::FpgaError(
                "partition must be active before locking".to_string(),
            ));
        }

        tracing::info!("Locking FPGA partition {}", self.config.partition_id);
        self.state = PartitionState::Locked;
        Ok(())
    }

    /// Check if the partition is active.
    pub fn is_active(&self) -> bool {
        matches!(
            self.state,
            PartitionState::Active | PartitionState::Locked
        )
    }

    /// Check if the partition is locked.
    pub fn is_locked(&self) -> bool {
        self.state == PartitionState::Locked
    }
}

/// Manager for multiple FPGA partitions.
pub struct FpgaPartitionManager {
    partitions: Vec<FpgaPartition>,
}

impl FpgaPartitionManager {
    /// Create a new partition manager.
    pub fn new() -> Self {
        Self {
            partitions: Vec::new(),
        }
    }

    /// Add a partition to the manager.
    pub fn add_partition(&mut self, config: FpgaPartitionConfig) -> Result<usize> {
        // Check for overlapping address ranges
        for existing in &self.partitions {
            let new_end = config.base_address + config.address_size;
            let existing_end = existing.config.base_address + existing.config.address_size;

            if config.base_address < existing_end && new_end > existing.config.base_address {
                return Err(SandboxError::FpgaError(format!(
                    "partition {} overlaps with existing partition {}",
                    config.partition_id, existing.config.partition_id
                )));
            }
        }

        let partition = FpgaPartition::new(config)?;
        self.partitions.push(partition);
        Ok(self.partitions.len() - 1)
    }

    /// Get a partition by index.
    pub fn get(&self, index: usize) -> Option<&FpgaPartition> {
        self.partitions.get(index)
    }

    /// Get a mutable partition by index.
    pub fn get_mut(&mut self, index: usize) -> Option<&mut FpgaPartition> {
        self.partitions.get_mut(index)
    }

    /// Get the number of partitions.
    pub fn len(&self) -> usize {
        self.partitions.len()
    }

    /// Check if there are no partitions.
    pub fn is_empty(&self) -> bool {
        self.partitions.is_empty()
    }

    /// Configure all partitions.
    pub fn configure_all(&mut self) -> Result<()> {
        for partition in &mut self.partitions {
            partition.configure_firewall()?;
        }
        Ok(())
    }

    /// Activate all partitions.
    pub fn activate_all(&mut self) -> Result<()> {
        for partition in &mut self.partitions {
            partition.activate()?;
        }
        Ok(())
    }
}

impl Default for FpgaPartitionManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_partition_lifecycle() {
        let config = FpgaPartitionConfig {
            partition_id: 0,
            base_address: 0x40000000,
            address_size: 0x10000,
            classification: "UNCLASSIFIED".to_string(),
            firewall_enabled: true,
            allowed_masters: vec![0, 1],
        };

        let mut partition = FpgaPartition::new(config).unwrap();
        assert!(!partition.is_active());

        partition.configure_firewall().unwrap();
        partition.activate().unwrap();
        assert!(partition.is_active());

        partition.lock().unwrap();
        assert!(partition.is_locked());
    }

    #[test]
    fn test_partition_manager_overlap_detection() {
        let mut manager = FpgaPartitionManager::new();

        // Add first partition
        manager
            .add_partition(FpgaPartitionConfig {
                partition_id: 0,
                base_address: 0x40000000,
                address_size: 0x10000,
                ..Default::default()
            })
            .unwrap();

        // Try to add overlapping partition - should fail
        let result = manager.add_partition(FpgaPartitionConfig {
            partition_id: 1,
            base_address: 0x40008000, // Overlaps!
            address_size: 0x10000,
            ..Default::default()
        });

        assert!(result.is_err());

        // Add non-overlapping partition - should succeed
        manager
            .add_partition(FpgaPartitionConfig {
                partition_id: 1,
                base_address: 0x40010000, // After first partition
                address_size: 0x10000,
                ..Default::default()
            })
            .unwrap();

        assert_eq!(manager.len(), 2);
    }
}
