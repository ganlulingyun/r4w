# USRP Workshop

Hands-on exercises for testing R4W waveforms with Ettus Research USRP hardware.

## Supported Hardware

| Device | Interface | Bandwidth | Notes |
|--------|-----------|-----------|-------|
| N210 | Gigabit Ethernet | 40 MHz | Requires daughterboard (WBX, SBX, UBX) |
| B200 mini | USB 3.0 | 56 MHz | Compact, bus-powered |
| B200/B210 | USB 3.0 | 56 MHz | Full featured |

## Prerequisites

### 1. Install UHD

```bash
# Ubuntu/Debian
sudo apt install libuhd-dev uhd-host

# Download FPGA images
sudo uhd_images_downloader

# Verify installation
uhd_find_devices
```

### 2. Network Setup (N210 only)

```bash
# Set static IP on interface connected to N210
sudo ip addr add 192.168.10.1/24 dev eth0

# N210 default IP is 192.168.10.2
ping 192.168.10.2

# Verify device
uhd_find_devices --args="addr=192.168.10.2"
```

### 3. USB Setup (B200)

```bash
# Add udev rules (included with UHD)
sudo cp /usr/lib/uhd/utils/uhd-usrp.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify
uhd_find_devices
```

## Digital Attenuator Setup (Optional)

For controlled testing, connect a digital attenuator between TX and RX:

```
USRP TX ─────► Attenuator ─────► USRP RX
                  │
                  ▼
              Control
              (USB/SPI)
```

Supported attenuators:
- Mini-Circuits RCDAT series (USB)
- PE43711 (SPI)
- Simulated (for development without hardware)

## Workshop Exercises

### Getting Started

1. **01_device_discovery.rs** - Find and enumerate USRPs on your network
2. **02_basic_rx.rs** - Receive samples and display spectrum
3. **03_basic_tx.rs** - Transmit a test tone

### LoRa Testing

4. **04_loopback.rs** - TX→RX loopback with attenuator
5. **05_lora_tx.rs** - Transmit LoRa packets
6. **06_lora_rx.rs** - Receive and decode LoRa packets

### Advanced

7. **07_over_the_air.rs** - Full OTA link between two USRPs
8. **08_timing_sync.rs** - PPS and GPSDO synchronization
9. **09_sensitivity_test.rs** - Automated sensitivity measurement with attenuator

## Running Exercises

```bash
# With simulator (no hardware needed)
cargo run --example 01_device_discovery -- --simulator

# With real hardware
cargo run --example 01_device_discovery -- --device uhd://type=b200

# With N210
cargo run --example 01_device_discovery -- --device uhd://addr=192.168.10.2
```

## Configuration Files

- `configs/n210.yaml` - N210-specific settings
- `configs/b200_mini.yaml` - B200 mini settings
- `configs/loopback_test.yaml` - Loopback test configuration

## Troubleshooting

### Device Not Found

```bash
# Check USB (B200)
lsusb | grep Ettus

# Check network (N210)
ping 192.168.10.2
arp -a | grep 192.168.10
```

### Permission Denied

```bash
# Add user to plugdev group
sudo usermod -aG plugdev $USER
# Log out and back in
```

### Underrun/Overflow Errors

- Reduce sample rate
- Increase buffer size
- Check USB 3.0 port (not USB 2.0)
- For N210: Use Gigabit Ethernet, not 100 Mbps

### Clock Not Locking

```bash
# Check external reference
uhd_usrp_probe --args="addr=192.168.10.2"

# For GPSDO: wait for GPS lock (can take several minutes)
```

## Next Steps

After completing the workshop:
1. Explore the R4W waveform examples
2. Try different modulation schemes
3. Experiment with channel models
4. Contribute your own waveforms!
