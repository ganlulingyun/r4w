# Future Channel Effects & Signal Impairments

Ideas for enhancing the GNSS scenario generator with more realistic signal effects.
Captured 2026-02-07 during E1C signal generation/validation session.

## Phase Noise (Oscillator Model)

- **Receiver LO phase noise**: Random walk or filtered Gaussian process on carrier phase accumulator
- **Satellite clock phase noise**: Allan deviation model (typ. 1e-12 for Rb, 1e-14 for H-maser)
- Parameterize via L(f) phase noise mask or Allan deviation at specified tau
- Appears as close-in spectral skirts around the carrier
- Implementation: add phase jitter term to Doppler phase accumulator in `generate_block()`

## Ionospheric Scintillation

- **Phase scintillation**: rapid phase fluctuations from ionospheric irregularities
- **Amplitude scintillation**: signal fading from diffraction through ionospheric structures
- S4 index (amplitude) and sigma-phi (phase) parameterization
- More severe at low latitudes (equatorial) and high latitudes (auroral)
- Frequency-dependent: worse at lower frequencies (L-band affected more than L5)

## Multipath Fading (Dynamic)

- Current multipath_preset/multipath_enabled infrastructure exists but doesn't apply time-varying fading
- Need per-sample or per-block amplitude/phase modulation from multipath channel model
- Jake's/Clarke's fading already implemented in r4w-sim (`channel.rs`) — could be wired into GNSS scenario
- TDL (Tapped Delay Line) multipath profiles (EPA/EVA/ETU) also available in r4w-sim

## Time-Varying Doppler & Dynamics

- Over 60 seconds, MEO satellite Doppler only changes ~30-60 Hz (orbital period ~14 hours)
- For more dramatic time variation: model receiver dynamics (walking, driving, high-G)
- Receiver trajectory model: waypoints with velocity/acceleration profiles
- Could reuse r4w-sim trajectory framework

## Interference

- CW interference (narrowband jammers)
- Wideband interference (swept jammers, DME/TACAN near L5)
- Cross-correlation interference from other GNSS signals/PRNs
- Spoofing signals (for anti-spoofing testing)

## Antenna Effects

- Antenna phase center variation (PCV) vs elevation/azimuth
- Polarization mismatch (RHCP with multipath producing LHCP reflections)
- Near-field effects and mutual coupling (for antenna arrays)

## Implementation Priority (suggested)

1. Oscillator phase noise — relatively simple, high impact on tracking loop testing
2. Dynamic multipath fading — wire existing r4w-sim channel models into scenario generator
3. Receiver trajectory/dynamics — enable realistic driving/walking scenarios
4. Ionospheric scintillation — important for high-integrity applications
5. Interference — important for robustness testing
