# UAV-AudioLoc

Real-time acoustic sound source localization on an autonomous drone for Search and Rescue (SAR) missions.

## Problem

UAV ego-noise (motors/propellers) masks distress signals — human cries, shouts, screams — making acoustic detection nearly impossible during SAR operations in darkness, smoke, or fog where visual methods fail.

## Solution

A 3-tier hardware/software stack that captures multichannel audio, removes drone noise via DSP + ML, classifies human sounds, and estimates their direction in real time.

### Hardware Stack

| Tier | Component | Role |
|------|-----------|------|
| Sensing | Sipeed 6+1 Mic Array | 7 MEMS microphones in circular geometry, PDM output |
| Bridge | Tang Nano 9K (FPGA) | Synchronizes 7 PDM streams, CIC decimation, mu-law compression, UART output |
| Processor | Raspberry Pi 4B | DSP pipeline + ML inference + telemetry server |
| Output | Web Dashboard | Real-time monitoring for ground operators (served over WiFi) |

### Data Flow

```
Mic Array → [PDM] → FPGA → [USB-UART 2Mbaud] → Pi → [WebSocket] → Dashboard
                      ↓                          ↓
               CIC + mu-law              Spectral Sub → RNNoise
                                          → YAMNet → GCC-PHAT → DOA
```

## Project Structure

```
aurora/
├── docs/                    # Architecture and interface specifications
├── fpga/                    # Tang Nano 9K Verilog (Gowin)
│   ├── src/                 # RTL modules
│   ├── constraints/         # Pin assignments
│   └── sim/                 # Testbenches
├── pi/                      # Raspberry Pi 4B software
│   ├── capture/             # USB-UART frame capture
│   ├── dsp/                 # Signal processing (spectral sub, GCC-PHAT, DOA)
│   ├── ml/                  # ML models (RNNoise denoiser, YAMNet classifier)
│   ├── app/                 # Web dashboard (FastAPI + WebSocket)
│   └── tests/               # Unit tests
└── README.md
```

## Quick Start (Pi)

```bash
cd pi
pip install -r requirements.txt
python main.py                    # Starts capture + DSP + web dashboard
# Open http://<pi-ip>:8080 in browser
```

## Architecture

- **Phase 1 (current)**: Sensing → noise removal → sound classification → DOA estimation → logging + dashboard
- **Phase 2 (future)**: MAVLink integration for autonomous "search and turn" maneuvers

The system outputs machine-consumable telemetry packets at ~30 Hz with azimuth, confidence, and detected sound class — ready for autonomous flight controller integration.

See [docs/architecture.md](docs/architecture.md) for full technical details.
