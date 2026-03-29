# UAV-AudioLoc

Real-time acoustic sound source localization on an autonomous drone for Search and Rescue (SAR) missions.

## Problem

UAV ego-noise (motors/propellers) masks distress signals — human cries, shouts, screams — making acoustic detection nearly impossible during SAR operations in darkness, smoke, or fog where visual methods fail.

## Solution

A 3-tier hardware/software stack that captures multichannel audio, removes drone noise via DSP + ML, classifies human sounds, and estimates their direction in real time.

### Hardware Stack

| Tier | Component | Role |
|------|-----------|------|
| Sensing | Sipeed 6+1 Mic Array | 7 MEMS microphones in circular geometry, I2S output |
| Bridge | 2× Seeed XIAO ESP32-S3 | Captures 4 I2S data lines, runs LMS + spectral subtraction, sends filtered PCM over UART |
| Processor | Raspberry Pi 4B | DSP pipeline + ML inference + telemetry server |
| Output | Web Dashboard | Real-time monitoring for ground operators (served over WiFi) |

### Data Flow

```
Mic Array → [I2S] → ESP32-S3 ×2 → [UART 1.5Mbaud ×2] → Pi → [WebSocket] → Dashboard
                        ↓                                  ↓
                  LMS + Spectral Sub                Spectral Sub → RNNoise
                  (on-chip filtering)               → YAMNet → GCC-PHAT → DOA
```

## ESP32-S3 Dual-Board Architecture

The FPGA (Tang Nano 9K) has been replaced by two Seeed XIAO ESP32-S3 boards that split the four I2S data lines from the Sipeed 6+1 mic array.

### Board Assignments

| Board | board_id | I2S Lines | Mics | Channels | Role |
|-------|----------|-----------|------|----------|------|
| Master | 0x01 | MIC_D0, MIC_D1 | mic0(L), mic1(R), mic2(L), mic3(R) | 4 | I2S clock master, sync pulse output |
| Slave  | 0x02 | MIC_D2, MIC_D3 | mic4(L), mic5(R), mic6(L centre) | 3 | I2S clock slave, sync pulse input |

### Wiring Table

| Signal | Master Pin | Slave Pin | Notes |
|--------|-----------|-----------|-------|
| MIC_CK (BCLK) | D7 (GPIO7) | D7 (GPIO7) | Shared clock, Master drives |
| MIC_WS (LRCLK) | D8 (GPIO8) | D8 (GPIO8) | Shared word select |
| MIC_D0 | D9 (GPIO9) | — | mic0(L) / mic1(R) |
| MIC_D1 | D10 (GPIO10) | — | mic2(L) / mic3(R) |
| MIC_D2 | — | D9 (GPIO9) | mic4(L) / mic5(R) |
| MIC_D3 | — | D10 (GPIO10) | mic6(L) centre / empty(R) |
| SYNC | D2 (GPIO2) OUT | D2 (GPIO2) IN | Rising edge triggers Slave capture |
| UART TX | D43 (GPIO43) | D43 (GPIO43) | → Pi /dev/ttyUSB0 (Master), /dev/ttyUSB1 (Slave) |
| GND | Common | Common | |
| VIN | Shared rail | Shared rail | |

### Frame Format (both boards)

```
[0xAA][0x55][board_id][ch_count][interleaved int32 LE samples][CRC16]
  1B    1B      1B        1B       ch×256×4 bytes               2B
```

- Master: 4 + 4×256×4 + 2 = **4102 bytes** per frame
- Slave:  4 + 3×256×4 + 2 = **3078 bytes** per frame
- CRC16-CCITT (poly 0x1021, init 0xFFFF) computed over **payload only**
- Baud rate: **1,500,000**
- Sample rate: **22050 Hz**, 32-bit PCM, DMA buffer 256 samples

### On-Chip Filtering

Both boards run before UART transmission:

1. **NLMS (Normalized LMS)** — adaptive ego-noise cancellation using a reference mic
   - Master: mic2 as reference for mic0, mic1, mic3
   - Slave: mic6 (centre) as reference for mic4, mic5
2. **Spectral Subtraction** — hover-harmonic suppression (Goertzel at 180 Hz fundamental)

### Running the Pi Merger

```bash
cd pi

# Option 1: standalone merger (for testing)
python3 ../esp32/serial_merge.py --port0 /dev/ttyUSB0 --port1 /dev/ttyUSB1 --stats

# Option 2: full pipeline (uses DualSerialReader in capture/serial_reader.py)
python3 main.py
```

## Project Structure

```
aurora/
├── docs/                    # Architecture and interface specifications
├── esp32/                   # ESP32-S3 firmware and Pi merger
│   ├── esp32s3_master.ino   # Master board firmware (Arduino/ESP-IDF)
│   ├── esp32s3_slave.ino    # Slave board firmware
│   └── serial_merge.py      # Pi-side dual UART reassembly (standalone)
├── fpga_archive/            # Legacy Tang Nano 9K Verilog (archived)
│   └── fpga/
│       ├── src/             # RTL modules
│       └── constraints/     # Pin assignments
├── pi/                      # Raspberry Pi 4B software
│   ├── capture/             # Dual-port serial reader + frame parsing
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

Ensure both ESP32-S3 boards are connected and visible as `/dev/ttyUSB0` and `/dev/ttyUSB1`. Edit `pi/config.yaml` if your port names differ.

## Architecture

- **Phase 1 (current)**: Sensing → on-chip noise removal → Pi-side DSP → sound classification → DOA estimation → logging + dashboard
- **Phase 2 (future)**: MAVLink integration for autonomous "search and turn" maneuvers

The system outputs machine-consumable telemetry packets at ~30 Hz with azimuth, confidence, and detected sound class — ready for autonomous flight controller integration.

See [docs/architecture.md](docs/architecture.md) for full technical details.
