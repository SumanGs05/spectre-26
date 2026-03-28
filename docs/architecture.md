# UAV-AudioLoc — System Architecture

## Overview

UAV-AudioLoc is a real-time acoustic sound source localization system designed to operate on autonomous drones during Search and Rescue (SAR) missions. It detects human distress sounds (screams, shouts, cries) masked by drone ego-noise and estimates their direction-of-arrival (DOA), outputting machine-consumable telemetry for autonomous navigation.

## Hardware Stack

| Tier | Component | Function |
|------|-----------|----------|
| Sensing | Sipeed 6+1 Mic Array | 7 MEMS microphones — 6 outer (circular, r=35mm) + 1 center. PDM output. |
| Bridge | Tang Nano 9K (Gowin GW1NR-9) | FPGA: PDM capture, CIC decimation (→16 kHz PCM), mu-law compression, batched TDM frames, UART TX |
| Processor | Raspberry Pi 4B | DSP + ML inference: spectral subtraction, RNNoise denoising, YAMNet classification, GCC-PHAT, DOA estimation |
| Output | Web Dashboard | Real-time monitoring via browser (served over Pi WiFi hotspot) |

## Signal Processing Pipeline

```
                         FPGA (Tang Nano 9K)
┌──────────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│ 7x MEMS PDM  │───▶│ PDM Cap │───▶│ CIC Dec │───▶│ Mu-Law  │───▶│ TDM Mux │
│ @ 3 MHz      │    │ per-ch  │    │ R=192   │    │ 16b→8b  │    │ 32 samp │
└──────────────┘    └─────────┘    └─────────┘    └─────────┘    └────┬────┘
                                                                      │ UART
                                                                      │ 2 Mbaud
                         Raspberry Pi 4B                              │ USB
┌──────────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌───▼──────┐
│  Telemetry   │◀───│   DOA   │◀───│GCC-PHAT │◀───│  YAMNet │◀───│ Spectral │
│  Bus (JSON)  │    │ Estimat │    │ 15 pairs│    │ Gate    │    │ Sub +    │
│              │    │ azimuth │    │ TDOA    │    │ human?  │    │ RNNoise  │
└──────┬───────┘    └─────────┘    └─────────┘    └─────────┘    └──────────┘
       │
  ┌────▼────┐    ┌──────────┐
  │ CSV Log │    │ WebSocket│───▶ Browser Dashboard
  └─────────┘    └──────────┘
```

## Processing Stages

### Stage 1: Spectral Subtraction (DSP)
- Removes predictable motor harmonic frequencies (fundamental + 8 harmonics)
- Wiener-like gain mask with configurable oversubtraction
- Cheap first pass — handles the deterministic component of ego-noise

### Stage 2: RNNoise Denoiser (ML)
- Mozilla/Xiph.org GRU-based model (~100 KB weights)
- Runs per-channel (7 instances) at 48 kHz internally
- Handles non-stationary broadband noise (RPM changes, gusts, turbulence)
- Replaces traditional Adaptive LMS — superior at non-stationary noise

### Stage 3: YAMNet Sound Classifier (ML)
- Google's pretrained AudioSet classifier (521 classes)
- TFLite model (~3.7 MB), runs on center mic (CH6) only
- Gates GCC-PHAT: only runs DOA when human sound detected
- Target classes: Speech, Shout, Screaming, Crying, Whimper, Groan, Yell
- Fail-open design: if model unavailable, treats all audio as potential human

### Stage 4: GCC-PHAT (DSP)
- Generalized Cross-Correlation with Phase Transform
- Computes TDOA for 15 mic pairs (C(6,2) from outer ring)
- Parabolic interpolation for sub-sample TDOA accuracy
- FFT-based — O(N log N) per pair

### Stage 5: DOA Estimation (DSP)
- Maps 15 TDOA measurements to single azimuth (0-360°, drone-nose-relative)
- Histogram fusion with Gaussian smoothing resolves front-back ambiguity
- Confidence = peak-to-total energy ratio in angular histogram
- Update rate: ~31 Hz (1024-sample FFT, 512-sample hop at 16 kHz)

## Telemetry Packet Format

Every processing cycle (~32 ms) produces a standardized JSON packet:

```json
{
    "timestamp_ms": 1711612800000,
    "azimuth_deg": 137.4,
    "confidence": 0.82,
    "snr_db": 12.5,
    "detection": true,
    "sound_class": "Shout",
    "class_confidence": 0.91,
    "frame_id": 48201
}
```

This packet is the **autonomy contract**: consumed identically by the web dashboard (Phase 1) and the future MAVLink flight controller module (Phase 2).

## Key Design Decisions

### Center mic as classifier input (not DOA)
The center mic (CH6) is equidistant from all propellers, capturing a balanced ego-noise signature. It serves as the YAMNet classifier input because it has no directional bias. The 6 outer mics are used exclusively for GCC-PHAT DOA.

### Mu-law compression for bandwidth
Raw 7-ch 16-bit at 16 kHz = 224 KB/s, exceeding the USB-UART bridge capacity (200 KB/s at 2 Mbaud). Mu-law compression (ITU G.711) halves this to 112 KB/s while preserving 72 dB dynamic range. GCC-PHAT depends on phase, not amplitude precision.

### GCC-PHAT over MUSIC/ESPRIT
GCC-PHAT is computationally cheaper (no eigendecomposition), more robust in non-anechoic outdoor environments, and sufficient for single-source azimuth estimation.

### Fail-open ML modules
Both the RNNoise denoiser and YAMNet classifier degrade gracefully: if libraries/models are unavailable, audio passes through unmodified and all sounds are treated as potential human sounds. The system never silently fails.

## Parameters (config.yaml)

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Sample rate | 16 kHz | Covers voice fundamentals (85-300 Hz) + harmonics to 8 kHz |
| FFT size | 1024 | 64 ms window, good frequency resolution |
| Hop size | 512 | 50% overlap, ~31 Hz DOA update rate |
| CIC decimation | R=192 | 3.072 MHz PDM → 16 kHz PCM |
| Array radius | 35 mm | Max unambiguous freq ~4.9 kHz |
| Classifier threshold | 0.3 | Balance between sensitivity and false alarms |
| Motor frequency | 200 Hz | Configurable — update from RPM telemetry |
