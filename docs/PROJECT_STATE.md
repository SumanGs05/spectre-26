# UAV-AudioLoc — Full Project State (handoff document)

Use this file to onboard a new chat. It contains every critical decision,
discovery, known bug, and next step accumulated so far.

---

## 1. What this project is

Real-time acoustic sound source localization on an **autonomous drone** for
Search and Rescue. Detects human distress sounds (screams, shouts, cries),
estimates direction-of-arrival (DOA), and logs structured telemetry. The
drone has **no manual control** — all output must be machine-consumable
JSON for a future MAVLink autopilot (Phase 2).

## 2. Hardware stack

| Tier | Part | Role |
|------|------|------|
| Sensing | **Sipeed 6+1 Mic Array** | 7 MEMS mics (MSM261S4030H0R), circular, 35 mm radius |
| Bridge | **Tang Nano 9K** (Gowin GW1NR-9) | Captures mic data, packs into TDM frames, sends via USB-UART |
| Processor | **Raspberry Pi 4B** | DSP + ML inference + telemetry server |
| Output | **Web dashboard** (served from Pi WiFi) | Monitoring/logging for ground operators |

**Physical connection**: Tang Nano 9K → Pi via **USB cable** (onboard BL702 USB-UART bridge, up to 2 Mbaud).

---

## 3. CRITICAL DISCOVERY: Mic array is I2S, NOT PDM

**The entire FPGA capture layer must be rewritten.**

### What the Sipeed 6+1 actually outputs

The MSM261S4030H0R microphones use an **I2S interface**, not PDM.
The mic array board has a **10-pin 2x5 2.54mm header**:

| Pin | Name | Direction | Description |
|-----|------|-----------|-------------|
| 1 | VIN | Power | 3.3V or 5V input |
| 2 | GND | Power | Ground |
| 3 | MIC_D0 | Output | I2S data — mics 0 and 1 (L/R via WS) |
| 4 | MIC_D1 | Output | I2S data — mics 2 and 3 |
| 5 | MIC_D2 | Output | I2S data — mics 4 and 5 |
| 6 | MIC_D3 | Output | I2S data — center mic (mic 6) only |
| 7 | MIC_WS | Input | Word Select (sample rate clock, L/R select) |
| 8 | MIC_CK | Input | Serial clock (bit clock, ~3.072 MHz) |
| 9 | LED_CK | — | LED control (not needed) |
| 10 | LED_DA | — | LED data (not needed) |

### How I2S works on this array

- FPGA is the **master**: it generates `MIC_CK` (~3.072 MHz) and `MIC_WS` (toggles at sample rate).
- Each data line (D0–D3) carries **two mics** multiplexed by WS (left when WS=0, right when WS=1), except D3 which has only the center mic.
- Data is **MSB-first**, typically 24-bit or 16-bit per channel, clocked out on falling edge of CK, latched on rising edge.
- At 3.072 MHz CK and 32 bits per frame (16 per channel × 2 channels), WS toggles at **48 kHz**. For 16 kHz sample rate, use CK = 1.024 MHz (or 3.072 MHz with 96 bits per stereo frame and downsample).

### What this means for the FPGA code

| Current module | Status | What's needed instead |
|----------------|--------|-----------------------|
| `pdm_capture.v` | **WRONG** — assumes 1-bit PDM | Replace with `i2s_receiver.v` that deserializes I2S data |
| `cic_decimator.v` | **WRONG** — CIC filter for PDM→PCM | **Delete** — I2S already outputs multi-bit PCM directly |
| `sample_sync.v` | **Reusable** with minor changes | Still needed to latch all 7 channels simultaneously |
| `mulaw_encoder.v` | **OK** | No change — still takes 16-bit PCM, outputs 8-bit mu-law |
| `tdm_muxer.v` | **Has bugs** (see §5) | Fix UART backpressure, keep frame format |
| `uart_tx.v` | **OK** but unused `ready` | Wire `ready` into top-level flow control |
| `top.v` | **Must be rewritten** | New I2S master + receiver instantiation |

### New FPGA module needed: `i2s_receiver.v`

The FPGA must:
1. **Generate** `MIC_CK` (serial clock) and `MIC_WS` (word select) as outputs to the mic array.
2. **Receive** 4 data lines (MIC_D0–D3), deserialize MSB-first bits into 16-bit PCM words.
3. **Demux** L/R channels per data line using WS → gives 7 PCM channels total.
4. Output `pcm_valid` pulse when all 7 channels have a new sample.

---

## 4. Tang Nano 9K pin mapping (needs verification)

Current CST uses **placeholder pin numbers**. These must be verified against
the actual Tang Nano 9K board schematic for your specific revision.

### Known/confirmed

| Signal | CST pin | Confidence | Notes |
|--------|---------|------------|-------|
| `clk_27m` | 52 | HIGH | Standard for Tang Nano 9K |
| `rst_n` | 4 | MEDIUM | Typically button S1 |
| `uart_tx` | 17 | LOW | Must verify: which FPGA pin routes to BL702 UART RX |

### Must be assigned (6 signals to mic array)

| Signal | CST pin | Notes |
|--------|---------|-------|
| `i2s_ck` (was `pdm_clk`) | TBD | Output to mic array pin 8 (MIC_CK) |
| `i2s_ws` | TBD | Output to mic array pin 7 (MIC_WS) — NEW signal |
| `i2s_d0` (was `pdm_data[0,1]`) | TBD | Input from mic array pin 3 (mics 0+1) |
| `i2s_d1` (was `pdm_data[2,3]`) | TBD | Input from mic array pin 4 (mics 2+3) |
| `i2s_d2` (was `pdm_data[4,5]`) | TBD | Input from mic array pin 5 (mics 4+5) |
| `i2s_d3` (was `pdm_data[6]`) | TBD | Input from mic array pin 6 (center mic) |

Choose any 6 available GPIO pins on the Tang Nano 9K headers. Update CST accordingly.

### Wiring diagram

```
Sipeed 6+1 Mic Array          Tang Nano 9K              Raspberry Pi 4B
(10-pin 2.54mm header)        (FPGA board)
┌──────────────────┐       ┌────────────────┐         ┌──────────────┐
│ Pin 1  VIN ──────┼──────▶│ 3.3V           │         │              │
│ Pin 2  GND ──────┼──────▶│ GND            │         │              │
│ Pin 3  MIC_D0 ───┼──────▶│ GPIO (input)   │         │              │
│ Pin 4  MIC_D1 ───┼──────▶│ GPIO (input)   │         │              │
│ Pin 5  MIC_D2 ───┼──────▶│ GPIO (input)   │         │              │
│ Pin 6  MIC_D3 ───┼──────▶│ GPIO (input)   │         │              │
│ Pin 7  MIC_WS ◀──┼───────│ GPIO (output)  │         │              │
│ Pin 8  MIC_CK ◀──┼───────│ GPIO (output)  │         │              │
│ Pin 9  LED_CK    │ (NC)  │                │         │              │
│ Pin 10 LED_DA    │ (NC)  │ USB-C ─────────┼────USB──│ USB port     │
└──────────────────┘       │  (BL702 bridge  │  cable  │ /dev/ttyACM0 │
                           │   UART @ 2Mbaud)│         │              │
                           └────────────────┘         └──────────────┘
```

All signal wires should be **short** (< 15 cm) and ideally twisted with a ground return.

---

## 5. Known FPGA bugs (independent of I2S rewrite)

### Bug 1: No UART backpressure (CRITICAL)

`tdm_muxer.v` asserts `tx_valid` every clock cycle during frame send.
`uart_tx.v` exposes `ready` but **`top.v` never checks it**.
Result: bytes are lost, Pi sees garbage.

**Fix**: Gate `tx_valid` with `uart_ready`. Either:
- Add a small byte FIFO between muxer and UART, or
- Make the muxer FSM wait for `ready` before presenting each byte.

### Bug 2: Samples dropped during UART transmission

While the muxer is in `S_SEND_*` states, incoming `sample_valid` pulses
from the I2S receiver are **ignored**. At 16 kHz sample rate, sending a
228-byte frame at 2 Mbaud takes ~1.14 ms; a batch of 32 samples spans
2.0 ms. There is margin, but only if the muxer returns to `S_ACCUMULATE`
fast enough. If it doesn't, samples are dropped silently.

**Fix options**:
- A: Freeze upstream while sending (acceptable if timing margins hold).
- B: Ping-pong double buffer (CIC/I2S always writes buffer A or B; muxer sends the other).

---

## 6. TDM frame format (unchanged — Pi parser already matches)

```
| SYNC (0xAA55) | 32 × [CH0..CH6] mu-law bytes | CRC16 |
|   2 bytes     | 224 bytes                     | 2 bytes|
Total: 228 bytes per frame, 500 frames/sec, 114 KB/s
```

- CRC: CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, covers sync + payload
- Mu-law: ITU G.711, 16-bit PCM → 8-bit, ~72 dB dynamic range
- Bandwidth: 114 KB/s out of ~200 KB/s UART capacity (57% utilization)

Pi-side parser: `pi/capture/frame_parser.py` — already implements this exactly.

---

## 7. Pi software — current state

### Files that exist and pass tests (43/43 green)

```
pi/
├── capture/
│   ├── serial_reader.py      # Threaded USB-UART reader (pyserial, 2 Mbaud)
│   └── frame_parser.py       # Sync detect, CRC16, mu-law decode → 7ch int16
├── dsp/
│   ├── array_geometry.py     # Sipeed 6+1 mic positions, 15 pair enumeration
│   ├── gcc_phat.py           # GCC-PHAT with parabolic sub-sample interpolation
│   ├── doa_estimator.py      # TDOA→azimuth via histogram fusion (360° resolution)
│   ├── spectral_sub.py       # Motor harmonic removal (Wiener gain mask)
│   └── telemetry.py          # JSON packets, CSV log, WebSocket broadcast
├── ml/
│   ├── denoiser.py           # RNNoise ctypes wrapper (fail-safe passthrough)
│   ├── classifier.py         # YAMNet TFLite classifier (fail-open if no model)
│   └── model_utils.py        # Mel-spectrogram, TFLite loading, resampling
├── app/
│   ├── server.py             # FastAPI + WebSocket at :8080
│   └── static/               # Polar plot dashboard (HTML/JS/CSS, dark theme)
├── tests/
│   ├── test_gcc_phat.py      # TDOA + DOA accuracy on synthetic broadband signals
│   ├── test_denoiser.py      # Graceful degradation without librnnoise
│   ├── test_classifier.py    # Fail-open behavior, class selection logic
│   └── test_frame_parser.py  # CRC, sync recovery, partial frames, multi-frame
├── main.py                   # Pipeline orchestrator (all stages wired up)
├── config.yaml               # All tunable parameters
└── requirements.txt          # numpy scipy pyserial fastapi etc.
```

### Pipeline flow (in main.py)

```
Serial Capture → Frame Parse (mu-law decode)
    → Spectral Subtraction (motor harmonics)
    → RNNoise Denoiser (broadband, per-channel)
    → YAMNet Classifier (center mic gate)
    → GCC-PHAT (6 outer mics, 15 pairs)
    → DOA Estimator (histogram fusion → azimuth)
    → Telemetry Bus (JSON + CSV + WebSocket)
```

### ML models needed (not yet downloaded)

- **RNNoise**: `librnnoise.so` shared library — build from source or install package
- **YAMNet**: `yamnet.tflite` → place in `pi/models/yamnet.tflite`
- Both are **optional**: code degrades gracefully (passthrough / fail-open)

### Telemetry packet format

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

---

## 8. FPGA files — current state

```
fpga/
├── src/
│   ├── top.v              # WRONG — assumes PDM, must rewrite for I2S
│   ├── pdm_capture.v      # WRONG — delete or replace with i2s_receiver.v
│   ├── cic_decimator.v    # WRONG — not needed for I2S (already gives PCM)
│   ├── sample_sync.v      # OK — reusable (latch 7 channels)
│   ├── mulaw_encoder.v    # OK — 16-bit PCM → 8-bit mu-law (combinational)
│   ├── tdm_muxer.v        # BUG — no UART backpressure, needs fix
│   └── uart_tx.v          # OK — 2 Mbaud 8N1, has ready signal
└── constraints/
    └── tangnano9k.cst     # PLACEHOLDER pins — must verify against board
```

---

## 9. What to do next (in order)

### Phase A: Get FPGA streaming valid frames to Pi

1. **Verify Tang Nano 9K pin mapping** — check schematic for:
   - `clk_27m` (pin 52)
   - `uart_tx` to BL702 (claimed pin 17)
   - Choose 6 GPIO for I2S signals, update CST

2. **Write `i2s_receiver.v`** — I2S master that:
   - Generates CK (~3.072 MHz) and WS (toggles at 48 kHz or 16 kHz)
   - Receives 4 data lines, deserializes to 7 × 16-bit PCM
   - Asserts `pcm_valid` when all 7 channels ready

3. **Rewrite `top.v`** — replace PDM/CIC chain with I2S receiver

4. **Fix `tdm_muxer.v`** — gate bytes on `uart_tx.ready`

5. **Create Gowin project** — add all .v files, constraints, synthesize, place & route

6. **Flash and test** — connect mic array, USB to Pi, run frame sanity script

### Phase B: Prove Pi receives clean audio

7. **Write `pi/tools/serial_frame_sanity.py`** — minimal script that:
   - Opens serial port, feeds `FrameParser`
   - Prints FPS, drop rate, sync losses
   - Optionally dumps CH6 to .wav for listening

8. **Bench test**: Mic array + Tang Nano + Pi on table, play sounds from phone speaker

### Phase C: Full DSP + ML (deferred)

9. Download/build RNNoise and YAMNet models
10. Tune spectral subtraction for actual motor frequencies
11. Run full `main.py` pipeline with real audio
12. Web dashboard live testing

---

## 10. Key design decisions (for reference)

- **Mu-law compression**: Required because 7ch × 16kHz × 16bit = 224 KB/s exceeds USB-UART capacity (~200 KB/s). Mu-law halves to 112 KB/s. Phase unaffected for GCC-PHAT.
- **Center mic (CH6) as classifier input**: Equidistant from propellers, no directional bias. Only outer 6 mics used for DOA.
- **ML replaces LMS**: Drone noise is non-stationary. RNNoise + spectral subtraction beats adaptive LMS.
- **Fail-open ML**: If models missing, system still runs — treats all audio as potential human sound.
- **Autonomous-first telemetry**: JSON packet is the contract for Phase 2 MAVLink integration.
- **Web dashboard**: Monitoring tool served over Pi WiFi hotspot, not a pilot control interface.

---

## 11. Test results

All 43 Pi-side tests pass:
- 8 classifier tests (fail-open, class selection)
- 8 denoiser tests (graceful degradation)
- 10 frame parser tests (CRC, sync, partial frames)
- 12 GCC-PHAT tests (TDOA accuracy, DOA at cardinal directions)
- 5 array geometry tests (mic count, radius, pair count, max freq)

Run: `cd pi && python -m pytest tests/ -v`
