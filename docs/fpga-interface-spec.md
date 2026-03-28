# FPGA Interface Specification — Tang Nano 9K

## Physical Connection

```
Sipeed 6+1          Tang Nano 9K           Raspberry Pi 4B
Mic Array            (FPGA)
┌─────────┐     ┌──────────────┐        ┌──────────────┐
│ 7x PDM  │────▶│ PDM pins     │        │              │
│ data     │     │ [26..32]     │        │  USB port    │
│          │◀────│ PDM clk [25] │        │              │
└─────────┘     │              │        │              │
                │  USB-C port  │════════│  USB port    │
                │  (BL616      │  USB   │  /dev/ttyACM0│
                │   bridge)    │ cable  │              │
                └──────────────┘        └──────────────┘
```

## Pin Assignments

| Signal | FPGA Pin | Direction | IO Type | Notes |
|--------|----------|-----------|---------|-------|
| clk_27m | 52 | Input | LVCMOS33 | Onboard 27 MHz oscillator |
| rst_n | 4 | Input | LVCMOS33 PULL_UP | Button S1 (active low) |
| pdm_clk | 25 | Output | LVCMOS33 DRIVE=8 | ~3 MHz clock to mic array |
| pdm_data[0] | 26 | Input | LVCMOS33 PULL_DOWN | Outer mic CH0 |
| pdm_data[1] | 27 | Input | LVCMOS33 PULL_DOWN | Outer mic CH1 |
| pdm_data[2] | 28 | Input | LVCMOS33 PULL_DOWN | Outer mic CH2 |
| pdm_data[3] | 29 | Input | LVCMOS33 PULL_DOWN | Outer mic CH3 |
| pdm_data[4] | 30 | Input | LVCMOS33 PULL_DOWN | Outer mic CH4 |
| pdm_data[5] | 31 | Input | LVCMOS33 PULL_DOWN | Outer mic CH5 |
| pdm_data[6] | 32 | Input | LVCMOS33 PULL_DOWN | Center mic CH6 |
| uart_tx | 17 | Output | LVCMOS33 DRIVE=8 | To BL616 USB bridge |

## Clock Architecture

```
27 MHz (crystal)
    │
    ├──▶ System clock (all logic)
    │
    └──▶ ÷9 ──▶ ~3 MHz PDM clock
                    │
                    └──▶ ÷192 (CIC) ──▶ 16 kHz PCM sample rate
                                            │
                                            └──▶ ÷32 (TDM batch) ──▶ 500 frames/sec
```

## TDM Frame Format

Each frame carries 32 sample sets (each set = 7 mu-law bytes):

```
Byte offset  Field           Size     Description
──────────────────────────────────────────────────────────
0x00         SYNC_HI         1 byte   0xAA
0x01         SYNC_LO         1 byte   0x55
0x02-0xE1    Payload         224 B    32 × 7 mu-law samples
0xE2         CRC16_HI        1 byte   CRC-16/CCITT MSB
0xE3         CRC16_LO        1 byte   CRC-16/CCITT LSB
──────────────────────────────────────────────────────────
Total: 228 bytes per frame
Frame rate: 500 frames/sec (16000 samples / 32 samples per frame)
Data rate: 114,000 bytes/sec = 912 kbit/sec
```

### Payload Layout (interleaved)

```
Sample 0:  [CH0] [CH1] [CH2] [CH3] [CH4] [CH5] [CH6]    ← 7 bytes
Sample 1:  [CH0] [CH1] [CH2] [CH3] [CH4] [CH5] [CH6]    ← 7 bytes
  ...
Sample 31: [CH0] [CH1] [CH2] [CH3] [CH4] [CH5] [CH6]    ← 7 bytes
                                                 Total: 224 bytes
```

Each `[CHx]` is a single mu-law encoded byte (8-bit, ITU G.711).

## Mu-Law Encoding (ITU G.711)

Compresses 16-bit signed linear PCM to 8-bit:

```
Input:  16-bit signed PCM [-32768 .. +32767]
Output: 8-bit mu-law [0x00 .. 0xFF]

Bit layout of encoded byte (after bitwise NOT):
  Bit 7:    Sign (1 = positive)
  Bits 6-4: Exponent (0-7)
  Bits 3-0: Mantissa

Bias: 0x84 (132) added before encoding
Dynamic range: ~72 dB in 8 bits
```

Decoding (Pi side): Use the standard ITU G.711 mu-law decode table (256 entries, uint8 → int16).

## CRC-16 Specification

- Algorithm: CRC-16/CCITT-FALSE
- Polynomial: 0x1021
- Initial value: 0xFFFF
- Final XOR: None
- Input/output reflection: None
- Scope: Covers sync bytes + entire payload (226 bytes)

## UART Configuration

| Parameter | Value |
|-----------|-------|
| Baud rate | 2,000,000 (2 Mbaud) |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| FPGA divider | 27 MHz / 14 ≈ 1.928 Mbaud (within tolerance) |

### Bandwidth Budget

```
Raw audio data:   7 ch × 16000 Hz × 1 byte = 112,000 B/s
Frame overhead:   500 frames × 4 bytes (sync+CRC) = 2,000 B/s
Total:            114,000 B/s
UART capacity:    ~193,000 B/s (1.928 Mbaud / 10 bits per byte)
Utilization:      59%
Headroom:         79,000 B/s (available for future metadata)
```

## Timing Diagram

```
                    Sample Period (62.5 µs @ 16 kHz)
                   ├────────────────────────────────┤

PDM_CLK    ________┌─┐_┌─┐_┌─┐_ ... _┌─┐_┌─┐________
                   │ │ │ │ │ │       │ │ │ │
PDM_DATA   ────────X─X─X─X─X─X─...──X─X─X─X─────────
                   192 PDM bits per PCM sample

CIC_VALID  ────────────────────────────────────┌┐─────
                                               ││
                                               └┘
SYNC_VALID ──────────────────────────────────────┌┐───
                                                 └┘

After 32 SYNC_VALID pulses:
TDM_FRAME  ──────[AA][55][32×7 mu-law bytes][CRC16]──
                 └────────── 228 bytes ──────────────┘
                 Transmitted over UART at 2 Mbaud
                 Frame duration: ~1.18 ms
                 Frame interval: 2.0 ms (500 Hz)
```

## Error Handling

**Pi-side frame parser behavior:**
- Missing sync: Scans byte stream for 0xAA55, discards preceding bytes
- CRC mismatch: Drops frame, increments error counter
- Partial frame: Buffers and waits for remaining bytes
- Buffer overflow: Drops oldest data to maintain real-time

**FPGA-side guarantees:**
- All 7 channels latched simultaneously (sample_sync module)
- CRC covers sync + payload — any bit error is detectable
- UART idle line is logic high — easy to distinguish from data
