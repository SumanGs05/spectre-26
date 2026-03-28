# UAV-AudioLoc — Issues, Root Causes, and Fixes

Chronicle of hardware / FPGA / host bring-up problems solved while bringing **Tang Nano 9K → USB-UART → PC/Pi** online for **TDM mu-law frames** (`0xAA55` + 224 B payload + CRC16).

---

## 1. Symptom: No serial / zero bytes on COM

**Causes**

- Gowin **Programmer** or **IDE** still holding the USB device (COM port busy for Python).
- Wrong **COM** port; or **charge-only** USB cable.
- FPGA not programmed, or **`rst_n` held low** (S1 pressed).

**Fixes**

- Close Programmer/IDE before opening the port from Python or PuTTY.
- Confirm **COM** in Device Manager (often `USB Serial Port (COMx)`).
- Use **S1** only for reset (release for run); not S2 for `rst_n` on stock Nano 9K.

---

## 2. Symptom: Bytes on PC but never `AA55` / `frames_ok = 0`

**Root cause:** **UART baud mismatch.**

- System clock is **27 MHz**. “2,000,000 baud” with integer **CLKS_PER_BIT = 14** gives **27M/14 ≈ 1,928,571 baud** (~**3.6% error** vs 2,000,000 on the PC/BL702).
- That shifts sampling by **fractions of a bit** over a 10-bit character → **corrupted bytes** → sync `AA 55` almost never appears correctly.

**Fix**

- Use a baud that **divides evenly** from 27 MHz: **1,500,000** (= **27M / 18**), **0% rounding error**.
- Set **`UART_BAUD = 1_500_000`** (or equivalent) in FPGA `top` / `uart_tx`; set **`1500000`** in Python (`config.yaml`, `pc_audio_monitor`, `serial_raw_check`, `serial_reader` defaults).

**Throughput**

- TDM needs ~**114,000 B/s** (500 fps × 228 B). At **1.5 MBaud** 8N1, max ~**150,000 B/s** → enough headroom.

---

## 3. Symptom: I2S / sample rate wrong (audio timing)

**Root cause:** **Fractional NCO bug** in `i2s_receiver.v`.

- Old pattern did `acc <= acc + INC` and **in the same block**, on wrap, **`acc <= acc - MOD`** — the second NBA **overwrote** the first, **dropping one `INC` per wrap**.
- Effect: **~7% slow** bit-clock / sample rate vs intended **16 kHz**.

**Fix**

- **Add-then-check** with a wire: `acc_next = acc + INC`; if `acc_next >= MOD` then `acc <= acc_next - MOD` else `acc <= acc_next`.

---

## 4. Symptom: Gowin synthesis / behavior oddities on `top`

**Causes**

- **`generate`** around main audio path + **unpacked arrays** (`pcm_ch[0:6]`, `hold[]`, `mreg[]`) — some tool flows infer **RAM** or mis-optimize.
- **`EX3794` duplicate module** if `uart_hello.v` is in the project **twice** (e.g. copy under Gowin `Documents\...` and workspace).

**Fixes**

- Flatten `top.v`: **named regs** (`pcm_ch0..6`, `mreg0..6`, …), **`case`** writes for mu-law regs, **no** `generate` for normal vs debug.

Remove **`uart_hello.v`** from the **Gowin project** (or delete duplicate) when building **`top`**. Keep the file in repo for bring-up.

---

## 5. Symptom: PuTTY garbage at 115200 for `uart_hello`

**Cause**

- Session speed still **9600** (or wrong rate) while design sends **115200**.

**Fix**

- PuTTY **Speed = 115200** for `uart_hello` test.

---

## 6. Symptom: `serial_raw_check` shows `AA55=yes` @ 1.5M but **no** @ 2M

**Cause**

- FPGA line is **fixed** at **1.5M**; opening the PC side at **2M** **must** mis-decode. **Expected.**

**Fix**

- Use **one** host baud — **1,500,000** — matching the bitstream.

---

## 7. Symptom: `pc_audio_monitor` ~162 B then stall; `raw_B/s → 0`

**Cause**

- **Windows USB-CDC** + Python: **single-thread** `read()` competing with stats / **`sounddevice`** can stop delivering buffers after the first small packet.

**Fixes**

- **Background thread** dedicated to `ser.read()` → queue → main loop (same idea as Pi `SerialReader`).
- Avoid **flush** of RX by default (`--flush` optional).
- **`--stats-only`** to isolate serial without audio stack.

---

## 8. Constraint file corruption

**Symptom**

- `tangnano9k.cst` reduced to junk (e.g. accidental edit).

**Fix**

- Restore full CST: **clk 52**, **rst_n 4** `LVCMOS18`, **I2S 25–30**, **uart_tx 17**, **led_prog 10**.

---

## 9. Reference: minimal UART proof (`uart_hello.v`)

- Temporary **top module** sending **`HELLO\r\n`** at **115200** on **`uart_tx`**.
- Proves **pin 17** → **BL702** → **COM** without I2S/TDM.
- After success, set **top module back to `top`** for production build.

---

## Files touched (summary)

| Area | Files |
|------|--------|
| FPGA | `i2s_receiver.v`, `top.v`, `uart_tx.v`, `tangnano9k.cst`, `uart_debug_pump.v`, `uart_hello.v` |
| Pi | `config.yaml`, `capture/serial_reader.py`, `tools/pc_audio_monitor.py`, `tools/serial_raw_check.py`, `tools/serial_frame_probe.py`, `requirements-pc.txt` |

---

## Quick verification checklist

1. Program **`top`** @ **1.5M** UART; **close** Programmer; **unplug/replug** USB if COM acts weird.
2. `python tools/serial_raw_check.py COM3` → **`AA55=yes`** at **1500000**.
3. `python tools/pc_audio_monitor.py --port COM3 --stats-only` → **`raw_B/s`** high, **`frames_ok`** increasing.
4. Then run with playback (and `--device` if needed).

---

*Last updated: 2026-03-28*
