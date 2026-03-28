#!/usr/bin/env python3
"""
One-shot: read a chunk from COM, run FrameParser, count raw AA55 pairs.

Use when pc_audio_monitor shows bytes but frames_ok=0.

  cd pi
  python tools/serial_frame_probe.py COM3 2000000
  python tools/serial_frame_probe.py COM3 1928571
  python tools/serial_frame_probe.py COM3 2000000 --seconds 15 --no-flush
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PI_ROOT = Path(__file__).resolve().parent.parent
if str(PI_ROOT) not in sys.path:
    sys.path.insert(0, str(PI_ROOT))

try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

from capture.frame_parser import FrameParser  # noqa: E402


def count_aa55(data: bytes) -> int:
    return sum(1 for i in range(len(data) - 1) if data[i] == 0xAA and data[i + 1] == 0x55)


def main() -> None:
    ap = argparse.ArgumentParser(description="Serial TDM frame probe")
    ap.add_argument("port", nargs="?", default="COM3", help="COM port")
    ap.add_argument("baud", nargs="?", type=int, default=1_500_000, help="Baud rate")
    ap.add_argument(
        "--seconds",
        type=float,
        default=12.0,
        help="How long to read (default 12; use longer if stream is slow)",
    )
    ap.add_argument(
        "--no-flush",
        action="store_true",
        help="Do not clear RX buffer after open (keeps any bytes already queued)",
    )
    args = ap.parse_args()

    port, baud = args.port, args.baud
    print(
        f"Port {port} @ {baud} — close Gowin Programmer/IDE first.\n"
        f"Reading up to 300 KiB or {args.seconds:.0f} s…"
    )

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    time.sleep(0.25)
    if not args.no_flush:
        try:
            ser.reset_input_buffer()
        except (AttributeError, serial.SerialException):
            pass

    data = bytearray()
    t0 = time.perf_counter()
    while len(data) < 300_000 and (time.perf_counter() - t0) < args.seconds:
        chunk = ser.read(16384)
        if chunk:
            data.extend(chunk)
    ser.close()

    raw = bytes(data)
    print(f"Got {len(raw)} bytes in {time.perf_counter() - t0:.1f} s")

    if len(raw) == 0:
        print(
            "\nZero bytes — another app may hold the port, USB not enumerating, or FPGA not sending.\n"
            "  • Quit Programmer / PuTTY / other serial tools\n"
            "  • Unplug USB 5s, replug, confirm COM in Device Manager\n"
            "  • FPGA LED should blink; reprogram if needed\n"
            "  • Retry with:  python tools/serial_frame_probe.py COM3 2000000 --no-flush"
        )
        raise SystemExit(1)

    if len(raw) < 228:
        print(
            f"\nShort capture ({len(raw)} B) — still analyzing. "
            "If pc_audio_monitor gets ~10k B/s but this script gets almost nothing, "
            "try longer read:  --seconds 30   or   --no-flush"
        )
        preview = raw[: min(128, len(raw))]
        print(f"First bytes hex: {preview.hex()}")

    print(f"Raw AA55 pairs (anywhere): {count_aa55(raw)}")
    p = FrameParser()
    p.feed(raw)
    st = p.stats
    print(
        f"Parser: frames_ok={st['frames_parsed']}  "
        f"frames_dropped={st['frames_dropped']}  "
        f"sync_losses={st['sync_losses']}  leftover_buf_B={st['buffer_bytes']}"
    )

    if st["frames_parsed"] == 0 and count_aa55(raw) == 0:
        print(
            "\nNo AA55 in this capture → wrong baud for BL702/Windows, DEBUG_UART_BLIND only, "
            "or noise. Try bauds 2000000, 1928571, 2076923, then FPGA  parameter UART_BAUD = 921_600;  "
            "and probe @ 921600."
        )
    elif st["frames_parsed"] == 0 and st["frames_dropped"] > 0:
        print(
            "\nAA55 seen but CRC fails → baud marginal or format mismatch. "
            "Try other --baud; rebuild FPGA after uart_tx / top changes."
        )
    elif st["frames_parsed"] == 0 and count_aa55(raw) > 0 and st["frames_dropped"] == 0:
        print(
            "\nAA55 present but no full valid frame — run with --seconds 30 for a longer capture."
        )
    else:
        print("\nFrames decode OK at this baud — use the same --baud in pc_audio_monitor.py.")


if __name__ == "__main__":
    main()
