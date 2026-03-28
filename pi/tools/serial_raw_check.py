#!/usr/bin/env python3
"""
Raw serial smoke test — no frame parsing. Use when pc_audio_monitor shows raw_B/s=0.

  cd pi
  python tools/serial_raw_check.py COM3

Tries several baud rates; prints how many bytes arrived in 1 second and whether 0xAA 0x55 appears.
"""

from __future__ import annotations

import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

BAUDS = [1_500_000, 2_000_000, 1_928_571, 921_600, 115_200, 2_076_923, 1_800_000]


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    print(f"Port {port} — close Programmer/IDE first. Reading 1s per baud…\n")

    for baud in BAUDS:
        try:
            s = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.15)
            s.reset_input_buffer()
            t0 = time.perf_counter()
            acc = bytearray()
            while time.perf_counter() - t0 < 1.0:
                acc.extend(s.read(4096))
            s.close()
            n = len(acc)
            has_sync = b"\xaa\x55" in acc
            print(f"  baud={baud:>8}  bytes_1s={n:6}  AA55={'yes' if has_sync else 'no'}")
            if n > 0 and n <= 128:
                print(f"    hex: {acc.hex()}")
            elif n > 128:
                print(f"    first 64: {acc[:64].hex()}")
        except serial.SerialException as e:
            print(f"  baud={baud:>8}  ERROR: {e}")
        except OSError as e:
            print(f"  baud={baud:>8}  ERROR: {e}")

    print(
        "\nIf all bytes_1s=0: no data from cable/driver (wrong COM, charge-only USB, "
        "FPGA not sending uart_tx, or BL702 not in UART mode).\n"
        "If bytes at 115200 but not 2000000: try a different USB port / driver.\n"
        "If AA55=yes at some baud: use that baud in pc_audio_monitor.py --baud …"
    )


if __name__ == "__main__":
    main()
