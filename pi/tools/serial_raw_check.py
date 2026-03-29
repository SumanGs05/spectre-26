#!/usr/bin/env python3
"""
TODO(migration): This tool reads from a single serial port. For the ESP32-S3
dual-board architecture, run this once per port (ttyUSB0 and ttyUSB1) to verify
each board is sending data. The sync bytes (AA 55) and baud (1500000) are the
same, but the frame payload is int32 PCM not mu-law.

Raw serial smoke test — no frame parsing. Use when serial_merge.py shows no data.

  cd pi
  python tools/serial_raw_check.py /dev/ttyUSB0
  python tools/serial_raw_check.py /dev/ttyUSB1

Reads for 1 second at 1500000 baud; prints byte count and whether 0xAA 0x55 appears.
"""

from __future__ import annotations

import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pip install pyserial")

BAUD = 1500000


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    print(f"Port {port} @ {BAUD} — close Programmer/IDE first. Reading 1s…\n")

    try:
        s = serial.Serial(
            port=port,
            baudrate=BAUD,
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
        print(f"  baud={BAUD:>8}  bytes_1s={n:6}  AA55={'yes' if has_sync else 'no'}")
        if n > 0 and n <= 128:
            print(f"    hex: {acc.hex()}")
        elif n > 128:
            print(f"    first 64: {acc[:64].hex()}")
    except serial.SerialException as e:
        print(f"  baud={BAUD:>8}  ERROR: {e}")
    except OSError as e:
        print(f"  baud={BAUD:>8}  ERROR: {e}")

    print(
        "\nIf all bytes_1s=0: no data from cable/driver (wrong port, charge-only USB, "
        "ESP32-S3 not running, or USB not enumerated at 1500000).\n"
        "If AA55=no with bytes: baud/format mismatch vs ESP32-S3 firmware."
    )


if __name__ == "__main__":
    main()
