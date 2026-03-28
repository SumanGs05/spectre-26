#!/usr/bin/env python3
"""
Play FPGA TDM audio on a PC (Windows COMx or Linux /dev/ttyACM0).

Uses the same frame format as pi/capture/frame_parser.py (sync 0xAA55,
mu-law, CRC16). Close RealTerm / PuTTY first — only one app can open the port.

Serial data is read on a background thread so playback / main thread work cannot
starve the USB stack (fixes Windows CDC stalls after one small read).

Examples:
  cd pi
  pip install pyserial numpy sounddevice
  python tools/pc_audio_monitor.py --port COM3
  python tools/pc_audio_monitor.py --port COM3 --stats-only
  python tools/pc_audio_monitor.py --port COM3 --list-devices
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

# Repo root: .../pi
PI_ROOT = Path(__file__).resolve().parent.parent
if str(PI_ROOT) not in sys.path:
    sys.path.insert(0, str(PI_ROOT))

from capture.frame_parser import FrameParser  # noqa: E402

SAMPLE_RATE = 16_000
READ_Q_MAX = 512

try:
    import sounddevice as sd
except ImportError:
    sd = None

try:
    import serial
except ImportError as e:
    raise SystemExit("Install pyserial: pip install pyserial") from e


def main() -> None:
    p = argparse.ArgumentParser(description="Play 7ch TDM from Tang Nano UART")
    p.add_argument(
        "--port",
        default="COM3",
        help="Serial port (Windows: COM3, Linux: /dev/ttyACM0)",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=1_500_000,
        help="Baud rate (default 1500000; exact from 27 MHz FPGA clock)",
    )
    p.add_argument(
        "--channel",
        type=int,
        default=6,
        choices=range(7),
        help="Mic channel 0-6 (default 6 = center)",
    )
    p.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Stop after N seconds (0 = run until Ctrl+C)",
    )
    p.add_argument(
        "--wav",
        type=Path,
        default=None,
        help="If set, also write this WAV file (16 kHz mono int16)",
    )
    p.add_argument(
        "--list-devices",
        action="store_true",
        help="List audio output devices (find your headphones index) and exit",
    )
    p.add_argument(
        "--device",
        type=int,
        default=None,
        metavar="N",
        help="Play to this sounddevice output index (see --list-devices)",
    )
    p.add_argument(
        "--flush",
        action="store_true",
        help="Clear RX buffer after open (often bad on Windows CDC; default: off)",
    )
    p.add_argument(
        "--stats-only",
        action="store_true",
        help="No audio playback — only parse/print stats (no sounddevice needed)",
    )
    args = p.parse_args()

    if args.list_devices:
        if sd is None:
            raise SystemExit("Install sounddevice: pip install sounddevice")
        print(sd.query_devices())
        print(f"\nDefault output device index: {sd.default.device[1]}")
        raise SystemExit(0)

    if not args.stats_only and sd is None and args.wav is None:
        raise SystemExit(
            "Install sounddevice for live playback: pip install sounddevice\n"
            "Or use --stats-only or --wav out.wav without sounddevice."
        )

    parser = FrameParser()
    wav_chunks: list[np.ndarray] = []

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.05,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    time.sleep(0.15)
    if args.flush:
        try:
            ser.reset_input_buffer()
        except (AttributeError, serial.SerialException):
            pass

    read_q: queue.Queue[bytes] = queue.Queue(maxsize=READ_Q_MAX)
    stop_reader = threading.Event()

    def _reader_loop() -> None:
        while not stop_reader.is_set():
            try:
                block = ser.read(16384)
                if not block:
                    continue
                try:
                    read_q.put_nowait(block)
                except queue.Full:
                    try:
                        read_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        read_q.put_nowait(block)
                    except queue.Full:
                        pass
            except (serial.SerialException, OSError):
                break

    reader_thread = threading.Thread(
        target=_reader_loop, name="pc-audio-serial", daemon=True
    )
    reader_thread.start()

    print(f"Opened {args.port} @ {args.baud}. Channel {args.channel}. Ctrl+C to stop.")
    print("(Serial read runs on a background thread.)")
    if args.stats_only:
        print("Stats-only mode — no playback.")
    elif sd:
        if args.device is not None:
            print(f"Playing on audio device index {args.device} (when frames decode)…")
        else:
            print(
                "Playing on Windows default output (when frames decode). "
                "If you hear nothing, run: python tools/pc_audio_monitor.py --list-devices"
            )
    print(
        "Status every 1s: raw_bytes/s, frames_ok, dropped, sync_losses, parser_buf_B\n"
        "  If stall: unplug USB ~5s, replug; close Programmer/PuTTY; try --stats-only first.\n"
    )
    sys.stdout.flush()

    t0 = time.monotonic()
    last_stat = t0
    bytes_rx_window = 0
    bytes_rx_total = 0
    warned_stall_decode = False

    try:
        while True:
            if args.seconds > 0 and (time.monotonic() - t0) >= args.seconds:
                break

            try:
                chunk = read_q.get(timeout=0.1)
            except queue.Empty:
                chunk = b""

            if chunk:
                bytes_rx_window += len(chunk)
                bytes_rx_total += len(chunk)
                audio = parser.feed(chunk)
                if audio is not None and not args.stats_only and sd is not None:
                    mono = audio[args.channel, :].astype(np.float32) / 32768.0
                    if args.device is not None:
                        sd.play(mono, SAMPLE_RATE, device=args.device, blocking=True)
                    else:
                        sd.play(mono, SAMPLE_RATE, blocking=True)
                if audio is not None and args.wav is not None:
                    wav_chunks.append(audio[args.channel, :].astype(np.int16))

            now = time.monotonic()
            if now - last_stat >= 1.0:
                st = parser.stats
                bps = bytes_rx_window
                bytes_rx_window = 0
                print(
                    f"  raw_B/s={bps:6d}  total_B={bytes_rx_total:8d}  "
                    f"frames_ok={st['frames_parsed']:5d}  dropped={st['frames_dropped']:5d}  "
                    f"sync_loss={st['sync_losses']:5d}  buf={st['buffer_bytes']:4d}"
                )
                if bps == 0:
                    if bytes_rx_total == 0:
                        print(
                            "    [!] No serial bytes yet — close PuTTY/Programmer, "
                            "confirm COM port."
                        )
                    else:
                        print(
                            "    [!] No bytes this second (stream stalled). "
                            "Replug USB; ensure only this script uses the port."
                        )
                if (
                    bps > 0
                    and st["frames_parsed"] == 0
                    and bytes_rx_total > 1000
                ):
                    small_buf = st["buffer_bytes"] <= 4
                    if (
                        small_buf
                        and st["frames_dropped"] == 0
                        and st["sync_losses"] == 0
                    ):
                        print(
                            "    [!] No AA55 in decoded stream at this baud. "
                            "Use --baud 1500000 (match FPGA)."
                        )
                    else:
                        print(
                            "    [!] Bytes but frames_ok=0 — try --baud 1500000; "
                            "if dropped>0, CRC errors at this baud."
                        )
                elif (
                    not warned_stall_decode
                    and st["frames_parsed"] == 0
                    and bytes_rx_total > 1000
                    and bps == 0
                ):
                    print(
                        "    [!] Earlier bytes never decoded. When data resumes use --baud 1500000."
                    )
                    warned_stall_decode = True
                sys.stdout.flush()
                last_stat = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_reader.set()
        reader_thread.join(timeout=2.0)
        ser.close()

    st = parser.stats
    print(
        f"Final: bytes={bytes_rx_total} frames_ok={st['frames_parsed']} "
        f"dropped={st['frames_dropped']}"
    )

    if args.wav is not None and wav_chunks:
        _write_wav(args.wav, np.concatenate(wav_chunks))
        print(f"Wrote {args.wav}")
    elif args.wav is not None:
        print("No audio captured — WAV not written (no decoded frames).")


def _write_wav(path: Path, pcm_int16: np.ndarray) -> None:
    try:
        from scipy.io import wavfile
    except ImportError:
        import wave

        path.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(path), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SAMPLE_RATE)
            w.writeframes(pcm_int16.astype(np.int16).tobytes())
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    wavfile.write(str(path), SAMPLE_RATE, pcm_int16.astype(np.int16))


if __name__ == "__main__":
    main()
