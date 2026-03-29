"""
UAV-AudioLoc — Pi 4B dual serial reassembly
Reads frames from ESP32-S3 #1 (ttyUSB0) and #2 (ttyUSB1),
merges into full 7-channel frame, feeds GCC-PHAT pipeline.

Frame format (both boards):
  [0xAA][0x55][board_id][ch_count][samples interleaved int32 LE][CRC16 2B]

Usage:
  python3 serial_merge.py [--port0 /dev/ttyUSB0] [--port1 /dev/ttyUSB1] [--stats]
"""

import serial
import struct
import threading
import queue
import argparse
import time

BAUD        = 1500000
MAGIC       = (0xAA, 0x55)
DMA_BUF_LEN = 256
BOARD1_CH   = 4   # mic0, mic1, mic2, mic3
BOARD2_CH   = 3   # mic4, mic5, mic6(centre)

# ── CRC16-CCITT ───────────────────────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc

# ── Frame parser ──────────────────────────────────────────────────────────────
def read_frame(ser: serial.Serial):
    """Block until a valid frame is received. Returns (board_id, samples_2d)."""
    while True:
        # Sync on magic bytes
        b = ser.read(1)
        if not b or b[0] != 0xAA:
            continue
        b = ser.read(1)
        if not b or b[0] != 0x55:
            continue

        hdr = ser.read(2)   # board_id, ch_count
        if len(hdr) < 2:
            continue

        board_id = hdr[0]
        ch_count = hdr[1]
        payload_bytes = ch_count * DMA_BUF_LEN * 4

        payload = ser.read(payload_bytes)
        if len(payload) < payload_bytes:
            continue

        crc_bytes = ser.read(2)
        if len(crc_bytes) < 2:
            continue

        rx_crc = (crc_bytes[0] << 8) | crc_bytes[1]
        if crc16(payload) != rx_crc:
            print(f"[!] CRC fail board {board_id:#x}")
            continue

        # Deinterleave: [s0_ch0, s0_ch1, ..., s1_ch0, ...]
        all_samples = struct.unpack(f"<{ch_count * DMA_BUF_LEN}i", payload)
        channels = []
        for c in range(ch_count):
            channels.append([all_samples[s * ch_count + c] for s in range(DMA_BUF_LEN)])

        return board_id, channels

# ── Per-board reader thread ───────────────────────────────────────────────────
def board_reader(port: str, q: queue.Queue):
    ser = serial.Serial(port, BAUD, timeout=2)
    print(f"[+] Opened {port}")
    while True:
        try:
            frame = read_frame(ser)
            q.put(frame)
        except Exception as e:
            print(f"[!] {port} error: {e}")
            time.sleep(0.1)

# ── GCC-PHAT stub — replace with your real implementation ────────────────────
def gcc_phat(channels_7ch):
    """
    channels_7ch: list of 7 lists, each DMA_BUF_LEN int32 samples
    mic order: 0,1,2,3,4,5 outer ring, 6 centre
    Returns azimuth angle in degrees (0-359) or None
    """
    # TODO: plug in your existing GCC-PHAT from Pi pipeline
    return None

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port0", default="/dev/ttyUSB0")
    parser.add_argument("--port1", default="/dev/ttyUSB1")
    parser.add_argument("--stats", action="store_true")
    args = parser.parse_args()

    q0 = queue.Queue(maxsize=8)
    q1 = queue.Queue(maxsize=8)

    t0 = threading.Thread(target=board_reader, args=(args.port0, q0), daemon=True)
    t1 = threading.Thread(target=board_reader, args=(args.port1, q1), daemon=True)
    t0.start()
    t1.start()

    frames_merged = 0
    t_start = time.time()

    print("[*] Waiting for frames from both boards...")

    while True:
        try:
            board_id_0, ch_0 = q0.get(timeout=2)   # mic0, mic1, mic2
            board_id_1, ch_1 = q1.get(timeout=2)   # mic3, mic4, mic5, mic6
        except queue.Empty:
            print("[!] Timeout waiting for frame — check connections")
            continue

        # Validate board IDs
        if board_id_0 != 0x01:
            ch_0, ch_1 = ch_1, ch_0   # swap if boards swapped

        # Merge to 7 channels: [mic0..mic5, mic6_centre]
        channels_7 = ch_0 + ch_1   # 3 + 4 = 7

        # Run GCC-PHAT
        azimuth = gcc_phat(channels_7)

        frames_merged += 1

        if args.stats:
            elapsed = time.time() - t_start
            fps = frames_merged / elapsed if elapsed > 0 else 0
            print(f"\r  frames={frames_merged}  fps={fps:.1f}  az={azimuth}°   ", end="")
        elif azimuth is not None:
            print(f"[DOA] Azimuth: {azimuth:.1f}°")

if __name__ == "__main__":
    main()
