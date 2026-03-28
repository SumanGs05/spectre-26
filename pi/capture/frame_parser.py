"""
TDM frame parser for FPGA-to-Pi data stream.

Handles sync word detection, CRC16 validation, mu-law decoding,
and unpacking of 7-channel audio from the batched TDM frame format:

    | SYNC (0xAA55) | 32 x [CH0..CH6] mu-law bytes | CRC16 |
    |   2 bytes     | 224 bytes                     | 2 bytes|
    Total: 228 bytes per frame
"""

import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)

SYNC_WORD = 0xAA55
SYNC_BYTES = b'\xAA\x55'
SAMPLES_PER_FRAME = 32
CHANNELS_PER_SAMPLE = 7
PAYLOAD_SIZE = SAMPLES_PER_FRAME * CHANNELS_PER_SAMPLE  # 224 bytes
CRC_SIZE = 2
FRAME_SIZE = 2 + PAYLOAD_SIZE + CRC_SIZE  # 228 bytes

# ITU G.711 mu-law decode table (8-bit index -> 16-bit linear PCM)
_MULAW_DECODE_TABLE = np.zeros(256, dtype=np.int16)


def _build_mulaw_table():
    """Pre-compute mu-law to linear PCM lookup table."""
    for i in range(256):
        val = ~i
        sign = val & 0x80
        exponent = (val >> 4) & 0x07
        mantissa = val & 0x0F
        sample = (mantissa << 4) + 0x08
        sample <<= exponent
        sample -= 0x84
        if sign:
            sample = -sample
        _MULAW_DECODE_TABLE[i] = np.int16(np.clip(sample, -32768, 32767))


_build_mulaw_table()


def mulaw_decode(encoded: np.ndarray) -> np.ndarray:
    """
    Decode mu-law encoded bytes to 16-bit linear PCM.

    Args:
        encoded: uint8 array of mu-law encoded samples.

    Returns:
        int16 array of linear PCM samples.
    """
    return _MULAW_DECODE_TABLE[encoded.astype(np.uint8)]


def crc16(data: bytes) -> int:
    """
    CRC-16/CCITT-FALSE checksum.

    Polynomial: 0x1021, Init: 0xFFFF, no final XOR.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


class FrameParser:
    """
    Stateful TDM frame parser with sync recovery.

    Maintains an internal byte buffer, finds sync words, validates CRC,
    and yields decoded multichannel audio blocks.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.frames_parsed = 0
        self.frames_dropped = 0
        self.sync_losses = 0

    def feed(self, raw_bytes: bytes) -> Optional[np.ndarray]:
        """
        Feed raw bytes from serial port and extract complete frames.

        Args:
            raw_bytes: Raw bytes from USB-UART.

        Returns:
            Decoded audio block of shape (7, 32) as int16, or None
            if no complete valid frame is available yet.
        """
        self._buffer.extend(raw_bytes)
        frames = []

        while len(self._buffer) >= FRAME_SIZE:
            sync_pos = self._find_sync()
            if sync_pos is None:
                # No sync found — discard everything except last byte
                if len(self._buffer) > 1:
                    self._buffer = self._buffer[-1:]
                break

            if sync_pos > 0:
                self.sync_losses += 1
                logger.debug("Sync loss: discarded %d bytes", sync_pos)
                self._buffer = self._buffer[sync_pos:]

            if len(self._buffer) < FRAME_SIZE:
                break

            frame_bytes = bytes(self._buffer[:FRAME_SIZE])
            self._buffer = self._buffer[FRAME_SIZE:]

            audio = self._parse_frame(frame_bytes)
            if audio is not None:
                frames.append(audio)
                self.frames_parsed += 1
            else:
                self.frames_dropped += 1

        if frames:
            return np.concatenate(frames, axis=1)  # shape (7, N*32)
        return None

    def _find_sync(self) -> Optional[int]:
        """Find position of next sync word in buffer."""
        buf = bytes(self._buffer)
        pos = buf.find(SYNC_BYTES)
        return pos if pos >= 0 else None

    def _parse_frame(self, frame: bytes) -> Optional[np.ndarray]:
        """
        Parse and validate a single TDM frame.

        Returns:
            Audio data shape (7, 32) as int16, or None if CRC fails.
        """
        sync = (frame[0] << 8) | frame[1]
        if sync != SYNC_WORD:
            return None

        payload = frame[2:2 + PAYLOAD_SIZE]
        received_crc = (frame[-2] << 8) | frame[-1]

        computed_crc = crc16(frame[:2 + PAYLOAD_SIZE])
        if computed_crc != received_crc:
            logger.debug("CRC mismatch: expected 0x%04X, got 0x%04X",
                         computed_crc, received_crc)
            return None

        # Unpack: payload is 32 samples x 7 channels, interleaved
        # [S0_CH0, S0_CH1, ..., S0_CH6, S1_CH0, ..., S31_CH6]
        encoded = np.frombuffer(payload, dtype=np.uint8)
        encoded = encoded.reshape(SAMPLES_PER_FRAME, CHANNELS_PER_SAMPLE)

        pcm = mulaw_decode(encoded)  # shape (32, 7) int16
        return pcm.T  # shape (7, 32)

    def reset(self):
        """Clear internal buffer and counters."""
        self._buffer = bytearray()
        self.frames_parsed = 0
        self.frames_dropped = 0
        self.sync_losses = 0

    @property
    def stats(self) -> dict:
        total = self.frames_parsed + self.frames_dropped
        return {
            "frames_parsed": self.frames_parsed,
            "frames_dropped": self.frames_dropped,
            "sync_losses": self.sync_losses,
            "drop_rate": self.frames_dropped / max(1, total),
            "buffer_bytes": len(self._buffer),
        }
