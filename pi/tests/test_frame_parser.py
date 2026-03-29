"""
Tests for LEGACY FPGA TDM frame parser.

TODO(migration): These tests cover the old FPGA mu-law frame format (228 bytes,
32×7 samples). The ESP32-S3 architecture uses a completely different frame format
(int32 PCM, board_id/ch_count header, CRC over payload only). New tests for the
ESP32-S3 frame parsing are needed — the parsing logic lives in
capture/serial_reader.py (DualSerialReader / _read_frame).

Verifies sync word detection, CRC validation, mu-law decoding,
and error handling for corrupted/partial frames.
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from capture.frame_parser import (
    FrameParser, mulaw_decode, crc16,
    SYNC_WORD, FRAME_SIZE, PAYLOAD_SIZE,
    SAMPLES_PER_FRAME, CHANNELS_PER_SAMPLE
)


def build_valid_frame(payload=None):
    """Build a valid TDM frame with proper sync and CRC."""
    if payload is None:
        payload = bytes(range(PAYLOAD_SIZE))  # deterministic test data

    frame = bytearray()
    frame.append(0xAA)  # sync high
    frame.append(0x55)  # sync low
    frame.extend(payload)

    crc_data = bytes(frame)
    crc_val = crc16(crc_data)
    frame.append((crc_val >> 8) & 0xFF)
    frame.append(crc_val & 0xFF)

    return bytes(frame)


class TestCRC16:
    def test_known_value(self):
        """CRC of known data should be deterministic."""
        crc = crc16(b"\xAA\x55\x00\x01\x02\x03")
        assert isinstance(crc, int)
        assert 0 <= crc <= 0xFFFF

    def test_empty_data(self):
        crc = crc16(b"")
        assert crc == 0xFFFF  # initial value, no data

    def test_different_data_different_crc(self):
        crc1 = crc16(b"\x00\x01\x02")
        crc2 = crc16(b"\x00\x01\x03")
        assert crc1 != crc2


class TestMuLawDecode:
    def test_output_shape(self):
        encoded = np.array([0, 128, 255], dtype=np.uint8)
        decoded = mulaw_decode(encoded)
        assert decoded.shape == (3,)
        assert decoded.dtype == np.int16

    def test_symmetry(self):
        """Complementary mu-law codes should decode to opposite signs."""
        decoded = mulaw_decode(np.arange(256, dtype=np.uint8))
        assert decoded.min() < 0
        assert decoded.max() > 0

    def test_full_range(self):
        decoded = mulaw_decode(np.arange(256, dtype=np.uint8))
        assert len(decoded) == 256


class TestFrameParser:
    def setup_method(self):
        self.parser = FrameParser()

    def test_valid_frame(self):
        """Parser should successfully decode a valid frame."""
        frame = build_valid_frame()
        result = self.parser.feed(frame)

        assert result is not None
        assert result.shape == (7, 32)
        assert result.dtype == np.int16
        assert self.parser.frames_parsed == 1
        assert self.parser.frames_dropped == 0

    def test_corrupted_crc(self):
        """Frame with bad CRC should be dropped."""
        frame = bytearray(build_valid_frame())
        frame[-1] ^= 0xFF  # corrupt CRC
        result = self.parser.feed(bytes(frame))

        assert result is None
        assert self.parser.frames_dropped == 1

    def test_missing_sync(self):
        """Random bytes without sync word should be discarded."""
        garbage = bytes([0x12, 0x34, 0x56, 0x78] * 100)
        result = self.parser.feed(garbage)
        assert result is None

    def test_partial_frame(self):
        """Incomplete frame should wait for more data."""
        frame = build_valid_frame()
        half = len(frame) // 2

        result1 = self.parser.feed(frame[:half])
        assert result1 is None

        result2 = self.parser.feed(frame[half:])
        assert result2 is not None
        assert result2.shape == (7, 32)

    def test_garbage_before_valid_frame(self):
        """Parser should skip garbage and find the valid frame."""
        garbage = b"\x00\x11\x22\x33\x44\x55"
        frame = build_valid_frame()

        result = self.parser.feed(garbage + frame)
        assert result is not None
        assert self.parser.sync_losses >= 1

    def test_multiple_frames(self):
        """Parser should handle multiple consecutive frames."""
        frame = build_valid_frame()
        result = self.parser.feed(frame + frame + frame)

        assert result is not None
        assert result.shape[0] == 7
        assert result.shape[1] == 32 * 3
        assert self.parser.frames_parsed == 3

    def test_stats(self):
        """Stats dictionary should have all expected fields."""
        stats = self.parser.stats
        assert "frames_parsed" in stats
        assert "frames_dropped" in stats
        assert "sync_losses" in stats
        assert "drop_rate" in stats
        assert "buffer_bytes" in stats

    def test_reset(self):
        """Reset should clear all state."""
        frame = build_valid_frame()
        self.parser.feed(frame)
        self.parser.reset()

        assert self.parser.frames_parsed == 0
        assert self.parser.frames_dropped == 0
        assert self.parser.sync_losses == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
