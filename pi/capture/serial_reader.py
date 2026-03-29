"""
Dual-port serial reader for ESP32-S3 pair → Pi audio capture.

Reads frames from two ESP32-S3 boards (Master on ttyUSB0, Slave on ttyUSB1),
validates CRC16, deinterleaves per-board channels, and merges into a single
7-channel audio array [mic0..mic5, mic6_centre].

Frame format (both boards):
  [0xAA][0x55][board_id][ch_count][samples interleaved int32 LE][CRC16 2B]

Master (board_id=0x01): 4 channels × 256 samples × 4 bytes = 4096 B payload
Slave  (board_id=0x02): 3 channels × 256 samples × 4 bytes = 3072 B payload

Based on esp32/serial_merge.py reference implementation.
"""

import threading
import queue
import struct
import logging
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

BAUD = 1500000
DMA_BUF_LEN = 256
BOARD_MASTER_ID = 0x01
BOARD_SLAVE_ID = 0x02
BOARD_MASTER_CH = 4
BOARD_SLAVE_CH = 3
TOTAL_CHANNELS = BOARD_MASTER_CH + BOARD_SLAVE_CH  # 7

DEFAULT_PORT_MASTER = "/dev/ttyUSB0"
DEFAULT_PORT_SLAVE = "/dev/ttyUSB1"


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc


def _read_frame(ser):
    """
    Block until a valid ESP32-S3 frame is received.

    Returns (board_id, channels) where channels is a list of ch_count
    lists each containing DMA_BUF_LEN int32 samples, or raises on serial
    error.
    """
    while True:
        b = ser.read(1)
        if not b or b[0] != 0xAA:
            continue
        b = ser.read(1)
        if not b or b[0] != 0x55:
            continue

        hdr = ser.read(2)
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
            logger.debug("CRC fail board %#x", board_id)
            continue

        all_samples = struct.unpack(f"<{ch_count * DMA_BUF_LEN}i", payload)
        channels = []
        for c in range(ch_count):
            channels.append(
                [all_samples[s * ch_count + c] for s in range(DMA_BUF_LEN)]
            )

        return board_id, channels


class DualSerialReader:
    """
    Threaded dual-port serial reader for ESP32-S3 pair.

    Two background threads read frames from Master (4ch: mic0-mic3)
    and Slave (3ch: mic4-mic6). A merge thread pairs them and produces
    7-channel numpy arrays of shape (7, DMA_BUF_LEN) int32.
    """

    def __init__(self,
                 port_master: str = DEFAULT_PORT_MASTER,
                 port_slave: str = DEFAULT_PORT_SLAVE,
                 baud: int = BAUD,
                 timeout: float = 2.0,
                 queue_maxsize: int = 32):
        self.port_master = port_master
        self.port_slave = port_slave
        self.baud = baud
        self.timeout = timeout

        self._ser_master = None
        self._ser_slave = None
        self._thread_master = None
        self._thread_slave = None
        self._merge_thread = None
        self._running = False

        self._q_master = queue.Queue(maxsize=8)
        self._q_slave = queue.Queue(maxsize=8)
        self._merged_q = queue.Queue(maxsize=queue_maxsize)

        self.frames_master = 0
        self.frames_slave = 0
        self.frames_merged = 0
        self.crc_errors = 0
        self.merge_timeouts = 0

    def start(self):
        """Open both serial ports and start background threads."""
        try:
            import serial
        except ImportError:
            raise RuntimeError(
                "pyserial not installed. Run: pip install pyserial"
            )

        self._ser_master = serial.Serial(
            self.port_master, self.baud, timeout=self.timeout
        )
        self._ser_slave = serial.Serial(
            self.port_slave, self.baud, timeout=self.timeout
        )

        logger.info("Opened Master: %s, Slave: %s @ %d baud",
                     self.port_master, self.port_slave, self.baud)

        self._running = True

        self._thread_master = threading.Thread(
            target=self._board_reader,
            args=(self._ser_master, self._q_master, "master"),
            name="serial-master", daemon=True,
        )
        self._thread_slave = threading.Thread(
            target=self._board_reader,
            args=(self._ser_slave, self._q_slave, "slave"),
            name="serial-slave", daemon=True,
        )
        self._merge_thread = threading.Thread(
            target=self._merge_loop,
            name="serial-merge", daemon=True,
        )

        self._thread_master.start()
        self._thread_slave.start()
        self._merge_thread.start()

        logger.info("Dual serial reader started")

    def read(self, timeout: float = 0.1) -> Optional[np.ndarray]:
        """
        Get the next merged 7-channel audio block.

        Returns:
            np.ndarray of shape (7, DMA_BUF_LEN) int32, or None if no
            merged frame is available within timeout.
        """
        try:
            return self._merged_q.get(timeout=timeout)
        except queue.Empty:
            return None

    def _board_reader(self, ser, q: queue.Queue, label: str):
        """Background thread: read frames from one ESP32-S3 board."""
        while self._running:
            try:
                result = _read_frame(ser)
                if result is not None:
                    q.put(result)
                    if label == "master":
                        self.frames_master += 1
                    else:
                        self.frames_slave += 1
            except Exception as e:
                if self._running:
                    logger.error("%s reader error: %s", label, e)
                    time.sleep(0.1)

    def _merge_loop(self):
        """Consume one frame from each board, merge to 7 channels."""
        while self._running:
            try:
                bid0, ch0 = self._q_master.get(timeout=2)
                bid1, ch1 = self._q_slave.get(timeout=2)
            except queue.Empty:
                self.merge_timeouts += 1
                logger.debug("Merge timeout — waiting for both boards")
                continue

            if bid0 != BOARD_MASTER_ID:
                ch0, ch1 = ch1, ch0

            # ch0 = [mic0, mic1, mic2, mic3], ch1 = [mic4, mic5, mic6]
            merged = np.array(ch0 + ch1, dtype=np.int32)  # (7, 256)

            try:
                self._merged_q.put_nowait(merged)
            except queue.Full:
                try:
                    self._merged_q.get_nowait()
                except queue.Empty:
                    pass
                self._merged_q.put_nowait(merged)

            self.frames_merged += 1

    def stop(self):
        """Stop all threads and close serial ports."""
        self._running = False
        for t in (self._thread_master, self._thread_slave, self._merge_thread):
            if t is not None:
                t.join(timeout=2.0)
        for s in (self._ser_master, self._ser_slave):
            if s is not None and s.is_open:
                s.close()
        logger.info(
            "Dual serial reader stopped. master=%d slave=%d merged=%d "
            "timeouts=%d",
            self.frames_master, self.frames_slave,
            self.frames_merged, self.merge_timeouts,
        )

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def stats(self) -> dict:
        return {
            "frames_master": self.frames_master,
            "frames_slave": self.frames_slave,
            "frames_merged": self.frames_merged,
            "merge_timeouts": self.merge_timeouts,
            "merged_queue_size": self._merged_q.qsize(),
            "running": self.is_running,
        }
