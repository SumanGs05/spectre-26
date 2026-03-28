"""
USB-UART serial reader for Tang Nano 9K FPGA data capture.

Reads raw bytes from the FPGA's USB-UART bridge (connected via USB cable)
using pyserial. Runs in a dedicated thread to prevent blocking the
DSP pipeline.
"""

import threading
import queue
import logging
from typing import Optional

logger = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1_500_000
READ_CHUNK_SIZE = 4096


class SerialReader:
    """
    Threaded serial reader for continuous FPGA data capture.

    Reads raw bytes from USB-UART in a background thread and pushes
    them to a thread-safe queue for the frame parser.
    """

    def __init__(self,
                 port: str = DEFAULT_PORT,
                 baudrate: int = DEFAULT_BAUDRATE,
                 timeout: float = 0.1,
                 queue_maxsize: int = 100):
        """
        Args:
            port: Serial device path (e.g., /dev/ttyACM0 or /dev/ttyUSB0).
            baudrate: Baud rate (2 Mbaud for our TDM stream).
            timeout: Read timeout in seconds.
            queue_maxsize: Max queued byte chunks before dropping.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._serial = None
        self._thread = None
        self._running = False
        self._queue = queue.Queue(maxsize=queue_maxsize)
        self.bytes_read = 0
        self.overflows = 0

    def start(self):
        """Open serial port and start background reader thread."""
        try:
            import serial
        except ImportError:
            raise RuntimeError(
                "pyserial not installed. Run: pip install pyserial"
            )

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

        self._running = True
        self._thread = threading.Thread(target=self._read_loop,
                                        name="serial-reader",
                                        daemon=True)
        self._thread.start()
        logger.info("Serial reader started: %s @ %d baud",
                     self.port, self.baudrate)

    def read(self, timeout: float = 0.1) -> Optional[bytes]:
        """
        Get the next chunk of raw bytes from the FPGA.

        Args:
            timeout: Maximum wait time in seconds.

        Returns:
            Raw bytes, or None if no data available.
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _read_loop(self):
        """Background thread: continuously read from serial port."""
        while self._running:
            try:
                data = self._serial.read(READ_CHUNK_SIZE)
                if data:
                    self.bytes_read += len(data)
                    try:
                        self._queue.put_nowait(data)
                    except queue.Full:
                        self.overflows += 1
                        # Drop oldest to keep up with real-time
                        try:
                            self._queue.get_nowait()
                        except queue.Empty:
                            pass
                        self._queue.put_nowait(data)
            except Exception as e:
                if self._running:
                    logger.error("Serial read error: %s", e)

    def stop(self):
        """Stop reader thread and close serial port."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        logger.info("Serial reader stopped. Total bytes: %d, overflows: %d",
                     self.bytes_read, self.overflows)

    @property
    def is_running(self) -> bool:
        return self._running and (self._thread is not None
                                  and self._thread.is_alive())

    @property
    def stats(self) -> dict:
        return {
            "bytes_read": self.bytes_read,
            "overflows": self.overflows,
            "queue_size": self._queue.qsize(),
            "running": self.is_running,
        }
