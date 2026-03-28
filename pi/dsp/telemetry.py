"""
Telemetry bus for DOA estimation results.

Produces standardized JSON telemetry packets, logs to CSV files,
and publishes over WebSocket for the monitoring dashboard.

This module is the autonomy-ready interface: the same telemetry packet
consumed by the web dashboard will be consumed by the MAVLink flight
controller module in Phase 2.
"""

import json
import time
import csv
import asyncio
import logging
from pathlib import Path
from typing import Optional, Set
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)


@dataclass
class TelemetryPacket:
    """Standardized DOA telemetry output."""
    timestamp_ms: int
    azimuth_deg: float
    confidence: float
    snr_db: float
    detection: bool
    sound_class: str
    class_confidence: float
    frame_id: int

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    def to_dict(self) -> dict:
        return asdict(self)


class TelemetryBus:
    """
    Central telemetry publisher: CSV log + WebSocket broadcast.
    """

    def __init__(self,
                 log_dir: str = "logs",
                 csv_enabled: bool = True,
                 websocket_enabled: bool = True):
        self.log_dir = Path(log_dir)
        self.csv_enabled = csv_enabled
        self.websocket_enabled = websocket_enabled

        self._csv_writer = None
        self._csv_file = None
        self._ws_clients: Set[asyncio.Queue] = set()
        self._frame_counter = 0
        self._latest_packet: Optional[TelemetryPacket] = None

    def start(self):
        """Initialize logging outputs."""
        if self.csv_enabled:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            csv_path = self.log_dir / f"telemetry_{timestamp}.csv"
            self._csv_file = open(csv_path, "w", newline="")
            self._csv_writer = csv.DictWriter(
                self._csv_file,
                fieldnames=[
                    "timestamp_ms", "azimuth_deg", "confidence",
                    "snr_db", "detection", "sound_class",
                    "class_confidence", "frame_id"
                ]
            )
            self._csv_writer.writeheader()
            logger.info("CSV telemetry log: %s", csv_path)

    def publish(self,
                azimuth_deg: float,
                confidence: float,
                snr_db: float,
                detection: bool,
                sound_class: str = "None",
                class_confidence: float = 0.0) -> TelemetryPacket:
        """
        Create and publish a telemetry packet.

        Args:
            azimuth_deg: Estimated DOA in degrees (0-360, relative to drone nose).
            confidence: DOA confidence (0-1).
            snr_db: Post-filter signal-to-noise ratio.
            detection: Whether a human sound was detected.
            sound_class: YAMNet class name of detected sound.
            class_confidence: YAMNet probability for detected class.

        Returns:
            The published TelemetryPacket.
        """
        self._frame_counter += 1

        packet = TelemetryPacket(
            timestamp_ms=int(time.time() * 1000),
            azimuth_deg=round(azimuth_deg, 1),
            confidence=round(confidence, 3),
            snr_db=round(snr_db, 1),
            detection=detection,
            sound_class=sound_class,
            class_confidence=round(class_confidence, 3),
            frame_id=self._frame_counter,
        )

        self._latest_packet = packet

        if self._csv_writer is not None:
            self._csv_writer.writerow(packet.to_dict())
            self._csv_file.flush()

        if self.websocket_enabled:
            self._broadcast_ws(packet)

        return packet

    def register_ws_client(self) -> asyncio.Queue:
        """Register a new WebSocket client and return its message queue."""
        q = asyncio.Queue(maxsize=50)
        self._ws_clients.add(q)
        logger.info("WebSocket client registered. Total: %d",
                     len(self._ws_clients))
        return q

    def unregister_ws_client(self, q: asyncio.Queue):
        """Remove a WebSocket client's queue."""
        self._ws_clients.discard(q)

    def _broadcast_ws(self, packet: TelemetryPacket):
        """Push packet to all connected WebSocket client queues."""
        msg = packet.to_json()
        dead_clients = set()

        for q in self._ws_clients:
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                # Client too slow — drop oldest message
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except (asyncio.QueueEmpty, asyncio.QueueFull):
                    dead_clients.add(q)

        self._ws_clients -= dead_clients

    @property
    def latest(self) -> Optional[TelemetryPacket]:
        return self._latest_packet

    def stop(self):
        """Flush and close all outputs."""
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self._ws_clients.clear()
        logger.info("Telemetry bus stopped. Total frames: %d",
                     self._frame_counter)
