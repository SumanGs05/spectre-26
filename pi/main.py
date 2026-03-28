"""
UAV-AudioLoc — Real-time pipeline orchestrator.

Ties together all processing stages:
    USB-Serial Capture → Frame Parse → Spectral Sub → RNNoise
    → YAMNet Classifier → GCC-PHAT → DOA → Telemetry

Designed for autonomous drone operation: outputs machine-consumable
telemetry packets at ~30 Hz. The web dashboard and future flight
controller both subscribe to the same telemetry bus.
"""

import time
import signal
import logging
import yaml
import numpy as np
from pathlib import Path

from capture.serial_reader import SerialReader
from capture.frame_parser import FrameParser
from dsp.spectral_sub import SpectralSubtractor
from dsp.gcc_phat import gcc_phat_batch
from dsp.doa_estimator import DOAEstimator
from dsp.array_geometry import (
    get_mic_positions, get_outer_pairs, CENTER_MIC_INDEX
)
from dsp.telemetry import TelemetryBus
from ml.denoiser import RNNoiseDenoiser
from ml.classifier import SoundClassifier

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("audioloc")


class AudioLocPipeline:
    """Main real-time audio localization pipeline."""

    def __init__(self, config_path: str = "config.yaml"):
        self.config = self._load_config(config_path)
        self._running = False

        # Processing components (initialized in setup())
        self.serial_reader = None
        self.frame_parser = None
        self.spectral_sub = None
        self.denoiser = None
        self.classifier = None
        self.doa_estimator = None
        self.telemetry = None

        # Audio accumulation buffer
        self._audio_buffer = None
        self._buffer_samples = 0

    def _load_config(self, path: str) -> dict:
        config_file = Path(__file__).parent / path
        with open(config_file) as f:
            return yaml.safe_load(f)

    def setup(self):
        """Initialize all pipeline components."""
        cfg = self.config
        audio_cfg = cfg["audio"]
        sr = audio_cfg["sample_rate"]
        fft_size = audio_cfg["fft_size"]
        hop_size = audio_cfg["hop_size"]
        n_ch = audio_cfg["num_channels"]

        # Capture
        serial_cfg = cfg["serial"]
        self.serial_reader = SerialReader(
            port=serial_cfg["port"],
            baudrate=serial_cfg["baudrate"],
            timeout=serial_cfg["timeout"],
        )
        self.frame_parser = FrameParser()

        # DSP
        ss_cfg = cfg["spectral_subtraction"]
        self.spectral_sub = SpectralSubtractor(
            sample_rate=sr,
            fft_size=fft_size,
            hop_size=hop_size,
            hover_frequency=ss_cfg["hover_frequency"],
            num_harmonics=ss_cfg["num_harmonics"],
            oversubtraction=ss_cfg["oversubtraction"],
            spectral_floor=ss_cfg["spectral_floor"],
        )

        # ML Denoiser
        dn_cfg = cfg["denoiser"]
        self.denoiser = RNNoiseDenoiser(
            num_channels=n_ch,
            input_sample_rate=sr,
            library_path=dn_cfg["library_path"],
        )
        if dn_cfg["enabled"]:
            self.denoiser.initialize()

        # ML Classifier
        cl_cfg = cfg["classifier"]
        self.classifier = SoundClassifier(
            model_path=cl_cfg["model_path"],
            confidence_threshold=cl_cfg["confidence_threshold"],
            human_classes=cl_cfg["human_classes"],
            input_sample_rate=sr,
        )
        if cl_cfg["enabled"]:
            self.classifier.initialize()

        # DOA Estimator
        mic_cfg = cfg["mic_array"]
        positions = get_mic_positions(
            radius_m=mic_cfg["radius_m"],
            num_outer=mic_cfg["num_outer"],
        )
        pairs = get_outer_pairs(num_outer=mic_cfg["num_outer"])

        doa_cfg = cfg["doa"]
        self.doa_estimator = DOAEstimator(
            positions=positions,
            pairs=pairs,
            speed_of_sound=mic_cfg["speed_of_sound"],
            histogram_bins=doa_cfg["histogram_bins"],
            histogram_sigma=doa_cfg["histogram_sigma"],
            min_confidence=doa_cfg["min_confidence"],
        )

        # Telemetry
        tel_cfg = cfg["telemetry"]
        self.telemetry = TelemetryBus(
            log_dir=tel_cfg["log_dir"],
            csv_enabled=tel_cfg["csv_enabled"],
            websocket_enabled=tel_cfg["websocket_enabled"],
        )

        # Audio buffer: accumulate until we have fft_size samples
        self._audio_buffer = np.zeros((n_ch, 0), dtype=np.float32)
        self._fft_size = fft_size
        self._hop_size = hop_size
        self._sample_rate = sr
        self._pairs = pairs

        logger.info("Pipeline setup complete")
        logger.info("  Sample rate: %d Hz", sr)
        logger.info("  FFT size: %d (%d ms)", fft_size,
                     int(1000 * fft_size / sr))
        logger.info("  Denoiser: %s",
                     "enabled" if self.denoiser.is_available else "disabled")
        logger.info("  Classifier: %s",
                     "enabled" if self.classifier.is_available else "disabled")

    def run(self):
        """Start the real-time processing loop."""
        self.serial_reader.start()
        self.telemetry.start()
        self._running = True

        logger.info("Pipeline running — waiting for audio data...")

        try:
            while self._running:
                raw = self.serial_reader.read(timeout=0.05)
                if raw is None:
                    continue

                audio_block = self.frame_parser.feed(raw)
                if audio_block is None:
                    continue

                # audio_block: shape (7, N) int16 → float32 normalized
                audio = audio_block.astype(np.float32) / 32768.0

                self._audio_buffer = np.concatenate(
                    [self._audio_buffer, audio], axis=1
                )

                while self._audio_buffer.shape[1] >= self._fft_size:
                    frame = self._audio_buffer[:, :self._fft_size]
                    self._audio_buffer = self._audio_buffer[:, self._hop_size:]
                    self._process_frame(frame)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.shutdown()

    def _process_frame(self, signals: np.ndarray):
        """
        Process one FFT-sized frame through the full pipeline.

        Args:
            signals: shape (7, fft_size), float32 normalized to [-1, 1].
        """
        # Stage 1: Spectral subtraction (remove motor harmonics)
        cleaned = self.spectral_sub.process_multichannel(signals)

        # Stage 2: ML denoising (broadband noise removal)
        if self.denoiser.is_available:
            cleaned = self.denoiser.denoise_multichannel(cleaned)

        # Stage 3: Sound classification on center mic (gate)
        center_audio = cleaned[CENTER_MIC_INDEX]
        is_human, sound_class, class_conf = self.classifier.classify(center_audio)

        if not is_human:
            self.telemetry.publish(
                azimuth_deg=0.0,
                confidence=0.0,
                snr_db=self._estimate_snr(signals, cleaned),
                detection=False,
                sound_class=sound_class,
                class_confidence=class_conf,
            )
            return

        # Stage 4: GCC-PHAT on 6 outer mics
        outer_signals = cleaned[:6]  # exclude center mic
        max_delay = int(
            self.config["mic_array"]["radius_m"]
            / self.config["mic_array"]["speed_of_sound"]
            * self._sample_rate
        ) + 2  # +2 for safety margin

        tdoas, gcc_confs = gcc_phat_batch(
            outer_signals, self._pairs,
            sample_rate=self._sample_rate,
            max_delay_samples=max_delay,
        )

        # Stage 5: DOA estimation
        azimuth, doa_conf = self.doa_estimator.estimate(tdoas, gcc_confs)

        # Stage 6: Publish telemetry
        self.telemetry.publish(
            azimuth_deg=azimuth,
            confidence=doa_conf,
            snr_db=self._estimate_snr(signals, cleaned),
            detection=True,
            sound_class=sound_class,
            class_confidence=class_conf,
        )

    def _estimate_snr(self, raw: np.ndarray, cleaned: np.ndarray) -> float:
        """Rough SNR estimate: ratio of cleaned signal to removed noise."""
        noise = raw - cleaned
        signal_power = np.mean(cleaned ** 2) + 1e-10
        noise_power = np.mean(noise ** 2) + 1e-10
        return float(10 * np.log10(signal_power / noise_power))

    def shutdown(self):
        """Gracefully stop all components."""
        self._running = False
        if self.serial_reader:
            self.serial_reader.stop()
        if self.denoiser:
            self.denoiser.cleanup()
        if self.telemetry:
            self.telemetry.stop()
        logger.info("Pipeline shutdown complete")
        logger.info("Frame stats: %s", self.frame_parser.stats)


def main():
    pipeline = AudioLocPipeline()
    pipeline.setup()

    def handle_signal(signum, frame):
        logger.info("Signal %d received, shutting down...", signum)
        pipeline.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    pipeline.run()


if __name__ == "__main__":
    main()
