"""
RNNoise-based audio denoiser for ego-noise removal.

Wraps the librnnoise shared library via ctypes for real-time per-channel
denoising. RNNoise is a GRU-based model (~100 KB) specifically designed
for voice noise suppression on low-power CPUs.

RNNoise operates at 48 kHz with 480-sample frames. This wrapper handles
resampling from/to 16 kHz transparently.
"""

import numpy as np
import ctypes
import ctypes.util
from typing import Optional
import logging

logger = logging.getLogger(__name__)

RNNOISE_SAMPLE_RATE = 48000
RNNOISE_FRAME_SIZE = 480


class RNNoiseDenoiser:
    """Per-channel RNNoise denoiser with automatic resampling."""

    def __init__(self,
                 num_channels: int = 7,
                 input_sample_rate: int = 16000,
                 library_path: str = "librnnoise"):
        """
        Args:
            num_channels: Number of audio channels to denoise.
            input_sample_rate: Sample rate of input audio (will resample to 48k).
            library_path: Path to librnnoise shared library (.so/.dll/.dylib).
        """
        self.num_channels = num_channels
        self.input_sample_rate = input_sample_rate
        self.library_path = library_path
        self._lib = None
        self._states = []
        self._initialized = False

        self._resample_ratio = RNNOISE_SAMPLE_RATE / input_sample_rate

    def initialize(self) -> bool:
        """
        Load librnnoise and create denoiser states for each channel.

        Returns:
            True if initialization succeeded, False otherwise.
        """
        try:
            lib_path = ctypes.util.find_library(self.library_path)
            if lib_path is None:
                lib_path = self.library_path
            self._lib = ctypes.CDLL(lib_path)
        except OSError:
            logger.warning(
                "librnnoise not found at '%s'. Denoiser disabled — "
                "audio will pass through unmodified.", self.library_path
            )
            return False

        self._lib.rnnoise_create.restype = ctypes.c_void_p
        self._lib.rnnoise_create.argtypes = [ctypes.c_void_p]

        self._lib.rnnoise_destroy.restype = None
        self._lib.rnnoise_destroy.argtypes = [ctypes.c_void_p]

        self._lib.rnnoise_process_frame.restype = ctypes.c_float
        self._lib.rnnoise_process_frame.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]

        self._states = []
        for _ in range(self.num_channels):
            state = self._lib.rnnoise_create(None)
            if state is None:
                logger.error("Failed to create RNNoise state")
                self.cleanup()
                return False
            self._states.append(state)

        self._initialized = True
        logger.info("RNNoise initialized: %d channels", self.num_channels)
        return True

    def denoise_frame(self, frame: np.ndarray, channel: int) -> np.ndarray:
        """
        Denoise a single frame for one channel.

        Args:
            frame: Audio frame at 48 kHz, shape (480,), float32.
            channel: Channel index (0-6).

        Returns:
            Denoised frame, shape (480,), float32.
        """
        if not self._initialized:
            return frame

        input_buf = frame.astype(np.float32)
        output_buf = np.zeros(RNNOISE_FRAME_SIZE, dtype=np.float32)

        in_ptr = input_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        out_ptr = output_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        self._lib.rnnoise_process_frame(self._states[channel], out_ptr, in_ptr)

        return output_buf

    def denoise_multichannel(self, signals: np.ndarray) -> np.ndarray:
        """
        Denoise all channels. Handles resampling 16 kHz <-> 48 kHz internally.

        Args:
            signals: Multichannel audio at input_sample_rate,
                     shape (num_channels, num_samples).

        Returns:
            Denoised signals at input_sample_rate, same shape as input.
        """
        if not self._initialized:
            return signals

        num_channels, num_samples = signals.shape
        output = np.zeros_like(signals)

        for ch in range(min(num_channels, self.num_channels)):
            upsampled = self._resample_up(signals[ch])

            num_frames = len(upsampled) // RNNOISE_FRAME_SIZE
            denoised_up = np.zeros_like(upsampled)

            for f in range(num_frames):
                start = f * RNNOISE_FRAME_SIZE
                frame = upsampled[start:start + RNNOISE_FRAME_SIZE]
                denoised_up[start:start + RNNOISE_FRAME_SIZE] = \
                    self.denoise_frame(frame, ch)

            output[ch] = self._resample_down(denoised_up, num_samples)

        return output

    def _resample_up(self, signal: np.ndarray) -> np.ndarray:
        """Resample from input_sample_rate to 48 kHz."""
        if self.input_sample_rate == RNNOISE_SAMPLE_RATE:
            return signal.astype(np.float32)

        from .model_utils import resample_audio
        return resample_audio(signal, self.input_sample_rate, RNNOISE_SAMPLE_RATE)

    def _resample_down(self, signal: np.ndarray,
                       target_length: int) -> np.ndarray:
        """Resample from 48 kHz back to input_sample_rate."""
        if self.input_sample_rate == RNNOISE_SAMPLE_RATE:
            return signal

        from .model_utils import resample_audio
        resampled = resample_audio(signal, RNNOISE_SAMPLE_RATE,
                                   self.input_sample_rate)
        # Ensure exact output length matches input
        if len(resampled) > target_length:
            resampled = resampled[:target_length]
        elif len(resampled) < target_length:
            resampled = np.pad(resampled, (0, target_length - len(resampled)))
        return resampled

    def cleanup(self):
        """Release all RNNoise states."""
        if self._lib is not None:
            for state in self._states:
                if state is not None:
                    self._lib.rnnoise_destroy(state)
        self._states = []
        self._initialized = False

    def __del__(self):
        self.cleanup()

    @property
    def is_available(self) -> bool:
        return self._initialized
