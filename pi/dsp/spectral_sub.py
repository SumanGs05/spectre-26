"""
Spectral subtraction for drone motor harmonic removal.

First-pass noise reduction that removes predictable harmonic components
from the drone's motors/propellers. The motor fundamental frequency and
its harmonics are estimated and subtracted using a Wiener-like gain mask.

This is intentionally a simple, cheap DSP stage that handles the
deterministic part of ego-noise. Broadband residual noise is handled
downstream by the RNNoise ML denoiser.
"""

import numpy as np
from typing import Optional


class SpectralSubtractor:
    def __init__(self,
                 sample_rate: int = 16000,
                 fft_size: int = 1024,
                 hop_size: int = 512,
                 hover_frequency: float = 200.0,
                 num_harmonics: int = 8,
                 oversubtraction: float = 2.0,
                 spectral_floor: float = 0.01):
        """
        Args:
            sample_rate: Audio sample rate in Hz.
            fft_size: FFT window size in samples.
            hop_size: Hop size in samples (overlap = fft_size - hop_size).
            hover_frequency: Motor fundamental frequency in Hz.
            num_harmonics: Number of harmonics to remove.
            oversubtraction: Alpha factor for aggressive noise estimation.
            spectral_floor: Minimum gain to prevent musical noise artifacts.
        """
        self.sample_rate = sample_rate
        self.fft_size = fft_size
        self.hop_size = hop_size
        self.hover_frequency = hover_frequency
        self.num_harmonics = num_harmonics
        self.oversubtraction = oversubtraction
        self.spectral_floor = spectral_floor

        self._window = np.hanning(fft_size)
        self._freq_resolution = sample_rate / fft_size
        self._noise_mask = self._build_harmonic_mask()

    def _build_harmonic_mask(self) -> np.ndarray:
        """
        Build a binary mask marking frequency bins corresponding to
        motor harmonics (±2 bins for spectral leakage).
        """
        num_bins = self.fft_size // 2 + 1
        mask = np.zeros(num_bins)

        for h in range(1, self.num_harmonics + 1):
            freq = self.hover_frequency * h
            bin_idx = int(round(freq / self._freq_resolution))
            if bin_idx >= num_bins:
                break
            spread = 2  # ±2 bins for spectral leakage
            lo = max(0, bin_idx - spread)
            hi = min(num_bins, bin_idx + spread + 1)
            mask[lo:hi] = 1.0

        return mask

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Apply spectral subtraction to a single frame.

        Args:
            frame: Input audio frame, shape (fft_size,).

        Returns:
            Cleaned frame, shape (fft_size,).
        """
        windowed = frame * self._window
        spectrum = np.fft.rfft(windowed, n=self.fft_size)

        magnitude = np.abs(spectrum)
        phase = np.angle(spectrum)

        noise_estimate = magnitude * self._noise_mask * self.oversubtraction

        cleaned_magnitude = magnitude - noise_estimate
        cleaned_magnitude = np.maximum(cleaned_magnitude,
                                       magnitude * self.spectral_floor)

        cleaned_spectrum = cleaned_magnitude * np.exp(1j * phase)
        cleaned_frame = np.fft.irfft(cleaned_spectrum, n=self.fft_size)

        return cleaned_frame

    def process_multichannel(self, signals: np.ndarray) -> np.ndarray:
        """
        Apply spectral subtraction to all channels using overlap-add.

        Args:
            signals: Multichannel audio, shape (num_channels, num_samples).

        Returns:
            Cleaned signals, same shape as input.
        """
        num_channels, num_samples = signals.shape
        output = np.zeros_like(signals)

        num_frames = (num_samples - self.fft_size) // self.hop_size + 1

        for ch in range(num_channels):
            for f in range(num_frames):
                start = f * self.hop_size
                frame = signals[ch, start:start + self.fft_size]
                cleaned = self.process_frame(frame)
                output[ch, start:start + self.fft_size] += cleaned

            # Normalize overlap-add regions
            norm = np.zeros(num_samples)
            for f in range(num_frames):
                start = f * self.hop_size
                norm[start:start + self.fft_size] += self._window ** 2
            norm[norm < 1e-10] = 1.0
            output[ch] /= norm

        return output

    def update_hover_frequency(self, new_freq: float):
        """Update motor frequency (e.g., from RPM telemetry) and rebuild mask."""
        self.hover_frequency = new_freq
        self._noise_mask = self._build_harmonic_mask()
