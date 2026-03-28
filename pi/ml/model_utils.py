"""
Shared ML utilities for audio model inference on Raspberry Pi.

Provides TFLite model loading, mel-spectrogram computation,
audio resampling, and model file management.
"""

import numpy as np
import os
from pathlib import Path
from typing import Optional, Tuple


def load_tflite_model(model_path: str):
    """
    Load a TFLite model, preferring tflite_runtime for Pi deployment.

    Returns:
        TFLite Interpreter instance.
    """
    try:
        from tflite_runtime.interpreter import Interpreter
    except ImportError:
        from tensorflow.lite.python.interpreter import Interpreter

    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter


def tflite_inference(interpreter, input_data: np.ndarray) -> np.ndarray:
    """
    Run inference on a loaded TFLite interpreter.

    Args:
        interpreter: TFLite Interpreter with allocated tensors.
        input_data: Input array matching the model's expected shape and dtype.

    Returns:
        Output array from the model.
    """
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    expected_dtype = input_details[0]['dtype']
    input_data = input_data.astype(expected_dtype)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    return interpreter.get_tensor(output_details[0]['index'])


def compute_mel_spectrogram(audio: np.ndarray,
                            sample_rate: int = 16000,
                            n_fft: int = 512,
                            hop_length: int = 160,
                            n_mels: int = 64,
                            fmin: float = 60.0,
                            fmax: float = 7800.0) -> np.ndarray:
    """
    Compute log-mel spectrogram suitable for YAMNet-style models.

    Args:
        audio: Mono audio signal, shape (num_samples,), float32 in [-1, 1].
        sample_rate: Audio sample rate.
        n_fft: FFT window size.
        hop_length: Hop size in samples.
        n_mels: Number of mel filter banks.
        fmin: Minimum frequency for mel filters.
        fmax: Maximum frequency for mel filters.

    Returns:
        Log-mel spectrogram, shape (num_frames, n_mels), float32.
    """
    mel_basis = _mel_filterbank(sample_rate, n_fft, n_mels, fmin, fmax)

    num_frames = 1 + (len(audio) - n_fft) // hop_length
    spectrogram = np.zeros((num_frames, n_fft // 2 + 1))

    window = np.hanning(n_fft)
    for i in range(num_frames):
        start = i * hop_length
        frame = audio[start:start + n_fft] * window
        spectrogram[i] = np.abs(np.fft.rfft(frame, n=n_fft)) ** 2

    mel_spectrogram = spectrogram @ mel_basis.T
    mel_spectrogram = np.maximum(mel_spectrogram, 1e-10)
    log_mel = np.log(mel_spectrogram)

    return log_mel.astype(np.float32)


def resample_audio(audio: np.ndarray,
                   orig_sr: int,
                   target_sr: int) -> np.ndarray:
    """
    Resample audio to a target sample rate.

    Uses scipy for resampling. Falls back to simple linear interpolation
    if scipy is unavailable.
    """
    if orig_sr == target_sr:
        return audio

    try:
        import resampy
        return resampy.resample(audio.astype(np.float32), orig_sr, target_sr)
    except ImportError:
        from scipy.signal import resample
        num_samples = int(len(audio) * target_sr / orig_sr)
        return resample(audio, num_samples).astype(np.float32)


def _mel_filterbank(sample_rate: int, n_fft: int, n_mels: int,
                    fmin: float, fmax: float) -> np.ndarray:
    """Build mel filterbank matrix, shape (n_mels, n_fft // 2 + 1)."""
    n_bins = n_fft // 2 + 1

    def hz_to_mel(hz):
        return 2595.0 * np.log10(1.0 + hz / 700.0)

    def mel_to_hz(mel):
        return 700.0 * (10.0 ** (mel / 2595.0) - 1.0)

    mel_min = hz_to_mel(fmin)
    mel_max = hz_to_mel(fmax)
    mel_points = np.linspace(mel_min, mel_max, n_mels + 2)
    hz_points = mel_to_hz(mel_points)

    bin_points = np.floor((n_fft + 1) * hz_points / sample_rate).astype(int)
    bin_points = np.clip(bin_points, 0, n_bins - 1)

    filterbank = np.zeros((n_mels, n_bins))

    for m in range(n_mels):
        left = bin_points[m]
        center = bin_points[m + 1]
        right = bin_points[m + 2]

        if center > left:
            filterbank[m, left:center] = (
                np.arange(left, center) - left
            ) / (center - left)
        if right > center:
            filterbank[m, center:right] = (
                right - np.arange(center, right)
            ) / (right - center)

    return filterbank


def get_model_dir() -> Path:
    """Get the models directory path, create if needed."""
    model_dir = Path(__file__).parent.parent / "models"
    model_dir.mkdir(exist_ok=True)
    return model_dir
