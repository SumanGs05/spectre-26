"""
GCC-PHAT (Generalized Cross-Correlation with Phase Transform) for TDOA estimation.

Computes time-delay-of-arrival between microphone pairs using frequency-domain
cross-correlation with phase weighting. Includes parabolic interpolation for
sub-sample TDOA resolution.
"""

import numpy as np
from typing import Tuple


def gcc_phat(sig1: np.ndarray, sig2: np.ndarray,
             sample_rate: int = 16000,
             max_delay_samples: int = None) -> Tuple[float, float]:
    """
    Estimate TDOA between two signals using GCC-PHAT.

    Args:
        sig1: First microphone signal, shape (N,).
        sig2: Second microphone signal, shape (N,).
        sample_rate: Sampling rate in Hz.
        max_delay_samples: Maximum expected delay in samples. If None,
            computed from array geometry (defaults to N//2).

    Returns:
        (tdoa_seconds, peak_value):
            tdoa_seconds — estimated delay (positive = sig2 arrives first).
            peak_value — correlation peak height (0-1, usable as confidence).
    """
    n = len(sig1)
    if max_delay_samples is None:
        max_delay_samples = n // 2

    nfft = next_power_of_2(2 * n)

    X1 = np.fft.rfft(sig1, n=nfft)
    X2 = np.fft.rfft(sig2, n=nfft)

    cross_spectrum = X1 * np.conj(X2)

    magnitude = np.abs(cross_spectrum)
    magnitude[magnitude < 1e-10] = 1e-10
    gcc = cross_spectrum / magnitude

    correlation = np.fft.irfft(gcc, n=nfft)

    # Extract the valid delay region: [-max_delay, +max_delay]
    max_d = min(max_delay_samples, nfft // 2)
    lags = np.concatenate([
        correlation[-max_d:],
        correlation[:max_d + 1]
    ])
    lag_indices = np.arange(-max_d, max_d + 1)

    peak_idx = np.argmax(np.abs(lags))
    peak_value = np.abs(lags[peak_idx])

    coarse_delay = lag_indices[peak_idx]

    # Parabolic interpolation for sub-sample precision
    fine_delay = _parabolic_interpolation(lags, peak_idx)
    delay_samples = coarse_delay + fine_delay

    tdoa_seconds = delay_samples / sample_rate

    return tdoa_seconds, peak_value


def gcc_phat_batch(signals: np.ndarray,
                   pairs: list,
                   sample_rate: int = 16000,
                   max_delay_samples: int = None) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute GCC-PHAT TDOA for multiple mic pairs.

    Args:
        signals: Multichannel audio, shape (num_channels, num_samples).
        pairs: List of (i, j) mic index tuples.
        sample_rate: Sampling rate in Hz.
        max_delay_samples: Maximum expected delay in samples.

    Returns:
        (tdoas, confidences):
            tdoas — shape (num_pairs,), TDOA in seconds for each pair.
            confidences — shape (num_pairs,), peak values for each pair.
    """
    num_pairs = len(pairs)
    tdoas = np.zeros(num_pairs)
    confidences = np.zeros(num_pairs)

    for k, (i, j) in enumerate(pairs):
        tdoas[k], confidences[k] = gcc_phat(
            signals[i], signals[j],
            sample_rate=sample_rate,
            max_delay_samples=max_delay_samples
        )

    return tdoas, confidences


def _parabolic_interpolation(data: np.ndarray, peak_idx: int) -> float:
    """
    Parabolic interpolation around a discrete peak for sub-sample accuracy.

    Fits a parabola through (peak-1, peak, peak+1) and returns the
    fractional offset from peak_idx.
    """
    if peak_idx <= 0 or peak_idx >= len(data) - 1:
        return 0.0

    alpha = np.abs(data[peak_idx - 1])
    beta = np.abs(data[peak_idx])
    gamma = np.abs(data[peak_idx + 1])

    denom = 2.0 * (2.0 * beta - alpha - gamma)
    if abs(denom) < 1e-10:
        return 0.0

    return (alpha - gamma) / denom


def next_power_of_2(n: int) -> int:
    """Return the smallest power of 2 >= n."""
    p = 1
    while p < n:
        p <<= 1
    return p
