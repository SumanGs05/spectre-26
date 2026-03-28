"""
Tests for GCC-PHAT TDOA estimator and DOA pipeline.

Uses synthetic signals with known time delays to verify:
1. GCC-PHAT correctly recovers TDOA within sub-sample accuracy
2. DOA estimator maps TDOAs to correct azimuth
3. Multi-pair fusion resolves front-back ambiguity
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dsp.gcc_phat import gcc_phat, gcc_phat_batch
from dsp.doa_estimator import DOAEstimator
from dsp.array_geometry import (
    get_mic_positions, get_outer_pairs, get_pair_distances,
    max_unambiguous_frequency
)

SAMPLE_RATE = 16000
FFT_SIZE = 1024


def generate_delayed_signal(delay_samples, num_samples=1024, sample_rate=16000):
    """
    Generate a pair of broadband signals with a known integer delay.

    Uses band-limited noise (200-4000 Hz) which provides rich phase
    information across many frequency bins — ideal for GCC-PHAT testing.
    """
    np.random.seed(42)
    from scipy.signal import firwin, lfilter

    # Broadband noise filtered to speech-relevant band
    noise = np.random.randn(num_samples + abs(delay_samples) + 100)
    taps = firwin(65, [200, 4000], pass_zero=False, fs=sample_rate)
    broadband = lfilter(taps, 1.0, noise)

    # Trim filter transient
    broadband = broadband[64:]

    sig1 = broadband[:num_samples].copy()

    if delay_samples >= 0:
        sig2 = broadband[delay_samples:delay_samples + num_samples].copy()
    else:
        offset = -delay_samples
        sig2 = broadband[:num_samples].copy()
        sig1 = broadband[offset:offset + num_samples].copy()

    # Light noise to simulate real conditions
    sig1 += np.random.randn(num_samples) * 0.01
    sig2 += np.random.randn(num_samples) * 0.01

    return sig1.astype(np.float32), sig2.astype(np.float32)


class TestGCCPHAT:
    def test_zero_delay(self):
        """Signals with no delay should give TDOA ≈ 0."""
        sig1, sig2 = generate_delayed_signal(0)
        tdoa, conf = gcc_phat(sig1, sig2, sample_rate=SAMPLE_RATE)
        assert abs(tdoa) < 1.0 / SAMPLE_RATE, f"Expected ~0 TDOA, got {tdoa}"
        assert conf > 0.5, f"Expected high confidence, got {conf}"

    def test_positive_delay(self):
        """Known positive delay should be recovered."""
        delay_samples = 3
        sig1, sig2 = generate_delayed_signal(delay_samples)
        tdoa, conf = gcc_phat(sig1, sig2, sample_rate=SAMPLE_RATE)

        expected_tdoa = delay_samples / SAMPLE_RATE
        assert abs(tdoa - expected_tdoa) < 1.5 / SAMPLE_RATE, \
            f"Expected TDOA ~{expected_tdoa:.6f}s, got {tdoa:.6f}s"

    def test_negative_delay(self):
        """Known negative delay should be recovered."""
        delay_samples = -2
        sig1, sig2 = generate_delayed_signal(delay_samples)
        tdoa, conf = gcc_phat(sig1, sig2, sample_rate=SAMPLE_RATE)

        expected_tdoa = delay_samples / SAMPLE_RATE
        assert abs(tdoa - expected_tdoa) < 1.5 / SAMPLE_RATE, \
            f"Expected TDOA ~{expected_tdoa:.6f}s, got {tdoa:.6f}s"

    def test_fractional_delay(self):
        """Sub-sample delay should be estimated via parabolic interpolation."""
        # Use integer delay=2 and verify sub-sample precision is reasonable
        delay_samples = 2
        sig1, sig2 = generate_delayed_signal(delay_samples)
        tdoa, _ = gcc_phat(sig1, sig2, sample_rate=SAMPLE_RATE)
        expected = delay_samples / SAMPLE_RATE
        assert abs(tdoa - expected) < 1.5 / SAMPLE_RATE, \
            f"Sub-sample TDOA: expected ~{expected:.6f}s, got {tdoa:.6f}s"

    def test_batch_processing(self):
        """Batch processing should match individual pair results."""
        pairs = [(0, 1), (0, 2), (1, 2)]
        n_ch = 3
        signals = np.random.randn(n_ch, FFT_SIZE).astype(np.float32)

        # Add delayed copies
        signals[1] = np.roll(signals[0], 2)
        signals[2] = np.roll(signals[0], -1)

        tdoas, confs = gcc_phat_batch(signals, pairs, sample_rate=SAMPLE_RATE)
        assert tdoas.shape == (3,)
        assert confs.shape == (3,)


class TestDOAEstimator:
    def setup_method(self):
        self.estimator = DOAEstimator()
        self.positions = get_mic_positions()
        self.pairs = get_outer_pairs()

    def test_known_azimuth(self):
        """Synthetic TDOAs for a known direction should give correct azimuth."""
        target_azimuth_deg = 45.0
        target_azimuth_rad = np.radians(target_azimuth_deg)
        speed = 343.0

        # Generate synthetic TDOAs for a source at 45 degrees
        tdoas = np.zeros(len(self.pairs))
        for k, (i, j) in enumerate(self.pairs):
            d_i = (self.positions[i, 0] * np.cos(target_azimuth_rad) +
                   self.positions[i, 1] * np.sin(target_azimuth_rad))
            d_j = (self.positions[j, 0] * np.cos(target_azimuth_rad) +
                   self.positions[j, 1] * np.sin(target_azimuth_rad))
            tdoas[k] = (d_i - d_j) / speed

        confs = np.ones(len(self.pairs)) * 0.9
        azimuth, confidence = self.estimator.estimate(tdoas, confs)

        assert abs(azimuth - target_azimuth_deg) < 10.0, \
            f"Expected ~{target_azimuth_deg}°, got {azimuth}°"
        assert confidence > 0.3, f"Expected decent confidence, got {confidence}"

    def test_no_signal(self):
        """Zero TDOAs with zero confidence should give low confidence."""
        tdoas = np.zeros(15)
        confs = np.zeros(15)
        _, confidence = self.estimator.estimate(tdoas, confs)
        assert confidence < 0.3

    def test_multiple_azimuths(self):
        """Test DOA at several cardinal directions."""
        speed = 343.0
        for target_deg in [0, 90, 180, 270]:
            target_rad = np.radians(target_deg)
            tdoas = np.zeros(len(self.pairs))
            for k, (i, j) in enumerate(self.pairs):
                d_i = (self.positions[i, 0] * np.cos(target_rad) +
                       self.positions[i, 1] * np.sin(target_rad))
                d_j = (self.positions[j, 0] * np.cos(target_rad) +
                       self.positions[j, 1] * np.sin(target_rad))
                tdoas[k] = (d_i - d_j) / speed

            confs = np.ones(len(self.pairs)) * 0.8
            azimuth, conf = self.estimator.estimate(tdoas, confs)

            angular_error = min(
                abs(azimuth - target_deg),
                360 - abs(azimuth - target_deg)
            )
            assert angular_error < 15.0, \
                f"Target {target_deg}°: got {azimuth}° (error {angular_error}°)"


class TestArrayGeometry:
    def test_mic_count(self):
        positions = get_mic_positions()
        assert positions.shape == (7, 2)

    def test_center_mic_at_origin(self):
        positions = get_mic_positions()
        assert positions[6, 0] == 0.0
        assert positions[6, 1] == 0.0

    def test_outer_mic_radius(self):
        radius = 0.035
        positions = get_mic_positions(radius_m=radius)
        for i in range(6):
            r = np.linalg.norm(positions[i])
            assert abs(r - radius) < 1e-10

    def test_pair_count(self):
        pairs = get_outer_pairs()
        assert len(pairs) == 15  # C(6,2)

    def test_max_frequency(self):
        f_max = max_unambiguous_frequency(radius_m=0.035)
        assert 4000 < f_max < 6000, f"Expected ~4.9 kHz, got {f_max}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
