"""
Direction-of-Arrival (DOA) estimator for circular microphone array.

Maps TDOA measurements from GCC-PHAT across 15 mic pairs to a single
azimuth angle (0-360 degrees, relative to drone nose) using histogram
fusion with Gaussian smoothing.
"""

import numpy as np
from typing import Tuple, Optional

from .array_geometry import (
    DEFAULT_POSITIONS, DEFAULT_PAIRS, DEFAULT_DISTANCES,
    DEFAULT_PAIR_ANGLES, get_pair_distances, get_pair_angles
)


class DOAEstimator:
    def __init__(self,
                 positions: np.ndarray = None,
                 pairs: list = None,
                 speed_of_sound: float = 343.0,
                 histogram_bins: int = 360,
                 histogram_sigma: float = 5.0,
                 min_confidence: float = 0.3):
        """
        Args:
            positions: Mic positions, shape (N, 2). Defaults to Sipeed 6+1.
            pairs: List of (i, j) pairs. Defaults to 15 outer-ring pairs.
            speed_of_sound: Speed of sound in m/s.
            histogram_bins: Angular resolution (360 = 1 degree per bin).
            histogram_sigma: Gaussian smoothing width in degrees.
            min_confidence: Minimum confidence to report a valid detection.
        """
        self.positions = positions if positions is not None else DEFAULT_POSITIONS
        self.pairs = pairs if pairs is not None else DEFAULT_PAIRS
        self.speed_of_sound = speed_of_sound
        self.histogram_bins = histogram_bins
        self.histogram_sigma = histogram_sigma
        self.min_confidence = min_confidence

        self.distances = get_pair_distances(self.positions, self.pairs)
        self.pair_angles = get_pair_angles(self.positions, self.pairs)

        self._angle_axis = np.linspace(0, 360, histogram_bins, endpoint=False)
        self._angle_axis_rad = np.deg2rad(self._angle_axis)

    def estimate(self,
                 tdoas: np.ndarray,
                 gcc_confidences: np.ndarray) -> Tuple[float, float]:
        """
        Estimate DOA azimuth from TDOA measurements.

        Args:
            tdoas: TDOA values in seconds, shape (num_pairs,).
            gcc_confidences: GCC-PHAT peak values, shape (num_pairs,).

        Returns:
            (azimuth_deg, confidence):
                azimuth_deg — 0-360 degrees (0 = drone nose, CW positive).
                confidence — 0.0 to 1.0, reliability of the estimate.
        """
        histogram = np.zeros(self.histogram_bins)

        for k, (i, j) in enumerate(self.pairs):
            pair_azimuth = self._tdoa_to_azimuth(tdoas[k], k)
            if pair_azimuth is None:
                continue

            weight = gcc_confidences[k]
            self._add_to_histogram(histogram, pair_azimuth, weight)

        if np.max(histogram) < 1e-10:
            return 0.0, 0.0

        histogram = self._smooth_histogram(histogram)

        peak_bin = np.argmax(histogram)
        azimuth_deg = self._angle_axis[peak_bin]

        confidence = self._compute_confidence(histogram, peak_bin)

        if confidence < self.min_confidence:
            return azimuth_deg, confidence

        return azimuth_deg, confidence

    def _tdoa_to_azimuth(self, tdoa: float, pair_idx: int) -> Optional[float]:
        """
        Convert a single TDOA measurement to an azimuth estimate.

        For far-field source at azimuth theta, the TDOA between mic i→j is:
            TDOA = -(r_j - r_i) · u_hat / c = -d * cos(theta - phi) / c
        So: cos(theta - phi) = -TDOA * c / d
        """
        d = self.distances[pair_idx]
        max_cos = d / self.speed_of_sound

        if max_cos < 1e-10:
            return None

        cos_val = -tdoa / max_cos
        cos_val = np.clip(cos_val, -1.0, 1.0)

        # Two candidate azimuths (front-back ambiguity)
        angle_offset = np.arccos(cos_val)
        baseline_angle = self.pair_angles[pair_idx]

        az1 = baseline_angle + angle_offset
        az2 = baseline_angle - angle_offset

        # Return both candidates; histogram fusion resolves ambiguity
        return (np.rad2deg(az1) % 360, np.rad2deg(az2) % 360)

    def _add_to_histogram(self, histogram: np.ndarray,
                          azimuths: Tuple[float, float],
                          weight: float):
        """Add Gaussian-weighted votes at candidate azimuth angles."""
        for az in azimuths:
            bin_idx = int(az / 360 * self.histogram_bins) % self.histogram_bins
            sigma_bins = self.histogram_sigma / 360 * self.histogram_bins
            half_width = int(3 * sigma_bins)

            for offset in range(-half_width, half_width + 1):
                idx = (bin_idx + offset) % self.histogram_bins
                gaussian_weight = np.exp(-0.5 * (offset / sigma_bins) ** 2)
                histogram[idx] += weight * gaussian_weight

    def _smooth_histogram(self, histogram: np.ndarray) -> np.ndarray:
        """Apply circular Gaussian smoothing to the DOA histogram."""
        sigma_bins = self.histogram_sigma / 360 * self.histogram_bins
        kernel_half = int(3 * sigma_bins)
        kernel = np.exp(-0.5 * (np.arange(-kernel_half, kernel_half + 1) / sigma_bins) ** 2)
        kernel /= kernel.sum()

        padded = np.concatenate([histogram[-kernel_half:], histogram, histogram[:kernel_half]])
        smoothed = np.convolve(padded, kernel, mode='valid')

        return smoothed[:self.histogram_bins]

    def _compute_confidence(self, histogram: np.ndarray, peak_bin: int) -> float:
        """
        Compute confidence as peak-to-sidelobe ratio.

        High confidence = sharp single peak. Low confidence = ambiguous
        or multi-modal histogram (multiple sources or noise).
        """
        total = np.sum(histogram)
        if total < 1e-10:
            return 0.0

        peak_val = histogram[peak_bin]

        # Sum energy in a window around the peak (±15 degrees)
        window_bins = int(15 / 360 * self.histogram_bins)
        peak_energy = 0.0
        for offset in range(-window_bins, window_bins + 1):
            idx = (peak_bin + offset) % self.histogram_bins
            peak_energy += histogram[idx]

        return min(1.0, peak_energy / total)
