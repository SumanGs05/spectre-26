"""
Sipeed 6+1 microphone array geometry and pair enumeration.

Physical layout: 6 outer MEMS mics equally spaced on a circle (radius ~35 mm),
1 center mic at the origin. Mic indices 0-5 are outer, index 6 is center.
"""

import numpy as np
from itertools import combinations
from typing import List, Tuple


def get_mic_positions(radius_m: float = 0.035, num_outer: int = 6) -> np.ndarray:
    """
    Compute (x, y) positions for the 6+1 mic array in meters.

    Outer mics are placed at equal angular intervals starting from 0 degrees
    (positive x-axis, aligned with drone nose). Center mic is at origin.

    Returns:
        np.ndarray of shape (7, 2) — [x, y] for each mic.
        Indices 0-5: outer ring, index 6: center.
    """
    positions = np.zeros((num_outer + 1, 2))
    for i in range(num_outer):
        angle = 2 * np.pi * i / num_outer
        positions[i, 0] = radius_m * np.cos(angle)
        positions[i, 1] = radius_m * np.sin(angle)
    # positions[6] = (0, 0) — center mic, already zeros
    return positions


def get_outer_pairs(num_outer: int = 6) -> List[Tuple[int, int]]:
    """
    Enumerate all unique pairs from the outer mic ring.
    C(6,2) = 15 pairs for DOA estimation via GCC-PHAT.

    Returns:
        List of (i, j) tuples where i < j, both in range [0, num_outer).
    """
    return list(combinations(range(num_outer), 2))


def get_pair_distances(positions: np.ndarray,
                       pairs: List[Tuple[int, int]]) -> np.ndarray:
    """
    Compute Euclidean distance between each mic pair.

    Returns:
        np.ndarray of shape (num_pairs,) — inter-mic distances in meters.
    """
    distances = np.array([
        np.linalg.norm(positions[i] - positions[j])
        for i, j in pairs
    ])
    return distances


def get_pair_angles(positions: np.ndarray,
                    pairs: List[Tuple[int, int]]) -> np.ndarray:
    """
    Compute the angle of the baseline vector (mic_j - mic_i) for each pair.

    This is the direction perpendicular to which the pair has maximum
    sensitivity (broadside direction).

    Returns:
        np.ndarray of shape (num_pairs,) — angles in radians.
    """
    angles = np.array([
        np.arctan2(positions[j, 1] - positions[i, 1],
                   positions[j, 0] - positions[i, 0])
        for i, j in pairs
    ])
    return angles


def max_unambiguous_frequency(radius_m: float = 0.035,
                              speed_of_sound: float = 343.0,
                              num_outer: int = 6) -> float:
    """
    Maximum frequency for unambiguous DOA estimation.

    For adjacent mics on the outer ring, the inter-mic distance d must satisfy
    d < lambda/2, i.e. f_max = c / (2 * d).

    For a regular hexagonal array, adjacent distance = radius (for N=6).
    """
    d_adjacent = radius_m  # for hexagonal array, adjacent spacing = radius
    return speed_of_sound / (2 * d_adjacent)


def max_tdoa(distance_m: float, speed_of_sound: float = 343.0) -> float:
    """Maximum possible TDOA (in seconds) for a given mic pair distance."""
    return distance_m / speed_of_sound


# Pre-computed geometry for the default Sipeed 6+1 array
DEFAULT_POSITIONS = get_mic_positions()
DEFAULT_PAIRS = get_outer_pairs()
DEFAULT_DISTANCES = get_pair_distances(DEFAULT_POSITIONS, DEFAULT_PAIRS)
DEFAULT_PAIR_ANGLES = get_pair_angles(DEFAULT_POSITIONS, DEFAULT_PAIRS)

# TODO(migration): mic6 (centre) now arrives from Slave ESP32-S3 board as the
# last channel in the merged 7-ch frame. Index 6 is correct, but the signal
# now passes through the Slave's I2S → float → back-to-int32 path and serves
# as the LMS noise reference for mic4/mic5 on that board. Verify that the
# centre mic signal quality is adequate for YAMNet classification.
CENTER_MIC_INDEX = 6
