"""
YAMNet-based sound classifier for human distress detection.

Runs TFLite inference on the center microphone (CH6) to classify audio
events. Acts as a gate: GCC-PHAT only runs when human-relevant sounds
are detected (Scream, Shout, Cry, Speech), preventing false DOA from
wind, birds, or mechanical noise.
"""

import numpy as np
import logging
from pathlib import Path
from typing import Tuple, List, Optional

from .model_utils import (
    load_tflite_model,
    tflite_inference,
    compute_mel_spectrogram,
    resample_audio,
    get_model_dir,
)

logger = logging.getLogger(__name__)

YAMNET_SAMPLE_RATE = 16000
YAMNET_PATCH_FRAMES = 96
YAMNET_PATCH_BANDS = 64
YAMNET_PATCH_HOP = 48

DEFAULT_HUMAN_CLASSES = [
    "Speech",
    "Shout",
    "Screaming",
    "Crying, sobbing",
    "Whimper",
    "Groan",
    "Yell",
]

# YAMNet class name to index mapping (AudioSet ontology, 521 classes)
# These are the indices in YAMNet's output tensor for human-relevant sounds
YAMNET_HUMAN_CLASS_INDICES = {
    "Speech": 0,
    "Shout": 14,
    "Screaming": 15,
    "Crying, sobbing": 25,
    "Whimper": 27,
    "Groan": 28,
    "Yell": 13,
}


class SoundClassifier:
    """YAMNet-based audio event classifier for human sound detection."""

    def __init__(self,
                 model_path: str = None,
                 confidence_threshold: float = 0.3,
                 human_classes: List[str] = None,
                 input_sample_rate: int = 16000):
        """
        Args:
            model_path: Path to yamnet.tflite. Defaults to models/yamnet.tflite.
            confidence_threshold: Minimum probability to trigger detection.
            human_classes: List of YAMNet class names considered "human sound".
            input_sample_rate: Sample rate of input audio.
        """
        if model_path is None:
            model_path = str(get_model_dir() / "yamnet.tflite")

        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.human_classes = human_classes or DEFAULT_HUMAN_CLASSES
        self.input_sample_rate = input_sample_rate

        self._interpreter = None
        self._initialized = False

        self._human_indices = [
            YAMNET_HUMAN_CLASS_INDICES.get(cls)
            for cls in self.human_classes
            if cls in YAMNET_HUMAN_CLASS_INDICES
        ]

    def initialize(self) -> bool:
        """
        Load the YAMNet TFLite model.

        Returns:
            True if model loaded, False if model file not found.
        """
        if not Path(self.model_path).exists():
            logger.warning(
                "YAMNet model not found at '%s'. Classifier disabled — "
                "all audio will be treated as potential human sound.",
                self.model_path
            )
            return False

        try:
            self._interpreter = load_tflite_model(self.model_path)
            self._initialized = True
            logger.info("YAMNet classifier loaded from %s", self.model_path)
            return True
        except Exception as e:
            logger.error("Failed to load YAMNet: %s", e)
            return False

    def classify(self, audio: np.ndarray) -> Tuple[bool, str, float]:
        """
        Classify an audio segment for human sound content.

        Args:
            audio: Mono audio from center mic, shape (num_samples,),
                   float32 normalized to [-1, 1].

        Returns:
            (is_human, top_class, confidence):
                is_human — True if a human-relevant class exceeds threshold.
                top_class — Name of the highest-scoring human class.
                confidence — Probability of that class.
        """
        if not self._initialized:
            # Fail-open: treat everything as potential human sound
            return True, "Unknown", 1.0

        if self.input_sample_rate != YAMNET_SAMPLE_RATE:
            audio = resample_audio(audio, self.input_sample_rate,
                                   YAMNET_SAMPLE_RATE)

        audio = audio.astype(np.float32)
        if np.max(np.abs(audio)) > 1.0:
            audio = audio / (np.max(np.abs(audio)) + 1e-10)

        mel_spec = compute_mel_spectrogram(
            audio,
            sample_rate=YAMNET_SAMPLE_RATE,
            n_fft=512,
            hop_length=160,
            n_mels=YAMNET_PATCH_BANDS,
        )

        patches = self._extract_patches(mel_spec)
        if len(patches) == 0:
            return False, "None", 0.0

        all_scores = []
        for patch in patches:
            input_data = patch[np.newaxis, :, :].astype(np.float32)
            scores = tflite_inference(self._interpreter, input_data)
            all_scores.append(scores.flatten())

        avg_scores = np.mean(all_scores, axis=0)

        return self._check_human_sound(avg_scores)

    def _extract_patches(self, mel_spec: np.ndarray) -> List[np.ndarray]:
        """Extract fixed-size patches from mel spectrogram for YAMNet input."""
        num_frames = mel_spec.shape[0]
        patches = []

        if num_frames < YAMNET_PATCH_FRAMES:
            padded = np.zeros((YAMNET_PATCH_FRAMES, YAMNET_PATCH_BANDS),
                              dtype=np.float32)
            padded[:num_frames] = mel_spec
            patches.append(padded)
        else:
            for start in range(0, num_frames - YAMNET_PATCH_FRAMES + 1,
                               YAMNET_PATCH_HOP):
                patch = mel_spec[start:start + YAMNET_PATCH_FRAMES]
                patches.append(patch)

        return patches

    def _check_human_sound(self,
                           scores: np.ndarray) -> Tuple[bool, str, float]:
        """
        Check if any human-relevant class exceeds the confidence threshold.
        """
        best_class = "None"
        best_confidence = 0.0

        for cls_name, idx in YAMNET_HUMAN_CLASS_INDICES.items():
            if cls_name not in self.human_classes:
                continue
            if idx is not None and idx < len(scores):
                if scores[idx] > best_confidence:
                    best_confidence = float(scores[idx])
                    best_class = cls_name

        is_human = best_confidence >= self.confidence_threshold
        return is_human, best_class, best_confidence

    @property
    def is_available(self) -> bool:
        return self._initialized
