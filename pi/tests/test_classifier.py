"""
Tests for the YAMNet sound classifier wrapper.

Since the YAMNet TFLite model may not be present in test environments,
these tests verify the wrapper's fail-open behavior and API correctness.
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ml.classifier import SoundClassifier, DEFAULT_HUMAN_CLASSES


class TestSoundClassifier:
    def test_initialization_without_model(self):
        """Classifier should handle missing model file gracefully."""
        clf = SoundClassifier(model_path="nonexistent_model.tflite")
        result = clf.initialize()
        assert result is False
        assert clf.is_available is False

    def test_fail_open_behavior(self):
        """When model unavailable, classifier should treat all audio as human."""
        clf = SoundClassifier(model_path="nonexistent_model.tflite")

        audio = np.random.randn(16000).astype(np.float32) * 0.1
        is_human, cls_name, conf = clf.classify(audio)

        assert is_human is True, "Fail-open: should assume human sound"
        assert conf == 1.0
        assert cls_name == "Unknown"

    def test_default_human_classes(self):
        """Default class list should include key distress sounds."""
        assert "Speech" in DEFAULT_HUMAN_CLASSES
        assert "Screaming" in DEFAULT_HUMAN_CLASSES
        assert "Shout" in DEFAULT_HUMAN_CLASSES
        assert "Crying, sobbing" in DEFAULT_HUMAN_CLASSES

    def test_custom_threshold(self):
        clf = SoundClassifier(confidence_threshold=0.8)
        assert clf.confidence_threshold == 0.8

    def test_custom_human_classes(self):
        custom = ["Speech", "Shout"]
        clf = SoundClassifier(human_classes=custom)
        assert clf.human_classes == custom


class TestClassifierCheckLogic:
    def test_check_human_sound_above_threshold(self):
        """Scores above threshold should trigger detection."""
        clf = SoundClassifier(confidence_threshold=0.3)
        # Simulate scores array where Speech (idx 0) has high probability
        scores = np.zeros(521)
        scores[0] = 0.85  # Speech
        scores[15] = 0.1  # Screaming

        is_human, cls_name, conf = clf._check_human_sound(scores)
        assert is_human is True
        assert cls_name == "Speech"
        assert conf == pytest.approx(0.85)

    def test_check_human_sound_below_threshold(self):
        """Scores below threshold should not trigger detection."""
        clf = SoundClassifier(confidence_threshold=0.5)
        scores = np.zeros(521)
        scores[0] = 0.2   # Speech, below threshold

        is_human, cls_name, conf = clf._check_human_sound(scores)
        assert is_human is False

    def test_selects_highest_human_class(self):
        """Should return the human class with highest probability."""
        clf = SoundClassifier(confidence_threshold=0.1)
        scores = np.zeros(521)
        scores[0] = 0.3   # Speech
        scores[15] = 0.7  # Screaming

        is_human, cls_name, conf = clf._check_human_sound(scores)
        assert is_human is True
        assert cls_name == "Screaming"
        assert conf == pytest.approx(0.7)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
