"""
Tests for the RNNoise denoiser wrapper.

Since librnnoise may not be available in CI/test environments, these tests
verify the wrapper's graceful degradation and API correctness.
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ml.denoiser import RNNoiseDenoiser, RNNOISE_FRAME_SIZE


class TestRNNoiseDenoiser:
    def test_initialization_without_library(self):
        """Denoiser should handle missing librnnoise gracefully."""
        denoiser = RNNoiseDenoiser(
            num_channels=7,
            library_path="nonexistent_library"
        )
        result = denoiser.initialize()
        assert result is False
        assert denoiser.is_available is False

    def test_passthrough_when_unavailable(self):
        """When library is unavailable, audio should pass through unchanged."""
        denoiser = RNNoiseDenoiser(num_channels=7)
        # Don't call initialize — should pass through

        signals = np.random.randn(7, 1024).astype(np.float32)
        output = denoiser.denoise_multichannel(signals)

        np.testing.assert_array_equal(output, signals)

    def test_single_frame_passthrough(self):
        """denoise_frame should return input when not initialized."""
        denoiser = RNNoiseDenoiser(num_channels=1)

        frame = np.random.randn(RNNOISE_FRAME_SIZE).astype(np.float32)
        output = denoiser.denoise_frame(frame, channel=0)

        np.testing.assert_array_equal(output, frame)

    def test_multichannel_shape_preservation(self):
        """Output shape should match input shape."""
        denoiser = RNNoiseDenoiser(num_channels=7)

        signals = np.random.randn(7, 2048).astype(np.float32)
        output = denoiser.denoise_multichannel(signals)

        assert output.shape == signals.shape

    def test_cleanup_without_init(self):
        """Cleanup should be safe even without initialization."""
        denoiser = RNNoiseDenoiser(num_channels=7)
        denoiser.cleanup()  # Should not raise


class TestRNNoiseAPI:
    def test_frame_size_constant(self):
        assert RNNOISE_FRAME_SIZE == 480

    def test_default_parameters(self):
        denoiser = RNNoiseDenoiser()
        assert denoiser.num_channels == 7
        assert denoiser.input_sample_rate == 16000

    def test_custom_parameters(self):
        denoiser = RNNoiseDenoiser(
            num_channels=2,
            input_sample_rate=48000,
            library_path="/custom/path"
        )
        assert denoiser.num_channels == 2
        assert denoiser.input_sample_rate == 48000


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
