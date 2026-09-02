"""Shared test utilities for LingTu framework tests.

Mock backends and helpers that multiple test files need.
No tests here — just importable definitions.
"""

import numpy as np


class _MockDetection:
    def __init__(self):
        self.bbox = np.array([10, 20, 100, 200])
        self.score = 0.9
        self.label = "chair"
        self.mask = None


class _MockDetectorBackend:
    def load_model(self): pass
    def detect(self, rgb, text_prompt):
        return [_MockDetection()]
    def shutdown(self): pass


class _MockEncoderBackend:
    def load_model(self): pass
    def encode_image(self, image):
        return np.random.randn(512).astype(np.float32)
    def encode_text(self, text):
        return np.random.randn(512).astype(np.float32)
    def shutdown(self): pass
