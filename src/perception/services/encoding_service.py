"""Encoding service skeleton — thin pass-through to the encoder backend."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


class EncodingService:
    """Encapsulates image/text encoding calls.

    Phase 2 behavior: direct delegate to the encoder backend.  Future phases
    can add batching, caching policy and feature-dimension normalization here.
    """

    def __init__(self, encoder: Any | None = None) -> None:
        self._encoder = encoder

    @property
    def encoder(self) -> Any | None:
        return self._encoder

    @encoder.setter
    def encoder(self, value: Any | None) -> None:
        self._encoder = value

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        """Encode a single image crop into a feature vector."""
        if self._encoder is None:
            return np.array([])

        try:
            return self._encoder.encode_image(image)
        except Exception as e:
            logger.warning("EncodingService encode_image() failed: %s", e)
            return np.array([])

    def encode_text(self, text: str | list[str]) -> np.ndarray:
        """Encode text into a normalized feature vector or matrix.

        Accepts either a single string or a list of strings and normalises the
        return value to a 1-D vector for the single-string case.
        """
        if self._encoder is None:
            return np.array([])

        try:
            if isinstance(text, str):
                result = self._encoder.encode_text([text])
                if result is None or result.size == 0:
                    return np.array([])
                if result.ndim == 2:
                    return result[0]
                return result
            return self._encoder.encode_text(text)
        except Exception as e:
            logger.warning("EncodingService encode_text() failed: %s", e)
            return np.array([])

    def text_image_similarity(
        self,
        text: str,
        image_features: list[np.ndarray],
    ) -> list[float]:
        """Compatibility shim used by goal resolution paths."""
        if self._encoder is None:
            return [0.0] * len(image_features)

        if hasattr(self._encoder, "text_image_similarity"):
            try:
                return self._encoder.text_image_similarity(text, image_features)
            except Exception as e:
                logger.warning("text_image_similarity failed: %s", e)

        text_feat = self.encode_text(text)
        if text_feat.size == 0:
            return [0.0] * len(image_features)

        return [
            float(max(0.0, np.dot(text_feat, img_feat))) if img_feat.size > 0 else 0.0 for img_feat in image_features
        ]
