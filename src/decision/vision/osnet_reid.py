"""OSNet Re-ID encoder — BPU .hbm on S100P, torchreid fallback on dev machines.

This module provides a single unified interface:
    encoder = OSNetReIDEncoder()
    feat = encoder.encode(crop_rgb)  # np.ndarray (512,) L2-normalised

Loading strategy (in order):
  1. Nash BPU via bpu_infer library: loads /opt/lingtu/models/osnet_x1_0_128x256_nashe.hbm
  2. torchreid CPU/GPU: loads osnet_x1_0 pretrained weights
  3. No silent fallback — RuntimeError if neither is available.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# Model configuration constants
_BPU_MODEL_PATH = "/opt/lingtu/models/osnet_x1_0_128x256_nashe.hbm"
_OSNET_INPUT_H = 256
_OSNET_INPUT_W = 128
_OSNET_FEAT_DIM = 512


def _try_load_bpu(model_path: str) -> object | None:
    """Attempt to load the BPU .hbm model via bpu_infer.

    Returns the loaded BPU model handle, or None if bpu_infer is unavailable.
    Raises RuntimeError if the model file exists but fails to load.
    """
    try:
        import bpu_infer  # type: ignore  # S100P-only binding
    except ImportError:
        return None

    if not Path(model_path).exists():
        return None

    try:
        model = bpu_infer.load(model_path)
        logger.info("OSNetReIDEncoder: BPU model loaded from %s", model_path)
        return model
    except Exception as exc:
        raise RuntimeError(
            f"OSNetReIDEncoder: BPU model at {model_path} exists but failed to load: {exc}"
        ) from exc


def _try_load_torchreid() -> object | None:
    """Attempt to load OSNet via torchreid (for dev/non-BPU machines).

    Returns a callable extractor, or None if torchreid is unavailable.
    """
    try:
        import torchreid  # type: ignore
    except ImportError:
        return None

    try:
        import torch
        extractor = torchreid.utils.FeatureExtractor(
            model_name="osnet_x1_0",
            model_path="",  # use pretrained weights
            device="cuda" if torch.cuda.is_available() else "cpu",
        )
        logger.info("OSNetReIDEncoder: torchreid OSNet loaded (device=%s)",
                    "cuda" if torch.cuda.is_available() else "cpu")
        return extractor
    except Exception as exc:
        raise RuntimeError(
            f"OSNetReIDEncoder: torchreid available but OSNet failed to load: {exc}"
        ) from exc


class OSNetReIDEncoder:
    """OSNet Re-ID feature extractor.

    Provides a single encode() method that returns a 512-dim L2-normalised
    appearance feature vector for a person crop.

    Backend selection:
      - S100P: Nash BPU via bpu_infer (.hbm model)
      - Dev machine: torchreid (osnet_x1_0 pretrained)
      - Missing both: RuntimeError — no silent fallback.

    Raises:
        RuntimeError: if neither BPU nor torchreid is available.
    """

    def __init__(self, bpu_model_path: str = _BPU_MODEL_PATH) -> None:
        self._bpu_model = None
        self._torchreid_extractor = None
        self._backend: str = ""

        # Try BPU first (S100P production path)
        self._bpu_model = _try_load_bpu(bpu_model_path)
        if self._bpu_model is not None:
            self._backend = "bpu"
            return

        # Try torchreid (dev machine path)
        self._torchreid_extractor = _try_load_torchreid()
        if self._torchreid_extractor is not None:
            self._backend = "torchreid"
            return

        raise RuntimeError(
            "OSNetReIDEncoder: no backend available. "
            "On S100P: ensure bpu_infer is installed and "
            f"{bpu_model_path} exists. "
            "On dev machine: install torchreid (`pip install torchreid`)."
        )

    @property
    def backend(self) -> str:
        """Active backend: 'bpu' or 'torchreid'."""
        return self._backend

    @property
    def feature_dim(self) -> int:
        """Dimension of the output feature vector."""
        return _OSNET_FEAT_DIM

    def encode(self, crop_rgb: np.ndarray) -> np.ndarray:
        """Extract a 512-dim L2-normalised Re-ID feature from a person crop.

        Args:
            crop_rgb: RGB image crop as np.ndarray, shape (H, W, 3), dtype uint8.

        Returns:
            np.ndarray of shape (512,), L2-normalised, dtype float32.

        Raises:
            ValueError: if crop_rgb is not a valid 3-channel image.
            RuntimeError: if inference fails unexpectedly.
        """
        if crop_rgb is None or crop_rgb.ndim != 3 or crop_rgb.shape[2] != 3:
            raise ValueError(
                f"OSNetReIDEncoder.encode: expected (H, W, 3) RGB array, "
                f"got shape {getattr(crop_rgb, 'shape', None)}"
            )

        # Resize to OSNet input resolution
        resized = self._resize(crop_rgb)

        if self._backend == "bpu":
            feat = self._encode_bpu(resized)
        elif self._backend == "torchreid":
            feat = self._encode_torchreid(resized)
        else:
            raise RuntimeError("OSNetReIDEncoder: no active backend")

        # L2 normalise
        norm = float(np.linalg.norm(feat))
        if norm > 1e-9:
            feat = feat / norm
        return feat.astype(np.float32)

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _resize(crop_rgb: np.ndarray) -> np.ndarray:
        """Resize crop to (_OSNET_INPUT_H, _OSNET_INPUT_W) using nearest-neighbour.

        Uses PIL if available (better quality), falls back to numpy slicing.
        """
        target_h, target_w = _OSNET_INPUT_H, _OSNET_INPUT_W
        h, w = crop_rgb.shape[:2]
        if h == target_h and w == target_w:
            return crop_rgb

        try:
            from PIL import Image  # type: ignore
            pil = Image.fromarray(crop_rgb)
            pil = pil.resize((target_w, target_h), Image.BILINEAR)
            return np.asarray(pil)
        except ImportError:
            pass

        # Numpy fallback: basic bilinear-ish via index mapping
        row_idx = (np.arange(target_h) * h / target_h).astype(int).clip(0, h - 1)
        col_idx = (np.arange(target_w) * w / target_w).astype(int).clip(0, w - 1)
        return crop_rgb[np.ix_(row_idx, col_idx)]

    def _encode_bpu(self, img: np.ndarray) -> np.ndarray:
        """Run BPU inference and return raw 512-dim float32 feature."""
        try:
            # bpu_infer API: model.forward(input_array) -> output_array
            output = self._bpu_model.forward(img.astype(np.uint8))
            # Output may be wrapped in a list/tuple; unwrap
            if isinstance(output, (list, tuple)):
                output = output[0]
            feat = np.asarray(output, dtype=np.float32).ravel()
            if feat.size != _OSNET_FEAT_DIM:
                raise RuntimeError(
                    f"BPU model returned {feat.size}-dim feature, expected {_OSNET_FEAT_DIM}"
                )
            return feat
        except Exception as exc:
            raise RuntimeError(f"OSNetReIDEncoder: BPU inference failed: {exc}") from exc

    def _encode_torchreid(self, img: np.ndarray) -> np.ndarray:
        """Run torchreid inference and return raw 512-dim float32 feature."""
        try:
            import torch
            # torchreid FeatureExtractor expects (H, W, C) uint8 or a list thereof
            with torch.no_grad():
                output = self._torchreid_extractor(img)
            feat = np.asarray(output, dtype=np.float32).ravel()
            if feat.size != _OSNET_FEAT_DIM:
                raise RuntimeError(
                    f"torchreid returned {feat.size}-dim feature, expected {_OSNET_FEAT_DIM}"
                )
            return feat
        except Exception as exc:
            raise RuntimeError(f"OSNetReIDEncoder: torchreid inference failed: {exc}") from exc
