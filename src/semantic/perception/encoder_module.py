"""EncoderModule — pluggable feature encoder as an independent Module.

Wraps any CLIP-family encoder (OpenCLIP, MobileCLIP, SigLIP, DINOv2)
into a core Module. Swapping encoders is a one-line Blueprint change.

Usage::

    bp.add(EncoderModule, encoder="clip", model_name="ViT-B/32")
    bp.add(EncoderModule, encoder="mobileclip")  # edge-optimized
"""

from __future__ import annotations

import logging
import time
from typing import Any

import numpy as np

from core.module import Module
from core.registry import get, list_plugins, register
from core.stream import In, Out

logger = logging.getLogger(__name__)


class FeatureResult:
    """Encoder output: feature vector + metadata."""
    __slots__ = ("encoder_name", "feature", "inference_ms", "timestamp")

    def __init__(self, feature: np.ndarray, timestamp: float = 0.0,
                 encoder_name: str = "", inference_ms: float = 0.0):
        self.feature = feature
        self.timestamp = timestamp or time.time()
        self.encoder_name = encoder_name
        self.inference_ms = inference_ms

    @property
    def dim(self) -> int:
        return self.feature.shape[-1] if self.feature is not None else 0

    def __repr__(self):
        return (f"FeatureResult(dim={self.dim}, "
                f"enc={self.encoder_name}, {self.inference_ms:.1f}ms)")


def _module_attr(module: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        if hasattr(module, name):
            return getattr(module, name)
    return default


class _CLIPEncoderProvider:
    label = "CLIPEncoder"

    @staticmethod
    def create(module):
        from .clip_encoder import CLIPEncoder

        return CLIPEncoder(
            model_name=_module_attr(module, "_model_name", default="ViT-B/32"),
            device=_module_attr(module, "_device", default="auto"),
        )


class _MobileCLIPEncoderProvider:
    label = "MobileCLIPEncoder"

    @staticmethod
    def create(module):
        from .mobileclip_encoder import MobileCLIPEncoder

        return MobileCLIPEncoder(device=_module_attr(module, "_device", default="auto"))


def _register_builtin_encoder_providers() -> None:
    register(
        "encoder",
        "clip",
        description="CLIP image/text encoder",
    )(_CLIPEncoderProvider)
    register(
        "encoder",
        "mobileclip",
        description="MobileCLIP text encoder",
    )(_MobileCLIPEncoderProvider)


_register_builtin_encoder_providers()


@register("encoder", "pluggable", description="Pluggable feature encoder module")
class EncoderModule(Module, layer=3):
    _run_in_worker = True
    _worker_group = "perception"
    """Pluggable feature encoder Module.

    In:  image (np.ndarray)       — BGR uint8 HxWx3
    Out: feature (FeatureResult)  — embedding vector + metadata

    Also supports text encoding via encode_text() method call.
    """

    image: In[np.ndarray]
    feature: Out[FeatureResult]

    def __init__(
        self,
        encoder: str = "clip",
        model_name: str = "ViT-B/32",
        device: str = "auto",
        **kw,
    ):
        super().__init__(**kw)
        self._encoder_name = encoder
        self._model_name = model_name
        self._device = device
        self._encoder_model_name = model_name
        self._encoder_device = device
        self._backend = None
        self._frame_count = 0
        self._total_inference_ms = 0.0

    def setup(self) -> None:
        self._backend = self._create_backend()
        try:
            self._backend.load_model()
            logger.info("EncoderModule: loaded '%s' (%s)",
                        self._encoder_name, self._model_name)
        except Exception:
            logger.exception("EncoderModule: failed to load '%s'", self._encoder_name)
            self._backend = None
        self.image.subscribe(self._on_image)
        # CLIP encode is ~200-500ms on S100P CPU; drop stale frames rather than
        # block the camera publisher (which would starve uvicorn of the GIL).
        self.image.set_policy("latest")

    def _create_backend(self):
        _register_builtin_encoder_providers()
        name = self._encoder_name.lower()
        try:
            provider = get("encoder", name)
        except KeyError:
            available = ", ".join(list_plugins("encoder")) or "<none>"
            raise ValueError(
                f"Unknown encoder backend '{name}'. Available: {available}"
            ) from None
        return provider.create(self)

    def _on_image(self, image: np.ndarray):
        if self._backend is None:
            return
        t0 = time.time()
        try:
            feat = self._backend.encode_image(image)
        except Exception:
            logger.exception("EncoderModule: encoding failed")
            return
        inference_ms = (time.time() - t0) * 1000

        self._frame_count += 1
        self._total_inference_ms += inference_ms

        result = FeatureResult(
            feature=feat,
            timestamp=time.time(),
            encoder_name=self._encoder_name,
            inference_ms=inference_ms,
        )
        self.feature.publish(result)

    def encode_text(self, text: str) -> np.ndarray | None:
        """Synchronous text encoding (for GoalResolver queries)."""
        if self._backend is None:
            return None
        try:
            return self._backend.encode_text(text)
        except Exception:
            logger.exception("EncoderModule: text encoding failed")
            return None

    def stop(self):
        if self._backend and hasattr(self._backend, 'shutdown'):
            try:
                self._backend.shutdown()
            except (AttributeError, TypeError):
                pass
        super().stop()

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        avg_ms = (self._total_inference_ms / self._frame_count
                  if self._frame_count > 0 else 0.0)
        info["encoder"] = {
            "backend": self._encoder_name,
            "model": self._model_name,
            "loaded": self._backend is not None,
            "frames": self._frame_count,
            "avg_ms": round(avg_ms, 1),
        }
        return info
