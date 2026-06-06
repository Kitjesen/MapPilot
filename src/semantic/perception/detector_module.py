"""DetectorModule — pluggable object detector as an independent Module.

Wraps any DetectorBase implementation (YOLO-World, YOLO-E, GroundingDINO,
BPU) into a core Module with typed In/Out ports. Swapping detectors is
a one-line Blueprint change.

Usage::

    from core.blueprints import Blueprint
    from semantic.perception.detector_module import DetectorModule

    # YOLO-World on GPU
    bp.add(DetectorModule, detector="yolo_world", model_size="l")

    # BPU on S100P
    bp.add(DetectorModule, detector="bpu", model_path="/opt/models/yolo.hbm")

    # Swap = change one argument, nothing else touches
"""

from __future__ import annotations

import logging
import time
from typing import Any

from core.module import Module
from core.msgs.numpy_compat import np
from core.registry import get, list_plugins, register
from core.stream import In, Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Output type — framework-level detection result
# ---------------------------------------------------------------------------

class DetectionResult:
    """Detector output: list of 2D detections + metadata.

    Serializable, transport-friendly. Decoupled from detector internals.
    """
    __slots__ = ("detections", "detector_name", "inference_ms", "timestamp")

    def __init__(self, detections: list, timestamp: float = 0.0,
                 detector_name: str = "", inference_ms: float = 0.0):
        self.detections = detections
        self.timestamp = timestamp or time.time()
        self.detector_name = detector_name
        self.inference_ms = inference_ms

    def __repr__(self):
        return (f"DetectionResult(n={len(self.detections)}, "
                f"det={self.detector_name}, {self.inference_ms:.1f}ms)")


def _module_attr(module: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        if hasattr(module, name):
            return getattr(module, name)
    return default


class _YOLOEDetectorProvider:
    label = "YOLOEDetector"

    @staticmethod
    def create(module):
        from .yoloe_detector import YOLOEDetector

        return YOLOEDetector(
            model_size=_module_attr(module, "_model_size", "_detector_model_size", default="l"),
            confidence=_module_attr(module, "_confidence", "_confidence_threshold", default=0.3),
            iou_threshold=_module_attr(
                module, "_iou_threshold", "_detector_iou_threshold", default=0.45
            ),
            device=_module_attr(module, "_device", "_detector_device", default=""),
            max_detections=_module_attr(
                module, "_max_detections", "_detector_max_detections", default=64
            ),
        )


class _YOLOWorldDetectorProvider:
    label = "YOLOWorldDetector"

    @staticmethod
    def create(module):
        from .yolo_world_detector import YOLOWorldDetector

        return YOLOWorldDetector(
            model_size=_module_attr(module, "_model_size", "_detector_model_size", default="l"),
            confidence=_module_attr(module, "_confidence", "_confidence_threshold", default=0.3),
            iou_threshold=_module_attr(
                module, "_iou_threshold", "_detector_iou_threshold", default=0.45
            ),
            device=_module_attr(module, "_device", "_detector_device", default=""),
        )


class _BPUDetectorProvider:
    label = "BPUDetector"

    @staticmethod
    def create(module):
        from .bpu_detector import BPUDetector

        return BPUDetector(
            model_path=_module_attr(module, "_model_path", "_detector_model_path", default=""),
            confidence=_module_attr(module, "_confidence", "_confidence_threshold", default=0.3),
            iou_threshold=_module_attr(
                module, "_iou_threshold", "_detector_iou_threshold", default=0.45
            ),
            max_detections=_module_attr(
                module, "_max_detections", "_detector_max_detections", default=64
            ),
            min_box_size_px=_module_attr(
                module, "_min_box_size_px", "_detector_min_box_size_px", default=12
            ),
        )


class _GroundingDINODetectorProvider:
    label = "GroundingDINODetector"

    @staticmethod
    def create(module):
        from .grounding_dino_detector import GroundingDINODetector

        kwargs = {
            "box_threshold": _module_attr(
                module, "_confidence", "_confidence_threshold", default=0.35
            ),
        }
        device = _module_attr(module, "_device", "_detector_device", default="")
        if device:
            kwargs["device"] = device
        return GroundingDINODetector(**kwargs)


def _register_builtin_detector_providers() -> None:
    register(
        "detector",
        "yoloe",
        description="YOLO-E open-vocabulary instance detector",
    )(_YOLOEDetectorProvider)
    register(
        "detector",
        "yolo_world",
        description="YOLO-World open-vocabulary detector",
    )(_YOLOWorldDetectorProvider)
    register(
        "detector",
        "bpu",
        description="D-Robotics Nash BPU detector",
    )(_BPUDetectorProvider)
    register(
        "detector",
        "grounding_dino",
        description="GroundingDINO open-vocabulary detector",
    )(_GroundingDINODetectorProvider)


_register_builtin_detector_providers()


# ---------------------------------------------------------------------------
# DetectorModule
# ---------------------------------------------------------------------------

@register("detector", "pluggable", description="Pluggable detector module")
class DetectorModule(Module, layer=3):
    """Pluggable object detector Module.

    In:  image (np.ndarray)  — BGR uint8 HxWx3
    Out: detections (DetectionResult) — list of Detection2D + metadata

    Swap the detector backend via ``detector=`` config parameter:
      "yolo_world", "yoloe", "grounding_dino", "bpu"
    """

    image: In[Any]
    detections: Out[DetectionResult]

    def __init__(
        self,
        detector: str = "yoloe",
        text_prompt: str = "",
        confidence: float = 0.3,
        iou_threshold: float = 0.45,
        max_detections: int = 64,
        min_box_size_px: int = 12,
        model_size: str = "l",
        model_path: str = "",
        device: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._detector_name = detector
        self._text_prompt = text_prompt
        self._confidence = confidence
        self._iou_threshold = iou_threshold
        self._max_detections = max_detections
        self._min_box_size_px = min_box_size_px
        self._model_size = model_size
        self._model_path = model_path
        self._device = device
        self._detector_model_size = model_size
        self._confidence_threshold = confidence
        self._detector_iou_threshold = iou_threshold
        self._detector_max_detections = max_detections
        self._detector_min_box_size_px = min_box_size_px
        self._detector_model_path = model_path
        self._detector_device = device
        self._backend = None
        self._frame_count = 0
        self._total_inference_ms = 0.0

    def setup(self) -> None:
        """Load the selected detector backend."""
        self._backend = self._create_backend()
        try:
            self._backend.load_model()
            logger.info("DetectorModule: loaded '%s' backend", self._detector_name)
        except Exception:
            logger.exception("DetectorModule: failed to load '%s'", self._detector_name)
            self._backend = None
        self.image.subscribe(self._on_image)

    def _create_backend(self):
        """Factory: instantiate the selected detector backend."""
        _register_builtin_detector_providers()
        name = self._detector_name.lower()
        try:
            provider = get("detector", name)
        except KeyError:
            available = ", ".join(list_plugins("detector")) or "<none>"
            raise ValueError(
                f"Unknown detector backend '{name}'. Available: {available}"
            ) from None
        return provider.create(self)

    def _on_image(self, image: np.ndarray):
        """Run detection on incoming image, publish results."""
        if self._backend is None:
            return

        t0 = time.time()
        try:
            dets = self._backend.detect(image, self._text_prompt)
        except Exception:
            logger.exception("DetectorModule: detection failed")
            return
        inference_ms = (time.time() - t0) * 1000

        self._frame_count += 1
        self._total_inference_ms += inference_ms

        result = DetectionResult(
            detections=dets,
            timestamp=time.time(),
            detector_name=self._detector_name,
            inference_ms=inference_ms,
        )
        self.detections.publish(result)

    def set_prompt(self, text_prompt: str):
        """Update detection classes at runtime."""
        self._text_prompt = text_prompt

    def stop(self):
        """Shutdown the detector backend."""
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
        info["detector"] = {
            "backend": self._detector_name,
            "loaded": self._backend is not None,
            "frames": self._frame_count,
            "avg_ms": round(avg_ms, 1),
        }
        return info
