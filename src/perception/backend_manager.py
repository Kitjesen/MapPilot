from __future__ import annotations

import logging
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.registry import get, list_plugins

logger = logging.getLogger(__name__)


class BackendManager:
    """Manages the heavy backend lifecycle behind PerceptionModule.

    All backends are resolved through ``runtime.registry`` providers.  This
    keeps PerceptionModule decoupled from concrete detector/encoder/tracker
    imports and removes the old factory-vs-direct split.
    """

    def __init__(self, module: PerceptionModule) -> None:
        self._module = module

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Create and load all backends, with graceful degradation."""
        module = self._module

        # 3D scene-graph tracker (always "instance" for now)
        module._tracker = self._init_tracker()

        # 2D detector
        module._detector, module._sim_scene_observer = self._init_detector()

        # Optional low-latency 2D tracker for detector outputs (BPU only)
        module._detector_tracker = self._init_detector_tracker()

        # Visual/text encoder
        module._clip_encoder = self._init_clip_encoder()

    def stop(self) -> None:
        """Release GPU and native resources held by backends."""
        module = self._module
        for attr, name in (
            ("_detector_tracker", "Detector tracker"),
            ("_detector", "Detector"),
            ("_clip_encoder", "Encoder"),
            ("_tracker", "Tracker"),
            ("_sim_scene_observer", "SimSceneObserver"),
        ):
            obj = getattr(module, attr, None)
            self._dispose_backend(obj, name)
            setattr(module, attr, None)

    # ------------------------------------------------------------------
    # Reconfiguration
    # ------------------------------------------------------------------

    def reconfigure_detector(self, backend: str, **_config: Any) -> dict[str, Any]:
        """Hot-swap the detector backend while preserving the old one on failure."""
        module = self._module
        try:
            get("detector", backend)
        except KeyError:
            return {
                "ok": False,
                "category": "detector",
                "requested_backend": backend,
                "reason": "unknown_backend",
                "available": list_plugins("detector"),
            }

        try:
            detector, observer = self._load_detector(backend)
            if detector is None:
                raise RuntimeError("detector unavailable")
            tracker = self._init_detector_tracker(backend, detector)
        except Exception as exc:
            return {
                "ok": False,
                "category": "detector",
                "requested_backend": backend,
                "reason": "backend_reconfigure_failed",
                "error": str(exc),
            }

        with module._backend_lock:
            previous_backend = module._detector_type
            previous_detector = module._detector
            previous_tracker = module._detector_tracker
            previous_observer = module._sim_scene_observer

            module._detector_type = backend
            module._detector_status = BackendStatus.configured_as(backend)
            module._detector_status.configured = backend
            module._detector_status.use(backend, degraded=False)
            module._detector = detector
            module._detector_tracker = tracker
            module._sim_scene_observer = observer

        disposed: list[Any] = []
        for obj, name in (
            (previous_tracker, "Detector tracker"),
            (previous_detector, "Detector"),
            (previous_observer, "SimSceneObserver"),
        ):
            if obj is None or any(obj is seen for seen in disposed):
                continue
            self._dispose_backend(obj, name)
            disposed.append(obj)

        return {
            "ok": True,
            "category": "detector",
            "previous_backend": previous_backend,
            "backend": backend,
            "degraded": False,
        }

    def reconfigure_encoder(self, backend: str, **_config: Any) -> dict[str, Any]:
        """Hot-swap the encoder backend while preserving the old one on failure."""
        module = self._module
        try:
            get("encoder", backend)
        except KeyError:
            return {
                "ok": False,
                "category": "encoder",
                "requested_backend": backend,
                "reason": "unknown_backend",
                "available": list_plugins("encoder"),
            }

        try:
            encoder = self._load_encoder(backend)
            if encoder is None:
                raise RuntimeError("encoder unavailable")
        except Exception as exc:
            return {
                "ok": False,
                "category": "encoder",
                "requested_backend": backend,
                "reason": "backend_reconfigure_failed",
                "error": str(exc),
            }

        with module._backend_lock:
            previous_backend = module._encoder_type
            previous_encoder = module._clip_encoder
            module._encoder_type = backend
            module._encoder_status = BackendStatus.configured_as(backend)
            module._encoder_status.configured = backend
            module._encoder_status.use(backend, degraded=False)
            module._clip_encoder = encoder

        self._dispose_backend(previous_encoder, "Encoder")
        return {
            "ok": True,
            "category": "encoder",
            "previous_backend": previous_backend,
            "backend": backend,
            "degraded": False,
        }

    # ------------------------------------------------------------------
    # Thin wrappers kept for backward compatibility with existing tests
    # ------------------------------------------------------------------

    def _init_detector(self) -> tuple[Any, Any]:
        """Create and load the configured detector backend."""
        try:
            return self._load_detector(self._module._detector_type)
        except Exception as e:
            logger.warning("Detector %r unavailable: %s", self._module._detector_type, e)
            self._module._detector_status.use("unavailable", reason=str(e))
            return None, None

    def _init_clip_encoder(self) -> Any:
        """Create and load the configured encoder backend."""
        try:
            return self._load_encoder(self._module._encoder_type)
        except Exception as e:
            logger.warning("Encoder %r unavailable: %s", self._module._encoder_type, e)
            self._module._encoder_status.use("unavailable", reason=str(e))
            return None

    def _init_tracker(self) -> Any:
        """Create the 3D instance tracker backend."""
        try:
            provider = get("perception_tracker", "instance")
            view = self._module._CandidateModuleView(self._module)
            tracker = provider.create(view)
            logger.info("InstanceTracker created via registry")
            return tracker
        except ImportError:
            logger.warning("perception.tracking.instance_tracker not available -- scene graph tracking disabled")
        except Exception as e:
            logger.warning("InstanceTracker unavailable: %s", e)
        return None

    def _init_detector_tracker(
        self,
        backend: str | None = None,
        detector: Any | None = None,
    ) -> Any:
        """Create a low-latency 2D tracker for detector outputs when available."""
        module = self._module
        backend = module._detector_type if backend is None else backend
        detector = module._detector if detector is None else detector
        if backend != "bpu" or detector is None:
            return None
        try:
            provider = get("perception_tracker", "bpu")
            view = module._CandidateModuleView(module, detector_type=backend, detector=detector)
            tracker = provider.create(view)
            logger.info("BPUTracker loaded: BPU detector outputs will include track_id")
            return tracker
        except Exception as e:
            logger.warning("BPUTracker unavailable (%s) -- falling back to raw BPU detections", e)
            return None

    # ------------------------------------------------------------------
    # Low-level backend loading
    # ------------------------------------------------------------------

    def _load_detector(self, backend: str) -> tuple[Any, Any]:
        """Load a detector backend from the registry."""
        module = self._module
        provider = get("detector", backend)
        view = module._CandidateModuleView(module, detector_type=backend)
        det = None
        try:
            det = provider.create(view)
            load_model = getattr(det, "load_model", None)
            if callable(load_model):
                load_model()
            logger.info("%s loaded", getattr(provider, "label", backend))
            module._detector_status.use(backend, degraded=False)
            return det, view._sim_scene_observer
        except Exception:
            self._dispose_backend(det, "Detector candidate")
            raise

    def _load_encoder(self, backend: str) -> Any:
        """Load an encoder backend from the registry."""
        module = self._module
        provider = get("encoder", backend)
        view = module._CandidateModuleView(module, encoder_type=backend)
        enc = None
        try:
            enc = provider.create(view)
            load_model = getattr(enc, "load_model", None)
            if callable(load_model):
                load_model()
            logger.info("%s loaded", getattr(provider, "label", backend))
            module._encoder_status.use(backend, degraded=False)
            return enc
        except Exception:
            self._dispose_backend(enc, "Encoder candidate")
            raise

    @staticmethod
    def _dispose_backend(obj: Any | None, name: str) -> None:
        """Call the first available cleanup method on a backend object."""
        if obj is None:
            return
        for method in ("shutdown", "close", "reset"):
            fn = getattr(obj, method, None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:
                    logger.warning("%s %s() error: %s", name, method, e)
                break
