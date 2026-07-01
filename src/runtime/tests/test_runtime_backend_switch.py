from __future__ import annotations

import pytest

from runtime.module import Module
from runtime.registry import register
from perception.perception_module import PerceptionModule
from decision.modules.llm_module import LLMModule


class _RuntimeSwitchDetector:
    def load_model(self) -> None:
        pass

    def detect(self, _image, _prompt):
        return []


class _RuntimeSwitchEncoder:
    def load_model(self) -> None:
        pass

    def encode_text(self, _text):
        return []


_PROBE_EVENTS: list[tuple[str, bool]] = []
_PROBE_CREATED: dict[str, list["_RuntimeSwitchProbeBackend"]] = {
    "detector": [],
    "encoder": [],
}
_PROBE_FAIL_LOAD = {"detector": False, "encoder": False}


class _TrackingLock:
    def __init__(self) -> None:
        self.depth = 0

    @property
    def held(self) -> bool:
        return self.depth > 0

    def __enter__(self):
        self.depth += 1
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.depth -= 1


class _DisposableBackend:
    def __init__(self, name: str, lock: _TrackingLock) -> None:
        self.name = name
        self.lock = lock
        self.dispose_calls: list[str] = []

    def shutdown(self) -> None:
        _PROBE_EVENTS.append((f"{self.name}.shutdown", self.lock.held))
        self.dispose_calls.append("shutdown")


class _RuntimeSwitchProbeBackend:
    def __init__(self, kind: str, lock: _TrackingLock) -> None:
        self.kind = kind
        self.lock = lock
        self.fail_load = _PROBE_FAIL_LOAD[kind]
        self.dispose_calls: list[str] = []

    def load_model(self) -> None:
        _PROBE_EVENTS.append((f"{self.kind}.load_model", self.lock.held))
        if self.fail_load:
            raise RuntimeError(f"{self.kind} load failed")

    def shutdown(self) -> None:
        _PROBE_EVENTS.append((f"{self.kind}.shutdown", self.lock.held))
        self.dispose_calls.append("shutdown")

    def detect(self, _image, _prompt):
        return []

    def encode_text(self, _text):
        return []


def _reset_probe_state() -> None:
    _PROBE_EVENTS.clear()
    for created in _PROBE_CREATED.values():
        created.clear()
    for kind in _PROBE_FAIL_LOAD:
        _PROBE_FAIL_LOAD[kind] = False


@register("detector", "runtime_switch_test_detector")
class _RuntimeSwitchDetectorProvider:
    @staticmethod
    def create(_module):
        return _RuntimeSwitchDetector()


@register("encoder", "runtime_switch_test_encoder")
class _RuntimeSwitchEncoderProvider:
    @staticmethod
    def create(_module):
        return _RuntimeSwitchEncoder()


@register("detector", "runtime_switch_lock_detector")
class _RuntimeSwitchLockDetectorProvider:
    @staticmethod
    def create(module):
        _PROBE_EVENTS.append(("detector.create", module._backend_lock.held))
        backend = _RuntimeSwitchProbeBackend("detector", module._backend_lock)
        _PROBE_CREATED["detector"].append(backend)
        return backend


@register("encoder", "runtime_switch_lock_encoder")
class _RuntimeSwitchLockEncoderProvider:
    @staticmethod
    def create(module):
        _PROBE_EVENTS.append(("encoder.create", module._backend_lock.held))
        backend = _RuntimeSwitchProbeBackend("encoder", module._backend_lock)
        _PROBE_CREATED["encoder"].append(backend)
        return backend


def test_motion_backend_reconfigure_is_unsupported_by_default():
    mod = Module()

    result = mod.reconfigure_backend("local_planner", "nav_kernel")

    assert result == {
        "ok": False,
        "category": "local_planner",
        "requested_backend": "nav_kernel",
        "reason": "backend_reconfigure_unsupported",
    }


@pytest.mark.parametrize(
    ("module_path", "class_name", "category", "backend"),
    [
        ("nav.mission.navigation", "nav.mission", "planner", "astar"),
        ("nav.local.local_planner", "nav.local_planner", "local_planner", "cmu_py"),
        ("nav.local.path_follower", "nav.path_follower", "path_follower", "pid"),
        ("nav.local.terrain", "nav.terrain", "terrain", "simple"),
        ("localization.bridge", "SlamBridgeModule", "slam", "bridge"),
    ],
)
def test_motion_modules_reconfigure_backend_fails_closed(module_path, class_name, category, backend):
    module = __import__(module_path, fromlist=[class_name])
    cls = getattr(module, class_name)
    mod = cls()

    result = mod.reconfigure_backend(category, backend)

    assert result == {
        "ok": False,
        "category": category,
        "requested_backend": backend,
        "reason": "backend_reconfigure_unsupported",
    }


def test_perception_reconfigure_detector_rejects_unknown_backend():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")

    result = mod.reconfigure_backend("detector", "missing_backend")

    assert result["ok"] is False
    assert result["category"] == "detector"
    assert result["requested_backend"] == "missing_backend"
    assert result["reason"] == "unknown_backend"


def test_perception_reconfigure_detector_updates_health_status():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    mod._detector_type = "sim_scene"
    mod._detector = object()

    result = mod.reconfigure_backend("detector", "runtime_switch_test_detector")

    assert result["ok"] is True
    assert result["previous_backend"] == "sim_scene"
    assert result["backend"] == "runtime_switch_test_detector"
    health = mod.health()
    assert health["detector"]["configured_backend"] == "runtime_switch_test_detector"
    assert health["detector"]["backend"] == "runtime_switch_test_detector"
    assert health["detector"]["degraded"] is False


def test_perception_reconfigure_detector_preloads_outside_backend_lock_and_disposes_old():
    _reset_probe_state()
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    lock = _TrackingLock()
    old_detector = _DisposableBackend("old_detector", lock)
    mod._backend_lock = lock
    mod._detector_type = "sim_scene"
    mod._detector = old_detector
    mod._detector_tracker = object()
    mod._sim_scene_observer = object()

    result = mod.reconfigure_backend("detector", "runtime_switch_lock_detector")

    assert result["ok"] is True
    assert mod._detector is _PROBE_CREATED["detector"][0]
    assert old_detector.dispose_calls == ["shutdown"]
    assert ("detector.create", False) in _PROBE_EVENTS
    assert ("detector.load_model", False) in _PROBE_EVENTS
    assert ("old_detector.shutdown", False) in _PROBE_EVENTS
    assert ("detector.create", True) not in _PROBE_EVENTS
    assert ("detector.load_model", True) not in _PROBE_EVENTS


def test_perception_reconfigure_detector_disposes_failed_candidate_and_keeps_active_backend():
    _reset_probe_state()
    _PROBE_FAIL_LOAD["detector"] = True
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    lock = _TrackingLock()
    active_detector = _DisposableBackend("active_detector", lock)
    active_observer = object()
    mod._backend_lock = lock
    mod._detector_type = "sim_scene"
    mod._detector = active_detector
    mod._detector_tracker = object()
    mod._sim_scene_observer = active_observer

    result = mod.reconfigure_backend("detector", "runtime_switch_lock_detector")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_failed"
    assert mod._detector_type == "sim_scene"
    assert mod._detector is active_detector
    assert mod._sim_scene_observer is active_observer
    assert active_detector.dispose_calls == []
    assert _PROBE_CREATED["detector"][0].dispose_calls == ["shutdown"]
    assert ("detector.shutdown", False) in _PROBE_EVENTS


def test_perception_reconfigure_encoder_updates_health_status():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    mod._encoder_type = "clip"
    mod._clip_encoder = object()

    result = mod.reconfigure_backend("encoder", "runtime_switch_test_encoder")

    assert result["ok"] is True
    assert result["previous_backend"] == "clip"
    assert result["backend"] == "runtime_switch_test_encoder"
    health = mod.health()
    assert health["encoder"]["configured_backend"] == "runtime_switch_test_encoder"
    assert health["encoder"]["backend"] == "runtime_switch_test_encoder"
    assert health["encoder"]["degraded"] is False


def test_perception_reconfigure_encoder_preloads_outside_backend_lock_and_disposes_old():
    _reset_probe_state()
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    lock = _TrackingLock()
    old_encoder = _DisposableBackend("old_encoder", lock)
    mod._backend_lock = lock
    mod._encoder_type = "clip"
    mod._clip_encoder = old_encoder

    result = mod.reconfigure_backend("encoder", "runtime_switch_lock_encoder")

    assert result["ok"] is True
    assert mod._clip_encoder is _PROBE_CREATED["encoder"][0]
    assert old_encoder.dispose_calls == ["shutdown"]
    assert ("encoder.create", False) in _PROBE_EVENTS
    assert ("encoder.load_model", False) in _PROBE_EVENTS
    assert ("old_encoder.shutdown", False) in _PROBE_EVENTS
    assert ("encoder.create", True) not in _PROBE_EVENTS
    assert ("encoder.load_model", True) not in _PROBE_EVENTS


def test_perception_reconfigure_encoder_disposes_failed_candidate_and_keeps_active_backend():
    _reset_probe_state()
    _PROBE_FAIL_LOAD["encoder"] = True
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    lock = _TrackingLock()
    active_encoder = _DisposableBackend("active_encoder", lock)
    mod._backend_lock = lock
    mod._encoder_type = "clip"
    mod._clip_encoder = active_encoder

    result = mod.reconfigure_backend("encoder", "runtime_switch_lock_encoder")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_failed"
    assert mod._encoder_type == "clip"
    assert mod._clip_encoder is active_encoder
    assert active_encoder.dispose_calls == []
    assert _PROBE_CREATED["encoder"][0].dispose_calls == ["shutdown"]
    assert ("encoder.shutdown", False) in _PROBE_EVENTS


def test_llm_reconfigure_backend_rejects_unknown_backend():
    mod = LLMModule(backend="mock")

    result = mod.reconfigure_backend("llm", "missing_backend")

    assert result["ok"] is False
    assert result["category"] == "llm"
    assert result["requested_backend"] == "missing_backend"
    assert result["reason"] == "unknown_backend"


def test_llm_reconfigure_backend_rejects_in_flight_request():
    mod = LLMModule(backend="mock")
    mod._in_flight = 1

    result = mod.reconfigure_backend("llm", "mock")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_in_flight"


def test_llm_reconfigure_backend_updates_health_status():
    mod = LLMModule(backend="mock")

    result = mod.reconfigure_backend("llm", "mock")

    assert result["ok"] is True
    assert result["previous_backend"] == "mock"
    assert result["backend"] == "mock"
    health = mod.health()
    assert health["llm"]["configured_backend"] == "mock"
    assert health["llm"]["backend"] == "mock"
    assert health["llm"]["degraded"] is False
