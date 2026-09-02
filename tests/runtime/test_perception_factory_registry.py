from __future__ import annotations

import pytest

from runtime.registry import register, restore, snapshot
from perception.api.exceptions import ConfigurationError
from perception.api.factory import PerceptionFactory


def test_perception_factory_available_lists_come_from_registry():
    assert "yolo_world" in PerceptionFactory.get_available_detectors()
    assert "clip" in PerceptionFactory.get_available_encoders()
    assert "instance" in PerceptionFactory.get_available_trackers()


def test_perception_module_backend_status_uses_configured_names_before_setup():
    from perception.perception_module import PerceptionModule

    module = PerceptionModule(detector_type="bpu", encoder_type="mobileclip")
    health = module.health()

    assert health["detector"]["configured_backend"] == "bpu"
    assert health["encoder"]["configured_backend"] == "mobileclip"


def test_perception_factory_uses_registered_detector_provider():
    saved = snapshot()
    created = object()

    try:
        @register("detector_factory", "fake")
        class FakeDetectorProvider:
            @staticmethod
            def create(_config):
                return created

        assert PerceptionFactory.create_detector("fake") is created
    finally:
        restore(saved)


def test_perception_factory_unknown_names_fail_with_available_plugins():
    with pytest.raises(ConfigurationError, match="Unknown detector type: bogus"):
        PerceptionFactory.create_detector("bogus")
    with pytest.raises(ConfigurationError, match="Supported types: .*yolo_world"):
        PerceptionFactory.create_detector("bogus")
