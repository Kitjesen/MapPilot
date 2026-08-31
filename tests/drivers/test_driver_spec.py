"""Driver contract tests for the sim, real, and stub driver port shape.

Verifies every registered driver backend satisfies the MotionDriver contract
(tests/drivers/driver_contract.py), and that sim drivers expose the full
sensor-source capabilities while minimal drivers (stub/thunder) expose motion
only.
"""

from __future__ import annotations

import importlib
from pathlib import Path

import pytest

from runtime.registry import get
from tests.drivers.driver_contract import (
    driver_capabilities,
    driver_contract_issues,
    is_camera_source,
    is_motion_driver,
    is_pointcloud_source,
)

pytestmark = [pytest.mark.sim]

_DRIVER_MODULES = (
    "lingtu.assembly.stub",
    "drivers.sim.mujoco.driver",
    "drivers.sim.endpoint",
)

_MOTION_DRIVER_BACKENDS = ("stub", "sim_mujoco", "sim_endpoint")


def test_python_thunder_motion_driver_is_physically_retired() -> None:
    source = Path("src/drivers/real/thunder/han_dog_module.py")
    package = importlib.import_module("drivers.real.thunder")

    assert not source.exists()
    assert not hasattr(package, "ThunderDriver")


def _ensure_drivers_registered() -> None:
    for mod in _DRIVER_MODULES:
        try:
            __import__(mod)
        except ImportError:
            # Optional native/grpc deps may be missing on dev machines.
            pass


def _get_driver(name: str):
    _ensure_drivers_registered()
    try:
        return get("driver", name)
    except KeyError:
        pytest.skip(f"driver '{name}' not registered in this environment")


def test_profile_motion_drivers_satisfy_motion_contract():
    _ensure_drivers_registered()
    for name in _MOTION_DRIVER_BACKENDS:
        cls = _get_driver(name)
        issues = driver_contract_issues(cls)
        assert issues == [], f"driver '{name}' violates MotionDriver: {issues}"


@pytest.mark.sim
@pytest.mark.parametrize("name", ["sim_mujoco", "sim_endpoint"])
def test_sim_drivers_expose_sensors(name):
    cls = _get_driver(name)
    assert is_motion_driver(cls)
    assert is_camera_source(cls), f"{name} should expose camera source ports"
    assert is_pointcloud_source(cls), f"{name} should expose map_cloud_frame"


@pytest.mark.parametrize("name", ["stub"])
def test_minimal_drivers_expose_motion_only(name):
    cls = _get_driver(name)
    assert is_motion_driver(cls)
    # Minimal drivers delegate sensors to camera-bridge / lidar modules.
    assert not is_pointcloud_source(cls), (
        f"{name} unexpectedly bundles map_cloud_frame; update spec tiers if intentional"
    )


def test_real_lidar_driver_is_sensor_source_not_motion_driver():
    from drivers.real.lidar import LidarModule

    assert not is_motion_driver(LidarModule)
    assert is_pointcloud_source(LidarModule) is False


def test_driver_capabilities_report_shape():
    cls = _get_driver("stub")
    report = driver_capabilities(cls)
    assert report["motion_driver"] is True
    assert report["issues"] == []
    assert set(report) == {"motion_driver", "issues", "camera_source", "pointcloud_source"}
