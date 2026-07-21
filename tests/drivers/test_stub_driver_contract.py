"""Contract tests for StubDogModule port shape, lifecycle, and spec compliance.

Verifies StubDogModule satisfies the MotionDriver contract from
``tests.drivers.driver_contract``, exposes the correct In/Out port types, and has
idempotent lifecycle methods.  StubDogModule is the canonical stub
driver registered as (``driver``, ``stub``).
"""

from __future__ import annotations

import pytest

from runtime.msgs.geometry import Twist
from runtime.msgs.nav import Odometry
from runtime.registry import get

# Ensure StubDogModule is imported so @register fires.
try:
    __import__("drivers.sim.stub")
except ImportError:
    pass


def _get_stub():
    """Return the StubDogModule class, skipping if not registered."""
    try:
        return get("driver", "stub")
    except KeyError:
        pytest.skip("StubDogModule not registered")


class TestStubDriverContract:
    """Contract verification for StubDogModule."""

    def test_instantiation(self):
        """Create StubDogModule with default params."""
        cls = _get_stub()
        mod = cls()
        assert mod._pos_x == 0.0
        assert mod._pos_y == 0.0
        assert mod._yaw == 0.0

    def test_instantiation_with_initial_pose(self):
        """Custom initial pose must be reflected in module state."""
        cls = _get_stub()
        mod = cls(initial_x=1.0, initial_y=2.0, initial_yaw=0.5)
        assert mod._pos_x == 1.0
        assert mod._pos_y == 2.0
        assert mod._yaw == 0.5

    def test_required_input_ports(self):
        """Must declare cmd_vel: In[Twist] and stop_signal: In[int]."""
        cls = _get_stub()
        mod = cls()
        assert "cmd_vel" in mod._ports_in
        assert mod._ports_in["cmd_vel"].msg_type is Twist
        assert "stop_signal" in mod._ports_in
        assert mod._ports_in["stop_signal"].msg_type is int

    def test_required_output_ports(self):
        """Must declare odometry: Out[Odometry] and robot_state: Out[dict]."""
        cls = _get_stub()
        mod = cls()
        assert "odometry" in mod._ports_out
        assert mod._ports_out["odometry"].msg_type is Odometry
        assert "robot_state" in mod._ports_out
        assert mod._ports_out["robot_state"].msg_type is dict

    def test_lifecycle_setup(self):
        """setup() registers subscribers without raising."""
        cls = _get_stub()
        mod = cls()
        mod.setup()

    def test_lifecycle_start_stop_idempotent(self):
        """start()/stop() transitions without error and stop is idempotent."""
        cls = _get_stub()
        mod = cls()
        mod.setup()
        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running
        # Second stop is idempotent
        mod.stop()
        assert not mod._running

    def test_driver_contract_issues_empty(self):
        """driver_contract_issues() must return an empty list."""
        from tests.drivers.driver_contract import driver_contract_issues

        cls = _get_stub()
        issues = driver_contract_issues(cls)
        assert issues == [], f"StubDogModule violates MotionDriver: {issues}"

    def test_is_motion_driver(self):
        """is_motion_driver() must return True."""
        from tests.drivers.driver_contract import is_motion_driver

        cls = _get_stub()
        assert is_motion_driver(cls) is True

    def test_capabilities_report(self):
        """driver_capabilities() must report correct contract tier."""
        from tests.drivers.driver_contract import driver_capabilities

        cls = _get_stub()
        report = driver_capabilities(cls)
        assert report["motion_driver"] is True
        assert report["issues"] == []
        # Minimal driver: no camera and no pointcloud source
        assert report["camera_source"] is False
        assert report["pointcloud_source"] is False
