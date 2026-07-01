"""Contract tests for ThunderDriver port shape, lifecycle, and spec compliance.

Verifies ThunderDriver satisfies the MotionDriver contract from
``tests.drivers.driver_contract``, exposes the correct In/Out port types, and has
idempotent lifecycle methods.
"""

from __future__ import annotations

import pytest

from runtime.msgs.geometry import Twist
from runtime.msgs.nav import Odometry
from runtime.registry import get

# Ensure the ThunderDriver module is imported so @register fires.
try:
    __import__("drivers.real.thunder.han_dog_module")
except ImportError:
    pass


def _get_thunder():
    """Return the ThunderDriver class, skipping if not registered (aarch64 only)."""
    try:
        return get("driver", "thunder")
    except KeyError:
        pytest.skip("ThunderDriver not registered; only available on aarch64")


class TestThunderDriverContract:
    """Contract verification for ThunderDriver."""

    def test_instantiation(self):
        """Create ThunderDriver with default params."""
        cls = _get_thunder()
        mod = cls(dog_host="127.0.0.1", dog_port=13145)
        assert mod._dog_host == "127.0.0.1"
        assert mod._dog_port == 13145
        assert not mod._shutdown

    def test_instantiation_with_custom_config(self):
        """Custom constructor parameters must be reflected in module state."""
        cls = _get_thunder()
        mod = cls(
            dog_host="192.168.1.100",
            dog_port=9999,
            max_linear_speed=0.5,
            max_angular_speed=0.3,
            control_rate=100.0,
            auto_enable=True,
        )
        assert mod._dog_host == "192.168.1.100"
        assert mod._dog_port == 9999
        assert mod._max_linear == 0.5
        assert mod._max_angular == 0.3
        assert mod._control_rate == 100.0
        assert mod._auto_enable is True

    def test_required_input_ports(self):
        """Must declare cmd_vel: In[Twist] and stop_signal: In[int]."""
        cls = _get_thunder()
        mod = cls()
        assert "cmd_vel" in mod._ports_in
        assert mod._ports_in["cmd_vel"].msg_type is Twist
        assert "stop_signal" in mod._ports_in
        assert mod._ports_in["stop_signal"].msg_type is int

    def test_required_output_ports(self):
        """Must declare odometry: Out[Odometry] and robot_state: Out[dict]."""
        cls = _get_thunder()
        mod = cls()
        assert "odometry" in mod._ports_out
        assert mod._ports_out["odometry"].msg_type is Odometry
        assert "robot_state" in mod._ports_out
        assert mod._ports_out["robot_state"].msg_type is dict

    def test_extra_ports(self):
        """ThunderDriver may expose slam_odom In and alive Out beyond spec."""
        cls = _get_thunder()
        mod = cls()
        assert "slam_odom" in mod._ports_in
        assert mod._ports_in["slam_odom"].msg_type is Odometry
        assert "alive" in mod._ports_out
        assert mod._ports_out["alive"].msg_type is bool

    def test_lifecycle_setup(self):
        """setup() registers subscribers without raising."""
        cls = _get_thunder()
        mod = cls()
        mod.setup()
        # No exception means subscribers registered successfully

    def test_lifecycle_start_stop_idempotent(self):
        """start()/stop() transitions without error and stop is idempotent."""
        cls = _get_thunder()
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

        cls = _get_thunder()
        issues = driver_contract_issues(cls)
        assert issues == [], f"ThunderDriver violates MotionDriver: {issues}"

    def test_is_motion_driver(self):
        """is_motion_driver() must return True."""
        from tests.drivers.driver_contract import is_motion_driver

        cls = _get_thunder()
        assert is_motion_driver(cls) is True

    def test_capabilities_report(self):
        """driver_capabilities() must report correct contract tier."""
        from tests.drivers.driver_contract import driver_capabilities

        cls = _get_thunder()
        report = driver_capabilities(cls)
        assert report["motion_driver"] is True
        assert report["issues"] == []
        # Minimal driver: no camera and no pointcloud source
        assert report["camera_source"] is False
        assert report["pointcloud_source"] is False
