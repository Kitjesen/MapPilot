"""Contract tests for MujocoDriverModule port shape, lifecycle, and spec compliance.

Verifies MujocoDriverModule satisfies the MotionDriver contract, keeps legacy
camera/lidar ports available for compatibility, and has idempotent lifecycle
methods. MuJoCo itself is never started; only the Module's port declarations and
contract compliance are checked.
"""

from __future__ import annotations

import sys

from runtime.msgs.geometry import Twist
from runtime.msgs.map import MapCloudFrame, MapObservationFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, PointCloud2
from runtime.registry import get

# Ensure MujocoDriverModule is imported so @register fires.
try:
    __import__("drivers.sim.mujoco.driver")
except ImportError:
    pass


def _get_mujoco():
    """Return the MujocoDriverModule class, skipping if not registered."""
    try:
        return get("driver", "sim_mujoco")
    except KeyError:
        import pytest

        pytest.skip("MujocoDriverModule not registered")


class TestMujocoDriverContract:
    """Contract verification for MujocoDriverModule."""

    def test_instantiation(self):
        """Create MujocoDriverModule with default params."""
        cls = _get_mujoco()
        mod = cls(world="building_scene")
        assert mod._world_name == "building_scene"
        assert not mod._running

    def test_instantiation_with_custom_params(self):
        """Custom constructor parameters must be reflected in module state."""
        cls = _get_mujoco()
        mod = cls(
            world="factory_scene",
            render=False,
            enable_camera=True,
            sim_rate=100.0,
            drive_mode="kinematic",
        )
        assert mod._world_name == "factory_scene"
        assert mod._enable_camera is True
        assert mod._sim_rate == 100.0
        assert mod._drive_mode == "kinematic"

    def test_legacy_camera_and_lidar_publish_defaults_stay_disabled(self):
        """Constructor defaults keep legacy camera/LiDAR publication silent."""
        cls = _get_mujoco()
        mod = cls()
        assert mod._publish_camera is False
        assert mod._publish_lidar is False

    def test_driver_camera_and_lidar_can_be_enabled_for_in_process_profiles(self):
        """In-process profiles can explicitly publish camera and LiDAR."""
        cls = _get_mujoco()
        mod = cls(enable_camera=True, publish_camera=True, publish_lidar=True)
        assert mod._publish_camera is True
        assert mod._publish_lidar is True

    def test_required_input_ports(self):
        """Must declare cmd_vel: In[Twist] and stop_signal: In[int]."""
        cls = _get_mujoco()
        mod = cls()
        assert "cmd_vel" in mod._ports_in
        assert mod._ports_in["cmd_vel"].msg_type is Twist
        assert "stop_signal" in mod._ports_in
        assert mod._ports_in["stop_signal"].msg_type is int

    def test_required_output_ports(self):
        """Must declare odometry: Out[Odometry] and robot_state: Out[dict]."""
        cls = _get_mujoco()
        mod = cls()
        assert "odometry" in mod._ports_out
        assert mod._ports_out["odometry"].msg_type is Odometry
        assert "robot_state" in mod._ports_out
        assert mod._ports_out["robot_state"].msg_type is dict
        assert "imu" not in mod._ports_out

    def test_legacy_camera_ports_remain_for_compatibility(self):
        """Legacy ports remain declared while canonical camera owns new streams."""
        cls = _get_mujoco()
        mod = cls()
        assert "camera_image" in mod._ports_out
        assert mod._ports_out["camera_image"].msg_type is Image
        assert "depth_image" in mod._ports_out
        assert mod._ports_out["depth_image"].msg_type is Image
        assert "camera_info" in mod._ports_out
        assert mod._ports_out["camera_info"].msg_type is CameraIntrinsics

    def test_pointcloud_ports_use_typed_map_frame(self):
        """Raw LiDAR stays PointCloud2 while map output carries map semantics."""
        cls = _get_mujoco()
        mod = cls()
        assert "map_cloud" not in mod._ports_out
        assert mod._ports_out["map_observation"].msg_type is MapObservationFrame
        assert mod._ports_out["map_cloud_frame"].msg_type is MapCloudFrame
        assert "lidar_cloud" in mod._ports_out
        assert mod._ports_out["lidar_cloud"].msg_type is PointCloud2

    def test_lifecycle_graceful_without_mujoco(self):
        """setup/start/stop must not crash when MuJoCo is unavailable."""
        cls = _get_mujoco()
        mod = cls()
        # setup may print a warning but must not raise
        mod.setup()
        mod.start()
        # Without MuJoCo, engine stays None; start publishes alive=False
        assert mod._running
        mod.stop()
        assert not mod._running
        # Idempotent
        mod.stop()
        assert not mod._running

    def test_setup_does_not_duplicate_repo_root_in_sys_path(self, monkeypatch):
        """setup() must not insert duplicate import roots when already configured."""
        import drivers.sim.mujoco.driver as module

        cls = _get_mujoco()
        repo_root = str(module._SIM_ROOT.parent)
        path_without_repo_root = [path for path in sys.path if path != repo_root]
        monkeypatch.setattr(sys, "path", [repo_root, *path_without_repo_root])

        mod = cls()
        mod.setup()

        assert sys.path.count(repo_root) == 1

    def test_driver_contract_issues_empty(self):
        """driver_contract_issues() must return an empty list."""
        from tests.drivers.driver_contract import driver_contract_issues

        cls = _get_mujoco()
        issues = driver_contract_issues(cls)
        assert issues == [], f"MujocoDriverModule violates MotionDriver: {issues}"

    def test_is_motion_driver(self):
        """is_motion_driver() must return True."""
        from tests.drivers.driver_contract import is_motion_driver

        cls = _get_mujoco()
        assert is_motion_driver(cls) is True

    def test_is_camera_source_for_legacy_compatibility(self):
        """Legacy contract helper still sees camera ports until aliases are removed."""
        from tests.drivers.driver_contract import is_camera_source

        cls = _get_mujoco()
        assert is_camera_source(cls) is True

    def test_is_pointcloud_source(self):
        """The typed map-frame output satisfies the point-cloud source contract."""
        from tests.drivers.driver_contract import is_pointcloud_source

        cls = _get_mujoco()
        assert is_pointcloud_source(cls) is True

    def test_capabilities_report(self):
        """driver_capabilities() remains compatibility-shaped until old aliases go away."""
        from tests.drivers.driver_contract import driver_capabilities

        cls = _get_mujoco()
        report = driver_capabilities(cls)
        assert report["motion_driver"] is True
        assert report["issues"] == []
        assert report["camera_source"] is True
        assert report["pointcloud_source"] is True
