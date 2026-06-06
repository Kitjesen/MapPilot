"""Contract tests for MujocoDriverModule — port shape, lifecycle, spec compliance.

Verifies MujocoDriverModule satisfies the MotionDriver contract, exposes
camera+pointcloud source ports (full sensor driver tier), and has idempotent
lifecycle methods.  MuJoCo itself is never started — only the Module's port
declarations and contract compliance are checked.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from core.msgs.geometry import Twist
from core.msgs.nav import Odometry
from core.msgs.sensor import CameraIntrinsics, Image, PointCloud2
from core.registry import get

# Ensure MujocoDriverModule is imported so @register fires.
try:
    __import__("drivers.sim.mujoco_driver_module")
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

    def test_camera_source_ports(self):
        """Full sensor driver must expose camera_image, depth_image, camera_info."""
        cls = _get_mujoco()
        mod = cls()
        assert "camera_image" in mod._ports_out
        assert mod._ports_out["camera_image"].msg_type is Image
        assert "depth_image" in mod._ports_out
        assert mod._ports_out["depth_image"].msg_type is Image
        assert "camera_info" in mod._ports_out
        assert mod._ports_out["camera_info"].msg_type is CameraIntrinsics

    def test_pointcloud_source_ports(self):
        """Full sensor driver must expose map_cloud."""
        cls = _get_mujoco()
        mod = cls()
        assert "map_cloud" in mod._ports_out
        assert mod._ports_out["map_cloud"].msg_type is PointCloud2
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

    def test_setup_soft_fails_when_numpy_runtime_probe_fails(self, monkeypatch):
        """setup() must not import the MuJoCo engine when NumPy is unsafe."""
        import drivers.sim.mujoco_driver_module as module

        cls = _get_mujoco()
        monkeypatch.setattr(module, "_NUMPY_RUNTIME_AVAILABLE", False)

        mod = cls()
        mod.setup()

        assert mod._engine is None

    def test_setup_does_not_duplicate_repo_root_in_sys_path(self, monkeypatch):
        """setup() must not insert duplicate import roots when already configured."""
        import drivers.sim.mujoco_driver_module as module

        cls = _get_mujoco()
        repo_root = str(module._SIM_ROOT.parent)
        path_without_repo_root = [path for path in sys.path if path != repo_root]
        monkeypatch.setattr(sys, "path", [repo_root, *path_without_repo_root])

        mod = cls()
        mod.setup()

        assert sys.path.count(repo_root) == 1

    def test_import_does_not_load_numpy(self):
        """Control-plane import must stay available without numerical runtime."""
        src_root = Path(__file__).resolve().parents[2]
        env = dict(os.environ)
        env["PYTHONPATH"] = (
            str(src_root)
            if not env.get("PYTHONPATH")
            else f"{src_root}{os.pathsep}{env['PYTHONPATH']}"
        )
        code = (
            "import sys; "
            "import drivers.sim.mujoco_driver_module; "
            "raise SystemExit(1 if 'numpy' in sys.modules else 0)"
        )

        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=str(src_root.parent),
            env=env,
            text=True,
            capture_output=True,
            timeout=10,
        )

        assert result.returncode == 0, result.stderr or result.stdout

    def test_driver_contract_issues_empty(self):
        """driver_contract_issues() must return an empty list."""
        from drivers.spec import driver_contract_issues

        cls = _get_mujoco()
        issues = driver_contract_issues(cls)
        assert issues == [], f"MujocoDriverModule violates MotionDriver: {issues}"

    def test_is_motion_driver(self):
        """is_motion_driver() must return True."""
        from drivers.spec import is_motion_driver

        cls = _get_mujoco()
        assert is_motion_driver(cls) is True

    def test_is_camera_source(self):
        """is_camera_source() must return True for full sim driver."""
        from drivers.spec import is_camera_source

        cls = _get_mujoco()
        assert is_camera_source(cls) is True

    def test_is_pointcloud_source(self):
        """is_pointcloud_source() must return True for full sim driver."""
        from drivers.spec import is_pointcloud_source

        cls = _get_mujoco()
        assert is_pointcloud_source(cls) is True

    def test_capabilities_report(self):
        """driver_capabilities() must report correct full-sensor tier."""
        from drivers.spec import driver_capabilities

        cls = _get_mujoco()
        report = driver_capabilities(cls)
        assert report["motion_driver"] is True
        assert report["issues"] == []
        assert report["camera_source"] is True
        assert report["pointcloud_source"] is True
