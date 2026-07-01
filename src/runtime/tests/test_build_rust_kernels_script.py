from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build" / "build_rust_kernels.py"
ROS_BUILD = ROOT / "scripts" / "build" / "build_ros_workspace.sh"


def _load_module():
    spec = importlib.util.spec_from_file_location("build_rust_kernels", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_rust_kernel_build_catalog_includes_migration_kernels() -> None:
    module = _load_module()

    targets = module.target_by_key()

    assert set(targets) >= {
        "path_safety",
        "pose_graph_opt",
        "gpmp_trajectory_optimizer",
        "camera_lidar_optimizer",
    }
    assert targets["gpmp_trajectory_optimizer"].required_binaries == ("gpmp_optimize",)
    assert targets["path_safety"].required_libraries == ("lingtu_path_safety",)
    assert targets["pose_graph_opt"].required_libraries == ("lingtu_pose_graph_opt",)
    assert targets["camera_lidar_optimizer"].required_libraries == (
        "lingtu_camera_lidar_optimizer",
    )
    assert targets["gpmp_trajectory_optimizer"].manifest.is_file()


def test_rust_kernel_build_command_targets_gpmp_release_binary() -> None:
    module = _load_module()
    target = module.target_by_key()["gpmp_trajectory_optimizer"]

    result = module.build_target(target, release=True, dry_run=True)

    assert result["key"] == "gpmp_trajectory_optimizer"
    assert result["command"][:3] == ["cargo", "build", "--manifest-path"]
    assert "--release" in result["command"]
    assert any(path.endswith("gpmp_optimize.exe") or path.endswith("gpmp_optimize") for path in result["required_binaries"])
    assert result["required_library_candidates"]


def test_rust_kernel_build_command_tracks_camera_lidar_library_artifact() -> None:
    module = _load_module()
    target = module.target_by_key()["camera_lidar_optimizer"]

    result = module.build_target(target, release=True, dry_run=True)

    flattened = [path for group in result["required_library_candidates"] for path in group]
    assert result["key"] == "camera_lidar_optimizer"
    assert any("lingtu_camera_lidar_optimizer" in path for path in flattened)


def test_rust_kernel_build_command_tracks_path_safety_library_artifact() -> None:
    module = _load_module()
    target = module.target_by_key()["path_safety"]

    result = module.build_target(target, release=True, dry_run=True)

    flattened = [path for group in result["required_library_candidates"] for path in group]
    assert result["key"] == "path_safety"
    assert any("lingtu_path_safety" in path for path in flattened)


def test_ros_workspace_builds_rust_gpmp_by_default() -> None:
    script = ROS_BUILD.read_text(encoding="utf-8")

    assert 'BUILD_RUST_KERNELS="${LINGTU_BUILD_RUST_KERNELS:-1}"' in script
    assert 'RUST_KERNEL_TARGETS="${LINGTU_RUST_KERNEL_TARGETS:-gpmp_trajectory_optimizer,camera_lidar_optimizer}"' in script
    assert "scripts/build/build_rust_kernels.py" in script
    assert 'if [[ "${BUILD_RUST_KERNELS}" != "1" ]]' in script
