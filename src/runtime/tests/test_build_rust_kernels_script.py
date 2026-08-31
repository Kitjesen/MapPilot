from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build" / "build_rust_kernels.py"


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
        "camera_lidar_optimizer",
    }
    assert targets["path_safety"].required_libraries == ("lingtu_path_safety",)
    assert targets["camera_lidar_optimizer"].required_libraries == (
        "lingtu_camera_lidar_optimizer",
    )


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
