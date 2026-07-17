"""Default local autonomy fallbacks must stay inside the process.

The product path prefers nanobind/C++ kernels.  Missing native product
backends must fail fast or use explicit ROS-free Python test backends, never
resurrect legacy NativeModule subprocesses.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
MODULES_DIR = ROOT / "src" / "nav" / "local"
LOCAL_PLANNER_SERVICE = ROOT / "src" / "nav" / "services" / "plan" / "local_planner" / "service.py"
LEGACY_ROS2_PACKAGE_DIRS = (
    ROOT / "src" / "nav" / "local" / "legacy_ros" / "local_planner",
    ROOT / "src" / "nav" / "local" / "legacy_ros" / "sensor_scan_generation",
    ROOT / "src" / "nav" / "local" / "legacy_ros" / "terrain_analysis",
    ROOT / "src" / "nav" / "local" / "legacy_ros" / "terrain_analysis_ext",
)

FORBIDDEN_IMPORT_PREFIXES = (
    "runtime.adapters.ros2",
    "runtime.adapters.ros2.native_module",
    "rclpy",
    "geometry_msgs",
    "nav_msgs",
    "sensor_msgs",
    "std_msgs",
    "subprocess",
)

FORBIDDEN_CALLS = {
    "NativeModule",
    "NativeModuleConfig",
    "subprocess.Popen",
    "subprocess.run",
    "subprocess.call",
    "subprocess.check_call",
    "subprocess.check_output",
}
FORBIDDEN_NAMES = {"NativeModule", "NativeModuleConfig"}


def _absolute_imports(tree: ast.AST) -> list[tuple[int, str]]:
    imports: list[tuple[int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend((node.lineno, alias.name) for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.append((node.lineno, node.module))
    return imports


def _call_name(node: ast.Call) -> str:
    parts: list[str] = []
    current: ast.AST = node.func
    while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
    if isinstance(current, ast.Name):
        parts.append(current.id)
    else:
        return ""
    return ".".join(reversed(parts))


def test_base_autonomy_modules_stay_inside_the_process() -> None:
    """Local autonomy Modules must not regain ROS2 or subprocess boundaries."""

    violations: list[str] = []
    for path in sorted(MODULES_DIR.glob("*.py")):
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))

        for lineno, imported in _absolute_imports(tree):
            if any(imported == prefix or imported.startswith(f"{prefix}.") for prefix in FORBIDDEN_IMPORT_PREFIXES):
                violations.append(f"{rel}:{lineno}: imports {imported}")

        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and node.id in FORBIDDEN_NAMES:
                violations.append(f"{rel}:{node.lineno}: references {node.id}")
            elif isinstance(node, ast.Call):
                name = _call_name(node)
                if name in FORBIDDEN_CALLS or name.startswith("subprocess."):
                    violations.append(f"{rel}:{node.lineno}: calls {name}")

    assert violations == []


def test_base_autonomy_backend_contracts_expose_only_in_process_backends() -> None:
    """Backend names are part of the ROS-free product contract."""

    import nav.local.contracts as local_contracts
    from nav.local.contracts import PATH_FOLLOWER_BACKENDS, TERRAIN_BACKENDS
    from nav.services.plan.contracts import LOCAL_PLANNER_BACKENDS

    assert TERRAIN_BACKENDS == ("nanobind", "simple")
    assert LOCAL_PLANNER_BACKENDS == ("nanobind", "cmu_py", "simple")
    assert PATH_FOLLOWER_BACKENDS == ("nav_kernel", "pid")
    assert not hasattr(local_contracts, "LOCAL_PLANNER_BACKENDS")
    assert not hasattr(local_contracts, "require_local_planner_backend")

    all_backends = TERRAIN_BACKENDS + LOCAL_PLANNER_BACKENDS + PATH_FOLLOWER_BACKENDS
    assert "native" not in all_backends
    assert "cmu" not in all_backends
    assert "pure_pursuit" not in all_backends


def test_base_autonomy_readme_does_not_advertise_ros2_module_backends() -> None:
    """Docs must not reintroduce rejected ROS2 NativeModule backend choices."""

    readme = (ROOT / "src" / "nav" / "local" / "README.md").read_text(encoding="utf-8")

    assert "| `native`" not in readme
    assert "| `cmu`" not in readme
    assert "| `pure_pursuit`" not in readme
    assert "subprocess via NativeModule" not in readme
    assert "normal path is in-process" in readme


def test_legacy_ros2_local_autonomy_packages_have_been_removed() -> None:
    """Historical ROS2 shells were retired outright, not just COLCON_IGNORE'd.

    Per ``docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md``: these four packages
    were confirmed unused by every production profile (they were
    already ``COLCON_IGNORE``'d and superseded by the nanobind ``nav_kernel`` /
    ``nav/services/plan/local_planner/cpp`` native path), so the formalized
    retirement removes the source outright rather than leaving it quarantined
    in place. Git history retains the original source if ever needed.
    """

    still_present = [
        package_dir.relative_to(ROOT).as_posix() for package_dir in LEGACY_ROS2_PACKAGE_DIRS if package_dir.exists()
    ]
    assert still_present == []
    assert not (ROOT / "src" / "nav" / "local" / "legacy_ros").exists()


def test_local_planner_product_runtime_uses_core_messages_not_ros_messages() -> None:
    """Fast deployment path uses LingTu messages and keeps ROS at adapters."""

    source = LOCAL_PLANNER_SERVICE.read_text(encoding="utf-8")

    assert "from runtime.msgs.nav import Odometry, Path" in source
    assert "from runtime.msgs.sensor import PointCloud2" in source
    assert "from runtime.msgs.geometry import Pose, PoseStamped" in source
    for token in (
        "rclcpp",
        "geometry_msgs",
        "nav_msgs",
        "sensor_msgs",
        "std_msgs",
    ):
        assert token not in source


def test_default_autonomy_fallback_chain_never_uses_native_module(
    monkeypatch,
) -> None:
    """Missing nav_kernel never resurrects ROS2 NativeModule fallbacks."""

    from nav.local import path_follower as path_follower_mod
    from nav.local import path_follower_runtime
    from nav.local import terrain as terrain_module
    from nav.local.path_follower_backend import (
        NavKernelPathFollowerAdapter,
        PidFallbackParams,
    )
    from nav.services.plan.local_planner import runtime as local_planner_setup
    from nav.services.plan.local_planner import service as local_planner
    from nav.services.plan.local_planner.backend import (
        LocalPlannerGridConfig,
        NanobindLocalPlannerBackend,
    )

    assert not hasattr(terrain_module, "create_native_terrain_backend")
    assert not hasattr(local_planner_setup, "create_cmu_native_module")
    assert not hasattr(path_follower_runtime, "create_pure_pursuit_native_adapter")

    def missing_terrain_core():
        raise RuntimeError(
            "Terrain [nanobind]: compatible LingTu native navigation kernel not found.\n"
            "  To build: scripts/build/build_nav_kernel.sh"
        )

    monkeypatch.setattr(
        terrain_module,
        "create_nanobind_terrain_backend",
        missing_terrain_core,
    )

    monkeypatch.setattr(
        local_planner_setup,
        "read_local_planner_grid_config",
        lambda: LocalPlannerGridConfig(),
    )
    monkeypatch.setattr(
        local_planner_setup,
        "create_nanobind_backend",
        lambda: NanobindLocalPlannerBackend(
            core=None,
            unavailable_reason="compatible LingTu native navigation kernel missing",
            build_hint="build nav core",
        ),
    )
    monkeypatch.setattr(
        local_planner_setup,
        "create_cmu_py_backend",
        lambda *_args, **_kwargs: pytest.fail("cmu_py must not be auto-created for nanobind"),
    )

    monkeypatch.setattr(
        path_follower_runtime,
        "create_nav_kernel_path_follower_adapter_from_tuning",
        lambda **_tuning: NavKernelPathFollowerAdapter(
            runtime=None,
            degraded_reason="compatible LingTu native navigation kernel missing",
            build_hint="build nav core",
        ),
    )
    monkeypatch.setattr(
        path_follower_runtime,
        "read_pid_fallback_params",
        lambda max_speed: PidFallbackParams(
            k_v=0.5,
            l_min=0.5,
            l_max=2.0,
            a_max=1.0,
            v_max=max_speed,
        ),
    )

    terrain = terrain_module.Terrain(backend="nanobind")
    local_planner = local_planner.LocalPlanner(backend="nanobind")
    path_follower = path_follower_mod.PathFollower(backend="nav_kernel")

    terrain.setup()
    with pytest.raises(RuntimeError, match="compatible LingTu native navigation kernel missing"):
        local_planner.setup()
    with pytest.raises(RuntimeError, match="compatible LingTu native navigation kernel missing"):
        path_follower.setup()

    assert terrain._backend == "simple"
    assert local_planner._backend == "nanobind"
    assert path_follower._backend == "nav_kernel"


def test_legacy_ros2_autonomy_backend_names_are_rejected() -> None:
    """Old NativeModule backend names must fail before setup can spawn anything."""

    from nav.local.path_follower import PathFollower
    from nav.local.terrain import Terrain
    from nav.services.plan.local_planner.service import LocalPlanner

    with pytest.raises(ValueError, match="Unknown terrain backend 'native'"):
        Terrain(backend="native")
    with pytest.raises(ValueError, match="Unknown terrain backend 'cmu'"):
        Terrain(backend="cmu")
    with pytest.raises(ValueError, match="Unknown local_planner backend 'cmu'"):
        LocalPlanner(backend="cmu")
    with pytest.raises(ValueError, match="Unknown path_follower backend 'pure_pursuit'"):
        PathFollower(backend="pure_pursuit")
