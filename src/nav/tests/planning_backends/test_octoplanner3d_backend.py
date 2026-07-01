"""Tests for optional OctoPlanner3D planner backend.

These tests do not require PCL, OctoMap, ROS2, or a compiled OctoPlanner3D
binary. The real native planner is optional; the Python backend must import and
fail soft when the headless executable is unavailable.
"""

from __future__ import annotations

import builtins
import importlib
import inspect
import json
import subprocess
import sys
from pathlib import Path

import pytest

from runtime.msgs.numpy_compat import np
from runtime.registry import get, get_metadata, list_plugins, restore, snapshot
from lingtu.plugin_seed import seed_builtin_plugins

MODULE = "nav.services.plan.global_planner.algorithm.octoplanner3d"
RUNTIME_MODULE = "nav.services.plan.global_planner.algorithm.octoplanner3d_runtime"
REGISTRY_KEY = "octoplanner3d"
REPO_ROOT = Path(__file__).resolve().parents[4]
OCTO_NATIVE_DIR = (
    REPO_ROOT
    / "src"
    / "nav"
    / "services"
    / "plan"
    / "global_planner"
    / "algorithm"
    / "OctoPlanner3D"
    / "runtime"
)


@pytest.fixture(autouse=True)
def _registry_isolation():
    state = snapshot()
    yield
    restore(state)


@pytest.fixture(autouse=True)
def _runtime_env_isolation(monkeypatch):
    monkeypatch.setenv("LINGTU_OCTOPLANNER3D_NATIVE_MODULE", "none")
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_EXECUTABLE", raising=False)
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_WSL_EXECUTABLE", raising=False)
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_TIMEOUT_S", raising=False)


def _dummy_map(tmp_path):
    path = tmp_path / "map.bt"
    path.write_bytes(b"dummy octomap placeholder; native executable is not invoked")
    return path


def _missing_backend(tmp_path):
    module = importlib.import_module(MODULE)
    missing_exe = tmp_path / "octoplanner3d_headless_missing"
    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(_dummy_map(tmp_path)),
        executable_path=str(missing_exe),
    )
    return backend, missing_exe


def _runtime_module():
    return importlib.import_module(RUNTIME_MODULE)


def test_seed_builtin_plugins_registers_octoplanner3d_backend(tmp_path):
    seed_builtin_plugins(groups=("planner_backend",), reload_loaded=True, strict=True)

    backends = list_plugins("planner_backend")
    assert REGISTRY_KEY in backends

    BackendCls = get("planner_backend", REGISTRY_KEY)
    assert BackendCls.__name__ == "OctoPlanner3DBackend"
    assert BackendCls.__module__ == MODULE

    instance = BackendCls(
        tomogram_path=str(_dummy_map(tmp_path)),
        executable_path=str(tmp_path / "missing-globalPlan"),
    )
    assert hasattr(instance, "available")
    assert callable(instance.plan)
    assert callable(instance.update_map)

    meta = get_metadata("planner_backend", REGISTRY_KEY)
    description = meta.get("description", "")
    assert "OctoPlanner3D" in description
    assert "OctoMap" in description
    assert "optional" in description.lower()
    assert "in-process native" in description
    assert meta["supported_map_extensions"] == [".bt", ".ot", ".octomap", ".pcd"]
    assert meta["input_schema"]["map_path"]["accepted_extensions"] == [".bt", ".ot", ".octomap", ".pcd"]
    assert meta["input_schema"]["start"]["type"] == "array[3]float"
    assert meta["input_schema"]["goal"]["frame"] == "LingTu planning/map frame"
    assert meta["output_schema"]["path"]["type"] == "array[array[3]float]"
    assert meta["output_schema"]["reached_goal"]["type"] == "bool"
    formats = {item["extension"]: item for item in meta["supported_map_formats"]}
    assert formats[".bt"]["source_kind"] == "octomap_file"
    assert formats[".bt"]["requires"] == []
    assert formats[".ot"]["source_kind"] == "octomap_file"
    assert formats[".octomap"]["source_kind"] == "octomap_file"
    assert formats[".pcd"]["source_kind"] == "point_cloud_file"
    assert formats[".pcd"]["requires"] == ["PCL"]


def test_octoplanner3d_backend_imports_without_ros2(monkeypatch):
    blocked_roots = (
        "rclpy",
        "ament_index_python",
        "geometry_msgs",
        "nav_msgs",
        "sensor_msgs",
        "std_msgs",
        "visualization_msgs",
        "octomap_msgs",
    )

    for name in list(sys.modules):
        if any(name == root or name.startswith(root + ".") for root in blocked_roots):
            monkeypatch.delitem(sys.modules, name, raising=False)

    real_import = builtins.__import__

    def forbid_ros2_import(name, *args, **kwargs):
        if any(name == root or name.startswith(root + ".") for root in blocked_roots):
            raise AssertionError(f"OctoPlanner3D backend imported ROS2 module: {name}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", forbid_ros2_import)
    sys.modules.pop(MODULE, None)

    module = importlib.import_module(MODULE)

    assert hasattr(module, "OctoPlanner3DBackend")
    assert all(root not in sys.modules for root in blocked_roots)


def test_missing_executable_marks_backend_unavailable(tmp_path):
    backend, missing_exe = _missing_backend(tmp_path)

    assert not missing_exe.exists()
    assert backend.available is False
    assert backend._load_error
    assert str(missing_exe) in backend._load_error

    path = backend.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 2.0, 0.5]))
    assert path == []
    assert backend._last_plan_error == "octoplanner3d backend unavailable"
    assert backend._last_plan_reached_goal is False
    assert backend._last_plan_diagnostics["planner"] == REGISTRY_KEY
    assert backend._last_plan_diagnostics["available"] is False
    assert backend._last_plan_diagnostics["runtime_mode"] == "headless_executable"
    assert backend._last_plan_diagnostics["process_boundary"] == "headless_subprocess"
    assert backend._last_plan_diagnostics["load_error"] == backend._load_error
    assert backend._last_plan_diagnostics["supported_map_extensions"] == [".bt", ".ot", ".octomap", ".pcd"]
    assert backend._last_plan_diagnostics["input_schema"]["map_path"]["required"] is True
    assert backend._last_plan_diagnostics["output_schema"]["path"]["frame"] == "LingTu planning/map frame"
    assert backend._last_plan_diagnostics["build_capabilities"]["bt_without_pcl"] is True
    formats = {item["extension"]: item for item in backend._last_plan_diagnostics["supported_map_formats"]}
    assert formats[".bt"]["source_kind"] == "octomap_file"
    assert formats[".ot"]["source_kind"] == "octomap_file"
    assert formats[".octomap"]["source_kind"] == "octomap_file"
    assert formats[".pcd"]["requires"] == ["PCL"]


def test_missing_map_path_marks_backend_unavailable(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_EXECUTABLE", raising=False)
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_WSL_EXECUTABLE", raising=False)
    monkeypatch.setattr(runtime_module.OctoPlanner3DRuntime, "local_build_candidates", staticmethod(lambda: []))
    monkeypatch.setattr(runtime_module.OctoPlanner3DRuntime, "local_wsl_build_candidates", staticmethod(lambda: []))

    exe = tmp_path / "octoplanner3d_headless"
    exe.write_text("# fake executable\n", encoding="utf-8")

    backend = module.OctoPlanner3DBackend(tomogram_path="", executable_path=str(exe))

    assert backend.available is False
    assert "OctoPlanner3D map path not configured" in backend._load_error


def test_unsupported_map_format_marks_backend_unavailable(tmp_path):
    module = importlib.import_module(MODULE)
    exe = tmp_path / "octoplanner3d_headless"
    exe.write_text("# fake executable\n", encoding="utf-8")
    pct_tomogram = tmp_path / "building2_9.pickle"
    pct_tomogram.write_bytes(b"not an octomap")

    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(pct_tomogram),
        executable_path=str(exe),
    )

    assert backend.available is False
    assert "OctoPlanner3D map format unsupported: .pickle" in backend._load_error


def test_constructor_keeps_tomogram_path_compatibility():
    module = importlib.import_module(MODULE)

    signature = inspect.signature(module.OctoPlanner3DBackend.__init__)
    params = list(signature.parameters.values())

    assert params[0].name == "self"
    assert params[1].name == "tomogram_path"
    assert params[1].default == ""
    assert params[2].name == "obstacle_thr"
    assert params[2].default == pytest.approx(49.9)
    assert params[3].kind is inspect.Parameter.KEYWORD_ONLY
    assert params[3].name == "executable_path"
    assert params[4].kind is inspect.Parameter.KEYWORD_ONLY
    assert params[4].name == "timeout_s"


def test_update_map_records_grid_for_lingtu_safety_checks(tmp_path):
    backend, _missing_exe = _missing_backend(tmp_path)

    grid = np.zeros((3, 4), dtype=np.float32)
    grid[1, 2] = 99.0

    backend.update_map(grid, resolution=0.25, origin=np.array([-1.0, 2.0]))

    assert backend._grid is not None
    assert backend._costmap is not None
    assert backend._grid.dtype == np.float32
    assert backend._grid.shape == (3, 4)
    assert backend._grid[1, 2] == pytest.approx(99.0)
    assert backend._resolution == pytest.approx(0.25)
    np.testing.assert_allclose(backend._origin, [-1.0, 2.0])
    assert backend._last_plan_diagnostics["map_update"] == "cached"


def test_plan_uses_headless_executable_json_protocol(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    exe = tmp_path / "octoplanner3d_headless"
    exe.write_text("# fake executable; subprocess.run is monkeypatched\n", encoding="utf-8")
    exe.chmod(0o755)

    calls = []

    def fake_run(
        cmd,
        *,
        input=None,
        text=None,
        encoding=None,
        errors=None,
        capture_output=None,
        timeout=None,
        check=None,
    ):
        calls.append(
            {
                "cmd": cmd,
                "input": input,
                "text": text,
                "encoding": encoding,
                "errors": errors,
                "capture_output": capture_output,
                "timeout": timeout,
                "check": check,
            }
        )
        return subprocess.CompletedProcess(
            cmd,
            0,
            stdout=json.dumps(
                {
                    "ok": True,
                    "path": [[0.0, 0.0, 0.0], [1.0, 2.0, 0.5]],
                    "reached_goal": True,
                    "diagnostics": {"source": "fake"},
                }
            ),
            stderr="",
        )

    monkeypatch.setattr(runtime_module.subprocess, "run", fake_run)

    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(_dummy_map(tmp_path)),
        executable_path=str(exe),
    )
    assert backend.available is True

    backend.update_map(
        np.zeros((2, 2), dtype=np.float32),
        resolution=0.25,
        origin=np.array([4.0, 5.0]),
    )

    path = backend.plan(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 2.0, 0.5]),
    )

    assert len(path) == 2
    np.testing.assert_allclose(path[0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(path[1], [1.0, 2.0, 0.5])
    assert backend._last_plan_error == ""
    assert backend._last_plan_reached_goal is True

    assert calls
    assert calls[-1]["cmd"] == [str(exe)]
    assert calls[-1]["text"] is True
    assert calls[-1]["encoding"] == "utf-8"
    assert calls[-1]["errors"] == "replace"
    assert calls[-1]["capture_output"] is True

    payload = json.loads(calls[-1]["input"])
    assert set(payload) == {
        "planner",
        "protocol_version",
        "map_path",
        "map_source",
        "map_format",
        "start",
        "goal",
        "obstacle_thr",
        "options",
        "map",
    }
    assert payload["planner"] == REGISTRY_KEY
    assert payload["protocol_version"] == 1
    assert payload["map_path"] == str(_dummy_map(tmp_path))
    assert payload["map_source"]["path"] == str(_dummy_map(tmp_path))
    assert payload["map_source"]["kind"] == "octomap_file"
    assert payload["map_source"]["format"] == "bt"
    assert payload["map_format"] == "bt"
    assert payload["options"]["planner_family"] == "octoplanner3d_constrained_global_planner"
    assert payload["options"]["search_algorithm"] == "octomap_3d_astar"
    assert payload["options"]["constraint_model"] == "quadruped_bounding_cylinder_ground_support"
    assert payload["options"]["robot_radius"] == pytest.approx(0.25)
    assert payload["options"]["require_ground_support"] is True
    assert payload["start"] == [0.0, 0.0, 0.0]
    assert payload["goal"] == [1.0, 2.0, 0.5]
    assert payload["obstacle_thr"] == pytest.approx(49.9)
    assert payload["map"]["resolution"] == pytest.approx(0.25)
    assert payload["map"]["origin"] == pytest.approx([4.0, 5.0])
    assert payload["map"]["shape"] == [2, 2]

    diagnostics = backend._last_plan_diagnostics
    assert diagnostics["planner"] == REGISTRY_KEY
    assert diagnostics["available"] is True
    assert diagnostics["runtime_mode"] == "headless_executable"
    assert diagnostics["process_boundary"] == "headless_subprocess"
    assert diagnostics["build_capabilities"]["in_process_native"] is False
    assert diagnostics["build_capabilities"]["ros2_required"] is False
    assert diagnostics["planner_family"] == "octoplanner3d_constrained_global_planner"
    assert diagnostics["search_algorithm"] == "octomap_3d_astar"
    assert diagnostics["constraints"]["robot_radius"] == pytest.approx(0.25)
    assert diagnostics["source"] == "fake"
    assert diagnostics["path_points"] == 2
    assert diagnostics["supported_map_extensions"] == [".bt", ".ot", ".octomap", ".pcd"]


def test_in_process_native_kernel_takes_precedence_over_headless_executable(
    tmp_path,
    monkeypatch,
):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()

    class FakeNativeKernel:
        def __init__(self):
            self.payloads = []

        def plan(self, payload):
            self.payloads.append(payload)
            return {
                "ok": True,
                "path": [[0.0, 0.0, 0.0], [2.0, 3.0, 1.0]],
                "reached_goal": True,
                "diagnostics": {"source": "fake-native"},
            }

    fake_kernel = FakeNativeKernel()
    monkeypatch.setattr(
        runtime_module.OctoPlanner3DRuntime,
        "resolve_native_kernel",
        staticmethod(lambda: (fake_kernel, "fake_octoplanner_native")),
    )
    monkeypatch.setattr(runtime_module.OctoPlanner3DRuntime, "local_build_candidates", staticmethod(lambda: []))
    monkeypatch.setattr(runtime_module.OctoPlanner3DRuntime, "local_wsl_build_candidates", staticmethod(lambda: []))
    monkeypatch.setattr(runtime_module.shutil, "which", lambda _name: None)

    def fail_subprocess(*_args, **_kwargs):
        raise AssertionError("in-process native kernel must not spawn the headless executable")

    monkeypatch.setattr(runtime_module.subprocess, "run", fail_subprocess)

    backend = module.OctoPlanner3DBackend(tomogram_path=str(_dummy_map(tmp_path)))

    assert backend.available is True
    assert backend.runtime_mode == "in_process_native"
    assert backend.executable_path == ""

    path = backend.plan([0.0, 0.0, 0.0], [2.0, 3.0, 1.0])

    assert len(path) == 2
    np.testing.assert_allclose(path[-1], [2.0, 3.0, 1.0])
    assert fake_kernel.payloads
    assert fake_kernel.payloads[-1]["planner"] == REGISTRY_KEY
    assert fake_kernel.payloads[-1]["map_format"] == "bt"
    assert fake_kernel.payloads[-1]["start"] == [0.0, 0.0, 0.0]
    assert fake_kernel.payloads[-1]["goal"] == [2.0, 3.0, 1.0]
    assert fake_kernel.payloads[-1]["options"]["planner_family"] == (
        "octoplanner3d_constrained_global_planner"
    )
    assert fake_kernel.payloads[-1]["options"]["search_algorithm"] == "octomap_3d_astar"

    diagnostics = backend._last_plan_diagnostics
    assert diagnostics["stage"] == "in_process_native_success"
    assert diagnostics["runtime_mode"] == "in_process_native"
    assert diagnostics["process_boundary"] == "in_process"
    assert diagnostics["native_module"] == "fake_octoplanner_native"
    assert diagnostics["build_capabilities"]["in_process_native"] is True
    assert diagnostics["build_capabilities"]["headless_executable"] is False
    assert diagnostics["build_capabilities"]["ros2_required"] is False
    assert diagnostics["source"] == "fake-native"
    assert diagnostics["path_points"] == 2


def test_octoplanner3d_constraints_are_configurable_in_payload(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    exe = tmp_path / "octoplanner3d_headless"
    exe.write_text("# fake executable; subprocess.run is monkeypatched\n", encoding="utf-8")
    exe.chmod(0o755)
    calls = []

    def fake_run(cmd, **kwargs):
        calls.append({"cmd": cmd, **kwargs})
        return subprocess.CompletedProcess(
            cmd,
            0,
            stdout=json.dumps(
                {
                    "ok": True,
                    "path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                    "reached_goal": True,
                    "diagnostics": {"source": "fake"},
                }
            ),
            stderr="",
        )

    monkeypatch.setattr(runtime_module.subprocess, "run", fake_run)

    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(_dummy_map(tmp_path)),
        executable_path=str(exe),
    )
    backend.configure_constraints(
        {
            "robot_radius": 0.6,
            "strict_direct_ground_support": True,
            "ground_support_depth_cells": 2,
            "preblocked_costmap_weight": 4.0,
            "lowest_traversable_only": True,
        }
    )

    assert backend.plan([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    payload = json.loads(calls[-1]["input"])

    assert payload["options"]["robot_radius"] == pytest.approx(0.6)
    assert payload["options"]["strict_direct_ground_support"] is True
    assert payload["options"]["ground_support_depth_cells"] == 2
    assert payload["options"]["preblocked_costmap_weight"] == pytest.approx(4.0)
    assert payload["options"]["lowest_traversable_only"] is True
    assert backend._last_plan_diagnostics["constraints"]["robot_radius"] == pytest.approx(0.6)


def test_nonzero_headless_json_failure_preserves_native_diagnostics(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    exe = tmp_path / "octoplanner3d_headless"
    exe.write_text("# fake executable; subprocess.run is monkeypatched\n", encoding="utf-8")
    exe.chmod(0o755)

    def fake_run(*args, **kwargs):
        cmd = args[0]
        return subprocess.CompletedProcess(
            cmd,
            2,
            stdout='native logs\n{"ok":false,"error":"empty path","diagnostics":{"path_points":0}}\n',
            stderr="",
        )

    monkeypatch.setattr(runtime_module.subprocess, "run", fake_run)

    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(_dummy_map(tmp_path)),
        executable_path=str(exe),
    )
    path = backend.plan([0.0, 0.0, 0.0], [1.0, 1.0, 0.0])

    assert path == []
    assert backend._last_plan_error == "empty path"
    assert backend._last_plan_diagnostics["stage"] == "native_plan_failed"
    assert backend._last_plan_diagnostics["returncode"] == 2
    assert backend._last_plan_diagnostics["path_points"] == 0


def test_wsl_executable_bridge_converts_map_path_and_command(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    monkeypatch.setenv(
        "LINGTU_OCTOPLANNER3D_WSL_EXECUTABLE",
        "/mnt/d/inovxio/brain/lingtu/build/octoplanner3d_headless_wsl/octoplanner3d_headless",
    )
    monkeypatch.setattr(runtime_module.shutil, "which", lambda name: "/usr/bin/wsl.exe" if name == "wsl.exe" else None)

    map_path = tmp_path / "building2_9.bt"
    map_path.write_bytes(b"dummy")
    backend = module.OctoPlanner3DBackend(
        tomogram_path=str(map_path),
    )

    expected_map_path = runtime_module.OctoPlanner3DRuntime.to_wsl_path(str(map_path))
    assert backend.available is True
    assert backend._runtime_map_path() == expected_map_path
    assert backend._runtime.command() == [
        "/usr/bin/wsl.exe",
        "bash",
        "-lc",
        "exec /mnt/d/inovxio/brain/lingtu/build/octoplanner3d_headless_wsl/octoplanner3d_headless",
    ]
    payload = backend._build_payload(
        np.asarray([0.0, 0.0, 0.0]),
        np.asarray([1.0, 2.0, 0.5]),
    )
    assert payload["map_path"] == expected_map_path


def test_default_wsl_build_output_is_auto_discovered(tmp_path, monkeypatch):
    module = importlib.import_module(MODULE)
    runtime_module = _runtime_module()
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_EXECUTABLE", raising=False)
    monkeypatch.delenv("LINGTU_OCTOPLANNER3D_WSL_EXECUTABLE", raising=False)
    monkeypatch.setattr(runtime_module.shutil, "which", lambda name: "/usr/bin/wsl.exe" if name == "wsl.exe" else None)

    exe = tmp_path / "build" / "octoplanner3d_headless_wsl" / "octoplanner3d_headless"
    exe.parent.mkdir(parents=True)
    exe.write_text("# fake linux executable\n", encoding="utf-8")
    map_path = tmp_path / "building2_9.bt"
    map_path.write_bytes(b"dummy")

    monkeypatch.setattr(runtime_module.OctoPlanner3DRuntime, "local_build_candidates", staticmethod(lambda: []))
    monkeypatch.setattr(
        runtime_module.OctoPlanner3DRuntime,
        "local_wsl_build_candidates",
        staticmethod(lambda: [exe]),
    )

    backend = module.OctoPlanner3DBackend(tomogram_path=str(map_path))

    assert backend.executable_path == ""
    assert backend.available is True
    assert backend._runtime.wsl_executable_path == runtime_module.OctoPlanner3DRuntime.to_wsl_path(str(exe))
    assert backend._runtime_map_path() == runtime_module.OctoPlanner3DRuntime.to_wsl_path(str(map_path))
    assert backend._runtime.command() == [
        "/usr/bin/wsl.exe",
        "bash",
        "-lc",
        "exec " + runtime_module.shlex.quote(backend._runtime.wsl_executable_path),
    ]


def test_headless_wrapper_is_a_non_ros2_octoplanner3d_core_adapter():
    source = (OCTO_NATIVE_DIR / "octoplanner3d_headless.cpp").read_text(encoding="utf-8")
    core = (OCTO_NATIVE_DIR / "octoplanner3d_core.cpp").read_text(encoding="utf-8")
    cmake = (OCTO_NATIVE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    combined = source + "\n" + core + "\n" + cmake

    assert '#include "octoplanner3d_core.hpp"' in source
    assert "octoplanner3d::runtime::runPlan(request)" in source
    assert '#include "global_planner.h"' in core
    assert '#include "pcd2octomap_converter.h"' in core
    assert "OCTOPLANNER3D_ENABLE_PCD" in core
    assert "PCD input requires a headless build with PCL support" in core
    assert "global_planner::GlobalPlanner planner" in core
    assert "applyPlannerOptions(planner, request.options)" in core
    assert "planner.robot_radius_ = options.robot_radius" in core
    assert "planner.require_ground_support_ = options.require_ground_support" in core
    assert "product code does not patch imported source" in core
    assert "planner.setOctomap(map)" in core
    assert "planner.makePlan(start, goal)" in core
    assert "planner.getPlannerResults(native_path)" in core
    assert 'extractString(json, "map_path")' in source
    assert 'extractNumberArray(json, "start")' in source
    assert 'extractNumberArray(json, "goal")' in source
    assert 'applyOptionsFromJson(request.options, json)' in source
    assert 'emitConstraints(result.options)' in source
    assert '\\"path\\"' in source

    forbidden_ros2_terms = (
        "rclcpp",
        "geometry_msgs",
        "nav_msgs",
        "visualization_msgs",
        "ament_target_dependencies",
        "ament_package",
    )
    for term in forbidden_ros2_terms:
        assert term not in combined


def test_headless_cmake_builds_dedicated_executable_against_upstream_core():
    cmake = (OCTO_NATIVE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_library(octoplanner3d_runtime STATIC" in cmake
    assert "octoplanner3d_core.cpp" in cmake
    assert "add_executable(octoplanner3d_headless octoplanner3d_headless.cpp)" in cmake
    assert "target_link_libraries(octoplanner3d_headless" in cmake
    assert "octoplanner3d_runtime" in cmake
    assert "planner/src/global_planner.cpp" in cmake
    assert "octomap/src/pcd2octomap_converter.cpp" in cmake
    assert "find_package(PCL QUIET COMPONENTS common io octree)" in cmake
    assert "set(OCTOPLANNER3D_ENABLE_PCD OFF)" in cmake
    assert "if(OCTOPLANNER3D_ENABLE_PCD)" in cmake
    assert "PCL not found: building octoplanner3d_headless with .bt OctoMap input support only" in cmake
    assert "PCL found but OctoPlanner3D PCD converter source/header missing" in cmake
    assert "OCTOPLANNER3D_ENABLE_PCD" in cmake
    assert "OCTOPLANNER3D_SOURCE_DIR" in cmake
    assert "colcon" not in cmake


def test_octoplanner3d_cmake_can_build_in_process_python_binding():
    cmake = (OCTO_NATIVE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    binding = (OCTO_NATIVE_DIR / "py_octoplanner3d.cpp").read_text(encoding="utf-8")

    assert "OCTOPLANNER3D_BUILD_PYTHON_BINDINGS" in cmake
    assert "find_package(Python COMPONENTS Interpreter Development REQUIRED)" in cmake
    assert "add_library(_native MODULE py_octoplanner3d.cpp)" in cmake
    assert "target_link_libraries(_native PRIVATE octoplanner3d_runtime Python::Python)" in cmake
    assert 'PREFIX ""' in cmake

    assert "PyMODINIT_FUNC PyInit__native()" in binding
    assert '"plan", pyPlan' in binding
    assert "octo::runPlan(request)" in binding
    assert "PyEval_SaveThread()" in binding
    assert "PyEval_RestoreThread(state)" in binding
    assert "octoplanner3d_cpython" in binding
    assert "rclcpp" not in binding


def test_headless_cmake_does_not_require_pcd_converter_for_octomap_inputs():
    cmake = (OCTO_NATIVE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    forbidden = (
        'message(FATAL_ERROR "OCTOPLANNER3D_SOURCE_DIR must point to a checkout '
        'with octomap/include/pcd2octomap_converter.h'
    )

    assert forbidden not in cmake
    assert "OCTOPLANNER3D_PCD_CONVERTER_HEADER" in cmake
    assert "OCTOPLANNER3D_PCD_CONVERTER_SOURCE" in cmake
    assert "building octoplanner3d_headless with .bt OctoMap input support only" in cmake


def test_build_script_points_at_headless_native_wrapper_not_ros2_launch():
    script = (REPO_ROOT / "scripts" / "build" / "build_octoplanner3d.sh").read_text(
        encoding="utf-8"
    )

    assert "src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/runtime" in script
    assert "octoplanner3d_headless" in script
    assert "--python|--bindings" in script
    assert "OCTOPLANNER3D_BUILD_PYTHON_BINDINGS" in script
    assert "_native*.so" in script
    assert "LINGTU_OCTOPLANNER3D_NATIVE_MODULE" in script
    assert "LINGTU_OCTOPLANNER3D_SOURCE_DIR" in script
    assert "LINGTU_OCTOPLANNER3D_EXECUTABLE" in script
    assert "colcon" not in script
    assert "ros2 launch" not in script.lower()
    assert "ament" not in script.lower()


def test_backend_searches_default_headless_build_output():
    runtime_module = _runtime_module()
    candidates = runtime_module.OctoPlanner3DRuntime.local_build_candidates()
    wsl_candidates = runtime_module.OctoPlanner3DRuntime.local_wsl_build_candidates()

    rendered = [str(path).replace("\\", "/") for path in candidates]
    rendered_wsl = [str(path).replace("\\", "/") for path in wsl_candidates]
    assert any("build/octoplanner3d_headless" in path for path in rendered)
    assert any(path.endswith("octoplanner3d_headless") for path in rendered)
    assert any(path.endswith("octoplanner3d_headless.exe") for path in rendered)
    assert any("build/octoplanner3d_headless_wsl" in path for path in rendered_wsl)
    assert any("build/octoplanner3d_headless_wsl_pcl" in path for path in rendered_wsl)
