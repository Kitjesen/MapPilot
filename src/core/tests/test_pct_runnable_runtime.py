from __future__ import annotations

import importlib.util
import sys
import os
from pathlib import Path
import pickle

import pytest
from global_planning.pct_planner_runnable import runtime as pct_runtime
from global_planning.pct_planner_runnable.runtime import (
    inspect_pct_runtime,
    prepare_pct_runtime,
    prepare_tomogram_for_pct,
    resolve_pct_runtime_paths,
)


def _touch_pct_extensions(lib_dir: Path) -> None:
    abi = f"{sys.version_info.major}{sys.version_info.minor}"
    for name in ("a_star", "ele_planner", "traj_opt"):
        (lib_dir / f"{name}.cpython-{abi}-x86_64-linux-gnu.so").write_bytes(b"")


def _touch_pct_shared_libs(lib_dir: Path) -> None:
    for name in (
        "libmetis-gtsam.so",
        "libgtsam.so",
        "libcommon_smoothing.so",
        "liba_star_search.so",
        "libmap_manager.so",
        "libgpmp_optimizer.so",
        "libele_planner_lib.so",
    ):
        (lib_dir / name).write_bytes(b"")


def _touch_pct_runtime(lib_dir: Path) -> None:
    _touch_pct_extensions(lib_dir)
    _touch_pct_shared_libs(lib_dir)


def test_resolve_prefers_arch_specific_lib_dir(tmp_path):
    repo = tmp_path
    lib_dir = repo / "src/global_planning/pct_planner/planner/lib/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == lib_dir
    assert paths.canonical_arch == "x86_64"


def test_resolve_supports_legacy_uppercase_pct_planner_dir(tmp_path):
    repo = tmp_path
    planner_root = repo / "src/global_planning/PCT_planner/planner"
    lib_dir = planner_root / "lib/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")
    report = inspect_pct_runtime(repo, machine="x86_64")

    assert paths.planner_root.samefile(planner_root)
    assert paths.lib_root.samefile(planner_root / "lib")
    assert paths.lib_dir.samefile(lib_dir)
    assert Path(report["lib_dir"]).samefile(lib_dir)


def test_pct_config_defaults_keep_optimizer_enabled():
    global_planning_root = Path(__file__).resolve().parents[2] / "global_planning"
    config_path = next(
        (
            global_planning_root / name / "planner" / "config" / "param.py"
            for name in ("pct_planner", "PCT_planner")
            if (global_planning_root / name / "planner" / "config" / "param.py").is_file()
        ),
        global_planning_root / "pct_planner" / "planner" / "config" / "param.py",
    )
    spec = importlib.util.spec_from_file_location("pct_param_test", config_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.Config.planner.optimize_trajectory is True


def test_host_build_script_requires_native_lib_source_before_selecting_planner_dir():
    script_path = (
        Path(__file__).resolve().parents[2]
        / "global_planning"
        / "pct_planner_runnable"
        / "build_host_x86_64.sh"
    )
    script = script_path.read_text(encoding="utf-8-sig")

    assert not script_path.read_bytes().startswith(b"\xef\xbb\xbf")
    assert 'if [[ -f "${candidate}/lib/CMakeLists.txt" ]]' in script
    assert 'pct_planner_DIR="${candidate}"' in script


def test_resolve_prefers_runnable_native_dir_over_original_lib(tmp_path):
    repo = tmp_path
    original_dir = repo / "src/global_planning/pct_planner/planner/lib/x86_64"
    runnable_dir = repo / "src/global_planning/pct_planner_runnable/native/x86_64"
    original_dir.mkdir(parents=True)
    runnable_dir.mkdir(parents=True)
    _touch_pct_runtime(original_dir)
    _touch_pct_runtime(runnable_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == runnable_dir


def test_resolve_prefers_env_lib_dir_over_runnable_native_dir(tmp_path, monkeypatch):
    repo = tmp_path
    env_dir = repo / "custom_pct_lib"
    runnable_dir = repo / "src/global_planning/pct_planner_runnable/native/x86_64"
    env_dir.mkdir(parents=True)
    runnable_dir.mkdir(parents=True)
    _touch_pct_runtime(env_dir)
    _touch_pct_runtime(runnable_dir)
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == env_dir


def test_inspect_pct_runtime_uses_same_env_dir_as_loader(tmp_path, monkeypatch):
    repo = tmp_path
    env_dir = repo / "custom_pct_lib"
    fallback_dir = repo / "src/global_planning/pct_planner_runnable/native/x86_64"
    env_dir.mkdir(parents=True)
    fallback_dir.mkdir(parents=True)
    _touch_pct_runtime(env_dir)
    _touch_pct_runtime(fallback_dir)
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    report = inspect_pct_runtime(repo, machine="x86_64")

    assert report["ok"] is True
    assert Path(report["lib_dir"]) == env_dir
    assert report["missing"] == []


def test_prepare_installs_lib_namespace_for_original_wrapper(tmp_path):
    repo = tmp_path
    planner_root = repo / "src/global_planning/pct_planner/planner"
    lib_dir = planner_root / "lib/x86_64"
    scripts_dir = planner_root / "scripts"
    lib_dir.mkdir(parents=True)
    scripts_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    previous_lib = sys.modules.pop("lib", None)
    try:
        paths = prepare_pct_runtime(repo, machine="x86_64", preload_shared=False)
        lib_module = sys.modules["lib"]
        assert str(paths.lib_dir) in list(lib_module.__path__)
        assert str(paths.lib_root) in list(lib_module.__path__)
        assert str(paths.scripts_dir) in sys.path
    finally:
        if previous_lib is not None:
            sys.modules["lib"] = previous_lib
        else:
            sys.modules.pop("lib", None)


def test_prepare_reports_unloadable_shared_library(tmp_path):
    if os.name == "nt":
        pytest.skip("Windows does not preload Linux shared libraries")
    repo = tmp_path
    lib_dir = repo / "src/global_planning/pct_planner/planner/lib/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    with pytest.raises(RuntimeError, match="PCT shared library failed to load"):
        prepare_pct_runtime(repo, machine="x86_64")


def test_inspect_pct_runtime_rejects_missing_shared_libraries(tmp_path):
    repo = tmp_path
    lib_dir = repo / "src/global_planning/pct_planner/planner/lib/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_extensions(lib_dir)

    report = inspect_pct_runtime(repo, machine="x86_64")

    assert report["ok"] is False
    assert "libmetis-gtsam.so" in report["shared_missing"]
    assert "Shared library gaps" in report["error"]


def test_inspect_pct_runtime_reports_linux_native_host_boundary(tmp_path, monkeypatch):
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Windows")

    report = inspect_pct_runtime(tmp_path, machine="x86_64")

    assert report["native_binary_format"] == "linux_elf"
    assert report["platform_system"] == "windows"
    assert report["host_platform_supported"] is False
    assert "Linux" in report["host_platform_blocker"]


def test_inspect_pct_runtime_reports_wrong_abi_candidate_extensions(
    tmp_path,
    monkeypatch,
):
    repo = tmp_path
    lib_dir = repo / "src/global_planning/pct_planner/planner/lib/x86_64"
    lib_dir.mkdir(parents=True)
    for name in ("a_star", "ele_planner", "traj_opt"):
        (lib_dir / f"{name}.cpython-310-x86_64-linux-gnu.so").write_bytes(b"")
    _touch_pct_shared_libs(lib_dir)
    monkeypatch.setattr(pct_runtime, "_python_tag", lambda: "py313")

    report = inspect_pct_runtime(repo, machine="x86_64")

    candidate = next(
        item
        for item in report["candidate_diagnostics"]
        if item["path"] == str(lib_dir)
    )
    assert candidate["exists"] is True
    assert candidate["has_current_abi_extensions"] is False
    assert sorted(candidate["available_extension_modules"]) == [
        "a_star.cpython-310-x86_64-linux-gnu.so",
        "ele_planner.cpython-310-x86_64-linux-gnu.so",
        "traj_opt.cpython-310-x86_64-linux-gnu.so",
    ]
    assert report["python_abi_matches_known_good"] is False
    assert report["known_good_python_tag"] == "py310"
    assert "build_host_x86_64.sh" in report["recommended_build_command"]


def test_prepare_tomogram_for_pct_transposes_builder_axes(tmp_path):
    if os.name == "nt":
        pytest.skip("Windows numpy import is unstable in this environment")
    import numpy as np

    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared != source
    with prepared.open("rb") as handle:
        normalized = pickle.load(handle)
    assert normalized["pct_axes_transposed"] is True
    assert normalized["data"].shape == (5, 2, 4, 3)


def test_prepare_tomogram_for_pct_returns_source_when_axes_already_transposed(tmp_path):
    if os.name == "nt":
        pytest.skip("Windows numpy import is unstable in this environment")
    import numpy as np

    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 4, 3), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
                "pct_axes_transposed": True,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared == source.resolve()
    assert not (tmp_path / ".pct_runnable_cache").exists()


def test_prepare_tomogram_for_pct_returns_cmu_yx_tomogram_source(tmp_path):
    if os.name == "nt":
        pytest.skip("Windows numpy import is unstable in this environment")
    import numpy as np

    source = tmp_path / "cmu_flat.pickle"
    data = np.zeros((5, 1, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
                "grid_info": {"axis_order": "row_y_col_x"},
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared == source.resolve()
    assert not (tmp_path / ".pct_runnable_cache").exists()


def test_prepare_tomogram_for_pct_reuses_cache_for_same_source_stat(tmp_path):
    if os.name == "nt":
        pytest.skip("Windows numpy import is unstable in this environment")
    import numpy as np

    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    first = prepare_tomogram_for_pct(source)
    second = prepare_tomogram_for_pct(source)

    assert second == first


def test_prepare_tomogram_for_pct_falls_back_when_map_cache_is_unwritable(tmp_path, monkeypatch):
    if os.name == "nt":
        pytest.skip("Windows numpy import is unstable in this environment")
    import numpy as np

    blocked_parent = tmp_path / "blocked_parent"
    blocked_parent.write_text("file blocks mkdir parents", encoding="utf-8")
    fallback_dir = tmp_path / "fallback_cache"
    monkeypatch.setattr(
        pct_runtime,
        "_pct_cache_dirs",
        lambda source: [blocked_parent / ".pct_runnable_cache", fallback_dir],
    )
    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared.parent == fallback_dir
    assert prepared.exists()
