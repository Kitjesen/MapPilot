from __future__ import annotations

import importlib.util
import sys
import os
from pathlib import Path
import pickle
import textwrap

import pytest
from nav.services.plan.global_planner.algorithm.pct.runtime import api as pct_runtime
from nav.services.plan.global_planner.algorithm.pct.runtime.api import (
    PCT_LEGACY_NATIVE_ALLOW_ENV,
    PCT_PLANNER_RUNTIME_ENV,
    PCT_RUST_FFI_BINARY_FORMAT,
    PCT_RUST_OPTIMIZER_BIN_ENV,
    PCT_RUST_OPTIMIZER_CALL_ENV,
    PCT_RUST_OPTIMIZER_LIB_ENV,
    inspect_pct_planner_runtime,
    inspect_pct_runtime,
    inspect_rust_gpmp_optimizer_binary,
    inspect_rust_gpmp_optimizer_library,
    load_pct_planner_runtime,
    prepare_pct_runtime,
    prepare_tomogram_for_pct,
    resolve_rust_gpmp_optimizer_call_mode,
    resolve_pct_planner_runtime,
    resolve_pct_runtime_paths,
)
from nav.services.plan.global_planner.algorithm.pct.runtime.preview import build_preview_report


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


def test_pct_planner_runtime_defaults_to_rust_process_on_linux(tmp_path, monkeypatch):
    monkeypatch.delenv(PCT_PLANNER_RUNTIME_ENV, raising=False)
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_BIN_ENV, raising=False)
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Linux")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"
    report = inspect_pct_planner_runtime(repo_root=tmp_path)

    assert report["requested"] == "rust_process"
    assert report["resolved"] == "rust_process"
    assert report["supported"] is True
    assert report["supported_runtimes"] == ["native", "rust_process"]
    assert report["default_runtime"] == "rust_process"


def test_pct_planner_runtime_defaults_to_rust_process_on_windows(tmp_path, monkeypatch):
    monkeypatch.delenv(PCT_PLANNER_RUNTIME_ENV, raising=False)
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_BIN_ENV, raising=False)
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Windows")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"
    report = inspect_pct_planner_runtime(repo_root=tmp_path)

    assert report["requested"] == "rust_process"
    assert report["resolved"] == "rust_process"
    assert report["supported"] is True
    assert report["default_runtime"] == "rust_process"


def test_pct_planner_runtime_auto_stays_rust_process_without_binary(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "auto")
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_BIN_ENV, raising=False)
    monkeypatch.delenv(PCT_LEGACY_NATIVE_ALLOW_ENV, raising=False)
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Linux")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"
    report = inspect_pct_planner_runtime(repo_root=tmp_path)
    assert report["resolved"] == "rust_process"
    assert report["legacy_native_allowed"] is False


def test_pct_planner_runtime_native_requires_explicit_legacy_allow(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.delenv(PCT_LEGACY_NATIVE_ALLOW_ENV, raising=False)

    with pytest.raises(ValueError, match=PCT_LEGACY_NATIVE_ALLOW_ENV):
        resolve_pct_planner_runtime(repo_root=tmp_path)

    report = inspect_pct_planner_runtime(repo_root=tmp_path)
    assert report["supported"] is False
    assert report["resolved"] == ""
    assert report["legacy_native_allowed"] is False
    assert PCT_LEGACY_NATIVE_ALLOW_ENV in report["error"]


def test_pct_planner_runtime_native_resolves_with_explicit_legacy_allow(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "native"
    report = inspect_pct_planner_runtime(repo_root=tmp_path)
    assert report["resolved"] == "native"
    assert report["legacy_native_allowed"] is True


def test_pct_planner_runtime_auto_prefers_rust_process_on_linux_when_binary_exists(
    tmp_path,
    monkeypatch,
):
    fake = tmp_path / "gpmp_fake.py"
    fake.write_text("print('fake')\n", encoding="utf-8")
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "auto")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_BIN_ENV, str(fake))
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Linux")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"
    report = inspect_pct_planner_runtime(repo_root=tmp_path)
    assert report["requested"] == "auto"
    assert report["resolved"] == "rust_process"


def test_pct_planner_runtime_auto_prefers_packaged_rust_artifact(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "auto")
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_BIN_ENV, raising=False)
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Linux")
    artifact_dir = (
        tmp_path
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "runtime"
        / "rust"
        / pct_runtime._canonical_arch()
    )
    artifact_dir.mkdir(parents=True)
    binary = artifact_dir / pct_runtime._rust_gpmp_binary_name()
    binary.write_bytes(b"fake optimizer")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"
    report = inspect_rust_gpmp_optimizer_binary(tmp_path)
    assert report["ok"] is True
    assert Path(report["binary"]) == binary.resolve()


def test_pct_planner_runtime_auto_resolves_to_rust_process_on_windows(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "auto")
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_BIN_ENV, raising=False)
    monkeypatch.setattr(pct_runtime.platform, "system", lambda: "Windows")

    assert resolve_pct_planner_runtime(repo_root=tmp_path) == "rust_process"


def test_pct_planner_runtime_rust_process_requires_optimizer_binary(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "rust_process")

    with pytest.raises(FileNotFoundError, match="No Rust GPMP optimizer binary found"):
        load_pct_planner_runtime(tmp_path / "tomogram.pickle", repo_root=tmp_path)

    report = inspect_pct_runtime(tmp_path, machine="x86_64")
    assert report["ok"] is False
    assert report["planner_runtime"]["requested"] == "rust_process"
    assert report["planner_runtime"]["supported"] is True
    assert any("gpmp_optimize" in item for item in report["missing"])
    assert "No Rust GPMP optimizer" in report["error"]


def test_inspect_rust_gpmp_optimizer_binary_uses_env(tmp_path, monkeypatch):
    fake = tmp_path / "gpmp_fake.py"
    fake.write_text("print('fake')\n", encoding="utf-8")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_BIN_ENV, str(fake))

    report = inspect_rust_gpmp_optimizer_binary(tmp_path)

    assert report["ok"] is True
    assert Path(report["binary"]) == fake


def test_inspect_rust_gpmp_optimizer_library_uses_env(tmp_path, monkeypatch):
    fake = tmp_path / "lingtu_gpmp_trajectory_optimizer.dll"
    fake.write_bytes(b"not a real library")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_LIB_ENV, str(fake))
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_CALL_ENV, "ffi")

    report = inspect_rust_gpmp_optimizer_library(tmp_path)
    runtime_report = inspect_pct_runtime(tmp_path, machine="x86_64")

    assert report["ok"] is True
    assert Path(report["library"]) == fake
    assert resolve_rust_gpmp_optimizer_call_mode() == "ffi"
    assert runtime_report["native_binary_format"] == PCT_RUST_FFI_BINARY_FORMAT
    assert runtime_report["rust_optimizer_call_mode"] == "ffi"
    assert runtime_report["ok"] is True


def test_inspect_rust_gpmp_optimizer_library_uses_packaged_artifact(tmp_path, monkeypatch):
    monkeypatch.delenv(PCT_RUST_OPTIMIZER_LIB_ENV, raising=False)
    artifact_dir = (
        tmp_path
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "runtime"
        / "rust"
        / pct_runtime._canonical_arch()
    )
    artifact_dir.mkdir(parents=True)
    library = artifact_dir / pct_runtime._rust_gpmp_library_names()[0]
    library.write_bytes(b"not a real library")

    report = inspect_rust_gpmp_optimizer_library(tmp_path)

    assert report["ok"] is True
    assert Path(report["library"]) == library.resolve()


def test_rust_process_runtime_loads_and_calls_optimizer_process(tmp_path, monkeypatch):
    np = pytest.importorskip("numpy")
    fake_optimizer = tmp_path / "gpmp_fake.py"
    fake_optimizer.write_text(
        textwrap.dedent(
            """
            import json
            import sys

            req = json.loads(sys.stdin.read())
            states = req["states"]
            layers = req.get("layers") or [0] * len(states)
            heights = req.get("height_hints") or [0.0] * len(states)
            interpolation_steps = int(req.get("interpolation_steps") or 0)
            trajectory_states = []
            trajectory_layers = []
            trajectory_heights = []
            for index, state in enumerate(states):
                trajectory_states.append(state)
                trajectory_layers.append(layers[index])
                trajectory_heights.append(heights[index])
                if index + 1 < len(states):
                    nxt = states[index + 1]
                    for step in range(1, interpolation_steps + 1):
                        fraction = step / (interpolation_steps + 1)
                        trajectory_states.append([
                            lhs + (rhs - lhs) * fraction
                            for lhs, rhs in zip(state, nxt)
                        ])
                        trajectory_layers.append(layers[index])
                        trajectory_heights.append(
                            heights[index] + (heights[index + 1] - heights[index]) * fraction
                        )
            config = req.get("config") or {}
            linear_solver = config.get("linear_solver")
            nonlinear_optimizer = config.get("nonlinear_optimizer")
            print(json.dumps({
                "schema": "lingtu.pct_gpmp.optimize.response.v1",
                "ok": True,
                "error": None,
                "states": states,
                "layers": layers,
                "heights": heights,
                "costs": [1.0] * len(states),
                "trajectory_states": trajectory_states,
                "trajectory_layers": trajectory_layers,
                "trajectory_heights": trajectory_heights,
                "trajectory_costs": [1.0] * len(trajectory_states),
                "initial_trajectory_states": trajectory_states,
                "initial_trajectory_layers": trajectory_layers,
                "report": {
                    "initial_cost": 10.0,
                    "final_cost": 1.0,
                    "iterations": 3,
                    "accepted_steps": 2,
                    "nonlinear_optimizer": nonlinear_optimizer,
                    "linear_solver": linear_solver,
                    "linear_solve_fallbacks": 0,
                },
            }))
            """
        ).strip(),
        encoding="utf-8",
    )
    tomogram = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 1, 9, 9), dtype=np.float32)
    data[0, :, :, :] = 1.0
    data[3, :, :, :] = 0.0
    data[4, :, :, :] = 3.0
    with tomogram.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 1.0,
                "center": [0.0, 0.0],
                "slice_h0": 0.0,
                "slice_dh": 1.0,
                "grid_info": {"axis_order": "row_y_col_x"},
            },
            handle,
        )
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "rust_process")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_BIN_ENV, str(fake_optimizer))
    monkeypatch.setenv("LINGTU_PCT_RUST_LINEAR_SOLVER", "block_tridiagonal")
    monkeypatch.setenv("LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER", "gauss_newton")

    runtime = load_pct_planner_runtime(tomogram, repo_root=tmp_path)
    start = np.asarray([-3.0, 0.0, 0.0], dtype=np.float64)
    goal = np.asarray([3.0, 0.0, 0.0], dtype=np.float64)
    result = runtime.planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))

    assert runtime.name == "rust_process"
    assert runtime.runtime_paths is not None
    assert result is not None
    assert result.shape[1] == 3
    assert runtime.planner.last_path_mode == "rust_optimized_trajectory"
    assert runtime.planner.last_optimizer_attempted is True
    assert runtime.planner.last_optimizer_accepted is True
    assert runtime.planner.last_optimizer_final_cost == 1.0
    assert runtime.planner.last_optimizer_nonlinear_optimizer == "gauss_newton"
    assert runtime.planner.last_optimizer_linear_solver == "block_tridiagonal"
    assert runtime.planner.last_optimizer_linear_solve_fallbacks == 0
    assert runtime.planner.last_optimizer_input_states > 0
    assert runtime.planner.last_optimizer_output_states == result.shape[0]
    assert runtime.planner.last_optimizer_output_states > runtime.planner.last_optimizer_input_states
    assert runtime.planner.last_optimizer_trajectory_expanded is True
    assert runtime.planner.last_optimizer_interpolation_steps == 8
    result_matrix = runtime.planner.get_result_matrix()
    layers = runtime.planner.get_layers()
    heights = runtime.planner.get_heights()
    opt_init_value = runtime.planner.get_opt_init_value()
    opt_init_layer = runtime.planner.get_opt_init_layer()
    assert result_matrix.shape == (result.shape[0], 6)
    assert layers.shape == (result.shape[0],)
    assert heights.shape == (result.shape[0],)
    assert opt_init_value.shape == (6, result.shape[0])
    assert opt_init_layer.shape == (result.shape[0],)
    assert np.allclose(result_matrix, opt_init_value.T)
    wnoj_optimizer = runtime.planner.get_trajectory_optimizer_wnoj()
    wnoj_optimizer.set_debug(True)
    assert wnoj_optimizer.get_result_matrix().shape == result_matrix.shape
    assert wnoj_optimizer.get_layers().shape == layers.shape
    assert wnoj_optimizer.get_heights().shape == heights.shape
    assert wnoj_optimizer.get_ceilings().shape == heights.shape
    assert wnoj_optimizer.get_heading_rate().shape == heights.shape
    assert wnoj_optimizer.get_opt_init_value().shape == opt_init_value.shape
    assert wnoj_optimizer.get_opt_init_layer().shape == opt_init_layer.shape
    wnoa_optimizer = runtime.planner.get_trajectory_optimizer()
    wnoa_optimizer.set_debug(True)
    gp_prior = wnoa_optimizer.gp_prior_test(
        np.asarray([0.0, 1.0, 0.0, 0.0], dtype=np.float64),
        np.asarray([2.0, 1.0, 1.0, 0.0], dtype=np.float64),
        1.0,
        3,
    )
    assert gp_prior.shape == (3, 4)
    assert np.allclose(gp_prior[0], [0.0, 1.0, 0.0, 0.0])
    assert np.allclose(gp_prior[-1], [2.0, 1.0, 1.0, 0.0])

    preview = build_preview_report(
        planner=runtime.planner,
        runtime_paths=runtime.runtime_paths,
        result=result,
        start=start,
        goal=goal,
        tomogram_path=tomogram,
        obstacle_thr=49.9,
        sample_count=3,
    )
    accessors = preview["optimizer_accessors"]
    assert preview["diagnostics"]["last_optimizer_nonlinear_optimizer"] == "gauss_newton"
    assert preview["diagnostics"]["last_optimizer_output_states"] == result.shape[0]
    assert preview["diagnostics"]["last_optimizer_trajectory_expanded"] is True
    assert preview["diagnostics"]["last_optimizer_interpolation_steps"] == 8
    assert accessors["available"] is True
    assert accessors["native_wrapper_compatible"] is True
    assert accessors["finite"] is True
    assert accessors["row_count"] == result.shape[0]
    assert accessors["state_dim"] == 6
    assert accessors["result_matrix_shape"] == [result.shape[0], 6]
    assert accessors["layers_shape"] == [result.shape[0]]
    assert accessors["heights_shape"] == [result.shape[0]]
    assert accessors["opt_init_value_shape"] == [6, result.shape[0]]
    assert accessors["opt_init_layer_shape"] == [result.shape[0]]


def test_rust_process_runtime_preserves_ros_config_and_relative_tomogram_lookup(
    tmp_path,
    monkeypatch,
):
    np = pytest.importorskip("numpy")
    fake_optimizer = tmp_path / "gpmp_fake.py"
    fake_optimizer.write_text(
        "import json, sys; req=json.loads(sys.stdin.read()); "
        "print(json.dumps({'schema':'lingtu.pct_gpmp.optimize.response.v1',"
        "'ok':True,'error':None,'states':req['states'],"
        "'layers':req.get('layers') or [0]*len(req['states']),"
        "'heights':req.get('height_hints') or [0.0]*len(req['states']),"
        "'costs':[0.0]*len(req['states']),'report':{"
        "'initial_cost':0.0,'final_cost':0.0,'iterations':0,"
        "'accepted_steps':0,'nonlinear_optimizer':"
        "(req.get('config') or {}).get('nonlinear_optimizer'),"
        "'linear_solver':(req.get('config') or {}).get('linear_solver'),"
        "'linear_solve_fallbacks':0}}))",
        encoding="utf-8",
    )
    tomo_dir = (
        tmp_path
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "vendor"
        / "pct_planner"
        / "rsc"
        / "tomogram"
    )
    tomo_dir.mkdir(parents=True)
    tomogram = tomo_dir / "relative_map.pickle"
    data = np.zeros((5, 1, 5, 5), dtype=np.float32)
    data[0, :, :, :] = 1.0
    data[3, :, :, :] = 0.0
    data[4, :, :, :] = 3.0
    with tomogram.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [0.0, 0.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
                "grid_info": {"axis_order": "row_y_col_x"},
            },
            handle,
        )

    class Cfg:
        class planner:
            use_quintic = False
            optimize_trajectory = True
            max_heading_rate = 3.5
            obstacle_thr = 12.0

        class wrapper:
            tomo_dir = "/rsc/tomogram/"
            pcd_dir = None

    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "rust_process")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_BIN_ENV, str(fake_optimizer))

    runtime = load_pct_planner_runtime(
        "relative_map",
        repo_root=tmp_path,
        obstacle_thr=33.0,
        planner_config=Cfg,
        load_tomogram_kwargs={"resolution": 0.1, "slice_dh": 0.2, "ground_h": 0.0},
    )

    assert runtime.name == "rust_process"
    assert runtime.planner.use_quintic is False
    assert runtime.planner.max_heading_rate == 3.5
    assert runtime.planner.obstacle_thr == 33.0
    assert Path(runtime.diagnostics["resolved_tomogram"]) == tomogram
    assert Path(runtime.diagnostics["prepared_tomogram"]).exists()


def _write_flat_rust_process_tomogram(tmp_path: Path):
    np = pytest.importorskip("numpy")
    tomogram = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 1, 9, 9), dtype=np.float32)
    data[0, :, :, :] = 1.0
    data[3, :, :, :] = 0.0
    data[4, :, :, :] = 3.0
    with tomogram.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 1.0,
                "center": [0.0, 0.0],
                "slice_h0": 0.0,
                "slice_dh": 1.0,
                "grid_info": {"axis_order": "row_y_col_x"},
            },
            handle,
        )
    return tomogram, np


def _plan_with_fake_rust_optimizer(
    tmp_path: Path,
    monkeypatch,
    fake_optimizer: Path,
    *,
    linear_solver: str = "block_tridiagonal",
):
    tomogram, np = _write_flat_rust_process_tomogram(tmp_path)
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "rust_process")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_BIN_ENV, str(fake_optimizer))
    monkeypatch.setenv("LINGTU_PCT_RUST_LINEAR_SOLVER", linear_solver)
    monkeypatch.setenv("LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER", "gauss_newton")

    runtime = load_pct_planner_runtime(tomogram, repo_root=tmp_path)
    start = np.asarray([-3.0, 0.0, 0.0], dtype=np.float64)
    goal = np.asarray([3.0, 0.0, 0.0], dtype=np.float64)
    result = runtime.planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))
    return runtime, result


def test_rust_process_runtime_accepts_explicit_sparse_solver_report(tmp_path, monkeypatch):
    fake_optimizer = tmp_path / "gpmp_fake_sparse.py"
    fake_optimizer.write_text(
        textwrap.dedent(
            """
            import json
            import sys

            req = json.loads(sys.stdin.read())
            states = req["states"]
            layers = req.get("layers") or [0] * len(states)
            heights = req.get("height_hints") or [0.0] * len(states)
            config = req.get("config") or {}
            print(json.dumps({
                "schema": "lingtu.pct_gpmp.optimize.response.v1",
                "ok": True,
                "error": None,
                "states": states,
                "layers": layers,
                "heights": heights,
                "costs": [1.0] * len(states),
                "report": {
                    "initial_cost": 10.0,
                    "final_cost": 1.0,
                    "iterations": 3,
                    "accepted_steps": 2,
                    "nonlinear_optimizer": config.get("nonlinear_optimizer"),
                    "linear_solver": "sparse_cholesky",
                    "linear_solve_fallbacks": 0,
                },
            }))
            """
        ).strip(),
        encoding="utf-8",
    )

    runtime, result = _plan_with_fake_rust_optimizer(
        tmp_path,
        monkeypatch,
        fake_optimizer,
        linear_solver="sparse",
    )

    assert result is not None
    assert runtime.planner.last_optimizer_attempted is True
    assert runtime.planner.last_optimizer_accepted is True
    assert runtime.planner.last_optimizer_linear_solver == "sparse_cholesky"


def test_rust_process_runtime_can_call_optimizer_via_ffi(tmp_path, monkeypatch):
    tomogram, np = _write_flat_rust_process_tomogram(tmp_path)
    fake_library = tmp_path / "lingtu_gpmp_trajectory_optimizer.dll"
    fake_library.write_bytes(b"fake")

    class FakeOptimizerLibrary:
        def __init__(self, path):
            self.path = Path(path)
            self.abi_version = 1

        def optimize_json(self, request_json: str):
            import json

            req = json.loads(request_json)
            states = req["states"]
            layers = req.get("layers") or [0] * len(states)
            heights = req.get("height_hints") or [0.0] * len(states)
            interpolation_steps = int(req.get("interpolation_steps") or 0)
            trajectory_states = []
            trajectory_layers = []
            trajectory_heights = []
            for index, state in enumerate(states):
                trajectory_states.append(state)
                trajectory_layers.append(layers[index])
                trajectory_heights.append(heights[index])
                if index + 1 < len(states):
                    nxt = states[index + 1]
                    for step in range(1, interpolation_steps + 1):
                        fraction = step / (interpolation_steps + 1)
                        trajectory_states.append(
                            [lhs + (rhs - lhs) * fraction for lhs, rhs in zip(state, nxt)]
                        )
                        trajectory_layers.append(layers[index])
                        trajectory_heights.append(
                            heights[index] + (heights[index + 1] - heights[index]) * fraction
                        )
            config = req.get("config") or {}
            return {
                "schema": "lingtu.pct_gpmp.optimize.response.v1",
                "ok": True,
                "error": None,
                "states": states,
                "layers": layers,
                "heights": heights,
                "costs": [1.0] * len(states),
                "trajectory_states": trajectory_states,
                "trajectory_layers": trajectory_layers,
                "trajectory_heights": trajectory_heights,
                "trajectory_costs": [1.0] * len(trajectory_states),
                "initial_trajectory_states": trajectory_states,
                "initial_trajectory_layers": trajectory_layers,
                "report": {
                    "initial_cost": 10.0,
                    "final_cost": 1.0,
                    "iterations": 3,
                    "accepted_steps": 2,
                    "nonlinear_optimizer": config.get("nonlinear_optimizer"),
                    "linear_solver": config.get("linear_solver"),
                    "linear_solve_fallbacks": 0,
                },
            }

    def fail_subprocess_run(*args, **kwargs):
        raise AssertionError("FFI optimizer path must not spawn gpmp_optimize")

    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "rust_process")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_CALL_ENV, "ffi")
    monkeypatch.setenv(PCT_RUST_OPTIMIZER_LIB_ENV, str(fake_library))
    monkeypatch.setenv("LINGTU_PCT_RUST_LINEAR_SOLVER", "block_tridiagonal")
    monkeypatch.setenv("LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER", "gauss_newton")
    monkeypatch.setattr("nav.services.plan.global_planner.algorithm.pct.runtime.loader.RustGpmpOptimizerLibrary", FakeOptimizerLibrary)
    monkeypatch.setattr(pct_runtime.subprocess, "run", fail_subprocess_run)

    runtime = load_pct_planner_runtime(tomogram, repo_root=tmp_path)
    start = np.asarray([-3.0, 0.0, 0.0], dtype=np.float64)
    goal = np.asarray([3.0, 0.0, 0.0], dtype=np.float64)
    result = runtime.planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))
    report = build_preview_report(
        planner=runtime.planner,
        runtime_paths=runtime.runtime_paths,
        result=result,
        start=start,
        goal=goal,
        tomogram_path=tomogram,
    )

    assert runtime.diagnostics["optimizer_call_mode"] == "ffi"
    assert runtime.diagnostics["optimizer_bin"] == ""
    assert Path(runtime.diagnostics["optimizer_library"]) == fake_library
    assert runtime.planner.last_optimizer_call_mode == "ffi"
    assert runtime.planner.last_optimizer_accepted is True
    assert runtime.planner.last_optimizer_output_states == result.shape[0]
    assert runtime.planner.last_optimizer_trajectory_expanded is True
    assert report["runtime"]["native_binary_format"] == PCT_RUST_FFI_BINARY_FORMAT
    assert report["runtime"]["rust_optimizer_call_mode"] == "ffi"
    assert report["diagnostics"]["last_optimizer_call_mode"] == "ffi"


def test_rust_process_runtime_rejects_solver_mismatch(tmp_path, monkeypatch):
    fake_optimizer = tmp_path / "gpmp_fake_solver_mismatch.py"
    fake_optimizer.write_text(
        textwrap.dedent(
            """
            import json
            import sys

            req = json.loads(sys.stdin.read())
            states = req["states"]
            layers = req.get("layers") or [0] * len(states)
            heights = req.get("height_hints") or [0.0] * len(states)
            config = req.get("config") or {}
            print(json.dumps({
                "schema": "lingtu.pct_gpmp.optimize.response.v1",
                "ok": True,
                "error": None,
                "states": states,
                "layers": layers,
                "heights": heights,
                "costs": [1.0] * len(states),
                "report": {
                    "initial_cost": 10.0,
                    "final_cost": 1.0,
                    "iterations": 3,
                    "accepted_steps": 2,
                    "nonlinear_optimizer": config.get("nonlinear_optimizer"),
                    "linear_solver": "dense",
                    "linear_solve_fallbacks": 1,
                },
            }))
            """
        ).strip(),
        encoding="utf-8",
    )

    runtime, result = _plan_with_fake_rust_optimizer(tmp_path, monkeypatch, fake_optimizer)

    assert result is not None
    assert runtime.planner.last_path_mode == "rust_astar_raw_path"
    assert runtime.planner.last_optimizer_attempted is True
    assert runtime.planner.last_optimizer_accepted is False
    assert runtime.planner.last_optimizer_reject_reason == "rust_optimizer_failed:ValueError"
    assert runtime.planner.get_result_matrix().shape == (0, 0)


def test_rust_process_runtime_rejects_nonfinite_optimizer_output(tmp_path, monkeypatch):
    fake_optimizer = tmp_path / "gpmp_fake_nonfinite.py"
    fake_optimizer.write_text(
        textwrap.dedent(
            """
            import json
            import sys

            req = json.loads(sys.stdin.read())
            states = req["states"]
            states[1][0] = float("nan")
            layers = req.get("layers") or [0] * len(states)
            heights = req.get("height_hints") or [0.0] * len(states)
            config = req.get("config") or {}
            print(json.dumps({
                "schema": "lingtu.pct_gpmp.optimize.response.v1",
                "ok": True,
                "error": None,
                "states": states,
                "layers": layers,
                "heights": heights,
                "costs": [1.0] * len(states),
                "report": {
                    "initial_cost": 10.0,
                    "final_cost": 1.0,
                    "iterations": 3,
                    "accepted_steps": 2,
                    "nonlinear_optimizer": config.get("nonlinear_optimizer"),
                    "linear_solver": config.get("linear_solver"),
                    "linear_solve_fallbacks": 0,
                },
            }))
            """
        ).strip(),
        encoding="utf-8",
    )

    runtime, result = _plan_with_fake_rust_optimizer(tmp_path, monkeypatch, fake_optimizer)

    assert result is not None
    assert runtime.planner.last_path_mode == "rust_astar_raw_path"
    assert runtime.planner.last_optimizer_attempted is True
    assert runtime.planner.last_optimizer_accepted is False
    assert runtime.planner.last_optimizer_reject_reason == "rust_optimizer_failed:ValueError"
    assert runtime.planner.get_result_matrix().shape == (0, 0)


def test_resolve_uses_runtime_native_arch_dir(tmp_path):
    repo = tmp_path
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == lib_dir
    assert paths.canonical_arch == "x86_64"


def test_resolve_supports_legacy_uppercase_pct_planner_dir(tmp_path, monkeypatch):
    repo = tmp_path
    planner_root = repo / "src/nav/services/plan/global_planner/algorithm/pct/vendor/PCT_planner/planner"
    planner_root.mkdir(parents=True)
    (planner_root / "lib").mkdir()
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")
    report = inspect_pct_runtime(repo, machine="x86_64")

    assert paths.planner_root.samefile(planner_root)
    assert paths.lib_root.samefile(planner_root / "lib")
    assert paths.lib_dir.samefile(lib_dir)
    assert Path(report["lib_dir"]).samefile(lib_dir)


def test_pct_config_defaults_keep_optimizer_enabled():
    planning_root = (
        Path(__file__).resolve().parents[2]
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "vendor"
    )
    config_path = next(
        (
            planning_root / name / "planner" / "config" / "param.py"
            for name in ("pct_planner", "PCT_planner")
            if (planning_root / name / "planner" / "config" / "param.py").is_file()
        ),
        planning_root / "pct_planner" / "planner" / "config" / "param.py",
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
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "runtime"
        / "build_legacy_native_x86_64.sh"
    )
    script = script_path.read_text(encoding="utf-8-sig")

    assert not script_path.read_bytes().startswith(b"\xef\xbb\xbf")
    assert 'BUILD_LEGACY_NATIVE="${LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE:-0}"' in script
    assert 'if [[ "${BUILD_LEGACY_NATIVE}" != "1" ]]; then' in script
    assert "-DLINGTU_PCT_BUILD_NATIVE_MODULES=ON" in script
    assert "-DLINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=ON" in script
    assert 'if [[ -f "${candidate}/lib/CMakeLists.txt" ]]' in script
    assert 'pct_planner_DIR="${candidate}"' in script


def test_resolve_prefers_runnable_native_dir_over_original_lib(tmp_path):
    repo = tmp_path
    original_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64"
    runnable_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    original_dir.mkdir(parents=True)
    runnable_dir.mkdir(parents=True)
    _touch_pct_runtime(original_dir)
    _touch_pct_runtime(runnable_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == runnable_dir


def test_resolve_rejects_original_lib_x86_64_as_runtime_candidate(tmp_path):
    repo = tmp_path
    original_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64"
    original_dir.mkdir(parents=True)
    _touch_pct_runtime(original_dir)

    with pytest.raises(FileNotFoundError) as exc:
        resolve_pct_runtime_paths(repo, machine="x86_64")

    message = str(exc.value).replace("\\", "/")
    assert "algorithm/pct/runtime" in message
    assert "planner/lib/x86_64" not in message


def test_resolve_prefers_env_lib_dir_over_runnable_native_dir(tmp_path, monkeypatch):
    repo = tmp_path
    env_dir = repo / "custom_pct_lib"
    runnable_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
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
    fallback_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    env_dir.mkdir(parents=True)
    fallback_dir.mkdir(parents=True)
    _touch_pct_runtime(env_dir)
    _touch_pct_runtime(fallback_dir)
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    report = inspect_pct_runtime(repo, machine="x86_64")

    assert report["ok"] is True
    assert Path(report["lib_dir"]) == env_dir
    assert report["missing"] == []


def test_prepare_installs_lib_namespace_for_original_wrapper(tmp_path):
    repo = tmp_path
    planner_root = repo / "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner"
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
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
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_runtime(lib_dir)

    with pytest.raises(RuntimeError, match="PCT shared library failed to load"):
        prepare_pct_runtime(repo, machine="x86_64")


def test_inspect_pct_runtime_rejects_missing_shared_libraries(tmp_path, monkeypatch):
    repo = tmp_path
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_extensions(lib_dir)
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")

    report = inspect_pct_runtime(repo, machine="x86_64")

    assert report["ok"] is False
    assert "libmetis-gtsam.so" in report["shared_missing"]
    assert "Shared library gaps" in report["error"]
    assert "pct_runtime" in " ".join(report["searched"])


def test_inspect_pct_runtime_reports_linux_native_host_boundary(tmp_path, monkeypatch):
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")
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
    lib_dir = repo / "src/nav/services/plan/global_planner/algorithm/pct/runtime/native/x86_64"
    lib_dir.mkdir(parents=True)
    for name in ("a_star", "ele_planner", "traj_opt"):
        (lib_dir / f"{name}.cpython-310-x86_64-linux-gnu.so").write_bytes(b"")
    _touch_pct_shared_libs(lib_dir)
    monkeypatch.setenv(PCT_PLANNER_RUNTIME_ENV, "native")
    monkeypatch.setenv(PCT_LEGACY_NATIVE_ALLOW_ENV, "1")
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
    assert "build_legacy_native_x86_64.sh" in report["recommended_build_command"]
    assert "LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1" in report["recommended_build_command"]


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
