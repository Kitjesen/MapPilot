"""Static regression checks for the original PCT planner wrapper."""

import importlib.util
from pathlib import Path
import sys
import types

import numpy as np


NAV_ROOT = Path(__file__).resolve().parents[2]
PCT_VENDOR_ROOT = (
    NAV_ROOT / "services" / "plan" / "global_planner" / "algorithm" / "pct" / "vendor"
)
PCT_PLANNER_ROOT = PCT_VENDOR_ROOT / "pct_planner"


def _pct_native_source_root() -> Path:
    for name in ("pct_planner", "PCT_planner"):
        candidate = PCT_VENDOR_ROOT / name
        if (
            candidate
            / "planner"
            / "lib"
            / "src"
            / "trajectory_optimization"
            / "gpmp_optimizer"
        ).is_dir():
            return candidate
    return PCT_PLANNER_ROOT


def test_pct_planner_wrapper_diagnostics_are_broken_pipe_safe():
    wrapper = PCT_PLANNER_ROOT / "planner" / "scripts" / "planner_wrapper.py"
    text = wrapper.read_text(encoding="utf-8")
    assert "def _safe_print(" in text
    assert "except BrokenPipeError" in text

    unsafe_lines = [
        line.strip()
        for line in text.splitlines()
        if "print(" in line and "_safe_print" not in line
        and line.strip() != "print(*args, **kwargs)"
    ]
    assert unsafe_lines == []


def test_pct_planner_wrapper_falls_back_to_native_raw_path_for_empty_optimizer_result():
    wrapper = PCT_PLANNER_ROOT / "planner" / "scripts" / "planner_wrapper.py"
    text = wrapper.read_text(encoding="utf-8")

    assert "optimizer returned empty trajectory" in text
    assert "return raw_world" in text
    assert "traj_raw.size == 0" in text


def test_pct_planner_wrapper_checks_hard_obstacle_neighbor_cells():
    wrapper = PCT_PLANNER_ROOT / "planner" / "scripts" / "planner_wrapper.py"
    text = wrapper.read_text(encoding="utf-8")

    assert "row - 1" in text
    assert "row + 1" in text
    assert "col - 1" in text
    assert "col + 1" in text
    assert "half-cell" in text


def test_pct_planner_wrapper_records_actual_optimizer_path_mode():
    wrapper = PCT_PLANNER_ROOT / "planner" / "scripts" / "planner_wrapper.py"
    text = wrapper.read_text(encoding="utf-8")

    assert "last_path_mode" in text
    assert "last_optimizer_attempted" in text
    assert "last_optimizer_accepted" in text
    assert "last_optimizer_reject_reason" in text
    assert "optimized_trajectory_hard_obstacle" in text
    assert 'self.last_path_mode = "native_astar_raw_path"' in text
    assert 'self.last_path_mode = "optimized_trajectory"' in text


def test_pct_planner_wrapper_rejects_when_optimized_and_raw_paths_are_blocked(monkeypatch):
    wrapper_path = PCT_PLANNER_ROOT / "planner" / "scripts" / "planner_wrapper.py"
    fake_lib = types.ModuleType("lib")
    fake_lib.a_star = types.SimpleNamespace(Astar=object)
    fake_lib.ele_planner = types.SimpleNamespace(OfflineElePlanner=object)
    fake_lib.traj_opt = types.SimpleNamespace(GPMPOptimizer=object)
    monkeypatch.setitem(sys.modules, "lib", fake_lib)

    spec = importlib.util.spec_from_file_location("_pct_wrapper_safety_test", wrapper_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    raw_native_path = np.array(
        [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    raw_world = np.array(
        [[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    optimized_world = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0]],
        dtype=np.float64,
    )

    class FakePathFinder:
        def get_result_matrix(self):
            return raw_native_path

    class FakeOptimizer:
        def get_opt_init_value(self):
            return np.zeros((2, 2), dtype=np.float64)

        def get_opt_init_layer(self):
            return np.zeros(2, dtype=np.float64)

        def get_result_matrix(self):
            return np.zeros((2, 2), dtype=np.float64)

        def get_layers(self):
            return np.zeros(2, dtype=np.float64)

        def get_heights(self):
            return np.zeros(2, dtype=np.float64)

    class FakeNativePlanner:
        def plan(self, _start, _goal, _optimize):
            return None

        def get_path_finder(self):
            return FakePathFinder()

        def get_trajectory_optimizer_wnoj(self):
            return FakeOptimizer()

    planner = module.TomogramPlanner.__new__(module.TomogramPlanner)
    planner.optimize_trajectory = True
    planner.use_quintic = True
    planner.start_idx = np.zeros(3, dtype=np.float64)
    planner.end_idx = np.zeros(3, dtype=np.float64)
    planner.obstacle_thr = 50.0
    planner.map_dim = [8, 8]
    planner.center = np.zeros(2, dtype=np.float64)
    planner.planner = FakeNativePlanner()
    planner.pos2idx = lambda _pos: np.zeros(2, dtype=np.float64)
    planner.pos2slice = lambda _height: 0.0
    planner._raw_path_to_world = lambda _path: raw_world
    planner._optimized_traj_to_world = lambda _xy, _height: optimized_world
    planner._hard_obstacle_sample_count = (
        lambda path: 3 if path is optimized_world else 2
    )

    result = planner.plan(
        np.array([0.0, 0.0], dtype=np.float64),
        np.array([1.0, 1.0], dtype=np.float64),
    )

    assert result is None
    assert planner.last_path_mode == "collision_rejected"
    assert planner.last_optimizer_accepted is False
    assert planner.last_optimizer_reject_reason == "all_candidate_paths_hard_obstacle"
    assert planner.last_optimizer_blocked_sample_count == 3
    assert planner.last_raw_path_blocked_sample_count == 2


def test_pct_gpmp_factors_guard_each_optional_jacobian_separately():
    factor_root = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
    )
    offenders = []
    for path in factor_root.rglob("*.cc"):
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if stripped.startswith("//"):
                continue
            if "if (H1 || H2)" in stripped or "if (H1||H2)" in stripped:
                offenders.append(f"{path.relative_to(PCT_PLANNER_ROOT)}:{lineno}")

    assert offenders == []


def test_pct_height_smoother_bypasses_osqp_for_short_or_invalid_inputs():
    smoother = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "height_smoother"
        / "height_smoother.cc"
    )
    text = smoother.read_text(encoding="utf-8")

    assert "N < 3" in text
    assert "upper_bound.size() != coarse_height.size()" in text
    assert "return coarse_height;" in text


def test_pct_wnoj_optimizer_bypasses_height_smoother_like_wnoa():
    optimizer = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
        / "gpmp_optimizer.cc"
    )
    text = optimizer.read_text(encoding="utf-8")
    active_calls = [
        line.strip()
        for line in text.splitlines()
        if "height_smoother_.Smooth" in line and not line.strip().startswith("//")
    ]

    assert active_calls == []
    assert "WNOA already" in text


def test_pct_gpmp_optimizers_do_not_rely_on_release_assert_for_short_paths():
    optimizer_root = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
    )
    for name in ("gpmp_optimizer.cc", "gpmp_optimizer_wnoa.cc"):
        text = (optimizer_root / name).read_text(encoding="utf-8")
        assert "if (path.size() < 2)" in text
        assert "return false;" in text
        assert "assert(path.size() > 1)" not in text
        assert "std::max(1, sample_interval_)" in text
        assert "clear_outputs();" in text


def test_pct_dense_elevation_map_clamps_layer_and_nonfinite_indices():
    dense_map = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "map_manager"
        / "dense_elevation_map.h"
    )
    text = dense_map.read_text(encoding="utf-8")

    assert "int inline layer_safe" in text
    assert "std::isfinite(coord)" in text
    assert "layer_safe(layer)" in text
    assert "bool valid_" in text

    dense_map_impl = dense_map.with_suffix(".cc")
    impl_text = dense_map_impl.read_text(encoding="utf-8")
    assert "num_layers <= 0" in impl_text
    assert "cost_map.rows() % num_layers != 0" in impl_text
    assert "if (!valid_)" in impl_text


def test_pct_wnoj_debug_logging_does_not_read_incremented_layer_index():
    optimizer = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
        / "gpmp_optimizer.cc"
    )
    lines = optimizer.read_text(encoding="utf-8").splitlines()
    offenders = []
    for lineno, line in enumerate(lines, 1):
        if line.strip() != "col_index++;":
            continue
        lookahead = lines[lineno : lineno + 8]
        if any("opt_init_layer_(col_index)" in later for later in lookahead):
            offenders.append(lineno)

    assert offenders == []


def test_pct_gpmp_optimizers_validate_interpolated_trajectory_shape():
    optimizer_root = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
    )
    required = {
        "gpmp_optimizer.cc": "trajectory_.cols() < 4",
        "gpmp_optimizer_wnoa.cc": "trajectory_.cols() < 3",
    }
    for name, col_guard in required.items():
        text = (optimizer_root / name).read_text(encoding="utf-8")
        assert "const int expected_points = N + (N - 1) * interpolate_num_" in text
        assert "trajectory_.rows() != expected_points" in text
        assert col_guard in text
        assert "return false;" in text


def test_pct_wnoa_debug_printf_has_matching_interpolation_arguments():
    optimizer = (
        _pct_native_source_root()
        / "planner"
        / "lib"
        / "src"
        / "trajectory_optimization"
        / "gpmp_optimizer"
        / "gpmp_optimizer_wnoa.cc"
    )
    text = optimizer.read_text(encoding="utf-8")

    assert "layer: %d, %f, height" not in text
    assert "inter_layer: %d" in text
    assert "static_cast<int>(opt_init_layer_.size() - 1)" in text
