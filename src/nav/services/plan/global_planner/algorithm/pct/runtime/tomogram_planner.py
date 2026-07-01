from __future__ import annotations

from .common import *  # noqa: F401,F403
from .ffi import RustGpmpOptimizerLibrary
from .rust_gpmp import RustProcessGPMPOptimizer, RustProcessGPMPOptimizerWnoa

class RustProcessTomogramPlanner:
    """Cross-platform PCT replacement using Python A* plus Rust GPMP smoothing."""

    def __init__(
        self,
        cfg: Any,
        optimizer_bin: str | os.PathLike[str] | None,
        *,
        optimizer_library: RustGpmpOptimizerLibrary | None = None,
        optimizer_call_mode: str = "process",
    ):
        self.cfg = cfg
        planner_cfg = getattr(cfg, "planner", None)
        self.use_quintic = getattr(planner_cfg, "use_quintic", True)
        self.optimize_trajectory = self._env_bool(
            "LINGTU_PCT_OPTIMIZE_TRAJECTORY",
            getattr(planner_cfg, "optimize_trajectory", True),
        )
        self.max_heading_rate = float(getattr(planner_cfg, "max_heading_rate", 10))
        self.obstacle_thr = float(getattr(planner_cfg, "obstacle_thr", 49.9))
        self.optimizer_bin = Path(optimizer_bin) if optimizer_bin is not None else None
        self.optimizer_library = optimizer_library
        self.optimizer_call_mode = optimizer_call_mode
        self.optimizer_timeout_s = float(
            os.environ.get("LINGTU_PCT_RUST_OPTIMIZER_TIMEOUT_S", "10")
        )
        self.local_map_margin = int(os.environ.get("LINGTU_PCT_RUST_LOCAL_MARGIN", "12"))
        self.max_optimizer_states = int(os.environ.get("LINGTU_PCT_RUST_MAX_STATES", "48"))
        self.optimizer_linear_solver = os.environ.get(
            "LINGTU_PCT_RUST_LINEAR_SOLVER",
            "auto",
        ).strip().lower() or "auto"
        self.optimizer_nonlinear_optimizer = os.environ.get(
            "LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER",
            "levenberg_marquardt",
        ).strip().lower() or "levenberg_marquardt"

        self.resolution: float | None = None
        self.center: Any = None
        self.n_slice: int | None = None
        self.slice_h0: float | None = None
        self.slice_dh: float | None = None
        self.map_dim: list[int] = []
        self.offset: Any = None
        self.layers_t: Any = None
        self.layers_g: Any = None
        self.layers_c: Any = None

        self.last_path_mode = ""
        self.last_optimizer_enabled = bool(self.optimize_trajectory)
        self.last_optimizer_attempted = False
        self.last_optimizer_accepted = None
        self.last_optimizer_reject_reason = ""
        self.last_optimizer_blocked_sample_count = 0
        self.last_raw_path_blocked_sample_count = 0
        self.last_optimizer_elapsed_ms = 0.0
        self.last_optimizer_initial_cost = None
        self.last_optimizer_final_cost = None
        self.last_optimizer_iterations = 0
        self.last_optimizer_accepted_steps = 0
        self.last_optimizer_nonlinear_optimizer = ""
        self.last_optimizer_linear_solver = ""
        self.last_optimizer_linear_solve_fallbacks = 0
        self.last_optimizer_input_states = 0
        self.last_optimizer_output_states = 0
        self.last_optimizer_trajectory_expanded = False
        self.last_optimizer_interpolation_steps = 0
        self.last_optimizer_call_mode = ""
        self.last_raw_path_count = 0
        self._trajectory_optimizer_wnoj = RustProcessGPMPOptimizer(self)
        self._trajectory_optimizer_wnoa = RustProcessGPMPOptimizerWnoa(self)
        self._last_returned_path = None
        self._last_result_matrix = None
        self._last_layers = None
        self._last_heights = None
        self._last_ceilings = None
        self._last_heading_rate = None
        self._last_opt_init_value = None
        self._last_opt_init_layer = None

    @staticmethod
    def _env_bool(name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return bool(default)
        text = value.strip().lower()
        if text == "":
            return bool(default)
        return text not in {"0", "false", "no", "off"}

    def _reset_last_plan_metadata(self) -> None:
        self.last_path_mode = ""
        self.last_optimizer_enabled = bool(self.optimize_trajectory)
        self.last_optimizer_attempted = False
        self.last_optimizer_accepted = None
        self.last_optimizer_reject_reason = ""
        self.last_optimizer_blocked_sample_count = 0
        self.last_raw_path_blocked_sample_count = 0
        self.last_optimizer_elapsed_ms = 0.0
        self.last_optimizer_initial_cost = None
        self.last_optimizer_final_cost = None
        self.last_optimizer_iterations = 0
        self.last_optimizer_accepted_steps = 0
        self.last_optimizer_nonlinear_optimizer = ""
        self.last_optimizer_linear_solver = ""
        self.last_optimizer_linear_solve_fallbacks = 0
        self.last_optimizer_input_states = 0
        self.last_optimizer_output_states = 0
        self.last_optimizer_trajectory_expanded = False
        self.last_optimizer_interpolation_steps = 0
        self.last_optimizer_call_mode = ""
        self.last_raw_path_count = 0
        self._last_returned_path = None
        self._last_result_matrix = None
        self._last_layers = None
        self._last_heights = None
        self._last_ceilings = None
        self._last_heading_rate = None
        self._last_opt_init_value = None
        self._last_opt_init_layer = None

    @staticmethod
    def _empty_matrix():
        import numpy as np

        return np.empty((0, 0), dtype=np.float64)

    @staticmethod
    def _empty_vector():
        import numpy as np

        return np.empty((0,), dtype=np.float64)

    def get_result_matrix(self):
        import numpy as np

        if self._last_result_matrix is None:
            return self._empty_matrix()
        return np.asarray(self._last_result_matrix, dtype=np.float64).copy()

    def get_layers(self):
        import numpy as np

        if self._last_layers is None:
            return self._empty_vector()
        return np.asarray(self._last_layers, dtype=np.float64).copy()

    def get_heights(self):
        import numpy as np

        if self._last_heights is None:
            return self._empty_vector()
        return np.asarray(self._last_heights, dtype=np.float64).copy()

    def get_ceilings(self):
        import numpy as np

        if self._last_ceilings is None:
            return self._empty_vector()
        return np.asarray(self._last_ceilings, dtype=np.float64).copy()

    def get_opt_init_value(self):
        import numpy as np

        if self._last_opt_init_value is None:
            return self._empty_matrix()
        return np.asarray(self._last_opt_init_value, dtype=np.float64).copy()

    def get_opt_init_layer(self):
        import numpy as np

        if self._last_opt_init_layer is None:
            return self._empty_vector()
        return np.asarray(self._last_opt_init_layer, dtype=np.float64).copy()

    def get_heading_rate(self):
        import numpy as np

        if self._last_heading_rate is None:
            return self._empty_vector()
        return np.asarray(self._last_heading_rate, dtype=np.float64).copy()

    def get_trajectory_optimizer_wnoj(self) -> RustProcessGPMPOptimizer:
        return self._trajectory_optimizer_wnoj

    def get_trajectory_optimizer(self) -> RustProcessGPMPOptimizerWnoa:
        return self._trajectory_optimizer_wnoa

    def loadTomogram(self, tomo_file: str, resolution=None, slice_dh=None, ground_h=None) -> None:
        del resolution, slice_dh, ground_h
        with open(tomo_file, "rb") as handle:
            data_dict = pickle.load(handle)  # noqa: S301  # trusted local tomogram
        self._initFromDataDict(data_dict)

    def _initFromDataDict(self, data_dict: dict[str, Any]) -> None:
        import numpy as np

        tomogram = np.asarray(data_dict["data"], dtype=np.float64)
        if tomogram.ndim != 4 or tomogram.shape[0] < 5:
            raise ValueError(f"expected tomogram data shape (5, slices, rows, cols), got {tomogram.shape}")

        self.resolution = float(data_dict["resolution"])
        self.center = np.asarray(data_dict["center"], dtype=np.float64)
        self.n_slice = int(tomogram.shape[1])
        self.slice_h0 = float(data_dict["slice_h0"])
        self.slice_dh = float(data_dict["slice_dh"])
        self.map_dim = [int(tomogram.shape[3]), int(tomogram.shape[2])]
        self.offset = np.array([self.map_dim[0] // 2, self.map_dim[1] // 2], dtype=np.int32)

        self.layers_t = np.nan_to_num(tomogram[0], nan=float("inf"), posinf=float("inf"))
        self.layers_g = tomogram[3].copy()
        self.layers_c = tomogram[4].copy()
        for layer in range(self.n_slice):
            nominal = self.slice_h0 + layer * self.slice_dh
            ground = self.layers_g[layer]
            ceiling = self.layers_c[layer]
            ground[~np.isfinite(ground)] = nominal
            ceiling[~np.isfinite(ceiling)] = nominal + 3.0

    def pos2idx(self, pos: Any) -> Any:
        import numpy as np

        if self.center is None or self.resolution is None or self.offset is None:
            raise RuntimeError("tomogram not loaded")
        idx = np.round((np.asarray(pos, dtype=np.float64) - self.center) / self.resolution).astype(np.int32)
        idx = idx + self.offset
        if self.map_dim:
            idx[0] = np.clip(idx[0], 0, self.map_dim[0] - 1)
            idx[1] = np.clip(idx[1], 0, self.map_dim[1] - 1)
        return idx.astype(np.float32)

    def pos2slice(self, z: float) -> float:
        if not self.slice_dh:
            return 0.0
        layer = round((float(z) - float(self.slice_h0 or 0.0)) / float(self.slice_dh))
        if self.n_slice is not None:
            layer = max(0, min(int(layer), self.n_slice - 1))
        return float(layer)

    def get_surface_height(self, pos: Any) -> float:
        import numpy as np

        if self.layers_g is None:
            return float(self.slice_h0 or 0.0)
        idx = self.pos2idx(pos)
        col = int(np.clip(idx[0], 0, self.layers_g.shape[2] - 1))
        row = int(np.clip(idx[1], 0, self.layers_g.shape[1] - 1))
        heights = self.layers_g[:, row, col]
        valid = heights[np.isfinite(heights)]
        return float(valid[0]) if len(valid) else float(self.slice_h0 or 0.0)

    def plan(self, start_pos: Any, end_pos: Any, start_height=0, end_height=0):
        import numpy as np

        self._reset_last_plan_metadata()
        if not (np.all(np.isfinite(start_pos)) and np.all(np.isfinite(end_pos))):
            return None
        if not (np.isfinite(start_height) and np.isfinite(end_height)):
            return None
        raw_cells = self._plan_raw_cells(start_pos, end_pos, float(start_height), float(end_height))
        if not raw_cells:
            self.last_path_mode = "rust_astar_no_path"
            return None
        self.last_raw_path_count = len(raw_cells)
        raw_world = self._cells_to_world(raw_cells)
        if not self.optimize_trajectory or len(raw_cells) < 3:
            self.last_path_mode = "rust_astar_raw_path"
            return self._return_path(raw_world)

        optimizer_cells = self._subsample_cells(raw_cells, self.max_optimizer_states)
        if len(optimizer_cells) < 3:
            self.last_path_mode = "rust_astar_raw_path"
            return self._return_path(raw_world)
        self.last_optimizer_input_states = len(optimizer_cells)

        self.last_optimizer_attempted = True
        try:
            request, crop_origin = self._build_optimizer_request(optimizer_cells)
            started = time.perf_counter()
            response = self._invoke_optimizer(request)
            self.last_optimizer_elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._validate_optimizer_response(response, request, expected_state_dim=6)
            optimized = self._optimizer_response_to_world(response, crop_origin)
            self._record_optimizer_report(response)
            self._record_optimizer_result(response, request, crop_origin)
        except Exception as exc:  # noqa: BLE001 - planner must safely fall back to raw path.
            self.last_path_mode = "rust_astar_raw_path"
            self.last_optimizer_accepted = False
            self.last_optimizer_reject_reason = f"rust_optimizer_failed:{type(exc).__name__}"
            return self._return_path(raw_world)

        blocked = self._hard_obstacle_sample_count(optimized)
        if blocked:
            self.last_optimizer_blocked_sample_count = int(blocked)
            self.last_raw_path_blocked_sample_count = int(self._hard_obstacle_sample_count(raw_world))
            if self.last_raw_path_blocked_sample_count == 0:
                self.last_path_mode = "rust_astar_raw_path"
                self.last_optimizer_accepted = False
                self.last_optimizer_reject_reason = "optimized_trajectory_hard_obstacle"
                return self._return_path(raw_world)
            self.last_path_mode = "rust_optimized_trajectory"
            self.last_optimizer_accepted = False
            self.last_optimizer_reject_reason = "optimized_trajectory_hard_obstacle"
            return self._return_path(optimized)

        self.last_path_mode = "rust_optimized_trajectory"
        self.last_optimizer_accepted = True
        return self._return_path(optimized)

    def _invoke_optimizer(self, request: dict[str, Any]) -> dict[str, Any]:
        request_json = json.dumps(request, separators=(",", ":"))
        if self.optimizer_library is not None:
            self.last_optimizer_call_mode = "ffi"
            return self.optimizer_library.optimize_json(request_json)

        if self.optimizer_bin is None:
            raise RuntimeError("Rust GPMP optimizer process path is not configured")
        self.last_optimizer_call_mode = "process"
        command = (
            [sys.executable, str(self.optimizer_bin)]
            if self.optimizer_bin.suffix.lower() == ".py"
            else [str(self.optimizer_bin)]
        )
        proc = subprocess.run(
            command,
            input=request_json,
            text=True,
            capture_output=True,
            timeout=self.optimizer_timeout_s,
            check=False,
        )
        if proc.returncode != 0 and not proc.stdout.strip():
            raise RuntimeError((proc.stderr or "").strip() or f"optimizer exited {proc.returncode}")
        return json.loads(proc.stdout)

    @staticmethod
    def _normalized_optimizer_name(value: Any) -> str:
        return str(value or "").strip().lower()

    @staticmethod
    def _finite_vector(value: Any, *, field: str, length: int):
        import numpy as np

        try:
            arr = np.asarray(value, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"optimizer response field {field!r} is not numeric") from exc
        if arr.shape != (length,):
            raise ValueError(
                f"optimizer response field {field!r} has shape {arr.shape}, "
                f"expected ({length},)"
            )
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"optimizer response field {field!r} contains non-finite values")
        return arr

    @staticmethod
    def _finite_matrix(value: Any, *, field: str, shape: tuple[int, int]):
        import numpy as np

        try:
            arr = np.asarray(value, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"optimizer response field {field!r} is not numeric") from exc
        if arr.shape != shape:
            raise ValueError(
                f"optimizer response field {field!r} has shape {arr.shape}, "
                f"expected {shape}"
            )
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"optimizer response field {field!r} contains non-finite values")
        return arr

    @staticmethod
    def _response_field(response: dict[str, Any], primary: str, fallback: str) -> Any:
        value = response.get(primary)
        if value is None:
            return response.get(fallback)
        try:
            if len(value) == 0:
                return response.get(fallback)
        except TypeError:
            pass
        return value

    def _validate_optimizer_response(
        self,
        response: dict[str, Any],
        request: dict[str, Any],
        *,
        expected_state_dim: int,
    ) -> None:
        import numpy as np

        if not isinstance(response, dict):
            raise ValueError("optimizer response must be a JSON object")
        if request.get("schema") != PCT_GPMP_OPTIMIZER_REQUEST_SCHEMA:
            raise ValueError(
                "optimizer request schema mismatch: "
                f"{request.get('schema')!r}"
            )
        if response.get("schema") != PCT_GPMP_OPTIMIZER_RESPONSE_SCHEMA:
            raise ValueError(
                "optimizer response schema mismatch: "
                f"{response.get('schema')!r}"
            )
        if response.get("ok") is not True:
            raise RuntimeError(str(response.get("error") or "optimizer returned ok=false"))
        if response.get("error") not in (None, ""):
            raise ValueError(f"optimizer response has ok=true with error={response.get('error')!r}")

        request_states = self._finite_matrix(
            request.get("states"),
            field="request.states",
            shape=(len(request.get("states") or []), expected_state_dim),
        )
        state_count = int(request_states.shape[0])
        if state_count <= 0:
            raise ValueError("optimizer request contains no states")
        self._finite_matrix(
            response.get("states"),
            field="states",
            shape=(state_count, expected_state_dim),
        )
        self._finite_vector(response.get("layers"), field="layers", length=state_count)
        self._finite_vector(response.get("heights"), field="heights", length=state_count)
        self._finite_vector(response.get("costs"), field="costs", length=state_count)

        interpolation_steps = int(
            request.get("interpolation_steps")
            if request.get("interpolation_steps") is not None
            else 2
        )
        if interpolation_steps < 0:
            raise ValueError("optimizer request interpolation_steps must be non-negative")
        expected_trajectory_count = state_count + max(0, state_count - 1) * interpolation_steps
        trajectory_states = response.get("trajectory_states")
        if trajectory_states is not None:
            self._finite_matrix(
                trajectory_states,
                field="trajectory_states",
                shape=(expected_trajectory_count, expected_state_dim),
            )
            self._finite_vector(
                response.get("trajectory_layers"),
                field="trajectory_layers",
                length=expected_trajectory_count,
            )
            self._finite_vector(
                response.get("trajectory_heights"),
                field="trajectory_heights",
                length=expected_trajectory_count,
            )
            self._finite_vector(
                response.get("trajectory_costs"),
                field="trajectory_costs",
                length=expected_trajectory_count,
            )
            self._finite_matrix(
                response.get("initial_trajectory_states"),
                field="initial_trajectory_states",
                shape=(expected_trajectory_count, expected_state_dim),
            )
            self._finite_vector(
                response.get("initial_trajectory_layers"),
                field="initial_trajectory_layers",
                length=expected_trajectory_count,
            )

        report = response.get("report")
        if not isinstance(report, dict):
            raise ValueError("optimizer response is missing report object")

        initial_cost = float(report.get("initial_cost"))
        final_cost = float(report.get("final_cost"))
        if not np.isfinite(initial_cost) or not np.isfinite(final_cost):
            raise ValueError("optimizer report cost fields must be finite")
        for field in ("iterations", "accepted_steps", "linear_solve_fallbacks"):
            value = int(report.get(field))
            if value < 0:
                raise ValueError(f"optimizer report field {field!r} must be non-negative")

        config = request.get("config") or {}
        requested_nonlinear = self._normalized_optimizer_name(
            config.get("nonlinear_optimizer") or self.optimizer_nonlinear_optimizer
        )
        reported_nonlinear = self._normalized_optimizer_name(report.get("nonlinear_optimizer"))
        if requested_nonlinear not in PCT_GPMP_NONLINEAR_OPTIMIZERS:
            raise ValueError(f"unsupported requested nonlinear optimizer: {requested_nonlinear!r}")
        if reported_nonlinear != requested_nonlinear:
            raise ValueError(
                "optimizer report nonlinear optimizer mismatch: "
                f"requested {requested_nonlinear!r}, got {reported_nonlinear!r}"
            )

        requested_solver = self._normalized_optimizer_name(
            config.get("linear_solver") or self.optimizer_linear_solver
        )
        reported_solver = self._normalized_optimizer_name(report.get("linear_solver"))
        if requested_solver not in PCT_GPMP_REQUESTED_LINEAR_SOLVERS:
            raise ValueError(f"unsupported requested linear solver: {requested_solver!r}")
        if reported_solver not in PCT_GPMP_REPORTED_LINEAR_SOLVERS:
            raise ValueError(f"optimizer report linear solver is invalid: {reported_solver!r}")
        fallbacks = int(report.get("linear_solve_fallbacks"))
        if requested_solver == "dense":
            expected_solvers = {"dense"}
        elif requested_solver == "sparse":
            expected_solvers = {"sparse_cholesky"}
        elif requested_solver == "block_tridiagonal":
            expected_solvers = {"block_tridiagonal"}
        else:
            expected_solvers = PCT_GPMP_REPORTED_LINEAR_SOLVERS
        if reported_solver not in expected_solvers:
            raise ValueError(
                "optimizer report linear solver mismatch: "
                f"requested {requested_solver!r}, got {reported_solver!r}"
            )
        if requested_solver in {"sparse", "block_tridiagonal"} and fallbacks:
            raise ValueError(f"optimizer fell back from {requested_solver} linear solve")

    def _return_path(self, path: Any):
        import numpy as np

        self._last_returned_path = np.asarray(path, dtype=np.float64).copy()
        return path

    def _record_optimizer_report(self, response: dict[str, Any]) -> None:
        report = response.get("report") or {}
        self.last_optimizer_initial_cost = report.get("initial_cost")
        self.last_optimizer_final_cost = report.get("final_cost")
        self.last_optimizer_iterations = int(report.get("iterations") or 0)
        self.last_optimizer_accepted_steps = int(report.get("accepted_steps") or 0)
        self.last_optimizer_nonlinear_optimizer = str(report.get("nonlinear_optimizer") or "")
        self.last_optimizer_linear_solver = str(report.get("linear_solver") or "")
        self.last_optimizer_linear_solve_fallbacks = int(report.get("linear_solve_fallbacks") or 0)

    def _record_optimizer_result(
        self,
        response: dict[str, Any],
        request: dict[str, Any],
        crop_origin: tuple[int, int],
    ) -> None:
        import numpy as np

        col0, row0 = crop_origin
        states = np.asarray(
            self._response_field(response, "trajectory_states", "states"),
            dtype=np.float64,
        )
        layers = np.asarray(
            self._response_field(response, "trajectory_layers", "layers"),
            dtype=np.float64,
        ).reshape(-1)
        heights = np.asarray(
            self._response_field(response, "trajectory_heights", "heights"),
            dtype=np.float64,
        ).reshape(-1)
        if states.ndim != 2 or states.shape[0] != layers.shape[0] or states.shape[0] != heights.shape[0]:
            raise ValueError("optimizer returned invalid result accessor shapes")

        full_states = states.copy()
        full_states[:, 0] += float(col0)
        y_index = 3 if full_states.shape[1] >= 6 else 2
        if full_states.shape[1] <= y_index:
            raise ValueError("optimizer returned state matrix without y coordinate")
        full_states[:, y_index] += float(row0)

        init_states_value = response.get("initial_trajectory_states")
        if init_states_value is None:
            init_states_value = request.get("states")
        init_states = np.asarray(init_states_value, dtype=np.float64)
        if init_states.ndim != 2 or init_states.shape[0] != states.shape[0]:
            raise ValueError("optimizer request has invalid initial state shape")
        full_init_states = init_states.copy()
        full_init_states[:, 0] += float(col0)
        if full_init_states.shape[1] <= y_index:
            raise ValueError("optimizer request state matrix without y coordinate")
        full_init_states[:, y_index] += float(row0)

        self._last_result_matrix = full_states
        self._last_layers = layers
        self._last_heights = heights
        self._last_ceilings = self._ceilings_for_result(full_states, layers, heights)
        self._last_heading_rate = self._heading_rate_for_result(full_states)
        self._last_opt_init_value = full_init_states.T.copy()
        init_layers_value = response.get("initial_trajectory_layers")
        if init_layers_value is None:
            init_layers_value = request.get("layers")
        self._last_opt_init_layer = np.asarray(init_layers_value, dtype=np.float64).reshape(-1).copy()
        self.last_optimizer_output_states = int(states.shape[0])
        self.last_optimizer_trajectory_expanded = int(states.shape[0]) > int(self.last_optimizer_input_states)

    def _ceilings_for_result(self, states: Any, layers: Any, heights: Any):
        import numpy as np

        states_arr = np.asarray(states, dtype=np.float64)
        layer_arr = np.asarray(layers, dtype=np.float64).reshape(-1)
        height_arr = np.asarray(heights, dtype=np.float64).reshape(-1)
        ceilings = np.empty(layer_arr.shape[0], dtype=np.float64)
        if self.layers_c is None or states_arr.ndim != 2 or states_arr.shape[0] != layer_arr.shape[0]:
            ceilings[:] = height_arr + 3.0
            return ceilings
        y_index = 3 if states_arr.shape[1] >= 6 else 2
        for index in range(layer_arr.shape[0]):
            try:
                layer = int(np.clip(round(float(layer_arr[index])), 0, self.layers_c.shape[0] - 1))
                col = int(np.clip(round(float(states_arr[index, 0])), 0, self.layers_c.shape[2] - 1))
                row = int(np.clip(round(float(states_arr[index, y_index])), 0, self.layers_c.shape[1] - 1))
                ceiling = float(self.layers_c[layer, row, col])
            except Exception:
                ceiling = float("nan")
            if not np.isfinite(ceiling):
                ceiling = float(height_arr[index]) + 3.0
            ceilings[index] = ceiling
        return ceilings

    @staticmethod
    def _heading_rate_for_result(states: Any):
        import numpy as np

        states_arr = np.asarray(states, dtype=np.float64)
        if states_arr.ndim != 2 or states_arr.shape[1] < 6:
            return np.zeros((states_arr.shape[0] if states_arr.ndim == 2 else 0,), dtype=np.float64)
        dx = states_arr[:, 1]
        ddx = states_arr[:, 2]
        dy = states_arr[:, 4]
        ddy = states_arr[:, 5]
        return (ddy * dx - dy * ddx) / (dx * dx + dy * dy + 1e-6)

    @staticmethod
    def _subsample_cells(
        cells: list[tuple[int, int, int]],
        max_states: int,
    ) -> list[tuple[int, int, int]]:
        if max_states <= 0 or len(cells) <= max_states:
            return list(cells)
        if max_states <= 2:
            return [cells[0], cells[-1]]
        selected: list[tuple[int, int, int]] = []
        last_index = len(cells) - 1
        for slot in range(max_states):
            index = int(round(slot * last_index / (max_states - 1)))
            cell = cells[index]
            if not selected or selected[-1] != cell:
                selected.append(cell)
        if selected[-1] != cells[-1]:
            selected.append(cells[-1])
        return selected

    def _world_to_cell(self, pos: Any, height: float) -> tuple[int, int, int]:
        import numpy as np

        idx = self.pos2idx(pos)
        col = int(np.clip(round(float(idx[0])), 0, self.map_dim[0] - 1))
        row = int(np.clip(round(float(idx[1])), 0, self.map_dim[1] - 1))
        layer = int(np.clip(round(self.pos2slice(height)), 0, int(self.n_slice or 1) - 1))
        return layer, row, col

    def _is_free_cell(self, layer: int, row: int, col: int) -> bool:
        import numpy as np

        if self.layers_t is None or self.n_slice is None:
            return False
        if layer < 0 or row < 0 or col < 0:
            return False
        if layer >= self.n_slice or row >= self.layers_t.shape[1] or col >= self.layers_t.shape[2]:
            return False
        cost = float(self.layers_t[layer, row, col])
        return bool(np.isfinite(cost) and cost < self.obstacle_thr)

    def _nearest_free_cell(self, cell: tuple[int, int, int], max_radius: int = 8) -> tuple[int, int, int] | None:
        layer, row, col = cell
        if self._is_free_cell(layer, row, col):
            return cell
        n_layers = int(self.n_slice or 0)
        n_rows = int(self.layers_t.shape[1])
        n_cols = int(self.layers_t.shape[2])
        for radius in range(1, max_radius + 1):
            candidates: list[tuple[float, int, int, int]] = []
            for dl in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        if max(abs(dl), abs(dr), abs(dc)) != radius:
                            continue
                        nl, nr, nc = layer + dl, row + dr, col + dc
                        if not (0 <= nl < n_layers and 0 <= nr < n_rows and 0 <= nc < n_cols):
                            continue
                        if self._is_free_cell(nl, nr, nc):
                            distance = (dc * dc + dr * dr) ** 0.5 + abs(dl) * 0.25
                            candidates.append((distance, nl, nr, nc))
            if candidates:
                _, nl, nr, nc = min(candidates, key=lambda item: item[0])
                return nl, nr, nc
        return None

    def _plan_raw_cells(self, start_pos: Any, end_pos: Any, start_height: float, end_height: float) -> list[tuple[int, int, int]]:
        start = self._nearest_free_cell(self._world_to_cell(start_pos, start_height))
        goal = self._nearest_free_cell(self._world_to_cell(end_pos, end_height))
        if start is None or goal is None:
            return []
        if start == goal:
            return [start]

        layer_goal, row_goal, col_goal = goal

        def heuristic(cell: tuple[int, int, int]) -> float:
            layer, row, col = cell
            return ((col_goal - col) ** 2 + (row_goal - row) ** 2) ** 0.5 + abs(layer_goal - layer) * 0.25

        neighbor_steps = [
            (dl, dr, dc)
            for dl in (-1, 0, 1)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if not (dl == 0 and dr == 0 and dc == 0)
            and (dl == 0 or (dr == 0 and dc == 0))
        ]
        open_q = [(heuristic(start), 0.0, start)]
        g_score: dict[tuple[int, int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int, int], tuple[int, int, int]] = {}
        reached = False

        while open_q:
            _, cost_so_far, current = heapq.heappop(open_q)
            if cost_so_far > g_score.get(current, float("inf")) + 1e-9:
                continue
            if current == goal:
                reached = True
                break
            layer, row, col = current
            for dl, dr, dc in neighbor_steps:
                nxt = (layer + dl, row + dr, col + dc)
                if not self._is_free_cell(*nxt):
                    continue
                step_xy = (dc * dc + dr * dr) ** 0.5
                step = max(step_xy + abs(dl) * 0.5, 0.25)
                new_cost = cost_so_far + step
                if new_cost < g_score.get(nxt, float("inf")):
                    g_score[nxt] = new_cost
                    came_from[nxt] = current
                    heapq.heappush(open_q, (new_cost + heuristic(nxt), new_cost, nxt))

        if not reached:
            return []
        path = []
        current = goal
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def _cell_height(self, layer: int, row: int, col: int) -> float:
        import numpy as np

        if self.layers_g is not None:
            height = float(self.layers_g[layer, row, col])
            if np.isfinite(height):
                return height
        return float(self.slice_h0 or 0.0) + layer * float(self.slice_dh or 0.0)

    def _cell_to_world(self, layer: int, row: int, col: int) -> tuple[float, float, float]:
        x = (col - self.map_dim[0] // 2) * float(self.resolution) + float(self.center[0])
        y = (row - self.map_dim[1] // 2) * float(self.resolution) + float(self.center[1])
        return float(x), float(y), self._cell_height(layer, row, col)

    def _cells_to_world(self, cells: list[tuple[int, int, int]]):
        import numpy as np

        return np.asarray([self._cell_to_world(*cell) for cell in cells], dtype=np.float64)

    def _build_optimizer_request(self, cells: list[tuple[int, int, int]]) -> tuple[dict[str, Any], tuple[int, int]]:
        import numpy as np

        rows = [row for _, row, _ in cells]
        cols = [col for _, _, col in cells]
        margin = max(0, int(self.local_map_margin))
        row0 = max(0, min(rows) - margin)
        row1 = min(self.layers_t.shape[1] - 1, max(rows) + margin)
        col0 = max(0, min(cols) - margin)
        col1 = min(self.layers_t.shape[2] - 1, max(cols) + margin)

        crop_t = np.ascontiguousarray(self.layers_t[:, row0 : row1 + 1, col0 : col1 + 1], dtype=np.float64)
        crop_g = np.ascontiguousarray(self.layers_g[:, row0 : row1 + 1, col0 : col1 + 1], dtype=np.float64)
        crop_c = np.ascontiguousarray(self.layers_c[:, row0 : row1 + 1, col0 : col1 + 1], dtype=np.float64)
        states = []
        layers = []
        heights = []
        for index, (layer, row, col) in enumerate(cells):
            if len(cells) == 1:
                prev_col, prev_row = col, row
                next_col, next_row = col + 1, row
            elif index == 0:
                _, next_row, next_col = cells[index + 1]
                prev_col, prev_row = col, row
            elif index == len(cells) - 1:
                _, prev_row, prev_col = cells[index - 1]
                next_col, next_row = col, row
            else:
                _, prev_row, prev_col = cells[index - 1]
                _, next_row, next_col = cells[index + 1]
            dx = float(next_col - prev_col)
            dy = float(next_row - prev_row)
            heading = math.atan2(dy, dx) if dx or dy else 0.0
            velocity = max((dx * dx + dy * dy) ** 0.5 / 2.0, 1.0)
            states.append(
                [
                    float(col - col0),
                    math.cos(heading) * velocity,
                    0.0,
                    float(row - row0),
                    math.sin(heading) * velocity,
                    0.0,
                ]
            )
            layers.append(int(layer))
            heights.append(self._cell_height(layer, row, col))

        interpolation_steps = max(
            0,
            int(os.environ.get("LINGTU_PCT_RUST_INTERPOLATION_STEPS", "8")),
        )
        self.last_optimizer_interpolation_steps = interpolation_steps

        return (
            {
                "schema": "lingtu.pct_gpmp.optimize.request.v1",
                "states": states,
                "layers": layers,
                "height_hints": heights,
                "map": {
                    "resolution": float(self.resolution),
                    "num_layers": int(self.n_slice or crop_t.shape[0]),
                    "max_x": int(crop_t.shape[2]),
                    "max_y": int(crop_t.shape[1]),
                    "cost": crop_t.reshape(-1).tolist(),
                    "height": crop_g.reshape(-1).tolist(),
                    "ceiling": crop_c.reshape(-1).tolist(),
                },
                "cost_threshold": float(self.obstacle_thr),
                "gp_qc": 10.0,
                "delta": 1.0,
                "obstacle_sigma": 0.2,
                "heading_rate_sigma": 1.0,
                "max_heading_rate": float(self.max_heading_rate),
                "interpolation_steps": interpolation_steps,
                "config": {
                    "max_iterations": int(os.environ.get("LINGTU_PCT_RUST_OPTIMIZER_ITERS", "60")),
                    "initial_lambda": 200.0,
                    "lambda_up": 10.0,
                    "lambda_down": 0.3,
                    "gradient_tolerance": 1e-8,
                    "step_tolerance": 1e-8,
                    "cost_tolerance": 1e-9,
                    "linear_solver": self.optimizer_linear_solver,
                    "nonlinear_optimizer": self.optimizer_nonlinear_optimizer,
                },
            },
            (col0, row0),
        )

    def _optimizer_response_to_world(self, response: dict[str, Any], crop_origin: tuple[int, int]):
        import numpy as np

        col0, row0 = crop_origin
        states = np.asarray(
            self._response_field(response, "trajectory_states", "states"),
            dtype=np.float64,
        )
        heights = np.asarray(
            self._response_field(response, "trajectory_heights", "heights"),
            dtype=np.float64,
        ).reshape(-1)
        if states.ndim != 2 or states.shape[1] < 4 or states.shape[0] != heights.shape[0]:
            raise ValueError("optimizer returned invalid states/heights")
        world = np.empty((states.shape[0], 3), dtype=np.float64)
        world[:, 0] = (states[:, 0] + col0 - self.map_dim[0] // 2) * float(self.resolution) + float(self.center[0])
        y_index = 3 if states.shape[1] >= 6 else 2
        world[:, 1] = (states[:, y_index] + row0 - self.map_dim[1] // 2) * float(self.resolution) + float(self.center[1])
        world[:, 2] = heights
        return world

    def _hard_obstacle_sample_count(self, path_world: Any) -> int:
        import numpy as np

        if path_world is None or self.layers_t is None:
            return 0
        pts = np.asarray(path_world, dtype=np.float64)
        if pts.ndim != 2 or len(pts) == 0:
            return 0
        blocked = 0
        for sample in pts:
            try:
                idx = self.pos2idx(sample[:2])
                col = int(np.clip(round(float(idx[0])), 0, self.layers_t.shape[2] - 1))
                row = int(np.clip(round(float(idx[1])), 0, self.layers_t.shape[1] - 1))
                layer = int(np.clip(round(self.pos2slice(float(sample[2]))), 0, self.layers_t.shape[0] - 1))
            except Exception:
                blocked += 1
                continue
            if not self._is_free_cell(layer, row, col):
                blocked += 1
        return blocked


