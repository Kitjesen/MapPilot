from __future__ import annotations

from .common import *  # noqa: F401,F403

class RustProcessGPMPOptimizer:
    """Native-like WNOJ optimizer result view backed by Rust process output."""

    def __init__(self, planner: "RustProcessTomogramPlanner"):
        self._planner = planner
        self._debug = False

    def set_debug(self, flag: bool) -> None:
        self._debug = bool(flag)

    def get_result_matrix(self):
        return self._planner.get_result_matrix()

    def get_layers(self):
        return self._planner.get_layers()

    def get_heights(self):
        return self._planner.get_heights()

    def get_ceilings(self):
        return self._planner.get_ceilings()

    def get_opt_init_value(self):
        return self._planner.get_opt_init_value()

    def get_opt_init_layer(self):
        return self._planner.get_opt_init_layer()

    def get_heading_rate(self):
        return self._planner.get_heading_rate()


class RustProcessGPMPOptimizerWnoa:
    """Native-like WNOA optimizer result view backed by Rust process output."""

    def __init__(self, planner: "RustProcessTomogramPlanner"):
        self._planner = planner
        self._debug = False

    def set_debug(self, flag: bool) -> None:
        self._debug = bool(flag)

    def get_result_matrix(self):
        return self._planner.get_result_matrix()

    def get_layers(self):
        return self._planner.get_layers()

    def get_heights(self):
        return self._planner.get_heights()

    def get_opt_init_value(self):
        return self._planner.get_opt_init_value()

    def get_opt_init_layer(self):
        return self._planner.get_opt_init_layer()

    def gp_prior_test(self, x0: Any, xN: Any, T: float, N: int):
        import numpy as np

        state_count = int(N)
        if state_count < 2:
            raise ValueError("N must be at least 2")
        start = np.asarray(x0, dtype=np.float64).reshape(-1)
        goal = np.asarray(xN, dtype=np.float64).reshape(-1)
        if start.shape[0] != 4 or goal.shape[0] != 4:
            raise ValueError("gp_prior_test expects Vector4 endpoints")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
            raise ValueError("gp_prior_test endpoints must be finite")
        total_time = float(T)
        if not np.isfinite(total_time) or total_time <= 0:
            raise ValueError("T must be positive and finite")

        states = [
            (start + (goal - start) * (index / (state_count - 1))).astype(float).tolist()
            for index in range(state_count)
        ]
        request = {
            "schema": "lingtu.pct_gpmp.optimize.request.v1",
            "mode": "wnoa",
            "states": states,
            "gp_qc": 0.1,
            "delta": total_time / (state_count - 1),
            "endpoint_prior_sigmas": [0.01, 0.01, 0.01, 0.01],
            "interpolation_steps": 0,
            "config": {
                "max_iterations": int(os.environ.get("LINGTU_PCT_RUST_OPTIMIZER_ITERS", "60")),
                "initial_lambda": 200.0,
                "lambda_up": 10.0,
                "lambda_down": 0.3,
                "gradient_tolerance": 1e-8,
                "step_tolerance": 1e-8,
                "cost_tolerance": 1e-9,
                "linear_solver": self._planner.optimizer_linear_solver,
                "nonlinear_optimizer": self._planner.optimizer_nonlinear_optimizer,
            },
        }
        response = self._planner._invoke_optimizer(request)
        self._planner._validate_optimizer_response(response, request, expected_state_dim=4)
        result = np.asarray(response.get("states"), dtype=np.float64)
        if result.ndim != 2 or result.shape != (state_count, 4):
            raise ValueError("optimizer returned invalid gp_prior_test result shape")
        return result


