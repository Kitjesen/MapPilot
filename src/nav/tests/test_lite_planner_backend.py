from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from runtime.msgs.numpy_compat import np
from runtime.msgs.nav import Path as NavPath
from nav.services.plan.contracts import GlobalPlanRequest, GlobalPlanResult
from nav.services.plan.global_planner.algorithm.direct_path import DirectPathBackend

SRC_DIR = Path(__file__).resolve().parents[2]


def test_direct_path_backend_returns_start_and_goal_without_map() -> None:
    backend = DirectPathBackend()

    path = backend.plan(np.asarray([1.0, 2.0, 0.3]), np.asarray([3.0, 4.0, 0.5]))

    assert len(path) == 2
    assert np.allclose(path[0], [1.0, 2.0, 0.3])
    assert np.allclose(path[1], [3.0, 4.0, 0.5])
    assert backend._last_plan_reached_goal is True
    assert backend._last_plan_diagnostics["map_required"] is False


def test_direct_path_backend_uses_plan_request_contract() -> None:
    backend = DirectPathBackend()

    result = backend.plan_request(
        GlobalPlanRequest(
            start=np.asarray([1.0, 2.0, 0.3]),
            goal=np.asarray([3.0, 4.0, 0.5]),
        )
    )

    assert isinstance(result, GlobalPlanResult)
    assert isinstance(result.path, NavPath)
    assert result.ok is True
    assert result.reached_goal is True
    assert len(result.path) == 2


def test_direct_path_backend_rejects_nonfinite_goal() -> None:
    backend = DirectPathBackend()

    path = backend.plan(np.asarray([1.0, 2.0, 0.0]), np.asarray([float("nan"), 4.0, 0.0]))

    assert path == []
    assert backend._last_plan_reached_goal is False
    assert "non-finite" in backend._last_plan_error


def test_direct_path_backend_collapses_near_zero_route_to_goal() -> None:
    backend = DirectPathBackend()

    path = backend.plan(np.asarray([1.0, 2.0, 0.3]), np.asarray([1.0, 2.0, 0.5]))

    assert len(path) == 1
    assert np.allclose(path[0], [1.0, 2.0, 0.5])


def test_direct_path_backend_import_does_not_load_map_backed_planning() -> None:
    script = (
        "import sys\n"
        "from nav.services.plan.global_planner.algorithm.direct_path import DirectPathBackend\n"
        "forbidden = {'nav.services.plan.global_planner.service', 'global_planning', 'nav.services.safety.plan_safety'}\n"
        "print(any(name == f or name.startswith(f + '.') for name in sys.modules for f in forbidden))\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    result = subprocess.run(
        [sys.executable, "-c", script],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.strip() == "False"
