from __future__ import annotations

import pytest

from explore import explore_kernel_available
from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np

requires_explore_kernel = pytest.mark.skipif(
    not explore_kernel_available(),
    reason="lingtu_explore_kernel not built; run scripts/build/build_explore_py.sh",
)


def _grid_payload() -> dict:
    grid = np.full((9, 9), -1, dtype=np.int16)
    grid[3:6, 2:6] = 0
    grid[4, 2] = 0
    return {
        "grid": grid,
        "resolution": 1.0,
        "origin": [0.0, 0.0],
        "frame_id": "odom",
    }


@requires_explore_kernel
def test_portable_tare_policy_selects_reachable_frontier_viewpoint() -> None:
    from explore.tare.policy import PortableTAREPolicy

    decision = PortableTAREPolicy().select(
        grid_payload=_grid_payload(),
        robot_xy=(2.5, 4.5),
        robot_yaw=0.0,
    )

    assert decision.goal is not None
    assert decision.path[0] == (2.5, 4.5, 0.0)
    assert decision.reason == "selected_viewpoint"
    assert decision.candidates


@requires_explore_kernel
def test_tare_module_in_process_emits_goal_from_exploration_grid() -> None:
    from explore.tare.module import TAREExplorerModule

    mod = TAREExplorerModule(
        auto_start=False,
        goal_frame_id="odom",
        transport_mode="in_process",
    )
    goals = []
    paths = []
    mod.exploration_goal._add_callback(goals.append)
    mod.exploration_path._add_callback(paths.append)
    mod._on_odom(
        Odometry(
            pose=Pose(
                position=Vector3(2.5, 4.5, 0.0),
                orientation=Quaternion.from_yaw(0.0),
            )
        )
    )
    mod._on_exploration_grid(_grid_payload())
    mod.start_tare_exploration()

    assert mod._run_policy_once() is True
    assert goals
    assert goals[-1].frame_id == "odom"
    assert paths and len(paths[-1]) >= 2
