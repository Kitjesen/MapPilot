"""Backend adapter for TARE exploration.

Owns optional ``lingtu_explore_kernel`` discovery and dict<->Grid2D conversion
so the policy/module keep only lifecycle and dataflow state. Mirrors
``nav.local.terrain_backend.create_nanobind_terrain_backend``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from explore.kernel import explore_kernel_build_hint, try_import_explore_kernel
from runtime.msgs.numpy_compat import np

_REQUIRED_SYMBOLS = ("Grid2D", "Pose2D", "ExploreDecision", "TarePolicyConfig", "TarePolicy")
_CONFIG_FIELDS = (
    "min_frontier_size",
    "sensor_range_m",
    "candidate_radius_m",
    "min_goal_distance_m",
    "novelty_radius_m",
    "max_candidates",
)


@dataclass(frozen=True)
class NanobindExploreBackend:
    """Holds the native TARE policy and performs data conversion."""

    kernel: Any
    policy: Any
    config: Any

    def plan(
        self,
        *,
        grid_payload: dict[str, Any],
        robot_xy: tuple[float, float],
        robot_yaw: float = 0.0,
        visited_goals: list[tuple[float, float]] | None = None,
    ) -> Any:
        """Run the native planner and return a C++ ``ExploreDecision``."""
        grid = _grid_from_payload(self.kernel, grid_payload)
        if grid is None:
            decision = self.kernel.ExploreDecision()
            decision.reason = "missing_grid"
            return decision
        robot = self.kernel.Pose2D(float(robot_xy[0]), float(robot_xy[1]), float(robot_yaw))
        visited = [self.kernel.Pose2D(float(vx), float(vy), 0.0) for vx, vy in (visited_goals or [])]
        return self.policy.select(grid, robot, visited)


def _grid_from_payload(kernel: Any, payload: dict[str, Any]) -> Any | None:
    """Convert a LingTu exploration grid payload to a native ``Grid2D``."""
    if not isinstance(payload, dict):
        return None
    raw = payload.get("grid")
    if raw is None:
        return None
    arr = np.asarray(raw)
    if arr.ndim != 2:
        return None
    resolution = float(payload.get("resolution") or 0.0)
    if resolution <= 0.0:
        return None
    origin = payload.get("origin") or [
        payload.get("origin_x", 0.0),
        payload.get("origin_y", 0.0),
    ]
    grid = kernel.Grid2D()
    grid.height = int(arr.shape[0])
    grid.width = int(arr.shape[1])
    grid.resolution = resolution
    grid.origin_x = float(origin[0])
    grid.origin_y = float(origin[1])
    grid.cells = [int(v) for v in arr.reshape(-1).tolist()]
    return grid


def create_nanobind_explore_backend(
    config: Any | None = None,
    *,
    kernel_importer: Callable[[tuple[str, ...]], Any | None] = try_import_explore_kernel,
    build_hint_provider: Callable[[], str] = explore_kernel_build_hint,
) -> NanobindExploreBackend:
    """Create the nanobind TARE policy backend or raise a clear build error."""
    kernel = kernel_importer(_REQUIRED_SYMBOLS)
    if kernel is None:
        raise RuntimeError(
            "Explore [nanobind]: compatible LingTu native exploration kernel not found. "
            "Build the C++ backend to run TARE exploration.\n"
            f"  To build: {build_hint_provider()}"
        )
    try:
        native_config = kernel.TarePolicyConfig()
        if config is not None:
            for attr in _CONFIG_FIELDS:
                if hasattr(config, attr) and hasattr(native_config, attr):
                    setattr(native_config, attr, getattr(config, attr))
        return NanobindExploreBackend(
            kernel=kernel,
            policy=kernel.TarePolicy(native_config),
            config=native_config,
        )
    except RuntimeError:
        raise
    except Exception as e:  # pragma: no cover - defensive
        raise RuntimeError(f"Explore [nanobind]: LingTu native exploration kernel init failed: {e}.") from e
