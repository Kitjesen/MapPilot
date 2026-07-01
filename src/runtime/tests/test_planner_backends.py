"""PCT planner unit tests without native libraries.

Coverage:
  PCTPlanner:
    - unavailable without native runtime
    - missing tomogram returns an empty path
    - registered as the pct planner
"""


import numpy as np
import pytest

from runtime.registry import get, restore, snapshot

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def registry_isolation():
    """Save registry state before each test, restore after to prevent pollution."""
    saved = snapshot()
    # Trigger @register decorators (executed at import time)
    import importlib
    import nav.services.plan.global_planner.algorithm.pct.planner
    importlib.reload(nav.services.plan.global_planner.algorithm.pct.planner)
    yield
    restore(saved)
    # Re-register after restore clears the registry
    importlib.reload(nav.services.plan.global_planner.algorithm.pct.planner)


# ---------------------------------------------------------------------------
# PCTPlanner tests
# ---------------------------------------------------------------------------

class TestPCTPlanner:
    def _planner(self, tomogram_path="", obs=49.9):
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        return PCTPlanner(tomogram_path=tomogram_path, obstacle_thr=obs)

    def test_registered_as_pct(self):
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        cls = get("planner_backend", "pct")
        assert cls is PCTPlanner

    def test_unavailable_on_missing_so(self):
        """ele_planner.so not available → available=False, no crash."""
        b = self._planner(tomogram_path="nonexistent.pickle")
        assert not b.available

    def test_plan_returns_empty_when_unavailable(self):
        """plan() returns [] when unavailable — must not raise."""
        b = self._planner()
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))
        assert result == [], "PCTPlanner unavailable must return [], not crash"

    def test_plan_returns_empty_on_missing_tomogram(self):
        b = self._planner(tomogram_path="/nonexistent/path/tomo.pickle")
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        assert result == []

    def test_load_error_message_present(self):
        """_load_error must be non-empty when unavailable for easier debugging."""
        b = self._planner(tomogram_path="not_a_real_file.pickle")
        assert len(b._load_error) > 0

    def test_near_zero_route_bypasses_native_pct_plan(self, monkeypatch):
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner

        class FakeNativePlanner:
            def __init__(self):
                self.plan_calls = 0

            def pos2idx(self, _pos):
                return np.array([10.0, 20.0])

            def plan(self, *_args):
                self.plan_calls += 1
                raise AssertionError("native PCT plan must not be called")

        planner = FakeNativePlanner()
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = planner
        backend._resolution = 0.2
        backend._load_error = ""
        monkeypatch.setattr(
            backend,
            "_select_traversable_height",
            lambda _pos, fallback_z, **_kw: float(fallback_z),
        )

        path = backend.plan(
            np.array([1.0, 2.0, 0.3]),
            np.array([1.35, 2.0, 0.4]),
        )

        assert planner.plan_calls == 0
        assert path == [(1.0, 2.0, 0.3), (1.35, 2.0, 0.4)]

    def test_near_xy_route_bypasses_native_pct_plan(self, monkeypatch):
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner

        class FakeNativePlanner:
            def __init__(self):
                self.plan_calls = 0

            def plan(self, *_args):
                self.plan_calls += 1
                raise AssertionError("native PCT plan must not be called")

        planner = FakeNativePlanner()
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = planner
        backend._resolution = 0.2
        backend._load_error = ""
        monkeypatch.setattr(
            backend,
            "_select_traversable_height",
            lambda _pos, fallback_z, **_kw: float(fallback_z),
        )

        path = backend.plan(
            np.array([1.0, 2.0, 0.3]),
            np.array([1.1, 2.0, 0.4]),
        )

        assert planner.plan_calls == 0
        assert path == [(1.0, 2.0, 0.3), (1.1, 2.0, 0.4)]
