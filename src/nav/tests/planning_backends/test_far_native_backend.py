"""Product contract tests for the optional native FAR planner adapter."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from nav.services.plan.contracts import (
    GlobalPlanRequest,
    GlobalPlanResult,
    GlobalPlanningMap,
)
from nav.services.plan.global_planner import backend_runtime
from nav.services.plan.global_planner.algorithm import far_planner


def _native_library() -> Path:
    repo = Path(__file__).resolve().parents[4]
    names = ("nav_far.dll", "libnav_far.so", "libnav_far.dylib")
    roots = (
        repo / ".tmp" / "nav-product-build" / "Release",
        repo / ".tmp" / "nav-product-build",
        repo / "build" / "nav" / "Release",
        repo / "build" / "nav",
    )
    for root in roots:
        for name in names:
            candidate = root / name
            if candidate.is_file():
                return candidate
    pytest.skip("native FAR library has not been built")


def _open_map(*, generation: int = 1) -> GlobalPlanningMap:
    return GlobalPlanningMap(
        grid=np.zeros((12, 20), dtype=np.float32),
        resolution=0.5,
        origin=np.asarray([0.0, 0.0]),
        frame_id="map",
        map_version="warehouse-v1",
        generation=generation,
        source="test",
    )


def _request(*, generation: int) -> GlobalPlanRequest:
    return GlobalPlanRequest(
        start=np.asarray([1.25, 3.25, 0.0]),
        goal=np.asarray([8.75, 3.25, 0.0]),
        frame_id="map",
        map_version="warehouse-v1",
        map_generation=generation,
    )


def test_generation_round_trips_through_global_planner_wire_contract() -> None:
    planning_map = GlobalPlanningMap.from_wire(_open_map(generation=7).to_wire())
    request = GlobalPlanRequest.from_wire(_request(generation=7).to_wire())
    result = GlobalPlanResult.from_wire(
        GlobalPlanResult(map_generation=7, frame_id="map").to_wire()
    )

    assert planning_map.generation == 7
    assert request.map_generation == 7
    assert result.map_generation == 7


def test_missing_native_library_fails_explicitly(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(far_planner, "_library_candidates", lambda: [])
    monkeypatch.setattr(far_planner.ctypes.util, "find_library", lambda _name: None)

    with pytest.raises(far_planner.FarNativeUnavailable, match="native FAR library not found"):
        far_planner.FarPlannerBackend()


def test_native_far_plans_known_free_and_rejects_stale_generation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LINGTU_NAV_FAR_LIB", str(_native_library()))
    backend = far_planner.FarPlannerBackend(
        options={"robot_radius_m": 0.0, "obstacle_clearance_m": 0.0}
    )
    try:
        backend.update_planning_map(_open_map(generation=7))
        result = backend.plan_request(_request(generation=7))
        assert result.ok
        assert result.reached_goal
        assert result.map_generation == 7
        assert result.diagnostics["planning_phase"] == "known_free"

        stale = backend.plan_request(_request(generation=6))
        assert not stale.ok
        assert "generation" in stale.error.lower()
        assert backend._generation == 7
    finally:
        backend.close()


def test_unversioned_identical_map_update_is_idempotent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LINGTU_NAV_FAR_LIB", str(_native_library()))
    backend = far_planner.FarPlannerBackend(
        options={"robot_radius_m": 0.0, "obstacle_clearance_m": 0.0}
    )
    try:
        value = _open_map(generation=0)
        backend.update_planning_map(value)
        first_generation = backend._generation
        backend.update_planning_map(value)
        assert backend._generation == first_generation
    finally:
        backend.close()


def test_unknown_space_requires_explicit_fallback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LINGTU_NAV_FAR_LIB", str(_native_library()))
    unknown_map = _open_map(generation=3)
    unknown_map.grid[:, 10] = -1.0

    strict = far_planner.FarPlannerBackend(
        options={"robot_radius_m": 0.0, "obstacle_clearance_m": 0.0}
    )
    permissive = far_planner.FarPlannerBackend(
        options={
            "robot_radius_m": 0.0,
            "obstacle_clearance_m": 0.0,
            "allow_unknown_fallback": True,
        }
    )
    try:
        strict.update_planning_map(unknown_map)
        strict_result = strict.plan_request(_request(generation=3))
        assert not strict_result.ok

        permissive.update_planning_map(unknown_map)
        fallback_result = permissive.plan_request(_request(generation=3))
        assert fallback_result.ok
        assert fallback_result.diagnostics["planning_phase"] == "unknown_fallback"
        assert fallback_result.diagnostics["used_unknown_space"] is True
    finally:
        strict.close()
        permissive.close()


def test_backend_factory_selects_far_without_python_planner_fallback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("LINGTU_NAV_FAR_LIB", str(_native_library()))
    backend = backend_runtime.create_planner_backend(
        "far",
        "",
        49.9,
        far_options={"robot_radius_m": 0.0, "obstacle_clearance_m": 0.0},
    )
    try:
        assert isinstance(backend, far_planner.FarPlannerBackend)
        assert backend.available
    finally:
        backend.close()
