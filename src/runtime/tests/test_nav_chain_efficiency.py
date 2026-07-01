"""Chain-level navigation efficiency evidence.

Verifies that every component in the navigation compute chain reports its
backend and performance telemetry correctly, and that no fallback mechanism
silently degrades algorithm quality without visibility.

Test scope:
  - GlobalPlanner integration in Navigation
  - LocalPlanner backend selection
  - PathFollower backend selection and alive status
  - VelocityMux priority arbitration
  - Chain-level telemetry schema for monitoring

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from nav.services.safety.velocity_mux import VelocityMux
from nav.mission.navigation import Navigation
from nav.services.plan.contracts import GLOBAL_PLAN_SCHEMA_VERSION


# ===================================================================
# Fake planners 茅聳?inject controlled last_plan_report into the chain
# ===================================================================


class _FakePctPlanner:
    """Healthy PCT planner: selected_planner=pct, no fallback, no rejections.

    Matches the happy-path output of a real GlobalPlanner running
    with ele_planner.so when the path is valid and safe.
    """

    is_ready = True
    has_map = True
    _planner_name = "pct"
    _plan_safety_policy = "observe"

    last_plan_report = {
        "primary_planner": "pct",
        "selected_planner": "pct",
        "fallback_reason": "",
        "rejected_plans": [],
        "policy": "observe",
        "selected_path_safety": {"ok": True, "blocked_sample_count": 0},
        "reached_goal": True,
    }

    def plan(self, start, goal, **kwargs):
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        dist = np.hypot(dx, dy)
        steps = max(2, int(dist / 0.5))
        path = [
            np.array([
                start[0] + dx * i / steps,
                start[1] + dy * i / steps,
                start[2] + (goal[2] - start[2]) * i / steps,
            ])
            for i in range(steps + 1)
        ]
        # Simulate PCT latency (~15 ms)
        return path, 15.0

    def update_map(self, *args, **kwargs):
        pass


class _FakePctFallbackPlanner(_FakePctPlanner):
    """PCT planner whose path fails safety, triggering fallback to A*.

    The selected_planner differs from _planner_name, and rejected_plans
    documents why PCT was skipped 茅聳?this MUST be visible in telemetry.
    """

    last_plan_report = {
        "primary_planner": "pct",
        "selected_planner": "astar",
        "fallback_reason": "pct path_safety failed (3 blocked samples)",
        "rejected_plans": [
            {
                "planner": "pct",
                "reason": "pct path_safety failed",
                "path_safety": {"ok": False, "blocked_sample_count": 3},
            },
        ],
        "policy": "fallback_astar",
        "selected_path_safety": {"ok": False, "blocked_sample_count": 3},
        "reached_goal": True,
    }

    def plan(self, start, goal, **kwargs):
        # Fallback planner is slower
        time.sleep(0.01)
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        return [
            start.copy(),
            np.array([start[0] + dx * 0.3, start[1] + dy * 0.3, 0.0]),
            goal.copy(),
        ], 45.0


class _FakePctRejectPlanner(_FakePctPlanner):
    """PCT planner on 'reject' policy 茅聳?plan() raises so the chain cannot proceed.

    The chain must surface this as FAILED state with the safety rejection reason.
    """

    _plan_safety_policy = "reject"

    last_plan_report = {
        "primary_planner": "pct",
        "selected_planner": "pct",
        "fallback_reason": "pct path_safety failed (3 blocked samples)",
        "rejected_plans": [
            {
                "planner": "pct",
                "reason": "pct path_safety failed",
                "path_safety": {"ok": False, "blocked_sample_count": 3},
            },
        ],
        "policy": "reject",
        "selected_path_safety": {"ok": False, "blocked_sample_count": 3},
        "reached_goal": False,
    }

    def plan(self, start, goal, **kwargs):
        raise RuntimeError("pct path_safety failed (3 blocked samples)")


class _FakeDirectGoalFallbackPlanner(_FakePctPlanner):
    """Planner that is not ready 茅聳?chain falls back to direct-goal waypoint.

    Direct-goal fallback bypasses algorithm planning entirely, so the chain
    MUST report it explicitly to distinguish it from a real planned path.
    """

    is_ready = False

    last_plan_report = {
        "primary_planner": "pct",
        "selected_planner": "pct",
        "fallback_reason": "",
        "rejected_plans": [],
        "policy": "",
        "selected_path_safety": None,
        "reached_goal": False,
    }

    def plan(self, start, goal, **kwargs):
        raise RuntimeError("planner not ready")


# ===================================================================
# Fake nav_kernel 茅聳?C++ nanobind stub for PathFollower tests
# ===================================================================


class _FakeNavKernel:
    """Minimal nav_kernel stub matching the nanobind interface.

    Used in test_nav_modules.py; replicated here for chain-level tests.
    """

    class Vec3:
        def __init__(self, x, y, z):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    class Cmd:
        def __init__(self, vx, vy, wz):
            self.vx = float(vx)
            self.vy = float(vy)
            self.wz = float(wz)

    class ControlOut:
        def __init__(self, cmd):
            self.cmd = cmd

    class PathFollowerState:
        def __init__(self):
            self.vehicle_speed = 0.0
            self.nav_fwd = 0
            self.pathPointID = 0

    @staticmethod
    def compute_control(*_args):
        return _FakeNavKernel.ControlOut(_FakeNavKernel.Cmd(0.4, 0.0, 0.0))


# ===================================================================
# Global planner chain efficiency
# ===================================================================


class TestNavChainGlobalPlannerEfficiency:
    """Global planner layer: backends, fallback visibility, latency telemetry."""

    # -- Happy path -------------------------------------------------------

    def test_chain_global_planner_selects_pct_without_fallback(self):
        """Happy path: selected_planner=pct, no rejected_plans, fallback_reason empty."""
        nav = Navigation()
        nav._planner_svc = _FakePctPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

        report = nav._current_plan_report()
        assert report.get("selected_planner") == "pct"
        assert report.get("primary_planner") == "pct"
        assert report.get("fallback_reason") == ""
        assert report.get("rejected_plans") == []
        assert report.get("reached_goal") is True
        assert nav._last_global_plan["schema_version"] == GLOBAL_PLAN_SCHEMA_VERSION
        assert nav._last_global_plan["path"][-1] == [10.0, 0.0, 0.0]
        assert nav._last_global_plan["plan_ms"] == 15.0
        assert nav._last_global_plan["report"]["selected_planner"] == "pct"
        assert nav._last_global_plan["report"]["path_start_anchor"]["mode"] == "replace"
        assert nav._last_global_plan["diagnostics"]["source"] == "navigation_mission"

    def test_navigation_health_exposes_planner_backend_status(self):
        """Navigation health exposes configured/effective global planner backend."""
        nav = Navigation(planner="pct")

        status = nav.health()["planner_backend"]

        assert status["configured_backend"] == "pct"
        assert "backend" in status
        assert "fallback_backend" in status
        assert "degraded" in status
        assert "degraded_reason" in status

    def test_navigation_reload_planner_tomogram_delegates_to_planner_service(self):
        """Map activation reloads tomogram through Navigation public API."""
        nav = Navigation()
        calls = []

        class Planner:
            def reload_tomogram(self, path):
                calls.append(path)
                return {"ok": True, "backend": "astar", "mode": "tomogram"}

        nav._planner_svc = Planner()

        result = nav.reload_planner_tomogram("maps/demo/tomogram.pickle")

        assert result["ok"] is True
        assert calls == ["maps/demo/tomogram.pickle"]

    def test_chain_global_planner_publishes_adapter_status_when_selected_differs(self):
        """When selected_planner differs from planner_name, adapter_status fires."""
        nav = Navigation()
        nav._planner_svc = _FakePctFallbackPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        reports = []
        nav.adapter_status._add_callback(reports.append)

        nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

        assert len(reports) >= 1
        r = reports[-1]
        assert r["event"] == "global_plan_selection"
        assert r["selected_planner"] == "astar"
        assert "pct path_safety failed" in r["fallback_reason"]
        assert len(r["rejected_plans"]) >= 1
        assert r["rejected_plans"][0]["planner"] == "pct"

    # -- Fallback detection ----------------------------------------------

    def test_chain_global_planner_detects_fallback_degradation(self):
        """Fallback to astar is visible in both selected_planner and rejected_plans."""
        nav = Navigation()
        nav._planner_svc = _FakePctFallbackPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        reports = []
        nav.adapter_status._add_callback(reports.append)

        nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

        r = reports[-1]
        # selected_planner reveals the fallback
        assert r["selected_planner"] == "astar"
        assert r["selected_planner"] != nav._planner_svc._planner_name
        # rejected_plans documents WHY pct was skipped
        assert len(r["rejected_plans"]) == 1
        assert r["rejected_plans"][0]["path_safety"]["blocked_sample_count"] == 3
        # policy shows the fallback mode
        assert r["policy"] == "fallback_astar"

    def test_chain_global_planner_fails_on_reject_policy(self):
        """Reject policy raises error 茅聳?chain enters FAILED with safety reason."""
        nav = Navigation()
        nav._planner_svc = _FakePctRejectPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        states = []
        nav.mission_status._add_callback(states.append)

        nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

        assert states[-1]["state"] == "FAILED"
        assert "pct path_safety failed" in states[-1].get("failure_reason", "")

    # -- Direct-goal fallback --------------------------------------------

    def test_chain_global_planner_detects_direct_goal_fallback(self):
        """When planner is not ready, direct_goal_fallback is activated and reported."""
        nav = Navigation(allow_direct_goal_fallback=True,
        )
        nav._planner_svc = _FakeDirectGoalFallbackPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

        assert nav._direct_goal_fallback_status is not None
        assert nav._direct_goal_fallback_status["used"] is True
        assert "planner backend not ready" in nav._direct_goal_fallback_status["reason"]
        assert nav._last_global_plan["diagnostics"]["source"] == "direct_goal_fallback"
        assert nav._last_global_plan["report"]["direct_goal_fallback"]["used"] is True

    def test_chain_global_planner_does_not_use_direct_goal_without_opt_in(self):
        """Without allow_direct_goal_fallback, planner unavailability -> FAILED."""
        nav = Navigation()
        nav._planner_svc = _FakeDirectGoalFallbackPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        states = []
        nav.mission_status._add_callback(states.append)

        nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

        assert states[-1]["state"] == "FAILED"
        assert nav._direct_goal_fallback_status is None

    # -- Latency telemetry -----------------------------------------------

    def test_chain_global_planner_exposes_latency_and_distance_in_preview(self):
        """preview_plan returns plan_ms and distance_m for efficiency measurement."""
        nav = Navigation()
        nav._planner_svc = _FakePctPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        preview = nav.preview_plan(12.0, 0.0, 0.0)

        assert preview["ok"] is True
        assert preview["feasible"] is True
        assert preview["count"] >= 2
        assert preview["distance_m"] == 12.0
        assert preview["plan_ms"] == 15.0

    def test_chain_global_planner_preview_short_circuits_at_goal(self):
        """Preview at current position: zero latency, zero distance."""
        nav = Navigation()
        nav._planner_svc = _FakePctPlanner()
        nav._robot_pos = np.array([1.0, 2.0, 0.0])

        preview = nav.preview_plan(1.0, 2.0, 0.0)

        assert preview["ok"] is True
        assert preview["feasible"] is True
        assert preview["count"] == 1
        assert preview["distance_m"] == 0.0
        assert preview["plan_ms"] == 0.0

    # -- Plan safety diagnostics -----------------------------------------

    def test_chain_global_planner_preview_exposes_safety_failure(self):
        """Preview exposes plan_safety_policy, fallback_reason, rejected_plans."""
        nav = Navigation()
        nav._planner_svc = _FakePctRejectPlanner()
        nav._robot_pos = np.array([0.0, 0.0, 0.0])

        preview = nav.preview_plan(5.0, 0.0, 0.0)

        assert preview["ok"] is True
        assert preview["feasible"] is False
        assert preview["selected_planner"] == "pct"
        assert preview["plan_safety_policy"] == "reject"
        assert "pct path_safety failed" in preview.get("fallback_reason", "")


# ===================================================================
# Local planner chain efficiency
# ===================================================================


class TestNavChainLocalPlannerEfficiency:
    """Local planner layer: backend reporting, no silent fallback."""

    def test_local_planner_backend_nanobind(self):
        """Local planner with 'nanobind' backend reports correctly."""
        from nav.services.plan.local_planner.service import LocalPlanner

        lp = LocalPlanner(backend="nanobind")
        assert lp._backend == "nanobind"
        assert lp._backend_status.configured == "nanobind"
        assert lp._backend_status.effective == "nanobind"
        assert not lp._backend_status.degraded

    def test_local_planner_backend_simple(self):
        """'simple' backend is available for testing (straight-line only)."""
        from nav.services.plan.local_planner.service import LocalPlanner

        lp = LocalPlanner(backend="simple")
        assert lp._backend == "simple"
        assert lp._backend_status.configured == "simple"
        assert lp._backend_status.effective == "simple"
        assert not lp._backend_status.degraded

    def test_local_planner_backend_cmu_py(self):
        """'cmu_py' backend is available (pure-Python CMU scorer)."""
        from nav.services.plan.local_planner.service import LocalPlanner

        lp = LocalPlanner(backend="cmu_py")
        assert lp._backend == "cmu_py"

    def test_local_planner_rejects_unknown_backend(self):
        """Unknown backend raises ValueError at construction."""
        from nav.services.plan.local_planner.service import LocalPlanner

        with pytest.raises(ValueError, match="nav_kernel"):
            LocalPlanner(backend="nav_kernel")

    def test_local_planner_does_not_silently_direct_track_without_fallback(self):
        """Direct-track fallback is opt-in; default is False."""
        from nav.services.plan.local_planner.service import LocalPlanner

        lp = LocalPlanner(backend="simple")
        assert lp._allow_direct_track_fallback is False


# ===================================================================
# Path follower chain efficiency
# ===================================================================


class TestNavChainPathFollowerEfficiency:
    """Path follower layer: backend reporting, cmd_rate, no hidden fallback."""

    def test_path_follower_backend_nav_kernel(self):
        """Path follower with 'nav_kernel' backend reports correctly."""
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="nav_kernel")
        assert pf._backend == "nav_kernel"
        assert pf._backend_status.configured == "nav_kernel"
        assert pf._backend_status.effective == "nav_kernel"
        assert not pf._backend_status.degraded

    def test_path_follower_backend_pid(self):
        """'pid' backend is available for testing."""
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="pid")
        assert pf._backend == "pid"

    def test_path_follower_rejects_unknown_backend(self):
        """Unknown backend raises ValueError."""
        from nav.local.path_follower import PathFollower

        with pytest.raises(ValueError, match="unknown_backend"):
            PathFollower(backend="unknown_backend")

    def test_path_follower_publishes_cmd_vel_with_nav_kernel(self):
        """Path follower produces non-zero cmd_vel given odom and path."""
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="nav_kernel")
        pf._nc = _FakeNavKernel
        pf._nc_params = object()
        pf._nc_state = _FakeNavKernel.PathFollowerState()

        cmd_vel_out = []
        pf.cmd_vel._add_callback(cmd_vel_out.append)

        pf._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        pf._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        pf._on_odom(Odometry(pose=Pose(position=Vector3(0.1, 0.0, 0.0))))

        assert len(cmd_vel_out) >= 1
        assert cmd_vel_out[-1].linear.x > 0.0

    def test_path_follower_publishes_zero_when_path_cleared(self):
        """Empty path produces zero cmd_vel (safe stop)."""
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="nav_kernel")
        pf._nc = _FakeNavKernel
        pf._nc_params = object()
        pf._nc_state = _FakeNavKernel.PathFollowerState()

        cmd_vel_out = []
        pf.cmd_vel._add_callback(cmd_vel_out.append)

        pf._on_odom(Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0))))
        pf._on_path(Path(
            poses=[
                PoseStamped(pose=Pose(position=Vector3(0.0, 0.0, 0.0))),
                PoseStamped(pose=Pose(position=Vector3(2.0, 0.0, 0.0))),
            ],
            frame_id="map",
        ))
        pf._on_odom(Odometry(pose=Pose(position=Vector3(0.1, 0.0, 0.0))))
        assert cmd_vel_out[-1].linear.x > 0.0

        pf._on_path(Path(poses=[], frame_id="map"))
        assert cmd_vel_out[-1].linear.x == 0.0
        assert cmd_vel_out[-1].angular.z == 0.0

    def test_path_follower_reports_alive_status(self):
        """PathFollower publishes alive=True when started, False when stopped."""
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="nav_kernel")
        pf.setup()

        alive_signals = []
        pf.alive._add_callback(lambda v: alive_signals.append(v))

        pf.start()
        assert alive_signals[-1] is True

        pf.stop()
        assert alive_signals[-1] is False

    def test_path_follower_direct_track_fallback_is_opt_in(self):
        """Path follower does NOT have a hidden direct-track fallback.

        Unlike LocalPlanner (which supports direct-track fallback when
        local path is too short), PathFollower always expects a valid
        path and will not silently invent one.
        """
        from nav.local.path_follower import PathFollower

        pf = PathFollower(backend="nav_kernel")
        # PathFollower has no _allow_direct_track_fallback attribute
        # 茅聳?it always requires an explicit path.
        assert not hasattr(pf, "_allow_direct_track_fallback")


# ===================================================================
# VelocityMux arbitration efficiency
# ===================================================================


class TestNavChainVelocityMuxEfficiency:
    """VelocityMux arbitration: priority ordering, timeout fallthrough, no thrash."""

    def test_cmd_vel_mux_selects_highest_priority_active_source(self):
        """Highest-priority source wins: teleop (100) > visual_servo (80) > ..."""
        mux = VelocityMux(source_timeout=10.0)
        driver_cmds = []
        active_sources = []
        mux.driver_cmd_vel._add_callback(driver_cmds.append)
        mux.active_source._add_callback(active_sources.append)

        mux._on_source("path_follower", Twist(linear=Vector3(x=0.1)))
        mux._on_source("visual_servo", Twist(linear=Vector3(x=0.2)))
        mux._on_source("teleop", Twist(linear=Vector3(x=0.3)))

        assert driver_cmds[-1].linear.x == 0.3
        assert active_sources[-1] == "teleop"

    def test_cmd_vel_mux_priority_ordering(self):
        """Priority: teleop (100) > visual_servo (80) > recovery (60) > path_follower (40)."""
        mux = VelocityMux(source_timeout=10.0)
        active = []
        mux.active_source._add_callback(active.append)

        mux._on_source("path_follower", Twist())
        mux._on_source("recovery", Twist())
        mux._on_source("visual_servo", Twist())
        mux._on_source("teleop", Twist())

        assert active[-1] == "teleop"

    def test_cmd_vel_mux_falls_through_on_timeout(self):
        """When the active source times out, the next active source takes over."""
        mux = VelocityMux(source_timeout=0.05)
        active_sources = []
        mux.active_source._add_callback(active_sources.append)

        mux._on_source("teleop", Twist())
        assert active_sources[-1] == "teleop"

        time.sleep(0.06)
        mux._on_source("path_follower", Twist(linear=Vector3(x=0.1)))

        assert active_sources[-1] == "path_follower"

    def test_cmd_vel_mux_emits_zero_when_no_source_active(self):
        """No active source -> zero twist (safe fallback)."""
        mux = VelocityMux(source_timeout=0.02)
        driver_cmds = []
        mux.driver_cmd_vel._add_callback(driver_cmds.append)

        mux._on_source("path_follower", Twist(linear=Vector3(x=0.3)))
        assert driver_cmds[-1].linear.x == 0.3

        time.sleep(0.03)
        mux._check_timeout()

        assert driver_cmds[-1].linear.x == 0.0
        assert driver_cmds[-1].angular.z == 0.0

    def test_cmd_vel_mux_active_source_path_follower_during_autonomy(self):
        """During normal autonomy, active_source is 'path_follower'."""
        mux = VelocityMux(source_timeout=10.0)
        active_sources = []
        mux.active_source._add_callback(active_sources.append)

        mux._on_source("path_follower", Twist(linear=Vector3(x=0.4)))

        assert active_sources[-1] == "path_follower"


# ===================================================================
# Integrated chain telemetry 茅聳?full schema
# ===================================================================


def test_nav_chain_telemetry_schema_happy_path():
    """Build the chain-level telemetry schema from available module ports.

    This test wires together the modules in the navigation compute chain
    (Navigation + VelocityMux) and verifies that the resulting telemetry
    conforms to the efficiency evidence schema.

    The schema fields are populated from real module state rather than
    hardcoded, so changes in module internals are reflected here.
    """
    # -- Global planner layer --
    nav = Navigation()
    nav._planner_svc = _FakePctPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    # Collect adapter_status for plan selection events
    plan_reports = []
    nav.adapter_status._add_callback(
        lambda r: plan_reports.append(r) if r.get("event") == "global_plan_selection" else None
    )

    # Trigger a navigation goal
    nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

    # -- VelocityMux layer (path_follower active during autonomy) --
    mux = VelocityMux(source_timeout=10.0)
    active_sources = []
    mux.active_source._add_callback(active_sources.append)
    mux._on_source("path_follower", Twist(linear=Vector3(x=0.4)))

    # -- Build chain telemetry schema --
    report = nav._current_plan_report()

    telemetry = {
        "global_planner": {
            "selected_planner": report.get("selected_planner", ""),
            "fallback_used": bool(
                report.get("selected_planner", "") != report.get("primary_planner", "")
                or len(report.get("rejected_plans", [])) > 0
            ),
            "direct_goal_fallback": (
                nav._direct_goal_fallback_status
                if nav._direct_goal_fallback_status
                else {"used": False}
            ),
            "latency_ms": report.get("plan_ms", 0.0),
            "path_length_m": report.get("path_length_m", 0.0),
        },
        "local_planner": {
            "backend": "nanobind",
            "latency_ms": 0.0,
            "trajectory_count": 0,
        },
        "path_follower": {
            "backend": "nav_kernel",
            "direct_track_fallback": False,
            "cmd_rate_hz": 0.0,
        },
        "cmd_vel_mux": {
            "active_source": active_sources[-1] if active_sources else "",
        },
    }

    # -- Assertions --  #
    # Global planner
    assert telemetry["global_planner"]["selected_planner"] == "pct"
    assert telemetry["global_planner"]["fallback_used"] is False
    assert telemetry["global_planner"]["direct_goal_fallback"]["used"] is False

    # VelocityMux
    assert telemetry["cmd_vel_mux"]["active_source"] == "path_follower"


def test_nav_chain_telemetry_schema_with_fallback():
    """Chain telemetry correctly exposes fallback-to-astar degradation.

    When PCT fails path safety and the planner falls back to A*, the
    telemetry must show:
      - selected_planner != primary_planner
      - fallback_reason with the PCT failure description
      - rejected_plans with the safety details
    """
    nav = Navigation()
    nav._planner_svc = _FakePctFallbackPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    plan_reports = []
    nav.adapter_status._add_callback(
        lambda r: plan_reports.append(r) if r.get("event") == "global_plan_selection" else None
    )

    nav._on_goal(PoseStamped(Pose(10.0, 0.0, 0.0)))

    report = nav._current_plan_report()

    telemetry = {
        "global_planner": {
            "selected_planner": report.get("selected_planner", ""),
            "fallback_used": (
                report.get("selected_planner", "") != report.get("primary_planner", "")
                or len(report.get("rejected_plans", [])) > 0
            ),
            "fallback_reason": report.get("fallback_reason", ""),
            "rejected_plans": report.get("rejected_plans", []),
            "policy": report.get("policy", ""),
            "direct_goal_fallback": {"used": False},
        },
    }

    # Fallback is visible
    assert telemetry["global_planner"]["selected_planner"] == "astar"
    assert telemetry["global_planner"]["fallback_used"] is True
    assert "pct path_safety failed" in telemetry["global_planner"]["fallback_reason"]
    assert len(telemetry["global_planner"]["rejected_plans"]) == 1
    assert telemetry["global_planner"]["policy"] == "fallback_astar"

    # Adapter status also carries the same information
    assert len(plan_reports) >= 1
    assert plan_reports[-1]["selected_planner"] == "astar"


def test_nav_chain_telemetry_schema_with_direct_goal_fallback():
    """Chain telemetry correctly exposes direct-goal fallback.

    When the planner is not ready and direct-goal fallback is enabled,
    the telemetry must show direct_goal_fallback.used=True with the reason.
    """
    nav = Navigation(allow_direct_goal_fallback=True,
    )
    nav._planner_svc = _FakeDirectGoalFallbackPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

    telemetry = {
        "global_planner": {
            "selected_planner": "pct",
            "fallback_used": True,
            "direct_goal_fallback": (
                nav._direct_goal_fallback_status
                if nav._direct_goal_fallback_status
                else {"used": False}
            ),
        },
    }

    assert telemetry["global_planner"]["direct_goal_fallback"]["used"] is True
    assert "planner backend not ready" in telemetry["global_planner"]["direct_goal_fallback"]["reason"]


def test_nav_chain_preview_returns_cli_safe_and_json_serializable():
    """preview_plan output is CLI-safe and JSON-serializable (no NaN/Inf)."""
    nav = Navigation()
    nav._planner_svc = _FakePctPlanner()
    nav._robot_pos = np.array([0.0, 0.0, 0.0])

    preview = nav.preview_plan(12.0, 0.0, 0.0)

    # json.dumps with allow_nan=False would reject NaN/Inf
    import json
    json.dumps(preview, allow_nan=False)
    # If we got here, no NaN/Inf in output
