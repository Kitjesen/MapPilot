"""Wave 1 simplification-audit fixes 闂?behavioural tests.

Each test targets a specific Wave 1 change from
``C:\\Users\\99563\\.claude\\plans\\streamed-moseying-patterson.md``.
The goal is to lock in safe fallback behaviour so later refactors can't regress
to the old "looks-fine-but-wrong" state.
"""
from __future__ import annotations

import asyncio
import unittest
from unittest import mock

import numpy as np


# ---------------------------------------------------------------------------
# W1-1  LocalPlanner 闂?no silent fallback to straight-line paths
# ---------------------------------------------------------------------------

class TestW1LocalPlannerNoFallback(unittest.TestCase):
    def test_straight_line_fallback_method_removed(self):
        """`_publish_straight_line` must not exist as an attribute anymore."""
        from nav.services.plan.local_planner.service import LocalPlanner
        self.assertFalse(
            hasattr(LocalPlanner, "_publish_straight_line"),
            "The straight-line fallback method must not be re-introduced 闂?"
            "it caused the robot to publish obstacle-ignoring paths when the "
            "C++ backend was unavailable.",
        )

    def test_nanobind_backend_fails_when_nav_kernel_missing(self):
        """setup() must fail fast instead of silently selecting cmu_py/simple."""
        from nav.services.plan.local_planner import service as mod
        from nav.services.plan.local_planner import runtime
        from nav.services.plan.local_planner.backend import (
            LocalPlannerGridConfig,
            NanobindLocalPlannerBackend,
        )

        planner = mod.LocalPlanner(backend="nanobind")
        with (
            mock.patch.object(
                runtime,
                "read_local_planner_grid_config",
                return_value=LocalPlannerGridConfig(),
            ),
            mock.patch.object(
                runtime,
                "create_nanobind_backend",
                return_value=NanobindLocalPlannerBackend(
                    core=None,
                    unavailable_reason="compatible LingTu native navigation kernel missing",
                    build_hint="build nav_kernel",
                ),
            ),
            mock.patch.object(
                runtime,
                "create_cmu_py_backend",
                side_effect=AssertionError("cmu_py must not be auto-created"),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "LingTu native navigation kernel"):
                planner._setup_nanobind()

        self.assertEqual(planner._backend, "nanobind")
        self.assertIsNone(planner._path_data)
        self.assertIsNone(planner._core)

    def test_failed_nanobind_setup_health_does_not_report_degraded_cmu_py(self):
        from nav.services.plan.local_planner import service as mod
        from nav.services.plan.local_planner import runtime
        from nav.services.plan.local_planner.backend import (
            LocalPlannerGridConfig,
            NanobindLocalPlannerBackend,
        )

        planner = mod.LocalPlanner(backend="nanobind")
        with (
            mock.patch.object(
                runtime,
                "read_local_planner_grid_config",
                return_value=LocalPlannerGridConfig(),
            ),
            mock.patch.object(
                runtime,
                "create_nanobind_backend",
                return_value=NanobindLocalPlannerBackend(
                    core=None,
                    unavailable_reason="compatible LingTu native navigation kernel missing",
                    build_hint="build nav_kernel",
                ),
            ),
            mock.patch.object(
                runtime,
                "create_cmu_py_backend",
                side_effect=AssertionError("cmu_py must not be auto-created"),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "LingTu native navigation kernel"):
                planner._setup_nanobind()

        info = planner.health()["local_planner"]
        self.assertEqual(info["configured_backend"], "nanobind")
        self.assertEqual(info["backend"], "nanobind")
        self.assertFalse(info["degraded"])
        self.assertEqual(info["degraded_reason"], "")
        self.assertFalse(info["paths_loaded"])
        self.assertFalse(info["running"])

    def test_nanobind_missing_nav_kernel_does_not_try_cmu_py_paths(self):
        from nav.services.plan.local_planner import service as mod
        from nav.services.plan.local_planner import runtime
        from nav.services.plan.local_planner.backend import (
            LocalPlannerGridConfig,
            NanobindLocalPlannerBackend,
        )

        planner = mod.LocalPlanner(backend="nanobind")
        with (
            mock.patch.object(
                runtime,
                "read_local_planner_grid_config",
                return_value=LocalPlannerGridConfig(),
            ),
            mock.patch.object(
                runtime,
                "create_nanobind_backend",
                return_value=NanobindLocalPlannerBackend(
                    core=None,
                    unavailable_reason="compatible LingTu native navigation kernel missing",
                    build_hint="build nav_kernel",
                ),
            ),
            mock.patch.object(
                runtime,
                "create_cmu_py_backend",
                side_effect=RuntimeError("LocalPlanner [cmu_py]: paths missing"),
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "LingTu native navigation kernel") as ctx:
                planner._setup_nanobind()

        self.assertNotIn("cmu_py]: paths missing", str(ctx.exception))
        self.assertNotEqual(planner._backend, "simple")

    def test_nanobind_backend_requires_symbols_before_claiming_nanobind(self):
        """A stale lingtu_nav_kernel extension must fail before claiming nanobind."""
        from nav.services.plan.local_planner import runtime
        from runtime.backend_status import BackendStatus

        calls = []

        def importer(symbols=()):
            calls.append(tuple(symbols))
            return None

        with self.assertRaisesRegex(RuntimeError, "LingTu native navigation kernel"):
            runtime.setup_local_planner_backend(
                "nanobind",
                status=BackendStatus.configured_as("nanobind"),
                nav_kernel_importer=importer,
                path_loader=lambda _paths_dir: {"paths": object()},
            )

        self.assertEqual(calls, [("LocalPlannerParams", "nav.local_planner")])


# ---------------------------------------------------------------------------
# W1-2  Terrain 闂?no silent fallback when native kernel missing
# ---------------------------------------------------------------------------

class TestW1TerrainNoFallback(unittest.TestCase):
    def test_nanobind_backend_degrades_to_simple_when_nav_kernel_missing(self):
        from nav.local import terrain as mod
        terrain = mod.Terrain(backend="nanobind")
        with mock.patch.object(
            mod,
            "create_nanobind_terrain_backend",
            side_effect=RuntimeError(
                "Terrain [nanobind]: compatible LingTu native navigation kernel not found"
            ),
        ):
            terrain._setup_nanobind()

        self.assertEqual(terrain._backend, "simple")
        self.assertIsNone(terrain._core)
        self.assertTrue(terrain._backend_status.degraded)
        self.assertIn("LingTu native navigation kernel", terrain._backend_status.degraded_reason)

    def test_strict_nanobind_backend_fails_when_nav_kernel_missing(self):
        from nav.local import terrain as mod
        terrain = mod.Terrain(backend="nanobind", strict_native=True)
        with mock.patch.object(
            mod,
            "create_nanobind_terrain_backend",
            side_effect=RuntimeError(
                "Terrain [nanobind]: compatible LingTu native navigation kernel not found"
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "Rebuild the native kernel"):
                terrain._setup_nanobind()

        self.assertEqual(terrain._backend, "nanobind")
        self.assertIsNone(terrain._core)
        self.assertFalse(terrain._backend_status.degraded)

    def test_nanobind_backend_requires_terrain_symbols(self):
        from nav.local.terrain_backend import (
            create_nanobind_terrain_backend,
        )

        calls = []

        def importer(symbols):
            calls.append(tuple(symbols))
            return None

        with self.assertRaises(RuntimeError):
            create_nanobind_terrain_backend(nav_kernel_importer=importer)
        self.assertEqual(calls, [("TerrainParams", "TerrainAnalysisCore")])

    def test_simple_backend_still_allowed_explicitly(self):
        """`backend="simple"` remains a valid passthrough choice for tests."""
        from nav.local.terrain import Terrain
        t = Terrain(backend="simple")
        # setup() must not raise 闂?simple backend = explicit passthrough
        t.setup()


# ---------------------------------------------------------------------------
# W1-3  Navigation 闂?direct-goal fallback OFF by default
# ---------------------------------------------------------------------------

class TestW1NavigationDefaultSafe(unittest.TestCase):
    def test_allow_direct_goal_fallback_defaults_to_false(self):
        """Default must be safe: no planner 闂?fail, not 闂傚倸鍊烽懗鍫曞磻閵娾晛纾块柤鐓庡娴滅椃blish raw goal闂?"""
        from nav.mission.navigation import Navigation
        m = Navigation(planner="astar")
        self.assertFalse(m._allow_direct_goal_fallback)

    def test_explicit_opt_in_still_works(self):
        """Callers can still enable the legacy behaviour deliberately."""
        from nav.mission.navigation import Navigation
        m = Navigation(planner="astar", allow_direct_goal_fallback=True)
        self.assertTrue(m._allow_direct_goal_fallback)


# ---------------------------------------------------------------------------
# W1-4 / W1-5  OccupancyGrid / ESDF 闂?scipy is a hard dependency now
# ---------------------------------------------------------------------------

_scipy_available = True
try:
    import scipy.ndimage  # noqa: F401 -- availability check for skipUnless
except ImportError:
    _scipy_available = False


@unittest.skipUnless(_scipy_available, "scipy not installed in this environment")
class TestW1ScipyHardDependency(unittest.TestCase):
    def test_occupancy_setup_succeeds_with_scipy(self):
        """If scipy is installed the module sets up cleanly."""
        from nav.services.map_layers.occupancy_grid_module import OccupancyGridModule
        og = OccupancyGridModule()
        og.setup()  # must not raise

    def test_esdf_setup_succeeds_with_scipy(self):
        from nav.services.map_layers.esdf_module import ESDFModule
        e = ESDFModule()
        e.setup()

    def test_occupancy_setup_raises_without_scipy(self):
        from nav.services.map_layers.occupancy_grid_module import OccupancyGridModule
        og = OccupancyGridModule()
        with mock.patch.dict("sys.modules", {"scipy.ndimage": None}):
            # mock.patch.dict with value=None makes `import` raise ImportError
            with self.assertRaises(RuntimeError) as ctx:
                og.setup()
            self.assertIn("scipy", str(ctx.exception))


# ---------------------------------------------------------------------------
# W1-6  WaypointTracker 闂?yaw-aware stuck detection
# ---------------------------------------------------------------------------

class TestW1WaypointTrackerYaw(unittest.TestCase):
    def _mk(self):
        from nav.mission.tracking.waypoint_tracker import WaypointTracker
        return WaypointTracker(
            threshold=0.5,
            stuck_timeout=0.2,
            stuck_dist=0.15,
            stuck_yaw_rad=0.35,
        )

    def test_distance_only_legacy_behaviour_preserved(self):
        """Callers that don't supply yaw see the original distance-only logic."""
        import time as _t

        from nav.mission.tracking.waypoint_tracker import EV_STUCK, EV_STUCK_WARN
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]))
        _t.sleep(0.25)
        # First update past timeout fires EV_STUCK_WARN (by design, warn-then-stuck).
        status1 = tracker.update(np.array([0.10, 0.0, 0.0]))  # under stuck_dist
        self.assertEqual(status1.event, EV_STUCK_WARN)
        # Next tick (after warn already sent) fires EV_STUCK.
        status2 = tracker.update(np.array([0.10, 0.0, 0.0]))
        self.assertEqual(status2.event, EV_STUCK)

    def test_yaw_progress_resets_stuck_timer(self):
        """When yaw is tracked, rotation alone counts as progress."""
        import time as _t
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]), robot_yaw=0.0)
        _t.sleep(0.12)
        # Rotate past threshold (~0.35 rad 闂?20闂? without translating
        status = tracker.update(
            np.array([0.02, 0.0, 0.0]),  # below stuck_dist
            robot_yaw=0.6,               # well above stuck_yaw
        )
        # Must be non-stuck 闂?yaw rotation counts as progress
        self.assertIsNone(status.event)

    def test_tiny_spin_with_no_distance_eventually_triggers_stuck(self):
        """Sub-threshold yaw + sub-threshold dist = still stuck after warn."""
        import time as _t

        from nav.mission.tracking.waypoint_tracker import EV_STUCK, EV_STUCK_WARN
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]), robot_yaw=0.0)
        _t.sleep(0.25)
        # First update past timeout fires EV_STUCK_WARN.
        status1 = tracker.update(
            np.array([0.02, 0.0, 0.0]),  # < stuck_dist
            robot_yaw=0.05,              # < stuck_yaw (0.35 rad)
        )
        self.assertEqual(status1.event, EV_STUCK_WARN)
        # Second update after warn fires EV_STUCK.
        status2 = tracker.update(
            np.array([0.02, 0.0, 0.0]),
            robot_yaw=0.05,
        )
        self.assertEqual(status2.event, EV_STUCK)


# ---------------------------------------------------------------------------
# W1-7  GNSS rtcm_age_s 闂?pipe from driver, don't hardcode 99.9
# ---------------------------------------------------------------------------

class TestW1GnssRtcmAge(unittest.TestCase):
    def test_parse_gga_extracts_rtcm_age_when_present(self):
        from localization.gnss_serial_driver import parse_gga
        # GGA with age=1.2 (field 13) and refID=0000
        fields = [
            "$GNGGA", "000000.00",
            "3113.43", "N", "12126.88", "E",
            "4", "12", "0.8",
            "20.3", "M", "0.0", "M",
            "1.2", "0000",
        ]
        parsed = parse_gga(fields)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["rtcm_age_s"], 1.2)

    def test_parse_gga_returns_none_when_age_field_empty(self):
        from localization.gnss_serial_driver import parse_gga
        fields = [
            "$GNGGA", "000000.00",
            "3113.43", "N", "12126.88", "E",
            "1", "8", "1.2",
            "20.3", "M", "0.0", "M",
            "", "",  # age empty
        ]
        parsed = parse_gga(fields)
        self.assertIsNotNone(parsed)
        self.assertIsNone(parsed["rtcm_age_s"])


# ---------------------------------------------------------------------------
# W1-8  MobileCLIPEncoder 闂?fake "try Apple MobileCLIP" path removed
# ---------------------------------------------------------------------------

class TestW1MobileCLIPDeadCodeRemoved(unittest.TestCase):
    def test_try_load_mobileclip_method_removed(self):
        from perception.encoding.mobileclip_encoder import (
            MobileCLIPEncoder,
        )
        self.assertFalse(
            hasattr(MobileCLIPEncoder, "_try_load_mobileclip"),
            "Dead path that tried to load non-existent /mobileclip_s2.pt "
            "must stay removed 闂?it was never reachable in practice.",
        )


# ---------------------------------------------------------------------------
# W1-9  CLIPEncoder 闂?degraded flag is exposed
# ---------------------------------------------------------------------------

class TestW1ClipDegradedFlag(unittest.TestCase):
    def test_is_degraded_property_exists(self):
        from perception.encoding.clip_encoder import CLIPEncoder
        enc = CLIPEncoder.__new__(CLIPEncoder)  # avoid loading the model
        enc._degraded = False
        enc._degraded_reason = ""
        self.assertFalse(enc.is_degraded)
        self.assertEqual(enc.degraded_reason, "")

        enc._degraded = True
        enc._degraded_reason = "GPU OOM"
        self.assertTrue(enc.is_degraded)
        self.assertEqual(enc.degraded_reason, "GPU OOM")


# ---------------------------------------------------------------------------
# W1-10  MockLLMClient 闂?confidence=0 + mock flag
# ---------------------------------------------------------------------------

class TestW1MockLLMSignals(unittest.TestCase):
    def test_mock_response_is_flagged_and_confidence_zero(self):
        from decision.llm.llm_client import (
            LLMConfig,
            MockLLMClient,
        )
        client = MockLLMClient(LLMConfig(backend="mock"))
        raw = asyncio.get_event_loop().run_until_complete(
            client.chat([{"role": "user", "content": "find the kitchen table"}])
        )
        import json as _json
        # MockLLMClient.chat returns a JSON string 闂?parse it
        # (the response dict is serialised somewhere downstream)
        try:
            payload = _json.loads(raw)
        except (_json.JSONDecodeError, TypeError):
            self.fail(f"MockLLM did not return JSON: {raw!r}")
        self.assertTrue(payload.get("mock"),
                        "Mock responses must be tagged with mock=True")
        self.assertEqual(payload.get("confidence"), 0.0,
                         "Mock must report confidence=0 so downstream never "
                         "trusts its output as real inference.")


if __name__ == "__main__":
    unittest.main()
