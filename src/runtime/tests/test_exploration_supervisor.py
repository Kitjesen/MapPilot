"""ExplorationSupervisorModule behavioural tests.

Covers:
  - initial mode = "uninit"
  - healthy → fires exploration_ready exactly once
  - waypoint silence > warn_timeout_s → "degraded"
  - waypoint silence > fallback_timeout_s → "fallback", flag raised
  - TARE finished=True → "finished"
  - recovery clears fallback_requested
  - clear_exploration_fallback skill resets state
"""
from __future__ import annotations

import json
import unittest


def _make_supervisor(warn: float = 20.0, fallback: float = 60.0):
    from explore.tare.supervisor import (
        ExplorationSupervisorModule,
    )
    m = ExplorationSupervisorModule(
        warn_timeout_s=warn,
        fallback_timeout_s=fallback,
    )
    # setup() only subscribes; we feed tare_stats via the port directly.
    m.setup()
    return m


def _collect_supervisor_states(sup) -> list:
    """Subscribe to supervisor_state and return the buffer list."""
    captured: list[dict] = []
    sup.supervisor_state.subscribe(captured.append)
    return captured


def _collect_ready_events(sup) -> list:
    captured: list[bool] = []
    sup.exploration_ready.subscribe(captured.append)
    return captured


class TestInitialState(unittest.TestCase):

    def test_mode_uninit_before_any_stats(self):
        sup = _make_supervisor()
        states = _collect_supervisor_states(sup)
        sup._evaluate_and_publish()
        self.assertEqual(len(states), 1)
        self.assertEqual(states[0]["mode"], "uninit")
        self.assertIsNone(states[0]["waypoint_age_s"])
        self.assertFalse(states[0]["ready_fired"])


class TestReadySignal(unittest.TestCase):

    def test_first_healthy_fires_exploration_ready(self):
        sup = _make_supervisor()
        readies = _collect_ready_events(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": True,
            "waypoint_age_s": 0.5, "waypoint_count": 1,
        })
        self.assertEqual(readies, [True])

    def test_ready_fires_only_once(self):
        sup = _make_supervisor()
        readies = _collect_ready_events(sup)
        for _ in range(5):
            sup._on_tare_stats({
                "alive": True, "healthy": True,
                "waypoint_age_s": 0.5, "waypoint_count": 1,
            })
        self.assertEqual(readies, [True])  # one entry total

    def test_ready_not_fired_on_unhealthy(self):
        sup = _make_supervisor()
        readies = _collect_ready_events(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": False,
            "waypoint_age_s": 50.0,
        })
        self.assertEqual(readies, [])


class TestDegradedMode(unittest.TestCase):

    def test_waypoint_silence_triggers_degraded(self):
        sup = _make_supervisor(warn=5.0, fallback=60.0)
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": False,
            "waypoint_age_s": 10.0,  # above warn, below fallback
        })
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "degraded")
        self.assertIn("10.0s", states[-1]["reason"])
        self.assertFalse(states[-1]["fallback_requested"])


class TestFallbackMode(unittest.TestCase):

    def test_long_silence_raises_fallback(self):
        sup = _make_supervisor(warn=5.0, fallback=30.0)
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": False,
            "waypoint_age_s": 90.0,
        })
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "fallback")
        self.assertTrue(states[-1]["fallback_requested"])

    def test_recovery_clears_fallback(self):
        sup = _make_supervisor(warn=5.0, fallback=30.0)
        _ = _collect_supervisor_states(sup)
        # Trigger fallback first
        sup._on_tare_stats({
            "alive": True, "healthy": False, "waypoint_age_s": 100.0,
        })
        sup._evaluate_and_publish()
        self.assertTrue(sup._fallback_requested)
        # Now TARE recovers
        sup._on_tare_stats({
            "alive": True, "healthy": True, "waypoint_age_s": 0.2,
        })
        sup._evaluate_and_publish()
        self.assertFalse(sup._fallback_requested)


class TestFinishedMode(unittest.TestCase):

    def test_finished_flag_propagates(self):
        sup = _make_supervisor()
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": True,
            "waypoint_age_s": 1.0, "finished": True,
        })
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "finished")


class TestStartingMode(unittest.TestCase):

    def test_alive_without_waypoint_is_starting(self):
        sup = _make_supervisor()
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": False,
            "waypoint_age_s": float("inf"),
        })
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "starting")

    def test_none_waypoint_age_is_starting(self):
        sup = _make_supervisor()
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({
            "alive": True, "healthy": False,
            "waypoint_age_s": None,
        })
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "starting")


class TestUninitMode(unittest.TestCase):

    def test_tare_not_alive_reports_uninit(self):
        sup = _make_supervisor()
        states = _collect_supervisor_states(sup)
        sup._on_tare_stats({"alive": False})
        sup._evaluate_and_publish()
        self.assertEqual(states[-1]["mode"], "uninit")
        self.assertIn("not alive", states[-1]["reason"])


class TestSkills(unittest.TestCase):

    def test_get_exploration_supervisor_returns_json(self):
        sup = _make_supervisor()
        sup._on_tare_stats({
            "alive": True, "healthy": True, "waypoint_age_s": 0.1,
        })
        raw = sup.get_exploration_supervisor()
        payload = json.loads(raw)
        self.assertIn("mode", payload)
        self.assertIn("ready_fired", payload)

    def test_clear_exploration_fallback_resets_state(self):
        sup = _make_supervisor(warn=5.0, fallback=30.0)
        sup._on_tare_stats({
            "alive": True, "healthy": False, "waypoint_age_s": 100.0,
        })
        sup._evaluate_and_publish()
        self.assertTrue(sup._fallback_requested)
        payload = json.loads(sup.clear_exploration_fallback())
        self.assertEqual(payload, {"status": "cleared"})
        self.assertFalse(sup._fallback_requested)
        self.assertEqual(sup._degraded_ticks, 0)


if __name__ == "__main__":
    unittest.main()
