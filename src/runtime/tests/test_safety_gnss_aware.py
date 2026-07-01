"""Tests for GNSS-aware SafetyRing 閳?distinguishes 'GNSS signal lost'
from 'SLAM sensor degeneracy' in the operator dialogue.
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


def _make_safety(**kw):
    from nav.services.safety.safety_ring import SafetyRing
    return SafetyRing(**kw)


class TestGnssAwareSafetyRing(unittest.TestCase):

    def test_gnss_port_declared(self):
        m = _make_safety()
        self.assertIn("gnss_fusion_health", m.ports_in)

    def test_gnss_state_initially_zero(self):
        m = _make_safety()
        self.assertFalse(m._gnss_enabled)
        self.assertFalse(m._gnss_alignment_locked)
        self.assertEqual(m._gnss_last_fix_type, "NONE")

    def test_gnss_health_callback_populates_state(self):
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": True,
            "alignment_locked": True,
            "last_fix_type": "RTK_FIXED",
            "last_gnss_age_s": 0.2,
            "last_residual_m": 0.05,
            "relock_count": 0,
        })
        self.assertTrue(m._gnss_enabled)
        self.assertTrue(m._gnss_alignment_locked)
        self.assertEqual(m._gnss_last_fix_type, "RTK_FIXED")
        self.assertAlmostEqual(m._gnss_age_s, 0.2)

    def test_root_cause_slam_weak_when_gnss_healthy(self):
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": True, "alignment_locked": True,
            "last_fix_type": "RTK_FIXED", "last_gnss_age_s": 0.1,
        })
        m._loc_state = "DEGRADED"
        m._loc_confidence = 0.5
        self.assertEqual(m._degraded_root_cause(), "slam_weak")

    def test_root_cause_gnss_lost_when_rtk_absent(self):
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": True, "alignment_locked": True,
            "last_fix_type": "SINGLE",
            "last_gnss_age_s": 10.0,  # stale
        })
        m._loc_state = "DEGRADED"
        m._loc_confidence = 0.5   # SLAM still ok
        self.assertEqual(m._degraded_root_cause(), "gnss_lost")

    def test_root_cause_both_when_slam_and_gnss_bad(self):
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": True, "alignment_locked": True,
            "last_fix_type": "SINGLE", "last_gnss_age_s": 30.0,
        })
        m._loc_state = "DEGRADED"
        m._loc_confidence = 0.1   # SLAM also weak
        self.assertEqual(m._degraded_root_cause(), "both")

    def test_root_cause_slam_weak_when_gnss_disabled(self):
        """When GNSS fusion is disabled entirely, we must not blame GNSS."""
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": False, "alignment_locked": False,
            "last_fix_type": "NONE", "last_gnss_age_s": float("inf"),
        })
        m._loc_state = "DEGRADED"
        self.assertEqual(m._degraded_root_cause(), "slam_weak")

    def test_dialogue_tagging_when_localization_healthy(self):
        """Healthy state should NOT pollute dialogue with GNSS root cause."""
        m = _make_safety()
        m.setup()
        received: list[dict] = []
        m.dialogue_state._add_callback(received.append)

        m._loc_state = "TRACKING"
        m._publish_dialogue()

        self.assertEqual(len(received), 1)
        payload = received[-1]
        self.assertNotIn("loc_root_cause", payload)

    def test_dialogue_tagging_when_degraded(self):
        m = _make_safety()
        m.setup()
        m._on_gnss_fusion_health({
            "enabled": True, "alignment_locked": True,
            "last_fix_type": "SINGLE", "last_gnss_age_s": 5.0,
        })
        received: list[dict] = []
        m.dialogue_state._add_callback(received.append)

        m._loc_state = "DEGRADED"
        m._loc_confidence = 0.4
        m._publish_dialogue()

        payload = received[-1]
        self.assertIn("loc_root_cause", payload)
        self.assertIn(payload["loc_root_cause"], ("slam_weak", "gnss_lost", "both"))
        self.assertEqual(payload["gnss_fix_type"], "SINGLE")
        self.assertAlmostEqual(payload["gnss_age_s"], 5.0)


if __name__ == "__main__":
    unittest.main()
