"""Tests for runtime.utils.scene_mode_detector — indoor/outdoor classification."""

import os
import unittest
from unittest.mock import patch

from runtime.utils.scene_mode_detector import (
    MODE_INDOOR,
    MODE_OUTDOOR,
    MODE_UNKNOWN,
    SceneModeConfig,
    SceneModeDetector,
)


class TestInitialState(unittest.TestCase):
    def test_default_is_unknown(self):
        d = SceneModeDetector()
        self.assertEqual(d.mode, MODE_UNKNOWN)
        self.assertEqual(d.source, "init")

    def test_env_var_indoor_takes_effect(self):
        with patch.dict(os.environ, {"LINGTU_SCENE_MODE": "indoor"}):
            d = SceneModeDetector()
            self.assertEqual(d.mode, MODE_INDOOR)
            self.assertEqual(d.source, "manual")

    def test_env_var_invalid_is_ignored(self):
        with patch.dict(os.environ, {"LINGTU_SCENE_MODE": "garbage"}):
            d = SceneModeDetector()
            self.assertEqual(d.mode, MODE_UNKNOWN)


class TestManualOverride(unittest.TestCase):
    def test_set_indoor_overrides_auto(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=0.0))
        # Two observations needed to seed candidate then confirm.
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.0)
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.1)
        self.assertEqual(d.mode, MODE_OUTDOOR)
        # Manual indoor wins
        d.set_manual_mode(MODE_INDOOR)
        self.assertEqual(d.mode, MODE_INDOOR)
        self.assertEqual(d.source, "manual")

    def test_clear_manual_falls_back_to_auto(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=0.0))
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.0)
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.1)
        d.set_manual_mode(MODE_INDOOR)
        d.set_manual_mode(None)
        self.assertEqual(d.mode, MODE_OUTDOOR)
        self.assertEqual(d.source, "auto")

    def test_invalid_manual_mode_raises(self):
        d = SceneModeDetector()
        with self.assertRaises(ValueError):
            d.set_manual_mode("garbage")


class TestAutoDetection(unittest.TestCase):
    def test_rtk_fixed_with_hold_flips_to_outdoor(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=5.0))
        # Same observation under hold_seconds → still UNKNOWN
        flipped = d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.0)
        self.assertFalse(flipped)
        self.assertEqual(d.mode, MODE_UNKNOWN)
        # 5s later → flips
        flipped = d.observe_gnss("RTK_FIXED", age_s=0.1, now=105.0)
        self.assertTrue(flipped)
        self.assertEqual(d.mode, MODE_OUTDOOR)

    def test_no_fix_flips_to_indoor(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=2.0))
        d.observe_gnss("NO_FIX", age_s=0.1, now=100.0)
        d.observe_gnss("NO_FIX", age_s=0.1, now=103.0)
        self.assertEqual(d.mode, MODE_INDOOR)

    def test_stale_gnss_counts_as_indoor(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=2.0, gnss_max_age_s=2.0))
        # RTK_FIXED but age too high → indoor evidence
        d.observe_gnss("RTK_FIXED", age_s=5.0, now=100.0)
        d.observe_gnss("RTK_FIXED", age_s=5.0, now=103.0)
        self.assertEqual(d.mode, MODE_INDOOR)

    def test_flip_resets_hysteresis(self):
        """Auto flips to OUTDOOR after 5s, then a single NO_FIX restarts the
        hysteresis timer for INDOOR — does not flip immediately."""
        d = SceneModeDetector(SceneModeConfig(hold_seconds=5.0))
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=100.0)
        d.observe_gnss("RTK_FIXED", age_s=0.1, now=106.0)  # OUTDOOR
        self.assertEqual(d.mode, MODE_OUTDOOR)
        # Single NO_FIX — does not flip yet
        flipped = d.observe_gnss("NO_FIX", age_s=0.1, now=107.0)
        self.assertFalse(flipped)
        self.assertEqual(d.mode, MODE_OUTDOOR)
        # 5s of consistent NO_FIX → flip
        flipped = d.observe_gnss("NO_FIX", age_s=0.1, now=112.5)
        self.assertTrue(flipped)
        self.assertEqual(d.mode, MODE_INDOOR)

    def test_observe_no_gnss_is_indoor_evidence(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=2.0))
        d.observe_no_gnss(now=100.0)
        d.observe_no_gnss(now=103.0)
        self.assertEqual(d.mode, MODE_INDOOR)


class TestRequireRtkFlag(unittest.TestCase):
    def test_require_rtk_off_accepts_single_fix(self):
        d = SceneModeDetector(SceneModeConfig(hold_seconds=2.0, require_rtk=False))
        d.observe_gnss("SINGLE", age_s=0.1, now=100.0)
        d.observe_gnss("SINGLE", age_s=0.1, now=103.0)
        # Without require_rtk, SINGLE counts as outdoor; we used the fall-through
        # else branch which classifies anything not NO_FIX / stale as outdoor.
        self.assertEqual(d.mode, MODE_OUTDOOR)


if __name__ == "__main__":
    unittest.main()
