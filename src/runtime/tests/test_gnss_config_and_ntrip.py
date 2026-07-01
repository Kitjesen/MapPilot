"""Tests for typed GnssConfig + NtripClientModule.

Covers:
  - GnssConfig dataclass loads nested antenna_offset/quality/fusion
  - validate_config catches bad gnss.fusion values when enabled
  - NtripClientModule: disabled mode is inert; GGA sentence formatting;
    @skill discovery; status snapshot schema
"""

from __future__ import annotations

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))


# â”€â”€â”€ GnssConfig â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestGnssConfigDataclass(unittest.TestCase):

    def test_defaults(self):
        from runtime.config import GnssConfig
        c = GnssConfig()
        self.assertFalse(c.enabled)
        self.assertEqual(c.antenna_offset.x, 0.0)
        self.assertEqual(c.antenna_offset.z, 0.0)
        self.assertEqual(c.quality.min_sat_used, 8)
        self.assertAlmostEqual(c.fusion.alpha_healthy, 0.05)
        self.assertAlmostEqual(c.fusion.residual_warn_m, 5.0)

    def test_fill_from_raw_yaml_dict(self):
        from runtime.config import _fill_gnss_config
        raw = {
            "enabled": True,
            "model": "TestRx",
            "antenna_offset": {"x": 0.1, "y": 0.2, "z": 0.5},
            "quality": {"min_sat_used": 6, "max_hdop": 3.0},
            "fusion": {
                "alpha_healthy": 0.1,
                "alpha_degraded": 0.8,
                "residual_warn_m": 3.0,
            },
        }
        c = _fill_gnss_config(raw)
        self.assertTrue(c.enabled)
        self.assertEqual(c.model, "TestRx")
        self.assertAlmostEqual(c.antenna_offset.x, 0.1)
        self.assertAlmostEqual(c.antenna_offset.z, 0.5)
        self.assertEqual(c.quality.min_sat_used, 6)
        self.assertAlmostEqual(c.fusion.alpha_healthy, 0.1)

    def test_missing_sections_fill_with_defaults(self):
        from runtime.config import _fill_gnss_config
        c = _fill_gnss_config({"enabled": True})
        self.assertEqual(c.antenna_offset.z, 0.0)
        self.assertEqual(c.quality.min_sat_used, 8)
        self.assertAlmostEqual(c.fusion.alpha_healthy, 0.05)

    def test_unknown_keys_ignored(self):
        from runtime.config import _fill_gnss_config
        c = _fill_gnss_config({
            "enabled": True,
            "bogus_key": "unused",
            "quality": {"min_sat_used": 10, "not_a_real_key": 7},
        })
        self.assertTrue(c.enabled)
        self.assertEqual(c.quality.min_sat_used, 10)

    def test_validate_bad_alpha_reports_error(self):
        from runtime.config import GnssConfig, RobotConfig, validate_config
        c = RobotConfig()
        c.gnss = GnssConfig()
        c.gnss.enabled = True
        c.gnss.fusion.alpha_healthy = 1.5
        errs = validate_config(c)
        self.assertTrue(
            any("alpha_healthy" in e for e in errs),
            f"expected alpha_healthy error, got {errs}"
        )

    def test_validate_bad_residual_reports_error(self):
        from runtime.config import GnssConfig, RobotConfig, validate_config
        c = RobotConfig()
        c.gnss = GnssConfig()
        c.gnss.enabled = True
        c.gnss.fusion.residual_warn_m = -1.0
        errs = validate_config(c)
        self.assertTrue(
            any("residual_warn_m" in e for e in errs),
            f"expected residual_warn_m error, got {errs}"
        )

    def test_validate_disabled_gnss_skips_checks(self):
        """Robots without GNSS must not trip on default 0s."""
        from runtime.config import GnssConfig, RobotConfig, validate_config
        c = RobotConfig()
        c.gnss = GnssConfig()      # enabled=False
        c.gnss.fusion.alpha_healthy = 99.0  # bad, but gated out
        errs = validate_config(c)
        self.assertFalse(any("alpha_healthy" in e for e in errs))


# â”€â”€â”€ NtripClientModule â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestNtripClient(unittest.TestCase):

    def _make(self, **kw):
        from localization.ntrip_client_module import NtripClientModule
        return NtripClientModule(**kw)

    def test_defaults_are_inert(self):
        m = self._make()
        self.assertFalse(m._enabled)

    def test_enabled_requires_host_and_mount(self):
        # Flag on but missing mount â†?still disabled
        m = self._make(enabled=True, host="caster.example.com")
        self.assertFalse(m._enabled)
        # Flag on but missing host â†?still disabled
        m = self._make(enabled=True, mount="RTCM32")
        self.assertFalse(m._enabled)

    def test_full_config_enables(self):
        m = self._make(enabled=True, host="caster.example.com",
                       mount="RTCM32", user="u", password="p")
        self.assertTrue(m._enabled)

    def test_disabled_start_does_not_spawn_threads(self):
        m = self._make()
        m.setup()
        m.start()
        try:
            self.assertIsNone(m._thread)
            self.assertIsNone(m._status_thread)
        finally:
            m.stop()

    def test_format_gga_returns_valid_nmea(self):
        from runtime.msgs.gnss import GnssFix, GnssFixType
        from localization.ntrip_client_module import _format_gga
        fix = GnssFix(
            lat=31.0, lon=121.0, alt=10.0,
            fix_type=GnssFixType.RTK_FIXED,
            num_sat=12, num_sat_used=12,
            ts=time.time(),
        )
        gga = _format_gga(fix)
        self.assertTrue(gga.startswith(b"$GPGGA"))
        self.assertTrue(gga.endswith(b"\r\n"))
        # Checksum delimiter present
        self.assertIn(b"*", gga)

    def test_skills_are_discoverable(self):
        m = self._make()
        names = {info.func_name for info in m.get_skill_infos()}
        self.assertIn("get_ntrip_status", names)

    def test_status_snapshot_keys(self):
        import json
        m = self._make(enabled=True, host="x", mount="y")
        snap = json.loads(m.get_ntrip_status())
        for key in ("enabled", "connected", "host", "mount",
                    "attempts", "failures", "rtcm_bytes_rx",
                    "gga_sent", "last_error"):
            self.assertIn(key, snap)


# â”€â”€â”€ GnssModule â†?NTRIP wiring â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class TestNtripRtcmAutoWire(unittest.TestCase):
    """The rtcm_bytes port should auto-connect GnssModule â†?NtripClientModule.
    Inject bytes manually to simulate an NTRIP session without network."""

    def test_gnss_module_subscribes_to_rtcm_bytes(self):
        from localization.gnss_module import GnssModule
        m = GnssModule(auto_init_origin=False, status_rate_hz=0.0)
        m.setup()
        # Even in stub mode the subscription is installed so publish works
        self.assertTrue(hasattr(m, "_on_rtcm_bytes"))

    def test_rtcm_forward_to_serial_no_crash_when_stub(self):
        from localization.gnss_module import GnssModule
        m = GnssModule(auto_init_origin=False, status_rate_hz=0.0)
        m.setup()
        # No serial driver; write must be a safe no-op
        m._on_rtcm_bytes(b"\xd3\x00\x04\x4c\xe0\x00\x80\x00")
        # If we didn't crash, the contract holds


if __name__ == "__main__":
    unittest.main()
