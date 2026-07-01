"""Tests for localization health monitoring (P0 safety).

Tests cover:
  - SlamBridgeModule watchdog: UNINIT 闁?TRACKING 闁?LOST 闁?recovery
  - SafetyRing: localization status 闁?stop_cmd
  - Navigation: pause/resume on localization loss

TODO: Migrate to event-driven waits (threading.Event / asyncio) to
      eliminate time.sleep polling. Most module.start()->status waits
      can be replaced with callback-based synchronization for faster,
      more deterministic tests.
"""

import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.ros2]

# Ensure src/ is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import Odometry

# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# SlamBridgeModule watchdog tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestSlamBridgeWatchdog(unittest.TestCase):

    def _make(self, **kw):
        from localization.bridge import SlamBridgeModule
        defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
        defaults.update(kw)
        return SlamBridgeModule(**defaults)

    def test_initial_state_is_uninit(self):
        m = self._make()
        self.assertEqual(m._loc_state, "UNINIT")

    def test_ports_declared(self):
        m = self._make()
        self.assertIn("localization_status", m.ports_out)
        self.assertIn("odometry", m.ports_out)
        self.assertIn("map_cloud", m.ports_out)

    def test_watchdog_publishes_uninit_initially(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        time.sleep(0.02)
        m.stop()
        self.assertTrue(len(received) > 0)
        self.assertEqual(received[0]["state"], "UNINIT")
        self.assertAlmostEqual(received[0]["confidence"], 0.0)

    def test_odom_arrival_sets_tracking(self):
        m = self._make(odom_timeout=0.5, cloud_timeout=0.5)
        received = []
        m.localization_status._add_callback(received.append)
        m._mark_odom_received()
        m._mark_cloud_received()
        m.start()
        time.sleep(0.025)
        m.stop()
        # Should have transitioned to TRACKING
        states = [r["state"] for r in received]
        self.assertIn("TRACKING", states)

    def test_super_lio_status_uses_odom_map_cloud_contract(self):
        m = self._make(backend_profile="super_lio", odom_timeout=0.5, cloud_timeout=0.5)
        received = []
        m.localization_status._add_callback(received.append)
        m._mark_odom_received()
        m._mark_cloud_received()
        m.start()
        time.sleep(0.025)
        m.stop()

        tracking = next(r for r in received if r["state"] == "TRACKING")
        self.assertEqual(tracking["backend"], "super_lio")
        self.assertEqual(tracking["health_source"], "odom_map_cloud")
        self.assertEqual(tracking["localizer_health"], "LIO_TRACKING")
        self.assertTrue(tracking["pose_fresh"])
        self.assertTrue(tracking["map_cloud_fresh"])
        self.assertEqual(tracking["map_state"], "live_map_cloud")
        self.assertTrue(tracking["map_save_supported"])
        self.assertEqual(tracking["map_save_source"], "live_map_cloud_snapshot")
        self.assertFalse(tracking["relocalization_supported"])
        self.assertFalse(tracking["saved_map_relocalization_supported"])
        self.assertTrue(tracking["restart_recovery_supported"])
        self.assertEqual(tracking["recovery_method"], "restart_super_lio")
        self.assertEqual(tracking["relocalization_state"], "unsupported")
        self.assertEqual(tracking["recovery_action"], "none")

    def test_watchdog_uses_running_localizer_contract(self):
        m = self._make(backend_profile="bridge", odom_timeout=0.5, cloud_timeout=0.5)
        m._current_backend_profile = lambda: "localizer"
        m._relocalization_state = "unsupported"
        received = []
        m.localization_status._add_callback(received.append)
        m._mark_odom_received()
        m._mark_cloud_received()
        m.start()
        time.sleep(0.025)
        m.stop()

        tracking = next(r for r in received if r["state"] == "TRACKING")
        self.assertEqual(tracking["backend"], "localizer")
        self.assertFalse(tracking["map_save_supported"])
        self.assertEqual(tracking["map_save_source"], "active_map")
        self.assertTrue(tracking["relocalization_supported"])
        self.assertTrue(tracking["saved_map_relocalization_supported"])
        self.assertTrue(tracking["restart_recovery_supported"])
        self.assertEqual(tracking["recovery_method"], "relocalize_service")
        self.assertEqual(tracking["relocalization_state"], "idle")
        self.assertEqual(tracking["recovery_action"], "none")

    def test_fresh_localizer_health_topic_overrides_synthesized_lio_health(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(backend_profile="super_lio", odom_timeout=0.5, cloud_timeout=0.5)
        received = []
        m.localization_status._add_callback(received.append)
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received()
        m._mark_cloud_received()
        m.start()
        time.sleep(0.025)
        m.stop()

        tracking = next(r for r in received if r["state"] == "TRACKING")
        self.assertEqual(tracking["health_source"], "localizer_health_topic")
        self.assertEqual(tracking["localizer_health"], "LOCKED")
        self.assertEqual(tracking["localizer_health_raw"], "LOCKED")
        self.assertEqual(tracking["localizer_health_source"], "localizer_health_topic")
        self.assertIsNotNone(tracking["localizer_health_topic_age_ms"])

    @pytest.mark.ros2
    def test_ros2_shaped_odom_cloud_and_health_topic_drive_bridge_status(self):
        import numpy as np

        def ns(**kw):
            return SimpleNamespace(**kw)

        def odom_msg():
            pose = ns(
                position=ns(x=1.25, y=-0.5, z=0.1),
                orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            twist = ns(
                linear=ns(x=0.1, y=0.0, z=0.0),
                angular=ns(x=0.0, y=0.0, z=0.02),
            )
            cov = [0.0] * 36
            cov[0] = 0.01
            cov[7] = 0.01
            cov[14] = 0.02
            return ns(
                header=ns(frame_id="odom"),
                child_frame_id="base_link",
                pose=ns(pose=pose, covariance=cov),
                twist=ns(twist=twist),
            )

        def cloud_msg():
            pts = np.asarray(
                [
                    [0.0, 0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.1, 1.0],
                    [0.0, 1.0, 0.2, 1.0],
                ],
                dtype=np.float32,
            )
            return ns(
                width=pts.shape[0],
                height=1,
                point_step=16,
                data=pts.tobytes(),
            )

        class HealthMsg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.5,
            cloud_timeout=0.5,
            localizer_health_timeout=0.5,
            watchdog_hz=50,
        )
        odom_seen = []
        cloud_seen = []
        status_seen = []
        m.odometry._add_callback(odom_seen.append)
        m.map_cloud._add_callback(cloud_seen.append)
        m.localization_status._add_callback(status_seen.append)

        m._on_rclpy_localization_health(HealthMsg())
        m._on_rclpy_odom(odom_msg())
        if m._odom_worker_thread is not None:
            m._odom_worker_thread.join(timeout=1.0)
        m._process_rclpy_cloud(cloud_msg())

        m.start()
        deadline = time.time() + 1.0
        try:
            while time.time() < deadline and not any(
                item.get("state") == "TRACKING" for item in status_seen
            ):
                time.sleep(0.006)
        finally:
            m.stop()

        self.assertTrue(odom_seen)
        self.assertEqual(odom_seen[-1].frame_id, "odom")
        self.assertEqual(odom_seen[-1].child_frame_id, "base_link")
        self.assertTrue(cloud_seen)
        self.assertEqual(cloud_seen[-1].frame_id, "odom")
        self.assertEqual(cloud_seen[-1].num_points, 3)
        tracking = next(item for item in status_seen if item["state"] == "TRACKING")
        self.assertEqual(tracking["backend"], "localizer")
        self.assertEqual(tracking["health_source"], "localizer_health_topic")
        self.assertEqual(tracking["localizer_health"], "LOCKED")
        self.assertEqual(tracking["localizer_health_raw"], "LOCKED")
        self.assertEqual(tracking["localizer_health_source"], "localizer_health_topic")
        self.assertEqual(tracking["localizer_health_iter"], 4)
        self.assertAlmostEqual(tracking["localizer_health_cov_trace"], 0.01, places=4)
        self.assertTrue(tracking["pose_fresh"])
        self.assertTrue(tracking["map_cloud_fresh"])
        self.assertEqual(tracking["map_state"], "live_map_cloud")
        self.assertIsNotNone(tracking["localizer_health_topic_age_ms"])

    def test_fresh_locked_localizer_health_graces_short_odom_gap(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.2,
            cloud_timeout=0.5,
            localizer_health_timeout=0.5,
            localizer_health_odom_grace_s=2.0,
            watchdog_hz=50,
        )
        received = []
        m.localization_status._add_callback(received.append)
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 0.8,
            mono_now=time.monotonic() - 0.8,
        )
        m._mark_cloud_received()
        m.start()
        time.sleep(0.02)
        m.stop()

        tracking = next(r for r in received if r["state"] == "TRACKING")
        self.assertTrue(tracking["localizer_health_grace"])
        self.assertEqual(tracking["health_source"], "localizer_health_topic")
        self.assertEqual(tracking["localizer_health"], "LOCKED")
        self.assertEqual(tracking["odom_timeout_ms"], 2000.0)
        self.assertTrue(tracking["pose_fresh"])

    def test_locked_localizer_health_restarts_chain_after_long_odom_gap(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=0.08,
            localizer_odom_loss_recovery_cooldown_s=60.0,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()

        def fake_restart():
            called.append(time.time())
            restarted.set()
            return True

        m._restart_localization_chain_for_recovery = fake_restart
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 0.2,
            mono_now=time.monotonic() - 0.2,
        )
        m._mark_cloud_received()

        m.start()
        self.assertTrue(restarted.wait(timeout=0.6))
        time.sleep(0.02)
        m.stop()

        self.assertEqual(len(called), 1)
        self.assertEqual(m._last_recovery_signal, "ODOM_MISSING")
        self.assertEqual(m._last_recovery_action, "restart_localization_chain")

    def test_locked_localizer_health_short_odom_gap_does_not_restart_chain(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=1.0,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()
        m._restart_localization_chain_for_recovery = (
            lambda: called.append(time.time()) or restarted.set() or True
        )
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 0.1,
            mono_now=time.monotonic() - 0.1,
        )
        m._mark_cloud_received()

        m.start()
        self.assertFalse(restarted.wait(timeout=0.2))
        m.stop()

        self.assertEqual(called, [])
        self.assertEqual(m._last_recovery_signal, "NONE")

    def test_localizer_odom_loss_recovery_waits_for_startup_grace(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=1.0,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()
        m._restart_localization_chain_for_recovery = (
            lambda: called.append(time.time()) or restarted.set() or True
        )
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 10.0,
            mono_now=time.monotonic() - 10.0,
        )
        m._mark_cloud_received()

        m.start()
        self.assertFalse(restarted.wait(timeout=0.2))
        m.stop()

        self.assertEqual(called, [])
        self.assertEqual(m._last_recovery_signal, "NONE")

    def test_localizer_odom_loss_recovery_requires_prior_odom(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=0.05,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()
        m._restart_localization_chain_for_recovery = (
            lambda: called.append(time.time()) or restarted.set() or True
        )
        m._watchdog_start_mono = time.monotonic() - 10.0
        m._on_rclpy_localization_health(Msg())
        m._mark_cloud_received()

        m.start()
        self.assertFalse(restarted.wait(timeout=0.2))
        m.stop()

        self.assertEqual(called, [])
        self.assertEqual(m._last_recovery_signal, "NONE")

    def test_localizer_odom_loss_recovery_cooldown_prevents_repeat_restart(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=0.05,
            localizer_odom_loss_recovery_cooldown_s=60.0,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()

        def fake_restart():
            called.append(time.time())
            restarted.set()
            return True

        m._restart_localization_chain_for_recovery = fake_restart
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 0.2,
            mono_now=time.monotonic() - 0.2,
        )
        m._mark_cloud_received()

        m.start()
        self.assertTrue(restarted.wait(timeout=0.6))
        time.sleep(0.02)
        m.stop()

        self.assertEqual(len(called), 1)

    def test_non_localizer_backend_does_not_run_localizer_odom_loss_recovery(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="super_lio",
            odom_timeout=0.05,
            cloud_timeout=0.5,
            localizer_health_timeout=1.0,
            localizer_health_odom_grace_s=2.0,
            localizer_odom_loss_recovery_s=0.05,
            reconnect_timeout=999.0,
            max_reconnects=0,
            watchdog_hz=100,
        )
        called = []
        restarted = threading.Event()
        m._restart_localization_chain_for_recovery = (
            lambda: called.append(time.time()) or restarted.set() or True
        )
        m._on_rclpy_localization_health(Msg())
        m._mark_odom_received(
            wall_now=time.time() - 0.2,
            mono_now=time.monotonic() - 0.2,
        )
        m._mark_cloud_received()

        m.start()
        self.assertFalse(restarted.wait(timeout=0.2))
        m.stop()

        self.assertEqual(called, [])

    def test_localizer_drift_recovery_uses_full_localization_chain_restart(self):
        m = self._make(backend_profile="localizer")
        called = []
        m._restart_localization_chain_for_recovery = (
            lambda: called.append("chain") or True
        )

        self.assertTrue(m._restart_backend_for_recovery("localizer"))
        self.assertEqual(called, ["chain"])

    def test_stale_localizer_health_does_not_grace_odom_gap(self):
        class Msg:
            data = "LOCKED|fitness=0.023|iter=4|cov=0.01"

        m = self._make(
            backend_profile="localizer",
            odom_timeout=0.2,
            cloud_timeout=0.5,
            localizer_health_timeout=0.2,
            localizer_health_odom_grace_s=2.0,
            watchdog_hz=50,
        )
        received = []
        m.localization_status._add_callback(received.append)
        m._on_rclpy_localization_health(Msg())
        m._localizer_health_ts = time.time() - 1.0
        m._mark_odom_received(
            wall_now=time.time() - 0.8,
            mono_now=time.monotonic() - 0.8,
        )
        m._mark_cloud_received()
        m.start()
        time.sleep(0.02)
        m.stop()

        lost = next(r for r in received if r["state"] == "LOST")
        self.assertFalse(lost["localizer_health_grace"])
        self.assertEqual(lost["health_source"], "odom_map_cloud")
        self.assertFalse(lost["pose_fresh"])

    def test_odom_timeout_sets_lost(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # Simulate odom arrival then timeout
        m._mark_odom_received()
        m._mark_cloud_received()
        time.sleep(0.02)  # Within timeout 闁?should be TRACKING
        # Now let odom age past timeout (0.1s)
        m._last_odom_time = time.time() - 0.2  # Artificially stale
        m._last_odom_mono = 0.0
        time.sleep(0.05)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("LOST", states)

    def test_cloud_timeout_sets_degraded(self):
        m = self._make(odom_timeout=0.5, cloud_timeout=0.05)
        received = []
        m.localization_status._add_callback(received.append)
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        time.sleep(0.025)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("DEGRADED", states)

    def test_recovery_from_lost(self):
        m = self._make(reconnect_timeout=999.0, max_reconnects=0)
        received = []
        m.localization_status._add_callback(received.append)
        # Go to LOST
        m._last_odom_time = time.time() - 0.5
        m._last_cloud_time = time.time() - 0.5
        m._last_odom_mono = 0.0
        m._last_cloud_mono = 0.0
        m.start()
        time.sleep(0.02)
        # Recover
        for _ in range(10):
            m._mark_odom_received()
            m._mark_cloud_received()
            time.sleep(0.006)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("LOST", states)
        # After recovery, should see TRACKING
        lost_idx = max(i for i, s in enumerate(states) if s == "LOST")
        remaining = states[lost_idx + 1:]
        self.assertIn("TRACKING", remaining)

    def test_health_report_includes_localization(self):
        m = self._make()
        h = m.health()
        self.assertIn("localization", h)
        self.assertEqual(h["localization"]["state"], "UNINIT")

    def test_stop_is_idempotent(self):
        m = self._make()
        m.start()
        m.stop()
        m.stop()  # Should not raise


class TestSlamBridgeFallbackTransition(unittest.TestCase):
    """LOC_DEGRADED 闁?LOC_FALLBACK_GNSS_ONLY promotion when GNSS is healthy.

    Wires the state machine half. The actual GNSS+IMU dead-reckoning takeover
    lands in S2.5 once fault-injection tooling exists; these tests lock in
    that the transition fires under the right preconditions and reverts on
    SLAM recovery.
    """

    def _make(self, **kw):
        from localization.bridge import SlamBridgeModule
        defaults = {
            "odom_timeout": 1.0,
            "cloud_timeout": 0.05,
            "watchdog_hz": 50,
            "fallback_after_degraded_s": 0.05,  # short for unit test
            "gnss_max_age_for_fallback_s": 1.0,
            "reconnect_timeout": 999.0,
            "max_reconnects": 0,
        }
        defaults.update(kw)
        return SlamBridgeModule(**defaults)

    def _stub_healthy_gnss(self, m):
        """Plant a fresh RTK_FIXED GnssOdom directly so the watchdog sees
        GNSS as healthy without spinning up the GNSS subscription path."""
        from runtime.msgs.gnss import GnssFixType, GnssOdom
        m._last_gnss_odom = GnssOdom(
            east=0.0, north=0.0, up=0.0, ve=0.0, vn=0.0, vu=0.0,
            cov_e=0.01, cov_n=0.01, cov_u=0.04,
            fix_type=GnssFixType.RTK_FIXED, ts=time.time(),
        )
        m._last_gnss_rx_ts = time.time()

    def _wait_for_state(
        self,
        m,
        received,
        state,
        *,
        timeout=2.0,
        refresh_gnss=False,
    ):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if any(r["state"] == state for r in received):
                return True
            if refresh_gnss:
                self._stub_healthy_gnss(m)
            time.sleep(0.006)
        return any(r["state"] == state for r in received)

    def test_degraded_promotes_to_fallback_when_gnss_healthy(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        # Force DEGRADED by leaving cloud stale, odom fresh.
        self._stub_healthy_gnss(m)
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        try:
            seen = self._wait_for_state(
                m, received, "FALLBACK_GNSS_ONLY", refresh_gnss=True
            )
            states = [r["state"] for r in received]
        finally:
            m.stop()
        self.assertTrue(
            seen,
            f"Expected fallback transition, saw: {set(states)}",
        )

    def test_degraded_stays_degraded_without_gnss(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        # No GNSS planted 闁?fallback gate must keep us at DEGRADED.
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        time.sleep(0.02)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("DEGRADED", states)
        self.assertNotIn("FALLBACK_GNSS_ONLY", states)

    def test_degraded_stays_degraded_with_stale_gnss(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        # GNSS rx > gnss_max_age_for_fallback_s 闁?fallback gate closed.
        self._stub_healthy_gnss(m)
        m._last_gnss_rx_ts = time.time() - 5.0  # stale
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        time.sleep(0.02)
        m.stop()
        states = [r["state"] for r in received]
        self.assertNotIn("FALLBACK_GNSS_ONLY", states)

    def test_degraded_stays_degraded_when_only_single_fix(self):
        """Single-point GNSS isn't precise enough to anchor on."""
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        from runtime.msgs.gnss import GnssFixType, GnssOdom
        m._last_gnss_odom = GnssOdom(
            east=0.0, north=0.0, up=0.0, ve=0.0, vn=0.0, vu=0.0,
            cov_e=0.5, cov_n=0.5, cov_u=1.0,
            fix_type=GnssFixType.SINGLE, ts=time.time(),
        )
        m._last_gnss_rx_ts = time.time()
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        time.sleep(0.02)
        m.stop()
        states = [r["state"] for r in received]
        self.assertNotIn("FALLBACK_GNSS_ONLY", states)

    def test_recovery_from_fallback_back_to_tracking(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        self._stub_healthy_gnss(m)
        m._mark_odom_received()
        m._last_cloud_time = time.time() - 0.5
        m._last_cloud_mono = 0.0
        m.start()
        self._wait_for_state(
            m, received, "FALLBACK_GNSS_ONLY", refresh_gnss=True
        )
        time.sleep(0.05)  # enter FALLBACK
        # Now restore both odom and cloud freshness 闁?keep them fresh while we
        # observe recovery so the watchdog does not race us into LOST.
        for _ in range(20):
            m._mark_odom_received()
            m._mark_cloud_received()
            time.sleep(0.006)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("FALLBACK_GNSS_ONLY", states)
        # After recovery, TRACKING should appear after the FALLBACK index.
        fb_idx = max(i for i, s in enumerate(states) if s == "FALLBACK_GNSS_ONLY")
        self.assertIn("TRACKING", states[fb_idx + 1:])


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# SlamBridgeModule degeneracy parsing + covariance tracking
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestSlamBridgeDegeneracyParsing(unittest.TestCase):
    """Locks in the index ordering for the /slam/degeneracy_detail 11-float
    Float32MultiArray published by FastLIO2 lio_node.cpp.

    Regression guard: a previous version of _on_rclpy_degeneracy_detail read
    d[0] as effective_ratio and d[1] as condition_number, but the C++ publisher
    emits [cond_number, min_eig, max_eig, eff_ratio, degen_count, mask*6].
    """

    def _make(self):
        from localization.bridge import SlamBridgeModule
        return SlamBridgeModule(watchdog_hz=100)

    def _make_msg(self, cond, min_eig, max_eig, eff_ratio, degen_count, mask):
        """Build a stand-in for the Float32MultiArray ROS message."""
        class _M:
            pass
        msg = _M()
        msg.data = [float(cond), float(min_eig), float(max_eig),
                    float(eff_ratio), float(degen_count)] + [float(x) for x in mask]
        return msg

    def test_degeneracy_detail_index_order(self):
        """d[0]=cond, d[3]=effective_ratio, d[4]=degen_count, d[5:11]=mask."""
        m = self._make()
        msg = self._make_msg(
            cond=123.4, min_eig=0.01, max_eig=5.0,
            eff_ratio=0.67, degen_count=2,
            mask=[1, 1, 0, 0, 1, 1],
        )
        m._on_rclpy_degeneracy_detail(msg)
        self.assertAlmostEqual(m._condition_number, 123.4, places=3)
        self.assertAlmostEqual(m._effective_ratio, 0.67, places=3)
        self.assertEqual(m._degenerate_dof_count, 2)
        self.assertEqual(m._dof_mask.tolist(), [1.0, 1.0, 0.0, 0.0, 1.0, 1.0])

    def test_legacy_11_field_payload_keeps_iekf_defaults(self):
        """Old fastlio2 publishers emit only 11 floats; new IEKF state stays default."""
        m = self._make()
        msg = self._make_msg(cond=10.0, min_eig=0.5, max_eig=5.0,
                             eff_ratio=0.9, degen_count=0,
                             mask=[1, 1, 1, 1, 1, 1])
        # Sanity check: this is the legacy 11-float layout.
        self.assertEqual(len(msg.data), 11)
        m._on_rclpy_degeneracy_detail(msg)
        self.assertEqual(m._pos_cov_trace, 0.0)
        self.assertEqual(m._ieskf_iter_num, 0)
        self.assertTrue(m._ieskf_converged)

    def test_extended_14_field_payload_populates_iekf_diagnostics(self):
        """New fastlio2 publishers append [pos_cov_trace, iter_num, converged]."""
        m = self._make()
        msg = self._make_msg(cond=10.0, min_eig=0.5, max_eig=5.0,
                             eff_ratio=0.9, degen_count=0,
                             mask=[1, 1, 1, 1, 1, 1])
        msg.data = list(msg.data) + [0.123, 7.0, 1.0]  # extend to 14
        m._on_rclpy_degeneracy_detail(msg)
        self.assertAlmostEqual(m._pos_cov_trace, 0.123, places=6)
        self.assertEqual(m._ieskf_iter_num, 7)
        self.assertTrue(m._ieskf_converged)

    def test_extended_payload_marks_non_convergence(self):
        """converged < 0.5 in slot 13 flips _ieskf_converged to False."""
        m = self._make()
        msg = self._make_msg(cond=10.0, min_eig=0.5, max_eig=5.0,
                             eff_ratio=0.9, degen_count=0,
                             mask=[1, 1, 1, 1, 1, 1])
        msg.data = [*list(msg.data), 42.0, 5.0, 0.0]
        m._on_rclpy_degeneracy_detail(msg)
        self.assertFalse(m._ieskf_converged)
        self.assertAlmostEqual(m._pos_cov_trace, 42.0, places=6)


    def test_severe_warning_triggers_above_cond_threshold(self):
        """condition_number > 1e6 triggers a throttled SEVERE warning."""
        import logging
        m = self._make()
        msg = self._make_msg(cond=5e6, min_eig=1e-7, max_eig=5.0,
                             eff_ratio=0.33, degen_count=4,
                             mask=[1, 1, 0, 0, 0, 0])
        with self.assertLogs("localization.bridge", level=logging.WARNING) as cm:
            m._on_rclpy_degeneracy_detail(msg)
        joined = " ".join(cm.output)
        self.assertIn("SEVERE", joined)
        self.assertIn("cond", joined.lower())

    def test_severe_warning_is_throttled(self):
        """Two SEVERE cases within 30s emit exactly one warning."""
        import logging
        m = self._make()
        msg = self._make_msg(cond=5e6, min_eig=1e-7, max_eig=5.0,
                             eff_ratio=0.33, degen_count=4,
                             mask=[1, 1, 0, 0, 0, 0])
        with self.assertLogs("localization.bridge", level=logging.WARNING) as cm:
            m._on_rclpy_degeneracy_detail(msg)
            m._on_rclpy_degeneracy_detail(msg)  # second call 闁?should be throttled
        severe_lines = [line for line in cm.output if "SEVERE" in line]
        self.assertEqual(len(severe_lines), 1,
                         f"Expected exactly one SEVERE warning, got {len(severe_lines)}: {severe_lines}")

    def test_cov_warning_tracks_odometry_covariance(self):
        """_max_pos_cov follows the largest of cov[0,7,14] from ROS Odometry."""
        m = self._make()

        class _Ros2Odom:
            class _Pose:
                class _P:
                    pass
                class _Q:
                    pass
                pose = None
                covariance = None
            class _Twist:
                class _Lin:
                    pass
                class _Ang:
                    pass
                twist = None
            header = type("_H", (), {"stamp": type("_S", (), {"sec": 0, "nanosec": 0})})()

        # Build a simple fake ROS2 Odometry with covariance[0] = 150.0
        msg = _Ros2Odom()
        msg.pose = _Ros2Odom._Pose()
        p = _Ros2Odom._Pose._P()
        p.x, p.y, p.z = 1.0, 2.0, 3.0
        q = _Ros2Odom._Pose._Q()
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        class _PP:
            pass
        pose_obj = _PP()
        pose_obj.position = p
        pose_obj.orientation = q
        msg.pose.pose = pose_obj
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 150.0  # x variance 闁?triggers cov_warning
        msg.pose.covariance[7] = 4.0
        msg.pose.covariance[14] = 0.5

        msg.twist = _Ros2Odom._Twist()
        lin = _Ros2Odom._Twist._Lin()
        lin.x = lin.y = lin.z = 0.0
        ang = _Ros2Odom._Twist._Ang()
        ang.x = ang.y = ang.z = 0.0
        tw = _PP()
        tw.linear = lin
        tw.angular = ang
        msg.twist.twist = tw

        m._on_rclpy_odom(msg)
        self.assertAlmostEqual(m._max_pos_cov, 150.0, places=3)

    @pytest.mark.ros2
    def test_rclpy_odom_uses_drift_guard_for_z_divergence(self):
        """The rclpy fallback must suppress impossible Z drift before fan-out."""
        m = self._make()
        m._drift_confirm_frames = 1
        m._drift_max_abs_z = 2.0
        m._drift_max_z_jump = 1.0
        m._drift_relocalize_cooldown = 10**12
        published = []
        m.odometry._add_callback(published.append)

        class _Msg:
            pass

        class _Obj:
            pass

        def odom_msg(x, y, z):
            msg = _Msg()
            msg.header = _Obj()
            msg.header.frame_id = "odom"
            msg.pose = _Obj()
            msg.pose.pose = _Obj()
            msg.pose.pose.position = _Obj()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = z
            msg.pose.pose.orientation = _Obj()
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance = [0.0] * 36
            msg.twist = _Obj()
            msg.twist.twist = _Obj()
            msg.twist.twist.linear = _Obj()
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular = _Obj()
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = 0.0
            msg.child_frame_id = "body"
            return msg

        m._on_rclpy_odom(odom_msg(0.0, 0.0, 0.3))
        worker = getattr(m, "_odom_worker_thread", None)
        if worker is not None:
            worker.join(timeout=0.2)
        self.assertEqual(len(published), 1)

        m._on_rclpy_odom(odom_msg(0.1, 0.0, 71.0))
        worker = getattr(m, "_odom_worker_thread", None)
        if worker is not None:
            worker.join(timeout=0.2)
        self.assertEqual(len(published), 1)
        self.assertEqual(m._loc_state, "DIVERGED")


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# P3: localizer-side multi-frame health (/nav/localization_health subscription)
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestSlamBridgeLocalizerHealth(unittest.TestCase):
    """P3: parse multi-frame confirmed localizer health from
    /nav/localization_health String topic."""

    def _make(self):
        from localization.bridge import SlamBridgeModule
        return SlamBridgeModule(watchdog_hz=100)

    def _msg(self, data):
        class _M:
            pass
        m = _M()
        m.data = data
        return m

    def test_initial_health_is_unknown(self):
        m = self._make()
        self.assertEqual(m._localizer_health, "UNKNOWN")
        self.assertEqual(m._localizer_health_fitness, 0.0)

    def test_locked_payload_parsed(self):
        m = self._make()
        m._on_rclpy_localization_health(self._msg("LOCKED|fitness=0.0234"))
        self.assertEqual(m._localizer_health, "LOCKED")
        self.assertAlmostEqual(m._localizer_health_fitness, 0.0234, places=4)

    def test_lost_payload_parsed(self):
        m = self._make()
        m._on_rclpy_localization_health(self._msg("LOST|fitness=0.45"))
        self.assertEqual(m._localizer_health, "LOST")
        self.assertAlmostEqual(m._localizer_health_fitness, 0.45, places=4)

    def test_recovered_payload_parsed(self):
        m = self._make()
        m._on_rclpy_localization_health(self._msg("RECOVERED|fitness=0.05"))
        self.assertEqual(m._localizer_health, "RECOVERED")

    def test_payload_without_fitness_keeps_state(self):
        """Defensive: payload missing the |fitness=... suffix should still
        update state without raising."""
        m = self._make()
        m._localizer_health_fitness = 0.1
        m._on_rclpy_localization_health(self._msg("LOST"))
        self.assertEqual(m._localizer_health, "LOST")
        self.assertAlmostEqual(m._localizer_health_fitness, 0.1, places=4)

    def test_malformed_payload_does_not_crash(self):
        m = self._make()
        m._on_rclpy_localization_health(self._msg("garbage|fitness=NaN"))
        self.assertEqual(m._localizer_health, "GARBAGE")

    def test_r4_extended_payload_parses_iter_and_cov(self):
        """R4: small_gicp adds iter + cov fields. Parser must pick them up
        for downstream three-axis health gating."""
        m = self._make()
        m._on_rclpy_localization_health(
            self._msg("LOCKED|fitness=0.0234|iter=8|cov=0.12"))
        self.assertEqual(m._localizer_health, "LOCKED")
        self.assertAlmostEqual(m._localizer_health_fitness, 0.0234, places=4)
        self.assertEqual(m._localizer_health_iter, 8)
        self.assertAlmostEqual(m._localizer_health_cov_trace, 0.12, places=4)

    def test_r4_v1_payload_keeps_iter_and_cov_at_default(self):
        """Backward-compat: a robot still publishing the v1 P3 payload
        (no iter / no cov) must NOT raise and must keep the iter/cov
        fields at their -1 sentinel."""
        m = self._make()
        m._on_rclpy_localization_health(self._msg("LOCKED|fitness=0.05"))
        self.assertEqual(m._localizer_health, "LOCKED")
        self.assertEqual(m._localizer_health_iter, -1)
        self.assertEqual(m._localizer_health_cov_trace, -1.0)

    def test_r4_unknown_keys_ignored(self):
        """Forward-compat: future localizer payload may add keys we don't
        know yet; parser must skip them silently without affecting known
        fields."""
        m = self._make()
        m._on_rclpy_localization_health(
            self._msg("LOCKED|fitness=0.05|iter=3|cov=0.01|future_key=42"))
        self.assertEqual(m._localizer_health_iter, 3)
        self.assertAlmostEqual(m._localizer_health_cov_trace, 0.01, places=4)

    def test_r4_field_order_independent(self):
        """Keys after the leading state must be order-independent."""
        m = self._make()
        m._on_rclpy_localization_health(
            self._msg("LOST|cov=5.5|iter=10|fitness=0.4"))
        self.assertEqual(m._localizer_health, "LOST")
        self.assertEqual(m._localizer_health_iter, 10)
        self.assertAlmostEqual(m._localizer_health_fitness, 0.4, places=4)
        self.assertAlmostEqual(m._localizer_health_cov_trace, 5.5, places=4)


class TestSlamBridgeTFJumpDetection(unittest.TestCase):
    """P4: detect map闁愁偅鏀糳om TF discontinuities and publish events."""

    def _make(self, **kw):
        from localization.bridge import SlamBridgeModule
        defaults = {"jump_t_threshold_m": 1.0, "jump_r_threshold_deg": 30.0}
        defaults.update(kw)
        return SlamBridgeModule(watchdog_hz=100, **defaults)

    def test_first_call_does_not_emit_jump(self):
        """No baseline 闁?cannot diff 闁?first call must NOT emit a jump event."""
        m = self._make()
        events = []
        m.map_frame_jump_event._add_callback(events.append)
        m._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)
        self.assertEqual(len(events), 0)

    def test_small_translation_does_not_emit(self):
        m = self._make()
        events = []
        m._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)
        m.map_frame_jump_event._add_callback(events.append)
        m._cache_map_odom_tf(0.1, 0.1, 0, 0, 0, 0, 1)  # 闁?.02 闁?0.14m
        self.assertEqual(len(events), 0)

    def test_large_translation_emits_jump(self):
        m = self._make()
        events = []
        m._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)
        m.map_frame_jump_event._add_callback(events.append)
        m._cache_map_odom_tf(2.0, 0, 0, 0, 0, 0, 1)  # 2m jump > 1m threshold
        self.assertEqual(len(events), 1)
        evt = events[0]
        self.assertGreater(evt["dt_m"], 1.0)
        self.assertEqual(evt["new_xyz"], [2.0, 0.0, 0.0])

    def test_large_rotation_emits_jump(self):
        """A 60閹?yaw jump (no translation) should still emit even though 閾绘潰=0."""
        import math
        m = self._make()
        events = []
        m._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)  # identity
        m.map_frame_jump_event._add_callback(events.append)
        # 60閹?yaw rotation as quaternion
        half = math.radians(60.0) / 2.0
        m._cache_map_odom_tf(0, 0, 0, 0, 0, math.sin(half), math.cos(half))
        self.assertEqual(len(events), 1)
        self.assertGreater(events[0]["dyaw_deg"], 30.0)

    def test_threshold_overrides_via_kw(self):
        m = self._make(jump_t_threshold_m=10.0)  # very loose
        events = []
        m._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)
        m.map_frame_jump_event._add_callback(events.append)
        m._cache_map_odom_tf(2.0, 0, 0, 0, 0, 0, 1)  # 2m, well below 10m
        self.assertEqual(len(events), 0)


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# SafetyRing localization tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestSafetyRingLocalization(unittest.TestCase):

    def _make(self):
        from nav.services.safety.safety_ring import SafetyRing
        return SafetyRing()

    def test_localization_status_port_exists(self):
        m = self._make()
        self.assertIn("localization_status", m.ports_in)

    def test_lost_triggers_stop(self):
        m = self._make()
        m.setup()
        # Prime odom so odom timeout doesn't interfere
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        # Should have published stop_cmd = 2
        self.assertTrue(any(cmd == 2 for cmd in stop_cmds),
                        f"Expected stop_cmd=2 in {stop_cmds}")

    def test_degraded_triggers_warn(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m._on_localization_status({"state": "DEGRADED", "confidence": 0.3})
        self.assertTrue(any(cmd == 1 for cmd in stop_cmds),
                        f"Expected stop_cmd=1 in {stop_cmds}")

    def test_tracking_clears_stop(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        # Go LOST then recover
        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertTrue(any(cmd == 0 for cmd in stop_cmds),
                        f"Expected stop_cmd=0 in {stop_cmds}")

    def test_health_includes_localization(self):
        m = self._make()
        m._on_localization_status({"state": "TRACKING", "confidence": 0.9})
        h = m.health()
        self.assertEqual(h["safety_ring"]["localization_state"], "TRACKING")
        self.assertAlmostEqual(h["safety_ring"]["localization_confidence"], 0.9)

    def test_dialogue_includes_localization(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()

        dialogue_msgs = []
        m.dialogue_state._add_callback(dialogue_msgs.append)

        odom = Odometry(pose=Pose(position=Vector3(1, 2, 0)), ts=time.time())
        m._on_odom(odom)
        self.assertTrue(len(dialogue_msgs) > 0)
        self.assertIn("localization", dialogue_msgs[-1])


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# Navigation pause/resume tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestNavigationPauseResume(unittest.TestCase):

    def _make(self, **kw):
        from nav.mission.navigation import MissionState, Navigation
        m = Navigation(planner="direct", **kw)
        m.setup()
        return m, MissionState

    def _pose_goal(self, x=1.0, y=2.0, z=0.0):
        from runtime.msgs.geometry import PoseStamped
        return PoseStamped(pose=Pose(position=Vector3(x, y, z)), ts=time.time())

    def _start_executing_mission(self, m, MS):
        import numpy as np
        m._state = MS.EXECUTING
        m._goal = np.array([10.0, 0.0, 0.0])
        m._tracker.reset([np.array([10.0, 0.0, 0.0])], np.zeros(3))

    def _localizer_restart_lost(self):
        return {
            "state": "LOST",
            "confidence": 0.0,
            "recovery_signal": "ODOM_MISSING",
            "recovery_action": "restart_localization_chain",
        }

    def _tracking_recovered_from_restart(self):
        return {
            "state": "TRACKING",
            "confidence": 1.0,
            "recovery_signal": "RECOVERED",
            "recovery_action": "none",
        }

    def test_localization_status_port_exists(self):
        from nav.mission.navigation import Navigation
        m = Navigation(planner="direct")
        self.assertIn("localization_status", m.ports_in)

    def test_lost_pauses_executing(self):
        m, MS = self._make()
        m._state = MS.EXECUTING
        m._goal = [1.0, 2.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})

        self.assertTrue(m._paused_for_localization)
        self.assertEqual(m._state, MS.PAUSED)
        self.assertEqual(m._pre_pause_state, MS.EXECUTING)

    def test_tracking_resumes_after_pause(self):
        m, MS = self._make()
        m._state = MS.EXECUTING
        m._goal = [1.0, 2.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._state, MS.PAUSED)

        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertFalse(m._paused_for_localization)
        self.assertEqual(m._state, MS.EXECUTING)

    def test_restarted_localizer_recovery_does_not_auto_resume_old_mission(self):
        m, MS = self._make(stuck_timeout=0.01)
        self._start_executing_mission(m, MS)
        waypoints = []
        m.waypoint._add_callback(waypoints.append)
        recovery_cmds = []
        m.recovery_cmd_vel._add_callback(recovery_cmds.append)
        recovery_attempts = []
        m._execute_recovery_motion = lambda: recovery_attempts.append("called")

        m._on_localization_status(self._localizer_restart_lost())
        self.assertEqual(m._state, MS.PAUSED)

        m._on_localization_status(self._tracking_recovered_from_restart())
        m._tracker._last_progress_time = time.time() - 1.0
        m._tracker._stuck_warn_sent = True
        m._on_odom(Odometry(
            pose=Pose(position=Vector3(0, 0, 0)),
            frame_id="map",
            ts=time.time(),
        ))

        self.assertEqual(m._state, MS.IDLE)
        self.assertEqual(waypoints, [])
        self.assertEqual(recovery_attempts, [])
        self.assertEqual(recovery_cmds, [])

    def test_new_goal_clears_restarted_localizer_hold(self):
        m, MS = self._make(allow_direct_goal_fallback=True)
        self._start_executing_mission(m, MS)
        waypoints = []
        m.waypoint._add_callback(waypoints.append)

        m._on_localization_status(self._localizer_restart_lost())
        m._on_localization_status(self._tracking_recovered_from_restart())
        self.assertEqual(m._state, MS.IDLE)

        m._on_goal(self._pose_goal(3.0, 4.0, 0.0))

        self.assertFalse(m._paused_for_localization)
        self.assertIsNone(m._pre_pause_state)
        self.assertEqual(m._state, MS.EXECUTING)
        self.assertEqual(len(waypoints), 1)

    def test_cancel_clears_restarted_localizer_hold(self):
        m, MS = self._make(stuck_timeout=0.01)
        self._start_executing_mission(m, MS)
        recovery_attempts = []
        m._execute_recovery_motion = lambda: recovery_attempts.append("called")

        m._on_localization_status(self._localizer_restart_lost())
        self.assertTrue(m._paused_for_localization)

        m._on_cancel("operator_cancel")
        m._on_localization_status(self._tracking_recovered_from_restart())
        m._tracker._last_progress_time = time.time() - 1.0
        m._tracker._stuck_warn_sent = True
        m._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0)), ts=time.time()))

        self.assertFalse(m._paused_for_localization)
        self.assertIsNone(m._pre_pause_state)
        self.assertNotEqual(m._state, MS.EXECUTING)
        self.assertEqual(recovery_attempts, [])

    def test_idle_not_affected_by_lost(self):
        m, MS = self._make()
        m._state = MS.IDLE

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._state, MS.IDLE)
        self.assertFalse(m._paused_for_localization)

    def test_odom_skipped_while_paused(self):
        m, MS = self._make()
        import numpy as np
        m._state = MS.EXECUTING
        m._goal = np.array([10.0, 10.0, 0.0])

        # Pause
        m._on_localization_status({"state": "LOST", "confidence": 0.0})

        # Send odom 闁?should be skipped (position updates but tracker not called)
        odom = Odometry(
            pose=Pose(position=Vector3(5, 5, 0)),
            frame_id="map",
            ts=time.time(),
        )
        m._on_odom(odom)
        # mission_status should not change (no tracker event)
        self.assertEqual(m._state, MS.PAUSED)

    def test_goal_preserved_during_pause(self):
        m, MS = self._make()
        import numpy as np
        m._state = MS.EXECUTING
        m._goal = np.array([10.0, 20.0, 0.0])

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        # Goal should still be set
        self.assertIsNotNone(m._goal)
        self.assertAlmostEqual(m._goal[0], 10.0)
        self.assertAlmostEqual(m._goal[1], 20.0)

    def test_patrolling_pause_resume(self):
        from nav.mission.model.state import MissionMode

        m, MS = self._make()
        m._state = MS.EXECUTING
        m._mission_mode = MissionMode.PATROL
        m._goal = [5.0, 5.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._pre_pause_state, MS.EXECUTING)

        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertEqual(m._state, MS.EXECUTING)
        self.assertEqual(m._mission_mode, MissionMode.PATROL)


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# WaypointTracker pause tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestWaypointTrackerPause(unittest.TestCase):

    def test_pause_resets_stuck_timer(self):
        import numpy as np

        from nav.mission.tracking.waypoint_tracker import WaypointTracker
        t = WaypointTracker(threshold=1.0, stuck_timeout=5.0)
        t.reset([np.array([10, 0, 0])], np.array([0, 0, 0]))
        # Simulate some time passing
        t._last_progress_time = time.time() - 100  # Very stale
        t._stuck_warn_sent = True
        t._stuck_sent = True
        # Pause should reset stuck state
        t.pause()
        self.assertFalse(t._stuck_warn_sent)
        self.assertFalse(t._stuck_sent)
        # last_progress_time should be recent
        self.assertAlmostEqual(t._last_progress_time, time.time(), delta=1.0)

    def test_pause_preserves_path(self):
        import numpy as np

        from nav.mission.tracking.waypoint_tracker import WaypointTracker
        t = WaypointTracker(threshold=1.0)
        path = [np.array([5, 0, 0]), np.array([10, 0, 0])]
        t.reset(path, np.array([0, 0, 0]))
        t.pause()
        self.assertEqual(len(t._path), 2)
        self.assertEqual(t._wp_index, 0)


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# SLAM degeneracy detection tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class TestSlamDegeneracyDetection(unittest.TestCase):

    def _make(self, **kw):
        from localization.bridge import SlamBridgeModule
        defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
        defaults.update(kw)
        return SlamBridgeModule(**defaults)

    def test_initial_degeneracy_is_none(self):
        from localization.bridge import DEGEN_NONE
        m = self._make()
        self.assertEqual(m._degen_level, DEGEN_NONE)

    def test_high_icp_fitness_triggers_critical(self):
        from localization.bridge import DEGEN_CRITICAL
        m = self._make()
        m._icp_fitness = 0.5  # Above fitness_critical (0.3)
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)

    def test_moderate_icp_fitness_triggers_severe(self):
        from localization.bridge import DEGEN_SEVERE
        m = self._make()
        m._icp_fitness = 0.2  # Above fitness_warn (0.15)
        m._effective_ratio = 0.5  # Not critical
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_SEVERE)

    def test_low_feature_ratio_triggers_critical(self):
        from localization.bridge import DEGEN_CRITICAL
        m = self._make()
        m._icp_fitness = 0.0
        m._effective_ratio = 0.05  # Below feature_ratio_critical (0.1)
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)

    def test_good_metrics_clears_degeneracy(self):
        from localization.bridge import DEGEN_CRITICAL, DEGEN_NONE
        m = self._make()
        m._icp_fitness = 0.5
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)
        # Now restore good metrics
        m._icp_fitness = 0.0
        m._effective_ratio = 1.0
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_NONE)

    def test_degeneracy_affects_watchdog_confidence(self):
        from localization.bridge import DEGEN_CRITICAL
        m = self._make(odom_timeout=0.5, cloud_timeout=0.5)
        received = []
        m.localization_status._add_callback(received.append)
        m._mark_odom_received()
        m._mark_cloud_received()
        m._degen_level = DEGEN_CRITICAL
        m.start()
        time.sleep(0.025)
        m.stop()
        # Should get DEGRADED with low confidence
        degraded = [r for r in received if r["state"] == "DEGRADED"]
        self.assertTrue(len(degraded) > 0, f"Expected DEGRADED, got states: {[r['state'] for r in received]}")
        self.assertLessEqual(degraded[0]["confidence"], 0.2)

    def test_localization_status_includes_degeneracy_fields(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        time.sleep(0.02)
        m.stop()
        self.assertTrue(len(received) > 0)
        self.assertIn("degeneracy", received[0])
        self.assertIn("icp_fitness", received[0])
        self.assertIn("effective_ratio", received[0])

    def test_health_includes_degeneracy(self):
        m = self._make()
        m._icp_fitness = 0.2
        m._effective_ratio = 0.5
        h = m.health()
        self.assertEqual(h["localization"]["icp_fitness"], 0.2)
        self.assertEqual(h["localization"]["effective_ratio"], 0.5)


# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾
# Navigation degeneracy speed limiting tests
# 闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾闁冲厜鍋撻柍鍏夊亾

class _RecoveryServiceManager:
    def __init__(self):
        self.stops = []
        self.ensures = []

    def stop(self, *services):
        self.stops.append(tuple(services))

    def ensure(self, *services, track_started=True):
        self.ensures.append((tuple(services), track_started))


def _make_successful_recovery_bridge(*, backend_profile):
    from localization.bridge import SlamBridgeModule

    m = SlamBridgeModule(
        backend_profile=backend_profile,
        odom_timeout=0.1,
        cloud_timeout=0.2,
        watchdog_hz=20,
    )
    m._wait_ros_topic_publishers = lambda *args, **kwargs: True
    m._wait_for_odom_sample_since = lambda *args, **kwargs: True
    m._wait_for_localizer_health_since = lambda *args, **kwargs: True
    return m


@pytest.mark.skipif(os.name == "nt", reason="systemd service manager not available on Windows")
def test_bridge_recovery_does_not_claim_external_localization_services(monkeypatch):
    import runtime.service_manager as service_manager

    fake = _RecoveryServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    m = _make_successful_recovery_bridge(backend_profile="bridge")

    assert m._restart_localization_chain_for_recovery_locked() is True
    assert fake.stops == [("localizer",), ("slam",)]
    assert fake.ensures == [(("slam",), False), (("localizer",), False)]


@pytest.mark.skipif(os.name == "nt", reason="systemd service manager not available on Windows")
def test_managed_localizer_recovery_keeps_service_ownership(monkeypatch):
    import runtime.service_manager as service_manager

    fake = _RecoveryServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    m = _make_successful_recovery_bridge(backend_profile="localizer")

    assert m._restart_localization_chain_for_recovery_locked() is True
    assert fake.stops == [("localizer",), ("slam",)]
    assert fake.ensures == [(("slam",), True), (("localizer",), True)]


class TestNavigationDegeneracyResponse(unittest.TestCase):

    def _make(self):
        from nav.mission.navigation import MissionState, Navigation
        m = Navigation(planner="astar")
        m.setup()
        return m, MissionState

    def test_severe_degeneracy_limits_speed(self):
        m, _MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 0.4,
            "degeneracy": "SEVERE",
        })
        self.assertAlmostEqual(m._speed_scale, 0.4)

    def test_mild_degeneracy_limits_speed(self):
        m, _MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 0.7,
            "degeneracy": "MILD",
        })
        self.assertAlmostEqual(m._speed_scale, 0.7)

    def test_no_degeneracy_full_speed(self):
        m, _MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 1.0,
            "degeneracy": "NONE",
        })
        self.assertAlmostEqual(m._speed_scale, 1.0)

    def test_fallback_gnss_only_caps_speed_below_severe(self):
        """FALLBACK is a more cautious mode than DEGEN SEVERE 闁?slower scale."""
        m, _MS = self._make()
        m._on_localization_status({
            "state": "FALLBACK_GNSS_ONLY", "confidence": 0.4,
            "degeneracy": "SEVERE",
        })
        self.assertAlmostEqual(m._speed_scale, 0.3)

    def test_fallback_overrides_mild_degeneracy(self):
        """Even if degeneracy is only MILD, FALLBACK state forces 0.3x."""
        m, _MS = self._make()
        m._on_localization_status({
            "state": "FALLBACK_GNSS_ONLY", "confidence": 0.4,
            "degeneracy": "MILD",
        })
        self.assertAlmostEqual(m._speed_scale, 0.3)


if __name__ == "__main__":
    unittest.main()
