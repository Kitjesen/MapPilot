"""Contract tests for ExplorationSupervisorModule — port shape, lifecycle, state machine.

Verifies ExplorationSupervisorModule declares correct In/Out port types,
idempotent lifecycle transitions, and the mode evaluation state machine
transitions correctly through uninit -> starting -> healthy -> degraded
-> fallback -> finished.

NOTE: Comprehensive functional tests (one-shot ready_fired, fallback
recovery, degraded tick counting) live in ``test_exploration_modules.py``.
This file focuses on the *contract* interface — port declarations,
lifecycle, and state machine evaluation.
"""

from __future__ import annotations

import pytest

pytestmark = [pytest.mark.ros2]


class TestExplorationSupervisorContract:
    """Contract verification for ExplorationSupervisorModule."""

    def test_instantiation(self):
        """Create ExplorationSupervisorModule with default params."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_UNINIT,
        )

        mod = ExplorationSupervisorModule()
        assert mod._warn_timeout_s == 20.0
        assert mod._fallback_timeout_s == 60.0
        assert mod._mode == MODE_UNINIT
        assert mod._reason == "no tare_stats received yet"
        assert not mod._fallback_requested

    def test_instantiation_with_custom_timeouts(self):
        """Custom timeout values must be reflected in module state."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule(
            warn_timeout_s=10.0, fallback_timeout_s=30.0, poll_hz=2.0
        )
        assert mod._warn_timeout_s == 10.0
        assert mod._fallback_timeout_s == 30.0
        assert mod._interval == 0.5

    def test_input_port_types(self):
        """Must declare tare_stats: In[dict]."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        assert "tare_stats" in mod._ports_in
        assert mod._ports_in["tare_stats"].msg_type is dict
        assert len(mod._ports_in) == 1

    def test_output_port_types(self):
        """Must declare supervisor_state: Out[dict] and exploration_ready: Out[bool]."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        expected_out = {
            "supervisor_state": dict,
            "exploration_ready": bool,
        }
        assert len(mod._ports_out) == len(expected_out)
        for name, expected_type in expected_out.items():
            assert name in mod._ports_out, f"missing Out port: {name}"
            assert mod._ports_out[name].msg_type is expected_type, (
                f"Out.{name}: expected {expected_type.__name__}, "
                f"got {mod._ports_out[name].msg_type.__name__}"
            )

    def test_lifecycle_setup(self):
        """setup() must not raise."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        mod.setup()

    def test_lifecycle_start_stop_idempotent(self):
        """start()/stop() transitions without error and stop is idempotent."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        mod.setup()
        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running
        # Second stop is idempotent
        mod.stop()
        assert not mod._running

    def test_evaluate_uninit(self):
        """_evaluate returns 'uninit' when no stats received."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_UNINIT,
        )

        mod = ExplorationSupervisorModule()
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_UNINIT
        assert "no tare_stats" in reason
        assert wp_age is None

    def test_evaluate_starting(self):
        """_evaluate returns 'starting' when alive but no waypoint yet."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_STARTING,
        )

        mod = ExplorationSupervisorModule()
        mod._last_stats = {"alive": True, "finished": False}
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_STARTING
        assert "waiting" in reason

    def test_evaluate_healthy(self):
        """_evaluate returns 'healthy' when stats are alive, have waypoints,
        and waypoint_age_s is within warn_timeout_s."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_HEALTHY,
        )

        mod = ExplorationSupervisorModule(warn_timeout_s=20.0, fallback_timeout_s=60.0)
        mod._last_stats = {
            "alive": True,
            "healthy": True,
            "finished": False,
            "waypoint_age_s": 1.0,
        }
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_HEALTHY
        assert reason == "ok"
        assert wp_age == 1.0

    def test_evaluate_degraded(self):
        """_evaluate returns 'degraded' when waypoint_age_s > warn_timeout_s."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_DEGRADED,
        )

        mod = ExplorationSupervisorModule(warn_timeout_s=5.0, fallback_timeout_s=30.0)
        mod._last_stats = {
            "alive": True,
            "healthy": True,
            "waypoint_age_s": 10.0,
        }
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_DEGRADED
        assert "stuck" in reason
        assert wp_age == 10.0

    def test_evaluate_fallback(self):
        """_evaluate returns 'fallback' when waypoint_age_s > fallback_timeout_s."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_FALLBACK,
        )

        mod = ExplorationSupervisorModule(warn_timeout_s=5.0, fallback_timeout_s=10.0)
        mod._last_stats = {
            "alive": True,
            "healthy": False,
            "waypoint_age_s": 15.0,
        }
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_FALLBACK
        assert "no waypoint" in reason
        assert wp_age == 15.0

    def test_evaluate_finished(self):
        """_evaluate returns 'finished' when stats say finished."""
        from nav.exploration.tare.supervisor import (
            ExplorationSupervisorModule,
            MODE_FINISHED,
        )

        mod = ExplorationSupervisorModule()
        mod._last_stats = {"finished": True, "alive": True}
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_FINISHED
        assert "complete" in reason
