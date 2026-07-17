"""Contract tests for exploration modules.

Verifies module instantiation, port registration, lifecycle (setup/start/stop),
and supervisor state machine for:
  - TAREExplorerModule
  - ExplorationSupervisorModule

These are CONTRACT tests — they verify the module interface contract, not
internal implementation details or algorithmic correctness.
"""

from __future__ import annotations

import pytest

pytestmark = [pytest.mark.ros2]


# =============================================================================
# TAREExplorerModule
# =============================================================================


class TestTAREExplorerModule:
    """Contract tests for TAREExplorerModule (layer=5, TARE exploration bridge)."""

    def test_instantiation(self):
        """Creating a TAREExplorerModule with default params should succeed."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule()
        # configured_backend is stored inside _backend_status, not as a separate attr
        assert mod._backend_status.configured == "tare"

    def test_instantiation_with_custom_params(self):
        """Custom constructor parameters must be reflected in module state."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule(
            configured_backend="custom_tare",
            way_point_topic="/custom/way_point",
            prefer_path_strategy=True,
            max_waypoint_distance_m=10.0,
        )
        # configured_backend is stored inside _backend_status, not as a separate attr
        assert mod._backend_status.configured == "custom_tare"
        assert mod._way_point_topic == "/custom/way_point"
        assert mod._prefer_path_strategy is True
        assert mod._max_waypoint_distance_m == 10.0

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from nav.exploration.tare.module import TAREExplorerModule
        from runtime.msgs.geometry import PoseStamped
        from runtime.msgs.nav import Odometry

        mod = TAREExplorerModule()

        # -- Input ports --
        expected_in = {
            "odometry": Odometry,
            "exploration_grid": dict,
            "navigation_status": dict,
        }
        assert len(mod._ports_in) == len(expected_in), (
            f"expected {len(expected_in)} In ports, got {list(mod._ports_in)}"
        )
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, got {mod._ports_in[name].msg_type.__name__}"
            )

        # -- Output ports --
        expected_out = {
            "exploration_goal": PoseStamped,
            "exploration_path": list,
            "exploring": bool,
            "tare_stats": dict,
            "alive": bool,
        }
        assert len(mod._ports_out) == len(expected_out), (
            f"expected {len(expected_out)} Out ports, got {list(mod._ports_out)}"
        )
        for name, expected_type in expected_out.items():
            assert name in mod._ports_out, f"missing Out port: {name}"
            assert mod._ports_out[name].msg_type is expected_type, (
                f"Out.{name}: expected {expected_type.__name__}, got {mod._ports_out[name].msg_type.__name__}"
            )

    def test_preflight_returns_error_without_dds(self):
        """Without cyclonedds, preflight() must return an error string."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule(transport_mode="dds")
        result = mod.preflight()
        assert result is not None
        assert "lingtu_explore_kernel" in result

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_alive_publishes_on_start(self):
        """alive Out[bool] must publish on start()."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        mod.setup()

        alive_values: list[bool] = []
        mod.alive._add_callback(alive_values.append)

        mod.start()
        assert len(alive_values) >= 1
        assert alive_values[-1] is True

    def test_skill_methods_registered(self):
        """TAREExplorerModule must expose @skill methods."""
        from nav.exploration.tare.module import TAREExplorerModule

        mod = TAREExplorerModule()
        skill_names = set(mod.skills.keys())
        assert "start_tare_exploration" in skill_names, "missing start_tare_exploration skill"
        assert "stop_tare_exploration" in skill_names, "missing stop_tare_exploration skill"
        assert "get_tare_status" in skill_names, "missing get_tare_status skill"


# =============================================================================
# ExplorationSupervisorModule
# =============================================================================


class TestExplorationSupervisorModule:
    """Contract tests for ExplorationSupervisorModule (layer=5, watchdog)."""

    def test_instantiation(self):
        """Creating an ExplorationSupervisorModule with default params should succeed."""
        from nav.exploration.tare.supervisor import (
            MODE_UNINIT,
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        assert mod._warn_timeout_s == 20.0
        assert mod._fallback_timeout_s == 60.0
        assert mod._mode == MODE_UNINIT
        assert mod._reason == "no tare_stats received yet"
        assert not mod._fallback_requested

    def test_instantiation_with_custom_timeouts(self):
        """Custom timeout values must be reflected in module state."""
        from nav.exploration.tare.supervisor import ExplorationSupervisorModule

        mod = ExplorationSupervisorModule(warn_timeout_s=10.0, fallback_timeout_s=30.0, poll_hz=2.0)
        assert mod._warn_timeout_s == 10.0
        assert mod._fallback_timeout_s == 30.0
        assert mod._interval == 0.5

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from nav.exploration.tare.supervisor import ExplorationSupervisorModule

        mod = ExplorationSupervisorModule()

        # -- Input ports --
        assert "tare_stats" in mod._ports_in
        assert mod._ports_in["tare_stats"].msg_type is dict
        assert len(mod._ports_in) == 1, f"expected 1 In port, got {list(mod._ports_in)}"

        # -- Output ports --
        expected_out = {
            "supervisor_state": dict,
            "exploration_ready": bool,
        }
        assert len(mod._ports_out) == len(expected_out), (
            f"expected {len(expected_out)} Out ports, got {list(mod._ports_out)}"
        )
        for name, expected_type in expected_out.items():
            assert name in mod._ports_out, f"missing Out port: {name}"
            assert mod._ports_out[name].msg_type is expected_type, (
                f"Out.{name}: expected {expected_type.__name__}, got {mod._ports_out[name].msg_type.__name__}"
            )

    def test_lifecycle(self):
        """setup() -> start() -> stop() transitions without error."""
        from nav.exploration.tare.supervisor import ExplorationSupervisorModule

        mod = ExplorationSupervisorModule()
        assert not mod._running

        mod.setup()
        mod.start()
        assert mod._running

        mod.stop()
        assert not mod._running

        # stop is idempotent
        mod.stop()
        assert not mod._running

    def test_initial_mode_uninit(self):
        """Before receiving any tare_stats, mode must be 'uninit'."""
        from nav.exploration.tare.supervisor import (
            MODE_UNINIT,
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        assert mod._mode == MODE_UNINIT
        assert mod._reason == "no tare_stats received yet"

    def test_evaluate_no_stats(self):
        """_evaluate returns (uninit, ...) when no stats have been received."""
        from nav.exploration.tare.supervisor import (
            MODE_UNINIT,
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_UNINIT
        assert "no tare_stats received yet" in reason
        assert wp_age is None

    def test_evaluate_finished(self):
        """_evaluate returns (finished, ...) when tare_stats says finished."""
        from nav.exploration.tare.supervisor import (
            MODE_FINISHED,
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule()
        mod._last_stats = {"finished": True, "alive": True}
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_FINISHED
        assert "complete" in reason

    def test_evaluate_degraded_triggers_after_warn_timeout(self):
        """When waypoint_age_s exceeds warn_timeout_s, mode becomes 'degraded'."""
        from nav.exploration.tare.supervisor import (
            MODE_DEGRADED,
            ExplorationSupervisorModule,
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

    def test_evaluate_fallback_triggers_after_fallback_timeout(self):
        """When waypoint_age_s exceeds fallback_timeout_s, mode becomes 'fallback'."""
        from nav.exploration.tare.supervisor import (
            MODE_FALLBACK,
            ExplorationSupervisorModule,
        )

        mod = ExplorationSupervisorModule(warn_timeout_s=5.0, fallback_timeout_s=10.0)
        mod._last_stats = {
            "alive": True,
            "healthy": False,
            "waypoint_age_s": 15.0,
        }
        mode, reason, wp_age = mod._evaluate()
        assert mode == MODE_FALLBACK
        assert wp_age == 15.0

    def test_evaluate_healthy(self):
        """When stats are fresh and healthy, mode must be 'healthy'."""
        from nav.exploration.tare.supervisor import (
            MODE_HEALTHY,
            ExplorationSupervisorModule,
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

    def test_ready_fired_once(self):
        """exploration_ready Out[bool] must fire exactly once."""
        from nav.exploration.tare.supervisor import ExplorationSupervisorModule

        mod = ExplorationSupervisorModule()
        mod.setup()

        ready_values: list[bool] = []
        mod.exploration_ready._add_callback(ready_values.append)

        # Simulate incoming stats that mark TARE healthy
        mod._on_tare_stats({"alive": True, "healthy": True})
        assert len(ready_values) == 1
        assert ready_values[-1] is True

        # Second healthy stats should NOT re-fire
        mod._on_tare_stats({"alive": True, "healthy": True})
        assert len(ready_values) == 1, "exploration_ready should fire only once"

    def test_skill_methods_registered(self):
        """ExplorationSupervisorModule must expose @skill methods."""
        from nav.exploration.tare.supervisor import ExplorationSupervisorModule

        mod = ExplorationSupervisorModule()
        skill_names = set(mod.skills.keys())
        assert "get_exploration_supervisor" in skill_names
        assert "clear_exploration_fallback" in skill_names


# =============================================================================
# Cross-module contract checks
# =============================================================================


def test_all_exploration_modules_have_alive_port():
    """Every exploration module must expose an alive Out[bool] port."""
    from nav.exploration.tare.module import TAREExplorerModule

    mod = TAREExplorerModule()
    assert "alive" in mod._ports_out
    assert mod._ports_out["alive"].msg_type is bool


def test_supervisor_tare_explorer_wiring():
    """ExplorationSupervisorModule wires to TAREExplorerModule via Blueprint.

    Verifies that both modules coexist in a single Blueprint and their
    shared exploration ports (tare_stats, supervisor_state) are declared
    for wiring.
    """
    from nav.exploration.tare.module import TAREExplorerModule
    from nav.exploration.tare.supervisor import ExplorationSupervisorModule
    from runtime.blueprint import Blueprint

    bp = Blueprint()
    bp.add(TAREExplorerModule)
    bp.add(ExplorationSupervisorModule)

    system = bp.build()

    assert "TAREExplorerModule" in system.modules
    assert "ExplorationSupervisorModule" in system.modules

    # Verify port declarations exist for wiring (wires not checked at build)
    tare_mod = system.modules["TAREExplorerModule"]
    sup_mod = system.modules["ExplorationSupervisorModule"]
    assert "tare_stats" in tare_mod.ports_out, f"TARE out ports: {list(tare_mod.ports_out)}"
    assert "tare_stats" in sup_mod.ports_in, f"Supervisor in ports: {list(sup_mod.ports_in)}"
    assert "supervisor_state" in sup_mod.ports_out, f"Supervisor out ports: {list(sup_mod.ports_out)}"
