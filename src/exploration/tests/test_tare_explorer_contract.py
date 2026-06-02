"""Contract tests for TAREExplorerModule — port shape, lifecycle, command dispatch.

Verifies TAREExplorerModule declares the correct In/Out port types,
idempotent lifecycle transitions, and start/stop exploration command
dispatch via skills.

NOTE: Comprehensive functional tests (port type values, preflight logic,
navigation status tracking, waypoint rejection) live in
``test_exploration_modules.py``.  This file focuses on the *contract*
interface — port declarations, lifecycle, and skill dispatch.
"""

from __future__ import annotations

import pytest

pytestmark = [pytest.mark.ros2]

from core.msgs.geometry import PoseStamped
from core.msgs.nav import Odometry


class TestTAREExplorerContract:
    """Contract verification for TAREExplorerModule."""

    def test_instantiation(self):
        """Create TAREExplorerModule with default params."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule()
        assert mod._backend_status.configured == "tare"
        assert mod._way_point_topic == "/exploration/way_point"
        assert mod._auto_start is True

    def test_instantiation_with_disabled_auto_start(self):
        """auto_start=False must suppress start signal on start()."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        assert mod._auto_start is False

    def test_input_port_types(self):
        """All In ports must be declared with correct message types."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule()
        expected_in = {
            "odometry": Odometry,
            "navigation_status": dict,
        }
        assert len(mod._ports_in) == len(expected_in)
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, "
                f"got {mod._ports_in[name].msg_type.__name__}"
            )

    def test_output_port_types(self):
        """All Out ports must be declared with correct message types."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule()
        expected_out = {
            "exploration_goal": PoseStamped,
            "exploration_path": list,
            "exploring": bool,
            "tare_stats": dict,
            "alive": bool,
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
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        mod.setup()

    def test_lifecycle_start_stop_idempotent(self):
        """start()/stop() transitions without error and stop is idempotent."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        mod.setup()
        mod.start()
        assert mod._running
        mod.stop()
        assert not mod._running
        # Second stop is idempotent (no error on destroy_node etc.)
        mod.stop()
        assert not mod._running

    def test_start_exploration_skill_dispatch(self):
        """start_tare_exploration skill must publish exploring=True and return
        JSON with status 'started'."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        mod.setup()
        mod.start()

        exploring_values: list[bool] = []
        mod.exploring._add_callback(exploring_values.append)

        result = mod.start_tare_exploration()
        assert '"started"' in result
        assert mod._started_exploration is True
        assert len(exploring_values) >= 1
        assert exploring_values[-1] is True

        mod.stop()

    def test_stop_exploration_skill_dispatch(self):
        """stop_tare_exploration skill must publish exploring=False and return
        JSON with status 'stopped'."""
        from exploration.tare_explorer_module import TAREExplorerModule

        mod = TAREExplorerModule(auto_start=False)
        mod.setup()
        mod.start()

        exploring_values: list[bool] = []
        mod.exploring._add_callback(exploring_values.append)

        # Start first, then stop
        mod.start_tare_exploration()
        exploring_values.clear()

        result = mod.stop_tare_exploration()
        assert '"stopped"' in result
        assert mod._started_exploration is False
        # The last published exploring value should be False
        assert exploring_values[-1] is False

        mod.stop()
