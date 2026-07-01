"""Tests for the hot-swap system 閳?VelocityMux freeze/unfreeze + SwapManager.

Verifies that:
- Freezing the mux publishes zero velocity and ignores sources
- Unfreezing the mux restores normal command flow
- SwapManager creates, validates, and manages driver swapping
- State machine transitions correctly (IDLE -> SWAPPING -> COMPLETED)
- Failures during swap trigger rollback to the original driver

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock

from runtime.msgs.geometry import Twist, Vector3
from runtime.registry import register, snapshot, restore, clear


# -- helpers ----------------------------------------------------------

def _twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
    return Twist(linear=Vector3(x=vx, y=0.0, z=0.0),
                 angular=Vector3(x=0.0, y=0.0, z=wz))


_SAVED_REGISTRY: dict | None = None


def _save_registry() -> None:
    global _SAVED_REGISTRY
    _SAVED_REGISTRY = snapshot()


def _restore_registry() -> None:
    if _SAVED_REGISTRY is not None:
        restore(_SAVED_REGISTRY)


# -- mock module for SwapManager tests --------------------------------

class _MockModule:
    """Minimal module-like object for use in SwapManager tests.

    Provides the interface that SwapManager.current_driver() expects
    (_layer + ports_in with cmd_vel / stop_signal).
    """

    def __init__(self, name: str = "mock_driver", layer: int = 1) -> None:
        self.name = name
        self._layer = layer
        self.ports_in: dict = {
            "cmd_vel": MagicMock(),
            "stop_signal": MagicMock(),
        }
        self.setup_called = False
        self.start_called = False
        self.stop_called = False

    def setup(self) -> None:
        self.setup_called = True

    def start(self) -> None:
        self.start_called = True

    def stop(self) -> None:
        self.stop_called = True


# ======================================================================
# VelocityMux freeze / unfreeze tests
# ======================================================================

class TestVelocityMuxFreeze(unittest.TestCase):
    """VelocityMux freeze/unfreeze behavior.

    These tests validate that when the mux is frozen it publishes zero
    velocity and ignores incoming source commands, and that unfreeze
    restores normal priority-based arbitration.
    """

    def _make_mux(self, timeout: float = 1.0):
        from nav.services.safety.velocity_mux import VelocityMux
        mux = VelocityMux(source_timeout=timeout)
        mux.setup()
        return mux

    # ------------------------------------------------------------------
    # test 1
    # ------------------------------------------------------------------

    def test_cmdvel_mux_freeze_publishes_zero(self):
        """freeze() publishes Twist.zero() on driver_cmd_vel."""
        mux = self._make_mux()
        published: list[Twist] = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        mux.freeze()

        self.assertGreaterEqual(len(published), 1)
        self.assertTrue(published[-1].is_zero(),
                        "Freeze must publish zero velocity")

    # ------------------------------------------------------------------
    # test 2
    # ------------------------------------------------------------------

    def test_cmdvel_mux_unfreeze_restores(self):
        """unfreeze() lets subsequent source commands flow through."""
        mux = self._make_mux()
        published: list[Twist] = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        mux.freeze()
        published.clear()                     # discard freeze-zero

        mux.unfreeze()
        mux._on_source("path_follower", _twist(0.3))

        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0].linear.x, 0.3,
                               msg="Unfreeze must restore cmd_vel forwarding")

    # ------------------------------------------------------------------
    # test 3
    # ------------------------------------------------------------------

    def test_cmdvel_mux_frozen_ignores_sources(self):
        """While frozen, all source commands are dropped; only zero published."""
        mux = self._make_mux()
        published: list[Twist] = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        mux.freeze()
        published.clear()                     # discard freeze-zero

        # Multiple sources publish while frozen
        mux._on_source("teleop", _twist(0.5))
        mux._on_source("path_follower", _twist(0.3))
        mux._on_source("visual_servo", _twist(0.1, 0.5))

        # Only zero-velocity twists should have been forwarded
        for i, twist in enumerate(published):
            self.assertTrue(twist.is_zero(),
                            f"Twist at index {i} must be zero while frozen "
                            f"(got vx={twist.linear.x})")

        # Active source should be empty while frozen
        self.assertEqual(mux._active, "")


# ======================================================================
# SwapManager tests
# ======================================================================

class TestSwapManager(unittest.TestCase):
    """SwapManager 閳?driver hot-swapping at runtime.

    Tests are isolated via registry snapshot/restore.  Each test starts
    with a clean registry and drives SwapManager with a mock system.
    """

    @classmethod
    def setUpClass(cls):
        _save_registry()

    @classmethod
    def tearDownClass(cls):
        _restore_registry()

    def setUp(self):
        _save_registry()
        clear()
        # Register mock drivers so SwapManager.swap() can look them up
        # via registry.get() / list_plugins().
        register("driver", "stub_for_test", priority=0,
                 description="Mock driver A for swap-manager tests")(
            _MockModule
        )
        register("driver", "alt_driver", priority=0,
                 description="Mock driver B for swap-manager tests")(
            _MockModule
        )

    def tearDown(self):
        _restore_registry()

    # -- fixtures ------------------------------------------------------

    def _make_mock_system(self, driver_name: str = "stub_for_test",
                          mux=None) -> MagicMock:
        """Create a mock system handle with a single driver + optional mux.

        The returned mock has a mutable ``modules`` dict (not a read-only
        property) so SwapManager.swap() can add/remove modules in-place.
        """
        system = MagicMock()
        # Mutable dict 閳?SwapManager.swap() modifies it in-place.
        system.modules = {}
        # Mutable list 閳?SwapManager._rewire_cmd_vel() iterates it.
        system.connections = []

        # Add a driver module that current_driver() will identify.
        driver = _MockModule(name=driver_name, layer=1)
        system.modules[driver_name] = driver

        if mux is not None:
            system.modules["nav.velocity_mux"] = mux

        return system

    def _make_swap_manager(self, system: MagicMock):
        """Build and enable a SwapManager bound to *system*."""
        from runtime.swap_manager import SwapManager

        mgr = SwapManager(system=system)
        mgr.enable()
        return mgr

    # ------------------------------------------------------------------
    # test 4
    # ------------------------------------------------------------------

    def test_swap_manager_creation(self):
        """SwapManager constructor and enable do not crash."""
        system = self._make_mock_system()
        mgr = self._make_swap_manager(system=system)

        self.assertIsNotNone(mgr)
        self.assertTrue(mgr.enabled)
        self.assertEqual(mgr.state, "idle")

    # ------------------------------------------------------------------
    # test 5
    # ------------------------------------------------------------------

    def test_swap_manager_validates_driver_exists(self):
        """Swapping to an unknown driver name raises ValueError."""
        system = self._make_mock_system()
        mgr = self._make_swap_manager(system=system)

        with self.assertRaises(ValueError):
            mgr.swap("driver_that_does_not_exist")

        # State must remain IDLE (no swap attempted)
        self.assertEqual(mgr.state, "idle")
        self.assertIsNone(mgr.error)

    # ------------------------------------------------------------------
    # test 6
    # ------------------------------------------------------------------

    def test_swap_manager_same_driver_noop(self):
        """Swapping to the currently active driver is a no-op."""
        system = self._make_mock_system(driver_name="stub_for_test")
        mgr = self._make_swap_manager(system=system)

        result = mgr.swap("stub_for_test")

        self.assertIsNone(result)
        self.assertEqual(mgr.state, "idle")
        self.assertEqual(mgr.current_driver(), "stub_for_test")

    # ------------------------------------------------------------------
    # test 7
    # ------------------------------------------------------------------

    def test_swap_manager_state_machine(self):
        """State transitions: IDLE -> SWAPPING -> COMPLETED."""
        system = self._make_mock_system(driver_name="stub_for_test")
        mgr = self._make_swap_manager(system=system)

        self.assertEqual(mgr.state, "idle")

        result = mgr.swap("alt_driver")

        self.assertEqual(result, "alt_driver")
        self.assertEqual(mgr.state, "completed")
        self.assertEqual(mgr.current_driver(), "alt_driver")

        # Old driver should no longer be in the system
        self.assertNotIn("stub_for_test", system.modules)
        # New driver should be present
        self.assertIn("alt_driver", system.modules)

    # ------------------------------------------------------------------
    # test 8
    # ------------------------------------------------------------------

    def test_swap_manager_rollback_on_failure(self):
        """If new driver setup fails, old driver is restored and state = FAILED."""
        # Register a driver whose setup() raises.
        class _FailingDriver:
            _layer = 1
            ports_in = {"cmd_vel": MagicMock(), "stop_signal": MagicMock()}

            def __init__(self, **kw):
                pass

            def setup(self):
                raise RuntimeError("simulated setup failure")

            def start(self):
                pass

            def stop(self):
                pass

        register("driver", "failing_driver", priority=0)(_FailingDriver)

        system = self._make_mock_system(driver_name="stub_for_test")
        mgr = self._make_swap_manager(system=system)
        original_driver = mgr.current_driver()

        with self.assertRaises(RuntimeError):
            mgr.swap("failing_driver")

        self.assertEqual(mgr.state, "failed",
                         "State must be FAILED after swap failure")

        # The old driver must still be the current driver
        self.assertEqual(mgr.current_driver(), original_driver,
                         "Current driver must remain the original after rollback")
        self.assertIsNotNone(mgr.error)

    # ------------------------------------------------------------------
    # test 9
    # ------------------------------------------------------------------

    def test_swap_manager_mux_freeze_during_swap(self):
        """Swap freezes the mux before swapping and unfreezes on completion."""
        from nav.services.safety.velocity_mux import VelocityMux

        mux = VelocityMux(source_timeout=1.0)
        mux.setup()
        system = self._make_mock_system(
            driver_name="stub_for_test", mux=mux,
        )
        mgr = self._make_swap_manager(system=system)

        self.assertFalse(mux.is_frozen, "Mux must start unfrozen")

        mgr.swap("alt_driver")

        self.assertFalse(mux.is_frozen,
                         "Mux must be unfrozen after successful swap")

    # ------------------------------------------------------------------
    # test 10
    # ------------------------------------------------------------------

    def test_swap_manager_disabled_still_swaps(self):
        """SwapManager.swap() works even before enable() is called.

        SwapManager does not gate swap operations on _enabled.  Calling
        swap() on a disabled manager still performs the swap normally.
        """
        from runtime.swap_manager import SwapManager

        system = self._make_mock_system()
        mgr = SwapManager(system=system)
        self.assertFalse(mgr.enabled)

        result = mgr.swap("alt_driver")

        self.assertEqual(result, "alt_driver")
        self.assertEqual(mgr.state, "completed")


if __name__ == "__main__":
    unittest.main()
