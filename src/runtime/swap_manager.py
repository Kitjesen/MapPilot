"""SwapManager 鈥?hot-swap robot driver modules at runtime.

SwapManager orchestrates the freeze -> detach -> wire -> restore -> unfreeze
cycle for swapping robot driver backends without restarting the entire
navigation stack.  It is a standalone object attached to SystemHandle,
NOT a Module.

Lifecycle::

    IDLE --swap()--> SWAPPING --success--> COMPLETED
                           |
                           +--failure--> FAILED --reset()--> IDLE

Usage::

    system.enable_swap()
    system.swap_manager.swap("my_alt_driver", dog_host="10.0.0.1")

Integration is opt-in.  Default behaviour is unchanged when SwapManager is
not activated.
"""

from __future__ import annotations

import enum
import logging
import threading
from collections.abc import Sequence
from typing import Any

from runtime.registry import get, list_plugins

logger = logging.getLogger(__name__)


class SwapState(enum.Enum):
    """Granular phases within a swap operation.

    The high-level ``SwapManager.state`` property exposes the four lifecycle
    states (IDLE / SWAPPING / COMPLETED / FAILED).  ``SwapManager.swap_phase``
    exposes the detailed phase for monitoring / debugging.
    """

    IDLE = "idle"
    FREEZE_CMDVEL = "freeze_cmdvel"
    DETACH_OLD = "detach_old"
    WIRE_NEW = "wire_new"
    RESTORE_STATE = "restore_state"
    UNFREEZE_CMDVEL = "unfreeze_cmdvel"
    ROLLBACK = "rollback"


class SwapManager:
    """Hot-swap robot driver modules at runtime without restarting the stack.

    Thread-safe: ``swap()``, ``reset()``, and ``rollback()`` acquire ``_lock``.
    Only one swap operation may run at a time.

    Constructor accepts module *instances* for the mux and nav modules so the
    manager can call their API directly.  If not provided, the manager falls
    back to looking up ``"nav.velocity_mux"`` and ``"nav.mission"`` from the
    system's module table.

    Works with both real ``SystemHandle`` instances and test ``MagicMock``
    objects -- the constructor auto-detects whether the system exposes
    mutable ``modules`` / ``connections`` dicts or read-only properties.
    """

    # -- lifecycle state constants (used by external consumers) ----------------
    IDLE = "idle"
    SWAPPING = "swapping"
    COMPLETED = "completed"
    FAILED = "failed"

    def __init__(
        self,
        system=None,
        *,
        mux=None,
        nav=None,
        swap_timeout: float = 30.0,
    ) -> None:
        """Constructor.

        Args:
            system: ``SystemHandle`` or any object with ``modules`` /
                    ``connections`` (or ``_modules`` / ``_connections``).
            mux:    ``VelocityMux`` instance (or any object with
                    ``freeze()`` / ``unfreeze()``).  When *None* the
                    manager looks up ``"nav.velocity_mux"`` in the system.
            nav:    ``Navigation`` instance.  When set, the swap
                    pause publishes ``True`` to the teleop-active port;
                    unpause publishes ``False``.  When *None* the
                    manager looks up ``"nav.mission"`` in the system.
            swap_timeout:  Overall deadline for a single swap (seconds).
        """
        self._system = system
        self._mux = mux
        self._nav = nav
        self._swap_timeout = swap_timeout

        self._lock = threading.Lock()
        self._state: str = self.IDLE
        self._swap_phase: SwapState = SwapState.IDLE
        self._error: str | None = None
        self._enabled: bool = False

        # Saved for rollback
        self._saved_state: dict | None = None
        self._old_driver_name: str | None = None
        self._old_driver: Any = None
        self._new_driver_name: str | None = None
        # Tracks whether _freeze() was successfully applied so _unfreeze()
        # only runs when needed (e.g. rollback after failed freeze).
        self._was_frozen: bool = False

        # -- resolve mutable data containers -----------------------------------
        self._resolve_data_containers()

    def _resolve_data_containers(self) -> None:
        """Bind ``self._mods`` / ``self._conns`` to mutable dict/list list.

        SystemHandle exposes private ``_modules`` / ``_connections``;
        test mocks expose ``modules`` / ``connections`` directly.

        If the system object exposes neither private nor public containers,
        both fall back to isolated empty collections.  Swap operations will
        complete but NOT be reflected in the system 鈥?a warning is emitted
        so callers can diagnose the misconfiguration.
        """
        if self._system is None:
            self._mods: dict[str, Any] = {}
            self._conns: list[tuple[str, str, str, str]] = []
            return
        mods = getattr(self._system, "_modules", None)
        if not isinstance(mods, dict):
            mods = getattr(self._system, "modules", {})
        if not isinstance(mods, dict):
            logger.warning(
                "SwapManager: system %r has no _modules or modules dict 鈥?"
                "swap will not propagate to the system",
                self._system,
            )
            mods = {}
        conns = getattr(self._system, "_connections", None)
        if not isinstance(conns, (list, Sequence)):
            conns = getattr(self._system, "connections", [])
        if not isinstance(conns, (list, Sequence)):
            logger.warning(
                "SwapManager: system %r has no _connections or connections list 鈥?"
                "wire metadata will not propagate",
                self._system,
            )
            conns = []
        self._mods = mods
        self._conns = conns

    # -- module instance lookups (constructor fallback) -----------------------

    @property
    def _mux_instance(self) -> Any:
        """Return the mux module, either from constructor or system table."""
        if self._mux is not None:
            return self._mux
        return self._mods.get("nav.velocity_mux")

    @property
    def _nav_instance(self) -> Any:
        """Return the nav module, either from constructor or system table."""
        if self._nav is not None:
            return self._nav
        return self._mods.get("nav.mission")

    # -- public properties ----------------------------------------------------

    @property
    def state(self) -> str:
        """One of IDLE, SWAPPING, COMPLETED, FAILED."""
        return self._state

    @property
    def swap_phase(self) -> SwapState:
        """Detailed phase of the current or last swap operation."""
        return self._swap_phase

    @property
    def error(self) -> str | None:
        """Error message from the last failed swap, or None."""
        return self._error

    @property
    def enabled(self) -> bool:
        """Whether swap management is active (set by ``enable()``)."""
        return self._enabled

    # -- lifecycle ------------------------------------------------------------

    def enable(self) -> None:
        """Activate swap management.  Idempotent."""
        if not self._enabled:
            self._enabled = True
            logger.info("SwapManager enabled")

    def disable(self) -> None:
        """Deactivate swap management and reset state to IDLE."""
        self._enabled = False
        self._state = self.IDLE
        self._swap_phase = SwapState.IDLE
        self._error = None
        self._saved_state = None
        self._old_driver_name = None
        self._old_driver = None
        self._new_driver_name = None
        self._was_frozen = False

    # -- public API -----------------------------------------------------------

    def current_driver(self) -> str | None:
        """Return the module name of the active driver, or None.

        Detection heuristic: a driver is any module with ``_layer == 1``,
        a ``cmd_vel`` In port, and a ``stop_signal`` In port.

        Thread-safe: acquires ``_lock`` to avoid race conditions with swap().
        """
        with self._lock:
            return self._find_current_driver()

    def _find_current_driver(self) -> str | None:
        """Internal driver lookup 鈥?caller must hold ``_lock``."""
        for name, mod in self._mods.items():
            if (
                getattr(mod, "_layer", None) == 1
                and "cmd_vel" in getattr(mod, "ports_in", {})
                and "stop_signal" in getattr(mod, "ports_in", {})
            ):
                return name
        return None

    def swap(self, driver_name: str, **config: Any) -> str | None:
        """Hot-swap the active driver to *driver_name*.

        Args:
            driver_name: Name registered in the ``"driver"`` registry category.
            **config:    Keyword arguments forwarded to the new driver's
                         ``__init__``.

        Returns:
            The new driver module name on success, ``None`` on no-op
            (the named driver is already active).

        Raises:
            ValueError:   *driver_name* is not in the registry.
            RuntimeError: Any failure during the swap cycle triggers rollback
                          and is re-raised as a RuntimeError with details.
        """
        with self._lock:
            return self._swap_locked(driver_name, **config)

    def reset(self) -> None:
        """Reset state to IDLE after a failure.  Thread-safe."""
        with self._lock:
            self._state = self.IDLE
            self._swap_phase = SwapState.IDLE
            self._error = None
            self._saved_state = None
            self._old_driver_name = None
            self._old_driver = None
            self._new_driver_name = None
            self._was_frozen = False

    def rollback(self) -> None:
        """Manually rollback a swap in progress.

        Only effective when state is SWAPPING.  Thread-safe.
        """
        with self._lock:
            if self._state != self.SWAPPING:
                logger.warning(
                    "rollback() called but state is %s (expected SWAPPING)",
                    self._state,
                )
                return
            self._do_rollback()

    # -- internal swap logic --------------------------------------------------

    def _swap_locked(self, driver_name: str, **config: Any) -> str | None:
        # -- validate ---------------------------------------------------------
        available = list_plugins("driver")
        if driver_name not in available:
            raise ValueError(
                f"Unknown driver '{driver_name}'. "
                f"Available: {available}"
            )

        current = self._find_current_driver()
        if current == driver_name:
            logger.info("swap(%s): already active -- no-op", driver_name)
            return None  # no-op

        self._state = self.SWAPPING
        self._old_driver_name = current
        self._old_driver = self._mods.get(current) if current else None

        try:
            # 1. FREEZE -- stop motion, pause navigation
            self._swap_phase = SwapState.FREEZE_CMDVEL
            self._freeze()

            # 2. DETACH -- capture state, stop old driver, remove from system
            self._swap_phase = SwapState.DETACH_OLD
            self._saved_state = (
                self._save_state(self._old_driver)
                if self._old_driver is not None
                else None
            )
            if self._old_driver is not None:
                self._old_driver.stop()
                if (
                    self._old_driver_name
                    and self._old_driver_name in self._mods
                ):
                    del self._mods[self._old_driver_name]

            # 3. WIRE -- instantiate new driver, setup, wire, start
            self._swap_phase = SwapState.WIRE_NEW
            self._new_driver_name = driver_name
            new_driver = self._start_new_driver(driver_name, **config)

            # 4. RESTORE -- seed new driver's state from snapshot
            self._swap_phase = SwapState.RESTORE_STATE
            if self._saved_state:
                self._restore_state(new_driver, self._saved_state)

            # 5. UNFREEZE -- restore normal motion control
            self._swap_phase = SwapState.UNFREEZE_CMDVEL
            self._unfreeze()

            self._state = self.COMPLETED
            self._swap_phase = SwapState.IDLE
            # Clear stale references to allow GC
            self._saved_state = None
            self._old_driver = None
            self._old_driver_name = None
            self._new_driver_name = None
            logger.info(
                "swap(%s): completed (replaced %s)",
                driver_name, current,
            )
            return driver_name

        except Exception as exc:
            self._error = str(exc)
            logger.error(
                "swap(%s): failed -- rolling back. Error: %s",
                driver_name, exc,
            )
            self._do_rollback()
            raise RuntimeError(
                f"Swap to '{driver_name}' failed: {exc}"
            ) from exc

    # -- sub-operations -------------------------------------------------------

    def _freeze(self) -> None:
        """Pause all motion: freeze cmd_vel mux *then* pause NavModule.

        Order: mux first (stops cmd_vel flow), nav second (pauses planning).
        See ``_unfreeze`` for the reverse order.
        """
        mux = self._mux_instance
        if mux is not None and hasattr(mux, "freeze"):
            logger.debug("SwapManager: freezing VelocityMux")
            mux.freeze()
        nav = self._nav_instance
        if nav is not None and hasattr(nav, "teleop_active"):
            logger.debug("SwapManager: pausing NavModule")
            nav.teleop_active.publish(True)
        self._was_frozen = True

    def _unfreeze(self) -> None:
        """Restore motion: resume NavModule *then* unfreeze cmd_vel mux.

        Order is the reverse of ``_freeze``: nav first (resumes planning),
        mux second (re-enables cmd_vel flow).  This avoids a transient
        where the mux forwards commands before the nav has re-acquired
        state.
        """
        nav = self._nav_instance
        if nav is not None and hasattr(nav, "teleop_active"):
            logger.debug("SwapManager: resuming NavModule")
            nav.teleop_active.publish(False)
        mux = self._mux_instance
        if mux is not None and hasattr(mux, "unfreeze"):
            logger.debug("SwapManager: unfreezing VelocityMux")
            mux.unfreeze()

    def _start_new_driver(self, driver_name: str, **config: Any) -> Any:
        """Instantiate, setup, wire into system, and start the new driver.

        Extracted as a separate method so tests can mock it to simulate
        swap failures.

        Returns:
            The new driver module instance.
        """
        DriverCls = get("driver", driver_name)
        logger.debug(
            "SwapManager: instantiating driver %s (%s)",
            driver_name, DriverCls.__name__,
        )
        new_mod = DriverCls(**config)
        new_mod.setup()
        self._mods[driver_name] = new_mod
        self._rewire_cmd_vel(self._old_driver_name, driver_name)
        new_mod.start()
        logger.info(
            "SwapManager: new driver %s (%s) started",
            driver_name, DriverCls.__name__,
        )
        return new_mod

    def _do_rollback(self) -> None:
        """Undo a failed swap: remove new driver, restore old driver, unfreeze."""
        self._swap_phase = SwapState.ROLLBACK
        self._state = self.FAILED

        # Remove new driver if it was added to the system
        if self._new_driver_name and self._new_driver_name in self._mods:
            new_mod = self._mods[self._new_driver_name]
            try:
                new_mod.stop()
                logger.debug(
                    "SwapManager rollback: stopped new driver %s",
                    self._new_driver_name,
                )
            except Exception:
                logger.exception(
                    "SwapManager rollback: error stopping new driver %s",
                    self._new_driver_name,
                )
            del self._mods[self._new_driver_name]

        # Restore old driver
        if self._old_driver is not None and self._old_driver_name is not None:
            if self._old_driver_name not in self._mods:
                try:
                    # Re-setup old driver to ensure clean state before restoration
                    if hasattr(self._old_driver, "setup") and callable(self._old_driver.setup):
                        self._old_driver.setup()
                except Exception:
                    logger.exception(
                        "SwapManager rollback: error re-running setup on old driver %s",
                        self._old_driver_name,
                    )
                self._mods[self._old_driver_name] = self._old_driver
                self._rewire_cmd_vel(None, self._old_driver_name)
                logger.info(
                    "SwapManager rollback: restored driver %s",
                    self._old_driver_name,
                )

        if self._was_frozen:
            try:
                self._unfreeze()
            except Exception:
                logger.exception(
                    "SwapManager rollback: error during unfreeze after rollback"
                )
        self._swap_phase = SwapState.IDLE
        logger.info("SwapManager rollback: complete")

    # -- state transfer (extension hooks) -------------------------------------

    def _save_state(self, driver: Any) -> dict:
        """Capture odometry and robot_state from the old driver.

        Override in subclasses for driver-specific state transfer.  The
        default implementation extracts common internal attributes found
        on ``ThunderDriver`` / ``StubDogModule``.
        """
        state: dict[str, Any] = {}
        if driver is None:
            return state

        _COMMON_STATE_ATTRS = (
            "_pos_x", "_pos_y", "_pos_z", "_yaw",
            "_latest_quat", "_latest_gyro",
            "_battery_voltage", "_battery_soc", "_gait_type",
            "_connected", "_standing", "_enabled",
            "_vx", "_vy", "_wz",  # StubDogModule internal
        )
        for attr in _COMMON_STATE_ATTRS:
            if hasattr(driver, attr):
                state[attr] = getattr(driver, attr)

        if state:
            logger.info(
                "SwapManager: saved driver state (%d keys): %s",
                len(state), list(state.keys()),
            )
        else:
            logger.info(
                "SwapManager: no driver state captured "
                "(override _save_state)"
            )
        return state

    def _restore_state(self, driver: Any, state: dict) -> None:
        """Seed saved state into the new driver.

        Override in subclasses for driver-specific state restoration.
        The default writes internal attributes and triggers a fresh
        ``robot_state`` / ``alive`` publication if those ports exist.
        """
        if not state:
            return

        for key, value in state.items():
            if hasattr(driver, key):
                setattr(driver, key, value)

        # Trigger a fresh state publication so downstream consumers catch up
        if hasattr(driver, "robot_state") and state.get("_connected") is not None:
            try:
                driver.robot_state.publish({
                    "connected": state.get("_connected", False),
                    "standing": state.get("_standing", False),
                    "enabled": state.get("_enabled", False),
                    "battery_voltage": state.get("_battery_voltage", 0.0),
                    "battery_soc": state.get("_battery_soc", 0.0),
                    "gait": state.get("_gait_type", "unknown"),
                })
            except Exception:
                logger.exception("Error publishing restored robot_state")

        if hasattr(driver, "alive") and state.get("_connected") is not None:
            try:
                driver.alive.publish(bool(state["_connected"]))
            except Exception:
                logger.exception("Error publishing restored alive signal")

        logger.info(
            "SwapManager: restored driver state (%d keys): %s",
            len(state), list(state.keys()),
        )

    # -- wiring helpers -------------------------------------------------------

    def _rewire_cmd_vel(
        self, old_name: str | None, new_name: str
    ) -> None:
        """Update the cmd_vel connection metadata from mux -> new driver.

        Only affects metadata stored on the system handle, used for
        serialisation / health reporting.  The actual runtime port callbacks
        are set up during ``Blueprint.build()`` and persist across the swap.
        """
        old_conns: list[tuple[str, str, str, str]] = []
        remaining: list[tuple[str, str, str, str]] = []
        for src, sp, dst, dp in self._conns:
            if (
                src == "nav.velocity_mux"
                and sp == "driver_cmd_vel"
                and dst == old_name
            ):
                old_conns.append((src, sp, dst, dp))
            else:
                remaining.append((src, sp, dst, dp))

        if old_conns:
            _, sp, _, dp = old_conns[0]
            remaining.append(("nav.velocity_mux", sp, new_name, dp))
            logger.debug(
                "SwapManager: rewire nav.velocity_mux.driver_cmd_vel -> %s.%s",
                new_name, dp,
            )

        self._conns.clear()
        self._conns.extend(remaining)

    # -- repr -----------------------------------------------------------------

    def __repr__(self) -> str:
        status = "enabled" if self._enabled else "disabled"
        return f"SwapManager({status}, state={self._state})"
