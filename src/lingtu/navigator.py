"""Navigator — go anywhere with one line."""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def _default_map_dir() -> str:
    """Resolve map dir: $NAV_MAP_DIR → existing ~/data/nova/maps → ~/data/lingtu/maps."""
    import os
    env = os.environ.get("NAV_MAP_DIR")
    if env:
        return env
    nova = os.path.expanduser("~/data/nova/maps")
    if os.path.isdir(nova):
        return nova
    return os.path.expanduser("~/data/lingtu/maps")


class Navigator:
    """Autonomous navigation — plan and track paths.

    Usage::

        nav = Navigator(slam)
        nav.start()
        nav.go("体育馆")                # semantic navigation
        nav.go_to(5.0, 3.0)            # coordinate navigation
        nav.status()                    # "IDLE" / "EXECUTING" / "SUCCESS"
        nav.stop()
    """

    def __init__(self, slam=None, planner: str = "astar", tomogram: str = ""):
        self._slam = slam
        self._planner = planner
        self._tomogram = tomogram
        self._nav_module = None
        self._planner_module = None
        self._started = False

    def start(self) -> Navigator:
        if self._started:
            return self
        try:
            from nav.navigation_module import NavigationModule
            self._nav_module = NavigationModule(
                planner=self._planner,
                tomogram=self._tomogram,
            )
            self._nav_module.setup()
            self._nav_module.start()

            # Wire SLAM odometry if available
            if self._slam and self._slam._bridge:
                self._slam._bridge.odometry._add_callback(
                    self._nav_module.odometry._deliver)

            self._started = True
            logger.info("Navigator started (planner=%s)", self._planner)
        except Exception as e:
            logger.error("Navigator start failed: %s", e)
        return self

    def stop(self) -> None:
        if self._nav_module:
            self._nav_module.stop()
        self._started = False

    def go(self, instruction: str) -> str:
        """Semantic navigation — natural language instruction."""
        if not self._nav_module:
            return "Navigator not started"
        self._nav_module.instruction._deliver(instruction)
        return "Navigating: %s" % instruction

    def go_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """Navigate to map coordinates."""
        if not self._nav_module:
            return "Navigator not started"
        return self._nav_module.navigate_to(x, y, yaw)

    def stop_motion(self) -> str:
        """Emergency stop."""
        if self._nav_module:
            return self._nav_module.stop_navigation()
        return "Not running"

    def cancel(self) -> str:
        """Cancel current mission."""
        if self._nav_module:
            return self._nav_module.cancel_mission()
        return "Not running"

    def status(self) -> str:
        """Current navigation state."""
        if self._nav_module:
            return self._nav_module._state
        return "NOT_STARTED"

    def use_map(self, name: str) -> None:
        """Switch to a named map."""
        import os
        map_dir = _default_map_dir()
        tomogram = os.path.join(map_dir, name, "tomogram.pickle")
        if os.path.exists(tomogram) and self._nav_module:
            self._nav_module._planner_svc._backend._load_tomogram(tomogram)
            # Update active symlink
            active_link = os.path.join(map_dir, "active")
            if os.path.islink(active_link):
                os.unlink(active_link)
            os.symlink(os.path.join(map_dir, name), active_link)
            logger.info("Using map: %s", name)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def __repr__(self):
        return "Navigator(state=%s)" % self.status()
