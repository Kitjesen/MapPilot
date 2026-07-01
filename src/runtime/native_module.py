"""Legacy ROS2 executable lifecycle wrapper for compatibility adapters.

This is not the product path for planners or autonomy modules. New algorithm
runtime should use in-process kernels (nanobind/Python) behind normal Modules.
Keep NativeModule usage inside explicit ``runtime.adapters.ros2`` adapters that still
need to launch a ROS2 executable and bridge through DDS topics.

Usage::

    from runtime.native_module import NativeModule, NativeModuleConfig

    cfg = NativeModuleConfig(
        executable="/opt/lingtu/nav/install/lib/livox_ros_driver2/livox_ros_driver2_node",
        name="livox_driver",
        parameters={"publish_freq": 10.0},
        remappings={"/livox/lidar": "/nav/lidar_points"},
    )
    mod = NativeModule(cfg)
    # Add only from an explicit runtime.adapters.ros2 adapter.
    bp.add(mod, alias="livox_driver")
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from .module import Module
from .stream import Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# NativeModuleConfig
# ---------------------------------------------------------------------------

@dataclass
class NativeModuleConfig:
    """Configuration for a legacy ROS2 executable managed as a NativeModule.

    Attributes:
        executable: Absolute path to compiled binary.
        name: Human-readable name for logging (defaults to executable basename).
        parameter_files: ROS2 parameter files passed as ``--params-file``.
        parameters: ROS2 parameters passed as ``--ros-args -p key:=value``.
        remappings: ROS2 topic remappings passed as ``-r /old:=/new``.
        env: Extra environment variables for the subprocess.
        shutdown_timeout: Seconds to wait after SIGTERM before SIGKILL.
        auto_restart: Whether to restart the process on unexpected crash.
        max_restarts: Maximum consecutive restart attempts (resets on successful run >10s).
        watchdog_interval: Seconds between health checks in the watchdog loop.
        log_json: If True, parse subprocess stdout as JSON structured logs.
    """
    executable: str
    name: str = ""
    parameter_files: list[str] = field(default_factory=list)
    parameters: dict[str, Any] = field(default_factory=dict)
    remappings: dict[str, str] = field(default_factory=dict)
    env: dict[str, str] = field(default_factory=dict)
    shutdown_timeout: float = 5.0
    auto_restart: bool = False
    max_restarts: int = 3
    watchdog_interval: float = 2.0
    log_json: bool = False

    def __post_init__(self):
        if not self.name:
            self.name = os.path.basename(self.executable)

    def to_ros_args(self) -> list[str]:
        """Generate ``--ros-args -p key:=val -r /old:=/new`` CLI arguments."""
        if not self.parameter_files and not self.parameters and not self.remappings:
            return []
        args = ["--ros-args"]
        for path in self.parameter_files:
            args.extend(["--params-file", str(path)])
        for key, val in self.parameters.items():
            if isinstance(val, bool):
                args.extend(["-p", f"{key}:={'true' if val else 'false'}"])
            elif isinstance(val, (int, float)):
                args.extend(["-p", f"{key}:={val}"])
            else:
                args.extend(["-p", f"{key}:={val}"])
        for old, new in self.remappings.items():
            args.extend(["-r", f"{old}:={new}"])
        return args

    def build_cmd(self) -> list[str]:
        """Full command line: [executable] + ros_args."""
        return [self.executable, *self.to_ros_args()]


# ---------------------------------------------------------------------------
# NativeModule
# ---------------------------------------------------------------------------

class NativeModule(Module):
    """Manage a legacy ROS2 executable as a compatibility Module.

    The external process communicates via DDS topics directly. NativeModule
    does not carry application In/Out data ports except ``alive`` for health
    status. It is purely a compatibility lifecycle wrapper: start, monitor,
    restart, stop.
    """

    alive: Out[bool]

    def __init__(self, config: NativeModuleConfig, **kw):
        super().__init__(**kw)
        self._native_config = config
        self._process: subprocess.Popen | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._log_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()
        self._restart_count = 0
        self._start_time: float = 0.0

    # -- Lifecycle -----------------------------------------------------------

    def setup(self) -> None:
        """Validate that the executable exists."""
        exe = self._native_config.executable
        if not os.path.isfile(exe):
            raise FileNotFoundError(
                f"NativeModule '{self._native_config.name}': "
                f"executable not found: {exe}"
            )
        if not os.access(exe, os.X_OK):
            raise PermissionError(
                f"NativeModule '{self._native_config.name}': "
                f"executable not executable: {exe}"
            )

    def start(self) -> None:
        """Launch the subprocess, start watchdog and log piping."""
        super().start()
        self._shutdown_event.clear()
        self._launch_process()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True,
            name=f"watchdog-{self._native_config.name}")
        self._watchdog_thread.start()
        logger.info(
            "NativeModule '%s' started (pid=%d)",
            self._native_config.name,
            self._process.pid if self._process else -1,
        )

    def stop(self) -> None:
        """Graceful shutdown: SIGTERM 鈫?timeout 鈫?SIGKILL."""
        self._shutdown_event.set()
        self._terminate_process()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=3.0)
        if self._log_thread and self._log_thread.is_alive():
            self._log_thread.join(timeout=2.0)
        self._process = None
        self._watchdog_thread = None
        self._log_thread = None
        self.alive.publish(False)
        super().stop()
        logger.info("NativeModule '%s' stopped", self._native_config.name)

    # -- Process management --------------------------------------------------

    def _launch_process(self) -> None:
        """Spawn the subprocess with correct env and log piping."""
        cmd = self._native_config.build_cmd()
        env = {**os.environ, **self._native_config.env}

        # Use process group on Unix for clean shutdown of child processes
        kwargs: dict[str, Any] = {
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "env": env,
        }
        if os.name != "nt":
            kwargs["preexec_fn"] = os.setsid

        self._process = subprocess.Popen(cmd, **kwargs)
        self._start_time = time.time()
        self.alive.publish(True)

        # Log pipe thread
        self._log_thread = threading.Thread(
            target=self._pipe_logs, daemon=True,
            name=f"logs-{self._native_config.name}")
        self._log_thread.start()

    def _terminate_process(self) -> None:
        """SIGTERM 鈫?wait 鈫?SIGKILL. Handles process groups on Unix."""
        if not self._process or self._process.poll() is not None:
            return

        pid = self._process.pid
        timeout = self._native_config.shutdown_timeout

        # Send SIGTERM (or terminate on Windows)
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(pid), signal.SIGTERM)
            else:
                self._process.terminate()
        except OSError:
            pass

        # Wait for graceful exit
        try:
            self._process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass

        # Escalate to SIGKILL
        logger.warning(
            "NativeModule '%s' (pid=%d) did not exit after %.1fs, sending SIGKILL",
            self._native_config.name, pid, timeout,
        )
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            else:
                self._process.kill()
        except OSError:
            pass

        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            logger.error(
                "NativeModule '%s' (pid=%d) still alive after SIGKILL",
                self._native_config.name, pid,
            )

    # -- Watchdog ------------------------------------------------------------

    def _watchdog_loop(self) -> None:
        """Monitor subprocess health, auto-restart on crash."""
        cfg = self._native_config
        while not self._shutdown_event.is_set():
            if self._process and self._process.poll() is not None:
                exit_code = self._process.returncode
                uptime = time.time() - self._start_time

                # Reset restart counter if process ran long enough (>10s)
                if uptime > 10.0:
                    self._restart_count = 0

                if self._shutdown_event.is_set():
                    return  # graceful shutdown, don't restart

                self.alive.publish(False)
                logger.error(
                    "NativeModule '%s' exited unexpectedly (code=%d, uptime=%.1fs)",
                    cfg.name, exit_code, uptime,
                )

                if cfg.auto_restart and self._restart_count < cfg.max_restarts:
                    self._restart_count += 1
                    logger.warning(
                        "Restarting '%s' (attempt %d/%d)",
                        cfg.name, self._restart_count, cfg.max_restarts,
                    )
                    try:
                        self._launch_process()
                    except Exception:
                        logger.exception("Failed to restart '%s'", cfg.name)
                        return
                else:
                    if self._restart_count >= cfg.max_restarts:
                        logger.error(
                            "NativeModule '%s' exceeded max restarts (%d), giving up",
                            cfg.name, cfg.max_restarts,
                        )
                    return

            self._shutdown_event.wait(timeout=cfg.watchdog_interval)

    # -- Log piping ----------------------------------------------------------

    def _pipe_logs(self) -> None:
        """Read subprocess stdout line-by-line, forward to Python logging."""
        if not self._process or not self._process.stdout:
            return
        stream = self._process.stdout
        name = self._native_config.name

        for raw_line in stream:
            line = raw_line.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue

            if self._native_config.log_json:
                try:
                    data = json.loads(line)
                    event = data.pop("event", line)
                    logger.info("[%s] %s", name, event, extra=data)
                    continue
                except (json.JSONDecodeError, TypeError):
                    pass

            logger.info("[%s] %s", name, line)

        try:
            stream.close()
        except Exception:
            pass

    # -- Health report -------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """Process health: running, pid, restart count, uptime."""
        info = super().port_summary()
        proc_running = self._process is not None and self._process.poll() is None
        info["native"] = {
            "name": self._native_config.name,
            "executable": self._native_config.executable,
            "running": proc_running,
            "pid": self._process.pid if proc_running else None,
            "restart_count": self._restart_count,
            "uptime": time.time() - self._start_time if proc_running else 0.0,
        }
        return info

    def __repr__(self) -> str:
        proc_running = self._process is not None and self._process.poll() is None
        status = "running" if proc_running else "stopped"
        return f"NativeModule('{self._native_config.name}', {status})"
