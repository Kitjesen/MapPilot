"""ServiceManager - start/stop robot systemd services on demand.

Industrial pattern: only run what you need, release when done.

Usage:
    svc = ServiceManager()
    svc.start("lidar", "slam")   # pull up when needed
    svc.stop("lidar", "slam")    # release when done
    svc.ensure("lidar")          # start only if not running
"""

from __future__ import annotations

import logging
import subprocess
import time

logger = logging.getLogger(__name__)


def _subprocess_text(value: bytes | str | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value or ""


SERVICE_ALIASES: dict[str, tuple[str, ...]] = {
    "lidar": ("robot-lidar.service", "lidar.service", "lidar"),
    "camera": ("robot-camera.service", "camera.service", "camera"),
    "brainstem": ("robot-brainstem.service", "brainstem.service", "brainstem"),
    "slam": ("robot-fastlio2.service", "localization.service", "slam"),
    "slam_pgo": ("robot-pgo.service", "slam_pgo.service", "slam_pgo"),
    "localizer": ("robot-localizer.service", "localizer.service", "localizer"),
    "super_lio": (
        "robot-super-lio.service",
        "super_lio.service",
        "super-lio.service",
        "super_lio",
    ),
    "super_lio_relocation": (
        "robot-super-lio-relocation.service",
        "robot-super-lio-reloc.service",
        "super_lio_relocation.service",
        "super_lio_reloc.service",
        "super-lio-relocation.service",
        "super_lio_relocation",
        "super_lio_reloc",
    ),
}


class ServiceManager:
    """Manage systemd services using stable logical robot service names."""

    def __init__(self):
        self._started: list[str] = []  # concrete units we started

    def _candidate_units(self, service: str) -> tuple[str, ...]:
        return SERVICE_ALIASES.get(service, (service,))

    def _is_active_unit(self, unit: str) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "is-active", "--quiet", unit],
                timeout=3,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            if result.returncode == 0:
                return True
            # returncode != 0 is expected for inactive/missing services
            return False
        except subprocess.TimeoutExpired:
            logger.warning("ServiceManager: systemctl is-active %s timed out", unit)
            return False
        except FileNotFoundError:
            logger.warning("ServiceManager: systemctl not found (not a systemd system?)")
            return False
        except Exception as e:
            logger.error("ServiceManager: _is_active_unit(%s) unexpected error: %s", unit, e)
            return False

    def _unit_exists(self, unit: str) -> bool:
        try:
            result = subprocess.run(
                ["systemctl", "show", "-p", "LoadState", "--value", unit],
                timeout=3,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
        except subprocess.TimeoutExpired:
            logger.warning("ServiceManager: systemctl show %s timed out", unit)
            return False
        except FileNotFoundError:
            logger.warning("ServiceManager: systemctl not found (not a systemd system?)")
            return False
        except Exception as e:
            logger.error("ServiceManager: _unit_exists(%s) unexpected error: %s", unit, e)
            return False
        if result.returncode != 0:
            return False
        load_state = (result.stdout or "").strip()
        return bool(load_state) and load_state not in {"not-found", "masked"}

    def _resolve_start_unit(self, service: str) -> str:
        candidates = self._candidate_units(service)
        for unit in candidates:
            if self._unit_exists(unit):
                return unit
        return candidates[0]

    def _forget_started(self, *units: str) -> None:
        for unit in units:
            while unit in self._started:
                self._started.remove(unit)

    def is_running(self, service: str) -> bool:
        """Check whether a logical service is active.

        When the canonical robot-* unit exists, it is the source of truth. A
        stale legacy alias such as camera.service should not make the manager
        believe the camera stack is healthy or skip starting robot-camera.
        """
        candidates = self._candidate_units(service)
        if not candidates:
            return False
        canonical = candidates[0]
        if self._unit_exists(canonical):
            return self._is_active_unit(canonical)
        return any(self._is_active_unit(unit) for unit in candidates)

    def start(self, *services: str, track_started: bool = True) -> list[str]:
        """Start services. Returns list of actually started logical services."""
        started = []
        for service in services:
            if self.is_running(service):
                logger.debug("Service %s already running", service)
                continue
            unit = self._resolve_start_unit(service)
            try:
                subprocess.run(
                    ["sudo", "systemctl", "start", unit],
                    timeout=10,
                    check=True,
                    capture_output=True,
                )
                if track_started:
                    self._started.append(unit)
                started.append(service)
                logger.info("Started service %s via unit %s", service, unit)
            except subprocess.CalledProcessError as e:
                stderr = _subprocess_text(e.stderr)
                logger.error("Failed to start %s: %s", service, str(stderr)[:100])
            except Exception as e:
                logger.error("Failed to start %s: %s", service, e)
        return started

    def stop(self, *services: str) -> None:
        """Stop services.

        Logical names stop every known concrete alias. This intentionally clears
        old and new robot units together so profile switching cannot leave a
        legacy service racing the current robot-* service.
        """
        for service in services:
            for unit in self._candidate_units(service):
                try:
                    subprocess.run(
                        ["sudo", "systemctl", "stop", unit],
                        timeout=10,
                        capture_output=True,
                    )
                    self._forget_started(unit, service)
                    logger.info("Stopped service %s via unit %s", service, unit)
                except Exception as e:
                    logger.warning("Failed to stop %s (%s): %s", service, unit, e)

    def ensure(self, *services: str, track_started: bool = True) -> None:
        """Ensure services are running (start if not)."""
        self.start(*services, track_started=track_started)

    def stop_all_started(self) -> None:
        """Stop all services that this manager started."""
        for unit in list(self._started):
            self.stop(unit)

    def status(self, *services: str) -> dict:
        """Batch query service status. Returns {name: "running"|"stopped"}."""
        return {
            svc: "running" if self.is_running(svc) else "stopped"
            for svc in services
        }

    def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
        """Wait until all services are active."""
        deadline = time.time() + timeout
        pending = list(services)
        while pending and time.time() < deadline:
            pending = [s for s in pending if not self.is_running(s)]
            if pending:
                time.sleep(0.5)
        if pending:
            logger.warning("Services not ready after %.0fs: %s", timeout, pending)
            return False
        return True


# Singleton
_manager = ServiceManager()


def get_service_manager() -> ServiceManager:
    return _manager


# Service groups: what each mode needs.
SERVICES_LIDAR = ["lidar"]
SERVICES_SLAM = ["slam"]
SERVICES_SLAM_MAPPING = ["slam", "slam_pgo"]
SERVICES_SLAM_NAV = ["slam", "localizer"]
SERVICES_SUPER_LIO = ["lidar", "super_lio"]
SERVICES_SUPER_LIO_RELOCATION = ["lidar", "super_lio_relocation"]
SERVICES_CAMERA = ["camera"]
SERVICES_HARDWARE = SERVICES_LIDAR + SERVICES_CAMERA
