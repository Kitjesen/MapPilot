"""ServiceManager - start/stop robot systemd services on demand.

Industrial pattern: only run what you need, release when done.

Usage:
    svc = ServiceManager()
    svc.start("lidar", "slam")   # pull up when needed
    svc.stop("lidar", "slam")    # release when done
    svc.ensure("lidar")          # start only if not running
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

from runtime.service_catalogs.thunder import (
    thunder_service_aliases,
    thunder_service_metadata,
    thunder_service_start_aliases,
)

logger = logging.getLogger(__name__)


def _subprocess_text(value: bytes | str | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value or ""


SERVICE_ALIASES: dict[str, tuple[str, ...]] = thunder_service_aliases()

SERVICE_START_ALIASES: dict[str, tuple[str, ...]] = thunder_service_start_aliases()

SERVICE_METADATA: dict[str, dict[str, object]] = thunder_service_metadata()


class ServiceManager:
    """Manage systemd services using stable logical robot service names."""

    def __init__(self):
        self._started: list[str] = []  # concrete units we started

    def _candidate_units(self, service: str) -> tuple[str, ...]:
        return SERVICE_ALIASES.get(service, (service,))

    def _start_candidate_units(self, service: str) -> tuple[str, ...]:
        return SERVICE_START_ALIASES.get(service, self._candidate_units(service))

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
        candidates = self._start_candidate_units(service)
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

        The first installed unit in the catalog order is the source of truth.
        A stale legacy alias such as camera.service should not make the manager
        believe the camera stack is healthy or skip starting the product unit.
        """
        candidates = self._candidate_units(service)
        if not candidates:
            return False
        for unit in candidates:
            if self._unit_exists(unit):
                return self._is_active_unit(unit)
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
        return {svc: "running" if self.is_running(svc) else "stopped" for svc in services}

    def status_details(
        self,
        *services: str,
        dds_check: bool | None = None,
    ) -> dict[str, dict[str, object]]:
        """Return logical service status plus concrete systemd unit evidence."""
        details: dict[str, dict[str, object]] = {}
        for service in services:
            candidates = self._candidate_units(service)
            canonical = candidates[0] if candidates else service
            installed = [unit for unit in candidates if self._unit_exists(unit)]
            active = [unit for unit in candidates if self._is_active_unit(unit)]
            if installed:
                running = installed[0] in active
            else:
                running = bool(active)
            metadata = SERVICE_METADATA.get(service, {})
            checks = list(metadata.get("checks", ["systemd"]))
            files = list(metadata.get("files", []))
            binaries = list(metadata.get("binaries", []))
            blockers: list[str] = []
            if "systemd" in checks and not running:
                if installed:
                    blockers.append("systemd_inactive")
                else:
                    blockers.append(f"systemd_unit_missing:{canonical}")
            observed: dict[str, object] = {"systemd": running}
            if "native_binary" in checks:
                native_binary = self._native_binary_observation(binaries)
                observed["native_binary"] = native_binary
                if not native_binary["ok"]:
                    blockers.extend(native_binary["blockers"])
            if "status_file" in checks:
                status_file = self._status_file_observation(files)
                status_contract_blockers: list[str] = []
                max_age_s = metadata.get("status_max_age_s")
                for item in status_file.get("files", []):
                    if not isinstance(item, dict):
                        continue
                    file_name = str(item.get("path") or "")
                    if item.get("ready") is False:
                        status_contract_blockers.append(f"status_file_not_ready:{file_name}")
                    if (
                        isinstance(max_age_s, (int, float))
                        and isinstance(item.get("age_s"), (int, float))
                        and float(item["age_s"]) > float(max_age_s)
                    ):
                        status_contract_blockers.append(f"status_file_stale:{file_name}")
                if status_contract_blockers:
                    status_file["ok"] = False
                    status_file.setdefault("blockers", []).extend(status_contract_blockers)
                observed["status_file"] = status_file
                if not status_file["ok"]:
                    blockers.extend(status_file["blockers"])
            if "dds" in checks:
                dds = self._dds_topic_observation(
                    list(metadata.get("topics", [])),
                    list(metadata.get("dds_topics", [])),
                    enabled_override=dds_check,
                )
                observed["dds"] = dds
                if not dds["ok"]:
                    blockers.extend(dds["blockers"])
            if "http" in checks:
                http = self._http_observation(service)
                observed["http"] = http
                if not http["ok"]:
                    blockers.extend(http["blockers"])
            details[service] = {
                "status": "running" if running else "stopped",
                "canonical_unit": canonical,
                "selected_unit": active[0] if active else (installed[0] if installed else canonical),
                "installed_units": installed,
                "active_units": active,
                "candidate_units": list(candidates),
                "role": metadata.get("role"),
                "group": metadata.get("group"),
                "product_default": metadata.get("product_default", False),
                "optional": metadata.get("optional", False),
                "ros2_compat": metadata.get("ros2_compat", False),
                "experimental": metadata.get("experimental", False),
                "ready": not blockers,
                "blockers": blockers,
                "observed": observed,
                "contract": {
                    "checks": checks,
                    "topics": list(metadata.get("topics", [])),
                    "dds_topics": list(metadata.get("dds_topics", [])),
                    "shm_topics": list(metadata.get("shm_topics", [])),
                    "shm_channels": list(metadata.get("shm_channels", [])),
                    "files": files,
                    "binaries": binaries,
                },
            }
        return details

    def _native_binary_observation(self, binaries: list[object]) -> dict[str, object]:
        observations: list[dict[str, object]] = []
        blockers: list[str] = []
        for item in binaries:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "")
            env_var = str(item.get("env") or "")
            default_path = str(item.get("path") or "")
            raw_path = os.environ.get(env_var, default_path) if env_var else default_path
            path = Path(raw_path)
            exists = path.exists()
            executable = exists and os.access(path, os.X_OK)
            observation = {
                "name": name,
                "env": env_var,
                "path": raw_path,
                "exists": exists,
                "executable": executable,
            }
            if not executable:
                blockers.append(f"native_binary_missing_or_not_executable:{name}:{raw_path}")
            observations.append(observation)
        return {
            "ok": not blockers,
            "binaries": observations,
            "blockers": blockers,
        }

    def _status_file_observation(self, files: list[str]) -> dict[str, object]:
        observations: list[dict[str, object]] = []
        blockers: list[str] = []
        for file_name in files:
            path = Path(file_name)
            item: dict[str, object] = {"path": file_name, "exists": path.exists()}
            if not path.exists():
                blockers.append(f"status_file_missing:{file_name}")
                observations.append(item)
                continue
            try:
                stat = path.stat()
                item["mtime"] = stat.st_mtime
                item["age_s"] = max(0.0, time.time() - stat.st_mtime)
            except OSError as exc:
                item["stat_error"] = str(exc)
                blockers.append(f"status_file_unreadable:{file_name}")
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(payload, dict):
                    status = str(payload.get("status") or "").strip().lower()
                    item["status"] = status
                    if isinstance(payload.get("ready"), bool):
                        item["ready"] = payload["ready"]
                    schema = payload.get("schema") or payload.get("schema_version")
                    if schema:
                        item["schema"] = schema
                    if status in {"error", "failed", "fatal"}:
                        blockers.append(f"status_file_error:{file_name}")
            except (OSError, json.JSONDecodeError) as exc:
                item["read_error"] = str(exc)
                blockers.append(f"status_file_unreadable:{file_name}")
            observations.append(item)
        return {
            "ok": not blockers,
            "files": observations,
            "blockers": blockers,
        }

    def _dds_topic_observation(
        self,
        topics: list[str],
        dds_topics: list[str],
        *,
        enabled_override: bool | None = None,
    ) -> dict[str, object]:
        """Optionally sample DDS topics for readiness.

        Python CycloneDDS is optional on the robot. By default this reports the
        declared DDS contract as unchecked evidence. A service that declares a
        DDS check is not ready until field checks set LINGTU_SERVICE_DDS_CHECK=1
        and at least one sample is observed per topic.
        """
        if not topics and not dds_topics:
            return {
                "ok": True,
                "checked": False,
                "enabled": False,
                "topics": [],
                "samples": {},
                "blockers": [],
            }
        if enabled_override is False:
            return {
                "ok": True,
                "checked": False,
                "enabled": False,
                "deferred": True,
                "reason": "set dds_check=1 to sample DDS topics",
                "topics": dds_topics,
                "samples": {},
                "blockers": [],
            }
        if enabled_override is not True and os.environ.get("LINGTU_SERVICE_DDS_CHECK", "").strip().lower() not in {
            "1",
            "true",
            "yes",
        }:
            return {
                "ok": False,
                "checked": False,
                "enabled": False,
                "reason": "set LINGTU_SERVICE_DDS_CHECK=1 to sample DDS topics",
                "topics": dds_topics,
                "samples": {},
                "blockers": ["dds_unchecked"],
            }

        timeout_s = float(os.environ.get("LINGTU_SERVICE_DDS_CHECK_TIMEOUT", "2.0") or 2.0)
        probe_topics = dds_topics or topics
        samples: dict[str, int] = {topic: 0 for topic in probe_topics}
        blockers: list[str] = []
        try:
            root = Path(__file__).resolve().parents[2]
            probe_script = Path(
                os.environ.get(
                    "LINGTU_DDS_PROBE_SCRIPT",
                    str(root / "scripts" / "diagnostics" / "dds_probe.py"),
                )
            )
            command = [
                sys.executable,
                str(probe_script),
                "--json",
                "--seconds",
                str(max(0.1, timeout_s)),
                "--domain",
                os.environ.get("LINGTU_DDS_DOMAIN_ID", "0"),
                *probe_topics,
            ]
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=str(root),
            )
            if result.returncode not in (0, 1):
                stderr = (result.stderr or "").strip()
                blockers.append(stderr or f"dds_probe_failed:{result.returncode}")
            else:
                rows = json.loads(result.stdout or "[]")
                for row in rows:
                    name = str(row.get("topic") or "")
                    samples[name] = int(row.get("samples", 0) or 0)
                for name in probe_topics:
                    if samples.get(name, 0) <= 0:
                        blockers.append(f"dds_topic_silent:{name}")
        except Exception as exc:
            blockers.append(f"dds_check_error:{type(exc).__name__}")
        return {
            "ok": not blockers,
            "checked": True,
            "enabled": True,
            "topics": probe_topics,
            "samples": samples,
            "blockers": blockers,
        }

    def _http_observation(self, service: str) -> dict[str, object]:
        """Optionally probe HTTP readiness for Gateway-owned services."""
        if os.environ.get("LINGTU_SERVICE_HTTP_CHECK", "").strip().lower() not in {
            "1",
            "true",
            "yes",
        }:
            return {
                "ok": False,
                "checked": False,
                "enabled": False,
                "reason": "set LINGTU_SERVICE_HTTP_CHECK=1 to probe HTTP readiness",
                "url": os.environ.get("LINGTU_SERVICE_HTTP_URL", "http://127.0.0.1:5050/health"),
                "blockers": ["http_unchecked"],
            }

        import urllib.error
        import urllib.request

        url = os.environ.get("LINGTU_SERVICE_HTTP_URL", "http://127.0.0.1:5050/health")
        timeout_s = float(os.environ.get("LINGTU_SERVICE_HTTP_CHECK_TIMEOUT", "1.0") or 1.0)
        blockers: list[str] = []
        status_code: int | None = None
        try:
            with urllib.request.urlopen(url, timeout=max(0.1, timeout_s)) as response:
                status_code = int(getattr(response, "status", 0) or 0)
        except urllib.error.HTTPError as exc:
            status_code = int(exc.code)
            blockers.append(f"http_status:{exc.code}")
        except Exception as exc:
            blockers.append(f"http_check_error:{type(exc).__name__}")

        if status_code is not None and not (200 <= status_code < 300) and f"http_status:{status_code}" not in blockers:
            blockers.append(f"http_status:{status_code}")
        return {
            "ok": not blockers,
            "checked": True,
            "enabled": True,
            "service": service,
            "url": url,
            "status_code": status_code,
            "blockers": blockers,
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
SERVICES_SLAM_MAPPING = ["slam"]
SERVICES_SLAM_NAV = ["slam"]
SERVICES_TRAVERSABILITY = ["traversability"]
SERVICES_NAV = ["nav"]
SERVICES_EXPLORE = ["explore"]
SERVICES_SUPER_LIO = ["lidar", "super_lio"]
SERVICES_SUPER_LIO_RELOCATION = ["lidar", "super_lio_relocation"]
SERVICES_CAMERA = ["camera"]
SERVICES_GATEWAY = ["gateway"]
SERVICES_LINGTU = ["lingtu"]
SERVICES_HARDWARE = SERVICES_LIDAR + SERVICES_CAMERA
