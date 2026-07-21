"""Product process control compiled from the active Profile and Endpoint."""

from __future__ import annotations

import os
from collections.abc import Mapping

from lingtu.assembly.profile_builder import Product, compile_product
from lingtu.launcher import LaunchReport, Launcher
from runtime.profiles.resolver import resolve_runtime_config
from runtime.runtime_policy import ServiceTransitionPlan


class ProductControl:
    """Operate the current product without reconstructing process targets."""

    def __init__(
        self,
        launcher: Launcher | None = None,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        self._launcher = launcher
        self._environment = environment if environment is not None else os.environ

    def product(self) -> Product:
        profile = str(self._environment.get("LINGTU_PROFILE") or "").strip()
        if not profile:
            raise RuntimeError("LINGTU_PROFILE is required for product process control")
        endpoint = str(self._environment.get("LINGTU_ENDPOINT") or "").strip() or None
        resolved = resolve_runtime_config(profile, runtime_endpoint_name=endpoint)
        return compile_product(
            resolved.profile,
            resolved.config,
            endpoint=resolved.runtime_endpoint,
        )

    def restart(self, process_name: str, *, dry_run: bool = False) -> LaunchReport:
        launcher = self._launcher or Launcher()
        return launcher.restart(
            self.product(),
            process_name,
            dry_run=dry_run,
        )

    def status(
        self,
        names: tuple[str, ...],
        *,
        dds_check: bool = False,
    ) -> tuple[dict[str, str], dict[str, object]]:
        """Read process health without changing the compiled product."""

        from runtime.service_manager import get_service_manager

        manager = get_service_manager()
        states = manager.status(*names)
        details: dict[str, object] = {}
        if hasattr(manager, "status_details"):
            try:
                details = manager.status_details(*names, dds_check=dds_check)
            except TypeError:
                details = manager.status_details(*names)
        return states, details

    def legacy_transition(
        self,
        plan: ServiceTransitionPlan,
        *,
        timeout_s: float,
    ) -> bool:
        """Apply an explicitly enabled compatibility-only service transition."""

        from runtime.service_manager import get_service_manager

        manager = get_service_manager()
        manager.stop(*plan.stop)
        if plan.ensure:
            manager.ensure(*plan.ensure)
        if not plan.wait_ready:
            return True
        return bool(manager.wait_ready(*plan.wait_ready, timeout=timeout_s))

    def legacy_restart(self, service: str, *, timeout_s: float) -> bool:
        """Restart a compatibility service not owned by a RuntimePlan."""

        from runtime.service_manager import get_service_manager

        manager = get_service_manager()
        manager.stop(service)
        manager.ensure(service)
        return bool(manager.wait_ready(service, timeout=timeout_s))

    def legacy_states(self, names: tuple[str, ...]) -> dict[str, bool]:
        """Read running state for explicitly enabled compatibility services."""

        from runtime.service_manager import get_service_manager

        manager = get_service_manager()
        states: dict[str, bool] = {}
        for name in names:
            try:
                states[name] = bool(manager.is_running(name))
            except Exception:
                states[name] = False
        return states

    def legacy_stop(self, names: tuple[str, ...]) -> None:
        from runtime.service_manager import get_service_manager

        get_service_manager().stop(*names)

    def legacy_ensure(self, names: tuple[str, ...], *, timeout_s: float) -> bool:
        if not names:
            return True
        from runtime.service_manager import get_service_manager

        manager = get_service_manager()
        manager.ensure(*names)
        return bool(manager.wait_ready(*names, timeout=timeout_s))
