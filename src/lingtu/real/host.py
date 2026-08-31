"""Internal Host entry point for Product-owned real runs."""

from __future__ import annotations

import os
import signal
import sys
import threading
from collections.abc import Mapping, Sequence
from typing import Any

from lingtu.run_plan import RunPlan


def run(stop_event: threading.Event | None = None) -> int:
    """Build and run the Host from the RunPlan published by ProductControl."""

    event = stop_event if stop_event is not None else threading.Event()
    plan = _load_plan(os.environ)
    system = plan.build()
    try:
        gateway = _module(system, "GatewayModule")
        if gateway is not None and callable(getattr(gateway, "_run_server", None)):
            gateway._defer_server = True

        system.start()

        if gateway is None or not callable(getattr(gateway, "_run_server", None)):
            event.wait()
            return 0

        stopped_cleanly = bool(gateway._run_server(event))
        return 0 if stopped_cleanly or event.is_set() else 1
    finally:
        system.stop()


def main(argv: Sequence[str] | None = None) -> int:
    """Run the real Host managed by systemd."""

    args = tuple(sys.argv[1:] if argv is None else argv)
    if args:
        raise SystemExit("lingtu.real.host does not accept CLI arguments")
    stop_event = threading.Event()
    _install_signal_handlers(stop_event)
    return run(stop_event)


def _load_plan(environment: Mapping[str, str]) -> RunPlan:
    raw_path = environment.get("LINGTU_RUN_PLAN", "")
    if not raw_path or raw_path != raw_path.strip():
        raise RuntimeError("LINGTU_RUN_PLAN is required")
    plan = RunPlan.load(raw_path)
    if plan.env != "real":
        raise RuntimeError("real Host requires env=real")
    if plan.process_control != "systemd":
        raise RuntimeError("real Host requires process_control=systemd")
    if not plan.has_process("host"):
        raise RuntimeError("real Host requires a host process role")
    return plan


def _module(system: Any, name: str) -> Any | None:
    try:
        return system.get_module(name)
    except (KeyError, AttributeError):
        return None


def _install_signal_handlers(stop_event: threading.Event) -> None:
    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
