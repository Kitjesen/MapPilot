"""Direct-child Host entry point for Product-owned simulation runs."""

# ruff: noqa: E402 - direct script execution must establish the source root first.

from __future__ import annotations

import os
import signal
import sys
import threading
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


def _prepare_direct_script_import_path() -> None:
    if __package__ not in {None, ""}:
        return
    source_root = Path(__file__).resolve().parents[2]
    script_root = Path(__file__).resolve().parent
    retained = [
        entry
        for entry in sys.path
        if Path(entry or os.curdir).resolve() != script_root
    ]
    sys.path[:] = [str(source_root), *retained]


_prepare_direct_script_import_path()

from lingtu.run_plan import RunPlan
from lingtu.sim.readiness import publish_host_readiness
from lingtu.switch_contracts import is_product_session_id


def run(stop_event: threading.Event | None = None) -> int:
    """Build and run the Host described by the current direct-child RunPlan."""

    event = stop_event if stop_event is not None else threading.Event()
    plan, session_root, product_session_id = _load_plan(os.environ)
    system: Any | None = None
    published: Path | None = None
    try:
        system = plan.build()
        system.start()
        published = publish_host_readiness(
            session_root,
            product_session_id=product_session_id,
            product=plan.product,
            process=plan.process("host").name,
        )
        event.wait()
        return 0
    finally:
        try:
            if system is not None:
                system.stop()
        finally:
            if published is not None:
                try:
                    published.unlink(missing_ok=True)
                except OSError:
                    pass


def main(argv: Sequence[str] | None = None) -> int:
    """Run the simulation Host direct child."""

    args = tuple(sys.argv[1:] if argv is None else argv)
    if args:
        raise SystemExit("lingtu.sim.host does not accept CLI arguments")
    stop_event = threading.Event()
    _install_signal_handlers(stop_event)
    return run(stop_event)


def _install_signal_handlers(stop_event: threading.Event) -> None:
    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)


def _required_environment(environment: Mapping[str, str], key: str) -> str:
    value = environment.get(key, "")
    if not isinstance(value, str) or not value or value != value.strip():
        raise RuntimeError(f"{key} is required")
    return value


def _load_plan(environment: Mapping[str, str]) -> tuple[RunPlan, Path, str]:
    run_plan_path = Path(_required_environment(environment, "LINGTU_RUN_PLAN"))
    if not run_plan_path.is_absolute():
        raise RuntimeError("LINGTU_RUN_PLAN must be an absolute path")
    product_session_id = _required_environment(environment, "LINGTU_PRODUCT_SESSION_ID")
    if not is_product_session_id(product_session_id):
        raise RuntimeError("LINGTU_PRODUCT_SESSION_ID is invalid")
    plan = RunPlan.load(run_plan_path)
    if plan.env != "sim":
        raise RuntimeError("formal Host direct child requires env=sim")
    if plan.process_control != "subprocess":
        raise RuntimeError("formal Host direct child requires process_control=subprocess")
    if not plan.has_process("host"):
        raise RuntimeError("formal Host direct child requires a host process role")
    return plan, run_plan_path.parent, product_session_id


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
