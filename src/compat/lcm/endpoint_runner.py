"""Runnable Thunder LCM endpoint process.

This module is the product deployment entrypoint for the field endpoint side.
It starts :class:`LCMEndpointService`, keeps the LCM details inside
``core.transport``, and optionally attaches a hardware/source plugin that
feeds normalized Thunder sensor and localization snapshots into LingTu.
"""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import time
from collections.abc import Callable
from typing import Any

from .contracts import THUNDER_FIELD_LCM_CONTRACT_NAME
from .endpoint_service import LCMEndpointEvent, LCMEndpointService

logger = logging.getLogger(__name__)


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the endpoint process."""

    parser = argparse.ArgumentParser(
        description="Run the LingTu Thunder LCM endpoint service.",
    )
    parser.add_argument(
        "--contract",
        default=THUNDER_FIELD_LCM_CONTRACT_NAME,
        help="LCM endpoint contract name.",
    )
    parser.add_argument(
        "--source",
        default="",
        help=(
            "Optional source plugin. Use 'smoke' for the built-in no-ROS smoke "
            "source, 'thunder_brainstem' for the built-in Brainstem command "
            "sink, 'jsonl' for a no-ROS JSONL sensor/localization feed, or "
            "'thunder_field' for the product field source group. Multiple "
            "sources may be comma-separated; deployment-specific sources use "
            "module:factory."
        ),
    )
    parser.add_argument(
        "--transport",
        default="lcm",
        choices=("lcm", "local"),
        help="Endpoint transport strategy. 'local' is for no-LCM smoke checks only.",
    )
    parser.add_argument(
        "--heartbeat-sec",
        type=float,
        default=5.0,
        help="Health heartbeat interval while the service is running.",
    )
    parser.add_argument(
        "--max-heartbeats",
        type=int,
        default=None,
        help="Stop after N heartbeats. Mainly useful for supervised smoke tests.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Start once, print health, and exit.",
    )
    parser.add_argument(
        "--describe",
        action="store_true",
        help="Describe the contract without opening the LCM transport.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON for describe/once/final results.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="Logging verbosity.",
    )
    return parser


def run_endpoint_service(
    args: argparse.Namespace,
    *,
    service_factory: Callable[..., LCMEndpointService] = LCMEndpointService,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Run or describe the endpoint service and return a result dictionary."""

    events: list[LCMEndpointEvent] = []
    sources: list[Any] = []

    def on_lingtu_message(event: LCMEndpointEvent) -> None:
        events.append(event)
        _notify_sources(sources, event)
        logger.info(
            "LCM endpoint received %s on %s",
            event.topic,
            event.channel,
        )

    service = service_factory(
        endpoint_contract_name=args.contract,
        on_lingtu_message=on_lingtu_message,
        transport_factory=_transport_factory(args.transport),
    )

    if args.describe:
        return {
            "ok": True,
            "mode": "describe",
            "transport_strategy": args.transport,
            "source_attached": False,
            "source_healths": [],
            "health": service.health(),
        }

    sources = _load_sources(args.source)
    try:
        service.start()
        source_attached = _start_sources(sources, service)
        if args.once:
            return {
                "ok": True,
                "mode": "once",
                "transport_strategy": args.transport,
                "source_attached": source_attached,
                "source_health": _source_health_compat(sources),
                "source_healths": _source_healths(sources),
                "health": service.health(),
            }

        heartbeat_count = 0
        while args.max_heartbeats is None or heartbeat_count < args.max_heartbeats:
            sleep(max(0.0, float(args.heartbeat_sec)))
            heartbeat_count += 1
            health = service.health()
            logger.info(
                "LCM endpoint heartbeat %s: sent=%s received=%s",
                heartbeat_count,
                sum(health["publish_counts"].values()),
                sum(health["receive_counts"].values()),
            )
        return {
            "ok": True,
            "mode": "run",
            "transport_strategy": args.transport,
            "source_attached": source_attached,
            "source_health": _source_health_compat(sources),
            "source_healths": _source_healths(sources),
            "heartbeats": heartbeat_count,
            "events": len(events),
            "health": service.health(),
        }
    finally:
        _stop_sources(sources)
        service.stop()


def main(argv: list[str] | None = None) -> int:
    """Run the endpoint process from the command line."""

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        result = run_endpoint_service(args)
    except KeyboardInterrupt:
        result = {"ok": True, "mode": "interrupted"}
    except Exception as exc:
        logger.exception("Thunder LCM endpoint service failed")
        result = {"ok": False, "error": str(exc), "error_type": type(exc).__name__}

    if args.json:
        print(json.dumps(result, ensure_ascii=True, sort_keys=True))
    elif args.describe or args.once or not result.get("ok"):
        _print_human_result(result)
    return 0 if result.get("ok") else 1


def _load_sources(spec: str) -> list[Any]:
    specs = [item.strip() for item in str(spec or "").split(",") if item.strip()]
    specs = _expand_source_specs(specs)
    return [_load_source(item) for item in specs]


def _expand_source_specs(specs: list[str]) -> list[str]:
    expanded: list[str] = []
    for spec in specs:
        if spec in {"field", "thunder_field", "builtin:thunder_field"}:
            expanded.extend(_thunder_field_source_specs())
        else:
            expanded.append(spec)
    return expanded


def _thunder_field_source_specs() -> list[str]:
    specs = ["thunder_brainstem"]
    if _jsonl_source_configured():
        specs.append("jsonl")
    return specs


def _jsonl_source_configured() -> bool:
    import os

    return any(
        os.getenv(name) not in (None, "")
        for name in (
            "LINGTU_ENDPOINT_JSONL_PATH",
            "LINGTU_THUNDER_JSONL_PATH",
            "LINGTU_ENDPOINT_JSONL_COMMAND",
        )
    )


def _load_source(spec: str) -> Any:
    if spec in {"smoke", "builtin:smoke"}:
        from .sources.smoke import create

        return create()
    if spec in {"jsonl", "jsonl_replay", "replay_jsonl", "builtin:jsonl"}:
        from .sources.jsonl import create

        return create()
    if spec in {
        "thunder",
        "brainstem",
        "thunder_brainstem",
        "builtin:thunder_brainstem",
    }:
        from .sources.thunder_brainstem import create

        return create()
    module_name, sep, factory_name = spec.partition(":")
    if not sep or not module_name or not factory_name:
        raise ValueError(
            "--source must be 'smoke', 'thunder_field', 'thunder_brainstem', "
            "'jsonl', or formatted as module:factory"
        )
    module = importlib.import_module(module_name)
    factory = getattr(module, factory_name)
    if not callable(factory):
        raise TypeError(f"{spec} did not resolve to a callable source factory")
    return factory()


def _start_sources(sources: list[Any], service: LCMEndpointService) -> bool:
    if not sources:
        logger.info("No Thunder endpoint source plugin configured")
        return False
    started: list[Any] = []
    try:
        for source in sources:
            _start_source(source, service)
            started.append(source)
    except Exception:
        _stop_sources(started)
        raise
    return True


def _start_source(source: Any, service: LCMEndpointService) -> None:
    start = getattr(source, "start", None)
    if callable(start):
        start(service)
        return
    if callable(source):
        source(service)
        return
    raise TypeError("endpoint source must expose start(service) or be callable")


def _notify_sources(sources: list[Any], event: LCMEndpointEvent) -> None:
    for source in sources:
        _notify_source(source, event)


def _notify_source(source: Any, event: LCMEndpointEvent) -> None:
    on_message = getattr(source, "on_lingtu_message", None)
    if callable(on_message):
        on_message(event)


def _source_health_compat(sources: list[Any]) -> dict[str, Any]:
    if not sources:
        return {}
    if len(sources) == 1:
        return _source_health(sources[0])
    return {
        "sources": [
            health.get("name", f"source_{idx}")
            for idx, health in enumerate(_source_healths(sources), start=1)
        ],
        "count": len(sources),
    }


def _source_healths(sources: list[Any]) -> list[dict[str, Any]]:
    return [_source_health(source) for source in sources]


def _source_health(source: Any) -> dict[str, Any]:
    health = getattr(source, "health", None)
    if callable(health):
        return dict(health())
    name = getattr(source, "name", type(source).__name__)
    return {"name": str(name)}


def _stop_sources(sources: list[Any]) -> None:
    for source in reversed(sources):
        _stop_source(source)


def _stop_source(source: Any) -> None:
    stop = getattr(source, "stop", None)
    if callable(stop):
        stop()


def _transport_factory(strategy: str) -> Callable[[], Any] | None:
    if strategy == "lcm":
        return None

    def create() -> Any:
        from core.transport.factory import create_transport

        return create_transport(strategy)

    return create


def _print_human_result(result: dict[str, Any]) -> None:
    if not result.get("ok"):
        print(f"Thunder LCM endpoint failed: {result.get('error', 'unknown error')}")
        return
    health = result.get("health") or {}
    print(f"Thunder LCM endpoint mode: {result.get('mode')}")
    if health:
        print(f"Contract: {health.get('endpoint_contract')}")
        print(f"Transport: {health.get('transport')}")
        print(f"Started: {health.get('started')}")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
