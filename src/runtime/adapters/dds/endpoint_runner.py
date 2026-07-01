"""Runnable Thunder typed DDS endpoint process."""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import os
import time
from collections.abc import Callable
from typing import Any

from runtime.adapters.dds.contracts import THUNDER_FIELD_DDS_CONTRACT_NAME
from runtime.adapters.dds.endpoint_service import DDSEndpointEvent, DDSEndpointService

logger = logging.getLogger(__name__)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the LingTu Thunder typed DDS endpoint service.",
    )
    parser.add_argument(
        "--contract",
        default=THUNDER_FIELD_DDS_CONTRACT_NAME,
        help="Typed DDS endpoint contract name.",
    )
    parser.add_argument(
        "--source",
        default="",
        help=(
            "Optional source plugin. Use 'smoke', 'jsonl', 'thunder_brainstem', "
            "'thunder_field', or module:factory. Multiple sources may be "
            "comma-separated."
        ),
    )
    parser.add_argument(
        "--transport",
        default="dds",
        choices=("dds", "local"),
        help="Endpoint transport strategy. 'local' is for no-DDS smoke checks only.",
    )
    parser.add_argument("--heartbeat-sec", type=float, default=5.0)
    parser.add_argument("--max-heartbeats", type=int, default=None)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
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
        logger.exception("Thunder DDS endpoint service failed")
        result = {"ok": False, "error": str(exc), "error_type": type(exc).__name__}
    if args.json:
        print(json.dumps(result, ensure_ascii=True, sort_keys=True))
    return 0 if result.get("ok") else 1


def run_endpoint_service(
    args: argparse.Namespace,
    *,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    events: list[DDSEndpointEvent] = []
    sources: list[Any] = []

    def on_lingtu_message(event: DDSEndpointEvent) -> None:
        events.append(event)
        for source in sources:
            on_message = getattr(source, "on_lingtu_message", None)
            if callable(on_message):
                on_message(event)

    service = DDSEndpointService(
        endpoint_contract_name=args.contract,
        on_lingtu_message=on_lingtu_message,
        transport_factory=_transport_factory(args.transport),
        transport_strategy=args.transport,
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
            return _result("once", args.transport, source_attached, sources, service, events)

        heartbeat_count = 0
        while args.max_heartbeats is None or heartbeat_count < args.max_heartbeats:
            sleep(max(0.0, float(args.heartbeat_sec)))
            heartbeat_count += 1
            health = service.health()
            logger.info(
                "DDS endpoint heartbeat %s: sent=%s received=%s",
                heartbeat_count,
                sum(health["publish_counts"].values()),
                sum(health["receive_counts"].values()),
            )
        result = _result("run", args.transport, source_attached, sources, service, events)
        result["heartbeats"] = heartbeat_count
        return result
    finally:
        _stop_sources(sources)
        service.stop()


def _result(
    mode: str,
    transport: str,
    source_attached: bool,
    sources: list[Any],
    service: DDSEndpointService,
    events: list[DDSEndpointEvent],
) -> dict[str, Any]:
    return {
        "ok": True,
        "mode": mode,
        "transport_strategy": transport,
        "source_attached": source_attached,
        "source_health": _source_health_compat(sources),
        "source_healths": [_source_health(source) for source in sources],
        "events": len(events),
        "health": service.health(),
    }


def _load_sources(spec: str) -> list[Any]:
    return [_load_source(item) for item in _expand_source_specs(spec)]


def _expand_source_specs(spec: str | list[str]) -> list[str]:
    items = spec if isinstance(spec, list) else str(spec or "").split(",")
    expanded: list[str] = []
    for item in (part.strip() for part in items):
        if not item:
            continue
        if item in {"field", "thunder_field", "builtin:thunder_field"}:
            expanded.append("thunder_brainstem")
            if _jsonl_source_configured():
                expanded.append("jsonl")
        else:
            expanded.append(item)
    return expanded


def _jsonl_source_configured() -> bool:
    return any(
        os.getenv(name) not in (None, "")
        for name in (
            "LINGTU_ENDPOINT_JSONL_PATH",
            "LINGTU_THUNDER_JSONL_PATH",
            "LINGTU_ENDPOINT_JSONL_COMMAND",
        )
    )


def _load_source(spec: str) -> Any:
    builtins = {
        "smoke": "runtime.adapters.lcm.sources.smoke:create",
        "builtin:smoke": "runtime.adapters.lcm.sources.smoke:create",
        "jsonl": "runtime.adapters.lcm.sources.jsonl:create",
        "builtin:jsonl": "runtime.adapters.lcm.sources.jsonl:create",
        "thunder_brainstem": "runtime.adapters.lcm.sources.brainstem:create",
        "brainstem": "runtime.adapters.lcm.sources.brainstem:create",
        "builtin:thunder_brainstem": "runtime.adapters.lcm.sources.brainstem:create",
    }
    target = builtins.get(spec, spec)
    module_name, sep, factory_name = target.partition(":")
    if not sep:
        raise ValueError("--source must be smoke, jsonl, thunder_field, thunder_brainstem, or module:factory")
    factory = getattr(importlib.import_module(module_name), factory_name)
    return factory()


def _start_sources(sources: list[Any], service: DDSEndpointService) -> bool:
    for source in sources:
        start = getattr(source, "start", None)
        if callable(start):
            start(service)
        elif callable(source):
            source(service)
        else:
            raise TypeError("endpoint source must expose start(service) or be callable")
    return bool(sources)


def _stop_sources(sources: list[Any]) -> None:
    for source in reversed(sources):
        stop = getattr(source, "stop", None)
        if callable(stop):
            stop()


def _source_health(source: Any) -> dict[str, Any]:
    health = getattr(source, "health", None)
    if callable(health):
        return dict(health())
    return {"name": str(getattr(source, "name", type(source).__name__))}


def _source_health_compat(sources: list[Any]) -> dict[str, Any]:
    if not sources:
        return {}
    if len(sources) == 1:
        return _source_health(sources[0])
    return {"sources": [item.get("name", "") for item in map(_source_health, sources)], "count": len(sources)}


def _transport_factory(strategy: str) -> Callable[[], Any] | None:
    if strategy == "dds":
        return None

    def create() -> Any:
        from runtime.transport.factory import create_transport

        return create_transport(strategy)

    return create


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
