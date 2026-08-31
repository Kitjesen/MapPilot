"""Sample native DDS topics once and report which ones are active."""

from __future__ import annotations

import argparse
import json
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from message.topics import TOPIC_SPECS, dds_topic_name

SCHEMA_VERSION = "lingtu.field_readiness_report.v1"
_WIRE_TOPICS = frozenset(dds_topic_name(topic) for topic in TOPIC_SPECS)


def _load_probe():
    from scripts.diagnostics.dds_probe import probe

    return probe


def _dead_topic(error: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "samples": 0,
        "rate_hz": 0.0,
        "first_ts": 0.0,
        "last_ts": 0.0,
        "frame_id": "",
    }
    if error:
        result["error"] = error
    return result


def _topic_result(stat: Any) -> dict[str, Any]:
    if stat is None:
        return _dead_topic()
    try:
        rate_hz = float(stat.hz())
    except (AttributeError, TypeError, ValueError):
        rate_hz = 0.0
    result: dict[str, Any] = {
        "samples": int(getattr(stat, "samples", 0) or 0),
        "rate_hz": round(rate_hz, 2),
        "first_ts": float(getattr(stat, "first_ts", 0.0) or 0.0),
        "last_ts": float(getattr(stat, "last_ts", 0.0) or 0.0),
        "frame_id": str(getattr(stat, "frame_id", "") or ""),
    }
    points = getattr(stat, "points", None)
    if points is not None:
        try:
            result["points"] = int(points)
        except (TypeError, ValueError):
            pass
    return result


def collect_readiness(
    topics: Sequence[str] | None = None,
    *,
    seconds: float = 5.0,
    domain_id: int = 0,
) -> dict[str, Any]:
    """Run one native probe and return its per-topic observations."""

    selected = tuple(topics or ())
    if not selected:
        raise ValueError("topics must be provided for the Product being checked")
    valid = tuple(topic for topic in selected if topic in _WIRE_TOPICS)
    invalid = set(selected) - set(valid)
    results = {topic: _dead_topic("no typed DDS contract") for topic in invalid}

    if valid:
        try:
            observed = _load_probe()(valid, seconds=seconds, domain_id=domain_id)
        except Exception as exc:
            error = f"DDS probe failed: {exc}"
            results.update({topic: _dead_topic(error) for topic in valid})
        else:
            results.update({topic: _topic_result(observed.get(topic)) for topic in valid})

    ordered_results = {topic: results[topic] for topic in selected}
    missing = [topic for topic, item in ordered_results.items() if item["samples"] <= 0]
    return {
        "schema_version": SCHEMA_VERSION,
        "collected_at": time.time(),
        "domain_id": int(domain_id),
        "duration_s": float(seconds),
        "ok": bool(selected) and not missing,
        "missing": missing,
        "topics": ordered_results,
    }


def _format_report(report: dict[str, Any]) -> str:
    topics = report["topics"]
    alive = len(topics) - len(report["missing"])
    lines = [
        f"DDS readiness: {'OK' if report['ok'] else 'FAIL'} "
        f"({alive}/{len(topics)} active, domain={report['domain_id']}, duration={report['duration_s']}s)",
        f"  {'topic':32} {'samples':>7} {'rate_hz':>8} {'frame':16}",
    ]
    for topic, item in topics.items():
        lines.append(f"  {topic:32} {item['samples']:7d} {item['rate_hz']:8.2f} {item['frame_id'][:16]:16}")
    if report["missing"]:
        lines.append(f"  missing: {', '.join(report['missing'])}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """Run the one-shot DDS readiness CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0, help="sampling window")
    parser.add_argument("--domain", type=int, default=0, help="CycloneDDS domain id")
    parser.add_argument("--topics", nargs="+", required=True, help="canonical rt/... topics required by this Product")
    parser.add_argument("--json", type=Path, help="optional JSON report path")
    args = parser.parse_args(argv)

    report = collect_readiness(args.topics, seconds=args.seconds, domain_id=args.domain)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(_format_report(report))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
