"""Field readiness collector: one-shot proof that native DDS topics are alive.

This is a diagnostic helper (outside the runtime kernel). It samples every
native DDS topic once and produces a machine-readable report proving which
topics are publishing on the wire. It reuses the DDS sampling logic in
``scripts/diagnostics/dds_probe.py`` instead of reimplementing subscriptions.
The probe is a native CycloneDDS helper, not the optional cyclonedds-python
package.

Usage on the robot (S100P)::

    PYTHONPATH=src:. python -m diagnostics.field.field_readiness_collector \
        --seconds 5 --domain 0 --json artifacts/field/readiness.json

The report is tagged with :data:`FIELD_READINESS_SCHEMA_VERSION` so downstream
consumers can validate its shape.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

# Message registry: used to resolve a topic name to a typed DDS contract and to
# translate rt/ wire names back to their internal runtime topic keys.
from message.dds import (
    TOPIC_SPECS,
    dds_topic_name,
)

FIELD_READINESS_SCHEMA_VERSION = "lingtu.field_readiness_report.v1"

# Native DDS wire topics that must be alive for a field-ready system. These are
# the typed ``rt/`` wire names as observed on the CycloneDDS bus.
DEFAULT_TOPICS: tuple[str, ...] = (
    "rt/camera/color",
    "rt/camera/depth",
    "rt/camera/info",
    "rt/lidar/raw_frame",
    "rt/imu/raw",
    "rt/slam/odometry",
    "rt/slam/map_cloud",
    "rt/slam/localization_health",
    "rt/nav/traversability",
    "rt/nav/terrain_map",
)


def _load_probe():
    """Return the DDS ``probe`` function (imported lazily).

    Keeping it lazy lets this module import cleanly on machines without the
    native CycloneDDS toolchain and lets tests substitute a fake probe without
    touching the DDS stack.
    """

    from scripts.diagnostics.dds_probe import probe

    return probe


def _build_wire_to_internal() -> dict[str, str]:
    """Map typed ``rt/`` wire names to their internal runtime topic keys."""

    mapping: dict[str, str] = {}
    for internal in TOPIC_SPECS:
        try:
            wire = dds_topic_name(internal, typed=True)
        except Exception:
            continue
        mapping.setdefault(wire, internal)
    return mapping


_WIRE_TO_INTERNAL = _build_wire_to_internal()


def _probe_topic_name(topic: str) -> str | None:
    """Resolve *topic* to a name that ``dds_probe.probe`` can subscribe to.

    Returns ``None`` when no typed DDS contract exists for the topic.
    """

    if topic in TOPIC_SPECS:
        return dds_topic_name(topic, typed=True)
    internal = _WIRE_TO_INTERNAL.get(topic)
    if internal in TOPIC_SPECS:
        return topic
    return None


class FieldReadinessCollector:
    """Collect one-shot liveness evidence for native DDS topics."""

    def __init__(
        self,
        topics: Sequence[str] = DEFAULT_TOPICS,
        domain_id: int = 0,
    ) -> None:
        self.topics: tuple[str, ...] = tuple(topics)
        self.domain_id = int(domain_id)
        self._error: str | None = None

    # ── collection ───────────────────────────────────────────────────────

    def collect(self, seconds: float) -> dict[str, dict[str, Any]]:
        """Sample all topics once and return per-topic result dicts.

        Reuses :func:`scripts.diagnostics.dds_probe.probe`. Degrades gracefully
        when the native probe is unavailable: every topic is reported dead with
        a clear error instead of raising.
        """

        self._error = None
        try:
            probe = _load_probe()
        except Exception as exc:
            self._error = f"Native DDS probe unavailable: {exc}"
            return {t: self._dead_result(error=self._error) for t in self.topics}

        resolved = {topic: _probe_topic_name(topic) for topic in self.topics}
        probe_names = tuple(sorted({name for name in resolved.values() if name}))

        raw: dict[str, Any] = {}
        if probe_names:
            try:
                raw = probe(
                    probe_names,
                    seconds=seconds,
                    domain_id=self.domain_id,
                )
            except Exception as exc:
                self._error = f"DDS probe failed: {exc}"
                return {t: self._dead_result(error=self._error) for t in self.topics}

        results: dict[str, dict[str, Any]] = {}
        for topic, name in resolved.items():
            if name is None:
                results[topic] = self._dead_result(error="no typed DDS contract")
                continue
            results[topic] = self._stat_to_result(raw.get(name))
        return results

    # ── header validation ────────────────────────────────────────────────

    def _validate_headers(self, stats: Any) -> dict[str, Any]:
        """Check frame_id / timestamp presence on a probe stat object."""

        frame_id = str(getattr(stats, "frame_id", "") or "")
        last_ts = float(getattr(stats, "last_ts", 0.0) or 0.0)
        samples = int(getattr(stats, "samples", 0) or 0)
        return {
            "frame_id": frame_id,
            "has_frame_id": bool(frame_id),
            "has_timestamp": last_ts > 0.0 or samples > 0,
        }

    def _stat_to_result(self, stat: Any) -> dict[str, Any]:
        if stat is None:
            return self._dead_result()
        samples = int(getattr(stat, "samples", 0) or 0)
        rate = 0.0
        hz = getattr(stat, "hz", None)
        if callable(hz):
            try:
                rate = float(hz())
            except Exception:
                rate = 0.0
        headers = self._validate_headers(stat)
        result: dict[str, Any] = {
            "samples": samples,
            "rate_hz": round(rate, 2),
            "frame_id": headers["frame_id"],
            "has_frame_id": headers["has_frame_id"],
            "has_timestamp": headers["has_timestamp"],
            "ok": samples > 0,
        }
        points = getattr(stat, "points", None)
        if points is not None:
            try:
                result["points"] = int(points)
            except (TypeError, ValueError):
                pass
        return result

    @staticmethod
    def _dead_result(error: str | None = None) -> dict[str, Any]:
        result: dict[str, Any] = {
            "samples": 0,
            "rate_hz": 0.0,
            "frame_id": "",
            "has_frame_id": False,
            "has_timestamp": False,
            "ok": False,
        }
        if error:
            result["error"] = error
        return result

    # ── report assembly ──────────────────────────────────────────────────

    def build_report(
        self,
        results: dict[str, dict[str, Any]],
        duration_s: float,
    ) -> dict[str, Any]:
        """Assemble the schema-tagged readiness report from collected results."""

        total = len(results)
        alive = sum(1 for r in results.values() if int(r.get("samples", 0) or 0) > 0)
        dead = total - alive
        missing = sorted(topic for topic, r in results.items() if int(r.get("samples", 0) or 0) <= 0)
        topic_errors = sorted({str(r["error"]) for r in results.values() if r.get("error")})
        ok = total > 0 and dead == 0 and self._error is None

        report: dict[str, Any] = {
            "schema": FIELD_READINESS_SCHEMA_VERSION,
            "schema_version": FIELD_READINESS_SCHEMA_VERSION,
            "ts": time.time(),
            "domain_id": self.domain_id,
            "duration_s": float(duration_s),
            "ok": ok,
            "topics": dict(results),
            "missing": missing,
            "summary": {"total": total, "alive": alive, "dead": dead},
        }
        if self._error:
            report["error"] = self._error
        elif topic_errors:
            report["errors"] = topic_errors
        return report


def collect_field_readiness(
    topics: Sequence[str] | None = None,
    *,
    seconds: float = 5.0,
    domain_id: int = 0,
) -> dict[str, Any]:
    """Collect readiness evidence and return a schema-tagged report."""

    collector = FieldReadinessCollector(
        topics=tuple(topics) if topics else DEFAULT_TOPICS,
        domain_id=domain_id,
    )
    results = collector.collect(seconds)
    return collector.build_report(results, seconds)


def _format_report(report: dict[str, Any]) -> str:
    lines: list[str] = []
    summary = report.get("summary", {})
    header = (
        f"field readiness: {'OK' if report.get('ok') else 'FAIL'} "
        f"({summary.get('alive', 0)}/{summary.get('total', 0)} alive, "
        f"domain={report.get('domain_id')}, "
        f"duration={report.get('duration_s')}s)"
    )
    lines.append(header)
    if report.get("error"):
        lines.append(f"  error: {report['error']}")
    lines.append(f"  {'topic':32} {'samples':>7} {'rate_hz':>8} {'frame':16} {'ok':>4}")
    for topic, item in report.get("topics", {}).items():
        lines.append(
            f"  {topic:32} {item.get('samples', 0):7d} "
            f"{item.get('rate_hz', 0.0):8.2f} "
            f"{str(item.get('frame_id', ''))[:16]:16} "
            f"{'yes' if item.get('ok') else 'no':>4}"
        )
    if report.get("missing"):
        lines.append(f"  missing: {', '.join(report['missing'])}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="Sampling window in seconds (default: 5.0).",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="CycloneDDS domain id (default: 0).",
    )
    parser.add_argument(
        "--topics",
        nargs="*",
        default=None,
        help="Optional subset of topics to probe (default: all native topics).",
    )
    parser.add_argument(
        "--json",
        type=str,
        default=None,
        help="Optional output path for the JSON report.",
    )
    args = parser.parse_args(argv)

    report = collect_field_readiness(
        topics=args.topics,
        seconds=args.seconds,
        domain_id=args.domain,
    )

    if args.json:
        out_path = Path(args.json)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(_format_report(report))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
