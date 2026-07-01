#!/usr/bin/env python3
"""Validate a no-ROS Thunder LCM JSONL endpoint feed before deployment."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from collections.abc import Mapping
from pathlib import Path
from typing import Any

ROOT_DIR = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, endpoint_contract  # noqa: E402
from compat.lcm.sources.jsonl import message_from_record  # noqa: E402
from core.runtime_interface import (  # noqa: E402
    REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS,
    TOPICS,
    body_frame_id,
    expand_frame_id_aliases,
    normalize_frame_id,
    runtime_topic_allowed_frame_ids,
)


def validate_feed(
    path: str | Path,
    *,
    contract_name: str = THUNDER_FIELD_LCM_CONTRACT_NAME,
    require_field_inputs: bool = False,
    required_topics: tuple[str, ...] = (),
) -> dict[str, Any]:
    """Validate one JSONL endpoint feed and return a JSON-ready report."""

    feed_path = Path(path).expanduser()
    contract = endpoint_contract(contract_name)
    allowed_frames = runtime_topic_allowed_frame_ids(contract.runtime_contract)
    blockers: list[str] = []
    warnings: list[str] = []
    topic_counts: Counter[str] = Counter()
    frame_counts: dict[str, Counter[str]] = defaultdict(Counter)
    checked_records = 0
    skipped_records = 0

    if not feed_path.exists():
        blockers.append(f"feed file does not exist: {feed_path}")
        return _report(
            path=feed_path,
            contract_name=contract.name,
            runtime_contract=contract.runtime_contract,
            checked_records=0,
            skipped_records=0,
            topic_counts=topic_counts,
            frame_counts=frame_counts,
            blockers=blockers,
            warnings=warnings,
        )

    with feed_path.open("r", encoding="utf-8-sig") as handle:
        for line_no, line in enumerate(handle, 1):
            raw = line.lstrip("\ufeff").strip()
            if not raw or raw.startswith("#"):
                skipped_records += 1
                continue
            try:
                record = json.loads(raw)
                if not isinstance(record, Mapping):
                    raise ValueError("record must be a JSON object")
                if _is_delay_record(record):
                    skipped_records += 1
                    continue
                topic = str(record.get("topic") or "")
                if not topic:
                    raise ValueError("record missing topic")
                binding = contract.binding_for_topic(topic)
                if binding.direction != "endpoint_to_lingtu":
                    raise ValueError(f"{topic} is not an endpoint-to-LingTu topic")
                message = message_from_record(binding, record)
                checked_records += 1
                topic_counts[topic] += 1
                _validate_message_frames(
                    topic=topic,
                    message=message,
                    line_no=line_no,
                    allowed_frames=allowed_frames,
                    frame_counts=frame_counts,
                    blockers=blockers,
                )
            except Exception as exc:
                blockers.append(f"line {line_no}: {exc}")

    if checked_records == 0:
        blockers.append("feed contains no endpoint records")

    required = tuple(required_topics)
    if require_field_inputs:
        required = (*REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS, *required)
    for topic in dict.fromkeys(required):
        if topic_counts.get(topic, 0) <= 0:
            blockers.append(f"missing required endpoint input topic: {topic}")

    return _report(
        path=feed_path,
        contract_name=contract.name,
        runtime_contract=contract.runtime_contract,
        checked_records=checked_records,
        skipped_records=skipped_records,
        topic_counts=topic_counts,
        frame_counts=frame_counts,
        blockers=blockers,
        warnings=warnings,
    )


def _validate_message_frames(
    *,
    topic: str,
    message: Any,
    line_no: int,
    allowed_frames: Mapping[str, tuple[str, ...]],
    frame_counts: dict[str, Counter[str]],
    blockers: list[str],
) -> None:
    """Validate frame_id evidence on one decoded endpoint message."""

    frames = _message_frame_ids(message)
    topic_allowed = expand_frame_id_aliases(allowed_frames.get(topic, ()))
    if topic_allowed and not frames:
        blockers.append(f"line {line_no}: {topic} message has no frame_id")
        return

    for frame_id in frames:
        frame_counts[topic][frame_id] += 1
        if topic_allowed and frame_id not in topic_allowed:
            allowed = ", ".join(topic_allowed)
            blockers.append(
                f"line {line_no}: {topic} frame_id {frame_id!r} "
                f"not allowed; expected one of: {allowed}"
            )

    child_frame_id = normalize_frame_id(getattr(message, "child_frame_id", None))
    if topic == TOPICS.odometry and child_frame_id is not None:
        allowed_child_frames = expand_frame_id_aliases((body_frame_id(),))
        frame_counts[f"{topic}:child_frame_id"][child_frame_id] += 1
        if child_frame_id not in allowed_child_frames:
            allowed = ", ".join(allowed_child_frames)
            blockers.append(
                f"line {line_no}: {topic} child_frame_id {child_frame_id!r} "
                f"not allowed; expected one of: {allowed}"
            )


def _message_frame_ids(message: Any) -> tuple[str, ...]:
    """Return normalized frame_id evidence from a decoded message."""

    frame_id = normalize_frame_id(getattr(message, "frame_id", None))
    if frame_id is not None:
        return (frame_id,)
    return ()


def _is_delay_record(record: Mapping[str, Any]) -> bool:
    """Return true when a JSONL row only controls replay pacing."""

    return "topic" not in record and (
        record.get("sleep_sec") is not None or record.get("delay_sec") is not None
    )


def _report(
    *,
    path: Path,
    contract_name: str,
    runtime_contract: str,
    checked_records: int,
    skipped_records: int,
    topic_counts: Counter[str],
    frame_counts: dict[str, Counter[str]],
    blockers: list[str],
    warnings: list[str],
) -> dict[str, Any]:
    """Build the normalized validator report."""

    return {
        "ok": not blockers,
        "path": str(path),
        "contract": contract_name,
        "runtime_contract": runtime_contract,
        "checked_records": checked_records,
        "skipped_records": skipped_records,
        "topics": dict(sorted(topic_counts.items())),
        "frames": {
            topic: dict(sorted(counts.items()))
            for topic, counts in sorted(frame_counts.items())
        },
        "blockers": blockers,
        "warnings": warnings,
    }


def _parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="JSONL feed file to validate")
    parser.add_argument(
        "--contract",
        default=THUNDER_FIELD_LCM_CONTRACT_NAME,
        help="LCM endpoint contract name",
    )
    parser.add_argument(
        "--require-field-inputs",
        action="store_true",
        help="Require the minimal Thunder field sensor inputs: lidar scan and IMU",
    )
    parser.add_argument(
        "--require-topic",
        action="append",
        default=[],
        help="Additional endpoint-to-LingTu topic that must appear at least once",
    )
    parser.add_argument("--json", action="store_true", help="Print a JSON report")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the JSONL feed validator."""

    args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    report = validate_feed(
        args.path,
        contract_name=args.contract,
        require_field_inputs=bool(args.require_field_inputs),
        required_topics=tuple(args.require_topic or ()),
    )
    if args.json:
        print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    else:
        _print_human_report(report)
    return 0 if report["ok"] else 1


def _print_human_report(report: Mapping[str, Any]) -> None:
    """Print a compact human-readable validation report."""

    status = "PASSED" if report["ok"] else "FAILED"
    print(f"{status}: {report['path']}")
    print(f"Contract: {report['contract']} ({report['runtime_contract']})")
    print(f"Records: {report['checked_records']} checked, {report['skipped_records']} skipped")
    topics = report.get("topics") or {}
    if topics:
        print("Topics:")
        for topic, count in sorted(topics.items()):
            print(f"  {topic}: {count}")
    frames = report.get("frames") or {}
    if frames:
        print("Frames:")
        for topic, counts in sorted(frames.items()):
            values = ", ".join(f"{frame}={count}" for frame, count in sorted(counts.items()))
            print(f"  {topic}: {values}")
    for warning in report.get("warnings") or ():
        print(f"WARNING: {warning}")
    for blocker in report.get("blockers") or ():
        print(f"ERROR: {blocker}")


if __name__ == "__main__":
    sys.exit(main())
