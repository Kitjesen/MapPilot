#!/usr/bin/env python3
"""Export user-visible messages from a local Codex rollout JSONL file."""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class VisibleMessage:
    """One completed user-visible Codex message."""

    timestamp: str
    role: str
    text: str
    phase: str | None = None


def find_rollout(thread_id: str, codex_home: Path) -> Path:
    """Return the newest active or archived rollout for an exact thread ID."""

    candidates: list[Path] = []
    expected_suffix = f"-{thread_id}.jsonl"
    for root_name in ("sessions", "archived_sessions"):
        root = codex_home / root_name
        if not root.is_dir():
            continue
        candidates.extend(
            path
            for path in root.rglob("rollout-*.jsonl")
            if path.is_file() and path.name.endswith(expected_suffix)
        )

    if not candidates:
        raise FileNotFoundError(
            f"No rollout found for thread {thread_id!r} below {codex_home}"
        )
    return max(candidates, key=lambda path: (path.stat().st_mtime_ns, str(path)))


def _content_text(content: Any) -> str:
    if not isinstance(content, list):
        return ""
    fragments = [
        part["text"]
        for part in content
        if isinstance(part, dict)
        and isinstance(part.get("text"), str)
        and part["text"]
    ]
    return "\n".join(fragments)


def _lines_with_last(source: Path) -> Iterator[tuple[int, str, bool]]:
    with source.open(encoding="utf-8") as handle:
        line = handle.readline()
        line_number = 1
        while line:
            following = handle.readline()
            yield line_number, line, not following
            line = following
            line_number += 1


def load_visible_messages(
    source: Path,
    *,
    expected_thread_id: str | None = None,
) -> tuple[list[VisibleMessage], int, str | None]:
    """Load completed UserMessage and AgentMessage records from a rollout."""

    messages: list[VisibleMessage] = []
    seen_items: dict[str, tuple[str, str, str | None, str]] = {}
    malformed_records = 0
    detected_thread_id: str | None = None

    for line_number, line, is_last in _lines_with_last(source):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as exc:
            # A live rollout can end in one record that is still being appended.
            if is_last and not line.endswith(("\n", "\r")):
                malformed_records += 1
                continue
            raise ValueError(f"Malformed JSON at {source}:{line_number}") from exc

        if not isinstance(record, dict) or record.get("type") != "event_msg":
            continue
        payload = record.get("payload")
        if not isinstance(payload, dict) or payload.get("type") != "item_completed":
            continue
        item = payload.get("item")
        if not isinstance(item, dict):
            continue

        item_type = item.get("type")
        if item_type not in {"UserMessage", "AgentMessage"}:
            continue
        text = _content_text(item.get("content"))
        if not text:
            continue

        thread_id = payload.get("thread_id")
        if not isinstance(thread_id, str) or not thread_id:
            raise ValueError(f"Visible item without a thread ID at {source}:{line_number}")
        if expected_thread_id is not None and thread_id != expected_thread_id:
            raise ValueError(
                f"Rollout thread {thread_id!r} does not match requested thread "
                f"{expected_thread_id!r} at {source}:{line_number}"
            )
        if detected_thread_id is None:
            detected_thread_id = thread_id
        elif thread_id != detected_thread_id:
            raise ValueError(
                f"Mixed thread IDs {detected_thread_id!r} and {thread_id!r} "
                f"at {source}:{line_number}"
            )

        phase = item.get("phase")
        normalized_phase = phase if isinstance(phase, str) and phase else None
        item_id = item.get("id")
        if isinstance(item_id, str) and item_id:
            signature = (thread_id, item_type, normalized_phase, text)
            previous = seen_items.get(item_id)
            if previous is not None:
                if previous != signature:
                    raise ValueError(
                        f"Conflicting completed item ID {item_id!r} "
                        f"at {source}:{line_number}"
                    )
                continue
            seen_items[item_id] = signature

        messages.append(
            VisibleMessage(
                timestamp=str(record.get("timestamp", "unknown time")),
                role="User" if item_type == "UserMessage" else "Assistant",
                text=text,
                phase=normalized_phase,
            )
        )

    if not messages:
        raise ValueError(f"No completed visible messages found in {source}")
    return messages, malformed_records, detected_thread_id


def render_markdown(
    messages: list[VisibleMessage],
    *,
    title: str,
    source_name: str,
    thread_id: str | None,
    malformed_records: int,
) -> str:
    """Render a recovery transcript without internal or tool records."""

    lines = [
        f"# {title}",
        "",
        "> Recovery copy generated from local Codex rollout data.",
        "> It contains only completed `UserMessage` and `AgentMessage` text.",
        "> It is a continuity aid, not product, architecture, or test authority.",
        "> Review visible text for secrets before committing this file.",
        "",
        f"- Thread: `{thread_id or 'unknown'}`",
        f"- Rollout: `{source_name}`",
        f"- Exported messages: {len(messages)}",
        f"- Malformed records skipped: {malformed_records}",
        "",
    ]
    for message in messages:
        phase = f" ({message.phase})" if message.phase else ""
        lines.extend(
            [
                f"## {message.timestamp} · {message.role}{phase}",
                "",
                message.text,
                "",
            ]
        )
    return "\n".join(lines).rstrip() + "\n"


def write_atomic(output: Path, content: str) -> None:
    """Write a transcript atomically beside its final path."""

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(content, encoding="utf-8", newline="\n")
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)


def _default_codex_home() -> Path:
    configured = os.environ.get("CODEX_HOME")
    return Path(configured).expanduser() if configured else Path.home() / ".codex"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export visible user/assistant text from a local Codex rollout."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--source", type=Path, help="Exact rollout JSONL path")
    source.add_argument("--thread-id", help="Codex thread ID to find locally")
    parser.add_argument(
        "--codex-home",
        type=Path,
        default=_default_codex_home(),
        help="Codex data directory (default: CODEX_HOME or ~/.codex)",
    )
    parser.add_argument("--output", type=Path, required=True, help="Markdown output path")
    parser.add_argument("--title", default="Codex visible transcript")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the transcript exporter CLI."""

    args = _build_parser().parse_args(argv)
    try:
        source = (args.source or find_rollout(args.thread_id, args.codex_home)).expanduser()
        output = args.output.expanduser()
        if source.resolve() == output.resolve():
            raise ValueError("Source rollout and Markdown output must be different files")
        messages, malformed_records, detected_thread_id = load_visible_messages(
            source,
            expected_thread_id=args.thread_id,
        )
        content = render_markdown(
            messages,
            title=args.title,
            source_name=source.name,
            thread_id=args.thread_id or detected_thread_id,
            malformed_records=malformed_records,
        )
        write_atomic(output, content)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(
        f"Exported {len(messages)} visible messages to {output} "
        f"({malformed_records} malformed records skipped)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
