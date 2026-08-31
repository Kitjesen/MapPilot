# ruff: noqa: D103, S101
from __future__ import annotations

import json
import os
from pathlib import Path

import pytest
from tools.docs.export_codex_thread import (
    find_rollout,
    load_visible_messages,
    main,
    render_markdown,
)


def _completed_item(
    item_type: str,
    item_id: str,
    text: str,
    *,
    phase: str | None = None,
) -> dict:
    item = {
        "type": item_type,
        "id": item_id,
        "content": [{"type": "Text", "text": text}],
    }
    if phase is not None:
        item["phase"] = phase
    return {
        "timestamp": "2026-08-24T00:00:00Z",
        "type": "event_msg",
        "payload": {
            "type": "item_completed",
            "thread_id": "thread-under-test",
            "item": item,
        },
    }


def test_export_keeps_only_completed_visible_messages(tmp_path: Path) -> None:
    source = tmp_path / "rollout-test-thread-under-test.jsonl"
    records = [
        {"type": "session_meta", "payload": {"instructions": "INTERNAL SYSTEM"}},
        _completed_item("UserMessage", "user-1", "hello"),
        _completed_item("Reasoning", "reasoning-1", "INTERNAL REASONING"),
        _completed_item("AgentMessage", "agent-1", "working", phase="commentary"),
        _completed_item("AgentMessage", "agent-2", "done", phase="final_answer"),
        _completed_item("UserMessage", "user-1", "hello"),
        {
            "type": "response_item",
            "payload": {
                "type": "message",
                "role": "assistant",
                "content": [{"type": "output_text", "text": "DUPLICATE RESPONSE"}],
            },
        },
        {
            "type": "response_item",
            "payload": {
                "type": "message",
                "role": "user",
                "content": [
                    {"type": "input_text", "text": "<recommended_plugins>INJECTED"}
                ],
            },
        },
        {
            "type": "event_msg",
            "payload": {
                "type": "item_completed",
                "item": {"type": "ToolOutput", "content": [{"text": "TOOL SECRET"}]},
            },
        },
    ]
    source.write_text(
        "\n".join(json.dumps(record) for record in records) + "\n{unfinished",
        encoding="utf-8",
    )

    messages, malformed, thread_id = load_visible_messages(source)
    markdown = render_markdown(
        messages,
        title="Recovered test",
        source_name=source.name,
        thread_id=thread_id,
        malformed_records=malformed,
    )

    assert [(message.role, message.phase, message.text) for message in messages] == [
        ("User", None, "hello"),
        ("Assistant", "commentary", "working"),
        ("Assistant", "final_answer", "done"),
    ]
    assert malformed == 1
    assert thread_id == "thread-under-test"
    assert "INTERNAL SYSTEM" not in markdown
    assert "INTERNAL REASONING" not in markdown
    assert "DUPLICATE RESPONSE" not in markdown
    assert "TOOL SECRET" not in markdown
    assert "Assistant (commentary)" in markdown
    assert "Assistant (final_answer)" in markdown


def test_middle_malformed_record_fails_instead_of_creating_output(
    tmp_path: Path,
) -> None:
    source = tmp_path / "rollout-thread-under-test.jsonl"
    output = tmp_path / "recovered.md"
    source.write_text(
        json.dumps(_completed_item("UserMessage", "user-1", "hello"))
        + "\n{broken}\n"
        + json.dumps(_completed_item("AgentMessage", "agent-1", "done"))
        + "\n",
        encoding="utf-8",
    )

    assert main(["--source", str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_conflicting_duplicate_item_and_mixed_threads_fail(tmp_path: Path) -> None:
    conflict = tmp_path / "conflict.jsonl"
    first = _completed_item("UserMessage", "same-id", "first")
    second = _completed_item("UserMessage", "same-id", "changed")
    conflict.write_text(
        "\n".join(json.dumps(record) for record in (first, second)) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="Conflicting completed item ID"):
        load_visible_messages(conflict)

    mixed = tmp_path / "mixed.jsonl"
    other_thread = _completed_item("AgentMessage", "agent-1", "done")
    other_thread["payload"]["thread_id"] = "other-thread"
    mixed.write_text(
        "\n".join(json.dumps(record) for record in (first, other_thread)) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="Mixed thread IDs"):
        load_visible_messages(mixed)

    single = tmp_path / "single.jsonl"
    single.write_text(json.dumps(first) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="does not match requested thread"):
        load_visible_messages(single, expected_thread_id="other-thread")


def test_source_cannot_be_overwritten_by_output_alias(tmp_path: Path) -> None:
    source = tmp_path / "rollout-thread-under-test.jsonl"
    original = json.dumps(_completed_item("UserMessage", "user-1", "hello")) + "\n"
    source.write_text(original, encoding="utf-8")

    alias = source.parent / "." / source.name
    assert main(["--source", str(source), "--output", str(alias)]) == 2
    assert source.read_text(encoding="utf-8") == original


def test_find_rollout_uses_exact_thread_id_and_newest_copy(tmp_path: Path) -> None:
    target_id = "11111111-1111-1111-1111-111111111111"
    other_id = "22222222-2222-2222-2222-222222222222"
    older = tmp_path / "archived_sessions" / f"rollout-old-{target_id}.jsonl"
    newer = tmp_path / "sessions" / "2026" / f"rollout-new-{target_id}.jsonl"
    other = tmp_path / "sessions" / f"rollout-new-{other_id}.jsonl"
    for path in (older, newer, other):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}\n", encoding="utf-8")
    os.utime(older, ns=(1, 1))
    os.utime(newer, ns=(2, 2))
    os.utime(other, ns=(3, 3))

    assert find_rollout(target_id, tmp_path) == newer
