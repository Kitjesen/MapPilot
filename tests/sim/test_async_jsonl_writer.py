from __future__ import annotations

import json
import threading
import time

from sim.scripts.mujoco.async_jsonl_writer import AsyncJsonlWriter


def test_async_jsonl_submit_does_not_block_behind_slow_sink_and_drops_when_full(tmp_path):
    first_started = threading.Event()
    release_first = threading.Event()
    written: list[dict[str, int]] = []

    def slow_sink(record: dict[str, int]) -> None:
        written.append(record)
        if len(written) == 1:
            first_started.set()
            assert release_first.wait(timeout=2.0)

    writer = AsyncJsonlWriter(
        tmp_path / "unused.jsonl",
        max_pending=2,
        record_sink=slow_sink,
    )
    assert writer.submit({"id": 1})
    assert first_started.wait(timeout=2.0)

    started = time.perf_counter()
    assert writer.submit({"id": 2})
    assert writer.submit({"id": 3})
    assert writer.submit({"id": 4}) is False
    assert time.perf_counter() - started < 0.10

    release_first.set()
    diagnostics = writer.close()

    assert [record["id"] for record in written] == [1, 2, 3]
    assert diagnostics == {
        "submitted": 4,
        "written": 3,
        "dropped": 1,
        "failures": 0,
        "max_pending": 2,
    }


def test_async_jsonl_default_sink_writes_complete_records(tmp_path):
    path = tmp_path / "records.jsonl"
    writer = AsyncJsonlWriter(path, flush_every=2)

    assert writer.submit({"id": 1, "ready": True})
    assert writer.submit({"id": 2, "ready": False})
    diagnostics = writer.close()

    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert records == [
        {"id": 1, "ready": True},
        {"id": 2, "ready": False},
    ]
    assert diagnostics["written"] == 2
    assert diagnostics["failures"] == 0
