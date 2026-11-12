import csv
import io
import json
import queue
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.observations import register_observation_routes
from gateway.services.imu_monitor import ImuMonitor


class Source:
    def __init__(self):
        self.queue = queue.Queue()
        self.started = threading.Event()
        self.cancelled = threading.Event()
        self.calls = 0

    def capabilities(self):
        return {"provider": "fixture", "sensors": {"imu": {"stream": True, "clock": "source_elapsed"}}}

    @contextmanager
    def stream(self, sensor):
        self.calls += 1
        self.cancelled.clear()
        self.started.set()
        def samples():
            while not self.cancelled.is_set():
                try:
                    yield self.queue.get(timeout=.05)
                except queue.Empty:
                    continue
        try:
            yield samples(), self.cancelled.set
        finally:
            self.cancelled.set()


def sample(z=1, seconds=1):
    return {"angular_velocity_rad_s": {"x": 0., "y": .1, "z": z}, "source_elapsed": {"seconds": seconds, "nanos": 0}}


def wait_samples(monitor, count):
    deadline = time.monotonic() + 3
    while monitor.sequence < count and time.monotonic() < deadline:
        time.sleep(.01)
    assert monitor.sequence >= count


@pytest.fixture
def monitor(tmp_path):
    service = ImuMonitor(Source(), tmp_path, "fixture-robot")
    yield service
    service.close()


def test_live_and_record_share_one_source_and_closing_the_view_does_not_stop_record(monitor):
    assert monitor.snapshot()["state"] == "connecting"
    assert monitor.source.started.wait(2)
    record = monitor.start_record()
    assert monitor.start_record()["id"] == record["id"]
    monitor.requested = -1000
    monitor.source.queue.put(sample(2))
    monitor.source.queue.put(sample(3, seconds=2))
    wait_samples(monitor, 2)
    assert monitor.source.calls == 1
    assert monitor.snapshot()["recording"]["sample_count"] == 2
    with pytest.raises(ValueError, match="identity"):
        monitor.stop_record("imu-wrong")
    assert monitor.file is not None
    result = monitor.stop_record(record["id"])
    assert result["state"] == "completed" and result["sample_count"] == 2
    history = monitor.read_record(record["id"])
    assert history["summary"]["axes"]["angular_velocity_rad_s.z"]["mean"] == 2.5
    rows = list(csv.DictReader(io.StringIO("".join(monitor.csv(record["id"])))))
    assert [row["angular_velocity_rad_s.z"] for row in rows] == ['2', '3']
    assert all(row["linear_acceleration_m_s2.x"] == '' for row in rows)
    assert [row["source_seconds"] for row in rows] == ['1', '2']


def test_rollback_is_a_gap_and_reconnect_preserves_all_written_values(monitor):
    record = monitor.start_record()
    monitor._append(sample(1, seconds=10))
    monitor._append(sample(2, seconds=9))
    assert monitor.samples[-1]["gap_before"]
    monitor.close()
    reopened = ImuMonitor(None, monitor.root)
    history = reopened.read_record(record["id"])
    assert history["record"]["state"] == "interrupted"
    assert history["record"]["sample_count"] == 2
    assert len(history["samples"]) == 2


def test_partial_record_keeps_complete_lines_and_recovers_the_count(monitor):
    record = monitor.start_record()
    monitor._append(sample(7))
    with monitor.lock:
        monitor.file.write('{"type":"sample"')
        monitor.file.close()
        monitor.file = None
    reopened = ImuMonitor(None, monitor.root)
    result = reopened.read_record(record["id"])
    assert result["record"]["state"] == "interrupted" and result["record"]["sample_count"] == 1
    assert result["samples"][0]["values"]["angular_velocity_rad_s"]["z"] == 7
    assert len(list(csv.DictReader(io.StringIO("".join(reopened.csv(record["id"])))))) == 1


def test_compressed_history_preserves_spikes_and_exact_full_interval_statistics(monitor):
    record = monitor.start_record()
    total = 4000
    for index in range(total):
        monitor._append(sample(99 if index == 1733 else 1, seconds=index))
    monitor.stop_record(record["id"])
    history = monitor.read_record(record["id"])
    assert len(history["samples"]) < total
    assert any(row["values"]["angular_velocity_rad_s"]["z"] == 99 for row in history["samples"])
    summary = history["summary"]["axes"]["angular_velocity_rad_s.z"]
    assert summary["count"] == total and summary["max"] == 99
    assert summary["mean"] == pytest.approx((3999 + 99) / total)


def test_write_failure_is_not_reported_as_saved(monitor):
    record = monitor.start_record()
    original = monitor._write
    def fail(value):
        if value.get('type') == 'sample':
            raise OSError('disk full')
        return original(value)
    monitor._write = fail
    monitor._append(sample())
    assert monitor.record["state"] == "failed" and monitor.file is None
    assert monitor.read_record(record["id"])["record"]["error"] == 'disk full'


def test_http_record_roundtrip_and_unavailable_source(monitor):
    app = FastAPI()
    register_observation_routes(app, SimpleNamespace(_imu_monitor=monitor))
    with TestClient(app) as client:
        record = client.post('/api/v1/robot/imu/records').json()
        monitor._append(sample(.5))
        assert client.get(f"/api/v1/robot/imu/records/{record['id']}/download").status_code == 409
        assert client.post('/api/v1/robot/imu/records/imu-other/stop').status_code == 409
        assert client.post(f"/api/v1/robot/imu/records/{record['id']}/stop").json()['state'] == 'completed'
        assert client.get('/api/v1/robot/imu/records').json()['records'][0]['id'] == record['id']
        response = client.get(f"/api/v1/robot/imu/records/{record['id']}/download")
        assert response.status_code == 200 and 'angular_velocity_rad_s.z' in response.text
        assert client.get(f"/api/v1/robot/imu/records/{record['id']}/download?format=jsonl").status_code == 200
        assert client.get(f"/api/v1/robot/imu/records/{record['id']}?start=10&end=1").status_code == 422
        assert client.get('/api/v1/robot/imu/records/invalid').status_code == 409
    unavailable = ImuMonitor(None, monitor.root)
    with pytest.raises(ValueError, match='no configured'):
        unavailable.snapshot()


def test_history_compression_does_not_join_across_an_unselected_gap(monitor):
    record = monitor.start_record()
    for index in range(40):
        if index == 20:
            monitor.gap = True
        monitor._append(sample(1, seconds=index))
    monitor.stop_record(record["id"])
    path = monitor.path(record["id"])
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    # Put all measurements in one display bucket; the second gap must survive even if its row is omitted.
    for row in rows:
        if row.get("type") == "sample":
            row["received_at_s"] = record["started_at_s"] + row["seq"] * .001
        elif row.get("type") == "footer":
            row["ended_at_s"] = record["started_at_s"] + 1000
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    history = monitor.read_record(record["id"])
    assert len(history["samples"]) < 40 and history["samples"][-1]["gap_before"]


def test_incremental_reads_keep_sequence_order_and_report_truncation(monitor):
    for index in range(3500):
        monitor._append(sample(1, seconds=index))
    assert len(monitor.snapshot()["samples"]) == 3500
    result = monitor.snapshot(after=3490)
    assert [row["seq"] for row in result["samples"]] == list(range(3491, 3501))
    assert not result["truncated"]
    result = monitor.snapshot(after=1)
    assert len(result["samples"]) == 3000 and result["samples"][0]["seq"] == 501
    assert result["truncated"]
