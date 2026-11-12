"""Navigation task snapshots keep their cadence and owner terminal semantics."""

import threading
from types import SimpleNamespace

import pytest

from gateway.rpc import services
from gateway.rpc.generated import navigation_pb2 as pb


@pytest.mark.parametrize(("read_times", "late_by", "expected_reads", "expected_waits"), [
    ([0.007] * 4, 0.003, [0.0, 0.023, 0.043, 0.063], [0.013, 0.010, 0.010]),
    ([0.055, 0.005, 0.005, 0.005], 0.0, [0.0, 0.055, 0.075, 0.095], [0.0, 0.015, 0.015]),
    ([0.015] * 4, 0.010, [0.0, 0.030, 0.045, 0.060], [0.005, 0.0, 0.0]),
])
@pytest.mark.parametrize("terminal", [pb.TASK_SUCCESS, pb.TASK_FAILED, pb.TASK_CANCELLED])
def test_watch_cadence_and_terminal_release(monkeypatch, read_times, late_by, expected_reads, expected_waits, terminal):
    clock = [0.0]
    read_at = []
    waits = []
    read_times = iter(read_times)

    def wait(delay):
        waits.append(delay)
        clock[0] += delay + (late_by if delay > 1e-9 else 0.0)
        return False

    def read(_request, _context):
        read_at.append(clock[0])
        clock[0] += next(read_times)
        return pb.Task(task_id="task-1", found=True, state=terminal if len(read_at) == 4 else pb.TASK_EXECUTING)

    slots = threading.BoundedSemaphore(1)
    service = services.Navigation(SimpleNamespace(), slots)
    service.Task = read
    monkeypatch.setattr(services, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(services, "threading", SimpleNamespace(Event=lambda: SimpleNamespace(wait=wait, set=lambda: None)))
    context = SimpleNamespace(add_callback=lambda _: True, is_active=lambda: True)
    updates = list(service.Watch(pb.TaskQuery(task_id="task-1"), context))
    assert read_at == pytest.approx(expected_reads)
    assert waits == pytest.approx(expected_waits)
    assert [update.state for update in updates] == [pb.TASK_EXECUTING] * 3 + [terminal]
    assert slots.acquire(blocking=False)


def test_watch_not_found_continues_and_cancellation_releases_slot():
    slots = threading.BoundedSemaphore(1)
    service = services.Navigation(SimpleNamespace(), slots)
    service.Task = lambda _request, _context: pb.Task(task_id="task-1", found=False)
    callbacks = []
    context = SimpleNamespace(add_callback=lambda callback: callbacks.append(callback) or True, is_active=lambda: True)
    watching = service.Watch(pb.TaskQuery(task_id="task-1"), context)
    assert not next(watching).found
    assert not next(watching).found
    callbacks[0]()
    assert list(watching) == []
    assert slots.acquire(blocking=False)
