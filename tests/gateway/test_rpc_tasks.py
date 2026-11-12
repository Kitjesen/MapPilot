import socket
import threading
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc import services, task_run
from gateway.rpc.codec import fields
from gateway.rpc.generated import tasks_pb2 as pb
from gateway.rpc.generated import tasks_pb2_grpc as rpc
from gateway.rpc.server import start


@contextmanager
def serve(agent=None):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    gateway = SimpleNamespace(_all_modules={"AgentPlannerModule": agent} if agent else {})
    server = start(gateway, address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield rpc.TasksStub(channel)
        finally:
            server.stop(0).wait(timeout=3)


@pytest.mark.parametrize("terminal", ["done", "failed", "cancelled"])
def test_watch_delivers_snapshots_and_terminal_without_commands(terminal):
    values = iter(["running", "running", terminal])
    agent = SimpleNamespace(get_agent_status=lambda: {"run_id": "run-1", "state": next(values)})
    with serve(agent) as client:
        updates = list(client.Watch(pb.RunQuery(run_id="run-1"), timeout=3))
    assert [item.details.fields["state"].string_value for item in updates] == ["running", "running", terminal]
    assert all(item.run_id == "run-1" and item.found for item in updates)


def test_watch_reports_missing_run_then_ends():
    with serve() as client:
        updates = list(client.Watch(pb.RunQuery(run_id="run-1"), timeout=3))
        assert len(updates) == 1 and not updates[0].found
        assert updates[0].details.fields["reason"].string_value == "agent_unavailable"
        with pytest.raises(grpc.RpcError) as error:
            next(client.Watch(pb.RunQuery(run_id=" "), timeout=3))
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT


def test_watch_closing_releases_slot_without_cancelling_task():
    agent = SimpleNamespace(get_agent_status=lambda: {"run_id": "run-1", "state": "running"})
    gateway = SimpleNamespace(_all_modules={"AgentPlannerModule": agent})
    slots = threading.BoundedSemaphore(1)
    service = services.Tasks(gateway, slots)
    context = SimpleNamespace(add_callback=lambda _: True, is_active=lambda: True)
    watching = service.Watch(pb.RunQuery(run_id="run-1"), context)
    assert next(watching).found
    assert not slots.acquire(blocking=False)
    watching.close()
    assert slots.acquire(blocking=False)


@pytest.mark.parametrize(("read_times", "late_by", "expected_reads", "expected_waits"), [
    ([0.007] * 4, 0.003, [0.0, 0.023, 0.043, 0.063], [0.013, 0.010, 0.010]),
    ([0.055, 0.005, 0.005, 0.005], 0.0, [0.0, 0.055, 0.075, 0.095], [0.0, 0.015, 0.015]),
    ([0.015] * 4, 0.010, [0.0, 0.030, 0.045, 0.060], [0.005, 0.0, 0.0]),
])
def test_watch_cadence_accounts_for_read_time_and_late_wakeup(
    monkeypatch, read_times, late_by, expected_reads, expected_waits,
):
    clock = [0.0]
    read_at = []
    waits = []
    read_times = iter(read_times)

    def wait(delay):
        waits.append(delay)
        clock[0] += delay + (late_by if delay > 1e-9 else 0.0)
        return False

    def read():
        read_at.append(clock[0])
        clock[0] += next(read_times)
        return pb.Run(
            run_id="run-1", found=True,
            details=fields({"state": "done" if len(read_at) == 4 else "running"}),
        )

    monkeypatch.setattr(task_run, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    monkeypatch.setattr(task_run, "threading", SimpleNamespace(Event=lambda: SimpleNamespace(wait=wait, set=lambda: None)))
    slots = threading.BoundedSemaphore(1)
    context = SimpleNamespace(add_callback=lambda _: True, is_active=lambda: True)
    updates = list(task_run.watch(context, slots, read))
    assert len(updates) == 4
    assert read_at == pytest.approx(expected_reads)
    assert waits == pytest.approx(expected_waits)
    assert slots.acquire(blocking=False)


def test_watch_cancellation_wakes_wait_and_releases_slot(monkeypatch):
    stopped = threading.Event()
    callbacks = []
    slots = threading.BoundedSemaphore(1)
    context = SimpleNamespace(add_callback=lambda callback: callbacks.append(callback) or True, is_active=lambda: True)
    monkeypatch.setattr(task_run, "threading", SimpleNamespace(Event=lambda: stopped))
    watching = task_run.watch(
        context, slots,
        lambda: pb.Run(run_id="run-1", found=True, details=fields({"state": "running"})),
    )
    assert next(watching).found
    callbacks[0]()
    assert list(watching) == []
    assert slots.acquire(blocking=False)
