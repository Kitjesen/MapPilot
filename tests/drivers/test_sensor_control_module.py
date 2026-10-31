"""Public sensor controls report only the source owner's matching ACK."""

import builtins
import importlib
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import pytest

from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlConflict, SensorControlRequest, SensorControlResult
from runtime.module import Module
from runtime.registry import get
from runtime.stream import Out


class Bridge:
    def __init__(self, results=()):
        self.results = deque(results)
        self.writes = []
        self.opened = []
        self.closed = 0

    def factory(self, **binding):
        self.opened.append(binding)
        return self

    def write_request(self, request):
        self.writes.append(request)

    def take_result(self):
        return self.results.popleft() if self.results else None

    def close(self):
        self.closed += 1


def request(**changes):
    return SensorControlRequest(
        **{
            "request_id": "request-1",
            "product_session_id": "product",
            "session_id": "source",
            "sensor_id": "thermal",
            "model_generation": 2,
            "reset_generation": 3,
            "deadline_s": time.time() + 1,
            "operation": SensorControlKind.SET_RATE,
            "rate_hz": 30.0,
            **changes,
        }
    )


def result(**changes):
    return SensorControlResult(
        **{
            "request_id": "request-1",
            "product_session_id": "product",
            "session_id": "source",
            "sensor_id": "thermal",
            "model_generation": 2,
            "reset_generation": 3,
            "applied_sim_time_ns": 123,
            "applied": True,
            "reason": "applied",
            "settings": {"rate_hz": 12.0},
            **changes,
        }
    )


def make_module(bridge, **changes):
    from drivers.sim.measurement.control_module import SensorControlModule

    return SensorControlModule(
        **{
            "product_session_id": "product",
            "session_id": "source",
            "domain_id": 6,
            "sensor_ids": ("thermal", "acoustic"),
            "model_generation": 2,
            "reset_generation": 3,
            "bridge_factory": bridge.factory,
            **changes,
        }
    )


def test_timeout_then_changed_request_cannot_accept_the_old_delayed_ack():
    bridge = Bridge()
    module = make_module(bridge)
    module.setup()
    original = request(deadline_s=time.time() + .02, rate_hz=10.)
    try:
        with pytest.raises(TimeoutError):
            module.execute(original)
        bridge.results.append(result(settings={"rate_hz": 10.}))
        changed = replace(original, deadline_s=time.time() + 1, rate_hz=20.)
        with pytest.raises(SensorControlConflict):
            module.execute(changed)
        assert len(bridge.writes) == 1 and module.state.msg_count == 0
        retry = module.execute(replace(original, deadline_s=time.time() + 1))
        assert retry.settings["rate_hz"] == 10.
    finally:
        module.close()


def test_exact_ack_is_returned_and_published_without_optimistic_settings():
    ack = result()
    bridge = Bridge([ack])
    module = make_module(bridge)
    received = []
    assert isinstance(module, Module) and module.layer == 1
    assert module.runtime_id == "sensor.controls"
    assert get("sensor_controls", "native_dds") is type(module)
    assert isinstance(module.state, Out) and module.state.msg_type is SensorControlResult
    assert bridge.opened == []
    module.state.subscribe(received.append)
    module.setup()
    assert module.state.msg_count == 0
    try:
        command = request()
        assert module.execute(command) is ack
        assert received == [ack] and received[0] is ack
        assert ack.settings == {"rate_hz": 12.0}
        assert bridge.writes == [command]
        assert bridge.opened == [
            {
                "domain_id": 6,
                "product_session_id": "product",
                "session_id": "source",
                "owner_mode": False,
            }
        ]
    finally:
        module.stop()
    assert bridge.closed == 1


@pytest.mark.parametrize(
    "changes",
    [
        {"request_id": "other-request"},
        {"product_session_id": "foreign-product"},
        {"session_id": "foreign-source"},
        {"sensor_id": "acoustic"},
        {"model_generation": 1},
        {"reset_generation": 2},
    ],
)
def test_other_ack_cannot_complete_or_publish_even_with_same_request_id(changes):
    ack = result(applied=False, reason="unsupported_operation", settings={})
    bridge = Bridge([result(**changes), ack])
    module = make_module(bridge)
    received = []
    module.state.subscribe(received.append)
    module.setup()
    try:
        assert module.execute(request()) is ack
        assert received == [ack]
        assert len(bridge.writes) == 1
        assert not bridge.results
    finally:
        module.close()


@pytest.mark.parametrize(
    "changes",
    [
        {"product_session_id": "foreign"},
        {"session_id": "foreign"},
        {"sensor_id": "undeclared"},
        {"model_generation": 1},
        {"reset_generation": 2},
    ],
)
def test_invalid_request_binding_is_rejected_before_write(changes):
    bridge = Bridge([result()])
    module = make_module(bridge)
    module.setup()
    try:
        with pytest.raises(ValueError):
            module.execute(request(**changes))
        assert bridge.writes == [] and module.state.msg_count == 0
    finally:
        module.close()


@pytest.fixture
def clock(monkeypatch):
    from drivers.sim.measurement import control_module

    seconds = [0.0]
    sleeps = []

    def sleep(duration):
        assert 0 < duration <= 0.01
        sleeps.append(duration)
        seconds[0] += duration

    monkeypatch.setattr(
        control_module,
        "time",
        SimpleNamespace(
            time=lambda: 1000.0 + seconds[0],
            monotonic=lambda: seconds[0],
            sleep=sleep,
        ),
    )
    return seconds, sleeps


@pytest.mark.parametrize("budget,elapsed,writes", [(-1.0, 0.0, 0), (0.025, 0.025, 1), (20.0, 2.0, 1)])
def test_deadline_and_two_second_cap_never_retry_or_publish(clock, budget, elapsed, writes):
    bridge = Bridge()
    module = make_module(bridge)
    module.setup()
    try:
        with pytest.raises(TimeoutError, match="timed out"):
            module.execute(request(deadline_s=1000.0 + budget))
        assert clock[0][0] == pytest.approx(elapsed)
        assert len(bridge.writes) == writes
        assert module.state.msg_count == 0
        assert bool(clock[1]) == (writes == 1)
    finally:
        module.close()


def test_ack_received_after_budget_is_not_published(clock):
    class LateBridge(Bridge):
        def take_result(self):
            clock[0][0] += 0.1
            return result()

    bridge = LateBridge()
    module = make_module(bridge)
    module.setup()
    try:
        with pytest.raises(TimeoutError, match="timed out"):
            module.execute(request(deadline_s=1000.05))
        assert module.state.msg_count == 0
    finally:
        module.close()


class GatedBridge(Bridge):
    def __init__(self):
        super().__init__()
        self.polling = Event()
        self.release = Event()
        self.pending = None

    def write_request(self, command):
        assert self.closed == 0
        assert self.pending is None, "requests must be serialized until ACK"
        super().write_request(command)
        self.pending = command

    def take_result(self):
        assert self.closed == 0
        self.polling.set()
        if not self.release.is_set():
            return None
        command, self.pending = self.pending, None
        return result(request_id=command.request_id)


def test_concurrent_requests_are_serialized_until_matching_ack():
    bridge = GatedBridge()
    module = make_module(bridge)
    module.setup()
    second_entered = Event()

    def second():
        second_entered.set()
        return module.execute(request(request_id="request-2"))

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(module.execute, request())
            assert bridge.polling.wait(1)
            following = pool.submit(second)
            try:
                assert second_entered.wait(1)
                time.sleep(0.025)
                assert len(bridge.writes) == 1 and not following.done()
            finally:
                bridge.release.set()
            assert first.result(timeout=1).request_id == "request-1"
            assert following.result(timeout=1).request_id == "request-2"
        assert len(bridge.writes) == 2 and module.state.msg_count == 2
    finally:
        bridge.release.set()
        module.close()


def test_queue_wait_consumes_request_budget_without_writing():
    bridge = GatedBridge()
    module = make_module(bridge)
    module.setup()
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(module.execute, request())
            assert bridge.polling.wait(1)
            following = pool.submit(module.execute, request(request_id="request-2", deadline_s=time.time() + 0.04))
            try:
                with pytest.raises(TimeoutError, match="timed out"):
                    following.result(timeout=0.5)
                assert len(bridge.writes) == 1 and module.state.msg_count == 0
            finally:
                bridge.release.set()
            assert first.result(timeout=1).request_id == "request-1"
    finally:
        bridge.release.set()
        module.close()


def test_close_waits_for_active_ack_and_closes_once_for_concurrent_callers():
    bridge = GatedBridge()
    module = make_module(bridge)
    module.setup()
    closing = Event()

    def close():
        closing.set()
        module.close()

    with ThreadPoolExecutor(max_workers=3) as pool:
        active = pool.submit(module.execute, request())
        assert bridge.polling.wait(1)
        closed = pool.submit(close)
        try:
            assert closing.wait(1)
            other_close = pool.submit(module.stop)
            time.sleep(0.025)
            assert bridge.closed == 0 and not closed.done()
        finally:
            bridge.release.set()
        assert active.result(timeout=1).request_id == "request-1"
        closed.result(timeout=1)
        other_close.result(timeout=1)
    module.close()
    assert bridge.closed == 1 and not module.running
    with pytest.raises(RuntimeError, match="closed"):
        module.execute(request())
    with pytest.raises(RuntimeError, match="closed"):
        module.setup()


def test_binding_is_an_immutable_copy_of_declared_sensor_and_generations():
    sensors = ["thermal"]
    module = make_module(Bridge(), sensor_ids=sensors)
    sensors.append("undeclared")
    try:
        binding = module.binding("thermal")
        assert dict(binding) == {
            "product_session_id": "product",
            "session_id": "source",
            "sensor_id": "thermal",
            "model_generation": 2,
            "reset_generation": 3,
        }
        with pytest.raises(TypeError):
            binding["reset_generation"] = 99
        with pytest.raises(ValueError, match="sensor"):
            module.binding("undeclared")
        assert module.binding("thermal") is not binding
    finally:
        module.close()


def test_readiness_means_bridge_setup_only_and_lifecycle_is_one_way():
    bridge = Bridge()
    module = make_module(bridge)
    assert module.startup_readiness() is not None
    with pytest.raises(RuntimeError, match="setup"):
        module.execute(request())
    with pytest.raises(RuntimeError, match="setup"):
        module.start()
    module.setup()
    module.setup()
    assert len(bridge.opened) == 1
    assert module.startup_readiness() is None
    assert bridge.writes == [] and module.state.msg_count == 0
    module.start()
    assert module.running
    module.close()
    assert module.startup_readiness() is not None
    with pytest.raises(RuntimeError, match="closed"):
        module.start()


def test_import_and_default_constructor_do_not_import_or_open_native_bridge(monkeypatch):
    from drivers.sim.measurement import control_module

    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        assert "control_bridge" not in name and name.split(".")[0] not in {"ctypes", "cyclonedds", "rti", "fastdds"}
        return original_import(name, *args, **kwargs)

    with monkeypatch.context() as scope:
        scope.setattr(builtins, "__import__", guarded_import)
        importlib.reload(control_module)
        module = make_module(Bridge(), bridge_factory=None)
        assert module.startup_readiness() is not None

    from drivers.sim.measurement import control_bridge

    bridge = Bridge()
    monkeypatch.setattr(control_bridge, "NativeSensorControlBridge", bridge.factory)
    try:
        assert bridge.opened == []
        module.setup()
        assert bridge.opened[0]["owner_mode"] is False
    finally:
        module.close()


@pytest.mark.parametrize("method", ["factory", "write_request", "take_result", "close"])
def test_bridge_failures_are_explicit_runtime_errors_and_cleanup_is_once(method):
    bridge = Bridge()

    def fail(*args, **kwargs):
        if method == "close":
            bridge.closed += 1
        raise OSError("endpoint failed")

    setattr(bridge, method, fail)
    module = make_module(bridge)
    try:
        with pytest.raises(RuntimeError, match="endpoint failed"):
            if method == "factory":
                module.setup()
            else:
                module.setup()
                if method == "close":
                    module.close()
                else:
                    module.execute(request())
        assert module.state.msg_count == 0
    finally:
        module.close()
        module.stop()
    assert bridge.closed == (0 if method == "factory" else 1)


@pytest.mark.parametrize(
    "changes",
    [
        {"domain_id": -1},
        {"domain_id": 233},
        {"domain_id": True},
        {"sensor_ids": "thermal"},
        {"sensor_ids": []},
        {"sensor_ids": [""]},
        {"product_session_id": ""},
        {"session_id": " source"},
        {"session_id": "bad\0id"},
        {"product_session_id": "é" * 128},
        {"model_generation": -1},
        {"reset_generation": True},
        {"reset_generation": 2**64},
    ],
)
def test_invalid_constructor_is_rejected_before_open(changes):
    bridge = Bridge()
    with pytest.raises(ValueError):
        make_module(bridge, **changes)
    assert bridge.opened == []


def test_close_without_setup_never_opens_bridge_and_cannot_reopen():
    bridge = Bridge()
    module = make_module(bridge)
    module.close()
    module.stop()
    with pytest.raises(RuntimeError, match="closed"):
        module.setup()
    assert bridge.opened == [] and bridge.closed == 0


def test_close_waits_for_request_timeout_without_closing_under_poll():
    bridge = GatedBridge()
    module = make_module(bridge)
    module.setup()
    with ThreadPoolExecutor(max_workers=2) as pool:
        active = pool.submit(module.execute, request(deadline_s=time.time() + 0.1))
        assert bridge.polling.wait(1)
        started = time.monotonic()
        closed = pool.submit(module.close)
        with pytest.raises(TimeoutError, match="timed out"):
            active.result(timeout=1)
        closed.result(timeout=1)
        assert time.monotonic() - started < 0.5
    assert bridge.closed == 1 and module.state.msg_count == 0
    module.close()
    assert bridge.closed == 1


def test_failed_request_releases_serialization_for_following_request():
    class FailingOnceBridge(Bridge):
        def write_request(self, command):
            super().write_request(command)
            if len(self.writes) == 1:
                raise OSError("write failed")

    ack = result(request_id="request-2")
    bridge = FailingOnceBridge([ack])
    module = make_module(bridge)
    module.setup()
    try:
        with pytest.raises(RuntimeError, match="write failed"):
            module.execute(request())
        assert module.state.msg_count == 0
        assert module.execute(request(request_id="request-2")) is ack
        assert len(bridge.writes) == 2 and module.state.msg_count == 1
    finally:
        module.close()


def test_duplicate_ack_is_returned_from_bridge_without_repeated_state_publication():
    first, duplicate = result(), result()
    bridge = Bridge([first, duplicate])
    module = make_module(bridge)
    received = []
    module.state.subscribe(received.append)
    module.setup()
    try:
        assert module.execute(request()) is first
        assert module.execute(request()) is duplicate
        assert received == [first] and module.state.msg_count == 1
        assert len(bridge.writes) == 2
    finally:
        module.close()


def test_ack_publication_deduplication_has_a_256_request_bound():
    bridge = Bridge([result(request_id=f"request-{index}") for index in range(257)] + [result(request_id="request-0")])
    module = make_module(bridge)
    module.setup()
    try:
        for index in range(257):
            module.execute(request(request_id=f"request-{index}"))
        module.execute(request(request_id="request-0"))
        assert module.state.msg_count == 258
    finally:
        module.close()
