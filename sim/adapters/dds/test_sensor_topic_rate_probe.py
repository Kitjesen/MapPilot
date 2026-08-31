from __future__ import annotations

import importlib
import json
import os
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
from contextlib import AbstractContextManager, contextmanager
from io import StringIO
from itertools import groupby
from pathlib import Path
from typing import NamedTuple, Protocol, TextIO, TypedDict, cast
from unittest.mock import patch


def _domain_id_from_environment() -> int:
    module_name = ".test_dds_domain" if __package__ else "test_dds_domain"
    module = importlib.import_module(module_name, package=__package__)
    resolver = cast(Callable[[], int], getattr(module, "domain_id_from_environment"))
    return resolver()


class SensorWriter(Protocol):
    def write(self, payload: bytes) -> int: ...


class SensorClient(SensorWriter, Protocol):
    def close(self) -> None: ...


class FeederReport(TypedDict):
    stream: str
    sent: int
    warmup_sent: int
    warmup_dropped: int
    dropped: int
    warmup_max_late_s: float
    max_late_s: float


class FeedStats(NamedTuple):
    sent: int
    warmup_sent: int
    warmup_dropped: int
    dropped: int
    warmup_max_late_s: float
    max_late_s: float


class TopicMetric(TypedDict):
    topic: str
    samples: int
    source_hz: float
    source_max_interval_s: float
    source_p95_jitter_s: float
    source_inferred_drop_rate: float
    dds_write_actual_hz: float
    dds_write_max_interval_s: float
    dds_write_p95_jitter_s: float
    take_max_interval_s: float
    take_p95_jitter_s: float
    first_sample_delay_s: float
    tail_silence_s: float
    window_delivery_ratio: float
    dds_sample_lost: int
    production_samples: int
    production_source_inferred_drop_rate: float
    production_window_delivery_ratio: float
    production_dds_sample_lost: int
    production_sample_rejected: int


class CameraInfoMetric(TypedDict):
    topic: str
    late_joiner_received: bool


class RawPacketMetric(TypedDict):
    topic: str
    writer_count: int


RATE_WARMUP_S = 0.5
CAMERA_PAYLOAD_WINDOW_S = 4.0
CAMERA_PAYLOAD_TAIL_MARGIN_S = 0.2


def _record(record_type: int, timestamp_ns: int, sequence: int, payload: bytes) -> bytes:
    return (
        b"LTU1"
        + bytes((record_type, 0, 0, 0))
        + struct.pack("<QIII", timestamp_ns, sequence, 1, len(payload))
        + payload
    )


def _lidar_records(base_ns: int, seconds: float) -> list[tuple[float, bytes]]:
    point = struct.pack("<4fIBBH", 1.0, 2.0, 3.0, 42.0, 0, 7, 8, 0)
    return [
        (index / 10.0, _record(1, base_ns + index * 100_000_000, index, point))
        for index in range(int(seconds * 10) + 1)
    ]


def _imu_records(base_ns: int, seconds: float) -> list[tuple[float, bytes]]:
    sample = struct.pack("<6f", 0.25, -0.5, 0.75, 0.15, 0.25, 9.81)
    return [
        (index / 200.0, _record(2, base_ns + index * 5_000_000, index, sample))
        for index in range(int(seconds * 200) + 1)
    ]


def _camera_header(
    kind: int,
    timestamp_ns: int,
    payload_size: int,
    *,
    width: int = 2,
    height: int = 2,
) -> bytes:
    if kind == 1:
        channels, pixel_format, fx, fy, cx, cy = 0, 0, 100.0, 101.0, 1.0, 1.0
    elif kind == 2:
        channels, pixel_format, fx, fy, cx, cy = 3, 1, 0.0, 0.0, 0.0, 0.0
    else:
        channels, pixel_format, fx, fy, cx, cy = 1, 3, 0.0, 0.0, 0.0, 0.0
    return struct.pack(
        "<4sHHIIIIddddddIddddd",
        b"LTOB",
        2,
        kind,
        width,
        height,
        channels,
        pixel_format,
        timestamp_ns / 1_000_000_000.0,
        fx,
        fy,
        cx,
        cy,
        0.001,
        payload_size,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _camera_records(base_ns: int, seconds: float) -> list[tuple[float, bytes]]:
    output: list[tuple[float, bytes]] = []
    intrinsics = _camera_header(1, base_ns, 0)
    output.append((0.0, _record(5, base_ns, 0, intrinsics)))
    period_ns = 1_000_000_000 // 30
    sequence = 1
    for index in range(int(seconds * 30) + 1):
        timestamp_ns = base_ns + index * period_ns
        color = bytes(range(12))
        depth = struct.pack("<4H", 1000, 1100, 1200, 1300)
        offset_s = index / 30.0
        output.append(
            (
                offset_s,
                _record(
                    5,
                    timestamp_ns,
                    sequence,
                    _camera_header(2, timestamp_ns, len(color)) + color,
                ),
            )
        )
        sequence += 1
        output.append(
            (
                offset_s,
                _record(
                    5,
                    timestamp_ns,
                    sequence,
                    _camera_header(3, timestamp_ns, len(depth)) + depth,
                ),
            )
        )
        sequence += 1
    return output


def _full_camera_records(base_ns: int, seconds: float) -> Iterator[tuple[float, bytes]]:
    """Yield production-sized RGB-D records one frame at a time."""
    width, height = 640, 480
    color = bytes(width * height * 3)
    depth = bytes(width * height * 2)
    yield (
        0.0,
        _record(
            5,
            base_ns,
            0,
            _camera_header(1, base_ns, 0, width=width, height=height),
        ),
    )
    period_ns = 1_000_000_000 // 30
    for index in range(int(seconds * 30) + 1):
        timestamp_ns = base_ns + index * period_ns
        offset_s = index / 30.0
        yield (
            offset_s,
            _record(
                5,
                timestamp_ns,
                2 * index + 1,
                _camera_header(
                    2,
                    timestamp_ns,
                    len(color),
                    width=width,
                    height=height,
                )
                + color,
            ),
        )
        yield (
            offset_s,
            _record(
                5,
                timestamp_ns,
                2 * index + 2,
                _camera_header(
                    3,
                    timestamp_ns,
                    len(depth),
                    width=width,
                    height=height,
                )
                + depth,
            ),
        )


def _process_environment(session: Path, product_session_id: str) -> dict[str, str]:
    plan = session / f"plan-{product_session_id}.json"
    plan.write_text("{}\n", encoding="utf-8")
    return {
        **os.environ,
        "LINGTU_RUN_PLAN": str(plan),
        "LINGTU_PRODUCT_SESSION_ID": product_session_id,
        "LINGTU_PRODUCT": "teleop_avoid",
        "LINGTU_ENV": "sim",
        "LINGTU_ENV_BACKEND": "mujoco",
        "LINGTU_SESSION_ROOT": str(session),
    }


def _start_publisher(
    executable: Path, session: Path, product_session_id: str, domain_id: int, stream: str
) -> subprocess.Popen[bytes]:
    command = [
        str(executable),
        "--local-endpoint",
        "--dds",
        "--stream",
        stream,
        "--domain-id",
        str(domain_id),
        "--scan-window",
        "0",
    ]
    if os.name == "nt":
        return subprocess.Popen(  # noqa: S603
            command,
            cwd=str(session),
            env=_process_environment(session, product_session_id),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
    return subprocess.Popen(  # noqa: S603
        command,
        cwd=str(session),
        env=_process_environment(session, product_session_id),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )


def _start_feeder(
    repository: Path,
    readiness: Path,
    role: str,
    product_session_id: str,
    stream: str,
    duration_s: float,
    base_ns: int,
) -> subprocess.Popen[str]:
    return subprocess.Popen(  # noqa: S603
        [
            sys.executable,
            str(Path(__file__).resolve()),
            "--feeder",
            str(repository),
            str(readiness),
            role,
            product_session_id,
            stream,
            str(duration_s),
            str(base_ns),
        ],
        cwd=str(repository),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _signal(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    if os.name == "nt":
        process.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        process.send_signal(signal.SIGTERM)


def _wait_process(process: subprocess.Popen[bytes]) -> None:
    try:
        process.communicate(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate(timeout=5.0)


def _wait_ready(path: Path, process: subprocess.Popen[bytes]) -> None:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if path.exists():
            return
        if process.poll() is not None:
            stdout, stderr = process.communicate()
            raise RuntimeError(f"publisher exited: stdout={stdout!r} stderr={stderr!r}")
        time.sleep(0.02)
    raise RuntimeError(f"publisher readiness timed out: {path}")


def _wait_feeder_ready(stream: str, process: subprocess.Popen[str]) -> None:
    if process.stdout is None:
        raise RuntimeError(f"{stream} feeder stdout pipe was not created")
    if process.stdout.readline().strip() == "READY":
        return
    stdout, stderr = process.communicate(timeout=2.0)
    raise RuntimeError(f"{stream} feeder did not become ready: stdout={stdout!r} stderr={stderr!r}")


def _cleanup_processes(
    *,
    probe: subprocess.Popen[str] | None,
    feeders: dict[str, subprocess.Popen[str]],
    publishers: dict[str, subprocess.Popen[bytes]],
    clients: dict[str, SensorClient],
) -> list[str]:
    """Stop, disconnect, then reap every test resource, aggregating failures."""
    errors: list[str] = []

    # Stage 1: request termination for every child before waiting on any child.
    text_processes = ({"probe": probe} if probe is not None else {}) | {
        f"feeder {name}": process for name, process in feeders.items()
    }
    for name, text_process in text_processes.items():
        if text_process.poll() is None:
            try:
                text_process.kill()
            except Exception as error:
                errors.append(f"terminate {name}: {error}")
    for name, publisher_process in publishers.items():
        try:
            _signal(publisher_process)
        except Exception as error:
            errors.append(f"signal publisher {name}: {error}")

    # Stage 2: release every endpoint/control connection.
    for name, client in clients.items():
        try:
            client.close()
        except Exception as error:
            errors.append(f"close client {name}: {error}")

    # Stage 3: reap every child even if another child failed to terminate.
    for name, text_process in text_processes.items():
        try:
            text_process.communicate(timeout=5.0)
        except Exception as error:
            errors.append(f"wait {name}: {error}")
    for name, publisher_process in publishers.items():
        try:
            _wait_process(publisher_process)
        except Exception as error:
            errors.append(f"wait publisher {name}: {error}")
    return errors


@contextmanager
def _cleanup_scope(cleanup: Callable[[], list[str]]) -> Iterator[None]:
    """Keep the business failure primary and attach any cleanup failures."""
    try:
        yield
    except BaseException as error:
        cleanup_errors = cleanup()
        if cleanup_errors:
            cleanup_error = RuntimeError("cleanup failed after business error: " + "; ".join(cleanup_errors))
            raise error from cleanup_error
        raise
    else:
        cleanup_errors = cleanup()
        if cleanup_errors:
            raise RuntimeError("cleanup failed after all resources: " + "; ".join(cleanup_errors))


@contextmanager
def _test_directory(repository: Path, prefix: str):
    root = repository / "build" / f"{prefix}-{os.getpid()}-{time.time_ns()}"
    root.mkdir()
    try:
        yield root
    finally:
        shutil.rmtree(root)


def _feed_records(
    clients: list[SensorWriter],
    event_streams: list[list[tuple[float, bytes]]],
    start_at: float,
    periods_s: list[float],
    *,
    clock: Callable[[], float] = time.perf_counter,
    waiter_factory: Callable[[threading.Event], AbstractContextManager[Callable[[float], bool]]] | None = None,
) -> FeedStats:
    """Feed one publisher component without bursting after a late deadline."""
    if len(clients) != len(event_streams) or len(clients) != len(periods_s):
        raise ValueError("sensor clients, event streams, and periods must align")
    if waiter_factory is None:
        from sim.runtime.windows_timing import deadline_waiter

        waiter_factory = deadline_waiter
    grouped: dict[tuple[float, int], list[bytes]] = {}
    for offset_s, stream_index, payload in (
        (offset_s, stream_index, payload)
        for stream_index, stream in enumerate(event_streams)
        for offset_s, payload in stream
    ):
        grouped.setdefault((offset_s, stream_index), []).append(payload)
    sent = 0
    warmup_sent = 0
    warmup_dropped = 0
    dropped = 0
    warmup_max_late_s = 0.0
    max_late_s = 0.0
    stop_event = threading.Event()
    with waiter_factory(stop_event) as wait:
        events = sorted(
            grouped.items(),
            key=lambda item: (item[0][0], periods_s[item[0][1]]),
        )
        for (offset_s, stream_index), payloads in events:
            target = start_at + offset_s
            now = clock()
            if now >= target + periods_s[stream_index]:
                kept = [payload for payload in payloads if _is_camera_info(payload)]
                dropped_now = len(payloads) - len(kept)
                if offset_s < RATE_WARMUP_S:
                    warmup_dropped += dropped_now
                else:
                    dropped += dropped_now
                payloads = kept
            else:
                wait(max(0.0, target - now))
                now = clock()
            late_s = max(0.0, now - target)
            if offset_s < RATE_WARMUP_S:
                warmup_max_late_s = max(warmup_max_late_s, late_s)
            else:
                max_late_s = max(max_late_s, late_s)
            client = clients[stream_index]
            for payload in payloads:
                if client.write(payload) != len(payload):
                    raise RuntimeError("sensor endpoint reported a short write")
                sent += 1
                if offset_s < RATE_WARMUP_S:
                    warmup_sent += 1
    return FeedStats(sent, warmup_sent, warmup_dropped, dropped, warmup_max_late_s, max_late_s)


def _run_feeder_session(
    stream: str,
    client: SensorWriter,
    records: list[tuple[float, bytes]],
    period_s: float,
    commands: TextIO,
    reports: TextIO,
    *,
    clock: Callable[[], float] = time.perf_counter,
    waiter_factory: Callable[[threading.Event], AbstractContextManager[Callable[[float], bool]]] | None = None,
) -> FeederReport:
    """Run the small READY -> shared start -> JSON feeder protocol."""
    reports.write("READY\n")
    reports.flush()
    command_line = commands.readline()
    if not command_line:
        raise RuntimeError("feeder received no start command")
    command = json.loads(command_line)
    if set(command) != {"start_perf_counter_ns"} or not isinstance(command["start_perf_counter_ns"], int):
        raise RuntimeError(f"invalid feeder start command: {command!r}")
    start_ns = command["start_perf_counter_ns"]
    if start_ns < 0:
        raise RuntimeError(f"invalid feeder start time: {start_ns}")
    stats = _feed_records(
        [client],
        [records],
        start_ns / 1_000_000_000.0,
        [period_s],
        clock=clock,
        waiter_factory=waiter_factory,
    )
    result: FeederReport = {
        "stream": stream,
        "sent": stats.sent,
        "warmup_sent": stats.warmup_sent,
        "warmup_dropped": stats.warmup_dropped,
        "dropped": stats.dropped,
        "warmup_max_late_s": stats.warmup_max_late_s,
        "max_late_s": stats.max_late_s,
    }
    reports.write(json.dumps(result, sort_keys=True) + "\n")
    reports.flush()
    return result


def _feed_camera_payload(client: SensorWriter, base_ns: int, seconds: float, start_at: float) -> tuple[int, int]:
    """Send production-sized camera pairs without retaining the observation window."""
    from sim.runtime.windows_timing import deadline_waiter

    sent = 0
    dropped = 0
    stop_event = threading.Event()
    with deadline_waiter(stop_event) as wait:
        for offset_s, group in groupby(_full_camera_records(base_ns, seconds), key=lambda item: item[0]):
            payloads = [payload for _, payload in group]
            target = start_at + offset_s
            now = time.perf_counter()
            if now >= target + 1.0 / 30.0:
                kept = [payload for payload in payloads if _is_camera_info(payload)]
                dropped += len(payloads) - len(kept)
                payloads = kept
            else:
                wait(max(0.0, target - now))
            for payload in payloads:
                if client.write(payload) != len(payload):
                    raise RuntimeError("camera endpoint reported a short write")
                sent += 1
    return sent, dropped


def _is_camera_info(record: bytes) -> bool:
    return len(record) >= 36 and record[28:32] == b"LTOB" and struct.unpack_from("<H", record, 34)[0] == 1


def test_warmup_drops_are_diagnostic_only_and_camera_info_survives() -> None:
    """Warmup loss stays visible without failing the formal-window gate."""

    class Client:
        def __init__(self) -> None:
            self.records: list[bytes] = []

        def write(self, payload: bytes) -> int:
            self.records.append(payload)
            return len(payload)

    @contextmanager
    def waiter(_: threading.Event):
        yield lambda timeout_s: False

    client = Client()
    records = _camera_records(2_000_000_000, 0.0)
    records.append((RATE_WARMUP_S, b"formal-window-sample"))
    stats = _feed_records(
        [client],
        [records],
        0.0,
        [1.0 / 30.0],
        clock=lambda: 0.04,
        waiter_factory=waiter,
    )

    assert stats == FeedStats(2, 1, 2, 0, 0.04, 0.0)  # noqa: S101
    _require_feeder_delivery(
        "camera",
        sent=stats.sent - stats.warmup_sent,
        dropped=stats.dropped,
        qualification=False,
    )
    assert len(client.records) == 2  # noqa: S101
    assert _is_camera_info(client.records[0])  # noqa: S101


def test_full_camera_records_use_production_shape_without_prebuilding_window() -> None:
    records = _full_camera_records(2_000_000_000, 0.0)

    assert isinstance(records, Iterator)  # noqa: S101
    info_offset, info = next(records)
    color_offset, color = next(records)
    depth_offset, depth = next(records)

    assert (info_offset, color_offset, depth_offset) == (0.0, 0.0, 0.0)  # noqa: S101
    camera_header_size = struct.calcsize("<4sHHIIIIddddddIddddd")
    assert len(color) == 28 + camera_header_size + 640 * 480 * 3  # noqa: S101
    assert len(depth) == 28 + camera_header_size + 640 * 480 * 2  # noqa: S101
    assert struct.unpack_from("<II", color, 36) == (640, 480)  # noqa: S101
    assert struct.unpack_from("<II", depth, 36) == (640, 480)  # noqa: S101


def test_feeder_protocol_waits_for_one_shared_start_and_reports_json() -> None:
    class Client:
        def write(self, payload: bytes) -> int:
            return len(payload)

    @contextmanager
    def waiter(_: threading.Event):
        yield lambda timeout_s: False

    commands = StringIO('{"start_perf_counter_ns": 2000000000}\n')
    reports = StringIO()
    result = _run_feeder_session(
        "imu",
        Client(),
        [(0.0, b"sample")],
        0.005,
        commands,
        reports,
        clock=lambda: 2.0,
        waiter_factory=waiter,
    )
    lines = reports.getvalue().splitlines()
    assert lines[0] == "READY"  # noqa: S101
    assert (
        json.loads(lines[1])
        == result
        == {  # noqa: S101
            "stream": "imu",
            "sent": 1,
            "warmup_sent": 1,
            "warmup_dropped": 0,
            "dropped": 0,
            "warmup_max_late_s": 0.0,
            "max_late_s": 0.0,
        }
    )


def test_feeders_use_normal_process_priority() -> None:
    with patch.object(subprocess, "Popen") as popen:
        for stream in ("lidar", "imu", "camera"):
            _start_feeder(Path("repo"), Path("ready"), "role", "f" * 32, stream, 1.0, 1)

    assert popen.call_count == 3  # noqa: S101
    for call in popen.call_args_list:
        assert "creationflags" not in call.kwargs  # noqa: S101


def test_business_error_stays_primary_when_cleanup_also_fails() -> None:
    try:
        with _cleanup_scope(lambda: ["wait probe: simulated cleanup failure"]):
            raise ValueError("simulated business failure")
    except ValueError as error:
        assert str(error) == "simulated business failure"  # noqa: S101
        assert isinstance(error.__cause__, RuntimeError)  # noqa: S101
        assert str(error.__cause__) == (  # noqa: S101
            "cleanup failed after business error: wait probe: simulated cleanup failure"
        )
    else:
        raise AssertionError("business failure was not preserved")


def test_cleanup_attempts_every_resource_in_three_stages() -> None:
    events: list[str] = []

    class FakeProcess:
        def __init__(self, name: str, *, fail_kill: bool = False) -> None:
            self.name = name
            self.fail_kill = fail_kill

        def poll(self) -> None:
            return None

        def kill(self) -> None:
            events.append(f"terminate:{self.name}")
            if self.fail_kill:
                raise RuntimeError("kill failed")

        def send_signal(self, _: int) -> None:
            events.append(f"signal:{self.name}")

        def communicate(self, timeout: float) -> tuple[bytes, bytes]:
            events.append(f"wait:{self.name}")
            return b"", b""

    class FakeClient:
        def __init__(self, name: str, *, fail_close: bool = False) -> None:
            self.name = name
            self.fail_close = fail_close

        def write(self, payload: bytes) -> int:
            return len(payload)

        def close(self) -> None:
            events.append(f"close:{self.name}")
            if self.fail_close:
                raise RuntimeError("close failed")

    errors = _cleanup_processes(
        probe=cast(subprocess.Popen[str], FakeProcess("probe", fail_kill=True)),
        feeders={"imu": cast(subprocess.Popen[str], FakeProcess("feeder"))},
        publishers={"camera": cast(subprocess.Popen[bytes], FakeProcess("publisher"))},
        clients={
            "bad": FakeClient("bad", fail_close=True),
            "good": FakeClient("good"),
        },
    )

    assert events == [  # noqa: S101
        "terminate:probe",
        "terminate:feeder",
        "signal:publisher",
        "close:bad",
        "close:good",
        "wait:probe",
        "wait:feeder",
        "wait:publisher",
    ]
    assert len(errors) == 2  # noqa: S101
    assert errors[0].startswith("terminate probe:")  # noqa: S101
    assert errors[1].startswith("close client bad:")  # noqa: S101


def test_default_records_maximum_intervals_but_qualification_gates_them() -> None:
    metric = _healthy_metric("rt/imu/raw", 200.0)
    metric["source_max_interval_s"] = 1.0
    metric["dds_write_max_interval_s"] = 1.0
    metric["take_max_interval_s"] = 1.0

    _require_metric(metric, 200.0, qualification=False)
    try:
        _require_metric(metric, 200.0, qualification=True)
    except RuntimeError:
        pass
    else:
        raise AssertionError("qualification accepted excessive maximum intervals")


def test_camera_default_accepts_0_9_period_p95_but_qualification_rejects_it() -> None:
    metric = _healthy_metric("rt/camera/color", 30.0)
    metric["dds_write_p95_jitter_s"] = 0.9 / 30.0

    _require_metric(metric, 30.0, qualification=False)
    try:
        _require_metric(metric, 30.0, qualification=True)
    except RuntimeError:
        return
    raise AssertionError("camera qualification accepted 0.9-period DDS write p95 jitter")


def test_camera_default_rejects_p95_above_one_period() -> None:
    metric = _healthy_metric("rt/camera/depth", 30.0)
    metric["dds_write_p95_jitter_s"] = 1.01 / 30.0

    try:
        _require_metric(metric, 30.0, qualification=False)
    except RuntimeError:
        return
    raise AssertionError("camera default accepted DDS write p95 jitter above one period")


def test_formal_window_drop_gate_rejects_389_of_2201_records() -> None:
    try:
        _require_feeder_delivery("imu", sent=1812, dropped=389, qualification=False)
    except RuntimeError:
        return
    raise AssertionError("default gate accepted 389 dropped records out of 2201")


def test_production_reader_loss_is_always_rejected() -> None:
    metric = _healthy_metric("rt/imu/raw", 200.0)
    metric["production_dds_sample_lost"] = 1

    try:
        _require_metric(metric, 200.0, qualification=False)
    except RuntimeError:
        return
    raise AssertionError("default gate accepted a production-reader DDS loss")


def test_production_reader_must_cover_the_full_window() -> None:
    metric = _healthy_metric("rt/imu/raw", 200.0)
    metric["production_samples"] = 1900
    metric["production_source_inferred_drop_rate"] = 0.0
    metric["production_window_delivery_ratio"] = 0.95

    try:
        _require_metric(metric, 200.0, qualification=False)
    except RuntimeError:
        return
    raise AssertionError("default gate accepted only 95% production-reader window coverage")


def _healthy_metric(topic: str, expected_hz: float) -> TopicMetric:
    period_s = 1.0 / expected_hz
    return {
        "topic": topic,
        "samples": int(expected_hz * 10.0),
        "source_hz": expected_hz,
        "source_max_interval_s": period_s,
        "source_p95_jitter_s": 0.0,
        "source_inferred_drop_rate": 0.0,
        "dds_write_actual_hz": expected_hz,
        "dds_write_max_interval_s": period_s,
        "dds_write_p95_jitter_s": 0.0,
        "take_max_interval_s": period_s,
        "take_p95_jitter_s": 0.0,
        "first_sample_delay_s": 0.01,
        "tail_silence_s": 0.01,
        "window_delivery_ratio": 1.0,
        "dds_sample_lost": 0,
        "production_samples": int(expected_hz * 10.0),
        "production_source_inferred_drop_rate": 0.0,
        "production_window_delivery_ratio": 1.0,
        "production_dds_sample_lost": 0,
        "production_sample_rejected": 0,
    }


def _require_feeder_delivery(stream: str, *, sent: int, dropped: int, qualification: bool) -> None:
    total = sent + dropped
    if total <= 0:
        raise RuntimeError(f"{stream} feeder reported no scheduled records")
    drop_rate = dropped / total
    if (qualification and dropped != 0) or drop_rate > 0.01:
        raise RuntimeError(f"{stream} scheduler dropped {dropped}/{total} records")


def _feeder_main() -> int:
    if len(sys.argv) != 9:
        raise RuntimeError(
            "usage: test_sensor_topic_rate_probe.py --feeder "
            "REPOSITORY READINESS ROLE FINGERPRINT STREAM DURATION BASE_NS"
        )
    repository = Path(sys.argv[2]).resolve(strict=True)
    readiness = Path(sys.argv[3]).resolve(strict=True)
    role = sys.argv[4]
    product_session_id = sys.argv[5]
    stream = sys.argv[6]
    duration_s = float(sys.argv[7])
    base_ns = int(sys.argv[8])
    fixtures: dict[str, tuple[Callable[[int, float], list[tuple[float, bytes]]], float]] = {
        "lidar": (_lidar_records, 0.1),
        "imu": (_imu_records, 0.005),
        "camera": (_camera_records, 1.0 / 30.0),
    }
    if stream not in fixtures:
        raise RuntimeError(f"unsupported feeder stream: {stream}")
    sys.path[:0] = [str(repository / "src"), str(repository)]
    from sim.scripts.mujoco.native_runtime_endpoint import SensorPublisherClient

    fixture, period_s = fixtures[stream]
    records = fixture(base_ns, duration_s)
    client = SensorPublisherClient.connect(
        readiness,
        role=role,
        product_session_id=product_session_id,
        timeout_s=20.0,
    )
    try:
        _run_feeder_session(stream, client, records, period_s, sys.stdin, sys.stdout)
    finally:
        client.close()
    return 0


def _require_metric(
    metric: TopicMetric,
    expected_hz: float,
    *,
    qualification: bool,
    minimum_duration_s: float = 9.5,
) -> None:
    topic = str(metric["topic"])
    if int(metric["samples"]) < int(expected_hz * minimum_duration_s):
        raise RuntimeError(f"{topic} did not sustain its source cadence for the observation window")
    source_hz = float(metric["source_hz"])
    source_rate_tolerance = 0.005 if qualification else 0.015
    if abs(source_hz - expected_hz) / expected_hz > source_rate_tolerance:
        raise RuntimeError(f"{topic} source rate {source_hz:.6f} differs from {expected_hz} Hz")
    expected_period = 1.0 / expected_hz
    if qualification:
        max_source_interval = 0.05 if expected_hz == 200.0 else expected_period * 2.0
        if float(metric["source_max_interval_s"]) > max_source_interval:
            raise RuntimeError(f"{topic} source maximum interval exceeded: {metric}")
    if float(metric["source_p95_jitter_s"]) > 2e-9:
        raise RuntimeError(f"{topic} source p95 jitter exceeded quantization tolerance: {metric}")
    source_drop_rate = float(metric["source_inferred_drop_rate"])
    source_drop_limit = 0.0 if qualification else 0.01
    if source_drop_rate > source_drop_limit:
        raise RuntimeError(f"{topic} source inferred a dropped sample: {metric}")
    dds_write_hz = float(metric["dds_write_actual_hz"])
    if abs(dds_write_hz - expected_hz) / expected_hz > 0.05:
        raise RuntimeError(f"{topic} DDS write rate {dds_write_hz:.6f} is outside 5%")
    if expected_hz == 200.0:
        max_periods, p95_periods = (10.0 if qualification else 20.0), 1.0
    elif expected_hz == 10.0:
        max_periods, p95_periods = 2.0, 0.5
    else:
        max_periods = 2.0 if qualification else 3.0
        p95_periods = 0.75 if qualification else 1.0
    if qualification and float(metric["dds_write_max_interval_s"]) > expected_period * max_periods:
        raise RuntimeError(f"{topic} DDS write maximum interval exceeded: {metric}")
    if float(metric["dds_write_p95_jitter_s"]) > expected_period * p95_periods:
        raise RuntimeError(f"{topic} DDS write p95 jitter exceeded: {metric}")
    # Local take timing includes observer scheduling and batching. It is kept
    # visible as a transport-health signal, but is not called DDS write timing.
    if qualification and float(metric["take_max_interval_s"]) > 0.75:
        raise RuntimeError(f"{topic} local take maximum interval exceeded: {metric}")
    if float(metric["take_p95_jitter_s"]) > 0.20:
        raise RuntimeError(f"{topic} local take p95 jitter exceeded: {metric}")
    if float(metric["first_sample_delay_s"]) > 0.25:
        raise RuntimeError(f"{topic} first sample was late: {metric}")
    tail_periods = 10.0 if expected_hz == 200.0 else 2.0
    if float(metric["tail_silence_s"]) > expected_period * tail_periods:
        raise RuntimeError(f"{topic} stopped before the observation window ended: {metric}")
    delivery_limit = 0.98
    if float(metric["window_delivery_ratio"]) < delivery_limit:
        raise RuntimeError(f"{topic} delivered too little of its full window: {metric}")
    if int(metric["dds_sample_lost"]) != 0:
        raise RuntimeError(f"{topic} DDS reported lost samples: {metric}")
    if metric["production_samples"] < int(expected_hz * minimum_duration_s):
        raise RuntimeError(f"{topic} production reader did not cover the observation window")
    if metric["production_source_inferred_drop_rate"] > source_drop_limit:
        raise RuntimeError(f"{topic} production reader inferred a dropped sample: {metric}")
    if metric["production_window_delivery_ratio"] < 0.98:
        raise RuntimeError(f"{topic} production reader delivered too little of its full window: {metric}")
    if metric["production_dds_sample_lost"] != 0:
        raise RuntimeError(f"{topic} production reader DDS reported lost samples: {metric}")
    if metric["production_sample_rejected"] != 0:
        raise RuntimeError(f"{topic} production reader DDS rejected samples: {metric}")


def _verify_window_gate_rejects_silence() -> None:
    stale: TopicMetric = {
        "topic": "rt/imu/raw",
        "samples": 1800,
        "source_hz": 200.0,
        "source_max_interval_s": 0.005,
        "source_p95_jitter_s": 0.0,
        "source_inferred_drop_rate": 0.0,
        "dds_write_actual_hz": 200.0,
        "dds_write_max_interval_s": 0.005,
        "dds_write_p95_jitter_s": 0.0,
        "take_max_interval_s": 0.005,
        "take_p95_jitter_s": 0.0,
        "first_sample_delay_s": 0.5,
        "tail_silence_s": 0.5,
        "window_delivery_ratio": 0.9,
        "dds_sample_lost": 0,
        "production_samples": 1800,
        "production_source_inferred_drop_rate": 0.0,
        "production_window_delivery_ratio": 0.9,
        "production_dds_sample_lost": 0,
        "production_sample_rejected": 0,
    }
    try:
        _require_metric(stale, 200.0, qualification=False)
    except RuntimeError:
        return
    raise RuntimeError("window gate accepted 0.5 s head and tail silence")


def _camera_payload_main(publisher: Path, probe_executable: Path, repository: Path, domain_id: int) -> int:
    """Validate the camera publisher with production-sized RGB-D messages."""
    from sim.scripts.mujoco.native_runtime_endpoint import SensorPublisherClient

    product_session_id = "8" * 32
    process: subprocess.Popen[bytes] | None = None
    client: SensorClient | None = None
    probe: subprocess.Popen[str] | None = None
    with _test_directory(repository, "lingtu-camera-payload-rate") as root:
        clients: dict[str, SensorClient] = {}

        def cleanup() -> list[str]:
            return _cleanup_processes(
                probe=probe,
                feeders={},
                publishers={"camera": process} if process is not None else {},
                clients=clients,
            )

        with _cleanup_scope(cleanup):
            process = _start_publisher(publisher, root, product_session_id, domain_id, "camera")
            readiness = root / "camera.ready.json"
            _wait_ready(readiness, process)
            client = SensorPublisherClient.connect(
                readiness,
                role="camera_publisher",
                product_session_id=product_session_id,
                timeout_s=20.0,
            )
            clients["camera"] = client
            probe = subprocess.Popen(  # noqa: S603
                [str(probe_executable), str(domain_id), str(CAMERA_PAYLOAD_WINDOW_S), "camera"],
                cwd=str(root),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            if probe.stdout is None:
                raise RuntimeError("camera probe stdout pipe was not created")
            if probe.stdout.readline().strip() != "READY":
                stdout, stderr = probe.communicate(timeout=2.0)
                raise RuntimeError(f"camera probe did not become ready: {stdout!r} {stderr!r}")
            with ThreadPoolExecutor(max_workers=1) as executor:
                feed = executor.submit(
                    _feed_camera_payload,
                    client,
                    2_000_000_000,
                    RATE_WARMUP_S + CAMERA_PAYLOAD_WINDOW_S + CAMERA_PAYLOAD_TAIL_MARGIN_S,
                    time.perf_counter() + 0.02,
                )
                stdout, stderr = probe.communicate(timeout=10.0)
                sent, dropped = feed.result(timeout=4.0)
            if probe.returncode != 0:
                raise RuntimeError(f"camera probe failed: stdout={stdout!r} stderr={stderr!r}")
            print(stdout, end="")
            if dropped / (sent + dropped) > 0.01:
                raise RuntimeError(f"camera scheduler dropped {dropped}/{sent + dropped} records")
            metrics: dict[str, object] = {
                item["topic"]: item for line in stdout.splitlines() if line.strip() for item in [json.loads(line)]
            }
            for topic in ("rt/camera/color", "rt/camera/depth"):
                _require_metric(
                    cast(TopicMetric, metrics[topic]),
                    30.0,
                    qualification=False,
                    minimum_duration_s=3.8,
                )
            camera_info = cast(CameraInfoMetric, metrics["rt/camera/info"])
            if camera_info["late_joiner_received"] is not True:
                raise RuntimeError("CameraInfo transient-local late joiner received no sample")
    print("camera publisher component full-payload rate: PASS")
    return 0


def main() -> int:
    """Validate sustained rates of the three split sensor publisher components."""
    if len(sys.argv) > 1 and sys.argv[1] == "--feeder":
        return _feeder_main()
    _verify_window_gate_rejects_silence()
    qualification = os.environ.get("LINGTU_SENSOR_RATE_QUALIFICATION") == "1"
    if len(sys.argv) not in {4, 5}:
        raise RuntimeError("usage: test_sensor_topic_rate_probe.py PUBLISHER PROBE REPOSITORY [camera-payload]")
    publisher = Path(sys.argv[1]).resolve(strict=True)
    probe_executable = Path(sys.argv[2]).resolve(strict=True)
    repository = Path(sys.argv[3]).resolve(strict=True)
    sys.path[:0] = [str(repository / "src"), str(repository)]

    domain_id = _domain_id_from_environment()
    if len(sys.argv) == 5:
        if sys.argv[4] != "camera-payload":
            raise RuntimeError(f"unsupported sensor rate mode: {sys.argv[4]}")
        return _camera_payload_main(publisher, probe_executable, repository, domain_id)
    product_session_id = "9" * 32
    streams = {
        "lidar": ("lidar_publisher", "lidar.ready.json"),
        "imu": ("imu_publisher", "imu.ready.json"),
        "camera": ("camera_publisher", "camera.ready.json"),
    }
    processes: dict[str, subprocess.Popen[bytes]] = {}
    feeders: dict[str, subprocess.Popen[str]] = {}
    probe: subprocess.Popen[str] | None = None
    with _test_directory(repository, "lingtu-sensor-rate") as root:

        def cleanup() -> list[str]:
            return _cleanup_processes(
                probe=probe,
                feeders=feeders,
                publishers=processes,
                clients={},
            )

        with _cleanup_scope(cleanup):
            for stream in streams:
                session = root / stream
                session.mkdir()
                processes[stream] = _start_publisher(publisher, session, product_session_id, domain_id, stream)
            duration_s = 11.0
            base_ns = 2_000_000_000
            for stream, (role, readiness_name) in streams.items():
                readiness = root / stream / readiness_name
                _wait_ready(readiness, processes[stream])
                feeders[stream] = _start_feeder(
                    repository,
                    readiness,
                    role,
                    product_session_id,
                    stream,
                    duration_s,
                    base_ns,
                )
            for stream, feeder in feeders.items():
                _wait_feeder_ready(stream, feeder)

            probe = subprocess.Popen(  # noqa: S603
                [str(probe_executable), str(domain_id), "10"],
                cwd=str(root),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            if probe.stdout is None:
                raise RuntimeError("probe stdout pipe was not created")
            if probe.stdout.readline().strip() != "READY":
                stdout, stderr = probe.communicate(timeout=2.0)
                raise RuntimeError(f"probe did not become ready: {stdout!r} {stderr!r}")
            start_command = json.dumps({"start_perf_counter_ns": time.perf_counter_ns() + 100_000_000}) + "\n"
            for stream, feeder in feeders.items():
                if feeder.stdin is None:
                    raise RuntimeError(f"{stream} feeder stdin pipe was not created")
                feeder.stdin.write(start_command)
                feeder.stdin.flush()
                feeder.stdin.close()
                feeder.stdin = None
            stdout, stderr = probe.communicate(timeout=18.0)
            if probe.returncode != 0:
                raise RuntimeError(f"probe failed: stdout={stdout!r} stderr={stderr!r}")
            print(stdout, end="")
            feeder_reports: dict[str, FeederReport] = {}
            for stream, feeder in feeders.items():
                feeder_stdout, feeder_stderr = feeder.communicate(timeout=4.0)
                if feeder.returncode != 0:
                    raise RuntimeError(f"{stream} feeder failed: stdout={feeder_stdout!r} stderr={feeder_stderr!r}")
                report_lines = [line for line in feeder_stdout.splitlines() if line.strip()]
                if len(report_lines) != 1:
                    raise RuntimeError(f"{stream} feeder returned invalid report: {feeder_stdout!r}")
                report = cast(FeederReport, json.loads(report_lines[0]))
                if report.get("stream") != stream:
                    raise RuntimeError(f"{stream} feeder returned mismatched report: {report!r}")
                feeder_reports[stream] = report
            print(f"sensor publisher component feeder reports: {feeder_reports}")
            dropped_by_stream: dict[str, int] = {}
            warmup_dropped_by_stream: dict[str, int] = {}
            expected_records_by_stream = {
                "lidar": int(duration_s * 10) + 1,
                "imu": int(duration_s * 200) + 1,
                "camera": 1 + 2 * (int(duration_s * 30) + 1),
            }
            for name, report in feeder_reports.items():
                sent = int(report["sent"])
                warmup_sent = int(report["warmup_sent"])
                warmup_dropped = int(report["warmup_dropped"])
                dropped = int(report["dropped"])
                expected_records = expected_records_by_stream[name]
                if sent + warmup_dropped + dropped != expected_records:
                    raise RuntimeError(f"{name} feeder lost record accounting")
                _require_feeder_delivery(
                    name,
                    sent=sent - warmup_sent,
                    dropped=dropped,
                    qualification=qualification,
                )
                warmup_dropped_by_stream[name] = warmup_dropped
                dropped_by_stream[name] = dropped
            metrics: dict[str, object] = {
                item["topic"]: item for line in stdout.splitlines() if line.strip() for item in [json.loads(line)]
            }
            expected = {
                "rt/imu/raw": 200.0,
                "rt/lidar/raw_frame": 10.0,
                "rt/camera/color": 30.0,
                "rt/camera/depth": 30.0,
            }
            for topic, expected_hz in expected.items():
                _require_metric(cast(TopicMetric, metrics[topic]), expected_hz, qualification=qualification)
            raw_packet = cast(RawPacketMetric, metrics["rt/lidar/raw_packet"])
            if raw_packet["writer_count"] != 0:
                raise RuntimeError("formal lidar stream must not create a raw_packet writer")
            camera_info = cast(CameraInfoMetric, metrics["rt/camera/info"])
            if camera_info["late_joiner_received"] is not True:
                raise RuntimeError("CameraInfo transient-local late joiner received no sample")
            print(f"sensor rate qualification: {str(qualification).lower()}")
            print(f"sensor publisher component warmup drops: {warmup_dropped_by_stream}")
            print(f"sensor publisher component drops: {dropped_by_stream}")
            print(
                "sensor publisher component feeder warmup max lateness: "
                + str({name: report["warmup_max_late_s"] for name, report in feeder_reports.items()})
            )
            print(
                "sensor publisher component feeder max lateness: "
                + str({name: report["max_late_s"] for name, report in feeder_reports.items()})
            )
    print("sensor publisher component rate: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"sensor publisher component rate: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from None
