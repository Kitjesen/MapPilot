from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from sim.scripts.mujoco import native_dds_sensors as bridge

pytestmark = [pytest.mark.sim]


def test_local_publisher_starts_without_wsl_pid_handshake(monkeypatch, tmp_path):
    publisher = tmp_path / "lingtu_mujoco_sensor_publisher.exe"
    publisher.write_bytes(b"placeholder")
    args = bridge._build_parser().parse_args(
        ["--publisher-bin", str(publisher), "--json-out", str(tmp_path / "report.json")]
    )

    class Process:
        pid = 4242
        stdin = object()
        stdout = None

    launched = []

    def popen(command, **kwargs):
        launched.append((command, kwargs))
        return Process()

    ready_waits = []
    monkeypatch.setattr(bridge.subprocess, "Popen", popen)
    monkeypatch.setattr(
        bridge,
        "_wait_for_publisher_ready",
        lambda process, ready_file, *, timeout_s: ready_waits.append(
            (process, ready_file, timeout_s)
        ),
        raising=False,
    )
    monkeypatch.setattr(
        bridge,
        "_read_linux_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "a local publisher must not wait for a WSL PID file"
        ),
    )

    process = bridge._start_native_publisher(args)

    assert launched[0][0][0] == str(publisher.resolve())
    assert "wsl" not in launched[0][0][0].lower()
    assert process._lingtu_publisher_kind == "local"
    assert process._lingtu_owned_pid == 4242
    assert process._lingtu_linux_pid is None
    assert process._lingtu_linux_pid_file is None
    assert "--ready-file" in launched[0][0]
    ready_index = launched[0][0].index("--ready-file") + 1
    assert Path(launched[0][0][ready_index]) == process._lingtu_ready_file
    assert ready_waits == [(process, process._lingtu_ready_file, 10.0)]


def test_windows_default_native_dds_tools_ignore_retired_root_d_build(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(bridge.os, "name", "nt")
    monkeypatch.setattr(bridge, "ROOT", tmp_path)
    canonical = (
        tmp_path
        / "build/windows-native-dds-adapter/Release/lingtu_mujoco_sensor_publisher.exe"
    )
    retired = (
        tmp_path
        / "build/windows-native-dds-adapter-root-d/Release/lingtu_mujoco_sensor_publisher.exe"
    )
    canonical.parent.mkdir(parents=True)
    retired.parent.mkdir(parents=True)
    canonical.write_bytes(b"canonical")
    retired.write_bytes(b"retired")

    publisher_candidates = bridge._publisher_candidates("")
    resolved = bridge._resolve_publisher_bin("")

    assert publisher_candidates == [canonical]
    assert resolved == canonical.resolve()
    assert all("livox_sdk2_stream" not in str(path) for path in publisher_candidates)


def test_driver_bridge_requires_verified_explicit_artifact():
    with pytest.raises(FileNotFoundError, match="--driver-bridge-bin is required"):
        bridge._resolve_driver_bridge_bin("")


def test_sensor_cli_exposes_only_physical_driver_bridge_controls():
    parser = bridge._build_parser()
    args = parser.parse_args(
        [
            "--command-source",
            "dds",
            "--driver-bridge-bin",
            "bridge.exe",
            "--driver-expected-host-boot-id",
            "host-a",
            "--driver-max-linear-mps",
            "1.0",
            "--driver-max-angular-rps",
            "1.0",
        ]
    )

    assert args.driver_bridge_bin == "bridge.exe"
    assert args.driver_expected_host_boot_id == "host-a"
    assert args.driver_command_timeout_ms == 200
    assert args.driver_heartbeat_timeout_ms == 500
    assert args.driver_apply_timeout_ms == 500
    with pytest.raises(SystemExit):
        parser.parse_args(["--cmd-vel-tap-bin", "legacy-tap"])
    with pytest.raises(SystemExit):
        parser.parse_args(["--cmd-vel-timeout-s", "0.25"])


def test_publisher_ready_wait_fails_fast_when_child_exits(tmp_path):
    class Process:
        @staticmethod
        def poll():
            return 17

    with pytest.raises(RuntimeError, match=r"exited before DDS readiness.*17"):
        bridge._wait_for_publisher_ready(
            Process(),
            tmp_path / "missing.ready.json",
            timeout_s=0.1,
        )


@pytest.mark.parametrize("domain_id", ["-1", "233"])
def test_bridge_rejects_domain_outside_supported_udp_range(domain_id):
    with pytest.raises(SystemExit):
        bridge._build_parser().parse_args(["--domain-id", domain_id])


def test_local_publisher_finish_uses_owned_process_without_wsl_cleanup(monkeypatch):
    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "local"
        _lingtu_owned_pid = 4242
        _lingtu_linux_pid = None
        _lingtu_linux_pid_file = None
        pid = 4242

        def __init__(self):
            self.stdin = Stream()
            self.terminate_calls = 0
            self.kill_calls = 0

        def wait(self, timeout):
            assert timeout == pytest.approx(3.0)
            return 0

        def poll(self):
            return 0

        def terminate(self):
            self.terminate_calls += 1

        def kill(self):
            self.kill_calls += 1

    process = Process()
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "local publisher cleanup must not use WSL process control"
        ),
    )

    cleanup = bridge._finish_native_publisher(process)

    assert process.stdin.closed is True
    assert process.terminate_calls == 0
    assert process.kill_calls == 0
    assert cleanup["process_kind"] == "local"
    assert cleanup["pid"] == 4242
    assert cleanup["owned_pid"] is True
    assert cleanup["alive_after_cleanup"] is False
    assert cleanup["clean"] is True
    assert cleanup["errors"] == []
    assert cleanup["returncode_before_cleanup"] == 0
    assert cleanup["returncode"] == 0
    assert cleanup["exit_context"] == "natural_exit"
    assert cleanup["failed_before_cleanup"] is False
    assert cleanup["terminated_by_parent"] is False
    assert bridge._native_sensor_publisher_gaps(cleanup) == []


def test_local_publisher_natural_nonzero_exit_is_reported_as_failure(monkeypatch):
    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "local"
        _lingtu_owned_pid = 4343
        _lingtu_linux_pid = None
        _lingtu_linux_pid_file = None
        pid = 4343

        def __init__(self):
            self.stdin = Stream()
            self.terminate_calls = 0
            self.kill_calls = 0

        def wait(self, timeout):
            assert timeout == pytest.approx(3.0)
            return 17

        def poll(self):
            return 17

        def terminate(self):
            self.terminate_calls += 1

        def kill(self):
            self.kill_calls += 1

    process = Process()
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "local publisher cleanup must not use WSL process control"
        ),
    )

    cleanup = bridge._finish_native_publisher(process)

    assert process.stdin.closed is True
    assert process.terminate_calls == 0
    assert process.kill_calls == 0
    assert cleanup["returncode_before_cleanup"] == 17
    assert cleanup["returncode"] == 17
    assert cleanup["exit_context"] == "natural_exit"
    assert cleanup["failed_before_cleanup"] is True
    assert cleanup["terminated_by_parent"] is False
    assert cleanup["clean"] is False
    assert cleanup["errors"] == ["publisher_exited_nonzero:returncode=17"]


def test_wsl_publisher_natural_nonzero_exit_is_reported_as_failure(monkeypatch):
    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "wsl"
        _lingtu_owned_pid = 4545
        _lingtu_linux_pid = 4545
        _lingtu_linux_pid_file = None
        pid = 8080

        def __init__(self):
            self.stdin = Stream()

        def wait(self, timeout):
            assert timeout == pytest.approx(3.0)
            return 23

        def poll(self):
            return 23

    process = Process()
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda pid: {
            "linux_pid": pid,
            "owned_pid": True,
            "alive_before_cleanup": False,
            "term_sent": False,
            "kill_sent": False,
            "alive_after_cleanup": False,
            "clean": True,
            "errors": [],
        },
    )

    cleanup = bridge._finish_native_publisher(process)

    assert process.stdin.closed is True
    assert cleanup["process_kind"] == "wsl"
    assert cleanup["returncode_before_cleanup"] == 23
    assert cleanup["returncode"] == 23
    assert cleanup["exit_context"] == "natural_exit"
    assert cleanup["failed_before_cleanup"] is True
    assert cleanup["terminated_by_parent"] is False
    assert cleanup["clean"] is False
    assert cleanup["errors"] == ["publisher_exited_nonzero:returncode=23"]
    assert bridge._native_sensor_publisher_gaps(cleanup) == [
        "native_sensor_publisher_failed",
        "native_sensor_publisher_cleanup_failed",
    ]


def test_wsl_parent_terminated_publisher_is_not_reported_as_failure():
    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "wsl"
        _lingtu_owned_pid = 4646
        _lingtu_linux_pid = 4646
        _lingtu_linux_pid_file = None
        pid = 8181
        stdin = Stream()

        @staticmethod
        def wait(timeout):
            assert timeout == pytest.approx(3.0)
            return -15

        @staticmethod
        def poll():
            return -15

    cleanup = bridge._finish_native_publisher(
        Process(),
        termination_cleanup={
            "process_kind": "wsl",
            "pid": 4646,
            "linux_pid": 4646,
            "owned_pid": True,
            "alive_before_cleanup": True,
            "term_sent": True,
            "kill_sent": False,
            "alive_after_cleanup": False,
            "clean": True,
            "errors": [],
        },
    )

    assert cleanup["returncode"] == -15
    assert cleanup["exit_context"] == "parent_requested_termination"
    assert cleanup["failed_before_cleanup"] is False
    assert cleanup["terminated_by_parent"] is True
    assert cleanup["clean"] is True
    assert cleanup["errors"] == []
    assert bridge._native_sensor_publisher_gaps(cleanup) == []


def test_local_publisher_nonzero_exit_after_stdin_eof_is_reported_as_failure(
    monkeypatch,
):
    class Stream:
        closed = False

        def __init__(self, process):
            self.process = process

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "local"
        _lingtu_owned_pid = 4344
        _lingtu_linux_pid = None
        _lingtu_linux_pid_file = None
        pid = 4344

        def __init__(self):
            self.returncode = None
            self.stdin = Stream(self)
            self.terminate_calls = 0
            self.kill_calls = 0

        def wait(self, timeout):
            assert timeout == pytest.approx(3.0)
            self.returncode = 17
            return self.returncode

        def poll(self):
            return self.returncode

        def terminate(self):
            self.terminate_calls += 1

        def kill(self):
            self.kill_calls += 1

    process = Process()
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "local publisher cleanup must not use WSL process control"
        ),
    )

    cleanup = bridge._finish_native_publisher(process)

    assert process.stdin.closed is True
    assert process.terminate_calls == 0
    assert process.kill_calls == 0
    assert cleanup["returncode_before_cleanup"] is None
    assert cleanup["returncode"] == 17
    assert cleanup["exit_context"] == "stdin_eof_exit"
    assert cleanup["failed_before_cleanup"] is True
    assert cleanup["terminated_by_parent"] is False
    assert cleanup["clean"] is False
    assert cleanup["errors"] == ["publisher_exited_nonzero:returncode=17"]
    assert bridge._native_sensor_publisher_gaps(cleanup) == [
        "native_sensor_publisher_failed",
        "native_sensor_publisher_cleanup_failed",
    ]


def test_natural_publisher_failure_adds_explicit_report_gap():
    cleanup = {
        "returncode": 17,
        "failed_before_cleanup": True,
        "terminated_by_parent": False,
        "clean": False,
    }

    assert bridge._native_sensor_publisher_gaps(cleanup) == [
        "native_sensor_publisher_failed",
        "native_sensor_publisher_cleanup_failed",
    ]


def test_parent_terminated_publisher_is_not_reported_as_natural_failure(monkeypatch):
    class Stream:
        closed = False

        def close(self):
            self.closed = True

    class Process:
        _lingtu_publisher_kind = "local"
        _lingtu_owned_pid = 4444
        _lingtu_linux_pid = None
        _lingtu_linux_pid_file = None
        pid = 4444

        def __init__(self):
            self.stdin = Stream()
            self.returncode = None
            self.wait_calls = 0
            self.terminate_calls = 0
            self.kill_calls = 0

        def wait(self, timeout):
            self.wait_calls += 1
            if self.wait_calls == 1:
                assert timeout == pytest.approx(3.0)
                raise subprocess.TimeoutExpired("publisher", timeout)
            return self.returncode

        def poll(self):
            return self.returncode

        def terminate(self):
            self.terminate_calls += 1
            self.returncode = -15

        def kill(self):
            self.kill_calls += 1
            self.returncode = -9

    process = Process()
    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "local publisher cleanup must not use WSL process control"
        ),
    )

    cleanup = bridge._finish_native_publisher(process)

    assert process.stdin.closed is True
    assert process.terminate_calls == 1
    assert process.kill_calls == 0
    assert cleanup["returncode_before_cleanup"] is None
    assert cleanup["returncode"] == -15
    assert cleanup["exit_context"] == "parent_requested_termination"
    assert cleanup["failed_before_cleanup"] is False
    assert cleanup["terminated_by_parent"] is True
    assert cleanup["clean"] is True
    assert cleanup["errors"] == []
    assert bridge._native_sensor_publisher_gaps(cleanup) == []


def test_async_timeout_escalates_local_publisher_without_wsl_cleanup(monkeypatch):
    class Publisher:
        def __init__(self):
            self.join_calls = 0
            self.timed_out = False

        def request_stop(self, reason):
            assert reason == "shutdown"

        def join(self, *, timeout_s):
            assert timeout_s == pytest.approx(0.25)
            self.join_calls += 1
            return self.join_calls > 1

        def mark_shutdown_timeout(self):
            self.timed_out = True

        def stats(self):
            return {"timed_out": self.timed_out}

    class Process:
        _lingtu_publisher_kind = "local"
        _lingtu_owned_pid = 5151
        _lingtu_linux_pid = None
        pid = 5151

        def __init__(self):
            self.returncode = None
            self.wait_calls = 0

        def poll(self):
            return self.returncode

        def terminate(self):
            return None

        def kill(self):
            self.returncode = -9

        def wait(self, timeout):
            self.wait_calls += 1
            if self.wait_calls == 1:
                raise subprocess.TimeoutExpired("publisher", timeout)
            return self.returncode

    monkeypatch.setattr(
        bridge,
        "_terminate_wsl_pid",
        lambda *_args, **_kwargs: pytest.fail(
            "local publisher timeout must not use WSL process control"
        ),
    )

    cleanup = bridge._shutdown_async_native_publisher(
        Publisher(),
        Process(),
        timeout_s=0.25,
    )

    assert cleanup["timed_out"] is True
    assert cleanup["joined_after_terminate"] is True
    assert cleanup["termination"]["process_kind"] == "local"
    assert cleanup["termination"]["term_sent"] is True
    assert cleanup["termination"]["kill_sent"] is True
    assert cleanup["termination"]["alive_after_cleanup"] is False
    assert cleanup["termination"]["clean"] is True


class _BridgePipe:
    def __init__(self, events):
        self.events = events
        self.closed = False

    def write(self, value):
        self.events.append(("write", value))
        return len(value)

    def flush(self):
        self.events.append(("flush", None))

    def close(self):
        self.closed = True


class _BridgeProcess:
    pid = 6262
    stdout = object()
    stderr = object()

    def __init__(self, events):
        self.returncode = None
        self.stdin = _BridgePipe(events)
        self.terminate_calls = 0
        self.kill_calls = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("driver_bridge", timeout)
        return self.returncode

    def terminate(self):
        self.terminate_calls += 1
        self.returncode = -15

    def kill(self):
        self.kill_calls += 1
        self.returncode = -9


class _BridgeThread:
    def __init__(self, *, target, daemon):
        self.target = target
        self.daemon = daemon

    def start(self):
        return None

    def join(self, timeout):
        return None


def _make_driver_bridge(monkeypatch, tmp_path):
    events = []
    process = _BridgeProcess(events)
    launched = []

    def popen(command, **kwargs):
        launched.append((command, kwargs))
        return process

    tokens = iter(["a" * 32, "b" * 32])
    monkeypatch.setattr(bridge.subprocess, "Popen", popen)
    monkeypatch.setattr(bridge.threading, "Thread", _BridgeThread)
    monkeypatch.setattr(bridge.secrets, "token_hex", lambda size: next(tokens))
    monkeypatch.setattr(
        bridge,
        "_wait_for_driver_bridge_transport",
        lambda child, ready_file, *, timeout_s: events.append(("transport_ready", timeout_s)),
        raising=False,
    )
    binary = tmp_path / "lingtu_mujoco_driver_bridge.exe"
    binary.write_bytes(b"placeholder")
    source = bridge.NativeDriverBridge(
        binary=binary,
        domain_id=42,
        expected_host_boot_id="host-boot-a",
        max_linear_mps=1.5,
        max_angular_rps=2.0,
        pid_file=tmp_path / "driver_bridge.pid",
    )
    return source, process, events, launched


def test_driver_bridge_launches_bidirectional_v2_and_activates_after_transport_ready(
    monkeypatch, tmp_path
):
    source, process, events, launched = _make_driver_bridge(monkeypatch, tmp_path)

    command, kwargs = launched[0]
    assert command[0].endswith("lingtu_mujoco_driver_bridge.exe")
    assert kwargs["stdin"] is subprocess.PIPE
    assert kwargs["stdout"] is subprocess.PIPE
    assert kwargs["stderr"] is subprocess.PIPE
    assert command[command.index("--bridge-boot-id") + 1] == "a" * 32
    assert command[command.index("--expected-host-boot-id") + 1] == "host-boot-a"
    assert command[command.index("--max-linear-mps") + 1] == "1.5"
    assert command[command.index("--max-angular-rps") + 1] == "2"
    assert events[0][0] == "transport_ready"
    assert events[1] == (
        "write",
        f"LT_DRIVER_ACTIVATE_V2\t{'a' * 32}\t{'b' * 32}\t1\n",
    )

    source.close()
    assert process.terminate_calls == 1


def test_driver_bridge_applied_and_heartbeat_follow_successful_physics_step(
    monkeypatch, tmp_path
):
    source, _process, events, _launched = _make_driver_bridge(monkeypatch, tmp_path)
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tnav\thost-boot-a:1234:567890\t91\t1\t-0.5\t0.25"
    )

    velocity, command = source.prepare_step()

    assert command is not None
    assert velocity.linear_x == pytest.approx(1.5)
    assert velocity.linear_y == pytest.approx(-0.75)
    assert velocity.angular_z == pytest.approx(0.5)
    writes_before_step = [value for kind, value in events if kind == "write"]
    assert all("LT_DRIVER_APPLIED_V2" not in value for value in writes_before_step)

    source.complete_step(command, step_seq=7)

    writes = [value for kind, value in events if kind == "write"]
    assert writes[-2] == (
        f"LT_DRIVER_APPLIED_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tnav\thost-boot-a:1234:567890\t91\t1\t-0.5\t0.25\t7\n"
    )
    assert writes[-1] == (
        f"LT_DRIVER_HEARTBEAT_V2\t{'a' * 32}\t{'b' * 32}\t2\t7\n"
    )
    source._handle_stdout_line(
        f"LT_DRIVER_READY_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\thost-boot-a:1234:567890\t91"
    )
    stats = source.stats()
    assert stats["driver_ready"] is True
    assert stats["accepted_sequence"] == 2
    assert stats["accepted_producer_boot_id"] == "host-boot-a:1234:567890"
    assert stats["accepted_output_sequence"] == 91


def test_driver_bridge_rejects_command_from_unexpected_producer(monkeypatch, tmp_path):
    source, _process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)

    with pytest.raises(ValueError, match="producer identity mismatch"):
        source._handle_stdout_line(
            f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
            "\t2\tnav\tother-host:1234:567890\t91\t0.2\t0\t0"
        )


@pytest.mark.parametrize(
    "producer",
    (
        "other-host:1234:567890",
        "host-boot-a",
        "host-boot-a:1234:0",
    ),
)
def test_driver_bridge_rejects_ready_ack_from_unexpected_producer(
    monkeypatch, tmp_path, producer
):
    source, _process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)

    with pytest.raises(ValueError, match="READY producer identity mismatch"):
        source._handle_stdout_line(
            f"LT_DRIVER_READY_V2\t{'a' * 32}\t{'b' * 32}"
            f"\t2\t{producer}\t91"
        )


@pytest.mark.parametrize(
    "producer",
    (
        "host-boot-a",
        "host-boot-a:0:567890",
        "host-boot-a:1234:0",
        "host-boot-a:pid:567890",
        "host-boot-a:1234:567890:extra",
    ),
)
def test_driver_bridge_rejects_malformed_derived_producer(monkeypatch, tmp_path, producer):
    source, _process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)

    with pytest.raises(ValueError, match="producer identity mismatch"):
        source._handle_stdout_line(
            f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
            f"\t2\tnav\t{producer}\t91\t0.2\t0\t0"
        )


def test_driver_bridge_never_holds_state_condition_during_control_pipe_writes(
    monkeypatch, tmp_path
):
    source, _process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tnav\thost-boot-a:1234:567890\t91\t0.2\t0\t0"
    )
    _velocity, command = source.prepare_step()
    assert command is not None
    writes = []

    def write_without_state_lock(line):
        assert source._condition._is_owned() is False
        writes.append(line)

    source._write_control_line = write_without_state_lock
    source.complete_step(command, step_seq=2)
    source.heartbeat(3)
    source.begin_deactivate()

    assert [line.split("\t", 1)[0] for line in writes] == [
        "LT_DRIVER_APPLIED_V2",
        "LT_DRIVER_HEARTBEAT_V2",
        "LT_DRIVER_HEARTBEAT_V2",
        "LT_DRIVER_DEACTIVATE_V2",
    ]


def test_driver_bridge_deactivate_is_clean_only_after_physical_zero_applied(
    monkeypatch, tmp_path
):
    source, process, events, _launched = _make_driver_bridge(monkeypatch, tmp_path)
    source.begin_deactivate()
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tdeactivate_zero\t-\t0\t0\t0\t0"
    )
    _velocity, command = source.prepare_step()
    assert command is not None
    assert source.stats()["process_cleanup"].get("clean") is not True

    source.complete_step(command, step_seq=3)
    source._handle_stdout_line(
        f"LT_DRIVER_STOPPED_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\t3\tdeactivate_zero"
    )
    process.returncode = 0
    source.wait_stopped(timeout_s=0.1)
    source.close()

    writes = [value for kind, value in events if kind == "write"]
    assert "LT_DRIVER_DEACTIVATE_V2" in writes[-3]
    assert "\tdeactivate_zero\t-\t0\t0\t0\t0\t3\n" in writes[-2]
    assert source.stats()["stopped_evidence"] == {
        "bridge_boot_id": "a" * 32,
        "controller_boot_id": "b" * 32,
        "bridge_command_seq": 2,
        "applied_step_seq": 3,
        "kind": "deactivate_zero",
    }
    assert source.stats()["process_cleanup"]["clean"] is True
    assert process.stdin.closed is True


@pytest.mark.parametrize(
    "line, reason",
    (
        (
            f"LT_DRIVER_STOPPED_V2\t{'c' * 32}\t{'b' * 32}\t2\t3\tdeactivate_zero",
            "identity mismatch",
        ),
        (
            f"LT_DRIVER_STOPPED_V2\t{'a' * 32}\t{'b' * 32}\t3\t3\tdeactivate_zero",
            "physical evidence mismatch",
        ),
        (
            f"LT_DRIVER_STOPPED_V2\t{'a' * 32}\t{'b' * 32}\t2\t4\tdeactivate_zero",
            "physical evidence mismatch",
        ),
        (
            f"LT_DRIVER_STOPPED_V2\t{'a' * 32}\t{'b' * 32}\t2\t3\tsafety_zero",
            "kind mismatch",
        ),
    ),
)
def test_driver_bridge_rejects_stopped_without_exact_physical_deactivate_evidence(
    monkeypatch, tmp_path, line, reason
):
    source, _process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)
    source.begin_deactivate()
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tdeactivate_zero\t-\t0\t0\t0\t0"
    )
    _velocity, command = source.prepare_step()
    assert command is not None
    source.complete_step(command, step_seq=3)

    with pytest.raises(ValueError, match=reason):
        source._handle_stdout_line(line)


def test_driver_bridge_retains_historical_nav_ack_after_terminal_authority_is_cleared(
    monkeypatch, tmp_path
):
    source, process, _events, _launched = _make_driver_bridge(monkeypatch, tmp_path)
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\tnav\thost-boot-a:1234:567890\t91\t0.2\t0\t0"
    )
    _velocity, nav = source.prepare_step()
    assert nav is not None
    source.complete_step(nav, step_seq=2)
    source._handle_stdout_line(
        f"LT_DRIVER_READY_V2\t{'a' * 32}\t{'b' * 32}"
        "\t2\thost-boot-a:1234:567890\t91"
    )

    source.begin_deactivate()
    source._handle_stdout_line(
        f"LT_DRIVER_COMMAND_V2\t{'a' * 32}\t{'b' * 32}"
        "\t3\tdeactivate_zero\t-\t0\t0\t0\t0"
    )
    _velocity, terminal = source.prepare_step()
    assert terminal is not None
    source.complete_step(terminal, step_seq=3)
    source._handle_stdout_line(
        f"LT_DRIVER_STOPPED_V2\t{'a' * 32}\t{'b' * 32}"
        "\t3\t3\tdeactivate_zero"
    )
    process.returncode = 0
    source.wait_stopped(timeout_s=0.1)

    stats = source.stats()
    assert stats["driver_ready_observed"] is True
    assert stats["driver_ready"] is False
    assert stats["accepted_sequence"] == 0
    assert stats["accepted_producer_boot_id"] == ""
    assert stats["accepted_output_sequence"] == 0
    assert stats["observed_output_ack"] == {
        "accepted_sequence": 2,
        "producer_boot_id": "host-boot-a:1234:567890",
        "output_sequence": 91,
    }
    assert stats["fault"] == ""


def test_driver_bridge_shutdown_drains_pending_command_then_applies_terminal_zero():
    from sim.engine.core.engine import VelocityCommand

    events = []
    nav = bridge.DriverBridgeCommand(
        2, "nav", "host-boot-a:1234:567890", 91, 0.2, 0.0, 0.0
    )
    terminal = bridge.DriverBridgeCommand(3, "deactivate_zero", "", 0, 0.0, 0.0, 0.0)

    class Engine:
        def step_sensor_tick(self, command, *, dt_s):
            events.append(("step", command.linear_x, dt_s))
            return object()

    class DriverBridge:
        apply_timeout_ms = 500

        def __init__(self):
            self._prepared = [
                bridge.PreparedDriverBridgeStep(VelocityCommand(linear_x=0.2), nav),
                bridge.PreparedDriverBridgeStep(VelocityCommand(), terminal),
            ]

        def prepare_step(self, *, wait_for_command_s=0.0):
            events.append(("prepare", wait_for_command_s))
            return self._prepared.pop(0)

        def complete_step(self, command, *, step_seq):
            events.append(("applied", command.kind, step_seq))

        def begin_deactivate(self):
            events.append(("deactivate",))

        def wait_stopped(self, *, timeout_s):
            events.append(("stopped", timeout_s))

    last_step = bridge._deactivate_driver_bridge(
        Engine(),
        DriverBridge(),
        imu_period_s=0.005,
        step_seq=40,
    )

    assert last_step == 42
    assert events == [
        ("prepare", 0.0),
        ("step", 0.2, 0.005),
        ("applied", "nav", 41),
        ("deactivate",),
        ("prepare", pytest.approx(3.0, abs=0.05)),
        ("step", 0.0, 0.005),
        ("applied", "deactivate_zero", 42),
        ("stopped", 3.0),
    ]


def test_driver_bridge_shutdown_rejects_late_nav_without_physical_step():
    from sim.engine.core.engine import VelocityCommand

    events = []
    late_nav = bridge.DriverBridgeCommand(3, "nav", "host-a", 92, 0.2, 0.0, 0.0)

    class Engine:
        def step_sensor_tick(self, command, *, dt_s):
            events.append(("step", command.linear_x, dt_s))
            return object()

    class DriverBridge:
        apply_timeout_ms = 500

        def __init__(self):
            self._prepared = [
                bridge.PreparedDriverBridgeStep(VelocityCommand(), None),
                bridge.PreparedDriverBridgeStep(VelocityCommand(linear_x=0.2), late_nav),
            ]

        def prepare_step(self, *, wait_for_command_s=0.0):
            return self._prepared.pop(0)

        def complete_step(self, command, *, step_seq):
            events.append(("applied", command.kind, step_seq))

        def begin_deactivate(self):
            events.append(("deactivate",))

        def wait_stopped(self, *, timeout_s):
            raise AssertionError("late nav cannot produce a clean shutdown")

    with pytest.raises(RuntimeError, match="non-deactivate command after shutdown began"):
        bridge._deactivate_driver_bridge(
            Engine(),
            DriverBridge(),
            imu_period_s=0.005,
            step_seq=40,
        )

    assert events == [("deactivate",)]


def test_driver_bridge_warmup_anchor_rejects_early_motion_and_releases_at_drive_start():
    with pytest.raises(RuntimeError, match="remained physically anchored"):
        bridge._driver_bridge_anchor_state(
            "warmup",
            motion_started=False,
            driving=False,
            external_arm_gate=None,
            command_norm=0.2,
        )

    anchor_active, motion_started = bridge._driver_bridge_anchor_state(
        "warmup",
        motion_started=False,
        driving=True,
        external_arm_gate=None,
        command_norm=0.2,
    )
    assert anchor_active is False
    assert motion_started is True

    anchor_active, motion_started = bridge._driver_bridge_anchor_state(
        "warmup",
        motion_started=False,
        driving=True,
        external_arm_gate=None,
        command_norm=0.0,
    )
    assert anchor_active is False
    assert motion_started is True


def test_policy_anchor_keeps_requested_xy_and_physical_standing_height():
    anchor = bridge._anchor_position_after_policy_settle(
        [3.0, 4.0, 0.6],
        [3.006, 3.997, 0.407],
        policy_settled=True,
    )
    assert anchor.tolist() == pytest.approx([3.0, 4.0, 0.407])

    unconditioned = bridge._anchor_position_after_policy_settle(
        [3.0, 4.0, 0.6],
        [3.006, 3.997, 0.407],
        policy_settled=False,
    )
    assert unconditioned.tolist() == pytest.approx([3.0, 4.0, 0.6])
