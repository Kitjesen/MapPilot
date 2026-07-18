from __future__ import annotations

from pathlib import Path

import pytest

from sim.scripts.mujoco.native_dds_sensors import _build_parser as build_sensor_parser
from sim.scripts.mujoco import native_navigation_acceptance as native
from sim.scripts.mujoco import teleop_avoid_wasd as wasd
from sim.scripts.mujoco.teleop_avoid_wasd import (
    ContinuousTypedTeleop,
    KeyboardCommandState,
    NativeTeleopStream,
    TeleopTwist,
    WindowsKeyPoller,
    _clean_viewer_exit,
    _record_runtime_evidence,
    _wait_for_interactive_runtime_ready,
    build_interactive_plan,
    build_parser,
)


def test_wasd_maps_to_body_frame_intent_with_shift_deadman() -> None:
    state = KeyboardCommandState(
        linear_speed_mps=0.18,
        lateral_speed_mps=0.15,
        yaw_speed_rad_s=0.35,
    )

    assert state.command({"w"}) == TeleopTwist()
    assert state.command({"shift", "w"}) == TeleopTwist(vx=0.18)
    assert state.command({"shift", "s"}) == TeleopTwist(vx=-0.18)
    assert state.command({"shift", "a"}) == TeleopTwist(vy=0.15)
    assert state.command({"shift", "d"}) == TeleopTwist(vy=-0.15)
    assert state.command({"shift", "q"}) == TeleopTwist(wz=0.35)
    assert state.command({"shift", "e"}) == TeleopTwist(wz=-0.35)
    assert state.command({"shift", "w", "space"}) == TeleopTwist()


def test_key_release_only_clears_the_released_axis() -> None:
    state = KeyboardCommandState()

    assert state.command({"shift", "w", "d"}) == TeleopTwist(vx=0.18, vy=-0.15)
    assert state.command({"shift", "d"}) == TeleopTwist(vy=-0.15)
    assert state.command({"shift"}) == TeleopTwist()


def test_opposite_keys_cancel_each_axis() -> None:
    state = KeyboardCommandState(require_deadman=False)

    assert state.command({"w", "s", "a", "d", "q", "e"}) == TeleopTwist()


def test_windows_key_poller_reports_only_held_keys() -> None:
    held = {ord("W"), 0x10}
    poller = WindowsKeyPoller(lambda code: 0x8000 if code in held else 0)

    assert poller.snapshot() == {"w", "shift"}


def test_teleop_stream_command_uses_typed_navigation_control_boundary(tmp_path: Path) -> None:
    ready_file = tmp_path / "teleop.ready"

    assert hasattr(wasd, "build_teleop_stream_command"), (
        "WASD must use a single persistent teleop-stream process instead of "
        "restarting a teleop publisher for each key-state change"
    )
    command = wasd.build_teleop_stream_command(
        tmp_path / "lingtu_nav_control",
        ready_file=ready_file,
        domain_id=231,
        rate_hz=10.0,
        timeout_ms=1000,
    )

    assert "teleop-stream" in command
    assert "0.18" not in command
    assert "-0.1" not in command
    assert "0.25" not in command
    assert command[command.index("--domain-id") + 1] == "231"
    assert command[command.index("--rate-hz") + 1] == "10.0"
    assert command[command.index("--timeout-ms") + 1] == "1000"
    assert command[command.index("--input-timeout-ms") + 1] == "350"
    assert command[command.index("--ready-file") + 1] == native._linux_arg(ready_file)
    assert "rt/nav/cmd_vel" not in command


def test_native_teleop_stream_watchdog_sends_zero_and_stops_session() -> None:
    source = Path("src/nav/services/endpoint/cpp/nav_control.cpp").read_text(encoding="utf-8")
    start = source.index("if (input_timed_out) {")
    end = source.index("if (SteadyClock::now() >= next_publish)", start)
    timeout_branch = source[start:end]

    assert "vx = 0.0;" in timeout_branch
    assert 'stop_reason = "teleop_stream_input_timeout";' in timeout_branch
    assert "client.navigation().stop(stop_reason, cfg.timeout_ms);" in timeout_branch
    assert "break;" in timeout_branch


class _FakeStdin:
    def __init__(self) -> None:
        self.writes: list[str] = []
        self.flushes = 0
        self.closed = False

    def write(self, data: str) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        self.flushes += 1

    def close(self) -> None:
        self.closed = True


class _FakeStream:
    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.started = False
        self.returncode: int | None = None
        self.sent: list[TeleopTwist] = []
        self.close_reasons: list[str] = []

    def start(self) -> None:
        self.started = True

    def send(self, twist: TeleopTwist) -> None:
        self.sent.append(twist)

    def poll(self) -> int | None:
        return self.returncode

    def close(self, reason: str) -> dict[str, object]:
        self.close_reasons.append(reason)
        if self.returncode is None:
            self.returncode = 0
        return {
            "ok": self.returncode == 0,
            "returncode": self.returncode,
            "typed_stop": self.returncode == 0,
        }


def test_typed_teleop_reuses_one_ready_stream_and_writes_updates_to_stdin(tmp_path: Path) -> None:
    streams: list[_FakeStream] = []
    stop_reasons: list[str] = []

    def factory(**kwargs: object) -> _FakeStream:
        stream = _FakeStream(**kwargs)
        streams.append(stream)
        return stream

    teleop = ContinuousTypedTeleop(
        binary=tmp_path / "lingtu_nav_control",
        domain_id=231,
        log_dir=tmp_path,
        stream_factory=factory,
        stop_sender=lambda reason: stop_reasons.append(reason) or {"returncode": 0},
    )

    teleop.apply(TeleopTwist(vx=0.18))
    teleop.apply(TeleopTwist(vx=0.18))
    assert len(streams) == 1
    teleop.apply(TeleopTwist(vy=-0.15))
    teleop.apply(TeleopTwist(wz=0.35))
    assert len(streams) == 1
    assert streams[0].started is True
    assert streams[0].sent == [
        TeleopTwist(vx=0.18),
        TeleopTwist(vx=0.18),
        TeleopTwist(vy=-0.15),
        TeleopTwist(wz=0.35),
    ]
    assert len(teleop.transitions) == 3

    result = teleop.close()

    assert streams[0].close_reasons == ["mujoco_wasd_exit"]
    assert stop_reasons == []
    assert result["returncode"] == 0


def test_typed_teleop_uses_fallback_stop_when_stream_exits_early(tmp_path: Path) -> None:
    streams: list[_FakeStream] = []
    stop_reasons: list[str] = []

    def factory(**kwargs: object) -> _FakeStream:
        stream = _FakeStream(**kwargs)
        streams.append(stream)
        return stream

    teleop = ContinuousTypedTeleop(
        binary=tmp_path / "lingtu_nav_control",
        domain_id=231,
        log_dir=tmp_path,
        stream_factory=factory,
        stop_sender=lambda reason: stop_reasons.append(reason) or {"returncode": 0},
    )

    teleop.apply(TeleopTwist(vx=0.18))
    streams[0].returncode = 17

    with pytest.raises(RuntimeError, match="typed teleop"):
        teleop.check()
    result = teleop.close()

    assert stop_reasons == ["mujoco_wasd_exit"]
    assert result["returncode"] == 0
    assert result["ok"] is False
    assert result["fallback_stop_used"] is True


class _FakePopen:
    def __init__(self) -> None:
        self.stdin = _FakeStdin()
        self.returncode: int | None = None

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float) -> int:
        self.returncode = 0
        return 0

    def terminate(self) -> None:
        self.returncode = -15

    def kill(self) -> None:
        self.returncode = -9


def test_native_stream_serializes_updates_and_graceful_quit(tmp_path: Path) -> None:
    stream = NativeTeleopStream(
        binary=tmp_path / "lingtu_nav_control",
        domain_id=231,
        log_path=tmp_path / "stream.log",
        ready_path=tmp_path / "stream.ready",
    )
    process = _FakePopen()
    stream.process = process  # type: ignore[assignment]

    stream.send(TeleopTwist(vx=0.18, vy=-0.15, wz=0.35))
    result = stream.close("mujoco_wasd_exit")

    assert process.stdin.writes == [
        "0.18 -0.15 0.35\n",
        "quit mujoco_wasd_exit\n",
    ]
    assert result["ok"] is True
    assert result["typed_stop"] is True


def test_native_stream_recognizes_watchdog_zero_stop_without_fallback(tmp_path: Path) -> None:
    log_path = tmp_path / "stream.log"
    log_path.write_text(f"{wasd.TELEOP_STREAM_TIMEOUT_STOP_MARKER}\n", encoding="utf-8")
    stream = NativeTeleopStream(
        binary=tmp_path / "lingtu_nav_control",
        domain_id=231,
        log_path=log_path,
        ready_path=tmp_path / "stream.ready",
    )
    process = _FakePopen()
    process.returncode = 0
    stream.process = process  # type: ignore[assignment]

    result = stream.close("mujoco_wasd_exit")

    assert result["ok"] is True
    assert result["typed_stop"] is True
    assert result["reason"] == "teleop_stream_input_timeout"


def test_interactive_plan_keeps_native_avoidance_and_dds_output(tmp_path: Path) -> None:
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam": tmp_path / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_interactive_plan(
        scenario="obstacle_stop",
        domain_id=231,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=60.0,
        warmup_s=10.0,
        manifest={},
        viewer=True,
        viewer_hz=24.0,
    )

    by_name = {item["name"]: item["command"] for item in plan["processes"]}
    assert by_name["navigation"][by_name["navigation"].index("--control-mode") + 1] == "teleop_avoid"
    assert by_name["sensor"][by_name["sensor"].index("--command-source") + 1] == "dds"
    assert "--require-cmd-vel" not in by_name["sensor"]
    assert "--viewer" in by_name["sensor"]
    assert by_name["sensor"][by_name["sensor"].index("--viewer-hz") + 1] == "24.0"
    assert "teleop_command" not in plan
    assert plan["interactive_control"]["direct_mujoco_control"] is False
    assert plan["interactive_control"]["state_provider"] == "mujoco_fixture"
    assert plan["interactive_control"]["require_nonzero_cmd_vel"] is False
    assert "slam" not in by_name
    assert "--navigation-fixture" in by_name["sensor"]
    assert "--publish-odom-prior" in by_name["sensor"]
    assert "--require-slam-output" not in by_name["sensor"]
    assert "--slam-status-json" not in by_name["sensor"]
    assert by_name["sensor"][by_name["sensor"].index("--scan-time-profile") + 1] == "instantaneous"


def test_fastlio_state_provider_keeps_product_slam_process(tmp_path: Path) -> None:
    binaries = {
        name: tmp_path / name
        for name in (
            "slam",
            "traversability",
            "navigation",
            "navigation_control",
            "sensor_publisher",
            "cmd_vel_tap",
        )
    }
    paths = {
        "slam": tmp_path / "map.pcd",
        "slam_config": tmp_path / "slam.yaml",
        "policy": tmp_path / "policy.onnx",
        "sensor_runner": tmp_path / "native_dds_sensors.py",
        "world": tmp_path / "scene.xml",
    }

    plan = build_interactive_plan(
        scenario="free",
        domain_id=231,
        binaries=binaries,
        paths=paths,
        case_dir=tmp_path / "case",
        duration_s=60.0,
        warmup_s=10.0,
        manifest={},
        viewer=False,
        viewer_hz=24.0,
        state_provider="fastlio2",
    )

    by_name = {item["name"]: item["command"] for item in plan["processes"]}
    assert "slam" in by_name
    assert "--require-slam-output" in by_name["sensor"]
    assert "--require-cmd-vel" in by_name["sensor"]
    assert "--navigation-fixture" not in by_name["sensor"]


class _RunningSensor:
    def poll(self) -> None:
        return None


def test_fixture_runtime_readiness_does_not_require_slam_status(tmp_path: Path) -> None:
    nav_status = tmp_path / "nav.json"
    traversability_status = tmp_path / "traversability.json"
    nav_status.write_text(
        '{"has_odom": true, "control_mode": "teleop_avoid", '
        '"input_gate": {"ready": true}}',
        encoding="utf-8",
    )
    traversability_status.write_text(
        '{"counters": {"published": 1}}',
        encoding="utf-8",
    )

    ready, reason = _wait_for_interactive_runtime_ready(
        sensor=_RunningSensor(),
        nav_status=nav_status,
        slam_status=tmp_path / "missing_slam.json",
        traversability_status=traversability_status,
        state_provider="mujoco_fixture",
        timeout_s=0.2,
    )

    assert ready is True
    assert reason == "ready_mujoco_fixture"


def test_runtime_evidence_records_obstacle_stop_and_final_zero(tmp_path: Path) -> None:
    nav_status = tmp_path / "nav.json"
    nav_status.write_text(
        '{"input_gate": {"ready": true}, "teleop": {'
        '"reason": "obstacle_stop", "obstacle_distance_m": 0.36, '
        '"traversability_cost": -1, "output": {"vx": 0, "vy": 0, "wz": 0}}}',
        encoding="utf-8",
    )
    evidence: dict[str, object] = {}

    _record_runtime_evidence(nav_status, TeleopTwist(vx=0.18), evidence)

    assert evidence["gate_ready_samples"] == 1
    assert evidence["motion_intent_samples"] == 1
    assert evidence["final_nonzero_samples"] == 0
    assert evidence["min_obstacle_distance_m"] == pytest.approx(0.36)
    assert evidence["teleop_reason_counts"] == {"obstacle_stop": 1}


def test_cli_defaults_to_safe_interactive_contract() -> None:
    args = build_parser().parse_args([])

    assert args.scenario == "obstacle_stop"
    assert args.state_provider == "mujoco_fixture"
    assert args.deadman == "shift"
    assert args.viewer is True
    assert args.linear_speed == pytest.approx(0.18)
    assert 200 <= args.domain_id <= 232

    with pytest.raises(SystemExit):
        build_parser().parse_args(["--domain-id", "233"])


class _ExitedSensor:
    def __init__(self, returncode: int) -> None:
        self.returncode = returncode

    def poll(self) -> int:
        return self.returncode


def test_viewer_exit_is_clean_only_with_successful_sensor_report(tmp_path: Path) -> None:
    report_path = tmp_path / "sensor_report.json"
    report_path.write_text(
        '{"ok": true, "viewer": {"enabled": true, "closed_early": true}}',
        encoding="utf-8",
    )

    assert _clean_viewer_exit(_ExitedSensor(0), report_path) is True
    assert _clean_viewer_exit(_ExitedSensor(1), report_path) is False

    report_path.write_text(
        '{"ok": false, "viewer": {"enabled": true, "closed_early": true}}',
        encoding="utf-8",
    )
    assert _clean_viewer_exit(_ExitedSensor(0), report_path) is False


def test_native_sensor_parser_exposes_presentation_only_viewer() -> None:
    args = build_sensor_parser().parse_args(["--viewer", "--viewer-hz", "24"])

    assert args.viewer is True
    assert args.viewer_hz == pytest.approx(24.0)
