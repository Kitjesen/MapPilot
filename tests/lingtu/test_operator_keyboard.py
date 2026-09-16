from __future__ import annotations

import io
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from lingtu import operator_keyboard as keyboard


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("w", (0.2, 0.0, 0.0)),
        ("s", (-0.2, 0.0, 0.0)),
        ("a", (0.0, 0.2, 0.0)),
        ("d", (0.0, -0.2, 0.0)),
        ("q", (0.0, 0.0, 0.35)),
        ("e", (0.0, 0.0, -0.35)),
    ],
)
def test_physical_key_mapping_keeps_sideways_separate_from_turning(key, expected):
    state = keyboard.KeyboardState(0.2, 0.35)
    assert state.command(True, set()) == (0.0, 0.0, 0.0)
    assert state.command(True, {key}) == expected
    assert state.command(True, set()) == (0.0, 0.0, 0.0)


def test_startup_focus_loss_and_space_require_released_keys_before_motion():
    state = keyboard.KeyboardState(0.2, 0.35)
    zero = (0.0, 0.0, 0.0)
    assert state.command(True, {"w"}) == zero
    state.command(True, set())
    assert state.command(True, {"w"})[0] == 0.2
    assert state.command(False, {"w"}) == zero
    assert state.command(True, {"w"}) == zero
    state.command(True, set())
    assert state.command(True, {"a"})[1] == 0.2
    assert state.command(True, {"space", "a"}) == zero
    assert state.command(True, {"a"}) == zero
    state.command(True, set())
    assert state.command(True, {"a"})[1] == 0.2
    assert state.command(True, {"esc", "a"}) == zero


def test_opposite_keys_cancel_and_diagonal_does_not_exceed_requested_speed():
    state = keyboard.KeyboardState(0.2, 0.35)
    state.command(True, set())
    assert state.command(True, set("wasdqe")) == (0.0, 0.0, 0.0)
    x, y, yaw = state.command(True, {"w", "a"})
    assert x == y
    assert x * x + y * y == pytest.approx(0.2**2)
    assert yaw == 0.0
    assert state.command(True, {"m"}) == (0.0, 0.0, 0.0)


def test_console_input_accepts_only_its_exact_window():
    keys = keyboard.ConsoleKeys.__new__(keyboard.ConsoleKeys)
    keys._window = 91
    calls = []
    keys._user = SimpleNamespace(
        GetForegroundWindow=lambda: 92,
        GetAsyncKeyState=lambda code: calls.append(code) or 0x8000,
    )
    assert keys.snapshot() == (False, set())
    assert calls == []
    keys._user.GetForegroundWindow = lambda: 91
    keys._user.GetAsyncKeyState = lambda code: 0x8000 if code == ord("A") else 0
    assert keys.snapshot() == (True, {"a"})


def test_hidden_windows_terminal_pseudoconsole_is_rejected(monkeypatch):
    kernel = SimpleNamespace(GetConsoleWindow=lambda: 91, SetConsoleTitleW=lambda _: True)
    user = SimpleNamespace(
        IsWindowVisible=lambda _: False,
        GetForegroundWindow=lambda: 92,
        GetAsyncKeyState=lambda _: 0,
    )
    monkeypatch.setattr(keyboard.os, "name", "nt")
    monkeypatch.setattr(
        keyboard.ctypes,
        "WinDLL",
        lambda name, **_: kernel if name == "kernel32" else user,
        raising=False,
    )
    with pytest.raises(RuntimeError, match=r"classic conhost.*Windows Terminal"):
        keyboard.ConsoleKeys()


@pytest.mark.parametrize("original_mode", [0x01F7, 0x0037])
def test_quickedit_is_disabled_preserving_other_flags_and_restored(monkeypatch, original_mode):
    modes = []

    def get_mode(handle, output):
        assert handle == 71
        keyboard.ctypes.cast(output, keyboard.ctypes.POINTER(keyboard.wintypes.DWORD)).contents.value = original_mode
        return True

    kernel = SimpleNamespace(
        GetConsoleWindow=lambda: 91,
        SetConsoleTitleW=lambda _: True,
        GetStdHandle=lambda _: 71,
        GetConsoleMode=get_mode,
        SetConsoleMode=lambda handle, value: modes.append((handle, value)) or True,
    )
    user = SimpleNamespace(
        IsWindowVisible=lambda _: True,
        GetForegroundWindow=lambda: 91,
        GetAsyncKeyState=lambda _: 0,
    )
    monkeypatch.setattr(keyboard.os, "name", "nt")
    monkeypatch.setattr(
        keyboard.ctypes,
        "WinDLL",
        lambda name, **_: kernel if name == "kernel32" else user,
        raising=False,
    )
    keys = keyboard.ConsoleKeys()
    assert modes == [(71, (original_mode | 0x80) & ~0x40)]
    keys.close()
    keys.close()
    assert modes == [(71, (original_mode | 0x80) & ~0x40), (71, original_mode | 0x80)]


def test_ssh_uses_explicit_config_native_environment_and_supported_stream():
    command = keyboard.ssh_command(Path("private-ssh.conf"), "lingtu-go2-nx", 7)
    assert command[:4] == ["ssh", "-F", "private-ssh.conf", "-T"]
    assert "BatchMode=yes" in command
    assert command[-2] == "lingtu-go2-nx"
    assert ". /opt/lingtu/config/go2-native.env;" in command[-1]
    assert "exec /opt/lingtu/current/bin/lingtu_nav_control teleop-stream" in command[-1]
    assert "--domain-id 7 --rate-hz 20 --input-timeout-ms 350" in command[-1]
    assert "gateway.env" not in command[-1]
    assert keyboard.command_line((0.2, -0.2, 0.35)) == "0.200000 -0.200000 0.350000 0\n"
    assert keyboard.command_line((0.0, 0.0, 0.0)).split()[-1] == "0"


class FakeInput(io.StringIO):
    captured = ""

    def close(self):
        self.captured = self.getvalue()
        super().close()


class FakeProcess:
    def __init__(self, *, ready=True, exit_code=None):
        self.stdin = FakeInput()
        self.stdout = io.StringIO("LT_TELEOP_STREAM_READY_V1\n" if ready else "")
        self.returncode = exit_code
        self.waits = []

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.waits.append(timeout)
        self.returncode = 0
        return 0


def test_mock_transport_starts_zero_and_closes_with_native_cleanup(monkeypatch):
    child = FakeProcess()
    launches = []

    def launch(command, **kwargs):
        launches.append((command, kwargs))
        return child

    monkeypatch.setattr(keyboard.subprocess, "Popen", launch)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    stream.wait_ready()
    stream.send((0.0, 0.0, 0.0))
    stream.send((0.2, 0.0, 0.0))
    stream.close()
    assert child.stdin.captured.splitlines() == [
        "0.000000 0.000000 0.000000 0",
        "0.200000 0.000000 0.000000 0",
        "0.000000 0.000000 0.000000 0",
        "quit keyboard_exit",
    ]
    assert child.waits == [15.0]
    assert launches[0][1]["stdin"] == keyboard.subprocess.PIPE
    assert "shell" not in launches[0][1]


def test_stream_exit_before_readiness_is_not_reported_ready(monkeypatch):
    child = FakeProcess(ready=False, exit_code=1)
    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    with pytest.raises(RuntimeError, match="before accepting control"):
        stream.wait_ready()
    with pytest.raises(RuntimeError, match="stop confirmation is not established"):
        stream.close()
    assert child.stdin.getvalue() == ""


def test_remote_cleanup_timeout_is_not_reported_as_confirmed_stop(monkeypatch):
    child = FakeProcess()
    killed = []

    def wait(timeout):
        child.waits.append(timeout)
        if timeout == 15.0:
            raise keyboard.subprocess.TimeoutExpired("ssh", timeout)
        return child.returncode

    def kill():
        killed.append(True)
        child.returncode = -9

    child.wait = wait
    child.kill = kill
    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    stream.wait_ready()
    with pytest.raises(RuntimeError, match="stop is unconfirmed"):
        stream.close()
    assert killed == [True]
    assert child.waits == [15.0, 2.0]
    assert child.stdin.captured.endswith("quit keyboard_exit\n")


def test_control_loop_sends_zero_on_focus_loss_and_escape(monkeypatch):
    observations = iter(
        [
            (True, {"w"}),
            (True, set()),
            (True, {"a"}),
            (False, {"a"}),
            (True, {"a"}),
            (True, set()),
            (True, {"d"}),
            (True, {"esc"}),
        ]
    )
    keys = SimpleNamespace(snapshot=lambda: next(observations))
    samples = []
    stream = SimpleNamespace(wait_ready=lambda: None, send=samples.append)
    monkeypatch.setattr(keyboard.time, "sleep", lambda _: None)
    keyboard.run_keyboard(keys, stream, keyboard.KeyboardState(0.2, 0.35))
    assert samples == [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.2, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, -0.2, 0.0),
        (0.0, 0.0, 0.0),
    ]


def test_blocked_ready_display_cannot_hold_control_loop_or_released_key_zero(monkeypatch):
    child = FakeProcess()
    display_started = threading.Event()
    release_display = threading.Event()

    def blocked_print(*args, **kwargs):
        if threading.current_thread() is threading.main_thread():
            pytest.fail("Control loop must not perform console output")
        display_started.set()
        release_display.wait(timeout=5)

    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    monkeypatch.setattr(keyboard, "print", blocked_print, raising=False)
    monkeypatch.setattr(keyboard.time, "sleep", lambda _: None)
    observations = iter([(True, set()), (True, {"w"}), (True, set()), (True, {"esc"})])
    stream = keyboard.NativeStream(["ssh", "test-only"])
    try:
        assert display_started.wait(timeout=1)
        assert stream._ready.is_set()
        keyboard.run_keyboard(
            SimpleNamespace(snapshot=lambda: next(observations)),
            stream,
            keyboard.KeyboardState(0.2, 0.35),
        )
        assert not release_display.is_set()
        assert child.stdin.getvalue().splitlines()[-3:] == [
            "0.200000 0.000000 0.000000 0",
            "0.000000 0.000000 0.000000 0",
            "0.000000 0.000000 0.000000 0",
        ]
    finally:
        release_display.set()
        stream.close()


def test_deadlines_recover_sleep_overshoot_without_accumulating_cadence_drift(monkeypatch):
    now = [0.0]
    samples = []
    observations = iter([(True, set())] * 5 + [(True, {"esc"})])

    def sleep(delay):
        assert delay > 0.0
        now[0] += delay + 0.0125

    monkeypatch.setattr(keyboard.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(keyboard.time, "sleep", sleep)
    keyboard.run_keyboard(
        SimpleNamespace(snapshot=lambda: next(observations)),
        SimpleNamespace(wait_ready=lambda: None, send=lambda _: samples.append(now[0])),
        keyboard.KeyboardState(0.2, 0.35),
    )
    assert samples[2:] == pytest.approx([0.0625, 0.1125, 0.1625, 0.2125, 0.2625])


def test_slow_send_records_timing_and_skips_missed_polls_without_old_command_replay(monkeypatch):
    now = [0.0]
    samples = []
    sleeps = []
    observations = iter([(True, set()), (True, {"w"}), (True, set()), (True, {"esc"})])

    def send(command):
        samples.append((now[0], command))
        if command[0] != 0.0:
            now[0] += 0.4

    def sleep(delay):
        sleeps.append(delay)
        now[0] += delay

    monkeypatch.setattr(keyboard.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(keyboard.time, "sleep", sleep)
    timing = keyboard.KeyboardTiming()
    keyboard.run_keyboard(
        SimpleNamespace(snapshot=lambda: next(observations)),
        SimpleNamespace(wait_ready=lambda: None, send=send),
        keyboard.KeyboardState(0.2, 0.35),
        timing,
    )
    assert [sample[0] for sample in samples] == pytest.approx([0, 0, 0.05, 0.5, 0.55])
    assert sleeps == pytest.approx([0.05, 0.05, 0.05])
    assert [sample[1][0] for sample in samples] == [0.0, 0.0, 0.2, 0.0, 0.0]
    assert timing.send_calls == 5
    assert timing.max_send_block_s == pytest.approx(0.4)
    assert timing.max_send_gap_s == pytest.approx(0.45)
    assert timing.gaps_ge_350ms == 1


def test_main_prints_instructions_before_ssh_and_restores_console_after_cleanup(tmp_path, monkeypatch):
    config = tmp_path / "ssh.conf"
    config.write_text("Host test\n", encoding="utf-8")
    events = []
    keys = SimpleNamespace(snapshot=lambda: (True, {"esc"}), close=lambda: events.append("restore_console"))
    stream = SimpleNamespace(
        wait_ready=lambda: None,
        send=lambda _: events.append("send"),
        close=lambda: events.append("native_cleanup"),
    )
    monkeypatch.setattr(keyboard, "ConsoleKeys", lambda: keys)
    monkeypatch.setattr(keyboard, "NativeStream", lambda _: events.append("ssh") or stream)
    monkeypatch.setattr(keyboard, "print", lambda message, **_: events.append(str(message)), raising=False)
    assert (
        keyboard.main(
            [
                "--ssh-config",
                str(config),
                "--ssh-target",
                "test",
                "--domain-id",
                "0",
            ]
        )
        == 0
    )
    instruction_index = next(i for i, text in enumerate(events) if "Release all movement keys" in text)
    assert instruction_index < events.index("ssh")
    assert events.index("native_cleanup") < events.index("restore_console")
    assert events[-1] == "restore_console"
    assert any(text.startswith("Keyboard timing: send_calls=2 ") for text in events)


def test_unsupported_console_never_starts_ssh(tmp_path, monkeypatch):
    config = tmp_path / "ssh.conf"
    config.write_text("Host test\n", encoding="utf-8")

    def reject_console():
        raise RuntimeError("classic conhost is required")

    monkeypatch.setattr(keyboard, "ConsoleKeys", reject_console)
    monkeypatch.setattr(keyboard, "NativeStream", lambda _: pytest.fail("SSH must not start"))
    assert (
        keyboard.main(
            [
                "--ssh-config",
                str(config),
                "--ssh-target",
                "test",
                "--domain-id",
                "0",
            ]
        )
        == 1
    )


def test_native_input_timeout_is_recorded_before_blocked_display_and_clean_exit(monkeypatch):
    child = FakeProcess()
    child.stdout = io.StringIO("LT_TELEOP_STREAM_READY_V1\nLT_TELEOP_STREAM_TIMEOUT_STOP_V1\n")
    display_started = threading.Event()
    release_display = threading.Event()

    def display(message, **kwargs):
        if message == "LT_TELEOP_STREAM_TIMEOUT_STOP_V1":
            display_started.set()
            release_display.wait(timeout=5)

    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    monkeypatch.setattr(keyboard, "print", display, raising=False)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    try:
        assert display_started.wait(timeout=1)
        assert stream._input_timeout.is_set()
        child.returncode = 0
        with pytest.raises(keyboard.NativeInputTimeout):
            stream.send((0.5, 0.0, 0.0))
        assert child.stdin.getvalue() == ""
    finally:
        release_display.set()
        stream.close()


@pytest.mark.parametrize("exit_code", [0, 1])
def test_exit_without_timeout_marker_is_not_recoverable_input_timeout(monkeypatch, exit_code):
    child = FakeProcess(exit_code=exit_code)
    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    stream._reader.join(timeout=1)
    with pytest.raises(RuntimeError) as error:
        stream.send((0.5, 0.0, 0.0))
    assert not isinstance(error.value, keyboard.NativeInputTimeout)
    assert child.stdin.getvalue() == ""


@pytest.mark.parametrize(
    "line",
    ["prefix LT_TELEOP_STREAM_TIMEOUT_STOP_V1", "LT_TELEOP_STREAM_TIMEOUT_STOP_V1 suffix"],
)
def test_timeout_marker_requires_a_complete_protocol_line(monkeypatch, line):
    child = FakeProcess()
    child.stdout = io.StringIO("LT_TELEOP_STREAM_READY_V1\n" + line + "\n")
    monkeypatch.setattr(keyboard.subprocess, "Popen", lambda *_, **__: child)
    stream = keyboard.NativeStream(["ssh", "test-only"])
    try:
        stream._reader.join(timeout=1)
        assert not stream._input_timeout.is_set()
        stream.send((0.0, 0.0, 0.0))
        assert child.stdin.getvalue() == keyboard.command_line((0.0, 0.0, 0.0))
    finally:
        stream.close()


def test_main_restarts_confirmed_input_timeout_with_fresh_release_gate(tmp_path, monkeypatch):
    config = tmp_path / "ssh.conf"
    config.write_text("Host test\n", encoding="utf-8")
    observations = iter(
        [(True, set()), (True, {"w"}), (True, {"w"}), (True, {"w"}),
         (True, set()), (True, {"w"}), (True, {"esc"})]
    )
    events = []
    streams = []

    def launch(_):
        index = len(streams)
        events.append(("launch", index))
        samples = []

        def send(command):
            if index == 0 and command[0] != 0.0:
                raise keyboard.NativeInputTimeout("native input timeout")
            samples.append(command)

        stream = SimpleNamespace(
            samples=samples,
            wait_ready=lambda: None,
            send=send,
            close=lambda: events.append(("confirmed_stop", index)),
        )
        streams.append(stream)
        return stream

    monkeypatch.setattr(
        keyboard, "ConsoleKeys",
        lambda: SimpleNamespace(snapshot=lambda: next(observations), close=lambda: None),
    )
    monkeypatch.setattr(keyboard, "NativeStream", launch)
    monkeypatch.setattr(keyboard.time, "sleep", lambda _: None)
    assert keyboard.main(
        ["--ssh-config", str(config), "--ssh-target", "test", "--domain-id", "0", "--speed", "0.5"]
    ) == 0
    assert len(streams) == 2
    assert events.index(("confirmed_stop", 0)) < events.index(("launch", 1))
    assert [sample[0] for sample in streams[1].samples] == [0.0, 0.0, 0.0, 0.0, 0.5, 0.0]
    assert events[-1] == ("confirmed_stop", 1)


@pytest.mark.parametrize("failure", ["ssh_exit", "unconfirmed_stop"])
def test_main_does_not_retry_generic_failure_or_unconfirmed_timeout_stop(tmp_path, monkeypatch, failure):
    config = tmp_path / "ssh.conf"
    config.write_text("Host test\n", encoding="utf-8")
    launches = []
    stops = []

    def send(_):
        if failure == "ssh_exit":
            raise RuntimeError("Native teleop stream ended; keyboard motion is disabled")
        raise keyboard.NativeInputTimeout("native input timeout")

    def close():
        stops.append(True)
        if failure == "unconfirmed_stop":
            raise RuntimeError("Remote stop confirmation timed out; stop is unconfirmed")

    def launch(_):
        launches.append(True)
        return SimpleNamespace(wait_ready=lambda: None, send=send, close=close)

    monkeypatch.setattr(
        keyboard, "ConsoleKeys",
        lambda: SimpleNamespace(snapshot=lambda: (True, {"w"}), close=lambda: None),
    )
    monkeypatch.setattr(keyboard, "NativeStream", launch)
    assert keyboard.main(
        ["--ssh-config", str(config), "--ssh-target", "test", "--domain-id", "0"]
    ) == 1
    assert len(launches) == 1
    assert stops


def test_long_sleep_overshoot_disarms_held_motion_until_release_and_new_press(monkeypatch):
    now = [0.0]
    sleep_calls = [0]
    samples = []
    observations = iter(
        [(True, set()), (True, {"w"}), (True, {"w"}), (True, {"w"}),
         (True, set()), (True, {"w"}), (True, {"esc"})]
    )

    def sleep(delay):
        sleep_calls[0] += 1
        now[0] += delay + (1.028 if sleep_calls[0] == 2 else 0.0)

    monkeypatch.setattr(keyboard.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(keyboard.time, "sleep", sleep)
    timing = keyboard.KeyboardTiming()
    keyboard.run_keyboard(
        SimpleNamespace(snapshot=lambda: next(observations)),
        SimpleNamespace(wait_ready=lambda: None, send=samples.append),
        keyboard.KeyboardState(0.5, 0.35),
        timing,
    )
    assert [sample[0] for sample in samples] == [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0]
    assert timing.max_send_gap_s == pytest.approx(1.078)
    assert timing.max_send_block_s == 0.0
    assert timing.max_snapshot_s == 0.0
    assert timing.max_wake_lateness_s == pytest.approx(1.028)
    assert timing.gaps_ge_350ms == 1
    assert timing.worst_gap_send_call == 4
    assert timing.worst_gap_elapsed_s == pytest.approx(1.128)


def test_slow_key_snapshot_discards_sample_and_requires_release_before_motion(monkeypatch):
    now = [0.0]
    snapshots = [0]
    samples = []
    observations = iter(
        [(True, set()), (True, {"w"}), (True, {"w"}),
         (True, set()), (True, {"w"}), (True, {"esc"})]
    )

    def snapshot():
        snapshots[0] += 1
        if snapshots[0] == 2:
            now[0] += 0.35
        return next(observations)

    def sleep(delay):
        now[0] += delay

    monkeypatch.setattr(keyboard.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(keyboard.time, "sleep", sleep)
    timing = keyboard.KeyboardTiming()
    keyboard.run_keyboard(
        SimpleNamespace(snapshot=snapshot),
        SimpleNamespace(wait_ready=lambda: None, send=samples.append),
        keyboard.KeyboardState(0.5, 0.35),
        timing,
    )
    assert [sample[0] for sample in samples] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0]
    assert timing.max_snapshot_s == pytest.approx(0.35)
    assert timing.max_send_block_s == 0.0
    assert timing.max_wake_lateness_s == 0.0
    assert timing.gaps_ge_350ms == 1


def test_interrupting_timeout_cleanup_finishes_stop_without_reconnecting(tmp_path, monkeypatch):
    config = tmp_path / "ssh.conf"
    config.write_text("Host test\n", encoding="utf-8")
    events = []
    close_calls = [0]

    def send(_):
        raise keyboard.NativeInputTimeout("native input timeout")

    def close():
        close_calls[0] += 1
        events.append(("close", close_calls[0]))
        if close_calls[0] == 1:
            raise KeyboardInterrupt
        events.append(("confirmed_stop", close_calls[0]))

    def launch(_):
        events.append(("launch",))
        return SimpleNamespace(wait_ready=lambda: None, send=send, close=close)

    monkeypatch.setattr(
        keyboard, "ConsoleKeys",
        lambda: SimpleNamespace(
            snapshot=lambda: (True, {"w"}),
            close=lambda: events.append(("restore_console",)),
        ),
    )
    monkeypatch.setattr(keyboard, "NativeStream", launch)
    assert keyboard.main(
        ["--ssh-config", str(config), "--ssh-target", "test", "--domain-id", "0"]
    ) == 0
    assert events == [
        ("launch",),
        ("close", 1),
        ("close", 2),
        ("confirmed_stop", 2),
        ("restore_console",),
    ]
