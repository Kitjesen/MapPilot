"""Focused Windows keyboard input for the native teleop-avoid stream over SSH."""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import shlex
import subprocess
import sys
import threading
import time
from ctypes import wintypes
from dataclasses import dataclass
from pathlib import Path

_PERIOD_S = 0.05
_INPUT_TIMEOUT_S = 0.35
_MOVEMENT_KEYS = frozenset("wasdqe")
_ZERO = (0.0, 0.0, 0.0)


class KeyboardState:
    """Require released keys after startup, focus loss, and an explicit stop."""

    def __init__(self, speed: float, turn_rate: float) -> None:
        self.speed = speed
        self.turn_rate = turn_rate
        self.armed = False

    def command(self, focused: bool, pressed: set[str]) -> tuple[float, float, float]:
        if not focused or "space" in pressed or "esc" in pressed:
            self.armed = False
            return _ZERO
        if not self.armed:
            if not pressed.intersection(_MOVEMENT_KEYS):
                self.armed = True
            return _ZERO
        x = int("w" in pressed) - int("s" in pressed)
        y = int("a" in pressed) - int("d" in pressed)
        scale = self.speed / max(1.0, math.hypot(x, y))
        return x * scale, y * scale, (int("q" in pressed) - int("e" in pressed)) * self.turn_rate


class ConsoleKeys:
    """Accept only the dedicated, visible classic console window's exact HWND."""

    def __init__(self) -> None:
        if os.name != "nt":
            raise RuntimeError("This keyboard client requires Windows and a classic conhost window")
        self._kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        self._user = ctypes.WinDLL("user32", use_last_error=True)
        self._kernel.GetConsoleWindow.restype = wintypes.HWND
        self._kernel.GetConsoleWindow.argtypes = []
        self._user.IsWindowVisible.argtypes = [wintypes.HWND]
        self._user.IsWindowVisible.restype = wintypes.BOOL
        self._user.GetForegroundWindow.argtypes = []
        self._user.GetForegroundWindow.restype = wintypes.HWND
        self._user.GetAsyncKeyState.argtypes = [ctypes.c_int]
        self._user.GetAsyncKeyState.restype = ctypes.c_short
        self._kernel.SetConsoleTitleW.argtypes = [wintypes.LPCWSTR]
        self._kernel.SetConsoleTitleW.restype = wintypes.BOOL
        self._window = self._kernel.GetConsoleWindow()
        if not self._window or not self._user.IsWindowVisible(self._window):
            raise RuntimeError(
                "A dedicated classic conhost window is required; Windows Terminal's hidden "
                "pseudoconsole is not accepted. Launch this command through conhost.exe."
            )
        self._kernel.GetStdHandle.argtypes = [wintypes.DWORD]
        self._kernel.GetStdHandle.restype = wintypes.HANDLE
        self._kernel.GetConsoleMode.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        self._kernel.GetConsoleMode.restype = wintypes.BOOL
        self._kernel.SetConsoleMode.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self._kernel.SetConsoleMode.restype = wintypes.BOOL
        self._input_handle = self._kernel.GetStdHandle(-10)
        mode = wintypes.DWORD()
        if not self._kernel.GetConsoleMode(self._input_handle, ctypes.byref(mode)):
            raise ctypes.WinError(ctypes.get_last_error())
        self._original_mode = mode.value
        # Extended flags must be enabled when changing QuickEdit. Preserve all
        # other input flags, including the existing Ctrl+C handling.
        if not self._kernel.SetConsoleMode(self._input_handle, (mode.value | 0x80) & ~0x40):
            raise ctypes.WinError(ctypes.get_last_error())
        self._kernel.SetConsoleTitleW("LingTu Go2 - WASD obstacle avoidance")

    def close(self) -> None:
        if self._original_mode is not None:
            if not self._kernel.SetConsoleMode(self._input_handle, self._original_mode | 0x80):
                raise ctypes.WinError(ctypes.get_last_error())
            self._original_mode = None

    def snapshot(self) -> tuple[bool, set[str]]:
        focused = self._user.GetForegroundWindow() == self._window
        if not focused:
            return False, set()
        codes = {key: ord(key.upper()) for key in _MOVEMENT_KEYS}
        codes.update(space=0x20, esc=0x1B)
        return True, {key for key, code in codes.items() if self._user.GetAsyncKeyState(code) & 0x8000}


def ssh_command(config: Path, target: str, domain_id: int) -> list[str]:
    if not target or target.startswith("-") or any(c.isspace() for c in target):
        raise ValueError("ssh-target must be an SSH configuration alias")
    if not 0 <= domain_id <= 232:
        raise ValueError("domain-id must be the active RunPlan DDS domain within 0..232")
    remote = (
        "set -e; set -a; . /opt/lingtu/config/go2-native.env; set +a; "
        "exec /opt/lingtu/current/bin/lingtu_nav_control teleop-stream "
        f"--domain-id {domain_id} --rate-hz 20 --input-timeout-ms 350 "
        "--source-id go2-keyboard"
    )
    return [
        "ssh",
        "-F",
        str(config),
        "-T",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=8",
        "-o",
        "ServerAliveInterval=2",
        "-o",
        "ServerAliveCountMax=2",
        target,
        "bash -c " + shlex.quote(remote),
    ]


def command_line(command: tuple[float, float, float]) -> str:
    # The fourth field is always assisted mode; no keyboard can enable manual bypass.
    return " ".join(f"{value:.6f}" for value in command) + " 0\n"


class NativeInputTimeout(RuntimeError):
    """The native stream explicitly reported its input-timeout stop."""


class NativeStream:
    """Pipe keyboard samples to the existing native authority/stop handshake."""

    def __init__(self, command: list[str]) -> None:
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
        )
        self._ready = threading.Event()
        self._input_timeout = threading.Event()
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    def _read_output(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            if line.strip() == "LT_TELEOP_STREAM_READY_V1":
                self._ready.set()
            if line.strip() == "LT_TELEOP_STREAM_TIMEOUT_STOP_V1":
                self._input_timeout.set()
            print(line.rstrip(), flush=True)

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 15.0
        while not self._ready.wait(0.05):
            if self.process.poll() is not None:
                raise RuntimeError("Native teleop stream exited before accepting control; see output above")
            if time.monotonic() >= deadline:
                raise RuntimeError("Native teleop stream readiness timed out; no motion was sent")

    def send(self, command: tuple[float, float, float]) -> None:
        if self._input_timeout.is_set():
            raise NativeInputTimeout("Native input timeout stopped motion")
        if self.process.poll() is not None:
            raise RuntimeError("Native teleop stream ended; keyboard motion is disabled")
        assert self.process.stdin is not None
        self.process.stdin.write(command_line(command))
        self.process.stdin.flush()

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                assert self.process.stdin is not None
                self.process.stdin.write(command_line(_ZERO) + "quit keyboard_exit\n")
                self.process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            finally:
                if self.process.stdin is not None:
                    try:
                        self.process.stdin.close()
                    except (BrokenPipeError, OSError):
                        pass
            try:
                self.process.wait(timeout=15.0)
            except subprocess.TimeoutExpired as exc:
                self.process.kill()
                self.process.wait(timeout=2.0)
                raise RuntimeError("Remote stop confirmation timed out; stop is unconfirmed") from exc
        self._reader.join(timeout=1.0)
        if self.process.returncode != 0:
            raise RuntimeError("Native stream/SSH failed; stop confirmation is not established")


@dataclass
class KeyboardTiming:
    send_calls: int = 0
    max_send_gap_s: float = 0.0
    max_send_block_s: float = 0.0
    gaps_ge_350ms: int = 0
    max_snapshot_s: float = 0.0
    max_wake_lateness_s: float = 0.0
    worst_gap_send_call: int = 0
    worst_gap_elapsed_s: float = 0.0


def run_keyboard(
    keys: ConsoleKeys,
    stream: NativeStream,
    state: KeyboardState,
    timing: KeyboardTiming | None = None,
) -> None:
    timing = timing if timing is not None else KeyboardTiming()
    last_send_started = None
    session_started = time.monotonic()

    def send(command: tuple[float, float, float]) -> None:
        nonlocal last_send_started
        started = time.monotonic()
        if last_send_started is not None:
            gap = started - last_send_started
            if gap > timing.max_send_gap_s:
                timing.max_send_gap_s = gap
                timing.worst_gap_send_call = timing.send_calls + 1
                timing.worst_gap_elapsed_s = started - session_started
            timing.gaps_ge_350ms += int(gap >= _INPUT_TIMEOUT_S)
        last_send_started = started
        timing.send_calls += 1
        try:
            stream.send(command)
        finally:
            timing.max_send_block_s = max(timing.max_send_block_s, time.monotonic() - started)

    stream.wait_ready()
    send(_ZERO)
    next_poll = time.monotonic()
    while True:
        snapshot_started = time.monotonic()
        focused, pressed = keys.snapshot()
        sampled_at = time.monotonic()
        timing.max_snapshot_s = max(timing.max_snapshot_s, sampled_at - snapshot_started)
        if sampled_at - last_send_started >= _INPUT_TIMEOUT_S:
            # A delayed observation cannot resume a previously held command.
            # Require a new, timely all-keys-released observation before motion.
            state.armed = False
            command = _ZERO
        else:
            command = state.command(focused, pressed)
        send(command)
        if focused and "esc" in pressed:
            return
        next_poll += _PERIOD_S
        now = time.monotonic()
        if next_poll <= now:
            next_poll = now + _PERIOD_S
        time.sleep(next_poll - now)
        timing.max_wake_lateness_s = max(
            timing.max_wake_lateness_s, time.monotonic() - next_poll
        )


def _positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh-config", type=Path, required=True)
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument("--domain-id", type=int, required=True, help="Active RunPlan DDS domain")
    parser.add_argument("--speed", type=_positive, default=0.2, help="Translation speed in m/s")
    parser.add_argument("--turn-rate", type=_positive, default=0.35, help="Turn rate in rad/s")
    args = parser.parse_args(argv)
    stream = None
    keys = None
    timing = KeyboardTiming()
    result = 0
    try:
        if not args.ssh_config.is_file():
            raise ValueError("SSH configuration file does not exist")
        command = ssh_command(args.ssh_config, args.ssh_target, args.domain_id)
        keys = ConsoleKeys()
        print("Connecting with zero input; start the required Product through ProductControl first.", flush=True)
        print(
            "Release all movement keys first. Focus THIS window to drive.\n"
            f"Requested speed cap: {args.speed:.2f} m/s | Turn rate: {args.turn_rate:.2f} rad/s\n"
            "W/S: forward/back | A/D: sideways | Q/E: turn\n"
            "Release keys / lose focus: zero | Space: stop | Esc: stop and exit\n"
            "Avoidance follows the active Product: teleop_avoid assists; map/teleop use direct control.\n"
            "This keyboard does not switch modes or disable the Product's safety checks.",
            flush=True,
        )
        while True:
            stream = NativeStream(command)
            try:
                run_keyboard(keys, stream, KeyboardState(args.speed, args.turn_rate), timing)
                break
            except NativeInputTimeout:
                # Reconnect only after the explicit native timeout marker AND
                # successful old-session cleanup; transport failures still exit.
                try:
                    stream.close()
                except RuntimeError as exc:
                    raise RuntimeError(f"STOP UNCONFIRMED: {exc}") from exc
                stream = None
                print(
                    "Input timeout: native zero+stop confirmed. Reconnecting with zero input.\n"
                    "Release ALL movement keys, then press again to drive.",
                    flush=True,
                )
    except KeyboardInterrupt:
        print("Stopping and releasing native control...", flush=True)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Keyboard stopped: {exc}", file=sys.stderr, flush=True)
        result = 1
    finally:
        if stream is not None:
            try:
                stream.close()
                print("Native hold/release/stop completed.", flush=True)
            except (OSError, RuntimeError) as exc:
                print(f"STOP UNCONFIRMED: {exc}", file=sys.stderr, flush=True)
                result = 1
        if timing.send_calls:
            print(
                f"Keyboard timing: send_calls={timing.send_calls} "
                f"max_send_gap_ms={timing.max_send_gap_s * 1000:.1f} "
                f"max_send_block_ms={timing.max_send_block_s * 1000:.1f} "
                f"gaps_ge_350ms={timing.gaps_ge_350ms} "
                f"max_snapshot_ms={timing.max_snapshot_s * 1000:.1f} "
                f"max_wake_lateness_ms={timing.max_wake_lateness_s * 1000:.1f} "
                f"worst_gap_send_call={timing.worst_gap_send_call} "
                f"worst_gap_elapsed_s={timing.worst_gap_elapsed_s:.3f}",
                flush=True,
            )
        if keys is not None:
            try:
                keys.close()
            except OSError as exc:
                print(f"Console settings could not be restored: {exc}", file=sys.stderr, flush=True)
                result = 1
    return result


if __name__ == "__main__":
    raise SystemExit(main())
