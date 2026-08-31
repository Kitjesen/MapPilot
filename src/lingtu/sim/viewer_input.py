"""Focused MuJoCo viewer keyboard input for simulated teleoperation."""

from __future__ import annotations

import ctypes
import math
import os
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

_REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class ViewerCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    manual_mode: bool = False

    def is_zero(self) -> bool:
        return (
            abs(self.vx) < 1e-9
            and abs(self.vy) < 1e-9
            and abs(self.wz) < 1e-9
            and not self.manual_mode
        )


@dataclass(frozen=True)
class ViewerInputConfig:
    linear_speed_mps: float = 0.5
    yaw_speed_rad_s: float = 0.6
    rate_hz: float = 20.0
    lease_ttl_ms: int = 1000
    freshness_budget_ms: int = 350
    source_id: str = "mujoco-viewer"

    def __post_init__(self) -> None:
        for name, value in (
            ("linear_speed_mps", self.linear_speed_mps),
            ("yaw_speed_rad_s", self.yaw_speed_rad_s),
            ("rate_hz", self.rate_hz),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 1 <= self.lease_ttl_ms <= 2000:
            raise ValueError("lease_ttl_ms must be within 1..2000")
        if self.freshness_budget_ms <= 0:
            raise ValueError("freshness_budget_ms must be positive")


class _OperatorClient(Protocol):
    def claim(self, source_id: str, source_epoch: int, sequence: int, **kwargs: Any) -> Any: ...

    def sample(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        vx: float,
        vy: float,
        wz: float,
        **kwargs: Any,
    ) -> bool: ...

    def hold(self, source_id: str, source_epoch: int, sequence: int, **kwargs: Any) -> Any: ...

    def release(self, source_id: str, source_epoch: int, sequence: int, **kwargs: Any) -> Any: ...

    def close(self) -> None: ...


def _foreground_window_title() -> str:
    if os.name != "nt":
        return ""
    user32 = ctypes.windll.user32
    window = user32.GetForegroundWindow()
    if not window:
        return ""
    length = int(user32.GetWindowTextLengthW(window))
    buffer = ctypes.create_unicode_buffer(max(1, length + 1))
    user32.GetWindowTextW(window, buffer, len(buffer))
    return buffer.value


class ViewerKeys:
    """Read held keys only while a MuJoCo window owns keyboard focus."""

    _VIRTUAL_KEYS = {
        "w": ord("W"),
        "a": ord("A"),
        "s": ord("S"),
        "d": ord("D"),
        "q": ord("Q"),
        "e": ord("E"),
        "m": ord("M"),
        "space": 0x20,
    }

    def __init__(
        self,
        *,
        key_state: Callable[[int], int] | None = None,
        foreground_title: Callable[[], str] | None = None,
    ) -> None:
        if key_state is None:
            if os.name != "nt":
                raise RuntimeError("MuJoCo viewer keyboard input currently requires Windows")
            key_state = ctypes.windll.user32.GetAsyncKeyState
        self._key_state = key_state
        self._foreground_title = foreground_title or _foreground_window_title

    def snapshot(self) -> set[str]:
        if not self._foreground_title().strip().lower().startswith("mujoco"):
            return set()
        return {
            name
            for name, code in self._VIRTUAL_KEYS.items()
            if int(self._key_state(code)) & 0x8000
        }

    def command(
        self,
        *,
        linear_speed_mps: float,
        yaw_speed_rad_s: float,
    ) -> ViewerCommand:
        pressed = self.snapshot()
        if "space" in pressed:
            return ViewerCommand()
        return ViewerCommand(
            vx=(int("w" in pressed) - int("s" in pressed)) * linear_speed_mps,
            vy=(int("a" in pressed) - int("d" in pressed)) * linear_speed_mps,
            wz=(int("q" in pressed) - int("e" in pressed)) * yaw_speed_rad_s,
            manual_mode="m" in pressed,
        )


class ViewerInput:
    """Publish focused viewer keys through native operator-motion authority."""

    def __init__(
        self,
        config: ViewerInputConfig,
        *,
        keys: ViewerKeys,
        client_factory: Callable[[], _OperatorClient],
        source_epoch: int | None = None,
    ) -> None:
        self.config = config
        self._keys = keys
        self._client_factory = client_factory
        self._source_epoch = max(1, int(source_epoch or time.time_ns()))
        self._sequence = 0
        self._client: _OperatorClient | None = None
        self._claimed = False
        self._armed = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_error = ""

    @property
    def last_error(self) -> str:
        return self._last_error

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="mujoco-viewer-input",
            daemon=True,
        )
        self._thread.start()

    def poll_once(self) -> None:
        command = self._keys.command(
            linear_speed_mps=self.config.linear_speed_mps,
            yaw_speed_rad_s=self.config.yaw_speed_rad_s,
        )
        if not self._armed:
            if command.is_zero():
                self._armed = True
            return
        if command.is_zero():
            if self._claimed:
                self._hold_and_release("viewer_keys_released")
            return
        client = self._ensure_client()
        if not self._claimed:
            receipt = client.claim(
                self.config.source_id,
                self._source_epoch,
                self._next_sequence(),
                lease_ttl_ms=self.config.lease_ttl_ms,
            )
            if not bool(getattr(receipt, "accepted", False)):
                raise RuntimeError(
                    "MuJoCo viewer operator claim rejected: "
                    + str(getattr(receipt, "reason", "rejected"))
                )
            self._claimed = True
        client.sample(
            self.config.source_id,
            self._source_epoch,
            self._next_sequence(),
            command.vx,
            command.vy,
            command.wz,
            deadman=True,
            manual_mode=command.manual_mode,
            freshness_budget_ms=self.config.freshness_budget_ms,
        )

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        if self._claimed:
            self._hold_and_release("viewer_closed")
        if self._client is not None:
            self._client.close()
            self._client = None

    def _run(self) -> None:
        period_s = 1.0 / self.config.rate_hz
        while not self._stop.wait(period_s):
            try:
                self.poll_once()
                self._last_error = ""
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                if message != self._last_error:
                    print(f"mujoco viewer input: {message}", file=sys.stderr, flush=True)
                self._last_error = message
                self._reset_client()

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    def _ensure_client(self) -> _OperatorClient:
        if self._client is None:
            self._client = self._client_factory()
        return self._client

    def _hold_and_release(self, reason: str) -> None:
        client = self._ensure_client()
        hold = client.hold(
            self.config.source_id,
            self._source_epoch,
            self._next_sequence(),
            reason=reason,
        )
        if not bool(getattr(hold, "accepted", False)):
            raise RuntimeError(
                "MuJoCo viewer hold rejected: " + str(getattr(hold, "reason", "rejected"))
            )
        release = client.release(
            self.config.source_id,
            self._source_epoch,
            self._next_sequence(),
            reason=reason,
        )
        if not bool(getattr(release, "accepted", False)):
            raise RuntimeError(
                "MuJoCo viewer release rejected: "
                + str(getattr(release, "reason", "rejected"))
            )
        self._claimed = False
        self._source_epoch += 1

    def _reset_client(self) -> None:
        self._claimed = False
        self._source_epoch += 1
        if self._client is not None:
            self._client.close()
            self._client = None


def viewer_input_from_run_plan(
    plan: Any,
    *,
    client_type: Callable[..., _OperatorClient],
) -> ViewerInput | None:
    """Build viewer input from the already-resolved simulation RunPlan."""

    if os.name != "nt" or str(plan.product) not in {"teleop", "teleop_avoid"}:
        return None
    host = plan.process("host")
    if host is None or host.command is None:
        raise RuntimeError("MuJoCo viewer input requires the Host native client binding")
    host_environment = dict(host.command.env)
    library_value = str(host_environment.get("LINGTU_NAV_CLIENT_LIB") or "").strip()
    if not library_value:
        raise RuntimeError("MuJoCo viewer input requires LINGTU_NAV_CLIENT_LIB")
    library_path = Path(library_value)
    if not library_path.is_absolute():
        library_path = (_REPOSITORY_ROOT / library_path).resolve()
    native_environment = plan.native_process_environment
    domain_id = int(
        native_environment.get("LINGTU_DDS_DOMAIN_ID")
        or host_environment.get("LINGTU_DDS_DOMAIN_ID")
        or 0
    )
    linear_speed = min(
        0.5,
        float(native_environment.get("LINGTU_TELEOP_MAX_SPEED_MPS") or 0.5),
    )
    yaw_speed = min(
        0.6,
        float(native_environment.get("LINGTU_TELEOP_MAX_YAW_RATE") or 0.6),
    )
    return ViewerInput(
        ViewerInputConfig(
            linear_speed_mps=linear_speed,
            yaw_speed_rad_s=yaw_speed,
        ),
        keys=ViewerKeys(),
        client_factory=lambda: client_type(
            library_path,
            domain_id=domain_id,
            timeout_ms=1000,
            operator_motion_timeout_ms=1000,
        ),
    )
