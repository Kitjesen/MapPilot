"""Single assembly and lifecycle seam for the fixed RobotSimUE playable slice.

The module consumes an already validated :class:`ResolvedSessionBundle`.  It
does not resolve catalog inputs and construction alone never starts MuJoCo or
RobotSimUE.  Runtime side effects begin only in :func:`run_playable_launch`.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time
import unicodedata
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol, TextIO, TypeVar

from sim.runtime.control import create_production_components
from sim.runtime.process_owner import ProcessShutdownSnapshot
from sim.runtime.recording import CameraShmPayloadSource
from sim.runtime.sensors import (
    ImuEndpointFactory,
    Mid360EndpointFactory,
    SensorEndpointFactory,
    SensorEndpointRouter,
    TruthOdometryEndpointFactory,
)
from sim.runtime.windows_cpu_isolation import (
    WindowsCpuIsolationConfig,
    WindowsCpuIsolationPlan,
    resolve_windows_cpu_isolation,
)

from .control_intent_udp import (
    BoundedRuntimeRequestInbox,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    UdpLoopbackControlAckPublisher,
    UdpLoopbackOperatorIntentReceiver,
)
from .controlled_run import BaseTwistTarget
from .coordinator import RuntimeCoordinator, RuntimeState
from .external_evidence import ExternalEvidenceWatcher
from .interactive_session import InteractiveSimulationSession
from .live_snapshot import UdpLoopbackSnapshotPublisher
from .mujoco_process import MujocoProcess
from .playable_control import PlayableControlPump
from .playable_evidence import (
    PinnedPlayableMediaToolchain,
    PlayableEvidenceError,
    read_stable_playable_jsonl,
    snapshot_playable_media_toolchain,
)
from .playable_recording import InteractiveRecordingController
from .run_allocation import (
    ResolvedSessionBundle,
    RunAllocationError,
    load_resolved_session_bundle,
)
from .session_host import SessionHost
from .unreal_process import PackagedUnrealProcess, UnrealLaunchProfile, UnrealProcess

PLAYABLE_SESSION_ID = "thunderv4_factory_park_hf"
PLAYABLE_LEVEL = (
    "/Game/RobotSim/Generated/FactoryParkHF/Maps/"
    "FactoryPark_HF_Instanced_9d314c86ef83ea06"
)
PLAYABLE_WORLD_PACKAGE = ("factory_park_hf", "1.2.1")
PLAYABLE_ROBOT_PACKAGE = ("thunderv4", "1.0.3")
PLAYABLE_CONTROLLER_PACKAGE = ("thunderv4_locomotion", "1.0.0")
PLAYABLE_CONTROLLER_MANIFEST_PATH = "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml"
PLAYABLE_CONTROLLER_COMMAND_CALIBRATION: Mapping[str, Any] = MappingProxyType(
    {
        "schema": "lingtu.sim.controller-command-calibration.v1",
        "scope": "quadruped_him_observation_only",
        "provenance": {
            "source_id": "factory_park_turn_truth_qa_20260809",
            "audit_note": (
                "Source identifier records the calibration rationale only; no warehouse "
                "artifact is claimed as yaw-turn qualification proof."
            ),
            "qualification_claim": False,
        },
        "external_yaw_cap_radps": 0.35,
        "policy_yaw_observation_gain": 1.2857142857142858,
        "policy_yaw_observation_limit_radps": 0.45,
        "leaves_base_twist_unchanged": True,
        "affected_axes": ["angular_z"],
    }
)
PLAYABLE_INSTANCE_ID = "thunder_01"
PLAYABLE_CONTROLLER_ID = "thunder_01.thunderv4_locomotion"
PLAYABLE_BASE_TWIST_CHANNEL = "thunder_01.control.base_twist"
PLAYABLE_CONTROL_SOURCE_ID = "robotsimue.local_player.0"
PLAYABLE_SESSION_CAMERA_TAG = "PreviewTarget:south_gate_robot_eye"
PLAYABLE_MIN_FRAME_COUNT = 600
PLAYABLE_DEFAULT_FRAME_CAPTURE_MAX = 2000
PLAYABLE_DEFAULT_FRAME_CAPTURE_WAIT_TIMEOUT_S = 900.0
PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME = "editor_game"
PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE = "packaged_release"
PLAYABLE_ADVANCE_WAIT_S = 0.005
PLAYABLE_PORT_NAMES = frozenset({"visual_snapshot_udp", "control_intent_udp", "control_status_udp"})
PLAYABLE_REQUIRED_BINDINGS = frozenset({"physics", "control", "visual", "sensors"})
PLAYABLE_SENSOR_ROUTES: Mapping[str, tuple[str, str, str, str]] = MappingProxyType(
    {
        "thunder_01.front_depth": (
            "depth",
            "visual",
            "unreal_camera",
            "camera_shm",
        ),
        "thunder_01.front_rgb": (
            "rgb",
            "visual",
            "unreal_camera",
            "camera_shm",
        ),
        "thunder_01.imu": ("imu", "physics", "mujoco_sensor", "typed_dds"),
        "thunder_01.mid360": (
            "mid360",
            "physics",
            "mujoco_livox_model",
            "typed_dds",
        ),
        "thunder_01.truth_odom": (
            "truth_odom",
            "physics",
            "mujoco_truth",
            "typed_dds",
        ),
    }
)
PLAYABLE_SENSOR_RATES: Mapping[str, int] = MappingProxyType(
    {
        "thunder_01.front_depth": 30,
        "thunder_01.front_rgb": 30,
        "thunder_01.imu": 200,
        "thunder_01.mid360": 10,
        "thunder_01.truth_odom": 100,
    }
)

_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_FRAME_RE = re.compile(r"frame_[0-9]{6}\.png\Z")
_ROBOTSIMUE_GAME_WINDOW_TITLE_RE = re.compile(r"^RobotSimUE(?:$|[ (\uFF08])")
_CAPTURE_LOG_RE = re.compile(
    r"LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED "
    r"capture_index=(?P<frame>[0-9]+) "
    r"model_generation=(?P<model>[0-9]+) "
    r"reset_generation=(?P<reset>[0-9]+) "
    r"sequence=(?P<sequence>[0-9]+) "
    r"sim_time_ns=(?P<time>[0-9]+) "
    r"path=(?P<path>.+?) requested=(?P<requested>[0-9]+) max=(?P<max>[0-9]+)"
)
_UE_ORIGIN_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "datagram_sha256",
        "datagram_bytes",
        "successful_send",
    }
)


class PlayableLaunchError(ValueError):
    """The requested launch cannot represent the fixed playable product."""


class PlayableLifecycleError(RuntimeError):
    """One or more resources could not be closed at the playable seam."""


@dataclass(frozen=True)
class PlayableInputAction:
    """One UE keyboard maneuver followed by an explicit neutral interval."""

    maneuver: str
    key: str
    hold_s: float
    neutral_after_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.maneuver, str) or not self.maneuver or self.maneuver != self.maneuver.strip():
            raise ValueError("maneuver must be non-empty trimmed text")
        if not isinstance(self.key, str) or len(self.key) != 1 or self.key != self.key.upper():
            raise ValueError("key must be one uppercase character")
        for name in ("hold_s", "neutral_after_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be finite numeric seconds")
            number = float(value)
            if not math.isfinite(number) or number < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            object.__setattr__(self, name, number)
        if self.hold_s <= 0.0:
            raise ValueError("hold_s must be positive")


PLAYABLE_INPUT_SCHEDULE = (
    PlayableInputAction("forward", "W", 3.0, 0.5),
    PlayableInputAction("backward", "S", 3.0, 0.5),
    PlayableInputAction("left", "A", 3.0, 0.5),
    PlayableInputAction("right", "D", 3.0, 0.5),
    # 5.3 s at the fixed external 0.35 rad/s cap yields ~1.855 rad of
    # commanded yaw while the ThunderV4 controller-local HIM observation is
    # calibrated to the historically proven 0.45 rad/s policy input.
    PlayableInputAction("turn_left", "Q", 5.3, 0.5),
    PlayableInputAction("turn_right", "E", 5.3, 0.5),
)


@dataclass(frozen=True)
class PlayableActionContext:
    """Capability-limited input context; it exposes no controller submitter."""

    run_id: str
    boot_id: str
    session_id: str
    run_dir: Path
    unreal_pid: int
    deadman_key: str
    actions: tuple[PlayableInputAction, ...]
    hud_screenshot_paths: Mapping[str, Path]
    runtime_surface: str
    unreal_log_path: Path
    frame_capture_wait_timeout_s: float


@dataclass(frozen=True)
class PlayableClosedRun:
    """Immutable post-close artifact snapshot passed to qualification."""

    run_id: str
    boot_id: str
    session_id: str
    run_dir: Path
    unreal_log_path: Path
    frames: tuple[Path, ...]
    hud_screenshot_paths: Mapping[str, Path]
    exit_event_id: str | None
    media_toolchain: PinnedPlayableMediaToolchain
    unreal_shutdown: ProcessShutdownSnapshot | None = None
    mujoco_shutdown: ProcessShutdownSnapshot | None = None
    resources_closed: Mapping[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.media_toolchain, PinnedPlayableMediaToolchain):
            raise TypeError("media_toolchain must be a pre-launch pinned toolchain")
        object.__setattr__(self, "unreal_log_path", Path(self.unreal_log_path))


class PlayableControlEvidenceWriter:
    """Append owner-thread control traces to run-local evidence files.

    Only admitted motion and the accepted ``cleared:exit`` zero share the
    qualification input.  Other safety zeros remain audit evidence and cannot
    accidentally masquerade as the unique final zero.
    """

    _SCHEMA_FILES = {
        "lingtu.sim.control-intent-received.v1": "control-intent-received.jsonl",
        "lingtu.sim.control-intent-rejected.v1": "control-intent-rejected.jsonl",
        "lingtu.sim.runtime-request-trace.v1": "runtime-request-trace.jsonl",
        "lingtu.sim.ue-control-status.v1": "control-status-authority.jsonl",
    }

    def __init__(self, run_dir: Path) -> None:
        self._run_dir = Path(run_dir).resolve()
        self._lock = threading.RLock()
        self._streams: dict[str, TextIO] = {}
        self._closed = False

    def __call__(self, record: Mapping[str, Any]) -> None:
        """Validate and durably append one pump-owned trace record."""

        if not isinstance(record, Mapping):
            raise TypeError("playable control trace must be a mapping")
        document = dict(record)
        schema = document.get("schema")
        if schema == "lingtu.sim.control-intent-accepted.v1":
            if document.get("event") != "control_command_accepted":
                raise ValueError("accepted playable control trace event is invalid")
            filename = "control-intent-accepted.jsonl"
        elif schema == "lingtu.sim.control-command-zero.v1":
            if document.get("event") != "control_command_zero":
                raise ValueError("playable zero trace event is invalid")
            if document.get("reason") == "cleared:exit" and document.get("submit_result") == "accepted":
                _validate_correlated_exit_zero(document)
                filename = "control-intent-accepted.jsonl"
            else:
                filename = "control-command-zero-audit.jsonl"
        elif isinstance(schema, str) and schema in self._SCHEMA_FILES:
            filename = self._SCHEMA_FILES[schema]
        else:
            raise ValueError(f"unsupported playable control trace schema: {schema!r}")
        self._append(filename, document)

    def close(self) -> None:
        """Flush and close every evidence stream exactly once."""

        with self._lock:
            if self._closed:
                return
            self._closed = True
            streams = tuple(self._streams.values())
            self._streams.clear()
        errors: list[BaseException] = []
        for stream in streams:
            try:
                stream.flush()
                os.fsync(stream.fileno())
            except BaseException as exc:
                errors.append(exc)
            finally:
                try:
                    stream.close()
                except BaseException as exc:
                    errors.append(exc)
        if errors:
            raise PlayableLifecycleError(
                "playable control evidence close failed: " + "; ".join(str(error) for error in errors)
            ) from errors[0]

    def _append(self, filename: str, document: Mapping[str, Any]) -> None:
        try:
            payload = json.dumps(
                document,
                allow_nan=False,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("playable control trace is not strict JSON") from exc
        with self._lock:
            if self._closed:
                raise PlayableLifecycleError("playable control evidence writer is closed")
            if not self._run_dir.is_dir():
                raise PlayableLifecycleError("playable run directory must exist before control evidence")
            stream = self._streams.get(filename)
            if stream is None:
                path = self._run_dir / filename
                try:
                    stream = path.open("x", encoding="utf-8", newline="\n")
                except FileExistsError as exc:
                    raise PlayableLifecycleError(f"playable control evidence already exists: {path}") from exc
                self._streams[filename] = stream
            stream.write(payload + "\n")
            stream.flush()


@dataclass(frozen=True)
class _TopLevelWindowSnapshot:
    """One immutable Win32 top-level-window observation."""

    hwnd: int
    pid: int
    title: str
    visible: bool
    enabled: bool
    owner_hwnd: int | None
    window_area: int
    client_area: int


@dataclass(frozen=True)
class OwnedRobotSimUEWindowCandidate:
    """Strict JSON-safe observation of one owned-PID top-level window."""

    hwnd: int
    pid: int
    title: str
    visible: bool
    enabled: bool
    owner_hwnd: int | None
    window_area: int
    client_area: int
    eligible: bool

    def __post_init__(self) -> None:
        for name in ("hwnd", "pid"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise PlayableLifecycleError(f"RobotSimUE window candidate {name} must be a positive integer")
        if not isinstance(self.title, str):
            raise PlayableLifecycleError("RobotSimUE window candidate title must be text")
        for name in ("visible", "enabled", "eligible"):
            if not isinstance(getattr(self, name), bool):
                raise PlayableLifecycleError(f"RobotSimUE window candidate {name} must be boolean")
        if self.owner_hwnd is not None and (
            isinstance(self.owner_hwnd, bool) or not isinstance(self.owner_hwnd, int) or self.owner_hwnd <= 0
        ):
            raise PlayableLifecycleError("RobotSimUE window candidate owner_hwnd must be null or positive")
        for name in ("window_area", "client_area"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise PlayableLifecycleError(f"RobotSimUE window candidate {name} must be non-negative")

    def to_dict(self) -> dict[str, Any]:
        """Return a fresh strict-JSON-ready candidate record."""

        return {
            "hwnd": self.hwnd,
            "pid": self.pid,
            "title": self.title,
            "visible": self.visible,
            "enabled": self.enabled,
            "owner_hwnd": self.owner_hwnd,
            "window_area": self.window_area,
            "client_area": self.client_area,
            "eligible": self.eligible,
        }


@dataclass(frozen=True)
class OwnedRobotSimUEWindowPresenceProof:
    """Non-qualification proof that one owned RobotSimUE game window exists."""

    owned_unreal_pid: int
    candidates: tuple[OwnedRobotSimUEWindowCandidate, ...]
    selected_hwnd: int
    selected_pid: int
    observed_at_unix_ns: int
    schema: str = field(
        default="lingtu.sim.owned-robotsimue-window-presence-proof.v1",
        init=False,
    )
    evidence_class: str = field(default="manual_diagnostic_only", init=False)
    qualification: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        for name in (
            "owned_unreal_pid",
            "selected_hwnd",
            "selected_pid",
            "observed_at_unix_ns",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise PlayableLifecycleError(
                    f"RobotSimUE window presence proof {name} must be a positive integer"
                )
        if not isinstance(self.candidates, tuple) or not self.candidates:
            raise PlayableLifecycleError(
                "RobotSimUE window presence proof requires observed candidates"
            )
        if not all(
            isinstance(candidate, OwnedRobotSimUEWindowCandidate)
            for candidate in self.candidates
        ):
            raise PlayableLifecycleError(
                "RobotSimUE window presence proof candidates are invalid"
            )
        if len(set(self.candidates)) != len(self.candidates):
            raise PlayableLifecycleError(
                "RobotSimUE window presence proof candidates must be deduplicated"
            )
        selected = tuple(
            candidate
            for candidate in self.candidates
            if candidate.hwnd == self.selected_hwnd
            and candidate.pid == self.selected_pid
            and candidate.eligible
        )
        if len(selected) != 1 or self.selected_pid != self.owned_unreal_pid:
            raise PlayableLifecycleError(
                "RobotSimUE window presence proof must identify one eligible owned window"
            )

    def to_dict(self) -> dict[str, Any]:
        """Return fresh strict JSON evidence without input-readiness claims."""

        candidates: list[dict[str, Any]] = []
        for candidate in self.candidates:
            document = candidate.to_dict()
            document["title"] = candidate.title if candidate.eligible else None
            document["title_redacted"] = not candidate.eligible
            candidates.append(document)
        return {
            "schema": self.schema,
            "owned_unreal_pid": self.owned_unreal_pid,
            "candidates": candidates,
            "selected_hwnd": self.selected_hwnd,
            "selected_pid": self.selected_pid,
            "observed_at_unix_ns": self.observed_at_unix_ns,
            "evidence_class": self.evidence_class,
            "qualification": self.qualification,
        }


@dataclass(frozen=True)
class OwnedRobotSimUEWindowProof:
    """Non-qualification proof that one owned RobotSimUE window is foreground."""

    owned_unreal_pid: int
    candidates: tuple[OwnedRobotSimUEWindowCandidate, ...]
    selected_hwnd: int
    foreground_hwnd: int
    foreground_pid: int
    observed_at_unix_ns: int
    schema: str = field(
        default="lingtu.sim.owned-robotsimue-window-proof.v1",
        init=False,
    )
    evidence_class: str = field(default="manual_diagnostic_only", init=False)
    qualification: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        for name in (
            "owned_unreal_pid",
            "selected_hwnd",
            "foreground_hwnd",
            "foreground_pid",
            "observed_at_unix_ns",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise PlayableLifecycleError(f"RobotSimUE window proof {name} must be a positive integer")
        if not isinstance(self.candidates, tuple) or not self.candidates:
            raise PlayableLifecycleError("RobotSimUE window proof requires observed candidates")
        if not all(isinstance(candidate, OwnedRobotSimUEWindowCandidate) for candidate in self.candidates):
            raise PlayableLifecycleError("RobotSimUE window proof candidates are invalid")
        selected = tuple(
            candidate for candidate in self.candidates if candidate.hwnd == self.selected_hwnd and candidate.eligible
        )
        if len(selected) != 1:
            raise PlayableLifecycleError("RobotSimUE window proof must identify one eligible selected window")
        if (
            self.foreground_hwnd != self.selected_hwnd
            or self.foreground_pid != self.owned_unreal_pid
            or selected[0].pid != self.owned_unreal_pid
        ):
            raise PlayableLifecycleError("RobotSimUE window proof foreground identity is inconsistent")

    def to_dict(self) -> dict[str, Any]:
        """Return fresh strict JSON evidence without a qualification claim."""

        return {
            "schema": self.schema,
            "owned_unreal_pid": self.owned_unreal_pid,
            "candidates": [candidate.to_dict() for candidate in self.candidates],
            "selected_hwnd": self.selected_hwnd,
            "foreground_hwnd": self.foreground_hwnd,
            "foreground_pid": self.foreground_pid,
            "observed_at_unix_ns": self.observed_at_unix_ns,
            "evidence_class": self.evidence_class,
            "qualification": self.qualification,
        }


def _is_eligible_robotsimue_game_window(
    candidate: _TopLevelWindowSnapshot,
    *,
    expected_pid: int,
) -> bool:
    if not isinstance(candidate, _TopLevelWindowSnapshot):
        return False
    if candidate.pid != expected_pid or candidate.hwnd <= 0:
        return False
    if not candidate.visible or not candidate.enabled or candidate.owner_hwnd is not None:
        return False
    if candidate.window_area <= 0 or candidate.client_area <= 0:
        return False
    normalized_title = _normalized_robotsimue_game_title(candidate.title)
    if normalized_title is None:
        return False
    folded_title = normalized_title.casefold()
    if "/" in normalized_title or "\\" in normalized_title or ".exe" in folded_title:
        return False
    return _ROBOTSIMUE_GAME_WINDOW_TITLE_RE.match(normalized_title) is not None


def _normalized_robotsimue_game_title(title: str) -> str | None:
    """Allow UE's trailing ASCII spaces, never leading/control whitespace."""

    if not title or title.startswith(" "):
        return None
    normalized = title.rstrip(" ")
    if not normalized or _has_disallowed_window_title_character(normalized):
        return None
    return normalized


def _has_disallowed_window_title_character(title: str) -> bool:
    return any(
        character != " "
        and (
            character.isspace()
            or unicodedata.category(character).startswith("C")
        )
        for character in title
    )


def _window_rejection_counts(
    candidates: Sequence[OwnedRobotSimUEWindowCandidate],
    *,
    expected_pid: int,
) -> str:
    labels = (
        "pid_mismatch",
        "not_visible",
        "not_enabled",
        "owned_window",
        "window_area_zero",
        "client_area_zero",
        "title_leading_whitespace",
        "title_control_or_nonspace_whitespace",
        "title_path_or_executable",
        "title_mismatch",
    )
    counts = {label: 0 for label in labels}
    for candidate in candidates:
        if candidate.eligible:
            continue
        if candidate.pid != expected_pid:
            counts["pid_mismatch"] += 1
        if not candidate.visible:
            counts["not_visible"] += 1
        if not candidate.enabled:
            counts["not_enabled"] += 1
        if candidate.owner_hwnd is not None:
            counts["owned_window"] += 1
        if candidate.window_area <= 0:
            counts["window_area_zero"] += 1
        if candidate.client_area <= 0:
            counts["client_area_zero"] += 1
        normalized_title = candidate.title.rstrip(" ")
        if candidate.title != candidate.title.lstrip():
            counts["title_leading_whitespace"] += 1
        if _has_disallowed_window_title_character(normalized_title):
            counts["title_control_or_nonspace_whitespace"] += 1
        folded_title = normalized_title.casefold()
        if "/" in normalized_title or "\\" in normalized_title or ".exe" in folded_title:
            counts["title_path_or_executable"] += 1
        if _ROBOTSIMUE_GAME_WINDOW_TITLE_RE.match(normalized_title) is None:
            counts["title_mismatch"] += 1
    populated = tuple(f"{label}={counts[label]}" for label in labels if counts[label])
    return ",".join(populated) if populated else "none"


class PlayableWindowsInputBackend(Protocol):
    """Minimal Win32 surface used by the owned-window SendInput driver."""

    def top_level_windows(
        self,
        pid: int,
        *,
        title_fragment: str,
    ) -> Sequence[_TopLevelWindowSnapshot]: ...

    def focus_window(self, hwnd: int) -> None: ...

    def foreground_window(self) -> int | None: ...

    def window_process_id(self, hwnd: int) -> int | None: ...

    def send_key(self, key: str, *, pressed: bool) -> None: ...


class PlayableInputDriver(Protocol):
    """Production-shaped UE input driver; tests may inject a fake."""

    def perform_actions(self, context: PlayableActionContext) -> None: ...

    def request_exit(self, context: PlayableActionContext) -> None: ...


@dataclass(frozen=True)
class _OwnedRobotSimUEWindowSelection:
    candidates: tuple[OwnedRobotSimUEWindowCandidate, ...]
    selected: OwnedRobotSimUEWindowCandidate
    deadline: float


def prove_owned_robotsimue_window_presence(
    unreal_pid: int,
    *,
    backend: PlayableWindowsInputBackend | None = None,
    sleep: Callable[[float], object] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    timeout_s: float = 1.0,
) -> OwnedRobotSimUEWindowPresenceProof:
    """Select one owned game window without foreground or input operations.

    The short bound keeps the mandatory post-probe observation from extending
    beyond the manual profile deadline while still tolerating a late Win32
    enumeration refresh.
    """

    input_backend, timeout = _window_proof_dependencies(
        unreal_pid,
        backend=backend,
        sleep=sleep,
        monotonic=monotonic,
        timeout_s=timeout_s,
    )
    selection = _select_owned_robotsimue_game_window(
        unreal_pid,
        input_backend=input_backend,
        sleep=sleep,
        monotonic=monotonic,
        timeout_s=timeout,
    )
    return OwnedRobotSimUEWindowPresenceProof(
        owned_unreal_pid=unreal_pid,
        candidates=selection.candidates,
        selected_hwnd=selection.selected.hwnd,
        selected_pid=selection.selected.pid,
        observed_at_unix_ns=time.time_ns(),
    )


def prove_owned_robotsimue_foreground(
    unreal_pid: int,
    *,
    backend: PlayableWindowsInputBackend | None = None,
    sleep: Callable[[float], object] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    timeout_s: float = 10.0,
) -> OwnedRobotSimUEWindowProof:
    """Select and foreground one owned game window without sending input."""

    input_backend, focus_timeout_s = _window_proof_dependencies(
        unreal_pid,
        backend=backend,
        sleep=sleep,
        monotonic=monotonic,
        timeout_s=timeout_s,
    )
    selection = _select_owned_robotsimue_game_window(
        unreal_pid,
        input_backend=input_backend,
        sleep=sleep,
        monotonic=monotonic,
        timeout_s=focus_timeout_s,
    )
    selected = selection.selected
    deadline = selection.deadline

    last_focus_error: Exception | None = None
    while True:
        try:
            input_backend.focus_window(selected.hwnd)
            foreground = input_backend.foreground_window()
            if foreground is None:
                raise PlayableLifecycleError("RobotSimUE foreground query returned no window")
            if (
                isinstance(foreground, bool)
                or not isinstance(foreground, int)
                or foreground <= 0
                or foreground != selected.hwnd
            ):
                raise PlayableLifecycleError("RobotSimUE target is not the owned foreground RobotSimUE window")
            foreground_pid = input_backend.window_process_id(foreground)
            if foreground_pid != unreal_pid:
                raise PlayableLifecycleError("RobotSimUE foreground window PID does not match owned process")
            return OwnedRobotSimUEWindowProof(
                owned_unreal_pid=unreal_pid,
                candidates=selection.candidates,
                selected_hwnd=selected.hwnd,
                foreground_hwnd=foreground,
                foreground_pid=foreground_pid,
                observed_at_unix_ns=time.time_ns(),
            )
        except Exception as exc:
            last_focus_error = exc

        now = _finite_clock_value(monotonic(), "monotonic")
        if now >= deadline:
            raise PlayableLifecycleError(
                f"timed out focusing owned RobotSimUE game window: {last_focus_error}"
            ) from last_focus_error
        sleep(min(0.05, deadline - now))


def _window_proof_dependencies(
    unreal_pid: int,
    *,
    backend: PlayableWindowsInputBackend | None,
    sleep: Callable[[float], object],
    monotonic: Callable[[], float],
    timeout_s: float,
) -> tuple[PlayableWindowsInputBackend, float]:
    if isinstance(unreal_pid, bool) or not isinstance(unreal_pid, int) or unreal_pid <= 0:
        raise ValueError("unreal_pid must be a positive integer")
    if not callable(sleep) or not callable(monotonic):
        raise TypeError("sleep and monotonic must be callable")
    normalized_timeout_s = _positive_finite_seconds(timeout_s, "timeout_s")
    input_backend = backend if backend is not None else _User32SendInputBackend()
    return input_backend, normalized_timeout_s


def _select_owned_robotsimue_game_window(
    unreal_pid: int,
    *,
    input_backend: PlayableWindowsInputBackend,
    sleep: Callable[[float], object],
    monotonic: Callable[[], float],
    timeout_s: float,
) -> _OwnedRobotSimUEWindowSelection:
    deadline = _finite_clock_value(monotonic(), "monotonic") + timeout_s

    observed_candidates: list[OwnedRobotSimUEWindowCandidate] = []
    selected: OwnedRobotSimUEWindowCandidate | None = None
    while True:
        snapshots = tuple(
            input_backend.top_level_windows(
                unreal_pid,
                title_fragment="RobotSimUE",
            )
        )
        current_candidates = tuple(
            _window_candidate_evidence(snapshot, expected_pid=unreal_pid) for snapshot in snapshots
        )
        for candidate in current_candidates:
            if candidate not in observed_candidates:
                observed_candidates.append(candidate)
        eligible = tuple(candidate for candidate in current_candidates if candidate.eligible)
        if eligible:
            largest_area = max(candidate.client_area for candidate in eligible)
            largest = tuple(candidate for candidate in eligible if candidate.client_area == largest_area)
            if len(largest) != 1:
                raise PlayableLifecycleError(
                    "owned RobotSimUE PID has multiple equally large eligible game client areas"
                )
            selected = largest[0]
            break
        now = _finite_clock_value(monotonic(), "monotonic")
        if now >= deadline:
            raise PlayableLifecycleError(
                "owned RobotSimUE top-level window was not found; "
                f"observed_candidates={len(observed_candidates)}; "
                "rejection_counts="
                + _window_rejection_counts(
                    tuple(observed_candidates),
                    expected_pid=unreal_pid,
                )
            )
        sleep(min(0.05, deadline - now))

    if input_backend.window_process_id(selected.hwnd) != unreal_pid:
        raise PlayableLifecycleError("RobotSimUE window PID does not match owned process")
    return _OwnedRobotSimUEWindowSelection(
        candidates=tuple(observed_candidates),
        selected=selected,
        deadline=deadline,
    )


def _window_candidate_evidence(
    snapshot: _TopLevelWindowSnapshot,
    *,
    expected_pid: int,
) -> OwnedRobotSimUEWindowCandidate:
    if not isinstance(snapshot, _TopLevelWindowSnapshot):
        raise PlayableLifecycleError("RobotSimUE window enumeration returned an invalid candidate")
    return OwnedRobotSimUEWindowCandidate(
        hwnd=snapshot.hwnd,
        pid=snapshot.pid,
        title=snapshot.title,
        visible=snapshot.visible,
        enabled=snapshot.enabled,
        owner_hwnd=snapshot.owner_hwnd,
        window_area=snapshot.window_area,
        client_area=snapshot.client_area,
        eligible=_is_eligible_robotsimue_game_window(
            snapshot,
            expected_pid=expected_pid,
        ),
    )


class OwnedRobotSimUEInput:
    """Target dependency-free ``user32.SendInput`` at one owned UE window."""

    def __init__(
        self,
        *,
        backend: PlayableWindowsInputBackend | None = None,
        sleep: Callable[[float], object] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        focus_timeout_s: float = 10.0,
        key_tap_s: float = 0.05,
        mode_settle_s: float = 0.75,
        evidence_timeout_s: float = 15.0,
    ) -> None:
        self._backend = backend or _User32SendInputBackend()
        if not callable(sleep) or not callable(monotonic):
            raise TypeError("sleep and monotonic must be callable")
        self._sleep = sleep
        self._monotonic = monotonic
        self._focus_timeout_s = _positive_finite_seconds(
            focus_timeout_s,
            "focus_timeout_s",
        )
        self._key_tap_s = _positive_finite_seconds(key_tap_s, "key_tap_s")
        self._mode_settle_s = _positive_finite_seconds(
            mode_settle_s,
            "mode_settle_s",
        )
        self._evidence_timeout_s = _positive_finite_seconds(
            evidence_timeout_s,
            "evidence_timeout_s",
        )
        self._hwnd: int | None = None
        self._pid: int | None = None
        self._pressed: list[str] = []
        self._ready_for_exit_after_sequence: int | None = None

    def perform_actions(self, context: PlayableActionContext) -> None:
        """Record and drive the fixed schedule through the owned UE viewport."""

        self._bind_owned_window(context)
        if context.actions != PLAYABLE_INPUT_SCHEDULE or context.deadman_key != "SHIFT":
            raise PlayableLifecycleError("playable input context changed its fixed schedule")
        self._ready_for_exit_after_sequence = None
        try:
            initial_status = self._wait_for_status(
                context,
                lambda status: (
                    _status_runtime_state(status) == "RUNNING"
                    and _status_recording_state(status) == "idle"
                    and _status_ui_mode(status) == "drive"
                    and _status_camera_mode(status) in {"follow", "inspection", "free"}
                ),
                description="RUNNING/idle Drive authority",
            )
            self._tap("R")
            recording_status = self._wait_for_status(
                context,
                lambda status: _status_recording_state(status) == "recording",
                after_sequence=_status_server_sequence(initial_status),
                description="record_start acceptance",
            )
            recording_start_sim_time_ns = _status_sim_time_ns(recording_status)
            for action in context.actions:
                self._key_down(context.deadman_key)
                self._key_down(action.key)
                self._sleep(action.hold_s)
                self._key_up(action.key)
                self._key_up(context.deadman_key)
                self._sleep(action.neutral_after_s)

            # Mode transitions are real UE input.  UE owns screenshot timing and
            # evidence; the runner never captures the desktop as a substitute.
            self._wait_for_hud_capture(context, "drive")
            self._tap("C")
            camera_status = self._wait_for_status(
                context,
                lambda status: (
                    _status_camera_mode(status) in {"follow", "inspection", "free"}
                    and _status_camera_mode(status) != _status_camera_mode(initial_status)
                ),
                after_sequence=_status_server_sequence(initial_status),
                description="camera mode transition",
            )
            self._tap("TAB")
            tactical_status = self._wait_for_status(
                context,
                lambda status: _status_ui_mode(status) == "tactical" and _status_recording_state(status) == "recording",
                after_sequence=_status_server_sequence(camera_status),
                description="Tactical status",
            )
            self._wait_for_hud_capture(context, "tactical")
            self._tap("TAB")
            self._tap("ESCAPE")
            self._wait_for_status(
                context,
                lambda status: _status_ui_mode(status) == "menu" and _status_recording_state(status) == "recording",
                after_sequence=_status_server_sequence(tactical_status),
                description="Menu recording status",
            )
            self._wait_for_hud_capture(context, "menu_recording")
            self._tap("ESCAPE")
            self._wait_for_recording_frame_window(
                context,
                recording_start_sim_time_ns=recording_start_sim_time_ns,
            )
            self._tap("R")
            committed_status = self._wait_for_status(
                context,
                lambda status: _status_recording_state(status) == "committed",
                description="record_stop_commit acceptance",
            )
            self._ready_for_exit_after_sequence = _status_server_sequence(committed_status)
            self._wait_for_file(context.run_dir / "simulation-recording.json")
        except BaseException as exc:
            self._release_all(exc)
            raise
        self._release_all(None)

    def _wait_for_status(
        self,
        context: PlayableActionContext,
        predicate: Callable[[Mapping[str, Any]], bool],
        *,
        description: str,
        after_sequence: int = 0,
    ) -> Mapping[str, Any]:
        path = context.run_dir / "control-status-authority.jsonl"
        deadline = _finite_clock_value(self._monotonic(), "monotonic") + self._evidence_timeout_s
        while True:
            if path.is_file():
                try:
                    payload = path.read_bytes()
                    if payload.endswith(b"\n"):
                        records = _strict_status_records(payload, context)
                        for record in reversed(records):
                            if _status_server_sequence(record) > after_sequence and predicate(record):
                                return record
                except (OSError, UnicodeError, ValueError):
                    pass
            now = _finite_clock_value(self._monotonic(), "monotonic")
            if now >= deadline:
                raise PlayableLifecycleError(f"timed out waiting for authoritative {description}")
            self._sleep(min(0.02, deadline - now))

    def _wait_for_hud_capture(
        self,
        context: PlayableActionContext,
        key: str,
    ) -> None:
        try:
            screenshot = context.hud_screenshot_paths[key]
        except KeyError as exc:
            raise PlayableLifecycleError(f"HUD capture path is missing for {key}") from exc
        self._wait_for_file(screenshot)
        self._wait_for_file(screenshot.with_suffix(".evidence.json"))

    def _wait_for_file(self, path: Path) -> None:
        deadline = _finite_clock_value(self._monotonic(), "monotonic") + self._evidence_timeout_s
        while True:
            try:
                if path.is_file() and path.stat().st_size > 0:
                    return
            except OSError:
                pass
            now = _finite_clock_value(self._monotonic(), "monotonic")
            if now >= deadline:
                raise PlayableLifecycleError(f"timed out waiting for runtime artifact: {path.name}")
            self._sleep(min(0.02, deadline - now))

    def _wait_for_recording_frame_window(
        self,
        context: PlayableActionContext,
        *,
        recording_start_sim_time_ns: int,
    ) -> None:
        deadline = _finite_clock_value(self._monotonic(), "monotonic") + context.frame_capture_wait_timeout_s
        last_error = "no capture log yet"
        while True:
            try:
                status = self._latest_current_recording_status(context)
                if not _status_has_current_five_streams(status):
                    last_error = "five sensor streams are not current and ACTIVE"
                else:
                    count = _count_mapped_recording_frames(
                        context,
                        recording_start_sim_time_ns=recording_start_sim_time_ns,
                    )
                    if count >= PLAYABLE_MIN_FRAME_COUNT:
                        return
                    last_error = f"only {count}/{PLAYABLE_MIN_FRAME_COUNT} recording-window UE frames are mapped"
            except (OSError, UnicodeError, ValueError) as exc:
                last_error = str(exc)
            now = _finite_clock_value(self._monotonic(), "monotonic")
            if now >= deadline:
                raise PlayableLifecycleError(
                    f"timed out waiting for 600 recording-window UE frames before record_stop_commit: {last_error}"
                )
            self._sleep(min(1.0, deadline - now))

    def _latest_current_recording_status(
        self,
        context: PlayableActionContext,
    ) -> Mapping[str, Any]:
        path = context.run_dir / "control-status-authority.jsonl"
        payload = path.read_bytes()
        if not payload.endswith(b"\n"):
            raise ValueError("control status authority is partial")
        for record in reversed(_strict_status_records(payload, context)):
            if _status_runtime_state(record) == "RUNNING" and _status_recording_state(record) == "recording":
                return record
        raise ValueError("no current recording status is available")

    def request_exit(self, context: PlayableActionContext) -> None:
        """Open UE Menu and invoke its product-visible Exit action."""

        self._bind_owned_window(context)
        after_sequence = self._ready_for_exit_after_sequence
        if after_sequence is None:
            raise PlayableLifecycleError("UE Menu exit is unavailable before the fixed recording schedule commits")
        try:
            self._tap("ESCAPE")
            self._sleep(self._mode_settle_s)
            self._wait_for_status(
                context,
                lambda status: (
                    _status_runtime_state(status) == "RUNNING"
                    and _status_recording_state(status) == "committed"
                    and _status_ui_mode(status) == "menu"
                ),
                after_sequence=after_sequence,
                description="committed Menu exit status",
            )
            self._tap("X")
            self._ready_for_exit_after_sequence = None
        except BaseException as exc:
            self._release_all(exc)
            raise
        self._release_all(None)

    def _bind_owned_window(self, context: PlayableActionContext) -> None:
        if not isinstance(context, PlayableActionContext):
            raise TypeError("context must be PlayableActionContext")
        proof = prove_owned_robotsimue_foreground(
            context.unreal_pid,
            backend=self._backend,
            sleep=self._sleep,
            monotonic=self._monotonic,
            timeout_s=self._focus_timeout_s,
        )
        self._hwnd = proof.selected_hwnd
        self._pid = proof.owned_unreal_pid

    def _require_owned_foreground(self) -> None:
        hwnd = self._hwnd
        pid = self._pid
        if hwnd is None or pid is None:
            raise PlayableLifecycleError("RobotSimUE input window is not bound")
        foreground = self._backend.foreground_window()
        if foreground is None:
            raise PlayableLifecycleError("RobotSimUE foreground query returned no window")
        if (
            isinstance(foreground, bool)
            or not isinstance(foreground, int)
            or foreground <= 0
            or foreground != hwnd
            or self._backend.window_process_id(hwnd) != pid
        ):
            raise PlayableLifecycleError("SendInput target is not the owned foreground RobotSimUE window")

    def _tap(self, key: str) -> None:
        self._key_down(key)
        try:
            self._sleep(self._key_tap_s)
        finally:
            self._key_up(key)

    def _key_down(self, key: str) -> None:
        self._require_owned_foreground()
        if key in self._pressed:
            raise PlayableLifecycleError(f"playable key is already pressed: {key}")
        self._backend.send_key(key, pressed=True)
        self._pressed.append(key)

    def _key_up(self, key: str) -> None:
        if key not in self._pressed:
            return
        self._require_owned_foreground()
        self._backend.send_key(key, pressed=False)
        self._pressed.remove(key)

    def _release_all(self, original: BaseException | None) -> None:
        errors: list[BaseException] = []
        for key in reversed(tuple(self._pressed)):
            try:
                self._key_up(key)
            except BaseException as exc:
                errors.append(exc)
        if not errors:
            return
        if original is not None:
            for error in errors:
                _add_exception_note(
                    original,
                    f"SendInput key release failed: {type(error).__name__}: {error}",
                )
            return
        raise PlayableLifecycleError(
            "SendInput key release failed: " + "; ".join(str(error) for error in errors)
        ) from errors[0]


class _User32SendInputBackend:
    """Small ctypes adapter over the exact Win32 APIs used by the runner."""

    _VIRTUAL_KEYS = {
        "SHIFT": 0x10,
        "TAB": 0x09,
        "ESCAPE": 0x1B,
        "W": 0x57,
        "S": 0x53,
        "A": 0x41,
        "D": 0x44,
        "Q": 0x51,
        "E": 0x45,
        "C": 0x43,
        "R": 0x52,
        "X": 0x58,
    }

    def __init__(self) -> None:
        if os.name != "nt":
            raise PlayableLifecycleError("RobotSimUE SendInput qualification requires Windows")
        import ctypes
        from ctypes import wintypes

        self._ctypes = ctypes
        self._wintypes = wintypes
        self._user32 = ctypes.WinDLL("user32", use_last_error=True)
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._enum_callback_type = ctypes.WINFUNCTYPE(
            wintypes.BOOL,
            wintypes.HWND,
            wintypes.LPARAM,
        )

        ulong_ptr = ctypes.c_ulonglong if ctypes.sizeof(ctypes.c_void_p) == 8 else ctypes.c_ulong

        class KeybdInput(ctypes.Structure):
            _fields_ = (
                ("wVk", wintypes.WORD),
                ("wScan", wintypes.WORD),
                ("dwFlags", wintypes.DWORD),
                ("time", wintypes.DWORD),
                ("dwExtraInfo", ulong_ptr),
            )

        class MouseInput(ctypes.Structure):
            _fields_ = (
                ("dx", wintypes.LONG),
                ("dy", wintypes.LONG),
                ("mouseData", wintypes.DWORD),
                ("dwFlags", wintypes.DWORD),
                ("time", wintypes.DWORD),
                ("dwExtraInfo", ulong_ptr),
            )

        class HardwareInput(ctypes.Structure):
            _fields_ = (
                ("uMsg", wintypes.DWORD),
                ("wParamL", wintypes.WORD),
                ("wParamH", wintypes.WORD),
            )

        class InputUnion(ctypes.Union):
            # INPUT's native ABI is sized by the largest union member.  Keeping
            # only KEYBDINPUT produces a 32-byte x64 record; SendInput requires
            # the real 40-byte INPUT layout even for keyboard events.
            _fields_ = (
                ("mi", MouseInput),
                ("ki", KeybdInput),
                ("hi", HardwareInput),
            )

        class Input(ctypes.Structure):
            _anonymous_ = ("payload",)
            _fields_ = (("type", wintypes.DWORD), ("payload", InputUnion))

        class Rect(ctypes.Structure):
            _fields_ = (
                ("left", wintypes.LONG),
                ("top", wintypes.LONG),
                ("right", wintypes.LONG),
                ("bottom", wintypes.LONG),
            )

        self._keybd_input_type = KeybdInput
        self._input_union_type = InputUnion
        self._input_type = Input
        self._rect_type = Rect
        self._user32.EnumWindows.argtypes = (
            self._enum_callback_type,
            wintypes.LPARAM,
        )
        self._user32.EnumWindows.restype = wintypes.BOOL
        self._user32.IsWindowVisible.argtypes = (wintypes.HWND,)
        self._user32.IsWindowVisible.restype = wintypes.BOOL
        self._user32.IsWindowEnabled.argtypes = (wintypes.HWND,)
        self._user32.IsWindowEnabled.restype = wintypes.BOOL
        self._user32.GetWindow.argtypes = (wintypes.HWND, wintypes.UINT)
        self._user32.GetWindow.restype = wintypes.HWND
        self._user32.GetWindowRect.argtypes = (
            wintypes.HWND,
            ctypes.POINTER(Rect),
        )
        self._user32.GetWindowRect.restype = wintypes.BOOL
        self._user32.GetClientRect.argtypes = (
            wintypes.HWND,
            ctypes.POINTER(Rect),
        )
        self._user32.GetClientRect.restype = wintypes.BOOL
        self._user32.GetWindowTextLengthW.argtypes = (wintypes.HWND,)
        self._user32.GetWindowTextLengthW.restype = ctypes.c_int
        self._user32.GetWindowTextW.argtypes = (
            wintypes.HWND,
            wintypes.LPWSTR,
            ctypes.c_int,
        )
        self._user32.GetWindowTextW.restype = ctypes.c_int
        self._user32.ShowWindow.argtypes = (wintypes.HWND, ctypes.c_int)
        self._user32.ShowWindow.restype = wintypes.BOOL
        self._user32.SetForegroundWindow.argtypes = (wintypes.HWND,)
        self._user32.SetForegroundWindow.restype = wintypes.BOOL
        self._user32.BringWindowToTop.argtypes = (wintypes.HWND,)
        self._user32.BringWindowToTop.restype = wintypes.BOOL
        self._user32.SetFocus.argtypes = (wintypes.HWND,)
        self._user32.SetFocus.restype = wintypes.HWND
        self._user32.GetForegroundWindow.argtypes = ()
        self._user32.GetForegroundWindow.restype = wintypes.HWND
        self._user32.GetWindowThreadProcessId.argtypes = (
            wintypes.HWND,
            ctypes.POINTER(wintypes.DWORD),
        )
        self._user32.GetWindowThreadProcessId.restype = wintypes.DWORD
        self._user32.AttachThreadInput.argtypes = (
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.BOOL,
        )
        self._user32.AttachThreadInput.restype = wintypes.BOOL
        self._kernel32.GetCurrentThreadId.argtypes = ()
        self._kernel32.GetCurrentThreadId.restype = wintypes.DWORD
        self._user32.SendInput.argtypes = (
            wintypes.UINT,
            ctypes.POINTER(Input),
            ctypes.c_int,
        )
        self._user32.SendInput.restype = wintypes.UINT

    def top_level_windows(
        self,
        pid: int,
        *,
        title_fragment: str,
    ) -> Sequence[_TopLevelWindowSnapshot]:
        """Snapshot candidate top-level windows for one owned process."""

        windows: list[_TopLevelWindowSnapshot] = []
        callback_failure: BaseException | None = None

        def visit(hwnd: Any, _lparam: Any) -> bool:
            nonlocal callback_failure
            try:
                handle = self._handle_value(hwnd)
                if handle is None:
                    return True
                process_id = self.window_process_id(handle)
                if process_id != pid:
                    return True
                length = int(self._user32.GetWindowTextLengthW(hwnd))
                if length <= 0:
                    return True
                buffer = self._ctypes.create_unicode_buffer(length + 1)
                self._user32.GetWindowTextW(hwnd, buffer, length + 1)
                if title_fragment.casefold() not in buffer.value.casefold():
                    return True
                owner = self._user32.GetWindow(hwnd, 4)  # GW_OWNER
                windows.append(
                    _TopLevelWindowSnapshot(
                        hwnd=handle,
                        pid=process_id,
                        title=buffer.value,
                        visible=bool(self._user32.IsWindowVisible(hwnd)),
                        enabled=bool(self._user32.IsWindowEnabled(hwnd)),
                        owner_hwnd=self._handle_value(owner),
                        window_area=self._window_area(hwnd, self._user32.GetWindowRect),
                        client_area=self._window_area(hwnd, self._user32.GetClientRect),
                    )
                )
                return True
            except BaseException as exc:
                # ctypes callback exceptions otherwise escape only through an
                # unraisable hook and EnumWindows may look like an empty scan.
                callback_failure = exc
                return False

        callback = self._enum_callback_type(visit)
        enumeration_succeeded = bool(self._user32.EnumWindows(callback, 0))
        if callback_failure is not None:
            raise PlayableLifecycleError(
                "RobotSimUE window enumeration callback failed"
            ) from callback_failure
        if not enumeration_succeeded:
            raise self._ctypes.WinError(self._ctypes.get_last_error())
        return tuple(windows)

    @staticmethod
    def _handle_value(hwnd: Any) -> int | None:
        raw_value = getattr(hwnd, "value", hwnd)
        if raw_value is None:
            return None
        try:
            value = int(raw_value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise PlayableLifecycleError("Win32 returned an invalid window handle") from exc
        return value if value > 0 else None

    def _window_area(self, hwnd: Any, getter: Callable[[Any, Any], Any]) -> int:
        rect = self._rect_type()
        if not getter(hwnd, self._ctypes.byref(rect)):
            return 0
        width = int(rect.right) - int(rect.left)
        height = int(rect.bottom) - int(rect.top)
        return width * height if width > 0 and height > 0 else 0

    def focus_window(self, hwnd: int) -> None:
        """Attach input queues, focus one HWND, then always detach."""

        target = self._wintypes.HWND(hwnd)
        foreground = self._user32.GetForegroundWindow()
        foreground_thread = self._window_thread_id(foreground) if foreground else None
        target_thread = self._window_thread_id(target)
        current_thread = int(self._kernel32.GetCurrentThreadId())
        if current_thread <= 0:
            raise PlayableLifecycleError("failed to focus RobotSimUE window: current thread ID is unavailable")

        attached_threads: list[int] = []
        rejected_attach_threads: list[int] = []
        operation_error: BaseException | None = None
        detach_errors: list[BaseException] = []
        try:
            for thread_id in (foreground_thread, target_thread):
                if thread_id is None or thread_id == current_thread or thread_id in attached_threads:
                    continue
                if self._user32.AttachThreadInput(
                    current_thread,
                    thread_id,
                    True,
                ):
                    attached_threads.append(thread_id)
                else:
                    rejected_attach_threads.append(thread_id)
            self._user32.ShowWindow(target, 9)  # SW_RESTORE
            self._user32.BringWindowToTop(target)
            self._user32.SetForegroundWindow(target)
            self._user32.SetFocus(target)
        except BaseException as exc:
            operation_error = exc
        finally:
            for thread_id in reversed(attached_threads):
                try:
                    if not self._user32.AttachThreadInput(
                        current_thread,
                        thread_id,
                        False,
                    ):
                        detach_errors.append(RuntimeError(f"AttachThreadInput detach failed for thread {thread_id}"))
                except BaseException as exc:
                    detach_errors.append(exc)

        if operation_error is not None or detach_errors:
            detail = ""
            if detach_errors:
                detail = f"; {len(detach_errors)} input-queue detach operation(s) failed"
            raise PlayableLifecycleError(f"failed to focus RobotSimUE window{detail}") from (
                operation_error or detach_errors[0]
            )
        if rejected_attach_threads:
            foreground_after = self.foreground_window()
            if foreground_after != hwnd:
                rejected = ", ".join(str(thread_id) for thread_id in rejected_attach_threads)
                observed = "null" if foreground_after is None else str(foreground_after)
                raise PlayableLifecycleError(
                    f"AttachThreadInput failed for thread IDs {rejected}; foreground={observed}"
                )

    def _window_thread_id(self, hwnd: Any) -> int:
        process_id = self._wintypes.DWORD()
        thread_id = int(
            self._user32.GetWindowThreadProcessId(
                hwnd,
                self._ctypes.byref(process_id),
            )
        )
        if thread_id <= 0:
            raise PlayableLifecycleError("failed to focus RobotSimUE window: window thread ID is unavailable")
        return thread_id

    def foreground_window(self) -> int | None:
        """Return the current foreground HWND."""

        hwnd = self._user32.GetForegroundWindow()
        return self._handle_value(hwnd)

    def window_process_id(self, hwnd: int) -> int | None:
        """Return the process ID owning one HWND."""

        process_id = self._wintypes.DWORD()
        self._user32.GetWindowThreadProcessId(
            self._wintypes.HWND(hwnd),
            self._ctypes.byref(process_id),
        )
        return int(process_id.value) if process_id.value else None

    def send_key(self, key: str, *, pressed: bool) -> None:
        """Inject one keyboard transition with ``user32.SendInput``."""

        virtual_key = self._VIRTUAL_KEYS.get(key)
        if virtual_key is None:
            raise PlayableLifecycleError(f"unsupported playable SendInput key: {key}")
        flags = 0 if pressed else 0x0002  # KEYEVENTF_KEYUP
        record = self._input_type(
            type=1,  # INPUT_KEYBOARD
            payload=self._input_union_type(
                ki=self._keybd_input_type(
                    wVk=virtual_key,
                    wScan=0,
                    dwFlags=flags,
                    time=0,
                    dwExtraInfo=0,
                )
            ),
        )
        sent = self._user32.SendInput(
            1,
            self._ctypes.byref(record),
            self._ctypes.sizeof(self._input_type),
        )
        if sent != 1:
            raise self._ctypes.WinError(self._ctypes.get_last_error())


@dataclass(frozen=True)
class PlayableRuntimeConfig:
    """Trusted process paths and run-local values for one playable launch."""

    repo_root: Path
    run_root: Path
    mujoco_host: Path
    ffmpeg: Path
    ffprobe: Path
    media_toolchain: PinnedPlayableMediaToolchain
    ports: Mapping[str, int]
    runtime_surface: str = PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME
    unreal_editor_executable: Path | None = None
    unreal_project: Path | None = None
    robotsimue_executable: Path | None = None
    imu_publisher: Path | None = None
    truth_odom_publisher: Path | None = None
    mid360_publisher: Path | None = None
    truth_odom_parent_frame: str = "map"
    dds_domain: int = 0
    run_id: str | None = None
    boot_id: str | None = None
    warmup_steps: int = 1
    ready_timeout_s: float = 600.0
    sleep_s: float = 0.01
    frame_capture_max: int = PLAYABLE_DEFAULT_FRAME_CAPTURE_MAX
    frame_capture_wait_timeout_s: float = PLAYABLE_DEFAULT_FRAME_CAPTURE_WAIT_TIMEOUT_S
    depth_capture_in_main_renderer: bool = False
    shared_color_depth_capture: bool = False
    main_view_screen_percentage: int = 100
    windows_cpu_isolation: WindowsCpuIsolationConfig | None = None

    def __post_init__(self) -> None:
        for name in (
            "repo_root",
            "run_root",
            "mujoco_host",
            "ffmpeg",
            "ffprobe",
        ):
            object.__setattr__(self, name, Path(getattr(self, name)).resolve())
        for name in (
            "unreal_editor_executable",
            "unreal_project",
            "robotsimue_executable",
            "imu_publisher",
            "truth_odom_publisher",
            "mid360_publisher",
        ):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, Path(value).resolve())
        if self.runtime_surface not in {
            PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
            PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE,
        }:
            raise PlayableLaunchError("runtime_surface must be editor_game or packaged_release")
        if self.runtime_surface == PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME:
            if self.unreal_editor_executable is None:
                raise PlayableLaunchError("editor_game runtime requires unreal_editor_executable")
            if self.unreal_editor_executable.name.casefold() != "unrealeditor.exe":
                raise PlayableLaunchError("unreal_editor_executable must be UnrealEditor.exe")
            if self.unreal_project is None:
                raise PlayableLaunchError("editor_game runtime requires unreal_project")
            if self.unreal_project.name.casefold() != "robotsimue.uproject":
                raise PlayableLaunchError("unreal_project must be RobotSimUE.uproject")
        else:
            if self.robotsimue_executable is None:
                raise PlayableLaunchError("packaged_release runtime requires robotsimue_executable")
            if self.robotsimue_executable.name.casefold() != "robotsimue-win64-release.exe":
                raise PlayableLaunchError(
                    "robotsimue_executable must be the process-owned packaged RobotSimUE-Win64-Release.exe"
                )
        object.__setattr__(self, "ports", MappingProxyType(_validated_ports(self.ports)))
        if not isinstance(self.media_toolchain, PinnedPlayableMediaToolchain):
            raise TypeError("media_toolchain must be a pre-launch pinned toolchain")
        pinned_paths = getattr(self.media_toolchain, "paths", None)
        if not isinstance(pinned_paths, Mapping) or any(
            pinned_paths.get(name) != getattr(self, name) for name in ("ffmpeg", "ffprobe")
        ):
            raise ValueError("media_toolchain paths must exactly match ffmpeg and ffprobe")
        if isinstance(self.dds_domain, bool) or not isinstance(self.dds_domain, int) or self.dds_domain < 0:
            raise PlayableLaunchError("dds_domain must be a non-negative integer")
        if self.windows_cpu_isolation is not None and not isinstance(
            self.windows_cpu_isolation,
            WindowsCpuIsolationConfig,
        ):
            raise TypeError("windows_cpu_isolation must be WindowsCpuIsolationConfig or None")
        if not isinstance(self.depth_capture_in_main_renderer, bool):
            raise TypeError("depth_capture_in_main_renderer must be boolean")
        if not isinstance(self.shared_color_depth_capture, bool):
            raise TypeError("shared_color_depth_capture must be boolean")
        if (
            isinstance(self.main_view_screen_percentage, bool)
            or not isinstance(self.main_view_screen_percentage, int)
            or not 50 <= self.main_view_screen_percentage <= 100
        ):
            raise PlayableLaunchError(
                "main_view_screen_percentage must be an integer in [50, 100]"
            )
        if isinstance(self.warmup_steps, bool) or not isinstance(self.warmup_steps, int) or self.warmup_steps < 1:
            raise PlayableLaunchError("warmup_steps must be a positive integer")
        if (
            isinstance(self.frame_capture_max, bool)
            or not isinstance(self.frame_capture_max, int)
            or not PLAYABLE_MIN_FRAME_COUNT <= self.frame_capture_max <= 100_000
        ):
            raise PlayableLaunchError("frame_capture_max must be an integer in [600, 100000]")
        for name in ("ready_timeout_s", "sleep_s", "frame_capture_wait_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise PlayableLaunchError(f"{name} must be numeric")
            number = float(value)
            if not math.isfinite(number):
                raise PlayableLaunchError(f"{name} must be finite")
            if name in {"ready_timeout_s", "frame_capture_wait_timeout_s"} and number <= 0.0:
                raise PlayableLaunchError(f"{name} must be positive")
            if name == "sleep_s" and number < 0.0:
                raise PlayableLaunchError("sleep_s must be non-negative")
            object.__setattr__(self, name, number)
        if (
            not isinstance(self.truth_odom_parent_frame, str)
            or not self.truth_odom_parent_frame
            or self.truth_odom_parent_frame != self.truth_odom_parent_frame.strip()
        ):
            raise PlayableLaunchError("truth_odom_parent_frame must be non-empty trimmed text")
        for name in ("run_id", "boot_id"):
            value = getattr(self, name)
            if value is not None and (not isinstance(value, str) or _RUN_ID_RE.fullmatch(value) is None):
                raise PlayableLaunchError(f"{name} must be a canonical run identity when provided")


@dataclass(frozen=True)
class PlayableLaunchDependencies:
    """Injectable adapters at process, transport, and owner-thread seams."""

    physics_host_factory: Callable[..., Any] = MujocoProcess
    coordinator_factory: Callable[..., Any] = RuntimeCoordinator
    editor_process_factory: Callable[..., Any] = UnrealProcess
    packaged_process_factory: Callable[..., Any] = PackagedUnrealProcess
    snapshot_publisher_factory: Callable[..., Any] = UdpLoopbackSnapshotPublisher
    visual_watcher_factory: Callable[..., Any] = ExternalEvidenceWatcher
    camera_watcher_factory: Callable[..., Any] = ExternalEvidenceWatcher
    session_host_factory: Callable[..., Any] = SessionHost
    camera_payload_source_factory: Callable[..., Any] = CameraShmPayloadSource
    motion_inbox_factory: Callable[..., Any] = LatestOperatorIntentInbox
    request_inbox_factory: Callable[..., Any] = BoundedRuntimeRequestInbox
    control_receiver_factory: Callable[..., Any] = UdpLoopbackOperatorIntentReceiver
    control_ack_publisher_factory: Callable[..., Any] = UdpLoopbackControlAckPublisher
    control_evidence_writer_factory: Callable[..., Any] = PlayableControlEvidenceWriter
    recording_controller_factory: Callable[..., Any] = InteractiveRecordingController
    control_pump_factory: Callable[..., Any] = PlayableControlPump.from_bundle
    interactive_session_factory: Callable[..., Any] = InteractiveSimulationSession
    cpu_isolation_resolver: Callable[..., WindowsCpuIsolationPlan] = resolve_windows_cpu_isolation

    def __post_init__(self) -> None:
        for name in (
            "physics_host_factory",
            "coordinator_factory",
            "editor_process_factory",
            "packaged_process_factory",
            "snapshot_publisher_factory",
            "visual_watcher_factory",
            "camera_watcher_factory",
            "session_host_factory",
            "camera_payload_source_factory",
            "motion_inbox_factory",
            "request_inbox_factory",
            "control_receiver_factory",
            "control_ack_publisher_factory",
            "control_evidence_writer_factory",
            "recording_controller_factory",
            "control_pump_factory",
            "interactive_session_factory",
            "cpu_isolation_resolver",
        ):
            if not callable(getattr(self, name)):
                raise TypeError(f"{name} must be callable")


@dataclass
class PlayableLaunch:
    """All resources owned by one not-yet-started playable run."""

    session: Any
    host: Any
    coordinator: Any
    physics_host: Any
    unreal_process: Any
    snapshot_publisher: Any
    evidence_watchers: tuple[Any, Any]
    camera_payload_source: Any
    control_receiver: Any
    control_ack_publisher: Any
    control_evidence_writer: Any
    recording_controller: Any
    control_pump: Any
    motion_inbox: Any
    request_inbox: Any
    target: BaseTwistTarget
    run_id: str
    boot_id: str
    run_dir: Path
    session_id: str
    ports: Mapping[str, int]
    ffmpeg: Path
    ffprobe: Path
    media_toolchain: PinnedPlayableMediaToolchain
    runtime_surface: str
    unreal_log_name: str
    frame_capture_wait_timeout_s: float
    cpu_isolation_plan: WindowsCpuIsolationPlan | None
    _bundle_dir: Path = field(repr=False)
    _closed: bool = field(default=False, init=False, repr=False)
    _close_results: dict[str, bool] = field(default_factory=dict, init=False, repr=False)

    def close(self) -> None:
        """Close every outer and nested runtime resource exactly once."""

        _close_playable_launch(self, original=None)


def create_playable_launch(
    bundle: ResolvedSessionBundle,
    *,
    runtime: PlayableRuntimeConfig,
    sensor_endpoint_factory_builder: Callable[[Any], SensorEndpointFactory] | None = None,
    dependencies: PlayableLaunchDependencies | None = None,
) -> PlayableLaunch:
    """Assemble, but do not start, the one fixed production playable graph."""

    _validate_fixed_bundle(bundle)
    if not isinstance(runtime, PlayableRuntimeConfig):
        raise TypeError("runtime must be PlayableRuntimeConfig")
    if bundle.repo_root is None or bundle.repo_root.resolve() != runtime.repo_root:
        raise PlayableLaunchError("runtime repo_root must equal the validated bundle repo_root")
    deps = dependencies or PlayableLaunchDependencies()
    if not isinstance(deps, PlayableLaunchDependencies):
        raise TypeError("dependencies must be PlayableLaunchDependencies")
    if sensor_endpoint_factory_builder is not None and not callable(sensor_endpoint_factory_builder):
        raise TypeError("sensor_endpoint_factory_builder must be callable")

    cpu_isolation_plan = None
    if runtime.windows_cpu_isolation is not None:
        cpu_isolation_plan = deps.cpu_isolation_resolver(runtime.windows_cpu_isolation)
        if not isinstance(cpu_isolation_plan, WindowsCpuIsolationPlan):
            raise PlayableLaunchError("cpu_isolation_resolver must return WindowsCpuIsolationPlan")

    run_id = runtime.run_id or f"playable-{uuid.uuid4().hex}"
    boot_id = runtime.boot_id or f"playable-boot-{uuid.uuid4().hex}"
    run_dir = Path(os.path.abspath(os.fspath(runtime.run_root / run_id)))
    if run_dir.exists():
        raise PlayableLaunchError(f"playable run directory already exists: {run_dir}")
    target = BaseTwistTarget(
        controller_id=PLAYABLE_CONTROLLER_ID,
        instance_id=PLAYABLE_INSTANCE_ID,
        channel_id=PLAYABLE_BASE_TWIST_CHANNEL,
    )
    ports = dict(runtime.ports)
    cleanup: list[tuple[Any, str]] = []
    try:
        evidence_writer = deps.control_evidence_writer_factory(run_dir)
        if not callable(evidence_writer) or not callable(getattr(evidence_writer, "close", None)):
            raise PlayableLaunchError("control_evidence_writer_factory must return a callable closeable writer")
        cleanup.append((evidence_writer, "close"))
        physics_kwargs = (
            {"affinity_mask": cpu_isolation_plan.mujoco_affinity_mask} if cpu_isolation_plan is not None else {}
        )
        physics_kwargs["sample_stride_steps"] = _playable_physics_sample_stride_steps(bundle)
        physics_host = deps.physics_host_factory(
            runtime.mujoco_host,
            **physics_kwargs,
        )
        cleanup.append((physics_host, "stop"))
        sensor_factory = (
            sensor_endpoint_factory_builder(physics_host)
            if sensor_endpoint_factory_builder is not None
            else _production_sensor_endpoint_factory(runtime, physics_host)
        )
        if not callable(sensor_factory):
            raise PlayableLaunchError("sensor_endpoint_factory_builder must return a callable factory")
        coordinator = deps.coordinator_factory(
            bundle_dir=bundle.bundle_dir,
            repo_root=runtime.repo_root,
            run_root=runtime.run_root,
            physics_host=physics_host,
            run_id=run_id,
            boot_id=boot_id,
            dds_domain=runtime.dds_domain,
            ports=ports,
            artifact_root_mode="run",
            controller_factory=create_production_components,
            sensor_endpoint_factory=sensor_factory,
        )
        cleanup.append((coordinator, "stop"))
        unreal_kwargs = {
            "frame_capture_dir": run_dir / "frames",
            "frame_capture_every": 1,
            "frame_capture_max": runtime.frame_capture_max,
            "session_camera_tag": PLAYABLE_SESSION_CAMERA_TAG,
            "motion_camera_stable_id": "thunder_01/base_link",
            "depth_capture_in_main_renderer": runtime.depth_capture_in_main_renderer,
            "shared_color_depth_capture": runtime.shared_color_depth_capture,
            "main_view_screen_percentage": runtime.main_view_screen_percentage,
        }
        if cpu_isolation_plan is not None:
            unreal_kwargs["affinity_mask"] = cpu_isolation_plan.unreal_affinity_mask
        if runtime.runtime_surface == PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME:
            unreal = deps.editor_process_factory(
                runtime.unreal_editor_executable,
                runtime.unreal_project,
                PLAYABLE_LEVEL,
                launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
                **unreal_kwargs,
            )
        else:
            unreal = deps.packaged_process_factory(
                runtime.robotsimue_executable,
                PLAYABLE_LEVEL,
                **unreal_kwargs,
            )
        cleanup.append((unreal, "terminate"))
        publisher = deps.snapshot_publisher_factory(ports["visual_snapshot_udp"])
        cleanup.append((publisher, "close"))
        log_dir = run_dir / "logs"
        visual_watcher = deps.visual_watcher_factory(
            log_dir / "visual-readiness.json",
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-visual",
        )
        camera_watcher = deps.camera_watcher_factory(
            log_dir / "sensor-readiness.json",
            session_id=bundle.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-camera",
        )
        watchers = (visual_watcher, camera_watcher)
        host = deps.session_host_factory(
            coordinator=coordinator,
            unreal_process=unreal,
            publisher=publisher,
            evidence_watchers=watchers,
            snapshot_port=ports["visual_snapshot_udp"],
            warmup_steps=runtime.warmup_steps,
            ready_timeout_s=runtime.ready_timeout_s,
            sleep_s=runtime.sleep_s,
        )
        cleanup = [
            (evidence_writer, "close"),
            (physics_host, "stop"),
            (host, "close"),
        ]
        camera_payload_source = deps.camera_payload_source_factory(
            sensor_plan=bundle.plans["sensor.plan.json"],
            allocation_provider=lambda: coordinator.allocation,
        )
        cleanup.append((camera_payload_source, "close"))
        motion_inbox = deps.motion_inbox_factory()
        request_inbox = deps.request_inbox_factory()
        identity = OperatorIntentIdentity(
            run_id=run_id,
            session_id=bundle.session_id,
            boot_id=boot_id,
            model_generation=0,
            reset_generation=0,
            source_id=PLAYABLE_CONTROL_SOURCE_ID,
        )
        receiver = deps.control_receiver_factory(
            ports["control_intent_udp"],
            expected_identity=identity,
            motion_inbox=motion_inbox,
            request_inbox=request_inbox,
        )
        cleanup.append((receiver, "close"))
        ack_publisher = deps.control_ack_publisher_factory(ports["control_status_udp"])
        cleanup.append((ack_publisher, "close"))
        recording_controller = deps.recording_controller_factory(
            run_dir=run_dir,
            run_id=run_id,
            session_id=bundle.session_id,
            allocation_provider=lambda: coordinator.allocation,
        )
        control_pump = deps.control_pump_factory(
            session_host=host,
            bundle_dir=bundle.bundle_dir,
            controller_id=PLAYABLE_CONTROLLER_ID,
            motion_inbox=motion_inbox,
            request_inbox=request_inbox,
            ack_publisher=ack_publisher,
            trace_sink=evidence_writer,
            recording_controller=recording_controller,
        )
        interactive_kwargs = {}
        if cpu_isolation_plan is not None:
            interactive_kwargs["owner_thread_affinity_mask"] = cpu_isolation_plan.owner_thread_affinity_mask
        session = deps.interactive_session_factory(
            host,
            steps_per_tick=_playable_steps_per_tick(bundle),
            advance_wait_s=PLAYABLE_ADVANCE_WAIT_S,
            control_pump=control_pump,
            sensor_payload_source=camera_payload_source,
            **interactive_kwargs,
        )
        bind_recording_session = getattr(recording_controller, "bind_session", None)
        if not callable(bind_recording_session):
            raise PlayableLaunchError("recording_controller_factory must return a controller with bind_session()")
        bind_recording_session(session)
    except BaseException as exc:
        _close_after_assembly_failure(cleanup, exc)
        raise

    return PlayableLaunch(
        session=session,
        host=host,
        coordinator=coordinator,
        physics_host=physics_host,
        unreal_process=unreal,
        snapshot_publisher=publisher,
        evidence_watchers=watchers,
        camera_payload_source=camera_payload_source,
        control_receiver=receiver,
        control_ack_publisher=ack_publisher,
        control_evidence_writer=evidence_writer,
        recording_controller=recording_controller,
        control_pump=control_pump,
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
        target=target,
        run_id=run_id,
        boot_id=boot_id,
        run_dir=run_dir,
        session_id=bundle.session_id,
        ports=MappingProxyType(ports),
        ffmpeg=runtime.ffmpeg,
        ffprobe=runtime.ffprobe,
        media_toolchain=runtime.media_toolchain,
        runtime_surface=runtime.runtime_surface,
        unreal_log_name=_unreal_log_name(runtime.runtime_surface),
        frame_capture_wait_timeout_s=runtime.frame_capture_wait_timeout_s,
        cpu_isolation_plan=cpu_isolation_plan,
        _bundle_dir=bundle.bundle_dir.resolve(),
    )


_RunResult = TypeVar("_RunResult")


def run_playable_launch(
    launch: PlayableLaunch,
    *,
    run_body: Callable[[], _RunResult],
) -> _RunResult:
    """Run one prepared interactive session around a UE-input-only action.

    ``run_body`` deliberately receives no coordinator or command submitter.  A
    production body may drive the owned foreground RobotSimUE window or wait
    for UE runtime requests, but this seam does not provide a scripted direct
    controller-submit path.
    """

    if not isinstance(launch, PlayableLaunch):
        raise TypeError("launch must be PlayableLaunch")
    if not callable(run_body):
        raise TypeError("run_body must be callable")
    if launch._closed:
        raise PlayableLifecycleError("playable launch is already closed")
    original: BaseException | None = None
    try:
        launch.control_receiver.start()
        launch.session.prepare()
        _declare_playable_episode_artifacts(launch.coordinator)
        launch.session.start()
        return run_body()
    except BaseException as exc:
        original = exc
        raise
    finally:
        _close_playable_launch(launch, original=original)


def _declare_playable_episode_artifacts(coordinator: Any) -> None:
    declare = getattr(coordinator, "declare_episode_artifact", None)
    if not callable(declare):
        raise PlayableLifecycleError("production coordinator must expose declare_episode_artifact()")
    declare("recording_manifest", "recording/recording.manifest.json")
    declare("shutdown_evidence", "shutdown-evidence.json")


def run_playable_vertical_slice(
    launch: PlayableLaunch,
    *,
    input_driver: PlayableInputDriver | None = None,
    qualify_closed_run: Callable[[PlayableClosedRun], _RunResult] | None = None,
    natural_exit_timeout_s: float = 10.0,
    unreal_exit_timeout_s: float = 10.0,
    poll_interval_s: float = 0.01,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], object] = time.sleep,
) -> _RunResult | Path:
    """Drive the capability-limited action seam and qualify only after close.

    The default driver uses ``user32.SendInput`` against the verified owned
    RobotSimUE foreground window.  The injected driver seam is for tests.  Exit
    must act through the UE Menu.  The
    runner never treats its cleanup ``session.stop()`` as a successful exit:
    it first waits for the interactive owner to reach ``STOPPED`` naturally,
    then requires the correlated accepted ``cleared:exit`` zero trace.
    """

    if not isinstance(launch, PlayableLaunch):
        raise TypeError("launch must be PlayableLaunch")
    driver = input_driver or OwnedRobotSimUEInput()
    if not callable(getattr(driver, "perform_actions", None)) or not callable(getattr(driver, "request_exit", None)):
        raise TypeError("input_driver must provide perform_actions and request_exit")
    qualifier: Callable[[PlayableClosedRun], Any]
    if qualify_closed_run is None:
        qualifier = _write_default_qualification
    elif callable(qualify_closed_run):
        qualifier = qualify_closed_run
    else:
        raise TypeError("qualify_closed_run must be callable when provided")
    timeout = _positive_finite_seconds(
        natural_exit_timeout_s,
        "natural_exit_timeout_s",
    )
    unreal_timeout = _positive_finite_seconds(
        unreal_exit_timeout_s,
        "unreal_exit_timeout_s",
    )
    poll_interval = _positive_finite_seconds(poll_interval_s, "poll_interval_s")
    if not callable(monotonic) or not callable(sleep):
        raise TypeError("monotonic and sleep must be callable")

    action_context: PlayableActionContext | None = None

    def run_body() -> None:
        nonlocal action_context
        action_context = _build_action_context(launch)
        driver.perform_actions(action_context)
        driver.request_exit(action_context)
        _wait_for_natural_exit(
            launch.session,
            timeout_s=timeout,
            poll_interval_s=poll_interval,
            monotonic=monotonic,
            sleep=sleep,
        )
        _wait_for_owned_unreal_exit(
            launch.unreal_process,
            timeout_s=unreal_timeout,
            poll_interval_s=poll_interval,
            monotonic=monotonic,
            sleep=sleep,
        )

    try:
        run_playable_launch(launch, run_body=run_body)
    except BaseException as exc:
        _scan_and_qualify_after_failure(launch, qualifier, exc)
        raise

    if action_context is None:
        error = PlayableLifecycleError("playable action context was not constructed")
        _scan_and_qualify_after_failure(launch, qualifier, error)
        raise error

    try:
        exit_event_id = _require_correlated_ue_exit_zero(launch)
    except BaseException as exc:
        _scan_and_qualify_after_failure(launch, qualifier, exc)
        raise
    closed_run = _build_closed_run(launch, exit_event_id=exit_event_id)
    return qualifier(closed_run)


def _close_playable_launch(
    launch: PlayableLaunch,
    *,
    original: BaseException | None,
) -> None:
    if launch._closed:
        return
    launch._closed = True
    errors: list[BaseException] = []
    operations = (
        ("control_intent_udp", launch.control_receiver, "close"),
        ("session", launch.session, "stop"),
        ("control_evidence", launch.control_evidence_writer, "close"),
        ("runtime_host", launch.host, "close"),
        ("mujoco", launch.physics_host, "stop"),
        ("control_status_udp", launch.control_ack_publisher, "close"),
        ("camera", launch.camera_payload_source, "close"),
    )
    for resource_name, resource, method_name in operations:
        method = getattr(resource, method_name, None)
        if not callable(method):
            launch._close_results[resource_name] = False
            errors.append(PlayableLifecycleError(f"owned resource lacks {method_name}(): {type(resource).__name__}"))
            continue
        try:
            method()
        except BaseException as cleanup_error:
            launch._close_results[resource_name] = False
            errors.append(cleanup_error)
        else:
            launch._close_results[resource_name] = True
    if not errors:
        return
    if original is not None:
        for close_error in errors:
            _add_exception_note(
                original,
                f"playable cleanup failed: {type(close_error).__name__}: {close_error}",
            )
        return
    summary = "; ".join(f"{type(close_error).__name__}: {close_error}" for close_error in errors)
    raise PlayableLifecycleError("playable lifecycle cleanup failed: " + summary) from errors[0]


def _build_action_context(launch: PlayableLaunch) -> PlayableActionContext:
    pid = getattr(launch.unreal_process, "pid", None)
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
        raise PlayableLifecycleError("owned RobotSimUE process must expose a positive PID before input")
    return PlayableActionContext(
        run_id=launch.run_id,
        boot_id=launch.boot_id,
        session_id=launch.session_id,
        run_dir=launch.run_dir,
        unreal_pid=pid,
        deadman_key="SHIFT",
        actions=PLAYABLE_INPUT_SCHEDULE,
        hud_screenshot_paths=_hud_screenshot_paths(launch.run_dir),
        runtime_surface=launch.runtime_surface,
        unreal_log_path=launch.run_dir / "logs" / launch.unreal_log_name,
        frame_capture_wait_timeout_s=launch.frame_capture_wait_timeout_s,
    )


def _hud_screenshot_paths(run_dir: Path) -> Mapping[str, Path]:
    screenshot_dir = run_dir / "screenshots"
    return MappingProxyType(
        {
            "drive": screenshot_dir / "hud-drive.png",
            "tactical": screenshot_dir / "hud-tactical.png",
            "menu_recording": screenshot_dir / "hud-menu-recording.png",
        }
    )


def _unreal_log_name(runtime_surface: str) -> str:
    if runtime_surface == PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME:
        return "Unreal.log"
    if runtime_surface == PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE:
        return "RobotSimUE.log"
    raise PlayableLaunchError("runtime_surface must be editor_game or packaged_release")


def _wait_for_natural_exit(
    session: Any,
    *,
    timeout_s: float,
    poll_interval_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], object],
) -> None:
    start = _finite_clock_value(monotonic(), "monotonic")
    deadline = start + timeout_s
    while True:
        state = getattr(session, "state", None)
        if state is RuntimeState.STOPPED:
            return
        if state is RuntimeState.FAILED:
            reason = getattr(session, "last_error", None)
            detail = f": {reason}" if isinstance(reason, str) and reason else ""
            raise PlayableLifecycleError("playable session failed before UE-origin exit completed" + detail)
        if state not in {RuntimeState.RUNNING, RuntimeState.PAUSED}:
            raise PlayableLifecycleError(f"playable session entered invalid pre-exit state: {state!r}")
        now = _finite_clock_value(monotonic(), "monotonic")
        if now < start:
            raise PlayableLifecycleError("monotonic clock moved backwards")
        if now >= deadline:
            raise PlayableLifecycleError("UE-origin exit request did not naturally stop the playable session")
        sleep(min(poll_interval_s, deadline - now))


def _wait_for_owned_unreal_exit(
    unreal_process: Any,
    *,
    timeout_s: float,
    poll_interval_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], object],
) -> None:
    poll = getattr(unreal_process, "poll", None)
    if not callable(poll):
        raise PlayableLifecycleError("owned RobotSimUE process must expose poll()")
    start = _finite_clock_value(monotonic(), "monotonic")
    deadline = start + timeout_s
    while True:
        shutdown = getattr(unreal_process, "last_shutdown", None)
        if shutdown is not None:
            if (
                getattr(shutdown, "exit_code", None) == 0
                and getattr(shutdown, "direct_child_running_after_close", None) is False
                and getattr(shutdown, "process_owner_closed", None) is True
                and getattr(shutdown, "termination_mode", None) == "natural"
            ):
                return
            raise PlayableLifecycleError("owned RobotSimUE immutable shutdown evidence is not natural zero-exit")
        exit_code = poll()
        if exit_code is not None:
            if isinstance(exit_code, bool) or not isinstance(exit_code, int):
                raise PlayableLifecycleError("owned RobotSimUE process returned an invalid exit code")
            if exit_code != 0:
                raise PlayableLifecycleError(f"owned RobotSimUE process exited with code {exit_code}")
            return
        now = _finite_clock_value(monotonic(), "monotonic")
        if now < start:
            raise PlayableLifecycleError("monotonic clock moved backwards")
        if now >= deadline:
            raise PlayableLifecycleError("owned RobotSimUE process did not exit naturally after the UE Menu exit")
        sleep(min(poll_interval_s, deadline - now))


def _build_closed_run(
    launch: PlayableLaunch,
    *,
    exit_event_id: str | None,
) -> PlayableClosedRun:
    if not launch._closed:
        raise PlayableLifecycleError("playable artifacts may be scanned only after runtime close")
    frame_dir = launch.run_dir / "frames"
    frames = (
        tuple(
            sorted(
                path.resolve()
                for path in frame_dir.glob("frame_*.png")
                if path.is_file() and _FRAME_RE.fullmatch(path.name) is not None
            )
        )
        if frame_dir.is_dir()
        else ()
    )
    return PlayableClosedRun(
        run_id=launch.run_id,
        boot_id=launch.boot_id,
        session_id=launch.session_id,
        run_dir=launch.run_dir,
        unreal_log_path=launch.run_dir / "logs" / launch.unreal_log_name,
        frames=frames,
        hud_screenshot_paths=_hud_screenshot_paths(launch.run_dir),
        exit_event_id=exit_event_id,
        media_toolchain=launch.media_toolchain,
        unreal_shutdown=getattr(launch.unreal_process, "last_shutdown", None),
        mujoco_shutdown=getattr(launch.physics_host, "last_shutdown", None),
        resources_closed=MappingProxyType(
            {
                "control_intent_udp": launch._close_results.get(
                    "control_intent_udp",
                    False,
                ),
                "control_status_udp": launch._close_results.get(
                    "control_status_udp",
                    False,
                ),
                "sensors": all(
                    launch._close_results.get(name, False) for name in ("session", "runtime_host", "mujoco", "camera")
                ),
                "recording": (
                    launch._close_results.get("control_evidence", False)
                    and _recording_committed(launch.recording_controller, launch.run_dir)
                ),
            }
        ),
    )


def _recording_committed(recording_controller: Any, run_dir: Path) -> bool:
    status_snapshot = getattr(recording_controller, "status_snapshot", None)
    if not callable(status_snapshot):
        return False
    try:
        status = status_snapshot()
    except Exception:
        return False
    lifecycle = getattr(status, "lifecycle_state", None)
    lifecycle_value = getattr(lifecycle, "value", lifecycle)
    return (
        lifecycle_value == "COMMITTED"
        and getattr(status, "state", None) == "committed"
        and getattr(status, "artifact_id", None) == "simulation-recording.json"
        and (run_dir / "simulation-recording.json").is_file()
    )


def _require_correlated_ue_exit_zero(launch: PlayableLaunch) -> str:
    try:
        accepted = _read_jsonl(launch.run_dir / "control-intent-accepted.jsonl")
        runtime_requests = _read_jsonl(launch.run_dir / "runtime-request-trace.jsonl")
        origins = _read_jsonl(
            launch.run_dir / "logs" / "ue-control-origin.jsonl"
        )
    except (OSError, ValueError, PlayableEvidenceError) as exc:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence") from exc
    if not accepted:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence")
    zero = accepted[-1]
    try:
        _validate_correlated_exit_zero(zero)
    except ValueError as exc:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence") from exc
    expected_identity = {
        "run_id": launch.run_id,
        "session_id": launch.session_id,
        "boot_id": launch.boot_id,
        "source_id": PLAYABLE_CONTROL_SOURCE_ID,
    }
    if any(zero.get(field) != value for field, value in expected_identity.items()):
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence")
    join_fields = (
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "datagram_sha256",
    )
    exit_request = next(
        (
            record
            for record in runtime_requests
            if record.get("schema") == "lingtu.sim.runtime-request-trace.v1"
            and record.get("event") == "runtime_request_accepted"
            and record.get("request") == "exit"
            and record.get("status") == "accepted"
            and all(record.get(field) == zero.get(field) for field in join_fields)
        ),
        None,
    )
    if exit_request is None:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence")
    event_id = zero.get("event_id")
    if not isinstance(event_id, str) or not event_id:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence")
    origin_fields = (
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "datagram_sha256",
    )
    origin = next(
        (
            record
            for record in origins
            if set(record) == set(_UE_ORIGIN_FIELDS)
            and record.get("schema") == "lingtu.sim.ue-control-origin.v1"
            and record.get("successful_send") is True
            and isinstance(record.get("datagram_bytes"), int)
            and not isinstance(record.get("datagram_bytes"), bool)
            and int(record["datagram_bytes"]) > 0
            and all(record.get(field) == zero.get(field) for field in origin_fields)
        ),
        None,
    )
    if origin is None:
        raise PlayableLifecycleError("playable completion lacks correlated UE-origin exit zero evidence")
    return event_id


def _validate_correlated_exit_zero(record: Mapping[str, Any]) -> None:
    if (
        record.get("schema") != "lingtu.sim.control-command-zero.v1"
        or record.get("event") != "control_command_zero"
        or record.get("reason") != "cleared:exit"
        or record.get("submit_result") != "accepted"
    ):
        raise ValueError("final zero is not an accepted UE exit zero")
    required = (
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "source_monotonic_ns",
        "arrival_monotonic_ns",
        "datagram_sha256",
        "controller_sequence",
        "apply_time_ns",
    )
    if any(record.get(field) is None for field in required):
        raise ValueError("final exit zero lacks correlation identity")
    twist = record.get("admitted_twist")
    if not isinstance(twist, Mapping) or set(twist) != {
        "linear_x",
        "linear_y",
        "angular_z",
    }:
        raise ValueError("final exit zero has an invalid twist")
    for value in twist.values():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) != 0.0
        ):
            raise ValueError("final exit zero is not zero")


def _read_jsonl(path: Path) -> tuple[Mapping[str, Any], ...]:
    return read_stable_playable_jsonl(path)


def _strict_status_records(
    payload: bytes,
    context: PlayableActionContext,
) -> tuple[Mapping[str, Any], ...]:
    from .control_status import encode_control_status

    if not payload or not payload.endswith(b"\n"):
        raise ValueError("control status authority is partial")
    records: list[Mapping[str, Any]] = []
    previous_sequence = 0
    for index, raw_line in enumerate(payload.splitlines(), 1):
        document = _strict_json_object(raw_line, f"control status line {index}")
        encode_control_status(document)
        for identity_field, expected in (
            ("run_id", context.run_id),
            ("boot_id", context.boot_id),
            ("session_id", context.session_id),
        ):
            if document.get(identity_field) != expected:
                raise ValueError(f"control status {identity_field} mismatch")
        sequence = _status_server_sequence(document)
        if sequence <= previous_sequence:
            raise ValueError("control status server sequence is not strictly increasing")
        previous_sequence = sequence
        records.append(document)
    return tuple(records)


def _status_server_sequence(status: Mapping[str, Any]) -> int:
    value = status.get("server_status_sequence")
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("control status server_status_sequence is invalid")
    return value


def _status_sim_time_ns(status: Mapping[str, Any]) -> int:
    value = status.get("sim_time_ns")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("control status sim_time_ns is invalid")
    return value


def _status_nested_text(
    status: Mapping[str, Any],
    section: str,
    field: str,
) -> str:
    raw_section = status.get(section)
    if not isinstance(raw_section, Mapping):
        raise ValueError(f"control status {section} is invalid")
    value = raw_section.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"control status {section}.{field} is invalid")
    return value


def _status_runtime_state(status: Mapping[str, Any]) -> str:
    return _status_nested_text(status, "runtime", "runtime_state")


def _status_recording_state(status: Mapping[str, Any]) -> str:
    return _status_nested_text(status, "recording", "state")


def _status_ui_mode(status: Mapping[str, Any]) -> str:
    return _status_nested_text(status, "ui", "ui_mode")


def _status_camera_mode(status: Mapping[str, Any]) -> str:
    return _status_nested_text(status, "ui", "camera_mode")


def _status_has_current_five_streams(status: Mapping[str, Any]) -> bool:
    sensors = status.get("sensors")
    if not isinstance(sensors, Sequence) or isinstance(sensors, (str, bytes)):
        return False
    states: dict[str, int] = {}
    for item in sensors:
        if not isinstance(item, Mapping):
            return False
        stream_id = item.get("stream_id")
        state = item.get("state")
        sample_count = item.get("sample_count")
        blocker = item.get("blocker")
        if (
            not isinstance(stream_id, str)
            or state != "ACTIVE"
            or isinstance(sample_count, bool)
            or not isinstance(sample_count, int)
            or sample_count <= 0
            or blocker not in {"", None}
        ):
            return False
        states[stream_id] = sample_count
    return set(states) == set(PLAYABLE_SENSOR_RATES)


def _count_mapped_recording_frames(
    context: PlayableActionContext,
    *,
    recording_start_sim_time_ns: int,
) -> int:
    if not context.unreal_log_path.is_file():
        raise ValueError(f"UE frame capture log is missing: {context.unreal_log_path.name}")
    text = context.unreal_log_path.read_text(encoding="utf-8", errors="strict")
    captures: dict[int, int] = {}
    for match in _CAPTURE_LOG_RE.finditer(text):
        frame_index = int(match.group("frame"))
        sim_time_ns = int(match.group("time"))
        if sim_time_ns < recording_start_sim_time_ns:
            continue
        path = Path(match.group("path")).resolve()
        expected = (context.run_dir / "frames" / f"frame_{frame_index:06d}.png").resolve()
        if path != expected:
            raise ValueError("UE frame capture path escapes the allocated frame directory")
        if frame_index in captures:
            raise ValueError("UE frame capture log repeats a frame index")
        if not expected.is_file():
            raise ValueError(f"UE frame PNG is missing: {expected.name}")
        _require_png_1920x1080(expected)
        captures[frame_index] = sim_time_ns
    if not captures:
        return 0
    ordered = sorted(captures)
    expected_indices = list(range(ordered[0], ordered[-1] + 1))
    if ordered != expected_indices:
        raise ValueError("recording-window UE frame captures are not contiguous")
    return len(ordered)


def _require_png_1920x1080(path: Path) -> None:
    with path.open("rb") as stream:
        header = stream.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError(f"UE frame is not a PNG: {path.name}")
    width = int.from_bytes(header[16:20], "big")
    height = int.from_bytes(header[20:24], "big")
    if (width, height) != (1920, 1080):
        raise ValueError(f"UE frame is not 1920x1080: {path.name}")


def _qualify_after_failure(
    qualifier: Callable[[PlayableClosedRun], Any],
    closed_run: PlayableClosedRun,
    original: BaseException,
) -> None:
    try:
        qualifier(closed_run)
    except BaseException as qualification_error:
        _add_exception_note(
            original,
            "post-close rejection qualification also failed: "
            f"{type(qualification_error).__name__}: {qualification_error}",
        )


def _scan_and_qualify_after_failure(
    launch: PlayableLaunch,
    qualifier: Callable[[PlayableClosedRun], Any],
    original: BaseException,
) -> None:
    try:
        closed_run = _build_closed_run(launch, exit_event_id=None)
    except BaseException as scan_error:
        _add_exception_note(
            original,
            f"post-close artifact rescan also failed: {type(scan_error).__name__}: {scan_error}",
        )
        return
    _qualify_after_failure(qualifier, closed_run, original)


def _write_default_qualification(closed_run: PlayableClosedRun) -> Path:
    from sim.runtime.coordinator.playable_evidence import (
        PlayableEvidenceInputs,
        finalize_playable_evidence,
    )
    from sim.runtime.qualification.playable import write_playable_qualification

    toolchain = closed_run.media_toolchain
    trusted_probe = getattr(toolchain, "trusted_probe", None)
    trusted_descriptor = getattr(toolchain, "trusted_descriptor", None)
    assembly_error: BaseException | None = None
    try:
        finalize_playable_evidence(
            PlayableEvidenceInputs(
                run_dir=closed_run.run_dir,
                run_id=closed_run.run_id,
                boot_id=closed_run.boot_id,
                session_id=closed_run.session_id,
                unreal_log_path=closed_run.unreal_log_path,
                media_toolchain=toolchain,
                unreal_shutdown=closed_run.unreal_shutdown,
                mujoco_shutdown=closed_run.mujoco_shutdown,
                resources_closed=closed_run.resources_closed,
            )
        )
    except BaseException as exc:
        assembly_error = exc

    verdict_path = Path(
        write_playable_qualification(
            closed_run.run_dir,
            trusted_media_probe=trusted_probe,
            trusted_media_toolchain=trusted_descriptor,
        )
    )
    try:
        verdict = _strict_json_object(
            verdict_path.read_bytes(),
            verdict_path.name,
        )
    except (OSError, UnicodeError, ValueError) as exc:
        raise PlayableLifecycleError("playable qualification did not produce a readable strict verdict") from exc
    if (
        verdict.get("schema") != "lingtu.sim.ue5-playable-vertical-slice.v1"
        or verdict.get("result") != "PASS"
        or verdict.get("qualified") is not True
    ):
        reasons = verdict.get("reasons")
        detail = (
            "; ".join(str(reason) for reason in reasons)
            if isinstance(reasons, Sequence) and not isinstance(reasons, (str, bytes))
            else "completed evidence did not pass the strict product gate"
        )
        raise PlayableLifecycleError("playable qualification rejected evidence: " + detail) from assembly_error
    if assembly_error is not None:
        raise PlayableLifecycleError(
            "playable evidence assembly failed but qualification did not reject it"
        ) from assembly_error
    return verdict_path


def _positive_finite_seconds(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be positive finite seconds")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be positive finite seconds")
    return number


def _finite_clock_value(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlayableLifecycleError(f"{name} clock returned a non-finite value")
    number = float(value)
    if not math.isfinite(number):
        raise PlayableLifecycleError(f"{name} clock returned a non-finite value")
    return number


def _production_sensor_endpoint_factory(
    runtime: PlayableRuntimeConfig,
    physics_host: Any,
) -> SensorEndpointRouter:
    publishers = {
        "imu_publisher": runtime.imu_publisher,
        "truth_odom_publisher": runtime.truth_odom_publisher,
        "mid360_publisher": runtime.mid360_publisher,
    }
    missing = tuple(name for name, path in publishers.items() if path is None)
    if missing:
        raise PlayableLaunchError(
            "playable native sensor endpoints require all publisher executables; missing: " + ", ".join(missing)
        )
    imu = runtime.imu_publisher
    truth = runtime.truth_odom_publisher
    mid360 = runtime.mid360_publisher
    if imu is None or truth is None or mid360 is None:
        raise PlayableLaunchError("native sensor publisher invariant failed")
    return SensorEndpointRouter(
        (
            ImuEndpointFactory(imu),
            TruthOdometryEndpointFactory(
                truth,
                parent_frame=runtime.truth_odom_parent_frame,
            ),
            Mid360EndpointFactory(mid360, physics_host),
        )
    )


def _playable_steps_per_tick(bundle: ResolvedSessionBundle) -> int:
    physics = _plan(
        bundle,
        "physics.plan.json",
        "lingtu.sim.physics-plan.v1",
    )
    policy = _mapping(physics.get("global_policy"), "physics.plan.global_policy")
    raw_timestep = policy.get("timestep_s")
    if (
        isinstance(raw_timestep, bool)
        or not isinstance(raw_timestep, (int, float))
        or not math.isfinite(float(raw_timestep))
        or float(raw_timestep) <= 0.0
    ):
        raise PlayableLaunchError("physics.plan.global_policy.timestep_s must be positive and finite")
    exact_steps = PLAYABLE_ADVANCE_WAIT_S / float(raw_timestep)
    rounded_steps = round(exact_steps)
    if rounded_steps < 1 or not math.isclose(
        exact_steps,
        rounded_steps,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise PlayableLaunchError("playable owner tick must be an exact integer number of MuJoCo steps")
    return rounded_steps


def _playable_physics_sample_stride_steps(
    bundle: ResolvedSessionBundle,
) -> int:
    """Return the exact shared physics-sensor sampling stride."""

    physics = _plan(
        bundle,
        "physics.plan.json",
        "lingtu.sim.physics-plan.v1",
    )
    policy = _mapping(physics.get("global_policy"), "physics.plan.global_policy")
    timestep = Fraction(str(policy.get("timestep_s")))
    sensors = _plan(
        bundle,
        "sensor.plan.json",
        "lingtu.sim.sensor-plan.v1",
    )
    groups = _mapping(sensors.get("streams"), "sensor.plan.streams")
    periods: list[int] = []
    for group, declarations in groups.items():
        for stream in _object_sequence(
            declarations,
            f"sensor.plan.streams.{group}",
        ):
            if stream.get("owner") != "physics":
                continue
            rate = Fraction(str(stream.get("rate_hz")))
            period_steps = Fraction(1, 1) / (rate * timestep)
            if period_steps.denominator != 1 or period_steps.numerator <= 0:
                raise PlayableLaunchError(
                    "playable physics sensor period must align exactly to MuJoCo steps"
                )
            periods.append(period_steps.numerator)
    if not periods:
        raise PlayableLaunchError("playable requires at least one physics-owned sensor")
    stride = periods[0]
    for period in periods[1:]:
        stride = math.gcd(stride, period)
    return min(stride, _playable_steps_per_tick(bundle))


def _strict_json_object(payload: bytes, filename: str) -> Mapping[str, Any]:
    def object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"{filename} contains duplicate key {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise ValueError(f"{filename} contains non-finite JSON constant {value!r}")

    document = json.loads(
        payload.decode("utf-8"),
        object_pairs_hook=object_pairs,
        parse_constant=reject_constant,
    )
    if type(document) is not dict:
        raise ValueError(f"{filename} must contain a JSON object")
    return document


def _validate_fixed_bundle(bundle: ResolvedSessionBundle) -> None:
    if not isinstance(bundle, ResolvedSessionBundle):
        raise TypeError("bundle must be a validated ResolvedSessionBundle")
    _plan(bundle, "physics.plan.json", "lingtu.sim.physics-plan.v1")
    _plan(bundle, "transport.intent.json", "lingtu.sim.transport-intent.v1")
    session = _mapping(bundle.session_spec, "session.yaml")
    if session.get("schema") != "lingtu.sim.session.v1":
        raise PlayableLaunchError("playable session declaration has an invalid schema")
    if session.get("session_id") != PLAYABLE_SESSION_ID:
        raise PlayableLaunchError(f"playable bundle session_id must be {PLAYABLE_SESSION_ID!r}")
    runtime = _mapping(session.get("runtime"), "session.runtime")
    if runtime.get("backend") != "mujoco" or runtime.get("mode") != "unreal":
        raise PlayableLaunchError("playable bundle runtime must be MuJoCo with Unreal presentation")
    bindings = _text_sequence(
        runtime.get("required_bindings"),
        "session.runtime.required_bindings",
    )
    if len(bindings) != len(PLAYABLE_REQUIRED_BINDINGS) or set(bindings) != set(PLAYABLE_REQUIRED_BINDINGS):
        raise PlayableLaunchError("playable bundle requires exactly physics, control, visual, and sensors bindings")

    visual = _plan(bundle, "visual.plan.json", "lingtu.sim.visual-plan.v1")
    _require_backends(visual, "visual.plan")
    world = _mapping(visual.get("world"), "visual.plan.world")
    if world.get("level") != PLAYABLE_LEVEL:
        raise PlayableLaunchError(f"playable world level must be {PLAYABLE_LEVEL}")
    _require_package(
        world.get("package"),
        expected=PLAYABLE_WORLD_PACKAGE,
        context="visual.plan.world.package",
    )
    robots = _object_sequence(visual.get("robots"), "visual.plan.robots")
    if len(robots) != 1 or robots[0].get("instance_id") != PLAYABLE_INSTANCE_ID:
        raise PlayableLaunchError("playable visual plan requires exactly robot instance thunder_01")
    _require_package(
        robots[0].get("package"),
        expected=PLAYABLE_ROBOT_PACKAGE,
        context="visual.plan.robots[0].package",
    )

    control = _plan(bundle, "control.plan.json", "lingtu.sim.control-plan.v1")
    _require_backends(control, "control.plan")
    controllers = _object_sequence(control.get("controllers"), "control.plan.controllers")
    if len(controllers) != 1:
        raise PlayableLaunchError("playable control plan requires exactly one production controller")
    controller = controllers[0]
    if (
        controller.get("instance_id") != PLAYABLE_INSTANCE_ID
        or controller.get("controller_id") != PLAYABLE_CONTROLLER_ID
    ):
        raise PlayableLaunchError("playable controller must be thunder_01.thunderv4_locomotion")
    _require_package(
        controller.get("package"),
        expected=PLAYABLE_CONTROLLER_PACKAGE,
        context="control.plan.controllers[0].package",
    )
    channels = _object_sequence(control.get("command_channels"), "control.plan.command_channels")
    base_twist = tuple(
        channel
        for channel in channels
        if channel.get("channel_id") == PLAYABLE_BASE_TWIST_CHANNEL
        and channel.get("command_type") == "base_twist"
        and channel.get("direction") == "subscribe"
    )
    if len(base_twist) != 1:
        raise PlayableLaunchError("playable controller requires its exact subscribed base_twist channel")

    sensors = _plan(bundle, "sensor.plan.json", "lingtu.sim.sensor-plan.v1")
    _require_backends(sensors, "sensor.plan")
    groups = _mapping(sensors.get("streams"), "sensor.plan.streams")
    if set(groups) != {route[0] for route in PLAYABLE_SENSOR_ROUTES.values()}:
        raise PlayableLaunchError(
            "playable sensor plan must contain only rgb, depth, imu, mid360, and truth_odom groups"
        )
    actual: dict[str, tuple[str, str, str, str]] = {}
    for group, declarations in groups.items():
        streams = _object_sequence(declarations, f"sensor.plan.streams.{group}")
        for stream in streams:
            sensor_id = stream.get("sensor_id")
            if not isinstance(sensor_id, str):
                raise PlayableLaunchError(f"sensor.plan.streams.{group} contains an invalid sensor_id")
            if sensor_id in actual:
                raise PlayableLaunchError(f"playable sensor plan duplicates stream {sensor_id!r}")
            actual[sensor_id] = (
                str(group),
                str(stream.get("owner")),
                str(stream.get("source")),
                str(stream.get("transport")),
            )
            expected_rate = PLAYABLE_SENSOR_RATES.get(sensor_id)
            rate_hz = stream.get("rate_hz")
            if expected_rate is not None and (
                isinstance(rate_hz, bool)
                or not isinstance(rate_hz, (int, float))
                or not math.isfinite(float(rate_hz))
                or float(rate_hz) != expected_rate
            ):
                raise PlayableLaunchError(f"playable sensor {sensor_id!r} has a stale canonical rate")
    if actual != dict(PLAYABLE_SENSOR_ROUTES):
        raise PlayableLaunchError("playable sensor plan must declare the exact five production streams and routes")


def _plan(
    bundle: ResolvedSessionBundle,
    filename: str,
    schema: str,
) -> Mapping[str, Any]:
    plan = _mapping(bundle.plans.get(filename), filename)
    if plan.get("schema") != schema:
        raise PlayableLaunchError(f"{filename} has an invalid schema")
    if plan.get("session_id") != bundle.session_id:
        raise PlayableLaunchError(f"{filename} session_id does not match bundle")
    return plan


def _require_backends(plan: Mapping[str, Any], context: str) -> None:
    if plan.get("backends") != {"physics": "mujoco", "visual": "unreal"}:
        raise PlayableLaunchError(f"{context} must use MuJoCo physics and Unreal visual backends")


def _require_package(
    value: object,
    *,
    expected: tuple[str, str],
    context: str,
) -> None:
    package = _mapping(value, context)
    actual = (package.get("id"), package.get("version"))
    if actual != expected:
        raise PlayableLaunchError(f"{context} must be {expected[0]}@{expected[1]}")


def _mapping(value: object, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlayableLaunchError(f"{context} must be an object")
    return value


def _object_sequence(value: object, context: str) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise PlayableLaunchError(f"{context} must be an array")
    result: list[Mapping[str, Any]] = []
    for index, item in enumerate(value):
        result.append(_mapping(item, f"{context}[{index}]"))
    return tuple(result)


def _text_sequence(value: object, context: str) -> tuple[str, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise PlayableLaunchError(f"{context} must be an array")
    if any(not isinstance(item, str) for item in value):
        raise PlayableLaunchError(f"{context} must contain strings only")
    return tuple(value)


def _validated_ports(value: Mapping[str, int]) -> dict[str, int]:
    if not isinstance(value, Mapping) or set(value) != set(PLAYABLE_PORT_NAMES):
        raise PlayableLaunchError(
            "playable ports must be exactly visual_snapshot_udp, control_intent_udp, and control_status_udp"
        )
    ports: dict[str, int] = {}
    for name in sorted(PLAYABLE_PORT_NAMES):
        port = value[name]
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise PlayableLaunchError(f"{name} must be an integer in [1, 65535]")
        ports[name] = port
    if len(set(ports.values())) != len(ports):
        raise PlayableLaunchError("playable allocation ports must be distinct")
    return ports


def _close_after_assembly_failure(
    resources: Sequence[tuple[Any, str]],
    original: BaseException,
) -> None:
    for resource, method_name in reversed(tuple(resources)):
        method = getattr(resource, method_name, None)
        if not callable(method):
            continue
        try:
            method()
        except BaseException as cleanup_error:
            _add_exception_note(
                original,
                f"playable assembly cleanup failed: {type(cleanup_error).__name__}: {cleanup_error}",
            )


def _add_exception_note(error: BaseException, note: str) -> None:
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)


def _load_stable_playable_bundle(
    bundle_dir: Path,
    *,
    repo_root: Path,
) -> ResolvedSessionBundle:
    """Load twice and prove one unchanged full bundle before any process ctor.

    The catalog loader independently revalidates every declared repository
    artifact on both reads.  Raw hashes additionally reject a same-document
    byte rewrite between validation and construction.  Only the second,
    equality-proven value is handed to the production assembly.
    """

    first = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)
    _validate_fixed_bundle(first)

    second = load_resolved_session_bundle(bundle_dir, repo_root=repo_root)
    _validate_fixed_bundle(second)
    if (
        first.session_id != second.session_id
        or first.session_spec != second.session_spec
        or first.plans != second.plans
    ):
        raise PlayableLaunchError("resolved playable bundle changed during pre-launch validation")
    return second


def _existing_file(path: Path, name: str) -> Path:
    resolved = Path(path).resolve()
    if not resolved.is_file():
        raise PlayableLaunchError(f"{name} does not exist as a file: {resolved}")
    return resolved


def _existing_absolute_pinned_file(path: Path, name: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute() or candidate != Path(os.path.abspath(os.fspath(candidate))):
        raise PlayableLaunchError(f"{name} must be an explicit normalized absolute path")
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except OSError as exc:
            raise PlayableLaunchError(f"{name} does not exist as a file: {candidate}") from exc
        if current.is_symlink() or (getattr(metadata, "st_file_attributes", 0) & 0x400):
            raise PlayableLaunchError(f"{name} path contains a link or reparse point")
    resolved = candidate.resolve(strict=True)
    if resolved != candidate or not resolved.is_file():
        raise PlayableLaunchError(f"{name} must resolve to its exact regular file")
    return resolved


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run and strictly qualify the fixed FactoryPark + black ThunderV4 RobotSimUE playable vertical slice."
        )
    )
    parser.add_argument(
        "bundle",
        type=Path,
        help="validated thunderv4_factory_park_hf ResolvedSessionBundle",
    )
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument(
        "--run-root",
        type=Path,
        default=Path("build/playable-runs"),
    )
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument(
        "--runtime-surface",
        choices=(
            PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
            PLAYABLE_RUNTIME_SURFACE_PACKAGED_RELEASE,
        ),
        default=PLAYABLE_RUNTIME_SURFACE_EDITOR_GAME,
    )
    parser.add_argument("--unreal-editor-executable", type=Path)
    parser.add_argument("--unreal-project", type=Path)
    parser.add_argument("--robotsimue-executable", type=Path)
    parser.add_argument("--imu-publisher", type=Path, required=True)
    parser.add_argument("--truth-odom-publisher", type=Path, required=True)
    parser.add_argument("--mid360-publisher", type=Path, required=True)
    parser.add_argument("--ffmpeg", type=Path, required=True)
    parser.add_argument("--ffprobe", type=Path, required=True)
    parser.add_argument("--visual-snapshot-port", type=int, required=True)
    parser.add_argument("--control-intent-port", type=int, required=True)
    parser.add_argument("--control-status-port", type=int, required=True)
    parser.add_argument("--run-id")
    parser.add_argument("--boot-id")
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument("--truth-odom-parent-frame", default="map")
    parser.add_argument("--warmup-steps", type=int, default=8)
    parser.add_argument("--ready-timeout-s", type=float, default=600.0)
    parser.add_argument("--sleep-s", type=float, default=0.01)
    parser.add_argument(
        "--frame-capture-max",
        type=int,
        default=PLAYABLE_DEFAULT_FRAME_CAPTURE_MAX,
    )
    parser.add_argument(
        "--frame-capture-wait-timeout-s",
        type=float,
        default=PLAYABLE_DEFAULT_FRAME_CAPTURE_WAIT_TIMEOUT_S,
    )
    parser.add_argument("--natural-exit-timeout-s", type=float, default=30.0)
    parser.add_argument("--unreal-exit-timeout-s", type=float, default=15.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Production CLI; it has no scripted-command or fake-qualifier seam."""

    args = _parser().parse_args(argv)
    repo_root = args.repo_root.resolve()
    run_root = args.run_root.resolve()
    try:
        mujoco_host = _existing_file(args.mujoco_host, "mujoco_host")
        unreal_editor_executable = (
            _existing_file(args.unreal_editor_executable, "unreal_editor_executable")
            if args.unreal_editor_executable is not None
            else None
        )
        unreal_project = (
            _existing_file(args.unreal_project, "unreal_project") if args.unreal_project is not None else None
        )
        robotsimue_executable = (
            _existing_file(args.robotsimue_executable, "robotsimue_executable")
            if args.robotsimue_executable is not None
            else None
        )
        ffmpeg = _existing_absolute_pinned_file(args.ffmpeg, "ffmpeg")
        ffprobe = _existing_absolute_pinned_file(args.ffprobe, "ffprobe")
        imu_publisher = _existing_file(args.imu_publisher, "imu_publisher")
        truth_odom_publisher = _existing_file(
            args.truth_odom_publisher,
            "truth_odom_publisher",
        )
        mid360_publisher = _existing_file(
            args.mid360_publisher,
            "mid360_publisher",
        )
        bundle = _load_stable_playable_bundle(
            args.bundle,
            repo_root=repo_root,
        )
        media_toolchain = snapshot_playable_media_toolchain(
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
        runtime = PlayableRuntimeConfig(
            repo_root=repo_root,
            run_root=run_root,
            mujoco_host=mujoco_host,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            media_toolchain=media_toolchain,
            runtime_surface=args.runtime_surface,
            unreal_editor_executable=unreal_editor_executable,
            unreal_project=unreal_project,
            robotsimue_executable=robotsimue_executable,
            imu_publisher=imu_publisher,
            truth_odom_publisher=truth_odom_publisher,
            mid360_publisher=mid360_publisher,
            ports={
                "visual_snapshot_udp": args.visual_snapshot_port,
                "control_intent_udp": args.control_intent_port,
                "control_status_udp": args.control_status_port,
            },
            truth_odom_parent_frame=args.truth_odom_parent_frame,
            dds_domain=args.dds_domain,
            run_id=args.run_id,
            boot_id=args.boot_id,
            warmup_steps=args.warmup_steps,
            ready_timeout_s=args.ready_timeout_s,
            sleep_s=args.sleep_s,
            frame_capture_max=args.frame_capture_max,
            frame_capture_wait_timeout_s=args.frame_capture_wait_timeout_s,
        )
        launch = create_playable_launch(bundle, runtime=runtime)
        verdict_path = run_playable_vertical_slice(
            launch,
            natural_exit_timeout_s=args.natural_exit_timeout_s,
            unreal_exit_timeout_s=args.unreal_exit_timeout_s,
        )
        if not isinstance(verdict_path, Path):
            raise PlayableLifecycleError("production playable runner did not return its strict verdict path")
    except (
        OSError,
        RunAllocationError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "result": "EVIDENCE_REJECTED",
                    "error": str(exc),
                },
                allow_nan=False,
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1

    print(
        json.dumps(
            {
                "ok": True,
                "result": "PASS",
                "playable_qualification": str(verdict_path.resolve()),
            },
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


__all__ = [
    "PLAYABLE_DEFAULT_FRAME_CAPTURE_MAX",
    "PLAYABLE_INPUT_SCHEDULE",
    "PLAYABLE_MIN_FRAME_COUNT",
    "PLAYABLE_PORT_NAMES",
    "PLAYABLE_REQUIRED_BINDINGS",
    "PLAYABLE_SENSOR_ROUTES",
    "OwnedRobotSimUEWindowCandidate",
    "OwnedRobotSimUEWindowPresenceProof",
    "OwnedRobotSimUEWindowProof",
    "PlayableActionContext",
    "PlayableClosedRun",
    "PlayableControlEvidenceWriter",
    "PlayableInputAction",
    "PlayableLaunch",
    "PlayableLaunchDependencies",
    "PlayableLaunchError",
    "PlayableLifecycleError",
    "PlayableRuntimeConfig",
    "create_playable_launch",
    "main",
    "prove_owned_robotsimue_foreground",
    "prove_owned_robotsimue_window_presence",
    "run_playable_launch",
    "run_playable_vertical_slice",
]


if __name__ == "__main__":
    raise SystemExit(main())
