"""Process adapter for the RobotSimUE visual runtime."""

from __future__ import annotations

import os
import re
import subprocess
from collections.abc import Callable, Mapping
from enum import Enum
from pathlib import Path, PureWindowsPath
from typing import TYPE_CHECKING, Any, TextIO, TypeAlias

from sim.runtime.process_owner import ProcessShutdownSnapshot, ProcessTreeOwner
from sim.runtime.windows_cpu_isolation import validate_windows_affinity_mask

from .coordinator import CoordinatorError, PhysicsPlan
from .run_allocation import RunAllocation

if TYPE_CHECKING:
    PopenFactory: TypeAlias = Callable[..., subprocess.Popen[Any]]
else:
    PopenFactory: TypeAlias = Callable[..., Any]

_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_CONTROL_INTENT_PORT = "control_intent_udp"
_CONTROL_STATUS_PORT = "control_status_udp"
_CONTROL_SOURCE_ID = "robotsimue.local_player.0"
_UE_SKIP_UBT_SDK_SETUP = "UE_SKIP_UBT_SDK_SETUP"
_WINDOWS_ABSOLUTE_PATH_RE = re.compile(r"^(?:[A-Za-z]:[\\/]|\\\\)")
_DEFAULT_MAIN_VIEW_SCREEN_PERCENTAGE = 100
_PLAYABLE_HUD_SCREENSHOTS: tuple[tuple[str, str], ...] = (
    ("LingTuHudDriveScreenshot", "hud-drive.png"),
    ("LingTuHudTacticalScreenshot", "hud-tactical.png"),
    ("LingTuHudMenuRecordingScreenshot", "hud-menu-recording.png"),
)


class UnrealLaunchProfile(Enum):
    """Select the operator-visible or automation-oriented Editor argv."""

    UNATTENDED = "unattended"
    INTERACTIVE = "interactive"
    INTERACTIVE_SDK_QUIET = "interactive_sdk_quiet"
    FOREGROUND_SDK_QUIET = "foreground_sdk_quiet"
    PLAYABLE_SDK_QUIET = "playable_sdk_quiet"


def _validate_map_name(map_name: str) -> str:
    if not isinstance(map_name, str) or not map_name.strip():
        raise ValueError("map_name must be a non-empty string")
    return map_name.strip()


def _validate_run_id(allocation: RunAllocation) -> str:
    run_id = allocation.run_id
    if not isinstance(run_id, str) or _RUN_ID_RE.fullmatch(run_id) is None:
        raise ValueError("allocation run_id must be a non-empty canonical run ID")
    return run_id


def _validate_snapshot_port(snapshot_port: int) -> None:
    if isinstance(snapshot_port, bool) or not isinstance(snapshot_port, int):
        raise ValueError("snapshot_port must be an integer in [1, 65535]")
    if snapshot_port < 1 or snapshot_port > 65535:
        raise ValueError("snapshot_port must be an integer in [1, 65535]")


def _validate_main_view_screen_percentage(value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("main_view_screen_percentage must be an integer")
    if not 50 <= value <= 100:
        raise ValueError("main_view_screen_percentage must be in [50, 100]")
    return value


def _main_view_exec_commands(*, capped: bool, screen_percentage: int) -> str | None:
    commands: list[str] = []
    if capped:
        commands.append("t.MaxFPS 30")
    if screen_percentage != _DEFAULT_MAIN_VIEW_SCREEN_PERCENTAGE:
        commands.extend(
            (
                "r.AntiAliasingMethod 4",
                "r.DynamicRes.OperationMode 0",
                f"r.ScreenPercentage {screen_percentage}",
            )
        )
    return ",".join(commands) if commands else None


def _validate_control_port(port: object, *, name: str) -> int:
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError(f"{name} control port must be an integer in [1, 65535]")
    return port


def _is_windows_absolute_path(value: Path | str) -> bool:
    return _WINDOWS_ABSOLUTE_PATH_RE.match(os.fspath(value)) is not None


def _command_path_value(value: Path | str) -> Path | PureWindowsPath:
    raw = os.fspath(value)
    if _is_windows_absolute_path(value):
        return PureWindowsPath(raw)
    return Path(value).resolve()


def _command_path(value: Path | str) -> str:
    return str(_command_path_value(value))


def _process_path(value: Path | str) -> Path:
    candidate = Path(value)
    if os.name != "nt" and _is_windows_absolute_path(value):
        return candidate
    return candidate.resolve()


def _playable_control_arguments(
    allocation: RunAllocation,
    *,
    snapshot_port: int,
) -> list[str]:
    """Return allocation-owned playable-control argv without implicit ports."""

    ports = getattr(allocation, "ports", None)
    if ports is None:
        return []
    if not isinstance(ports, Mapping):
        raise ValueError("allocation ports must be a mapping")
    allocated_snapshot_port = ports.get("visual_snapshot_udp")
    if allocated_snapshot_port is not None:
        if (
            isinstance(allocated_snapshot_port, bool)
            or not isinstance(allocated_snapshot_port, int)
            or not 1 <= allocated_snapshot_port <= 65535
        ):
            raise ValueError(
                "allocation visual snapshot port must be an integer in [1, 65535]"
            )
        if allocated_snapshot_port != snapshot_port:
            raise ValueError("snapshot_port does not match the RunAllocation")

    has_intent = _CONTROL_INTENT_PORT in ports
    has_status = _CONTROL_STATUS_PORT in ports
    if not has_intent and not has_status:
        return []
    if has_intent != has_status:
        raise ValueError(
            "control intent and control status ports must be allocated together"
        )

    intent_port = _validate_control_port(
        ports[_CONTROL_INTENT_PORT],
        name="intent",
    )
    status_port = _validate_control_port(
        ports[_CONTROL_STATUS_PORT],
        name="status",
    )
    if len({snapshot_port, intent_port, status_port}) != 3:
        raise ValueError(
            "visual snapshot, control intent, and control status ports must be distinct"
        )

    allocation_path = _command_path_value(allocation.path)
    screenshot_dir = allocation_path.parent / "screenshots"
    arguments = [
        f"-LingTuControlIntentPort={intent_port}",
        f"-LingTuControlStatusPort={status_port}",
        f"-LingTuControlSourceId={_CONTROL_SOURCE_ID}",
    ]
    arguments.extend(
        f"-{argument_name}={_command_path(screenshot_dir / filename)}"
        for argument_name, filename in _PLAYABLE_HUD_SCREENSHOTS
    )
    return arguments


def _validate_frame_capture(
    frame_capture_dir: Path | None,
    frame_capture_every: int,
    frame_capture_max: int,
) -> Path | PureWindowsPath | None:
    if frame_capture_dir is None:
        return None
    if not str(frame_capture_dir).strip():
        raise ValueError("frame capture directory must be non-empty")
    if (
        isinstance(frame_capture_every, bool)
        or not isinstance(frame_capture_every, int)
        or frame_capture_every < 1
    ):
        raise ValueError("frame capture interval must be a positive integer")
    if (
        isinstance(frame_capture_max, bool)
        or not isinstance(frame_capture_max, int)
        or frame_capture_max < 1
    ):
        raise ValueError("frame capture maximum must be a positive integer")
    return _command_path_value(frame_capture_dir)


def _validate_optional_text(value: str | None, label: str) -> str | None:
    if value is not None and not value.strip():
        raise ValueError(f"{label} must be non-empty")
    return value.strip() if value is not None else None


def _add_exception_note(error: BaseException, note: str) -> None:
    add_note = getattr(error, "add_note", None)
    if callable(add_note):
        add_note(note)
        return

    # BaseException.add_note was added in Python 3.11.  Preserve the same
    # diagnostic evidence for the Python 3.10 host used by LingTu.
    notes = list(getattr(error, "__notes__", ()))
    notes.append(note)
    try:
        error.__notes__ = notes
    except (AttributeError, TypeError):
        pass


class _UnrealProcessLifecycle:
    _popen_factory: PopenFactory
    _timeout_s: float
    _process: subprocess.Popen[Any] | None
    _process_owner: ProcessTreeOwner | None
    _stderr: TextIO | None
    _last_shutdown: ProcessShutdownSnapshot | None
    _affinity_mask: int | None

    @property
    def pid(self) -> int | None:
        """Return the Unreal child process ID while owned."""

        return self._process.pid if self._process is not None else None

    @property
    def last_shutdown(self) -> ProcessShutdownSnapshot | None:
        """Return immutable facts from the most recent owned process closure."""

        return self._last_shutdown

    def _launch(
        self,
        *,
        command: list[str],
        cwd: Path,
        allocation: RunAllocation,
        stderr_name: str,
    ) -> None:
        if self._process is not None and self._process.poll() is None:
            raise CoordinatorError("Unreal process is already running")
        self._last_shutdown = None
        self._stderr = (allocation.log_dir / stderr_name).open("w", encoding="utf-8")
        creationflags = (
            subprocess.CREATE_NO_WINDOW
            if os.name == "nt" and hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        try:
            self._process_owner = (
                ProcessTreeOwner(affinity_mask=self._affinity_mask)
                if self._affinity_mask is not None
                else ProcessTreeOwner()
            )
            self._process = self._popen_factory(
                command,
                cwd=cwd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                shell=False,
                env=self._child_environment(allocation),
                **self._process_owner.popen_options(creationflags=creationflags),
            )
            self._process_owner.attach(self._process)
        except BaseException as launch_error:
            process = self._process
            owner = self._process_owner
            if owner is not None:
                try:
                    if process is not None:
                        owner.terminate(process, timeout_s=self._timeout_s)
                    else:
                        owner.close()
                except BaseException as cleanup_error:
                    _add_exception_note(
                        launch_error,
                        f"Unreal process-owner cleanup also failed: {cleanup_error}",
                    )
                    try:
                        owner.close()
                    except BaseException as close_error:
                        _add_exception_note(
                            launch_error,
                            f"Unreal process-owner close also failed: {close_error}",
                        )
            if process is not None and process.poll() is None:
                try:
                    process.terminate()
                    try:
                        process.wait(timeout=self._timeout_s)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=self._timeout_s)
                except BaseException as direct_error:
                    _add_exception_note(
                        launch_error,
                        f"Unreal direct-child cleanup also failed: {direct_error}",
                    )
            self._process = None
            self._process_owner = None
            try:
                self._close_stderr()
            except BaseException as stderr_error:
                _add_exception_note(
                    launch_error,
                    f"Unreal stderr cleanup also failed: {stderr_error}",
                )
            raise

    def _child_environment(self, allocation: RunAllocation) -> dict[str, str]:
        return dict(allocation.child_environment())

    def poll(self) -> int | None:
        """Return the process exit code when Unreal has exited."""

        return self._process.poll() if self._process is not None else None

    def terminate(self) -> None:
        """Quiesce Unreal and release process resources."""

        process = self._process
        if process is None:
            self._close_stderr()
            return
        pid = process.pid
        natural = False
        owner_closed = False
        primary_error: BaseException | None = None
        cleanup_errors: list[BaseException] = []
        try:
            natural = process.poll() is not None
            if self._process_owner is not None:
                self._process_owner.terminate(process, timeout_s=self._timeout_s)
            elif process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=self._timeout_s)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=self._timeout_s)
        except BaseException as error:
            primary_error = error
        if self._process_owner is not None:
            try:
                self._process_owner.close_after_exit()
                owner_closed = True
            except BaseException as error:
                cleanup_errors.append(error)
            finally:
                self._process_owner = None
        exit_code: int | None = None
        try:
            exit_code = process.poll()
        except BaseException as error:
            cleanup_errors.append(error)
        self._last_shutdown = ProcessShutdownSnapshot(
            pid=pid,
            exit_code=exit_code,
            direct_child_running_after_close=exit_code is None,
            process_owner_closed=owner_closed,
            termination_mode="natural" if natural else "owned_terminate",
        )
        self._process = None
        try:
            self._close_stderr()
        except BaseException as error:
            cleanup_errors.append(error)
        if primary_error is not None:
            for cleanup_error in cleanup_errors:
                _add_exception_note(
                    primary_error,
                    f"Unreal terminate cleanup also failed: {cleanup_error}",
                )
            raise primary_error
        if cleanup_errors:
            error = cleanup_errors[0]
            for additional in cleanup_errors[1:]:
                _add_exception_note(
                    error,
                    f"additional Unreal terminate cleanup failure: {additional}",
                )
            raise error

    def _close_stderr(self) -> None:
        stream = self._stderr
        self._stderr = None
        if stream is not None:
            stream.close()


class UnrealProcess(_UnrealProcessLifecycle):
    """Launch and quiesce one Unreal Editor game session without a shell."""

    def __init__(
        self,
        editor_executable: Path,
        uproject: Path,
        map_name: str,
        *,
        frame_capture_dir: Path | None = None,
        frame_capture_every: int = 1,
        frame_capture_max: int = 1,
        session_camera_tag: str | None = None,
        motion_camera_stable_id: str | None = None,
        depth_capture_in_main_renderer: bool = False,
        shared_color_depth_capture: bool = False,
        main_view_screen_percentage: int = _DEFAULT_MAIN_VIEW_SCREEN_PERCENTAGE,
        launch_profile: UnrealLaunchProfile = UnrealLaunchProfile.UNATTENDED,
        popen_factory: PopenFactory = subprocess.Popen,
        timeout_s: float = 10.0,
        affinity_mask: int | None = None,
    ) -> None:
        if not isinstance(launch_profile, UnrealLaunchProfile):
            raise TypeError("launch_profile must be UnrealLaunchProfile")
        self._editor = _process_path(editor_executable)
        self._uproject = _process_path(uproject)
        self._map_name = _validate_map_name(map_name)
        self._frame_capture_dir = _validate_frame_capture(
            frame_capture_dir,
            frame_capture_every,
            frame_capture_max,
        )
        self._frame_capture_every = frame_capture_every
        self._frame_capture_max = frame_capture_max
        self._session_camera_tag = _validate_optional_text(
            session_camera_tag,
            "session camera tag",
        )
        self._motion_camera_stable_id = _validate_optional_text(
            motion_camera_stable_id,
            "motion camera stable ID",
        )
        if not isinstance(depth_capture_in_main_renderer, bool):
            raise TypeError("depth_capture_in_main_renderer must be boolean")
        self._depth_capture_in_main_renderer = depth_capture_in_main_renderer
        if not isinstance(shared_color_depth_capture, bool):
            raise TypeError("shared_color_depth_capture must be boolean")
        self._shared_color_depth_capture = shared_color_depth_capture
        self._main_view_screen_percentage = _validate_main_view_screen_percentage(
            main_view_screen_percentage
        )
        self._launch_profile = launch_profile
        self._popen_factory = popen_factory
        self._timeout_s = timeout_s
        self._affinity_mask = (
            validate_windows_affinity_mask(affinity_mask, "Unreal affinity mask")
            if affinity_mask is not None
            else None
        )
        self._process: subprocess.Popen[Any] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: TextIO | None = None
        self._last_shutdown: ProcessShutdownSnapshot | None = None

    def command(
        self,
        *,
        bundle_dir: Path,
        allocation: RunAllocation,
        snapshot_port: int,
        model_generation: int,
        reset_generation: int,
    ) -> list[str]:
        """Build the exact argv used for RobotSimUE."""

        _validate_snapshot_port(snapshot_port)
        run_id = _validate_run_id(allocation)
        ddc_path = self._ddc_path(allocation)
        if self._launch_profile in {
            UnrealLaunchProfile.INTERACTIVE,
            UnrealLaunchProfile.INTERACTIVE_SDK_QUIET,
        }:
            exec_commands = _main_view_exec_commands(
                capped=True,
                screen_percentage=self._main_view_screen_percentage,
            )
            launch_profile_arguments = ["-NoSound", f"-ExecCmds={exec_commands}"]
        elif self._launch_profile is UnrealLaunchProfile.FOREGROUND_SDK_QUIET:
            # Keep the Editor startup non-interactive for Turnkey/SDK prompts,
            # but leave OS input enabled for a real foreground game window.
            launch_profile_arguments = [
                "-unattended",
                "-LingTuRuntimeUI",
                "-NoSound",
                f"-ExecCmds={_main_view_exec_commands(capped=True, screen_percentage=self._main_view_screen_percentage)}",
            ]
        elif self._launch_profile is UnrealLaunchProfile.PLAYABLE_SDK_QUIET:
            # The child env skips AutoSDK/UBT setup; unattended skips Turnkey
            # and prompts, so input and LingTu UI are restored explicitly.
            launch_profile_arguments = [
                "-unattended",
                "-UnattendedInput",
                "-LingTuRuntimeUI",
                "-NoSound",
                f"-ExecCmds={_main_view_exec_commands(capped=True, screen_percentage=self._main_view_screen_percentage)}",
            ]
        else:
            launch_profile_arguments = ["-unattended"]
            exec_commands = _main_view_exec_commands(
                capped=False,
                screen_percentage=self._main_view_screen_percentage,
            )
            if exec_commands is not None:
                launch_profile_arguments.append(f"-ExecCmds={exec_commands}")
        command = [
            _command_path(self._editor),
            _command_path(self._uproject),
            self._map_name,
            "-game",
            "-log",
            f"-abslog={_command_path(Path(allocation.log_dir) / 'Unreal.log')}",
            *launch_profile_arguments,
            "-nop4",
            "-NoSplash",
            "-NoCompile",
            "-DDC=InstalledNoZenLocalFallback",
            f"-LocalDataCachePath={_command_path(ddc_path)}",
            "-windowed",
            "-ResX=1920",
            "-ResY=1080",
            "-ForceRes",
            f"-LingTuBundle={_command_path(bundle_dir)}",
            f"-LingTuRunAllocation={_command_path(allocation.path)}",
            f"-LingTuRunId={run_id}",
            f"-LingTuArtifactRoot={_command_path(allocation.artifact_root)}",
            f"-LingTuSnapshotPort={snapshot_port}",
            f"-LingTuModelGeneration={model_generation}",
            f"-LingTuResetGeneration={reset_generation}",
            f"-LingTuScreenshot={_command_path(Path(allocation.log_dir) / 'visual-first-frame.png')}",
        ]
        command.extend(
            _playable_control_arguments(
                allocation,
                snapshot_port=snapshot_port,
            )
        )
        if self._frame_capture_dir is not None:
            command.extend(
                (
                    f"-LingTuFrameCaptureDir={_command_path(self._frame_capture_dir)}",
                    f"-LingTuFrameCaptureEvery={self._frame_capture_every}",
                    f"-LingTuFrameCaptureMax={self._frame_capture_max}",
                )
            )
        if self._session_camera_tag is not None:
            command.append(f"-LingTuSessionCameraTag={self._session_camera_tag}")
        if self._motion_camera_stable_id is not None:
            command.append(
                f"-LingTuMotionCameraStableId={self._motion_camera_stable_id}"
            )
        if self._depth_capture_in_main_renderer:
            command.append("-LingTuDepthCaptureInMainRenderer")
        if self._shared_color_depth_capture:
            command.append("-LingTuSharedColorDepthCapture")
        return command

    def _child_environment(self, allocation: RunAllocation) -> dict[str, str]:
        environment = super()._child_environment(allocation)
        if self._launch_profile in {
            UnrealLaunchProfile.INTERACTIVE_SDK_QUIET,
            UnrealLaunchProfile.FOREGROUND_SDK_QUIET,
            UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
        }:
            # SDK-quiet profiles are runtime-only and must not start Editor
            # ValidatePlatforms; build/package entry points own SDK preflight.
            for name in tuple(environment):
                if name.casefold() == _UE_SKIP_UBT_SDK_SETUP.casefold():
                    environment.pop(name)
            environment[_UE_SKIP_UBT_SDK_SETUP] = "1"
        return environment

    def start(
        self,
        *,
        bundle_dir: Path,
        allocation: RunAllocation,
        plan: PhysicsPlan,
        snapshot_port: int,
    ) -> None:
        """Launch Unreal Editor in game mode for one prepared allocation."""

        if self._process is not None and self._process.poll() is None:
            raise CoordinatorError("Unreal process is already running")
        if not self._editor.is_file():
            raise CoordinatorError(f"UnrealEditor.exe does not exist: {self._editor}")
        if not self._uproject.is_file():
            raise CoordinatorError(f"Unreal project does not exist: {self._uproject}")
        command = self.command(
            bundle_dir=bundle_dir,
            allocation=allocation,
            snapshot_port=snapshot_port,
            model_generation=plan.model_generation,
            reset_generation=plan.reset_generation,
        )
        Path(os.fspath(self._ddc_path(allocation))).mkdir(
            parents=True,
            exist_ok=True,
        )
        self._launch(
            command=command,
            cwd=plan.repo_root,
            allocation=allocation,
            stderr_name="unreal.stderr.log",
        )

    @staticmethod
    def _ddc_path(allocation: RunAllocation) -> Path | PureWindowsPath:
        """Return the writable, Zen-free cache shared by local UE runs."""
        return _command_path_value(
            Path(allocation.artifact_root) / "build" / "unreal-ddc"
        )


class PackagedUnrealProcess(_UnrealProcessLifecycle):
    """Launch and quiesce one trusted packaged RobotSimUE session without a shell."""

    def __init__(
        self,
        executable: Path,
        map_name: str,
        *,
        frame_capture_dir: Path | None = None,
        frame_capture_every: int = 1,
        frame_capture_max: int = 1,
        session_camera_tag: str | None = None,
        motion_camera_stable_id: str | None = None,
        depth_capture_in_main_renderer: bool = False,
        shared_color_depth_capture: bool = False,
        main_view_screen_percentage: int = _DEFAULT_MAIN_VIEW_SCREEN_PERCENTAGE,
        popen_factory: PopenFactory = subprocess.Popen,
        timeout_s: float = 10.0,
        affinity_mask: int | None = None,
    ) -> None:
        self._executable = _process_path(executable)
        if self._executable.name.lower() != "robotsimue-win64-release.exe":
            raise ValueError(
                "packaged Unreal runtime must be RobotSimUE-Win64-Release.exe; the "
                "short-lived RobotSimUE.exe bootstrap and build-configuration "
                "binary cannot be process-owned"
            )
        self._map_name = _validate_map_name(map_name)
        self._frame_capture_dir = _validate_frame_capture(
            frame_capture_dir,
            frame_capture_every,
            frame_capture_max,
        )
        self._frame_capture_every = frame_capture_every
        self._frame_capture_max = frame_capture_max
        self._session_camera_tag = _validate_optional_text(
            session_camera_tag,
            "session camera tag",
        )
        self._motion_camera_stable_id = _validate_optional_text(
            motion_camera_stable_id,
            "motion camera stable ID",
        )
        if not isinstance(depth_capture_in_main_renderer, bool):
            raise TypeError("depth_capture_in_main_renderer must be boolean")
        self._depth_capture_in_main_renderer = depth_capture_in_main_renderer
        if not isinstance(shared_color_depth_capture, bool):
            raise TypeError("shared_color_depth_capture must be boolean")
        self._shared_color_depth_capture = shared_color_depth_capture
        self._main_view_screen_percentage = _validate_main_view_screen_percentage(
            main_view_screen_percentage
        )
        self._popen_factory = popen_factory
        self._timeout_s = timeout_s
        self._affinity_mask = (
            validate_windows_affinity_mask(affinity_mask, "Unreal affinity mask")
            if affinity_mask is not None
            else None
        )
        self._process: subprocess.Popen[Any] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: TextIO | None = None
        self._last_shutdown: ProcessShutdownSnapshot | None = None

    def command(
        self,
        *,
        bundle_dir: Path,
        allocation: RunAllocation,
        snapshot_port: int,
        model_generation: int,
        reset_generation: int,
    ) -> list[str]:
        """Build the exact argv used for a packaged RobotSimUE executable."""

        _validate_snapshot_port(snapshot_port)
        run_id = _validate_run_id(allocation)
        command = [
            _command_path(self._executable),
            self._map_name,
            "-log",
            f"-abslog={_command_path(Path(allocation.log_dir) / 'RobotSimUE.log')}",
            "-unattended",
            "-NoSplash",
            "-windowed",
            "-ResX=1920",
            "-ResY=1080",
            "-ForceRes",
            f"-LingTuBundle={_command_path(bundle_dir)}",
            f"-LingTuRunAllocation={_command_path(allocation.path)}",
            f"-LingTuRunId={run_id}",
            f"-LingTuArtifactRoot={_command_path(allocation.artifact_root)}",
            f"-LingTuSnapshotPort={snapshot_port}",
            f"-LingTuModelGeneration={model_generation}",
            f"-LingTuResetGeneration={reset_generation}",
            f"-LingTuScreenshot={_command_path(Path(allocation.log_dir) / 'visual-first-frame.png')}",
        ]
        exec_commands = _main_view_exec_commands(
            capped=False,
            screen_percentage=self._main_view_screen_percentage,
        )
        if exec_commands is not None:
            command.append(f"-ExecCmds={exec_commands}")
        command.extend(
            _playable_control_arguments(
                allocation,
                snapshot_port=snapshot_port,
            )
        )
        if self._frame_capture_dir is not None:
            command.extend(
                (
                    f"-LingTuFrameCaptureDir={_command_path(self._frame_capture_dir)}",
                    f"-LingTuFrameCaptureEvery={self._frame_capture_every}",
                    f"-LingTuFrameCaptureMax={self._frame_capture_max}",
                )
            )
        if self._session_camera_tag is not None:
            command.append(f"-LingTuSessionCameraTag={self._session_camera_tag}")
        if self._motion_camera_stable_id is not None:
            command.append(
                f"-LingTuMotionCameraStableId={self._motion_camera_stable_id}"
            )
        if self._depth_capture_in_main_renderer:
            command.append("-LingTuDepthCaptureInMainRenderer")
        if self._shared_color_depth_capture:
            command.append("-LingTuSharedColorDepthCapture")
        return command

    def start(
        self,
        *,
        bundle_dir: Path,
        allocation: RunAllocation,
        plan: PhysicsPlan,
        snapshot_port: int,
    ) -> None:
        """Launch packaged RobotSimUE for one prepared allocation."""

        if not self._executable.is_file():
            raise CoordinatorError(f"RobotSimUE.exe does not exist: {self._executable}")
        command = self.command(
            bundle_dir=bundle_dir,
            allocation=allocation,
            snapshot_port=snapshot_port,
            model_generation=plan.model_generation,
            reset_generation=plan.reset_generation,
        )
        self._launch(
            command=command,
            cwd=self._executable.parent,
            allocation=allocation,
            stderr_name="robotsimue.stderr.log",
        )
