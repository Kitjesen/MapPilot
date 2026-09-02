"""Run Build.bat or direct UBT under one owned process tree and project lock."""

# ruff: noqa: S603

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import subprocess
import sys
import time
from contextlib import AbstractContextManager
from pathlib import Path
from types import TracebackType
from typing import BinaryIO, Sequence

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim.runtime.process_owner import ProcessTreeOwner  # noqa: I001


DEFAULT_BUILD_TIMEOUT_S = 90.0 * 60.0
_LOCK_POLL_INTERVAL_S = 0.05
_OWNED_WORKER = "__owned_worker__"
_WORKER_MODE_BATCH = "batch"
_WORKER_MODE_DIRECT = "direct"
_WINDOWS_REPARSE_POINT = 0x400


class UnrealBuildError(RuntimeError):
    """Base error for an owned Unreal build launch."""


class UnrealBuildLockTimeoutError(UnrealBuildError):
    """Raised when another build owns the canonical project lock."""


class UnrealBuildTimeoutError(UnrealBuildError):
    """Raised when the complete lock-and-build deadline expires."""


def _positive_finite(value: float, *, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return number


def _canonical_file(path: Path, *, name: str, suffix: str | None = None) -> Path:
    try:
        canonical = path.expanduser().resolve(strict=True)
    except OSError as exc:
        raise ValueError(f"{name} is missing: {path}") from exc
    if not canonical.is_file():
        raise ValueError(f"{name} is not a file: {canonical}")
    if suffix is not None and canonical.suffix.casefold() != suffix.casefold():
        raise ValueError(f"{name} must end with {suffix}: {canonical}")
    return canonical


def _explicit_absolute_regular_file(
    path: Path,
    *,
    name: str,
    suffix: str | None = None,
) -> Path:
    candidate = _explicit_normalized_absolute(path, name=name)
    metadata = _no_follow_path_metadata(candidate, name=name)
    if metadata is None or not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"{name} is not a regular file: {candidate}")
    if suffix is not None and candidate.suffix.casefold() != suffix.casefold():
        raise ValueError(f"{name} must end with {suffix}: {candidate}")
    return candidate


def _explicit_absolute_log_path(path: Path) -> Path:
    candidate = _explicit_normalized_absolute(path, name="UBT log")
    parent = _explicit_absolute_directory(candidate.parent, name="UBT log parent")
    result = parent / candidate.name
    try:
        metadata = os.lstat(result)
    except FileNotFoundError:
        return result
    except OSError as exc:
        raise ValueError(f"cannot inspect UBT log path: {result}") from exc
    if _is_link_or_reparse(metadata):
        raise ValueError(f"UBT log path contains a link or reparse point: {result}")
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"UBT log path is not a regular file: {result}")
    return result


def _explicit_absolute_directory(path: Path, *, name: str) -> Path:
    candidate = _explicit_normalized_absolute(path, name=name)
    metadata = _no_follow_path_metadata(candidate, name=name)
    if metadata is None or not stat.S_ISDIR(metadata.st_mode):
        raise ValueError(f"{name} is not a directory: {candidate}")
    return candidate


def _explicit_normalized_absolute(path: Path, *, name: str) -> Path:
    candidate = Path(path)
    absolute = Path(os.path.abspath(os.fspath(candidate)))
    if not candidate.is_absolute() or candidate != absolute:
        raise ValueError(f"{name} must be an explicit normalized absolute path")
    return candidate


def _no_follow_path_metadata(path: Path, *, name: str) -> os.stat_result | None:
    current = Path(path.anchor)
    metadata: os.stat_result | None = None
    for part in path.parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except OSError as exc:
            raise ValueError(f"{name} is missing: {path}") from exc
        if _is_link_or_reparse(metadata):
            raise ValueError(f"{name} path contains a link or reparse point: {current}")
    return metadata


def _is_link_or_reparse(metadata: os.stat_result) -> bool:
    return stat.S_ISLNK(metadata.st_mode) or bool(
        getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT
    )


def _is_no_mutex(argument: str) -> bool:
    normalized = argument.strip().lstrip("-/").split("=", maxsplit=1)[0]
    return normalized.casefold() == "nomutex"


def _validate_build_arguments(arguments: Sequence[str]) -> tuple[str, ...]:
    result = tuple(str(argument) for argument in arguments)
    rejected = next((argument for argument in result if _is_no_mutex(argument)), None)
    if rejected is not None:
        raise ValueError(
            f"Unreal build argument {rejected!r} is forbidden; -WaitMutex is mandatory"
        )
    return result


def _argument_option(argument: str) -> str:
    return argument.strip().lstrip("-/").split("=", maxsplit=1)[0].casefold()


def _validate_build_identity(value: str, *, name: str) -> str:
    result = str(value)
    if (
        not result
        or result != result.strip()
        or result.startswith(("-", "/"))
        or "=" in result
        or any(character.isspace() for character in result)
        or "\0" in result
    ):
        raise ValueError(f"{name} must be an Unreal identity, not an option")
    return result


def _validate_runner_extra_arguments(arguments: Sequence[str]) -> tuple[str, ...]:
    result = _validate_build_arguments(arguments)
    reserved = next(
        (
            argument
            for argument in result
            if _argument_option(argument) in {"waitmutex", "nohotreloadfromide"}
        ),
        None,
    )
    if reserved is not None:
        raise ValueError(
            f"Unreal build argument {reserved!r} cannot override runner-owned arguments"
        )
    return result


def _validate_direct_ubt_extra_arguments(arguments: Sequence[str]) -> tuple[str, ...]:
    result = _validate_runner_extra_arguments(arguments)
    reserved = next(
        (
            argument
            for argument in result
            if _argument_option(argument)
            in {"project", "log", "rootdirectory", "session"}
        ),
        None,
    )
    if reserved is not None:
        raise ValueError(
            f"direct UBT argument {reserved!r} cannot override runner-owned arguments"
        )
    return result


def _direct_ubt_environment(
    *,
    engine_root: Path,
    user_settings_dir: Path,
) -> dict[str, str]:
    environment = {
        key: value
        for key, value in os.environ.items()
        if not key.casefold().startswith(("lingtu_unreal_", "lingtu_ubt_"))
    }
    canonical_engine_root = _explicit_absolute_directory(
        engine_root,
        name="UE engine root",
    )
    _explicit_absolute_regular_file(
        canonical_engine_root / "Engine" / "Build" / "Build.version",
        name="UE Build.version",
    )
    canonical_user_settings = _explicit_absolute_directory(
        user_settings_dir,
        name="UBT user settings directory",
    )
    environment["LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"] = str(
        canonical_engine_root
    )
    environment["LINGTU_UNREAL_USER_SETTING_DIRECTORY"] = str(
        canonical_user_settings
    )
    return environment


def project_lock_path(uproject: Path) -> Path:
    """Return the repository-local advisory lock for one canonical project."""

    canonical = _canonical_file(uproject, name="uproject", suffix=".uproject")
    identity = os.path.normcase(str(canonical)).encode("utf-8")
    digest = hashlib.sha256(identity).hexdigest()
    return REPO_ROOT / "build" / "locks" / "ue-build" / f"{digest}.lock"


class _ProjectBuildLock(AbstractContextManager["_ProjectBuildLock"]):
    def __init__(self, uproject: Path, *, deadline: float) -> None:
        self._uproject = uproject
        self._deadline = deadline
        self._path = project_lock_path(uproject)
        self._file: BinaryIO | None = None
        self._locked = False

    def __enter__(self) -> _ProjectBuildLock:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        lock_file = self._path.open("a+b", buffering=0)
        self._file = lock_file
        if lock_file.seek(0, os.SEEK_END) == 0:
            lock_file.write(b"\0")
        try:
            while True:
                lock_file.seek(0)
                try:
                    self._try_lock(lock_file)
                except OSError as exc:
                    if time.monotonic() >= self._deadline:
                        raise UnrealBuildLockTimeoutError(
                            "timed out waiting for the Unreal build lock for "
                            f"{self._uproject}; lock={self._path}"
                        ) from exc
                    time.sleep(
                        min(
                            _LOCK_POLL_INTERVAL_S,
                            max(0.0, self._deadline - time.monotonic()),
                        )
                    )
                    continue
                self._locked = True
                return self
        except BaseException:
            self._locked = False
            self._file = None
            lock_file.close()
            raise

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback
        lock_file = self._file
        self._file = None
        try:
            if lock_file is not None and self._locked:
                lock_file.seek(0)
                self._unlock(lock_file)
        finally:
            self._locked = False
            if lock_file is not None:
                lock_file.close()

    @staticmethod
    def _try_lock(lock_file: BinaryIO) -> None:
        if os.name == "nt":
            import msvcrt

            msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
            return
        if os.name == "posix":
            import fcntl

            fcntl.flock(  # type: ignore[attr-defined]
                lock_file.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,  # type: ignore[attr-defined]
            )
            return
        raise OSError(f"advisory file locking is unsupported on {os.name}")

    @staticmethod
    def _unlock(lock_file: BinaryIO) -> None:
        if os.name == "nt":
            import msvcrt

            msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
        elif os.name == "posix":
            import fcntl

            fcntl.flock(  # type: ignore[attr-defined]
                lock_file.fileno(),
                fcntl.LOCK_UN,  # type: ignore[attr-defined]
            )


def _worker_command(build_command: Sequence[str], *, mode: str) -> list[str]:
    payload = json.dumps(
        {"command": list(build_command), "mode": mode},
        ensure_ascii=True,
        separators=(",", ":"),
    )
    return [sys.executable, str(Path(__file__).resolve()), _OWNED_WORKER, payload]


def _quote_cmd_argument(argument: str) -> str:
    if any(character in argument for character in ('"', "\r", "\n", "\0", "%")):
        raise UnrealBuildError(
            "Unreal build arguments cannot contain quotes, newlines, NUL, or percent "
            f"characters on Windows: {argument!r}"
        )
    return f'"{argument}"'


def _run_owned_worker(payload: str) -> int:
    request = json.loads(payload)
    if not isinstance(request, dict):
        raise UnrealBuildError("owned Unreal build worker received an invalid request")
    command = request.get("command")
    mode = request.get("mode")
    if not isinstance(command, list) or not command or not all(
        isinstance(argument, str) and argument for argument in command
    ):
        raise UnrealBuildError("owned Unreal build worker received an invalid command")
    if mode not in {_WORKER_MODE_BATCH, _WORKER_MODE_DIRECT}:
        raise UnrealBuildError("owned Unreal build worker received an invalid launch mode")
    if sys.stdin.buffer.read(1) != b"\n":
        raise UnrealBuildError("owned Unreal build worker was not released by its owner")
    if mode == _WORKER_MODE_BATCH and os.name == "nt":
        comspec = os.environ.get("COMSPEC") or "cmd.exe"
        batch_command = " ".join(_quote_cmd_argument(argument) for argument in command)
        command_line = (
            f"{subprocess.list2cmdline([comspec])} /d /v:off /s /c call {batch_command}"
        )
        completed = subprocess.run(
            command_line,
            executable=comspec,
            check=False,
        )
    else:
        completed = subprocess.run(command, check=False, shell=False)
    return int(completed.returncode)


def _remaining(deadline: float) -> float:
    return max(0.0, deadline - time.monotonic())


def _cleanup_owned_process(
    owner: ProcessTreeOwner,
    process: subprocess.Popen[bytes],
) -> None:
    if process.poll() is None:
        owner.terminate(process, timeout_s=5.0)
    else:
        owner.close_after_exit()


def run_unreal_build(
    *,
    uproject: Path,
    build_bat: Path | None = None,
    ubt_dll: Path | None = None,
    dotnet: Path | None = None,
    log_path: Path | None = None,
    engine_root: Path | None = None,
    ubt_user_settings_dir: Path | None = None,
    timeout_s: float = DEFAULT_BUILD_TIMEOUT_S,
    target: str = "RobotSimUEEditor",
    platform: str = "Win64",
    configuration: str = "Development",
    extra_args: Sequence[str] = (),
) -> int:
    """Run one serialized Build.bat or direct UBT build and return its exit code."""

    timeout_s = _positive_finite(timeout_s, name="timeout_s")
    canonical_uproject = _canonical_file(
        uproject,
        name="uproject",
        suffix=".uproject",
    )
    if (build_bat is None) == (ubt_dll is None):
        raise ValueError("exactly one of build_bat or ubt_dll must be provided")
    target = _validate_build_identity(target, name="target")
    platform = _validate_build_identity(platform, name="platform")
    configuration = _validate_build_identity(configuration, name="configuration")
    if build_bat is not None:
        if any(
            value is not None
            for value in (dotnet, log_path, engine_root, ubt_user_settings_dir)
        ):
            raise ValueError(
                "dotnet, log_path, engine_root, and ubt_user_settings_dir are "
                "only valid with ubt_dll"
            )
        canonical_build_bat = _canonical_file(build_bat, name="Build.bat")
        validated_extra_args = list(_validate_runner_extra_arguments(extra_args))
        build_command = [
            str(canonical_build_bat),
            target,
            platform,
            configuration,
            str(canonical_uproject),
            "-WaitMutex",
            "-NoHotReloadFromIDE",
            *validated_extra_args,
        ]
        worker_mode = _WORKER_MODE_BATCH
        worker_environment = None
    else:
        if ubt_dll is None:
            raise UnrealBuildError("direct UBT mode lost its validated UBT DLL")
        if (
            dotnet is None
            or log_path is None
            or engine_root is None
            or ubt_user_settings_dir is None
        ):
            raise ValueError(
                "direct UBT mode requires explicit dotnet and log_path plus "
                "engine_root and ubt_user_settings_dir"
            )
        canonical_ubt_dll = _explicit_absolute_regular_file(
            ubt_dll,
            name="UBT DLL",
            suffix=".dll",
        )
        canonical_dotnet = _explicit_absolute_regular_file(dotnet, name="dotnet")
        canonical_log_path = _explicit_absolute_log_path(log_path)
        validated_extra_args = list(_validate_direct_ubt_extra_arguments(extra_args))
        build_command = [
            str(canonical_dotnet),
            str(canonical_ubt_dll),
            target,
            platform,
            configuration,
            f"-Project={canonical_uproject}",
            "-WaitMutex",
            "-NoHotReloadFromIDE",
            f"-Log={canonical_log_path}",
            *validated_extra_args,
        ]
        worker_mode = _WORKER_MODE_DIRECT
        worker_environment = _direct_ubt_environment(
            engine_root=engine_root,
            user_settings_dir=ubt_user_settings_dir,
        )
    deadline = time.monotonic() + timeout_s

    with _ProjectBuildLock(canonical_uproject, deadline=deadline):
        owner = ProcessTreeOwner()
        process: subprocess.Popen[bytes] | None = None
        try:
            process = subprocess.Popen(
                _worker_command(build_command, mode=worker_mode),
                stdin=subprocess.PIPE,
                env=worker_environment,
                **owner.popen_options(),
            )
            owner.attach(process)
            if process.stdin is None:
                raise UnrealBuildError("owned Unreal build worker has no release pipe")
            process.stdin.write(b"\n")
            process.stdin.flush()
            process.stdin.close()
            try:
                returncode = process.wait(timeout=_remaining(deadline))
            except subprocess.TimeoutExpired as exc:
                _cleanup_owned_process(owner, process)
                raise UnrealBuildTimeoutError(
                    f"Unreal build timed out after {timeout_s:g} seconds for {canonical_uproject}"
                ) from exc
            owner.close_after_exit()
            return int(returncode)
        except BaseException:
            if process is not None:
                _cleanup_owned_process(owner, process)
            raise
        finally:
            owner.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run Unreal Build.bat or direct dotnet + UBT DLL with tree ownership "
            "and a canonical project lock."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--build-bat", type=Path)
    source.add_argument("--ubt-dll", type=Path)
    parser.add_argument("--dotnet", type=Path)
    parser.add_argument("--log", dest="log_path", type=Path)
    parser.add_argument("--engine-root", type=Path)
    parser.add_argument("--ubt-user-settings-dir", type=Path)
    parser.add_argument("--uproject", type=Path, required=True)
    parser.add_argument("--target", default="RobotSimUEEditor")
    parser.add_argument("--platform", default="Win64")
    parser.add_argument("--configuration", default="Development")
    parser.add_argument("--timeout-seconds", type=float, default=DEFAULT_BUILD_TIMEOUT_S)
    parser.add_argument("extra_args", nargs=argparse.REMAINDER)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line Unreal build request."""

    args = _parser().parse_args(argv)
    extra_args = tuple(args.extra_args)
    if extra_args[:1] == ("--",):
        extra_args = extra_args[1:]
    try:
        return run_unreal_build(
            build_bat=args.build_bat,
            ubt_dll=args.ubt_dll,
            dotnet=args.dotnet,
            log_path=args.log_path,
            engine_root=args.engine_root,
            ubt_user_settings_dir=args.ubt_user_settings_dir,
            uproject=args.uproject,
            timeout_s=args.timeout_seconds,
            target=args.target,
            platform=args.platform,
            configuration=args.configuration,
            extra_args=extra_args,
        )
    except KeyboardInterrupt:
        print("Unreal build interrupted; owned process tree was closed", file=sys.stderr)
        return 130
    except (UnrealBuildError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == _OWNED_WORKER:
        raise SystemExit(_run_owned_worker(sys.argv[2]))
    raise SystemExit(main())


__all__ = [
    "DEFAULT_BUILD_TIMEOUT_S",
    "UnrealBuildError",
    "UnrealBuildLockTimeoutError",
    "UnrealBuildTimeoutError",
    "main",
    "project_lock_path",
    "run_unreal_build",
]
