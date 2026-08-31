"""Safe access to artifacts declared by the native recording manifest.

The native recorder remains the catalog authority.  This module only resolves
one user-requested artifact after the native manifest has validated the session
identity and declared the relative artifact path.
"""

from __future__ import annotations

import stat
from pathlib import Path, PurePosixPath
from typing import Any

from gateway.services.recording import NativeRecordingError, NativeRecordingService

_TERMINAL_STATES = frozenset({"completed", "failed"})


def _artifact_error(code: str, message: str, status_code: int) -> NativeRecordingError:
    return NativeRecordingError(code, message, status_code=status_code)


def resolve_declared_artifact(
    service: NativeRecordingService,
    *,
    session_id: str,
    artifact_name: str,
) -> tuple[Path, dict[str, Any]]:
    """Return a regular file declared by one terminal native session."""
    session = service.manifest(session_id=session_id)
    state = str(session.get("state") or "")
    if state not in _TERMINAL_STATES:
        raise _artifact_error(
            "recording_not_terminal",
            "recording artifacts are available only after the session stops",
            409,
        )

    try:
        relative = PurePosixPath(str(artifact_name))
    except (TypeError, ValueError) as exc:
        raise _artifact_error(
            "recording_artifact_invalid", "recording artifact name is invalid", 422
        ) from exc
    if (
        not artifact_name
        or "\\" in artifact_name
        or "\x00" in artifact_name
        or relative.is_absolute()
        or not relative.parts
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise _artifact_error(
            "recording_artifact_invalid", "recording artifact must be a relative declared path", 422
        )
    normalized = relative.as_posix()

    declared = False
    for child in session.get("children", []):
        if not isinstance(child, dict):
            continue
        artifacts = child.get("artifacts")
        if not isinstance(artifacts, list):
            continue
        if any(str(item) == normalized for item in artifacts):
            declared = True
            break
    if not declared:
        raise _artifact_error(
            "recording_artifact_not_found", "recording artifact is not declared by the session", 404
        )

    normalized_session_id = service._validate_session_id(session_id)
    session_directory = service._root() / normalized_session_id
    artifact_path = session_directory.joinpath(*relative.parts)
    try:
        root = service._root().resolve(strict=True)
        session_root = session_directory.resolve(strict=True)
        resolved = artifact_path.resolve(strict=True)
    except FileNotFoundError as exc:
        raise _artifact_error(
            "recording_artifact_not_found", "recording artifact does not exist", 404
        ) from exc
    except OSError as exc:
        raise _artifact_error(
            "recording_artifact_unavailable", "recording artifact cannot be read", 503
        ) from exc

    try:
        resolved.relative_to(session_root)
    except ValueError as exc:
        raise _artifact_error(
            "recording_artifact_invalid", "recording artifact escaped its session directory", 422
        ) from exc
    if session_root.parent != root:
        raise _artifact_error(
            "recording_artifact_invalid", "recording artifact escaped its session directory", 422
        )
    current = session_root
    for part in relative.parts:
        try:
            current = current / part
            mode = current.lstat().st_mode
        except OSError as exc:
            raise _artifact_error(
                "recording_artifact_unavailable", "recording artifact cannot be inspected", 503
            ) from exc
        if stat.S_ISLNK(mode):
            raise _artifact_error(
                "recording_artifact_invalid", "recording artifact cannot be a symbolic link", 422
            )
    try:
        mode = resolved.stat().st_mode
        if not stat.S_ISREG(mode):
            raise _artifact_error(
                "recording_artifact_invalid", "recording artifact is not a regular file", 422
            )
    except OSError as exc:
        raise _artifact_error(
            "recording_artifact_unavailable", "recording artifact cannot be read", 503
        ) from exc
    return resolved, {"session_id": normalized_session_id, "path": normalized}
