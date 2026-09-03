"""Plan deterministic Blender conditioning for generated static visual props.

The plan deliberately separates the Unreal visual facet from the MuJoCo
collision facet.  It is a quarantined build instruction, not permission to
spawn the generated render mesh in a production world.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import math
import os
import re
import stat
import struct
import sys
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any

PLAN_SCHEMA = "lingtu.sim.static-prop-conditioning-plan.v1"
CONDITIONER_CONTRACT = "lingtu.sim.static-prop-conditioner.v1"
MAX_SOURCE_GLB_BYTES = 512 * 1024 * 1024
MAX_TASK_JSON_BYTES = 1024 * 1024
PROFILE_PATH = Path(__file__).with_name("tripo_hero_static_pbr.v1.json")


def _open_link_free_file(path: Path) -> int:
    """Open an absolute file without following any path component."""

    candidate = Path(os.path.abspath(os.fspath(path)))
    if os.name != "nt":
        directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        directory_fd = os.open(candidate.anchor, directory_flags)
        try:
            for component in candidate.parts[1:-1]:
                next_fd = os.open(component, directory_flags, dir_fd=directory_fd)
                os.close(directory_fd)
                directory_fd = next_fd
            return os.open(
                candidate.name,
                os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=directory_fd,
            )
        finally:
            os.close(directory_fd)

    import msvcrt

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_file = kernel32.CreateFileW
    create_file.argtypes = [
        ctypes.c_wchar_p,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.c_void_p,
    ]
    create_file.restype = ctypes.c_void_p
    handle = create_file(
        os.fspath(candidate),
        0x80000000,
        0x00000001 | 0x00000002 | 0x00000004,
        None,
        3,
        0x00200000,
        None,
    )
    invalid_handle = ctypes.c_void_p(-1).value
    if handle == invalid_handle:
        error = ctypes.get_last_error()
        raise OSError(error, "CreateFileW failed", candidate)

    class FileAttributeTagInfo(ctypes.Structure):
        _fields_ = [("attributes", ctypes.c_uint32), ("reparse_tag", ctypes.c_uint32)]

    info = FileAttributeTagInfo()
    get_info = kernel32.GetFileInformationByHandleEx
    get_info.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32]
    get_info.restype = ctypes.c_int
    if not get_info(handle, 9, ctypes.byref(info), ctypes.sizeof(info)):
        error = ctypes.get_last_error()
        kernel32.CloseHandle(handle)
        raise OSError(error, "GetFileInformationByHandleEx failed", candidate)
    if info.attributes & 0x00000400:
        kernel32.CloseHandle(handle)
        raise ValueError("file path must not end in a reparse point")
    final_path = ctypes.create_unicode_buffer(32768)
    get_final_path = kernel32.GetFinalPathNameByHandleW
    get_final_path.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32]
    get_final_path.restype = ctypes.c_uint32
    length = get_final_path(handle, final_path, len(final_path), 0)
    if not length or length >= len(final_path):
        error = ctypes.get_last_error()
        kernel32.CloseHandle(handle)
        raise OSError(error, "GetFinalPathNameByHandleW failed", candidate)
    observed = final_path.value.removeprefix("\\\\?\\")
    expected = os.path.abspath(os.fspath(candidate))
    if os.path.normcase(observed) != os.path.normcase(expected):
        kernel32.CloseHandle(handle)
        raise ValueError("file path changed through a reparse point")
    return msvcrt.open_osfhandle(handle, os.O_RDONLY | getattr(os, "O_BINARY", 0))


def _directory_identity(path: Path) -> tuple[int, int]:
    """Return a stable directory identity from an opened directory handle."""

    candidate = Path(os.path.abspath(os.fspath(path)))
    if os.name != "nt":
        descriptor = os.open(
            candidate,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
        )
        try:
            metadata = os.fstat(descriptor)
            return metadata.st_dev, metadata.st_ino
        finally:
            os.close(descriptor)

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_file = kernel32.CreateFileW
    create_file.argtypes = [
        ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
    ]
    create_file.restype = ctypes.c_void_p
    handle = create_file(
        os.fspath(candidate), 0x80000000,
        0x00000001 | 0x00000002 | 0x00000004,
        None, 3, 0x00200000 | 0x02000000, None,
    )
    if handle == ctypes.c_void_p(-1).value:
        error = ctypes.get_last_error()
        raise OSError(error, "CreateFileW directory open failed", candidate)

    class ByHandleFileInformation(ctypes.Structure):
        _fields_ = [
            ("attributes", ctypes.c_uint32), ("creation_low", ctypes.c_uint32),
            ("creation_high", ctypes.c_uint32), ("access_low", ctypes.c_uint32),
            ("access_high", ctypes.c_uint32), ("write_low", ctypes.c_uint32),
            ("write_high", ctypes.c_uint32), ("volume_serial", ctypes.c_uint32),
            ("file_size_high", ctypes.c_uint32), ("file_size_low", ctypes.c_uint32),
            ("links", ctypes.c_uint32), ("file_index_high", ctypes.c_uint32),
            ("file_index_low", ctypes.c_uint32),
        ]

    info = ByHandleFileInformation()
    try:
        get_info = kernel32.GetFileInformationByHandle
        get_info.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        get_info.restype = ctypes.c_int
        if not get_info(handle, ctypes.byref(info)):
            error = ctypes.get_last_error()
            raise OSError(error, "GetFileInformationByHandle failed", candidate)
        if info.attributes & 0x00000400:
            raise ValueError("trusted directory must not be a reparse point")
        return info.volume_serial, (info.file_index_high << 32) | info.file_index_low
    finally:
        kernel32.CloseHandle(handle)


def _open_file_with_parent_identity(path: Path) -> tuple[int, tuple[int, int]]:
    candidate = Path(os.path.abspath(os.fspath(path)))
    before = _directory_identity(candidate.parent)
    descriptor = _open_link_free_file(candidate)
    after = _directory_identity(candidate.parent)
    if after != before:
        os.close(descriptor)
        raise ValueError("file parent directory changed while opening the file")
    return descriptor, before


def _assert_directory_identity(path: Path, expected: tuple[int, int]) -> None:
    if _directory_identity(path) != expected:
        raise RuntimeError("trusted plan parent directory identity changed")


class _BoundDirectoryPublication:
    def __init__(self, trusted_root: "_TrustedDirectory", source: Path) -> None:
        self._trusted_root = trusted_root
        self._source_relative = source.relative_to(trusted_root.path).as_posix()
        self._source_handle: int | None = None
        if os.name == "nt":
            self._source_handle = trusted_root._open_windows_directory(  # noqa: SLF001
                source, desired_access=0x00010000 | 0x00100000
            )

    def close(self) -> None:
        if self._source_handle is not None:
            ctypes.WinDLL("kernel32", use_last_error=True).CloseHandle(self._source_handle)
            self._source_handle = None

    def publish(self, target_relative: str) -> None:
        if os.name != "nt":
            self._trusted_root._renameat2(self._source_relative, target_relative)  # noqa: SLF001
            return
        if self._source_handle is None:
            raise RuntimeError("bound Windows publication source is closed")
        name = target_relative.encode("utf-16-le")

        class FileRenameInfo(ctypes.Structure):
            _fields_ = [
                ("replace_if_exists", ctypes.c_ubyte),
                ("root_directory", ctypes.c_void_p),
                ("file_name_length", ctypes.c_uint32),
                ("file_name", ctypes.c_wchar * 1),
            ]

        name_offset = FileRenameInfo.file_name.offset
        buffer = ctypes.create_string_buffer(name_offset + len(name) + 2)
        header = FileRenameInfo.from_buffer(buffer)
        header.replace_if_exists = 0
        header.root_directory = self._trusted_root.handle
        header.file_name_length = len(name)
        ctypes.memmove(ctypes.addressof(buffer) + name_offset, name, len(name))
        class IoStatusBlock(ctypes.Structure):
            _fields_ = [("status", ctypes.c_void_p), ("information", ctypes.c_size_t)]

        io_status = IoStatusBlock()
        ntdll = ctypes.WinDLL("ntdll")
        rename = ntdll.NtSetInformationFile
        rename.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(IoStatusBlock),
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_int,
        ]
        rename.restype = ctypes.c_long
        status = rename(self._source_handle, ctypes.byref(io_status), buffer, len(buffer), 10)
        if status != 0:
            unsigned_status = ctypes.c_uint32(status).value
            if unsigned_status == 0xC0000035:
                raise FileExistsError(target_relative)
            rtl_error = ntdll.RtlNtStatusToDosError
            rtl_error.argtypes = [ctypes.c_long]
            rtl_error.restype = ctypes.c_uint32
            error = rtl_error(status)
            raise OSError(error, "handle-relative NtSetInformationFile failed", target_relative)


class _TrustedDirectory:
    def __init__(self, path: Path, expected: tuple[int, int]) -> None:
        self.path = Path(os.path.abspath(os.fspath(path)))
        self.identity = expected
        self.handle: int | None = None
        self._fd: int | None = None
        if os.name == "nt":
            self.handle = self._open_windows_directory(self.path, desired_access=0x80000000)
            observed = _directory_identity(self.path)
        else:
            self._fd = os.open(
                self.path,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
            )
            metadata = os.fstat(self._fd)
            observed = (metadata.st_dev, metadata.st_ino)
        if observed != expected:
            self.close()
            raise RuntimeError("trusted plan parent changed before its handle was retained")

    @staticmethod
    def _open_windows_directory(path: Path, *, desired_access: int) -> int:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        create_file = kernel32.CreateFileW
        create_file.argtypes = [
            ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ]
        create_file.restype = ctypes.c_void_p
        handle = create_file(
            os.fspath(path), desired_access,
            0x00000001 | 0x00000002 | 0x00000004,
            None, 3, 0x00200000 | 0x02000000, None,
        )
        if handle == ctypes.c_void_p(-1).value:
            error = ctypes.get_last_error()
            raise OSError(error, "CreateFileW directory binding failed", path)
        return int(handle)

    def _renameat2(self, source_relative: str, target_relative: str) -> None:
        if self._fd is None:
            raise RuntimeError("trusted POSIX root handle is closed")
        libc = ctypes.CDLL(None, use_errno=True)
        renameat2 = getattr(libc, "renameat2", None)
        if renameat2 is None:
            raise RuntimeError("renameat2(RENAME_NOREPLACE) is unavailable")
        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        if renameat2(
            self._fd, os.fsencode(source_relative), self._fd, os.fsencode(target_relative), 1
        ) != 0:
            error = ctypes.get_errno()
            if error == 17:
                raise FileExistsError(target_relative)
            raise OSError(error, "handle-relative renameat2 failed", target_relative)

    def bind_publication(self, source: Path) -> _BoundDirectoryPublication:
        return _BoundDirectoryPublication(self, source)

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        if self.handle is not None:
            ctypes.WinDLL("kernel32", use_last_error=True).CloseHandle(self.handle)
            self.handle = None
_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_AXES = {"x", "y", "z"}
_LOD_RATIOS: tuple[tuple[str, float], ...] = (
    ("LOD0", 1.0),
    ("LOD1", 0.5),
    ("LOD2", 0.2),
)


def topology_requires_review(topology: Mapping[str, Any]) -> bool:
    """Return whether post-weld topology contains real mesh defects.

    Blender may split coincident vertices at UV or material seams.  Callers
    must therefore provide counts from a position-welded analysis copy, not
    the untouched render mesh.
    """

    defect_counts: list[int] = []
    for field in ("boundary_edges", "non_manifold_edges", "wire_edges"):
        value = topology.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"topology.{field} must be a non-negative integer")
        defect_counts.append(value)
    return any(defect_counts)


def _digest_document(value: Any) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_bound_file(path: Path, *, maximum_bytes: int, context: str) -> bytes:
    """Read one link-free file through the same handle used for identity checks."""

    candidate = _absolute_link_free_path(path, context)
    descriptor, _parent_identity = _open_file_with_parent_identity(candidate)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ValueError(f"{context} must identify one non-empty regular file")
        if before.st_size > maximum_bytes:
            raise ValueError(f"{context} exceeds the physical byte budget")
        chunks: list[bytes] = []
        remaining = maximum_bytes + 1
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            if not block:
                break
            chunks.append(block)
            remaining -= len(block)
        payload = b"".join(chunks)
        after = os.fstat(descriptor)
        if len(payload) != before.st_size or (before.st_dev, before.st_ino) != (
            after.st_dev,
            after.st_ino,
        ):
            raise ValueError(f"{context} changed while it was being read")
        return payload
    finally:
        os.close(descriptor)


def _validate_self_contained_glb(payload: bytes, context: str = "source model") -> None:
    """Validate a GLB v2 container and reject every external resource URI."""

    if len(payload) < 20:
        raise ValueError(f"{context} is not a complete GLB v2 file")
    magic, version, declared_length = struct.unpack_from("<4sII", payload)
    if magic != b"glTF" or version != 2 or declared_length != len(payload):
        raise ValueError(f"{context} has an invalid GLB v2 header")
    offset = 12
    chunks: list[tuple[int, bytes]] = []
    while offset < len(payload):
        if offset + 8 > len(payload):
            raise ValueError(f"{context} has a truncated GLB chunk header")
        length, chunk_type = struct.unpack_from("<II", payload, offset)
        offset += 8
        if length % 4 or offset + length > len(payload):
            raise ValueError(f"{context} has an invalid GLB chunk length")
        chunks.append((chunk_type, payload[offset : offset + length]))
        offset += length
    if not chunks or chunks[0][0] != 0x4E4F534A:
        raise ValueError(f"{context} must begin with exactly one JSON chunk")
    if sum(kind == 0x4E4F534A for kind, _ in chunks) != 1:
        raise ValueError(f"{context} must contain exactly one JSON chunk")
    if any(kind not in {0x4E4F534A, 0x004E4942} for kind, _ in chunks):
        raise ValueError(f"{context} contains an unsupported GLB chunk")
    if sum(kind == 0x004E4942 for kind, _ in chunks) > 1:
        raise ValueError(f"{context} contains multiple BIN chunks")
    try:
        document = json.loads(chunks[0][1].rstrip(b" \t\r\n\x00").decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{context} contains invalid GLB JSON") from exc
    if not isinstance(document, Mapping):
        raise ValueError(f"{context} GLB JSON must be one object")
    asset = document.get("asset")
    if not isinstance(asset, Mapping) or asset.get("version") != "2.0":
        raise ValueError(f"{context} GLB asset.version must be 2.0")
    for collection in ("buffers", "images"):
        records = document.get(collection, [])
        if not isinstance(records, list):
            raise ValueError(f"{context} {collection} must be one array")
        for record in records:
            if not isinstance(record, Mapping) or "uri" in record:
                raise ValueError(f"{context} must not reference external {collection} URIs")
    buffers = document.get("buffers", [])
    bin_chunks = [chunk for kind, chunk in chunks if kind == 0x004E4942]
    if buffers:
        if len(buffers) != 1 or not bin_chunks:
            raise ValueError(f"{context} buffer closure is incomplete")
        byte_length = buffers[0].get("byteLength")
        if isinstance(byte_length, bool) or not isinstance(byte_length, int) or byte_length <= 0:
            raise ValueError(f"{context} buffer byteLength is invalid")
        padding = bin_chunks[0][byte_length:]
        if byte_length > len(bin_chunks[0]) or len(padding) > 3 or any(padding):
            raise ValueError(f"{context} BIN chunk does not match its declared buffer")
    elif bin_chunks:
        raise ValueError(f"{context} contains an unreferenced BIN chunk")
    buffer_views = document.get("bufferViews", [])
    if not isinstance(buffer_views, list):
        raise ValueError(f"{context} bufferViews must be one array")
    declared_buffer_bytes = buffers[0]["byteLength"] if buffers else 0
    for view in buffer_views:
        if not isinstance(view, Mapping) or view.get("buffer") != 0:
            raise ValueError(f"{context} bufferView references an invalid buffer")
        offset_value = view.get("byteOffset", 0)
        length_value = view.get("byteLength")
        if (
            isinstance(offset_value, bool)
            or not isinstance(offset_value, int)
            or offset_value < 0
            or isinstance(length_value, bool)
            or not isinstance(length_value, int)
            or length_value <= 0
            or offset_value + length_value > declared_buffer_bytes
        ):
            raise ValueError(f"{context} bufferView exceeds the BIN closure")
    images = document.get("images", [])
    for image in images:
        view_index = image.get("bufferView")
        if (
            isinstance(view_index, bool)
            or not isinstance(view_index, int)
            or not 0 <= view_index < len(buffer_views)
        ):
            raise ValueError(f"{context} image references an invalid bufferView")
        if image.get("mimeType") not in {"image/png", "image/jpeg", "image/webp"}:
            raise ValueError(f"{context} image mimeType is unsupported")


def _atomic_publish_no_replace(source: Path, target: Path) -> None:
    """Atomically publish one file/directory without replacing a winner."""

    source_text = os.fspath(source)
    target_text = os.fspath(target)
    if os.name == "nt":
        move = ctypes.WinDLL("kernel32", use_last_error=True).MoveFileExW
        move.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p, ctypes.c_uint32]
        move.restype = ctypes.c_int
        if not move(source_text, target_text, 0):
            error = ctypes.get_last_error()
            if error in {80, 183}:
                raise FileExistsError(target)
            raise OSError(error, "MoveFileExW failed", target_text)
        return
    if sys.platform.startswith("linux"):
        libc = ctypes.CDLL(None, use_errno=True)
        renameat2 = getattr(libc, "renameat2", None)
        if renameat2 is None:
            raise RuntimeError("renameat2(RENAME_NOREPLACE) is unavailable")
        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        if renameat2(-100, os.fsencode(source), -100, os.fsencode(target), 1) != 0:
            error = ctypes.get_errno()
            if error == 17:
                raise FileExistsError(target)
            raise OSError(error, "renameat2 failed", target_text)
        return
    raise RuntimeError("atomic no-replace publication is unsupported on this platform")


def _write_json(path: Path, value: Any) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return target


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} must be a JSON object")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    observed = set(value)
    if observed != expected:
        raise ValueError(
            f"{context} keys must match the contract; "
            f"missing={sorted(expected - observed)}, "
            f"unexpected={sorted(observed - expected)}"
        )


def _identity(value: Any, context: str) -> str:
    if not isinstance(value, str) or _IDENTITY.fullmatch(value) is None:
        raise ValueError(f"{context} must be one stable identifier")
    return value


def _positive_vector(value: Any, context: str) -> list[float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 3
    ):
        raise ValueError(f"{context} must contain exactly three numbers")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{context}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{context}[{index}] must be finite and positive")
        result.append(number)
    return result


def _axis_order(value: Any) -> list[str]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 3
        or not all(isinstance(item, str) for item in value)
        or set(value) != _AXES
    ):
        raise ValueError("source_axis_order must be one permutation of x, y, z")
    return list(value)


def _portable_path(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{context} must be a non-empty path")
    path = PurePosixPath(value)
    if (
        path.is_absolute()
        or "\\" in value
        or ":" in value
        or any(part in {"", ".", ".."} for part in path.parts)
        or any(character.isspace() for character in value)
    ):
        raise ValueError(f"{context} must be one canonical relative path")
    return path.as_posix()


def _game_asset_path(value: Any) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("/Game/")
        or "\\" in value
        or "//" in value
        or any(character.isspace() for character in value)
        or any(part in {"", ".", ".."} for part in value[1:].split("/"))
    ):
        raise ValueError("unreal_asset_path must be one canonical /Game asset path")
    return value


def _absolute_link_free_path(path: Path, context: str) -> Path:
    candidate = Path(os.path.abspath(os.fspath(path)))
    current = Path(candidate.anchor)
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    for component in candidate.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            continue
        if current.is_symlink() or (
            reparse_flag
            and getattr(metadata, "st_file_attributes", 0) & reparse_flag
        ):
            raise ValueError(f"{context} must identify one link-free path")
    return candidate


def _relative_regular_file(path: Path, root: Path, context: str) -> str:
    unresolved = _absolute_link_free_path(Path(path), context)
    resolved = unresolved.resolve()
    if not resolved.is_file():
        raise ValueError(f"{context} must identify one regular, link-free file")
    try:
        return resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError(f"{context} must remain inside artifact_root") from exc


def _load_task(path: Path) -> Mapping[str, Any]:
    try:
        return _mapping(json.loads(Path(path).read_text(encoding="utf-8")), "task")
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read task: {exc}") from exc


def _canonical_lod_outputs(asset_id: str, directory: str) -> list[dict[str, str]]:
    return [
        {
            "name": name,
            "glb": f"{directory}/{asset_id}.{name}.glb",
            "fbx": f"{directory}/{asset_id}.{name}.fbx",
        }
        for name, _ratio in _LOD_RATIOS
    ]


def validate_static_prop_conditioning_plan(
    document: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate and canonicalize one static-prop conditioning plan."""

    plan = _mapping(document, "conditioning plan")
    _exact_keys(
        plan,
        {
            "schema",
            "asset",
            "source",
            "geometry",
            "lods",
            "binding",
            "outputs",
            "qualification",
            "digest",
        },
        "conditioning plan",
    )
    if plan.get("schema") != PLAN_SCHEMA:
        raise ValueError(f"schema must be {PLAN_SCHEMA}")
    digest = plan.get("digest")
    if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
        raise ValueError("digest must be one lowercase SHA-256 digest")
    body = {key: value for key, value in plan.items() if key != "digest"}
    if _digest_document(body) != digest:
        raise ValueError("conditioning plan digest does not match its content")

    asset = _mapping(plan.get("asset"), "asset")
    _exact_keys(asset, {"id", "kind", "role"}, "asset")
    asset_id = _identity(asset.get("id"), "asset.id")
    if asset.get("kind") != "static_mesh" or asset.get("role") != "world_visual_candidate":
        raise ValueError("asset must be one static world visual candidate")

    source = _mapping(plan.get("source"), "source")
    _exact_keys(source, {"model", "task", "profile"}, "source")
    model = _mapping(source.get("model"), "source.model")
    _exact_keys(model, {"path", "bytes", "sha256"}, "source.model")
    _portable_path(model.get("path"), "source.model.path")
    if isinstance(model.get("bytes"), bool) or not isinstance(model.get("bytes"), int) or model["bytes"] <= 0:
        raise ValueError("source.model.bytes must be a positive integer")
    if not isinstance(model.get("sha256"), str) or _SHA256.fullmatch(model["sha256"]) is None:
        raise ValueError("source.model.sha256 must be one lowercase SHA-256 digest")
    task = _mapping(source.get("task"), "source.task")
    _exact_keys(
        task,
        {"path", "bytes", "sha256", "id", "type", "model", "pbr", "texture", "credits_consumed"},
        "source.task",
    )
    _portable_path(task.get("path"), "source.task.path")
    if isinstance(task.get("bytes"), bool) or not isinstance(task.get("bytes"), int) or task["bytes"] <= 0:
        raise ValueError("source.task.bytes must be a positive integer")
    if not isinstance(task.get("sha256"), str) or _SHA256.fullmatch(task["sha256"]) is None:
        raise ValueError("source.task.sha256 must be one lowercase SHA-256 digest")
    _identity(task.get("id"), "source.task.id")
    if task.get("type") not in {"text_to_model", "multiview_to_model"}:
        raise ValueError("source.task.type is unsupported")
    _identity(task.get("model"), "source.task.model")
    if task.get("pbr") is not True or task.get("texture") is not True:
        raise ValueError("source task must contain textured PBR output")
    if (
        isinstance(task.get("credits_consumed"), bool)
        or not isinstance(task.get("credits_consumed"), int)
        or task["credits_consumed"] < 0
    ):
        raise ValueError("source.task.credits_consumed must be a non-negative integer")
    profile = _mapping(source.get("profile"), "source.profile")
    _exact_keys(profile, {"path", "bytes", "sha256"}, "source.profile")
    _portable_path(profile.get("path"), "source.profile.path")
    if isinstance(profile.get("bytes"), bool) or not isinstance(profile.get("bytes"), int) or profile["bytes"] <= 0:
        raise ValueError("source.profile.bytes must be a positive integer")
    if not isinstance(profile.get("sha256"), str) or _SHA256.fullmatch(profile["sha256"]) is None:
        raise ValueError("source.profile.sha256 must be one lowercase SHA-256 digest")

    geometry = _mapping(plan.get("geometry"), "geometry")
    _exact_keys(
        geometry,
        {"target_dimensions_m", "source_axis_order", "normalization", "normal_policy"},
        "geometry",
    )
    target_dimensions = _positive_vector(
        geometry.get("target_dimensions_m"), "geometry.target_dimensions_m"
    )
    axis_order = _axis_order(geometry.get("source_axis_order"))
    if geometry.get("normalization") != "uniform_fit_center_xy_ground_z":
        raise ValueError("geometry.normalization is unsupported")
    if geometry.get("normal_policy") != "recalculate_consistent_outside":
        raise ValueError("geometry.normal_policy is unsupported")

    lods = plan.get("lods")
    expected_lods = [
        {"name": name, "triangle_ratio": ratio}
        for name, ratio in _LOD_RATIOS
    ]
    if lods != expected_lods:
        raise ValueError("lods must match the deterministic LOD0/LOD1/LOD2 policy")

    binding = _mapping(plan.get("binding"), "binding")
    _exact_keys(binding, {"world_entity_id", "semantic_class", "physics", "unreal"}, "binding")
    world_entity_id = _identity(binding.get("world_entity_id"), "binding.world_entity_id")
    semantic_class = _identity(binding.get("semantic_class"), "binding.semantic_class")
    physics = _mapping(binding.get("physics"), "binding.physics")
    _exact_keys(physics, {"authority", "proxy", "render_mesh_is_collider"}, "binding.physics")
    if physics.get("authority") != "mujoco":
        raise ValueError("binding.physics.authority must be mujoco")
    if physics.get("render_mesh_is_collider") is not False:
        raise ValueError("binding.physics.render_mesh_is_collider must be false")
    proxy = _mapping(physics.get("proxy"), "binding.physics.proxy")
    _exact_keys(proxy, {"shape", "size_m"}, "binding.physics.proxy")
    if proxy.get("shape") != "box":
        raise ValueError("binding.physics.proxy.shape must be box")
    proxy_size = _positive_vector(proxy.get("size_m"), "binding.physics.proxy.size_m")
    if any(target > limit for target, limit in zip(target_dimensions, proxy_size)):
        raise ValueError("target visual dimensions must fit inside the MuJoCo proxy")

    unreal = _mapping(binding.get("unreal"), "binding.unreal")
    _exact_keys(
        unreal,
        {
            "asset_path",
            "collision_profile",
            "collision_enabled",
            "simulate_physics",
            "generate_overlap_events",
            "can_ever_affect_navigation",
        },
        "binding.unreal",
    )
    unreal_asset_path = _game_asset_path(unreal.get("asset_path"))
    if unreal.get("collision_profile") != "NoCollision":
        raise ValueError("binding.unreal.collision_profile must be NoCollision")
    for field in (
        "collision_enabled",
        "simulate_physics",
        "generate_overlap_events",
        "can_ever_affect_navigation",
    ):
        if unreal.get(field) is not False:
            raise ValueError(f"binding.unreal.{field} must be false")

    outputs = _mapping(plan.get("outputs"), "outputs")
    _exact_keys(outputs, {"directory", "blend", "preview", "inspection", "lods"}, "outputs")
    directory = _portable_path(outputs.get("directory"), "outputs.directory")
    expected_outputs = {
        "directory": directory,
        "blend": f"{directory}/{asset_id}.blend",
        "preview": f"{directory}/preview.png",
        "inspection": f"{directory}/conditioning-report.json",
        "lods": _canonical_lod_outputs(asset_id, directory),
    }
    if dict(outputs) != expected_outputs:
        raise ValueError("outputs do not match the deterministic asset naming policy")

    qualification = _mapping(plan.get("qualification"), "qualification")
    _exact_keys(qualification, {"state", "blockers", "promotion_target"}, "qualification")
    if qualification.get("state") != "QUARANTINED":
        raise ValueError("qualification.state must be QUARANTINED")
    blockers = qualification.get("blockers")
    expected_blockers = [
        "license_and_usage_rights_unverified",
        "unreal_import_not_verified",
    ]
    if blockers != expected_blockers:
        raise ValueError("qualification.blockers must preserve the source quarantine")
    if qualification.get("promotion_target") != "WorldPackage.visual facet":
        raise ValueError("qualification.promotion_target is invalid")

    canonical_body: dict[str, Any] = {
        "schema": PLAN_SCHEMA,
        "asset": {
            "id": asset_id,
            "kind": "static_mesh",
            "role": "world_visual_candidate",
        },
        "source": {
            "model": dict(model),
            "task": dict(task),
            "profile": dict(profile),
        },
        "geometry": {
            "target_dimensions_m": target_dimensions,
            "source_axis_order": axis_order,
            "normalization": "uniform_fit_center_xy_ground_z",
            "normal_policy": "recalculate_consistent_outside",
        },
        "lods": expected_lods,
        "binding": {
            "world_entity_id": world_entity_id,
            "semantic_class": semantic_class,
            "physics": {
                "authority": "mujoco",
                "proxy": {"shape": "box", "size_m": proxy_size},
                "render_mesh_is_collider": False,
            },
            "unreal": {
                "asset_path": unreal_asset_path,
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
            },
        },
        "outputs": expected_outputs,
        "qualification": {
            "state": "QUARANTINED",
            "blockers": expected_blockers,
            "promotion_target": "WorldPackage.visual facet",
        },
    }
    return {**canonical_body, "digest": _digest_document(canonical_body)}


def build_static_prop_conditioning_plan(
    *,
    artifact_root: Path,
    asset_id: str,
    source_model_path: Path,
    task_path: Path,
    output_directory: str,
    world_entity_id: str,
    semantic_class: str,
    target_dimensions_m: Sequence[float],
    source_axis_order: Sequence[str],
    mujoco_box_proxy_size_m: Sequence[float],
    unreal_asset_path: str,
) -> dict[str, Any]:
    """Build one portable, content-addressed conditioning plan."""

    root = _absolute_link_free_path(Path(artifact_root), "artifact_root").resolve()
    if not root.is_dir():
        raise ValueError("artifact_root must identify one existing directory")
    canonical_asset_id = _identity(asset_id, "asset_id")
    model_relative = _relative_regular_file(
        Path(source_model_path), root, "source_model_path"
    )
    task_relative = _relative_regular_file(Path(task_path), root, "task_path")
    model_path = Path(os.path.abspath(os.fspath(source_model_path)))
    task_file = Path(os.path.abspath(os.fspath(task_path)))
    if model_path.suffix.lower() != ".glb":
        raise ValueError("source_model_path must be one single-file .glb closure")
    directory = _portable_path(output_directory, "output_directory")
    if "/" in directory:
        raise ValueError("output_directory must be one direct child directory")

    model_bytes = _read_bound_file(
        model_path, maximum_bytes=MAX_SOURCE_GLB_BYTES, context="source_model_path"
    )
    task_bytes = _read_bound_file(
        task_file, maximum_bytes=MAX_TASK_JSON_BYTES, context="task_path"
    )
    profile_bytes = _read_bound_file(
        PROFILE_PATH, maximum_bytes=MAX_TASK_JSON_BYTES, context="repository profile"
    )
    _validate_self_contained_glb(model_bytes, "source_model_path")
    try:
        task_document = _mapping(json.loads(task_bytes.decode("utf-8")), "task")
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read task: {exc}") from exc
    if task_document.get("status") != "success":
        raise ValueError("Tripo task must have completed successfully")
    task_type = task_document.get("type")
    if task_type not in {"text_to_model", "multiview_to_model"}:
        raise ValueError("Tripo task type is unsupported")
    task_input = _mapping(task_document.get("input"), "task.input")
    if task_input.get("pbr") is not True or task_input.get("texture") is not True:
        raise ValueError("Tripo task must have generated textured PBR output")

    body: dict[str, Any] = {
        "schema": PLAN_SCHEMA,
        "asset": {
            "id": canonical_asset_id,
            "kind": "static_mesh",
            "role": "world_visual_candidate",
        },
        "source": {
            "model": {
                "path": model_relative,
                "bytes": len(model_bytes),
                "sha256": hashlib.sha256(model_bytes).hexdigest(),
            },
            "task": {
                "path": task_relative,
                "bytes": len(task_bytes),
                "sha256": hashlib.sha256(task_bytes).hexdigest(),
                "id": _identity(task_document.get("task_id"), "task.task_id"),
                "type": task_type,
                "model": _identity(task_input.get("model_version"), "task.input.model_version"),
                "pbr": True,
                "texture": True,
                "credits_consumed": task_document.get("credits_consumed"),
            },
            "profile": {
                "path": "sim/tools/assets/tripo_hero_static_pbr.v1.json",
                "bytes": len(profile_bytes),
                "sha256": hashlib.sha256(profile_bytes).hexdigest(),
            },
        },
        "geometry": {
            "target_dimensions_m": _positive_vector(target_dimensions_m, "target_dimensions_m"),
            "source_axis_order": _axis_order(source_axis_order),
            "normalization": "uniform_fit_center_xy_ground_z",
            "normal_policy": "recalculate_consistent_outside",
        },
        "lods": [
            {"name": name, "triangle_ratio": ratio}
            for name, ratio in _LOD_RATIOS
        ],
        "binding": {
            "world_entity_id": _identity(world_entity_id, "world_entity_id"),
            "semantic_class": _identity(semantic_class, "semantic_class"),
            "physics": {
                "authority": "mujoco",
                "proxy": {
                    "shape": "box",
                    "size_m": _positive_vector(
                        mujoco_box_proxy_size_m,
                        "mujoco_box_proxy_size_m",
                    ),
                },
                "render_mesh_is_collider": False,
            },
            "unreal": {
                "asset_path": _game_asset_path(unreal_asset_path),
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
            },
        },
        "outputs": {
            "directory": directory,
            "blend": f"{directory}/{canonical_asset_id}.blend",
            "preview": f"{directory}/preview.png",
            "inspection": f"{directory}/conditioning-report.json",
            "lods": _canonical_lod_outputs(canonical_asset_id, directory),
        },
        "qualification": {
            "state": "QUARANTINED",
            "blockers": [
                "license_and_usage_rights_unverified",
                "unreal_import_not_verified",
            ],
            "promotion_target": "WorldPackage.visual facet",
        },
    }
    return validate_static_prop_conditioning_plan(
        {**body, "digest": _digest_document(body)}
    )


def write_static_prop_conditioning_plan(path: Path, plan: Mapping[str, Any]) -> Path:
    """Validate and write one deterministic conditioning plan."""

    target = _absolute_link_free_path(path, "plan output")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(f"conditioning plan already exists: {target}")
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=target.parent, prefix=".plan-", suffix=".json", delete=False
    ) as stream:
        json.dump(
            validate_static_prop_conditioning_plan(plan), stream,
            ensure_ascii=False, sort_keys=True, indent=2, separators=(",", ": "), allow_nan=False,
        )
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
        temporary = Path(stream.name)
    try:
        _atomic_publish_no_replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


def build_blender_conditioning_command(
    blender_executable: str | Path,
    *,
    repo_root: Path,
    plan_path: Path,
) -> list[str]:
    """Build the isolated headless-Blender command for one frozen plan."""

    frozen_plan = _absolute_link_free_path(Path(plan_path), "plan_path")
    if not frozen_plan.is_file():
        raise ValueError("plan_path must identify one regular, link-free file")
    try:
        plan_bytes = _read_bound_file(
            frozen_plan, maximum_bytes=MAX_TASK_JSON_BYTES, context="plan_path"
        )
        raw_plan = json.loads(plan_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read plan: {exc}") from exc
    validate_static_prop_conditioning_plan(_mapping(raw_plan, "conditioning plan"))
    script = _absolute_link_free_path(
        _absolute_link_free_path(Path(repo_root), "repo_root").resolve()
        / "sim"
        / "tools"
        / "assets"
        / "blender_static_prop_conditioner.py",
        "repository static-prop conditioner",
    )
    if not script.is_file():
        raise ValueError("repository static-prop conditioner is missing")
    script_sha256 = _sha256_file(script)
    executable = str(blender_executable)
    if re.match(r"^(?:[A-Za-z]:[\\/]|\\\\)", executable):
        executable = str(PureWindowsPath(executable))
    return [
        executable,
        "--factory-startup",
        "--background",
        "--disable-autoexec",
        "--python-exit-code",
        "1",
        "--python",
        str(script),
        "--",
        "--plan",
        str(frozen_plan),
        "--plan-bytes",
        str(len(plan_bytes)),
        "--plan-sha256",
        hashlib.sha256(plan_bytes).hexdigest(),
        "--conditioner-contract",
        CONDITIONER_CONTRACT,
        "--script-sha256",
        script_sha256,
    ]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a static-prop Blender conditioning plan")
    parser.add_argument("--artifact-root", required=True, type=Path)
    parser.add_argument("--asset-id", required=True)
    parser.add_argument("--source-model", required=True, type=Path)
    parser.add_argument("--task", required=True, type=Path)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--world-entity-id", required=True)
    parser.add_argument("--semantic-class", required=True)
    parser.add_argument("--target-dimensions-m", nargs=3, required=True, type=float)
    parser.add_argument("--source-axis-order", nargs=3, required=True, choices=tuple(sorted(_AXES)))
    parser.add_argument("--mujoco-box-proxy-size-m", nargs=3, required=True, type=float)
    parser.add_argument("--unreal-asset-path", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for producing one offline plan."""

    args = _parser().parse_args(argv)
    plan = build_static_prop_conditioning_plan(
        artifact_root=args.artifact_root,
        asset_id=args.asset_id,
        source_model_path=args.source_model,
        task_path=args.task,
        output_directory=args.output_directory,
        world_entity_id=args.world_entity_id,
        semantic_class=args.semantic_class,
        target_dimensions_m=args.target_dimensions_m,
        source_axis_order=args.source_axis_order,
        mujoco_box_proxy_size_m=args.mujoco_box_proxy_size_m,
        unreal_asset_path=args.unreal_asset_path,
    )
    write_static_prop_conditioning_plan(args.output, plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "PLAN_SCHEMA",
    "CONDITIONER_CONTRACT",
    "build_blender_conditioning_command",
    "build_static_prop_conditioning_plan",
    "topology_requires_review",
    "validate_static_prop_conditioning_plan",
    "write_static_prop_conditioning_plan",
]
