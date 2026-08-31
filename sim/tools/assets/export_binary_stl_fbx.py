"""Deterministic binary STL parsing for the offline Blender FBX pipeline."""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import math
import os
import re
import secrets
import shutil
import stat
import struct
import sys
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class StlAssetError(ValueError):
    """Raised when an STL cannot be converted without guessing."""


SMOOTHING_ANGLE_DEGREES = 45.0
MAX_STL_TRIANGLES = 2_000_000
MAX_STL_VERTICES = MAX_STL_TRIANGLES * 3
MAX_STL_BYTES = 84 + MAX_STL_TRIANGLES * 50
MAX_STL_ASSETS = 4096
MAX_TOTAL_STL_TRIANGLES = 4_000_000
MAX_TOTAL_STL_BYTES = 84 * MAX_STL_ASSETS + MAX_TOTAL_STL_TRIANGLES * 50
MAX_MJCF_BYTES = 4 * 1024 * 1024
MAX_FBX_BYTES_PER_ASSET = 2 * 1024 * 1024 * 1024
MAX_TOTAL_FBX_BYTES = 8 * 1024 * 1024 * 1024
_READ_CHUNK_BYTES = 1024 * 1024
_WINDOWS_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
_SAFE_ASSET_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_WINDOWS_DEVICE_NAMES = {
    "aux",
    "con",
    "nul",
    "prn",
    *(f"com{index}" for index in range(1, 10)),
    *(f"lpt{index}" for index in range(1, 10)),
}


def runtime_decimation_ratio(
    triangle_count: int,
    max_triangles_per_asset: int | None,
) -> float:
    """Return the deterministic Blender decimation ratio for one runtime mesh."""

    if triangle_count <= 0:
        raise StlAssetError("runtime mesh triangle_count must be positive")
    if max_triangles_per_asset is None:
        return 1.0
    if max_triangles_per_asset <= 0:
        raise StlAssetError("max_triangles_per_asset must be positive")
    return min(1.0, max_triangles_per_asset / triangle_count)


def validate_runtime_triangle_budget(
    triangle_counts: Sequence[int],
    *,
    max_triangles_per_asset: int | None,
    max_total_triangles: int | None,
) -> int:
    """Validate an exported runtime mesh set and return its total triangle count."""

    if max_triangles_per_asset is not None and max_triangles_per_asset <= 0:
        raise StlAssetError("max_triangles_per_asset must be positive")
    if max_total_triangles is not None and max_total_triangles <= 0:
        raise StlAssetError("max_total_triangles must be positive")
    for index, triangle_count in enumerate(triangle_counts):
        if triangle_count <= 0:
            raise StlAssetError(f"runtime mesh {index} triangle count must be positive")
        if (
            max_triangles_per_asset is not None
            and triangle_count > max_triangles_per_asset
        ):
            raise StlAssetError(
                f"runtime mesh {index} exceeds per-asset triangle budget: "
                f"{triangle_count} > {max_triangles_per_asset}"
            )
    total = sum(triangle_counts)
    if max_total_triangles is not None and total > max_total_triangles:
        raise StlAssetError(
            f"runtime mesh set exceeds total triangle budget: "
            f"{total} > {max_total_triangles}"
        )
    return total


def _safe_asset_identifier(value: str, context: str) -> str:
    if _SAFE_ASSET_IDENTIFIER.fullmatch(value) is None:
        raise StlAssetError(f"{context} must be one safe asset identifier: {value!r}")
    if value.split(".", maxsplit=1)[0].casefold() in _WINDOWS_DEVICE_NAMES:
        raise StlAssetError(f"{context} uses a reserved Windows device name: {value!r}")
    return value


def _fbx_target(output_dir: Path, asset_name: str) -> Path:
    safe_name = _safe_asset_identifier(asset_name, "FBX asset name")
    root = Path(output_dir).absolute()
    target = (root / f"{safe_name}.fbx").absolute()
    try:
        target.relative_to(root)
    except ValueError as exc:
        raise StlAssetError(f"FBX target escapes output directory: {target}") from exc
    if target.parent != root:
        raise StlAssetError(f"FBX target must be one direct-child file: {target}")
    return target


@dataclass(frozen=True)
class StlMesh:
    """One indexed mesh decoded from a binary STL source."""

    vertices: tuple[tuple[float, float, float], ...]
    faces: tuple[tuple[int, int, int], ...]
    triangle_count: int
    bounds_min: tuple[float, float, float]
    bounds_max: tuple[float, float, float]
    source_sha256: str


@dataclass(frozen=True)
class FbxConversion:
    """One source-to-target item in the offline Blender conversion plan."""

    asset_name: str
    source: Path
    target: Path
    mesh: StlMesh


@dataclass(frozen=True)
class FbxConversionSource:
    """One lightweight STL-to-FBX work item without decoded triangle storage."""

    asset_name: str
    source: Path
    target: Path


@dataclass(frozen=True)
class _PreparedFbxExport:
    index_path: Path
    blender_version: str


@dataclass
class _DirectoryAnchor:
    path: Path
    identity: tuple[int, int]
    descriptor: int | None = None
    windows_handle: int | None = None

    def assert_stable(self) -> None:
        try:
            current = self.path.lstat()
        except OSError as exc:
            raise StlAssetError(f"FBX output parent disappeared: {self.path}") from exc
        if _directory_identity(current) != self.identity:
            raise StlAssetError(f"FBX output parent changed while exporting: {self.path}")
        if self.descriptor is not None:
            anchored = os.fstat(self.descriptor)
            if _directory_identity(anchored) != self.identity:
                raise StlAssetError(f"FBX output parent anchor changed: {self.path}")

    def close(self) -> None:
        if self.descriptor is not None:
            os.close(self.descriptor)
            self.descriptor = None
        if self.windows_handle is not None:
            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            close_handle = kernel32.CloseHandle
            close_handle.argtypes = [ctypes.c_void_p]
            close_handle.restype = ctypes.c_bool
            if not close_handle(ctypes.c_void_p(self.windows_handle)):
                error_number = ctypes.get_last_error()
                raise OSError(error_number, ctypes.FormatError(error_number))
            self.windows_handle = None


def _open_regular_nofollow(path: Path, *, context: str) -> int:
    """Open one regular file without following a final symlink or reparse point."""

    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    if os.name == "nt":
        import msvcrt

        create_file = ctypes.windll.kernel32.CreateFileW
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
            os.fspath(path),
            0x80000000,  # GENERIC_READ
            0x00000001 | 0x00000002,  # share read/write, but not delete
            None,
            3,  # OPEN_EXISTING
            0x00200000 | 0x08000000,  # OPEN_REPARSE_POINT | SEQUENTIAL_SCAN
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise StlAssetError(f"cannot open {context}: {path}")
        try:
            descriptor = msvcrt.open_osfhandle(int(handle), flags)
        except Exception:
            ctypes.windll.kernel32.CloseHandle(handle)
            raise
    else:
        try:
            descriptor = os.open(path, flags | getattr(os, "O_NOFOLLOW", 0))
        except OSError as exc:
            raise StlAssetError(f"cannot open {context}: {path}: {exc}") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT
        ):
            raise StlAssetError(f"{context} must be a regular link-free file: {path}")
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def _stable_file_identity(metadata: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _directory_identity(metadata: os.stat_result) -> tuple[int, int]:
    return metadata.st_dev, metadata.st_ino


def _read_exact(descriptor: int, size: int, *, context: str) -> bytes:
    chunks: list[bytes] = []
    observed = 0
    while observed < size:
        chunk = os.read(descriptor, min(_READ_CHUNK_BYTES, size - observed))
        if not chunk:
            raise StlAssetError(f"{context} ended before {size} bytes were available")
        chunks.append(chunk)
        observed += len(chunk)
    return b"".join(chunks)


def _hash_regular_bounded(path: Path, *, limit: int, context: str) -> tuple[int, str]:
    descriptor = _open_regular_nofollow(path, context=context)
    try:
        before = os.fstat(descriptor)
        if before.st_size > limit:
            raise StlAssetError(
                f"{context} exceeds physical byte budget: {before.st_size} > {limit}"
            )
        digest = hashlib.sha256()
        observed = 0
        while True:
            chunk = os.read(descriptor, _READ_CHUNK_BYTES)
            if not chunk:
                break
            observed += len(chunk)
            if observed > limit:
                raise StlAssetError(f"{context} exceeds physical byte budget: {path}")
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (
            _stable_file_identity(before) != _stable_file_identity(after)
            or observed != before.st_size
        ):
            raise StlAssetError(f"{context} changed while it was hashed: {path}")
        return observed, digest.hexdigest()
    finally:
        os.close(descriptor)


def _rename_directory_noreplace(
    anchor: _DirectoryAnchor,
    source_name: str,
    target_name: str,
) -> None:
    """Atomically publish one prepared directory without replacing a winner."""

    anchor.assert_stable()
    if os.name == "nt":
        source = anchor.path / source_name
        target = anchor.path / target_name
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        move_file_ex = kernel32.MoveFileExW
        move_file_ex.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p, ctypes.c_uint32]
        move_file_ex.restype = ctypes.c_bool
        if move_file_ex(str(source), str(target), 0x00000008):  # WRITE_THROUGH only
            return
        winerror = ctypes.get_last_error()
        if winerror in {80, 183}:
            raise FileExistsError(errno.EEXIST, "target already exists", str(target))
        raise OSError(winerror, ctypes.FormatError(winerror), str(target))

    try:
        libc = ctypes.CDLL(None, use_errno=True)
        renameat2 = libc.renameat2
    except (AttributeError, OSError) as exc:
        raise StlAssetError(
            "atomic no-replace directory publication is unavailable on this host"
        ) from exc
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    directory_descriptor = (
        anchor.descriptor
        if anchor.descriptor is not None
        else getattr(os, "AT_FDCWD", -100)
    )
    result = renameat2(
        directory_descriptor,
        os.fsencode(source_name),
        directory_descriptor,
        os.fsencode(target_name),
        0x1,  # RENAME_NOREPLACE
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number == errno.EEXIST:
        raise FileExistsError(
            errno.EEXIST,
            "target already exists",
            str(anchor.path / target_name),
        )
    raise OSError(
        error_number,
        os.strerror(error_number),
        str(anchor.path / target_name),
    )


def _require_link_free_directory_chain(path: Path, *, context: str) -> None:
    cursor = path.absolute()
    while True:
        metadata = cursor.lstat()
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or stat.S_ISLNK(metadata.st_mode)
            or bool(getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT)
        ):
            raise StlAssetError(f"{context} must not traverse a link or reparse point: {cursor}")
        parent = cursor.parent
        if parent == cursor:
            return
        cursor = parent


def _create_link_free_directory_chain(path: Path) -> None:
    missing: list[Path] = []
    cursor = path.absolute()
    while not os.path.lexists(cursor):
        missing.append(cursor)
        parent = cursor.parent
        if parent == cursor:
            break
        cursor = parent
    _require_link_free_directory_chain(cursor, context="FBX output parent")
    for directory in reversed(missing):
        try:
            directory.mkdir()
        except FileExistsError:
            pass
        metadata = directory.lstat()
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or stat.S_ISLNK(metadata.st_mode)
            or bool(getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT)
        ):
            raise StlAssetError(
                f"FBX output parent must not traverse a link or reparse point: {directory}"
            )


def _open_directory_anchor(path: Path) -> _DirectoryAnchor:
    _require_link_free_directory_chain(path, context="FBX output parent")
    metadata = path.lstat()
    identity = _directory_identity(metadata)
    if os.name == "nt":
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
            str(path),
            0x80000000,  # GENERIC_READ
            0x00000001 | 0x00000002,  # share read/write, but not delete
            None,
            3,  # OPEN_EXISTING
            0x02000000 | 0x00200000,  # BACKUP_SEMANTICS | OPEN_REPARSE_POINT
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise StlAssetError(f"cannot anchor FBX output parent: {path}")
        anchor = _DirectoryAnchor(path=path, identity=identity, windows_handle=int(handle))
    else:
        try:
            descriptor = os.open(
                path,
                os.O_RDONLY
                | getattr(os, "O_DIRECTORY", 0)
                | getattr(os, "O_NOFOLLOW", 0),
            )
        except OSError as exc:
            raise StlAssetError(f"cannot anchor FBX output parent: {path}: {exc}") from exc
        anchor = _DirectoryAnchor(path=path, identity=identity, descriptor=descriptor)
    try:
        anchor.assert_stable()
    except Exception:
        anchor.close()
        raise
    return anchor


def _create_private_staging(anchor: _DirectoryAnchor, target_name: str) -> tuple[str, Path]:
    for _attempt in range(32):
        staging_name = f".{target_name}.staging-{secrets.token_hex(16)}"
        try:
            if anchor.descriptor is not None:
                os.mkdir(staging_name, mode=0o700, dir_fd=anchor.descriptor)
            else:
                os.mkdir(anchor.path / staging_name, mode=0o700)
        except FileExistsError:
            continue
        staging = anchor.path / staging_name
        try:
            anchor.assert_stable()
            metadata = staging.lstat()
            if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
                raise StlAssetError(f"private FBX staging is not a directory: {staging}")
            if anchor.descriptor is not None:
                anchored_metadata = os.stat(
                    staging_name,
                    dir_fd=anchor.descriptor,
                    follow_symlinks=False,
                )
                if _directory_identity(metadata) != _directory_identity(anchored_metadata):
                    raise StlAssetError(f"private FBX staging path changed: {staging}")
                anchored_parent = Path(f"/proc/self/fd/{anchor.descriptor}")
                if anchored_parent.is_dir():
                    return staging_name, anchored_parent / staging_name
                raise StlAssetError(
                    "cannot expose the anchored FBX output parent through /proc/self/fd"
                )
            return staging_name, staging
        except BaseException:
            try:
                if anchor.descriptor is not None:
                    os.rmdir(staging_name, dir_fd=anchor.descriptor)
                else:
                    os.rmdir(staging)
            except OSError as cleanup_exc:
                raise StlAssetError(
                    f"cannot clean failed private staging creation; residual staging: {staging}"
                ) from cleanup_exc
            raise
    raise StlAssetError("cannot allocate a unique private FBX staging directory")


def _validate_prepared_export(
    index_path: Path,
    *,
    expected_blender_version: str,
) -> tuple[tuple[str, int, str], ...]:
    """Validate the complete private export set before its publication marker moves."""

    index_bytes, index_sha = _read_regular_bounded(
        index_path,
        limit=16 * 1024 * 1024,
        context="FBX asset index",
    )
    try:
        payload = json.loads(index_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise StlAssetError(f"cannot validate FBX asset index: {exc}") from exc
    generator = payload.get("generator")
    _script_size, current_script_sha = _hash_regular_bounded(
        Path(__file__),
        limit=16 * 1024 * 1024,
        context="FBX exporter script",
    )
    if (
        not isinstance(generator, dict)
        or generator.get("script") != Path(__file__).name
        or generator.get("script_sha256") != current_script_sha
        or not isinstance(generator.get("blender_version"), str)
        or not generator["blender_version"]
        or generator["blender_version"].strip().casefold() == "unknown"
        or generator["blender_version"] != expected_blender_version
    ):
        raise StlAssetError("FBX asset index is not bound to this exporter and Blender")
    assets = payload.get("assets")
    if not isinstance(assets, list) or not assets:
        raise StlAssetError("FBX asset index contains no assets")
    expected_names = {"asset-index.json"}
    expected_casefolded_names = {"asset-index.json"}
    records: list[tuple[str, int, str]] = [("asset-index.json", len(index_bytes), index_sha)]
    total_fbx_bytes = 0
    for asset in assets:
        if not isinstance(asset, dict):
            raise StlAssetError("FBX asset index contains an invalid asset record")
        name = asset.get("fbx")
        expected_size = asset.get("fbx_bytes")
        expected_sha = asset.get("fbx_sha256")
        if (
            not isinstance(name, str)
            or Path(name).name != name
            or not isinstance(expected_size, int)
            or expected_size <= 0
            or not isinstance(expected_sha, str)
        ):
            raise StlAssetError("FBX asset index contains an invalid output binding")
        if name.casefold() in expected_casefolded_names:
            raise StlAssetError(f"FBX asset index contains a duplicate output: {name}")
        actual_size, actual_sha = _hash_regular_bounded(
            index_path.parent / name,
            limit=MAX_FBX_BYTES_PER_ASSET,
            context=f"FBX output {name}",
        )
        if (actual_size, actual_sha) != (expected_size, expected_sha):
            raise StlAssetError(f"FBX output does not match its index binding: {name}")
        total_fbx_bytes += actual_size
        if total_fbx_bytes > MAX_TOTAL_FBX_BYTES:
            raise StlAssetError(
                "FBX output set exceeds total physical byte budget: "
                f"{total_fbx_bytes} > {MAX_TOTAL_FBX_BYTES}"
            )
        expected_names.add(name)
        expected_casefolded_names.add(name.casefold())
        records.append((name, actual_size, actual_sha))
    with os.scandir(index_path.parent) as entries:
        observed_names = {entry.name for entry in entries}
    if observed_names != expected_names:
        raise StlAssetError("private FBX export contains unindexed files")
    return tuple(records)


def _read_regular_bounded(path: Path, *, limit: int, context: str) -> tuple[bytes, str]:
    descriptor = _open_regular_nofollow(path, context=context)
    try:
        before = os.fstat(descriptor)
        if before.st_size > limit:
            raise StlAssetError(f"{context} exceeds physical byte budget: {path}")
        chunks: list[bytes] = []
        observed = 0
        digest = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, min(_READ_CHUNK_BYTES, limit + 1 - observed))
            if not chunk:
                break
            observed += len(chunk)
            if observed > limit:
                raise StlAssetError(f"{context} exceeds physical byte budget: {path}")
            chunks.append(chunk)
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (
            _stable_file_identity(before) != _stable_file_identity(after)
            or observed != before.st_size
        ):
            raise StlAssetError(f"{context} changed while it was read: {path}")
        return b"".join(chunks), digest.hexdigest()
    finally:
        os.close(descriptor)


def discover_stl_sources(source_dir: Path) -> tuple[Path, ...]:
    """Discover direct-child STL files in deterministic case-insensitive order."""

    source_dir = Path(source_dir).absolute()
    try:
        directory_metadata = source_dir.lstat()
    except OSError as exc:
        raise StlAssetError(f"STL source directory does not exist: {source_dir}") from exc
    if (
        not stat.S_ISDIR(directory_metadata.st_mode)
        or stat.S_ISLNK(directory_metadata.st_mode)
        or bool(getattr(directory_metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT)
    ):
        raise StlAssetError(f"STL source directory does not exist: {source_dir}")
    _require_link_free_directory_chain(source_dir, context="STL source directory")
    sources: list[Path] = []
    with os.scandir(source_dir) as entries:
        for entry in entries:
            if Path(entry.name).suffix.lower() != ".stl":
                continue
            metadata = entry.stat(follow_symlinks=False)
            if (
                entry.is_symlink()
                or not stat.S_ISREG(metadata.st_mode)
                or bool(getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT)
            ):
                raise StlAssetError(
                    f"binary STL source must be a regular link-free file: {entry.path}"
                )
            sources.append(Path(entry.path).absolute())
            if len(sources) > MAX_STL_ASSETS:
                raise StlAssetError(
                    f"STL source directory exceeds asset budget: {len(sources)} > {MAX_STL_ASSETS}"
                )
    return tuple(sorted(sources, key=lambda path: (path.name.casefold(), path.name)))


def build_conversion_plan(source_dir: Path, output_dir: Path) -> tuple[FbxConversion, ...]:
    """Parse all STL inputs and resolve stable per-link FBX output paths."""

    return tuple(
        FbxConversion(
            asset_name=item.asset_name,
            source=item.source,
            target=item.target,
            mesh=read_binary_stl(item.source),
        )
        for item in build_conversion_source_plan(source_dir, output_dir)
    )


def build_conversion_source_plan(
    source_dir: Path,
    output_dir: Path,
) -> tuple[FbxConversionSource, ...]:
    """Resolve STL-to-FBX work without decoding all high-detail meshes at once."""

    output_dir = Path(output_dir).absolute()
    plan: list[FbxConversionSource] = []
    targets: dict[str, str] = {}
    for source in discover_stl_sources(source_dir):
        asset_name = _safe_asset_identifier(source.stem, "STL asset name")
        target = _fbx_target(output_dir, asset_name)
        collision_key = target.name.casefold()
        previous = targets.get(collision_key)
        if previous is not None:
            raise StlAssetError(
                "case-insensitive FBX target collision between "
                f"{previous!r} and {asset_name!r}"
            )
        targets[collision_key] = asset_name
        plan.append(
            FbxConversionSource(
                asset_name=asset_name,
                source=source.absolute(),
                target=target,
            )
        )
    return tuple(plan)


def build_mjcf_conversion_plan(
    mjcf_path: Path,
    output_dir: Path,
    *,
    source_mesh_root: Path | None = None,
) -> tuple[FbxConversion, ...]:
    """Resolve only mesh assets referenced by MJCF visual geoms."""

    return tuple(
        FbxConversion(
            asset_name=item.asset_name,
            source=item.source,
            target=item.target,
            mesh=read_binary_stl(item.source),
        )
        for item in build_mjcf_conversion_source_plan(
            mjcf_path,
            output_dir,
            source_mesh_root=source_mesh_root,
        )
    )


def build_mjcf_conversion_source_plan(
    mjcf_path: Path,
    output_dir: Path,
    *,
    source_mesh_root: Path | None = None,
) -> tuple[FbxConversionSource, ...]:
    """Resolve referenced MJCF visual meshes without retaining decoded geometry."""

    mjcf_path = Path(mjcf_path).absolute()
    try:
        _require_link_free_directory_chain(
            mjcf_path.parent,
            context="MJCF source directory",
        )
        mjcf_bytes, _mjcf_sha256 = _read_regular_bounded(
            mjcf_path,
            limit=MAX_MJCF_BYTES,
            context="MuJoCo XML",
        )
        try:
            mjcf_text = mjcf_bytes.decode("utf-8-sig", errors="strict")
        except UnicodeDecodeError as exc:
            raise StlAssetError(
                f"MuJoCo XML must use UTF-8 or UTF-8-SIG: {mjcf_path}"
            ) from exc
        if "\x00" in mjcf_text:
            raise StlAssetError(f"MuJoCo XML must use UTF-8 or UTF-8-SIG: {mjcf_path}")
        upper_mjcf = mjcf_text.upper()
        if "<!DOCTYPE" in upper_mjcf or "<!ENTITY" in upper_mjcf:
            raise StlAssetError(f"MuJoCo XML must not declare DTDs or entities: {mjcf_path}")
        encoding_declaration = re.search(
            r"<\?xml\s+[^>]*encoding\s*=\s*['\"]([^'\"]+)['\"]",
            mjcf_text[:512],
            flags=re.IGNORECASE,
        )
        if (
            encoding_declaration is not None
            and encoding_declaration.group(1).casefold() not in {"utf-8", "utf8"}
        ):
            raise StlAssetError(f"MuJoCo XML must use UTF-8 or UTF-8-SIG: {mjcf_path}")
        root = ET.fromstring(  # noqa: S314 - DTD/entity declarations rejected above
            mjcf_text.encode("utf-8")
        )
    except (OSError, ET.ParseError) as exc:
        raise StlAssetError(f"cannot parse MuJoCo XML {mjcf_path}: {exc}") from exc

    compiler = root.find("compiler")
    meshdir_text = compiler.attrib.get("meshdir", ".") if compiler is not None else "."
    meshdir = (
        Path(source_mesh_root).absolute()
        if source_mesh_root is not None
        else (mjcf_path.parent / meshdir_text).absolute()
    )
    if not meshdir.is_dir():
        raise StlAssetError(f"mesh source directory does not exist: {meshdir}")
    _require_link_free_directory_chain(meshdir, context="mesh source directory")
    assets: dict[str, Path] = {}
    casefolded_asset_names: dict[str, str] = {}
    for element in root.findall("./asset/mesh"):
        name = element.attrib.get("name", "").strip()
        file_name = element.attrib.get("file", "").strip()
        if not name or not file_name:
            raise StlAssetError(f"MJCF mesh assets require name and file: {mjcf_path}")
        _safe_asset_identifier(name, "MJCF mesh name")
        if name in assets:
            raise StlAssetError(f"MJCF contains duplicate mesh asset {name!r}: {mjcf_path}")
        collision_key = name.casefold()
        previous = casefolded_asset_names.get(collision_key)
        if previous is not None:
            raise StlAssetError(
                "case-insensitive FBX target collision between MJCF mesh assets "
                f"{previous!r} and {name!r}: {mjcf_path}"
            )
        casefolded_asset_names[collision_key] = name
        source_path = (meshdir / file_name).absolute()
        try:
            source_path.relative_to(meshdir)
        except ValueError as exc:
            raise StlAssetError(
                f"MJCF mesh asset {name!r} escapes mesh source directory: {file_name}"
            ) from exc
        try:
            source_metadata = source_path.lstat()
        except OSError as exc:
            raise StlAssetError(
                f"MJCF mesh asset {name!r} does not exist: {source_path}"
            ) from exc
        if (
            not stat.S_ISREG(source_metadata.st_mode)
            or stat.S_ISLNK(source_metadata.st_mode)
            or bool(
                getattr(source_metadata, "st_file_attributes", 0)
                & _WINDOWS_REPARSE_POINT
            )
        ):
            raise StlAssetError(
                f"MJCF mesh asset {name!r} must be a regular link-free file: {source_path}"
            )
        assets[name] = source_path

    worldbody = root.find("worldbody")
    referenced_names = {
        geom.attrib["mesh"]
        for geom in (() if worldbody is None else worldbody.iter("geom"))
        if geom.attrib.get("mesh")
    }
    output_dir = Path(output_dir).absolute()
    plan: list[FbxConversionSource] = []
    for name in sorted(referenced_names, key=lambda value: (value.casefold(), value)):
        referenced_source_path = assets.get(name)
        if referenced_source_path is None:
            raise StlAssetError(f"MJCF geom references unknown mesh asset {name!r}: {mjcf_path}")
        plan.append(
            FbxConversionSource(
                asset_name=name,
                source=referenced_source_path,
                target=_fbx_target(output_dir, name),
            )
        )
    return tuple(plan)


def read_binary_stl(path: Path) -> StlMesh:
    """Read a strict binary STL and deduplicate identical vertex positions."""

    path = Path(path)
    descriptor = _open_regular_nofollow(path, context="binary STL source")
    try:
        before = os.fstat(descriptor)
        if before.st_size < 84:
            raise StlAssetError(f"binary STL is shorter than its 84-byte header: {path}")
        if before.st_size > MAX_STL_BYTES:
            raise StlAssetError(
                f"binary STL exceeds physical byte budget for {path}: "
                f"{before.st_size} > {MAX_STL_BYTES}"
            )

        header = _read_exact(descriptor, 84, context=f"binary STL header for {path}")
        triangle_count = struct.unpack_from("<I", header, 80)[0]
        if triangle_count == 0:
            raise StlAssetError(f"binary STL contains no triangles: {path}")
        if triangle_count > MAX_STL_TRIANGLES:
            raise StlAssetError(
                f"binary STL exceeds triangle budget for {path}: "
                f"{triangle_count} > {MAX_STL_TRIANGLES}"
            )
        expected_size = 84 + triangle_count * 50
        if expected_size > MAX_STL_BYTES:
            raise StlAssetError(f"binary STL exceeds physical byte budget: {path}")
        if before.st_size != expected_size:
            raise StlAssetError(
                f"binary STL size mismatch for {path}: expected {expected_size} bytes, "
                f"got {before.st_size}"
            )

        vertices: list[tuple[float, float, float]] = []
        vertex_indices: dict[tuple[float, float, float], int] = {}
        faces: list[tuple[int, int, int]] = []
        digest = hashlib.sha256(header)

        for _triangle_index in range(triangle_count):
            record = _read_exact(descriptor, 50, context=f"binary STL triangle for {path}")
            digest.update(record)
            values = struct.unpack("<12fH", record)
            face: list[int] = []
            for offset in (3, 6, 9):
                vertex = (values[offset], values[offset + 1], values[offset + 2])
                index = vertex_indices.get(vertex)
                if index is None:
                    if len(vertices) >= MAX_STL_VERTICES:
                        raise StlAssetError(f"binary STL exceeds vertex budget: {path}")
                    index = len(vertices)
                    vertex_indices[vertex] = index
                    vertices.append(vertex)
                face.append(index)
            faces.append((face[0], face[1], face[2]))

        after = os.fstat(descriptor)
        if _stable_file_identity(before) != _stable_file_identity(after):
            raise StlAssetError(f"binary STL changed while it was read: {path}")
    finally:
        os.close(descriptor)

    bounds_min = tuple(min(vertex[axis] for vertex in vertices) for axis in range(3))
    bounds_max = tuple(max(vertex[axis] for vertex in vertices) for axis in range(3))
    return StlMesh(
        vertices=tuple(vertices),
        faces=tuple(faces),
        triangle_count=triangle_count,
        bounds_min=(bounds_min[0], bounds_min[1], bounds_min[2]),
        bounds_max=(bounds_max[0], bounds_max[1], bounds_max[2]),
        source_sha256=digest.hexdigest(),
    )


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise StlAssetError("private STL snapshot write made no progress")
        offset += written


def _snapshot_stl_source(
    source: Path,
    snapshot: Path,
    *,
    remaining_bytes: int,
    remaining_triangles: int,
) -> tuple[int, int]:
    source_descriptor = _open_regular_nofollow(source, context="binary STL source")
    snapshot_descriptor: int | None = None
    try:
        before = os.fstat(source_descriptor)
        if before.st_size < 84:
            raise StlAssetError(f"binary STL is shorter than its 84-byte header: {source}")
        if before.st_size > MAX_STL_BYTES:
            raise StlAssetError(f"binary STL exceeds physical byte budget: {source}")
        header = _read_exact(
            source_descriptor,
            84,
            context=f"binary STL header for {source}",
        )
        triangle_count = struct.unpack_from("<I", header, 80)[0]
        if triangle_count == 0:
            raise StlAssetError(f"binary STL contains no triangles: {source}")
        if triangle_count > MAX_STL_TRIANGLES:
            raise StlAssetError(
                f"binary STL exceeds triangle budget for {source}: "
                f"{triangle_count} > {MAX_STL_TRIANGLES}"
            )
        expected_size = 84 + triangle_count * 50
        if before.st_size != expected_size:
            raise StlAssetError(
                f"binary STL size mismatch for {source}: expected {expected_size} bytes, "
                f"got {before.st_size}"
            )
        if expected_size > remaining_bytes:
            raise StlAssetError("selected STL files exceed total physical byte budget")
        if triangle_count > remaining_triangles:
            raise StlAssetError("selected STL files exceed total triangle budget")

        snapshot_descriptor = os.open(
            snapshot,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_BINARY", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
        )
        _write_all(snapshot_descriptor, header)
        remaining = expected_size - len(header)
        while remaining:
            chunk = os.read(source_descriptor, min(_READ_CHUNK_BYTES, remaining))
            if not chunk:
                raise StlAssetError(f"binary STL changed while it was snapshotted: {source}")
            _write_all(snapshot_descriptor, chunk)
            remaining -= len(chunk)
        os.fsync(snapshot_descriptor)
        after = os.fstat(source_descriptor)
        if _stable_file_identity(before) != _stable_file_identity(after):
            raise StlAssetError(f"binary STL changed while it was snapshotted: {source}")
        return expected_size, triangle_count
    finally:
        if snapshot_descriptor is not None:
            os.close(snapshot_descriptor)
        os.close(source_descriptor)


def _preflight_conversion_plan(
    source_plan: Sequence[FbxConversionSource],
    output_dir: Path,
) -> tuple[FbxConversion, ...]:
    if not source_plan:
        raise StlAssetError("asset selection contains no referenced STL meshes")
    if len(source_plan) > MAX_STL_ASSETS:
        raise StlAssetError(
            f"asset selection exceeds count budget: {len(source_plan)} > {MAX_STL_ASSETS}"
        )
    snapshot_root = output_dir / f".source-snapshots-{secrets.token_hex(16)}"
    snapshot_root.mkdir(mode=0o700)
    snapshots: list[tuple[FbxConversionSource, Path]] = []
    total_bytes = 0
    total_triangles = 0
    try:
        for index, source_item in enumerate(source_plan):
            snapshot = snapshot_root / f"{index:04d}.stl"
            physical_bytes, triangle_count = _snapshot_stl_source(
                source_item.source,
                snapshot,
                remaining_bytes=MAX_TOTAL_STL_BYTES - total_bytes,
                remaining_triangles=MAX_TOTAL_STL_TRIANGLES - total_triangles,
            )
            total_bytes += physical_bytes
            total_triangles += triangle_count
            snapshots.append((source_item, snapshot))
        plan = tuple(
            FbxConversion(
                asset_name=source_item.asset_name,
                source=source_item.source,
                target=source_item.target,
                mesh=read_binary_stl(snapshot),
            )
            for source_item, snapshot in snapshots
        )
    finally:
        try:
            shutil.rmtree(snapshot_root)
        except OSError as exc:
            raise StlAssetError(
                f"cannot remove private STL snapshots; residual staging: {snapshot_root}"
            ) from exc
    return plan


def build_box_projected_uv_loops(
    mesh: StlMesh,
) -> tuple[tuple[float, float], ...]:
    """Build one deterministic non-degenerate UV coordinate per face loop.

    Binary STL has no UV channel.  Unreal's MikkTSpace tangent generation can
    therefore collapse to zero tangents and visibly corrupt shading.  Each
    triangle is projected along its dominant geometric normal and normalized
    against the source AABB, which gives the FBX a stable tangent basis without
    changing geometry or collision semantics.
    """

    spans = tuple(
        maximum - minimum
        for minimum, maximum in zip(mesh.bounds_min, mesh.bounds_max, strict=True)
    )
    result: list[tuple[float, float]] = []
    for face_index, face in enumerate(mesh.faces):
        first, second, third = (mesh.vertices[index] for index in face)
        edge_ab = tuple(second[index] - first[index] for index in range(3))
        edge_ac = tuple(third[index] - first[index] for index in range(3))
        normal = (
            edge_ab[1] * edge_ac[2] - edge_ab[2] * edge_ac[1],
            edge_ab[2] * edge_ac[0] - edge_ab[0] * edge_ac[2],
            edge_ab[0] * edge_ac[1] - edge_ab[1] * edge_ac[0],
        )
        dominant_axis = max(range(3), key=lambda axis: abs(normal[axis]))
        if abs(normal[dominant_axis]) <= 1.0e-20:
            raise StlAssetError(f"triangle {face_index} is degenerate and cannot receive UVs")
        projection_axes = (
            (1, 2) if dominant_axis == 0 else (0, 2) if dominant_axis == 1 else (0, 1)
        )
        u_axis, v_axis = projection_axes
        if spans[u_axis] <= 1.0e-20 or spans[v_axis] <= 1.0e-20:
            raise StlAssetError(
                f"triangle {face_index} cannot be normalized for box-projected UVs"
            )
        projected = tuple(
            (
                (vertex[u_axis] - mesh.bounds_min[u_axis]) / spans[u_axis],
                (vertex[v_axis] - mesh.bounds_min[v_axis]) / spans[v_axis],
            )
            for vertex in (first, second, third)
        )
        signed_double_area = (
            (projected[1][0] - projected[0][0])
            * (projected[2][1] - projected[0][1])
            - (projected[1][1] - projected[0][1])
            * (projected[2][0] - projected[0][0])
        )
        if abs(signed_double_area) <= 1.0e-20:
            raise StlAssetError(
                f"triangle {face_index} produced a degenerate tangent-space UV"
            )
        result.extend(projected)
    return tuple(result)


def export_fbx_assets(
    source_dir: Path | None,
    output_dir: Path,
    *,
    mjcf_path: Path | None = None,
    source_mesh_root: Path | None = None,
    max_triangles_per_asset: int | None = None,
    max_total_triangles: int | None = None,
) -> Path:
    """Build and validate a complete FBX set, then publish it atomically."""

    requested = Path(output_dir).absolute()
    parent = requested.parent.absolute()
    target = parent / requested.name
    _create_link_free_directory_chain(parent)
    if target == parent:
        raise StlAssetError(f"FBX output must be one child directory: {target}")
    anchor = _open_directory_anchor(parent)
    staging_name = ""
    staging: Path | None = None
    published = False
    try:
        staging_name, staging = _create_private_staging(anchor, target.name)
        prepared = _export_fbx_assets_to_directory(
            source_dir,
            staging,
            mjcf_path=mjcf_path,
            source_mesh_root=source_mesh_root,
            max_triangles_per_asset=max_triangles_per_asset,
            max_total_triangles=max_total_triangles,
        )
        _validate_prepared_export(
            prepared.index_path,
            expected_blender_version=prepared.blender_version,
        )
        anchor.assert_stable()
        try:
            _rename_directory_noreplace(anchor, staging_name, target.name)
        except FileExistsError as exc:
            raise StlAssetError(f"FBX output set already exists: {target}") from exc
        published = True
        return target / "asset-index.json"
    finally:
        try:
            if not published and staging is not None and os.path.lexists(staging):
                try:
                    shutil.rmtree(staging)
                except OSError as exc:
                    raise StlAssetError(
                        f"cannot clean failed FBX export; residual staging: {staging}"
                    ) from exc
        finally:
            try:
                anchor.close()
            except OSError:
                if not published:
                    raise


def _export_fbx_assets_to_directory(
    source_dir: Path | None,
    output_dir: Path,
    *,
    mjcf_path: Path | None = None,
    source_mesh_root: Path | None = None,
    max_triangles_per_asset: int | None = None,
    max_total_triangles: int | None = None,
) -> _PreparedFbxExport:
    """Export every selected binary STL into one caller-owned private directory."""

    if (source_dir is None) == (mjcf_path is None):
        raise StlAssetError("provide exactly one of source_dir or mjcf_path")
    if source_mesh_root is not None and mjcf_path is None:
        raise StlAssetError("source_mesh_root requires mjcf_path")
    validate_runtime_triangle_budget(
        (),
        max_triangles_per_asset=max_triangles_per_asset,
        max_total_triangles=max_total_triangles,
    )
    output_dir = Path(output_dir).absolute()
    output_dir.mkdir(parents=True, exist_ok=True)
    source_plan = (
        build_mjcf_conversion_source_plan(
            mjcf_path,
            output_dir,
            source_mesh_root=source_mesh_root,
        )
        if mjcf_path is not None
        else build_conversion_source_plan(source_dir, output_dir)  # type: ignore[arg-type]
    )
    conversion_plan = _preflight_conversion_plan(source_plan, output_dir)

    try:
        import bmesh  # type: ignore[import-not-found]
        import bpy  # type: ignore[import-not-found]
    except ImportError as exc:
        raise StlAssetError("FBX export must run inside Blender's Python runtime") from exc
    blender_version_value = getattr(getattr(bpy, "app", None), "version_string", None)
    if (
        not isinstance(blender_version_value, str)
        or not blender_version_value.strip()
        or blender_version_value.strip().casefold() == "unknown"
    ):
        raise StlAssetError("Blender runtime must expose an exact non-unknown version_string")
    blender_version = blender_version_value.strip()

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0

    index_assets: list[dict[str, Any]] = []
    for item in conversion_plan:
        blender_mesh = bpy.data.meshes.new(item.asset_name)
        blender_mesh.from_pydata(item.mesh.vertices, [], item.mesh.faces)
        blender_mesh.validate(clean_customdata=False)
        blender_mesh.update(calc_edges=True)

        editable_mesh = bmesh.new()
        editable_mesh.from_mesh(blender_mesh)
        bmesh.ops.recalc_face_normals(editable_mesh, faces=editable_mesh.faces)
        editable_mesh.normal_update()
        smoothing_angle = math.radians(SMOOTHING_ANGLE_DEGREES)
        for face in editable_mesh.faces:
            face.smooth = True
        for edge in editable_mesh.edges:
            edge.smooth = (
                len(edge.link_faces) == 2
                and edge.calc_face_angle(0.0) <= smoothing_angle
            )
        editable_mesh.to_mesh(blender_mesh)
        editable_mesh.free()
        blender_mesh.update(calc_edges=True)

        blender_object = bpy.data.objects.new(item.asset_name, blender_mesh)
        bpy.context.scene.collection.objects.link(blender_object)
        bpy.ops.object.select_all(action="DESELECT")
        blender_object.select_set(True)
        bpy.context.view_layer.objects.active = blender_object

        validated_triangle_count = len(blender_mesh.polygons)
        decimation_ratio = runtime_decimation_ratio(
            validated_triangle_count,
            max_triangles_per_asset,
        )
        if decimation_ratio < 1.0:
            modifier = blender_object.modifiers.new(
                name="LingTuRuntimeDecimate",
                type="DECIMATE",
            )
            modifier.decimate_type = "COLLAPSE"
            modifier.ratio = decimation_ratio
            modifier.use_collapse_triangulate = True
            result = bpy.ops.object.modifier_apply(modifier=modifier.name)
            if "FINISHED" not in result:
                raise StlAssetError(
                    f"Blender could not materialize runtime mesh for {item.asset_name}"
                )
            blender_mesh = blender_object.data
            blender_mesh.validate(clean_customdata=False)
            blender_mesh.update(calc_edges=True)
            editable_mesh = bmesh.new()
            editable_mesh.from_mesh(blender_mesh)
            bmesh.ops.recalc_face_normals(editable_mesh, faces=editable_mesh.faces)
            editable_mesh.normal_update()
            for face in editable_mesh.faces:
                face.smooth = True
            for edge in editable_mesh.edges:
                edge.smooth = (
                    len(edge.link_faces) == 2
                    and edge.calc_face_angle(0.0) <= smoothing_angle
                )
            editable_mesh.to_mesh(blender_mesh)
            editable_mesh.free()
            blender_mesh.update(calc_edges=True)

        validated_vertices = tuple(
            (float(vertex.co.x), float(vertex.co.y), float(vertex.co.z))
            for vertex in blender_mesh.vertices
        )
        validated_faces: list[tuple[int, int, int]] = []
        for polygon in blender_mesh.polygons:
            indices = tuple(int(index) for index in polygon.vertices)
            if len(indices) != 3:
                raise StlAssetError(
                    f"Blender produced a non-triangle face for {item.asset_name}"
                )
            validated_faces.append((indices[0], indices[1], indices[2]))
        validated_bounds_min = tuple(
            min(vertex[axis] for vertex in validated_vertices) for axis in range(3)
        )
        validated_bounds_max = tuple(
            max(vertex[axis] for vertex in validated_vertices) for axis in range(3)
        )
        validated_mesh = StlMesh(
            vertices=validated_vertices,
            faces=tuple(validated_faces),
            triangle_count=len(validated_faces),
            bounds_min=(
                validated_bounds_min[0],
                validated_bounds_min[1],
                validated_bounds_min[2],
            ),
            bounds_max=(
                validated_bounds_max[0],
                validated_bounds_max[1],
                validated_bounds_max[2],
            ),
            source_sha256=item.mesh.source_sha256,
        )
        uv_coordinates = build_box_projected_uv_loops(validated_mesh)
        uv_layer = blender_mesh.uv_layers.new(name="UVMap")
        if len(uv_layer.data) != len(uv_coordinates):
            raise StlAssetError(
                f"Blender loop count changed while assigning UVs for {item.asset_name}"
            )
        for loop, coordinate in zip(uv_layer.data, uv_coordinates, strict=True):
            loop.uv = coordinate

        validate_runtime_triangle_budget(
            [
                *(asset["triangle_count"] for asset in index_assets),
                validated_mesh.triangle_count,
            ],
            max_triangles_per_asset=max_triangles_per_asset,
            max_total_triangles=max_total_triangles,
        )

        result = bpy.ops.export_scene.fbx(
            filepath=str(item.target),
            check_existing=False,
            use_selection=True,
            object_types={"MESH"},
            apply_unit_scale=True,
            apply_scale_options="FBX_SCALE_UNITS",
            axis_forward="-Y",
            axis_up="Z",
            bake_space_transform=False,
            mesh_smooth_type="EDGE",
            use_tspace=True,
            add_leaf_bones=False,
            path_mode="AUTO",
        )
        if "FINISHED" not in result or not item.target.is_file():
            raise StlAssetError(f"Blender did not produce FBX output: {item.target}")

        fbx_bytes, fbx_sha256 = _hash_regular_bounded(
            item.target,
            limit=MAX_FBX_BYTES_PER_ASSET,
            context=f"FBX output {item.target.name}",
        )
        index_assets.append(
            {
                "asset_name": item.asset_name,
                "source": item.source.name,
                "source_sha256": item.mesh.source_sha256,
                "fbx": item.target.name,
                "triangle_count": validated_mesh.triangle_count,
                "source_triangle_count": item.mesh.triangle_count,
                "validation_removed_triangle_count": (
                    item.mesh.triangle_count - validated_triangle_count
                ),
                "runtime_optimization": (
                    "blender_decimate_collapse" if decimation_ratio < 1.0 else "source_geometry"
                ),
                "runtime_decimation_ratio": decimation_ratio,
                "runtime_removed_triangle_count": (
                    validated_triangle_count - validated_mesh.triangle_count
                ),
                "smoothing_angle_degrees": SMOOTHING_ANGLE_DEGREES,
                "uv_policy": "normalized_box_projection",
                "tangent_policy": "fbx_exported_mikktspace",
                "bounds_min_m": list(item.mesh.bounds_min),
                "bounds_max_m": list(item.mesh.bounds_max),
                "fbx_bytes": fbx_bytes,
                "fbx_sha256": fbx_sha256,
            }
        )
        bpy.data.objects.remove(blender_object, do_unlink=True)
        bpy.data.meshes.remove(blender_mesh)

    index_path = output_dir / "asset-index.json"
    total_runtime_triangles = validate_runtime_triangle_budget(
        [asset["triangle_count"] for asset in index_assets],
        max_triangles_per_asset=max_triangles_per_asset,
        max_total_triangles=max_total_triangles,
    )
    _script_bytes, script_sha256 = _hash_regular_bounded(
        Path(__file__),
        limit=16 * 1024 * 1024,
        context="FBX exporter script",
    )
    index_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.fbx-asset-index.v1",
                "source_axis": "mujoco_rh_z_up_m",
                "fbx_axis": {"forward": "-Y", "up": "Z", "unit": "m"},
                "generator": {
                    "script": Path(__file__).name,
                    "script_sha256": script_sha256,
                    "blender_version": blender_version,
                },
                "runtime_triangle_budget": {
                    "max_triangles_per_asset": max_triangles_per_asset,
                    "max_total_triangles": max_total_triangles,
                    "actual_total_triangles": total_runtime_triangles,
                },
                "assets": index_assets,
            },
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return _PreparedFbxExport(
        index_path=index_path,
        blender_version=blender_version,
    )


def _script_arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export binary robot STL links to FBX with Blender")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--source-dir", type=Path)
    source.add_argument("--mjcf", type=Path)
    parser.add_argument("--source-mesh-root", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-triangles-per-asset", type=int)
    parser.add_argument("--max-total-triangles", type=int)
    args = parser.parse_args(list(argv))
    if args.source_mesh_root is not None and args.mjcf is None:
        parser.error("--source-mesh-root requires --mjcf")
    if args.max_triangles_per_asset is not None and args.max_triangles_per_asset <= 0:
        parser.error("--max-triangles-per-asset must be positive")
    if args.max_total_triangles is not None and args.max_total_triangles <= 0:
        parser.error("--max-total-triangles must be positive")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """Blender script entry point; arguments follow Blender's ``--`` separator."""

    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else ()
    args = _script_arguments(argv)
    index_path = export_fbx_assets(
        args.source_dir,
        args.output_dir,
        mjcf_path=args.mjcf,
        source_mesh_root=args.source_mesh_root,
        max_triangles_per_asset=args.max_triangles_per_asset,
        max_total_triangles=args.max_total_triangles,
    )
    print(f"LingTu FBX asset index: {index_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
