"""Safe source-directory and archive intake for simulation imports."""

from __future__ import annotations

import gzip
import hashlib
import os
import shutil
import stat
import struct
import tempfile
import zipfile
from collections.abc import Iterable
from pathlib import Path, PurePosixPath
from typing import IO, cast

from .contracts import (
    ImportCode,
    ImportFailure,
    SourceFile,
    SourceIntakeResult,
    assert_no_reparse_components,
    file_records,
    sha256_file,
)

_ARCHIVE_SUFFIXES = (".zip", ".tar", ".tar.gz", ".tgz")
_MAX_ARCHIVE_MEMBER_PATH_LENGTH = 1024
_MAX_ARCHIVE_MEMBER_DEPTH = 64
_MAX_ZIP_CENTRAL_DIRECTORY_BYTES_PER_MEMBER = 4096
_TAR_METADATA_BUDGET_PER_MEMBER = 16 * 1024
_TAR_BLOCK_SIZE = 512
_ZIP_EOCD_SIGNATURE = b"PK\x05\x06"
_ZIP_EOCD_SIZE = 22
_ZIP_MAX_COMMENT_SIZE = 65_535
_WINDOWS_REPARSE_POINT = 0x0400


class SourceIntake:
    """Materialize untrusted source below one owned destination."""

    def __init__(
        self,
        *,
        max_files: int = 10_000,
        max_file_bytes: int = 512 * 1024 * 1024,
        max_total_bytes: int = 4 * 1024 * 1024 * 1024,
    ) -> None:
        self.max_files = max_files
        self.max_file_bytes = max_file_bytes
        self.max_total_bytes = max_total_bytes

    def materialize(self, source: Path, destination: Path) -> SourceIntakeResult:
        """Copy or safely extract one source into an empty destination."""

        source = assert_no_reparse_components(Path(source), context=str(source)).resolve()
        assert_no_reparse_components(source, context=str(source))
        destination = Path(destination).resolve()
        if not source.exists():
            raise ImportFailure(
                f"import source does not exist: {source}",
                code=ImportCode.SOURCE_MISSING,
                context=str(source),
            )
        if source.is_dir():
            try:
                destination.relative_to(source)
            except ValueError:
                pass
            else:
                raise ImportFailure(
                    "intake destination must not be inside the source tree",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=str(destination),
                )
        if destination.exists():
            raise ImportFailure(
                f"intake destination already exists: {destination}",
                code=ImportCode.UNSAFE_SOURCE,
                context=str(destination),
            )
        destination.mkdir(parents=True)
        try:
            if source.is_dir():
                source_kind = "directory"
                records = self._copy_directory(source, destination)
                source_sha256 = self._tree_source_digest(records)
            elif source.is_file() and self._is_archive(source):
                source_kind = "archive"
                with tempfile.TemporaryFile(mode="w+b", dir=destination) as snapshot:
                    source_sha256, archive_size = self._snapshot_archive(source, snapshot)
                    self._extract_archive(source, destination, snapshot, archive_size)
                records = file_records(destination)
            else:
                raise ImportFailure(
                    f"import source must be a directory or supported archive: {source}",
                    code=ImportCode.SOURCE_FORMAT_UNSUPPORTED,
                    context=str(source),
                    details={"archive_suffixes": list(_ARCHIVE_SUFFIXES)},
                )
            self._enforce_limits(records)
            return SourceIntakeResult(destination, source_kind, source_sha256, records)
        except Exception:
            shutil.rmtree(destination, ignore_errors=True)
            raise

    @staticmethod
    def _is_archive(path: Path) -> bool:
        name = path.name.lower()
        return any(name.endswith(suffix) for suffix in _ARCHIVE_SUFFIXES)

    def _copy_directory(self, source: Path, destination: Path) -> tuple[SourceFile, ...]:
        files = self._scan_directory(source)
        records: list[SourceFile] = []
        for relative, size in sorted(files, key=lambda item: item[0].as_posix()):
            source_path = source.joinpath(*relative.parts)
            assert_no_reparse_components(source_path, below=source, context=str(source_path))
            target = destination.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            with source_path.open("rb") as reader, target.open("xb") as writer:
                copied_sha256 = self._copy_bounded_digest(reader, writer, size)
            if sha256_file(target) != copied_sha256:
                raise ImportFailure(
                    f"copied source file failed its content identity check: {relative}",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=relative.as_posix(),
                )
            records.append(SourceFile(relative.as_posix(), size, copied_sha256))
        return tuple(records)

    def _scan_directory(self, source: Path) -> list[tuple[PurePosixPath, int]]:
        pending = [(source, PurePosixPath())]
        files: list[tuple[PurePosixPath, int]] = []
        member_count = 0
        total_size = 0
        while pending:
            directory, relative_directory = pending.pop()
            try:
                with os.scandir(directory) as entries:
                    for entry in entries:
                        relative = relative_directory / entry.name
                        member_count += 1
                        self._enforce_archive_metadata_limits(member_count, (relative.as_posix(),))
                        metadata = entry.stat(follow_symlinks=False)
                        if stat.S_ISLNK(metadata.st_mode) or bool(
                            getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT
                        ):
                            raise ImportFailure(
                                f"source tree contains a symbolic link or Windows reparse point: {entry.path}",
                                code=ImportCode.UNSAFE_SOURCE,
                                context=str(entry.path),
                            )
                        if stat.S_ISDIR(metadata.st_mode):
                            pending.append((Path(entry.path), relative))
                            continue
                        if not stat.S_ISREG(metadata.st_mode):
                            raise ImportFailure(
                                f"source tree contains a non-regular file: {entry.path}",
                                code=ImportCode.UNSAFE_SOURCE,
                                context=str(entry.path),
                            )
                        total_size = self._enforce_regular_member_size(
                            relative.as_posix(), metadata.st_size, total_size
                        )
                        files.append((relative, metadata.st_size))
            except ImportFailure:
                raise
            except OSError as exc:
                raise ImportFailure(
                    f"cannot inspect source directory {directory}: {exc}",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=str(directory),
                ) from exc
        if not files:
            raise ImportFailure(
                "source tree contains no regular files",
                code=ImportCode.SOURCE_MISSING,
                context=str(source),
            )
        return files

    def _extract_archive(
        self,
        source: Path,
        destination: Path,
        snapshot: IO[bytes],
        archive_size: int,
    ) -> None:
        name = source.name.lower()
        if name.endswith(".zip"):
            self._extract_zip(source, destination, snapshot, archive_size)
        else:
            self._extract_tar(source, destination, snapshot)

    def _snapshot_archive(self, source: Path, snapshot: IO[bytes]) -> tuple[str, int]:
        try:
            reader = source.open("rb")
        except OSError as exc:
            raise ImportFailure(
                f"cannot open archive source {source}: {exc}",
                code=ImportCode.UNSAFE_SOURCE,
                context=str(source),
            ) from exc
        with reader:
            try:
                archive_size = os.fstat(reader.fileno()).st_size
            except OSError as exc:
                raise ImportFailure(
                    f"cannot inspect open archive source {source}: {exc}",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=str(source),
                ) from exc
            if archive_size > self.max_total_bytes:
                raise ImportFailure(
                    "archive exceeds the physical size limit",
                    code=ImportCode.ARCHIVE_LIMIT,
                    context=str(source),
                    details={"size": archive_size, "maximum": self.max_total_bytes},
                )
            digest = hashlib.sha256()
            remaining = archive_size
            while remaining:
                chunk = reader.read(min(1024 * 1024, remaining))
                if not chunk:
                    raise ImportFailure(
                        f"archive source shrank while snapshotting: {source}",
                        code=ImportCode.UNSAFE_SOURCE,
                        context=str(source),
                    )
                remaining -= len(chunk)
                digest.update(chunk)
                snapshot.write(chunk)
            if reader.read(1):
                raise ImportFailure(
                    f"archive source grew while snapshotting: {source}",
                    code=ImportCode.UNSAFE_SOURCE,
                    context=str(source),
                )
        snapshot.flush()
        snapshot.seek(0)
        return digest.hexdigest(), archive_size

    def _extract_zip(
        self,
        source: Path,
        destination: Path,
        snapshot: IO[bytes],
        archive_size: int,
    ) -> None:
        expected_members = self._preflight_zip(source, snapshot, archive_size)
        try:
            archive = zipfile.ZipFile(snapshot)
        except (OSError, zipfile.BadZipFile) as exc:
            raise ImportFailure(
                f"cannot read ZIP archive {source}: {exc}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            ) from exc
        with archive:
            members = archive.infolist()
            self._enforce_archive_metadata_limits(len(members), (item.filename for item in members))
            if len(members) != expected_members:
                raise ImportFailure(
                    "ZIP central-directory entry count changed after preflight",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=str(source),
                    details={"expected": expected_members, "actual": len(members)},
                )
            members = sorted(members, key=lambda item: item.filename)
            relative_paths = self._validated_archive_layout([(item.filename, item.is_dir()) for item in members])
            self._enforce_member_limits([(item.filename, item.file_size) for item in members if not item.is_dir()])
            for member, relative in zip(members, relative_paths):
                unix_mode = member.external_attr >> 16
                if stat.S_ISLNK(unix_mode):
                    raise ImportFailure(
                        f"archive contains a symbolic link: {member.filename}",
                        code=ImportCode.UNSAFE_ARCHIVE,
                        context=member.filename,
                    )
                target = self._archive_target(destination, relative)
                if member.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(member) as reader, target.open("xb") as writer:
                    self._copy_bounded(reader, writer, member.file_size)

    def _extract_tar(self, source: Path, destination: Path, snapshot: IO[bytes]) -> None:
        snapshot.seek(0)
        if source.name.lower().endswith((".tar.gz", ".tgz")):
            with gzip.GzipFile(fileobj=snapshot, mode="rb") as stream:
                self._extract_tar_stream(cast(IO[bytes], stream), source, destination)
        else:
            self._extract_tar_stream(snapshot, source, destination)

    def _extract_tar_stream(self, stream: IO[bytes], source: Path, destination: Path) -> None:
        remaining_budget = [
            self.max_total_bytes + self.max_files * _TAR_METADATA_BUDGET_PER_MEMBER
        ]
        member_count = 0
        total_size = 0
        kinds: dict[str, bool] = {}
        descendant_parents: set[str] = set()
        while True:
            header = self._read_tar_exact(stream, _TAR_BLOCK_SIZE, remaining_budget, source)
            if header == bytes(_TAR_BLOCK_SIZE):
                second = self._read_tar_exact(stream, _TAR_BLOCK_SIZE, remaining_budget, source)
                if second != bytes(_TAR_BLOCK_SIZE):
                    raise ImportFailure(
                        "TAR archive has an invalid end marker",
                        code=ImportCode.UNSAFE_ARCHIVE,
                        context=str(source),
                    )
                self._drain_tar_trailing_data(stream, remaining_budget, source)
                return
            name, size, typeflag = self._parse_tar_header(header, source)
            member_count += 1
            self._enforce_archive_metadata_limits(member_count, (name,))
            if typeflag in {b"x", b"g", b"L", b"K", b"S"}:
                raise ImportFailure(
                    f"TAR archive contains an unbounded metadata extension: {name}",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=name,
                )
            is_directory = typeflag == b"5"
            if typeflag not in {b"\0", b"0", b"5"}:
                raise ImportFailure(
                    f"archive contains a non-regular entry: {name}",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=name,
                )
            if is_directory and size != 0:
                raise ImportFailure(
                    f"TAR directory member declares data: {name}",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=name,
                )
            relative = self._validate_streamed_archive_layout(
                name, is_directory, kinds, descendant_parents
            )
            target = self._archive_target(destination, relative)
            if is_directory:
                continue
            total_size = self._enforce_regular_member_size(name, size, total_size)
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open("xb") as writer:
                self._copy_tar_member(stream, writer, size, remaining_budget, source)
            padding = (-size) % _TAR_BLOCK_SIZE
            if padding:
                self._read_tar_exact(stream, padding, remaining_budget, source)

    @staticmethod
    def _parse_tar_header(header: bytes, source: Path) -> tuple[str, int, bytes]:
        checksum_field = header[148:156]
        try:
            expected_checksum = int(checksum_field.rstrip(b"\0 ") or b"0", 8)
        except ValueError as exc:
            raise ImportFailure(
                "TAR archive contains an invalid header checksum",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            ) from exc
        actual_checksum = sum(header[:148]) + (8 * ord(" ")) + sum(header[156:])
        if actual_checksum != expected_checksum:
            raise ImportFailure(
                "TAR archive contains a corrupt header",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        size_field = header[124:136]
        if size_field[0] & 0x80:
            raise ImportFailure(
                "TAR archive uses an unsupported binary size extension",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        try:
            size = int(size_field.rstrip(b"\0 ") or b"0", 8)
            raw_name = header[:100].split(b"\0", 1)[0]
            raw_prefix = header[345:500].split(b"\0", 1)[0]
            name_bytes = raw_prefix + (b"/" if raw_prefix and raw_name else b"") + raw_name
            name = name_bytes.decode("utf-8")
        except (UnicodeDecodeError, ValueError) as exc:
            raise ImportFailure(
                "TAR archive contains invalid portable header fields",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            ) from exc
        return name, size, header[156:157]

    @classmethod
    def _validate_streamed_archive_layout(
        cls,
        name: str,
        is_directory: bool,
        kinds: dict[str, bool],
        descendant_parents: set[str],
    ) -> PurePosixPath:
        path = cls._archive_member_path(name)
        key = path.as_posix().casefold()
        if key in kinds or (not is_directory and key in descendant_parents):
            raise ImportFailure(
                f"archive contains a duplicate, case collision, or file/child conflict: {name}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=name,
            )
        for size in range(1, len(path.parts)):
            parent = PurePosixPath(*path.parts[:size]).as_posix().casefold()
            if kinds.get(parent) is False:
                raise ImportFailure(
                    f"archive member is nested below a regular file: {name}",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=name,
                )
            descendant_parents.add(parent)
        kinds[key] = is_directory
        return path

    def _copy_tar_member(
        self,
        stream: IO[bytes],
        writer: IO[bytes],
        size: int,
        remaining_budget: list[int],
        source: Path,
    ) -> None:
        remaining = size
        while remaining:
            chunk_size = min(1024 * 1024, remaining)
            chunk = self._read_tar_exact(stream, chunk_size, remaining_budget, source)
            writer.write(chunk)
            remaining -= len(chunk)

    @staticmethod
    def _read_tar_exact(
        stream: IO[bytes],
        size: int,
        remaining_budget: list[int],
        source: Path,
    ) -> bytes:
        if size > remaining_budget[0]:
            stream.read(max(0, remaining_budget[0]) + 1)
            raise ImportFailure(
                "TAR archive exceeds the bounded decompression and parsing budget",
                code=ImportCode.ARCHIVE_LIMIT,
                context=str(source),
            )
        data = stream.read(size)
        remaining_budget[0] -= len(data)
        if len(data) != size:
            raise ImportFailure(
                "TAR archive is truncated",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        return data

    @staticmethod
    def _drain_tar_trailing_data(
        stream: IO[bytes],
        remaining_budget: list[int],
        source: Path,
    ) -> None:
        while True:
            request_size = min(1024 * 1024, max(0, remaining_budget[0]) + 1)
            data = stream.read(request_size)
            if len(data) > remaining_budget[0]:
                raise ImportFailure(
                    "TAR archive exceeds the bounded decompression and parsing budget",
                    code=ImportCode.ARCHIVE_LIMIT,
                    context=str(source),
                )
            remaining_budget[0] -= len(data)
            if any(data):
                raise ImportFailure(
                    "TAR archive contains non-zero trailing data",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=str(source),
                )
            if len(data) < request_size:
                return

    def _preflight_zip(self, source: Path, snapshot: IO[bytes], archive_size: int) -> int:
        try:
            tail_size = min(archive_size, _ZIP_EOCD_SIZE + _ZIP_MAX_COMMENT_SIZE)
            snapshot.seek(archive_size - tail_size)
            tail = snapshot.read(tail_size)
        except ImportFailure:
            raise
        except OSError as exc:
            raise ImportFailure(
                f"cannot inspect ZIP archive {source}: {exc}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            ) from exc
        offset = tail.rfind(_ZIP_EOCD_SIGNATURE)
        if offset < 0 or len(tail) - offset < _ZIP_EOCD_SIZE:
            raise ImportFailure(
                f"ZIP archive has no bounded end-of-central-directory record: {source}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        (
            _signature,
            disk_number,
            central_directory_disk,
            disk_entries,
            total_entries,
            central_directory_size,
            central_directory_offset,
            comment_size,
        ) = struct.unpack_from("<4s4H2LH", tail, offset)
        if offset + _ZIP_EOCD_SIZE + comment_size != len(tail):
            raise ImportFailure(
                f"ZIP archive has an invalid end-of-central-directory record: {source}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        if disk_number != 0 or central_directory_disk != 0 or disk_entries != total_entries:
            raise ImportFailure(
                "multi-disk ZIP archives are unsupported",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        if total_entries == 0xFFFF or central_directory_size == 0xFFFFFFFF or central_directory_offset == 0xFFFFFFFF:
            raise ImportFailure(
                "Zip64 archives are unsupported by bounded intake",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        self._enforce_archive_metadata_limits(total_entries, ())
        maximum_directory_size = total_entries * _MAX_ZIP_CENTRAL_DIRECTORY_BYTES_PER_MEMBER
        if central_directory_size > maximum_directory_size:
            raise ImportFailure(
                "ZIP central directory exceeds the metadata size limit",
                code=ImportCode.ARCHIVE_LIMIT,
                context=str(source),
                details={"size": central_directory_size, "maximum": maximum_directory_size},
            )
        eocd_position = archive_size - tail_size + offset
        if central_directory_offset + central_directory_size > eocd_position:
            raise ImportFailure(
                "ZIP central directory escapes its bounded archive region",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(source),
            )
        return int(total_entries)

    @staticmethod
    def _archive_member_path(value: str) -> PurePosixPath:
        if not value or "\\" in value:
            raise ImportFailure(
                f"archive member is not a safe POSIX path: {value!r}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=value,
            )
        path = PurePosixPath(value)
        if path.is_absolute() or ":" in value or any(part in {"", ".", ".."} for part in path.parts):
            raise ImportFailure(
                f"archive member escapes the destination: {value!r}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=value,
            )
        return path

    @classmethod
    def _validated_archive_layout(
        cls,
        members: list[tuple[str, bool]],
    ) -> list[PurePosixPath]:
        paths = [cls._archive_member_path(name) for name, _ in members]
        kinds: dict[str, bool] = {}
        for (name, is_directory), path in zip(members, paths):
            key = path.as_posix().casefold()
            if key in kinds:
                raise ImportFailure(
                    f"archive contains a duplicate or case-colliding member: {name}",
                    code=ImportCode.UNSAFE_ARCHIVE,
                    context=name,
                )
            kinds[key] = is_directory
        for (name, _), path in zip(members, paths):
            for size in range(1, len(path.parts)):
                parent = PurePosixPath(*path.parts[:size]).as_posix().casefold()
                if kinds.get(parent) is False:
                    raise ImportFailure(
                        f"archive member is nested below a regular file: {name}",
                        code=ImportCode.UNSAFE_ARCHIVE,
                        context=name,
                    )
        return paths

    @staticmethod
    def _archive_target(destination: Path, relative: PurePosixPath) -> Path:
        target = destination.joinpath(*relative.parts).resolve()
        try:
            target.relative_to(destination)
        except ValueError as exc:
            raise ImportFailure(
                f"archive member escapes the destination: {relative}",
                code=ImportCode.UNSAFE_ARCHIVE,
                context=str(relative),
            ) from exc
        return target

    def _enforce_member_limits(self, members: list[tuple[str, int]]) -> None:
        if len(members) > self.max_files:
            raise ImportFailure(
                "archive contains too many files",
                code=ImportCode.ARCHIVE_LIMIT,
                details={"files": len(members), "maximum": self.max_files},
            )
        total = 0
        for name, size in members:
            total = self._enforce_regular_member_size(name, size, total)

    def _enforce_regular_member_size(self, name: str, size: int, total: int) -> int:
        if size < 0 or size > self.max_file_bytes:
            raise ImportFailure(
                f"archive member exceeds the file size limit: {name}",
                code=ImportCode.ARCHIVE_LIMIT,
                context=name,
                details={"size": size, "maximum": self.max_file_bytes},
            )
        total += size
        if total > self.max_total_bytes:
            raise ImportFailure(
                "archive exceeds the total extraction size limit",
                code=ImportCode.ARCHIVE_LIMIT,
                details={"size": total, "maximum": self.max_total_bytes},
            )
        return total

    def _enforce_archive_metadata_limits(self, member_count: int, member_names: Iterable[str]) -> None:
        if member_count > self.max_files:
            raise ImportFailure(
                "archive contains too many members",
                code=ImportCode.ARCHIVE_LIMIT,
                details={"members": member_count, "maximum": self.max_files},
            )
        for name in member_names:
            if len(name) > _MAX_ARCHIVE_MEMBER_PATH_LENGTH:
                raise ImportFailure(
                    f"archive member path exceeds the length limit: {name}",
                    code=ImportCode.ARCHIVE_LIMIT,
                    context=name,
                    details={"length": len(name), "maximum": _MAX_ARCHIVE_MEMBER_PATH_LENGTH},
                )
            path = self._archive_member_path(name)
            if len(path.parts) > _MAX_ARCHIVE_MEMBER_DEPTH:
                raise ImportFailure(
                    f"archive member path exceeds the depth limit: {name}",
                    code=ImportCode.ARCHIVE_LIMIT,
                    context=name,
                    details={"depth": len(path.parts), "maximum": _MAX_ARCHIVE_MEMBER_DEPTH},
                )

    def _enforce_limits(self, records: tuple[SourceFile, ...]) -> None:
        members = [(item.path, item.size) for item in records]
        self._enforce_member_limits(members)

    def _copy_bounded(self, reader: IO[bytes], writer: IO[bytes], declared_size: int) -> None:
        self._copy_bounded_digest(reader, writer, declared_size)

    def _copy_bounded_digest(self, reader: IO[bytes], writer: IO[bytes], declared_size: int) -> str:
        digest = hashlib.sha256()
        copied = 0
        while True:
            chunk = reader.read(min(1024 * 1024, self.max_file_bytes - copied + 1))
            if not chunk:
                break
            copied += len(chunk)
            if copied > self.max_file_bytes or copied > declared_size:
                raise ImportFailure(
                    "source file exceeds its declared or configured size",
                    code=ImportCode.ARCHIVE_LIMIT,
                )
            writer.write(chunk)
            digest.update(chunk)
        if copied != declared_size:
            raise ImportFailure(
                "source file size changed during intake",
                code=ImportCode.UNSAFE_SOURCE,
                details={"declared": declared_size, "copied": copied},
            )
        return digest.hexdigest()

    @staticmethod
    def _tree_source_digest(records: tuple[SourceFile, ...]) -> str:
        from .contracts import digest_document

        return cast(str, digest_document([item.to_dict() for item in records]))


__all__ = ["SourceIntake"]
