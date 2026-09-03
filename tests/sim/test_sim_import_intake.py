"""Contracts for fail-closed simulation source intake."""

from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
import subprocess
import tarfile
import zipfile
from pathlib import Path
from typing import IO

import pytest
from sim.catalog.importers import contracts as contracts_module
from sim.catalog.importers import intake as intake_module
from sim.catalog.importers.contracts import ImportFailure, safe_relative_path, validate_provenance
from sim.catalog.importers.intake import SourceIntake


def _code(error: BaseException) -> str:
    value = getattr(error, "code", "")
    return getattr(value, "value", value)


def test_directory_intake_is_byte_stable_and_content_identified(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "LICENSE.txt").write_text("fixture license\n", encoding="utf-8")
    (source / "model.xml").write_text("<mujoco/>\n", encoding="utf-8")

    first = SourceIntake().materialize(source, tmp_path / "first")
    second = SourceIntake().materialize(source, tmp_path / "second")

    assert first.to_dict() == second.to_dict()
    assert first.source_kind == "directory"
    assert [item.path for item in first.files] == ["LICENSE.txt", "model.xml"]
    assert (tmp_path / "first/model.xml").read_bytes() == (tmp_path / "second/model.xml").read_bytes()


def test_zip_intake_rejects_traversal_and_removes_partial_destination(tmp_path: Path) -> None:
    archive = tmp_path / "unsafe.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("safe/model.xml", "<mujoco/>\n")
        stream.writestr("../escaped.txt", "unsafe\n")
    destination = tmp_path / "intake"

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, destination)

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not destination.exists()
    assert not (tmp_path / "escaped.txt").exists()


def test_zip_intake_rejects_case_collisions(tmp_path: Path) -> None:
    archive = tmp_path / "collision.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("Meshes/Base.stl", "one")
        stream.writestr("meshes/base.STL", "two")

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not (tmp_path / "intake").exists()


@pytest.mark.parametrize("archive_kind", ["zip", "tar"])
def test_archive_member_limit_counts_directories_before_sorting_or_extraction(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    archive_kind: str,
) -> None:
    archive = tmp_path / f"directory-bomb.{archive_kind}"
    member_names = ["three/", "one/", "two/"]
    if archive_kind == "zip":
        with zipfile.ZipFile(archive, "w") as stream:
            for name in member_names:
                stream.writestr(name, b"")
    else:
        with tarfile.open(archive, "w") as stream:
            for name in member_names:
                member = tarfile.TarInfo(name)
                member.type = tarfile.DIRTYPE
                stream.addfile(member)

    def fail_sort(*_args: object, **_kwargs: object) -> list[object]:
        raise AssertionError("archive members must be limited before sorting")

    original_mkdir = Path.mkdir

    def guarded_mkdir(path: Path, *args: object, **kwargs: object) -> None:
        if path.name in {"one", "two", "three"}:
            raise AssertionError("archive member directories must not be created before the limit check")
        original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(intake_module, "sorted", fail_sort, raising=False)
    monkeypatch.setattr(Path, "mkdir", guarded_mkdir)
    if archive_kind == "zip":
        monkeypatch.setattr(
            intake_module.zipfile,
            "ZipFile",
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                AssertionError("ZIP metadata must not be loaded before EOCD limits")
            ),
        )
    else:
            monkeypatch.setattr(
                tarfile.TarFile,
            "getmembers",
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                AssertionError("TAR intake must stream members instead of loading the full table")
            ),
        )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_files=2).materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert exc_info.value.details == {"members": 3, "maximum": 2}
    assert not (tmp_path / "intake").exists()


def test_directory_member_limit_precedes_full_tree_sorting_and_hashing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source"
    source.mkdir()
    for name in ("three", "one", "two"):
        (source / name).mkdir()

    def fail(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("an over-limit directory must not be fully sorted or hashed")

    monkeypatch.setattr(intake_module, "file_records", fail)
    monkeypatch.setattr(intake_module, "sha256_file", fail)
    monkeypatch.setattr(intake_module, "sorted", fail, raising=False)

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_files=2).materialize(source, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert exc_info.value.details == {"members": 3, "maximum": 2}
    assert not (tmp_path / "intake").exists()


def test_directory_size_limit_precedes_hashing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "large.bin").write_bytes(b"12345")

    monkeypatch.setattr(
        intake_module,
        "sha256_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("an over-limit directory entry must not be hashed")
        ),
    )
    monkeypatch.setattr(
        contracts_module,
        "sha256_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("an over-limit directory entry must not be hashed")
        ),
    )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_file_bytes=4).materialize(source, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


def test_zip_physical_size_limit_precedes_metadata_loading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    archive = tmp_path / "oversized.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("model.xml", b"x")

    monkeypatch.setattr(
        intake_module.zipfile,
        "ZipFile",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("oversized ZIP metadata must not be loaded")
        ),
    )
    monkeypatch.setattr(
        intake_module,
        "sha256_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("an oversized archive must not be hashed")
        ),
    )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_total_bytes=archive.stat().st_size - 1).materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


def test_tar_physical_size_limit_precedes_hashing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    archive = tmp_path / "trailing-data.tar"
    with tarfile.open(archive, "w") as stream:
        member = tarfile.TarInfo("model.xml")
        member.size = 1
        stream.addfile(member, io.BytesIO(b"x"))
    with archive.open("ab") as stream:
        stream.write(b"\0" * 16_384)

    monkeypatch.setattr(
        intake_module,
        "sha256_file",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("an oversized archive must not be hashed")
        ),
    )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_total_bytes=archive.stat().st_size - 1).materialize(
            archive, tmp_path / "intake"
        )

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


@pytest.mark.parametrize("tar_format", [tarfile.PAX_FORMAT, tarfile.GNU_FORMAT])
def test_compressed_tar_rejects_unbounded_name_extensions(
    tmp_path: Path,
    tar_format: int,
) -> None:
    archive = tmp_path / "metadata-bomb.tar.gz"
    member = tarfile.TarInfo(("long-name-" * 10_000) + ".xml")
    member.size = 1
    with tarfile.open(archive, "w:gz", format=tar_format) as stream:
        stream.addfile(member, io.BytesIO(b"x"))

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not (tmp_path / "intake").exists()


def test_compressed_tar_trailing_data_is_bounded_by_decompression_budget(tmp_path: Path) -> None:
    raw = io.BytesIO()
    with tarfile.open(fileobj=raw, mode="w") as stream:
        member = tarfile.TarInfo("model.xml")
        member.size = 1
        stream.addfile(member, io.BytesIO(b"x"))
    archive = tmp_path / "trailing-bomb.tar.gz"
    archive.write_bytes(gzip.compress(raw.getvalue() + (b"\0" * 128_000)))

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_files=1, max_total_bytes=archive.stat().st_size).materialize(
            archive, tmp_path / "intake"
        )

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


def test_zip_central_directory_size_limit_precedes_metadata_loading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    archive = tmp_path / "metadata-bomb.zip"
    member = zipfile.ZipInfo("model.xml")
    member.extra = b"\xfe\xca\xfc\x0f" + (b"x" * 4092)
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr(member, b"x")

    monkeypatch.setattr(
        intake_module.zipfile,
        "ZipFile",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("oversized central-directory metadata must not be loaded")
        ),
    )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_files=1).materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


@pytest.mark.parametrize(
    ("field_offset", "value"),
    [
        (4, 1),
        (10, 0xFFFF),
    ],
    ids=["multi-disk", "zip64"],
)
def test_zip_preflight_rejects_unbounded_formats_before_metadata_loading(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    field_offset: int,
    value: int,
) -> None:
    archive = tmp_path / "unsupported.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("model.xml", b"x")
    payload = bytearray(archive.read_bytes())
    eocd_offset = payload.rfind(b"PK\x05\x06")
    assert eocd_offset >= 0
    payload[eocd_offset + field_offset : eocd_offset + field_offset + 2] = value.to_bytes(2, "little")
    archive.write_bytes(payload)

    monkeypatch.setattr(
        intake_module.zipfile,
        "ZipFile",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("unsupported ZIP metadata must not be loaded")
        ),
    )

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not (tmp_path / "intake").exists()


def test_archive_snapshot_ignores_oversized_path_swap_after_identity_binding(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    archive = tmp_path / "source.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("model.xml", b"snapshot-a")
    original_bytes = archive.read_bytes()
    replacement = tmp_path / "replacement.zip"
    with zipfile.ZipFile(replacement, "w") as stream:
        stream.writestr("model.xml", b"replacement-b")
    with replacement.open("ab") as stream:
        stream.write(b"\0" * (len(original_bytes) + 1))
    original_extract = SourceIntake._extract_archive

    def swap_then_extract(
        self: SourceIntake,
        source: Path,
        destination: Path,
        snapshot: IO[bytes],
        archive_size: int,
    ) -> None:
        replacement.replace(source)
        original_extract(self, source, destination, snapshot, archive_size)

    monkeypatch.setattr(SourceIntake, "_extract_archive", swap_then_extract)

    result = SourceIntake(max_total_bytes=len(original_bytes)).materialize(
        archive, tmp_path / "intake"
    )

    assert result.source_sha256 == hashlib.sha256(original_bytes).hexdigest()
    assert (tmp_path / "intake/model.xml").read_bytes() == b"snapshot-a"


def test_archive_snapshot_prevents_aba_digest_artifact_split(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    archive = tmp_path / "source.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("model.xml", b"snapshot-a")
    original_bytes = archive.read_bytes()
    alternate = tmp_path / "alternate.zip"
    with zipfile.ZipFile(alternate, "w") as stream:
        stream.writestr("model.xml", b"alternate-b")
    alternate_bytes = alternate.read_bytes()
    original_extract = SourceIntake._extract_archive

    def aba_then_extract(
        self: SourceIntake,
        source: Path,
        destination: Path,
        snapshot: IO[bytes],
        archive_size: int,
    ) -> None:
        source.write_bytes(alternate_bytes)
        try:
            original_extract(self, source, destination, snapshot, archive_size)
        finally:
            source.write_bytes(original_bytes)

    monkeypatch.setattr(SourceIntake, "_extract_archive", aba_then_extract)

    result = SourceIntake().materialize(archive, tmp_path / "intake")

    assert result.source_sha256 == hashlib.sha256(original_bytes).hexdigest()
    assert (tmp_path / "intake/model.xml").read_bytes() == b"snapshot-a"


@pytest.mark.parametrize("archive_kind", ["zip", "tar"])
@pytest.mark.parametrize(
    "member_name",
    [
        f"{'a' * 1021}.xml",
        "/".join(["n"] * 65) + "/model.xml",
    ],
)
def test_archive_intake_rejects_excessive_member_path_length_or_depth(
    tmp_path: Path,
    member_name: str,
    archive_kind: str,
) -> None:
    archive = tmp_path / f"path-bomb.{archive_kind}"
    if archive_kind == "zip":
        with zipfile.ZipFile(archive, "w") as stream:
            stream.writestr(member_name, b"<mujoco/>\n")
    else:
        tar_format = tarfile.PAX_FORMAT if len(member_name) > 255 else tarfile.USTAR_FORMAT
        with tarfile.open(archive, "w", format=tar_format) as stream:
            member = tarfile.TarInfo(member_name)
            member.size = len(b"<mujoco/>\n")
            stream.addfile(member)

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, tmp_path / "intake")

    expected_code = (
        "SIMIMPORT_UNSAFE_ARCHIVE"
        if archive_kind == "tar" and len(member_name) > 255
        else "SIMIMPORT_ARCHIVE_LIMIT"
    )
    assert _code(exc_info.value) == expected_code
    assert not (tmp_path / "intake").exists()


@pytest.mark.parametrize("value", ["nested/file:ads", "nested:dir/file.txt", "file.txt:"])
def test_relative_path_contract_rejects_colon_in_any_component(value: str) -> None:
    with pytest.raises(ImportFailure):
        safe_relative_path(value, "fixture.path")

    with pytest.raises(ImportFailure):
        SourceIntake._archive_member_path(value)


def test_zip_intake_rejects_nested_alternate_data_stream_name(tmp_path: Path) -> None:
    archive = tmp_path / "ads.zip"
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("nested/file:ads", "must not be extracted")

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(archive, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_ARCHIVE"
    assert not (tmp_path / "intake").exists()


@pytest.mark.skipif(os.name != "nt", reason="NTFS alternate data streams are Windows-only")
def test_windows_ntfs_ads_name_is_not_accepted_as_an_intake_path(tmp_path: Path) -> None:
    base = tmp_path / "source.bin"
    base.write_bytes(b"default stream")
    ads = Path(f"{base}:sim_ads")
    ads.write_bytes(b"alternate stream")

    # The name is rejected by the same archive/path contract before any
    # extraction target can be opened as an NTFS stream.
    with pytest.raises(ImportFailure):
        SourceIntake._archive_member_path(f"nested/{base.name}:sim_ads")


def test_directory_intake_rejects_destination_inside_source(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "model.xml").write_text("<mujoco/>\n", encoding="utf-8")

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(source, source / "nested-intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_SOURCE"
    assert not (source / "nested-intake").exists()


def test_directory_intake_rejects_top_level_symlink_before_hash_or_copy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = tmp_path / "target"
    target.mkdir()
    (target / "model.xml").write_text("<mujoco/>\n", encoding="utf-8")
    source = tmp_path / "source-link"
    try:
        source.symlink_to(target, target_is_directory=True)
    except (OSError, NotImplementedError):
        pytest.skip("symbolic links are unavailable in this Windows test environment")

    def fail_file_records(_path: Path) -> tuple[()]:
        raise AssertionError("source files must not be hashed through a symbolic link")

    monkeypatch.setattr(intake_module, "file_records", fail_file_records)

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(source, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_SOURCE"
    assert not (tmp_path / "intake").exists()


@pytest.mark.skipif(os.name != "nt", reason="Windows junction regression test")
def test_directory_intake_rejects_top_level_windows_junction_before_hash_or_copy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = tmp_path / "target"
    target.mkdir()
    (target / "model.xml").write_text("<mujoco/>\n", encoding="utf-8")
    source = tmp_path / "source-junction"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(source), str(target)],
        check=False,
        capture_output=True,
        text=True,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    def fail_file_records(_path: Path) -> tuple[()]:
        raise AssertionError("source files must not be hashed through a junction")

    monkeypatch.setattr(intake_module, "file_records", fail_file_records)
    try:
        with pytest.raises(ImportFailure) as exc_info:
            SourceIntake().materialize(source, tmp_path / "intake")
        assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_SOURCE"
        assert not (tmp_path / "intake").exists()
    finally:
        if source.exists():
            source.rmdir()


def test_directory_intake_rejects_simulated_windows_reparse_component_before_hash(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source"
    linked = source / "linked"
    linked.mkdir(parents=True)
    (linked / "foreign.xml").write_text("<mujoco/>\n", encoding="utf-8")
    original_lstat = contracts_module.os.lstat

    class ReparseMetadata:
        def __init__(self, metadata: os.stat_result) -> None:
            self._metadata = metadata
            self.st_mode = metadata.st_mode
            self.st_file_attributes = getattr(metadata, "st_file_attributes", 0) | 0x0400

        def __getattr__(self, name: str):
            return getattr(self._metadata, name)

    def fake_lstat(path: str | bytes | os.PathLike[str] | os.PathLike[bytes]):
        metadata = original_lstat(path)
        return ReparseMetadata(metadata) if Path(path) == linked else metadata

    def fail_hash(_path: Path) -> str:
        raise AssertionError("source files must not be hashed through a reparse component")

    monkeypatch.setattr(contracts_module.os, "lstat", fake_lstat)
    monkeypatch.setattr(contracts_module, "sha256_file", fail_hash)

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake().materialize(source, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_SOURCE"
    assert not (tmp_path / "intake").exists()


@pytest.mark.skipif(os.name != "nt", reason="Windows junction regression test")
def test_directory_intake_rejects_nested_windows_junction_before_hash_or_copy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source"
    source.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "foreign.xml").write_text("<mujoco/>\n", encoding="utf-8")
    junction = source / "aaa-junction"
    created = subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(junction), str(outside)],
        check=False,
        capture_output=True,
        text=True,
    )
    if created.returncode != 0:
        pytest.skip(f"cannot create Windows junction: {created.stderr.strip()}")

    def fail_hash(_path: Path) -> str:
        raise AssertionError("source files must not be hashed through a junction")

    original_scandir = contracts_module.os.scandir

    def guarded_scandir(path: str | bytes | int | os.PathLike[str] | os.PathLike[bytes]):
        if not isinstance(path, int) and Path(path) == junction:
            raise AssertionError("source intake must not enumerate through a junction")
        return original_scandir(path)

    monkeypatch.setattr(contracts_module, "sha256_file", fail_hash)
    monkeypatch.setattr(contracts_module.os, "scandir", guarded_scandir)
    try:
        with pytest.raises(ImportFailure) as exc_info:
            SourceIntake().materialize(source, tmp_path / "intake")
        assert _code(exc_info.value) == "SIMIMPORT_UNSAFE_SOURCE"
        assert not (tmp_path / "intake").exists()
    finally:
        if junction.exists():
            junction.rmdir()


def test_intake_enforces_file_and_total_size_limits(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "large.bin").write_bytes(b"12345")

    with pytest.raises(ImportFailure) as exc_info:
        SourceIntake(max_file_bytes=4).materialize(source, tmp_path / "intake")

    assert _code(exc_info.value) == "SIMIMPORT_ARCHIVE_LIMIT"
    assert not (tmp_path / "intake").exists()


def test_provenance_requires_real_license_file(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    request = {
        "owner": "Fixture owner",
        "license": "LicenseRef-Fixture",
        "license_file": "LICENSE.txt",
        "source_uri": "https://example.invalid/fixture",
    }

    with pytest.raises(ImportFailure) as missing:
        validate_provenance(request, source_root=source)
    assert _code(missing.value) == "SIMIMPORT_LICENSE_REQUIRED"

    (source / "LICENSE.txt").write_text("fixture license\n", encoding="utf-8")
    normalized = validate_provenance(request, source_root=source)

    assert normalized["license"] == "LicenseRef-Fixture"
    assert normalized["license_file"] == "LICENSE.txt"
    assert normalized["third_party_assets"] == []
    assert json.loads(json.dumps(normalized, sort_keys=True)) == normalized
