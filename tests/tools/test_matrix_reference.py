from __future__ import annotations

import json
import tarfile
import zipfile
from pathlib import Path

import pytest
from sim.catalog.importers.contracts import ImportFailure
from sim.tools.matrix_reference import stage_matrix_reference


def _release(source: Path) -> Path:
    binary = source / "UeSim/Binaries/Linux/UeSim"
    binary.parent.mkdir(parents=True)
    header = bytearray(64)
    header[:6] = b"\x7fELF\x02\x01"
    header[18:20] = (62).to_bytes(2, "little")
    binary.write_bytes(header)
    mesh = source / "UeSim/Content/model/xgb/assets/base.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_bytes(b"test mesh")
    (source / "README.md").write_text("# MATRiX v1.1.2\n", encoding="utf-8")
    return source


def test_stage_keeps_binary_and_mesh_as_an_external_reference(tmp_path: Path) -> None:
    source = _release(tmp_path / "source")
    destination = tmp_path / "third_party/matrix/runtime-v1.1.2"

    report = stage_matrix_reference(source, destination, version="1.1.2")

    assert (destination / "UeSim/Binaries/Linux/UeSim").read_bytes() == (
        source / "UeSim/Binaries/Linux/UeSim"
    ).read_bytes()
    assert (destination / "UeSim/Content/model/xgb/assets/base.stl").read_bytes() == b"test mesh"
    assert report["runtime"]["platform"] == "linux-x86_64"
    assert report["runtime"]["windows_native"] is False
    assert report["asset_counts"][".stl"] == 1
    assert report["redistribution"] == "unverified"
    assert report["catalog_registered"] is False
    assert report["launch_performed"] is False
    assert json.loads((destination / "matrix-reference.json").read_text(encoding="utf-8")) == report


def test_license_files_are_visible_without_granting_all_assets(tmp_path: Path) -> None:
    source = _release(tmp_path / "source")
    (source / "LICENSE").write_text("BSD 3-Clause License\n", encoding="utf-8")

    report = stage_matrix_reference(source, tmp_path / "reference", version="1.1.2")

    assert report["license_files"] == ["LICENSE"]
    assert report["redistribution"] == "unverified"


def test_empty_release_readme_is_rejected_without_a_success_receipt(tmp_path: Path) -> None:
    source = _release(tmp_path / "source")
    (source / "README.md").write_text("", encoding="utf-8")
    destination = tmp_path / "reference"

    with pytest.raises(ValueError, match="MATRiX README"):
        stage_matrix_reference(source, destination, version="1.1.2")

    assert not (destination / "matrix-reference.json").exists()
    assert not destination.exists()
    (source / "README.md").write_text("# MATRiX v1.1.2\n", encoding="utf-8")
    assert stage_matrix_reference(source, destination, version="1.1.2")["version"] == "1.1.2"


@pytest.mark.parametrize("suffix", [".zip", ".tar.gz"])
def test_wrapped_archive_preserves_upstream_layout(tmp_path: Path, suffix: str) -> None:
    source = _release(tmp_path / "source")
    policy = source / "UeSim/Plugins/MatrixMotionControl/Source/ThirdParty/MatrixMC/models/xg2/odom_balancestand_withyaw_0423"
    policy.parent.mkdir(parents=True)
    policy.write_bytes(b"reference policy")
    archive_path = tmp_path / f"release{suffix}"
    if suffix == ".zip":
        with zipfile.ZipFile(archive_path, "w") as archive:
            for path in source.rglob("*"):
                if path.is_file():
                    archive.write(path, f"MATRiX_v1.1.2/{path.relative_to(source).as_posix()}")
    else:
        with tarfile.open(archive_path, "w:gz", format=tarfile.GNU_FORMAT) as archive:
            archive.add(source, arcname="MATRiX_v1.1.2")

    report = stage_matrix_reference(archive_path, tmp_path / "reference", version="1.1.2")

    assert report["source"]["kind"] == "archive"
    assert report["source"]["file_count"] == 4
    assert report["runtime"]["entry"] == "MATRiX_v1.1.2/UeSim/Binaries/Linux/UeSim"
    assert report["release_readme"] == "MATRiX_v1.1.2/README.md"


def test_existing_reference_is_never_overwritten_or_removed(tmp_path: Path) -> None:
    source = _release(tmp_path / "source")
    destination = tmp_path / "reference"
    destination.mkdir()
    note = destination / "user-note.txt"
    note.write_text("keep this", encoding="utf-8")

    with pytest.raises(ImportFailure, match="already exists"):
        stage_matrix_reference(source, destination, version="1.1.2")

    assert note.read_text(encoding="utf-8") == "keep this"


def test_non_linux_binary_cannot_be_reported_as_a_linux_reference(tmp_path: Path) -> None:
    source = _release(tmp_path / "source")
    (source / "UeSim/Binaries/Linux/UeSim").write_bytes(b"MZ" + bytes(62))
    destination = tmp_path / "reference"

    with pytest.raises(ValueError, match="Linux x86-64 ELF"):
        stage_matrix_reference(source, destination, version="1.1.2")

    assert not destination.exists()
