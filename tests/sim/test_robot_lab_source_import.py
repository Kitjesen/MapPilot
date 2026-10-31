from __future__ import annotations

import json
from pathlib import Path

import pytest
from sim.tools.assets.import_robot_lab_sources import (
    ROBOTS_PATH,
    SourceImportError,
    build_inventory,
    materialize_source_library,
)


def test_replace_never_removes_an_unrelated_destination(tmp_path: Path) -> None:
    upstream = _upstream(tmp_path)
    destination = tmp_path / "user-assets"
    destination.mkdir()
    sentinel = destination / "keep.txt"
    sentinel.write_text("user data", encoding="utf-8")
    with pytest.raises(SourceImportError, match="existing RobotLab source intake"):
        materialize_source_library(upstream, destination, tmp_path / "inventory.json", revision="abc123", replace=True)
    assert sentinel.read_text(encoding="utf-8") == "user data"


def _upstream(tmp_path: Path) -> Path:
    root = tmp_path / "upstream"
    robots = root / ROBOTS_PATH
    first = robots / "vendor/alpha_description"
    second = robots / "vendor/beta_description"
    (first / "urdf").mkdir(parents=True)
    (first / "meshes").mkdir()
    (second / "urdf").mkdir(parents=True)
    (second / "meshes").mkdir()
    (root / "source/robot_lab/robot_lab/assets").mkdir(parents=True)
    (root / ".github").mkdir()
    (root / "LICENSE").write_text("Apache License 2.0\n", encoding="utf-8")
    (root / "VERSION").write_text("2.3.2\n", encoding="utf-8")
    (root / ".github/LICENSE_HEADER.txt").write_text(
        "SPDX-License-Identifier: Apache-2.0\n", encoding="utf-8"
    )
    (first / "meshes/body.STL").write_bytes(b"mesh")
    (second / "meshes/body.obj").write_bytes(b"mesh")
    (first / "urdf/alpha.urdf").write_text(
        """<robot name="alpha"><link name="base"><visual><geometry>
        <mesh filename="../meshes/body.STL"/>
        </geometry></visual><collision><geometry>
        <mesh filename="meshes/body.STL"/>
        </geometry></collision></link></robot>""",
        encoding="utf-8",
    )
    (first / "urdf/variant.urdf").write_text(
        """<robot name="alpha_variant"><link name="base"><visual><geometry>
        <mesh filename="../meshes/missing.STL"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    (second / "urdf/beta.urdf").write_text(
        """<robot name="beta"><link name="base"><visual><geometry>
        <mesh filename="package://beta_description/meshes/body.obj"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    (root / "source/robot_lab/robot_lab/assets/fixture.py").write_text(
        'PRIMARY = f"{DATA}/Robots/vendor/alpha_description/urdf/alpha.urdf"\n',
        encoding="utf-8",
    )
    return root


def test_inventory_preserves_variants_and_reports_reference_resolution(tmp_path: Path) -> None:
    inventory = build_inventory(_upstream(tmp_path), revision="abc123")

    assert inventory["source"]["revision"] == "abc123"
    assert inventory["summary"] == {
        "readme_family_count": 24,
        "local_model_count": 2,
        "external_family_count": 2,
        "robot_file_count": 5,
        "robot_size_bytes": sum(item["size_bytes"] for item in inventory["source_files"]),
        "urdf_count": 3,
        "configured_urdf_count": 1,
        "reference_count": 4,
        "resolved_reference_count": 3,
        "missing_reference_count": 1,
        "case_mismatch_reference_count": 0,
        "unresolved_reference_count": 0,
    }
    alpha = inventory["models"][0]
    assert [entry["path"] for entry in alpha["urdf_entries"]] == [
        "source/robot_lab/data/Robots/vendor/alpha_description/urdf/alpha.urdf",
        "source/robot_lab/data/Robots/vendor/alpha_description/urdf/variant.urdf",
    ]
    assert alpha["urdf_entries"][0]["configured_by_robot_lab"] is True
    assert alpha["urdf_entries"][0]["references"][1] == {
        "uri": "meshes/body.STL",
        "status": "resolved",
        "path": "vendor/alpha_description/meshes/body.STL",
        "resolution": "model_root_relative",
    }
    assert alpha["urdf_entries"][1]["references"] == [
        {
            "uri": "../meshes/missing.STL",
            "status": "missing",
            "path": "vendor/alpha_description/meshes/missing.STL",
        }
    ]
    assert inventory["presentation_entries"] == [
        {
            "id": "vendor_alpha",
            "version": "robot_lab_2.3.2",
            "label": "Vendor Alpha",
            "source_path": "source/robot_lab/data/Robots/vendor/alpha_description",
            "urdf": "source/robot_lab/data/Robots/vendor/alpha_description/urdf/alpha.urdf",
            "status": "source_only",
        },
        {
            "id": "vendor_beta",
            "version": "robot_lab_2.3.2",
            "label": "Vendor Beta",
            "source_path": "source/robot_lab/data/Robots/vendor/beta_description",
            "urdf": "source/robot_lab/data/Robots/vendor/beta_description/urdf/beta.urdf",
            "status": "source_only",
        },
    ]


def test_materialization_is_repeatable_and_does_not_create_active_packages(tmp_path: Path) -> None:
    upstream = _upstream(tmp_path)
    destination = tmp_path / "sim/.imports/robot_lab"
    inventory_path = tmp_path / "build/import/robot_lab-source-inventory.json"

    first = materialize_source_library(
        upstream, destination, inventory_path, revision="abc123"
    )
    second = materialize_source_library(
        upstream, destination, inventory_path, revision="abc123"
    )

    assert first == second
    assert (destination / "source/robot_lab/data/Robots/vendor/alpha_description/urdf/alpha.urdf").is_file()
    assert (destination / "LICENSE").is_file()
    assert (destination / ".github/LICENSE_HEADER.txt").is_file()
    assert json.loads(inventory_path.read_text(encoding="utf-8")) == first
    assert json.loads(
        (destination / "robot-library.presentation.json").read_text(encoding="utf-8")
    ) == {
        "robots": [
            {"id": "vendor_alpha", "version": "robot_lab_2.3.2", "label": "Vendor Alpha"},
            {"id": "vendor_beta", "version": "robot_lab_2.3.2", "label": "Vendor Beta"},
        ]
    }
    assert not (tmp_path / "sim/packages/robots").exists()


def test_existing_changed_destination_requires_explicit_replace(tmp_path: Path) -> None:
    upstream = _upstream(tmp_path)
    destination = tmp_path / "sim/.imports/robot_lab"
    inventory_path = tmp_path / "build/import/inventory.json"
    materialize_source_library(upstream, destination, inventory_path, revision="abc123")
    copied = destination / "source/robot_lab/data/Robots/vendor/alpha_description/meshes/body.STL"
    copied.write_bytes(b"changed")

    with pytest.raises(SourceImportError, match="--replace"):
        materialize_source_library(upstream, destination, inventory_path, revision="abc123")

    materialize_source_library(
        upstream, destination, inventory_path, revision="abc123", replace=True
    )
    assert copied.read_bytes() == b"mesh"
